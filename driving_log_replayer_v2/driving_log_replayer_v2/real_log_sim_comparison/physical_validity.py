"""Physical-validity verification pipeline for finalized reidentify caches.

Each verification step returns both machine-readable measurements and the HTML
section that presents them.  Only :func:`assemble_html` creates the document
shell and :func:`run` writes it to disk.
"""
from __future__ import annotations

import argparse
import html
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import plotly.graph_objects as go
import yaml
from plotly.offline import get_plotlyjs

from .lib._figures._physical_validity import build_fig_cross_long, build_fig_cross_steer
from .lib._physical_validity import (
    _pick_longest_contiguous_timeseries_row,
    _simulate_first_order,
    fit_k_us,
    fit_longitudinal,
    fit_steering,
    fit_xy_heading_rate_coeff,
    xy_residual,
    yaw_residual,
)
from .reidentify.load_data import build_resampled, discover_cached_datasets, read_dataset_csv


@dataclass
class PreparedDataset:
    """One cache that is valid on the common 10 ms verification grid."""

    dataset_id: str
    data: dict[str, Any]
    timeseries: dict[str, Any]


@dataclass
class ValidationContext:
    """Shared inputs prepared once for all physical-validity steps."""

    params: dict[str, Any]
    model_type: str
    wheelbase: float
    datasets: list[PreparedDataset] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)


@dataclass
class ValidationStep:
    """Machine-readable result and its self-contained report section."""

    name: str
    result: dict[str, Any]
    html: str


def _number(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "—"
    return f"{number:.6g}" if np.isfinite(number) else "—"


def _table(headers: list[str], rows: list[list[Any]], empty_message: str) -> str:
    head = "".join(f"<th>{html.escape(header)}</th>" for header in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{html.escape(str(value))}</td>" for value in row) + "</tr>"
        for row in rows
    ) or f'<tr><td colspan="{len(headers)}">{html.escape(empty_message)}</td></tr>'
    return f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>"


def _figure_html(fig: go.Figure, title: str) -> str:
    return f"<h3>{html.escape(title)}</h3>{fig.to_html(full_html=False, include_plotlyjs=False)}"


def _histogram(values: list[float], title: str) -> go.Figure:
    fig = go.Figure()
    if values:
        fig.add_trace(go.Histogram(x=values, name=title))
    fig.update_layout(title=title, xaxis_title="RMSE", yaxis_title="データセット数")
    return fig


def _model_type(scenario: Path | None, case: str, params: dict[str, Any]) -> str:
    if scenario is None:
        return str(params.get("vehicle_model_type", "current"))
    document = yaml.safe_load(scenario.read_text(encoding="utf-8")) or {}
    model = document.get("Evaluation", {}).get("Conditions", {}).get("models", {}).get(case, {})
    return str(model.get("vehicle_model_type") or params.get("vehicle_model_type", case))


def prepare_datasets(
    collection_dir: Path,
    params_path: Path,
    *,
    scenario: Path | None = None,
    case: str = "current",
) -> tuple[ValidationContext, ValidationStep]:
    """Read, resample, and enrich every cache exactly once."""
    document = yaml.safe_load(params_path.read_text(encoding="utf-8")) or {}
    params = dict(document.get("params", document))
    model_type = _model_type(scenario, case, params)
    wheelbase = float(params.get("wheelbase", 4.76012))
    context = ValidationContext(params=params, model_type=model_type, wheelbase=wheelbase)

    for dataset_id, csv_path in discover_cached_datasets(collection_dir):
        try:
            source = read_dataset_csv(csv_path)
            dataset = build_resampled(source, 0.01, context=f"physical_validity:{dataset_id}")
            if dataset is None:
                raise ValueError("共通時間範囲が短すぎるか、必須信号が空です")
            kin = source["kinematic"]
            if kin.empty:
                raise ValueError("kinematic が空です")
            # Keep x/y on exactly the same common interval selected by
            # ``build_resampled`` rather than starting at kinematic's origin.
            t0 = max(float(source[topic]["t_ns"].iloc[0]) for topic in ("cmd", "accel", "steering", "velocity", "kinematic"))
            t_grid = t0 + np.arange(len(dataset["vx"]), dtype=float) * 0.01e9
            source_t = kin["t_ns"].to_numpy(dtype=float)
            dataset["xy"] = tuple(
                np.interp(t_grid, source_t, kin[column].to_numpy(dtype=float))
                for column in ("x", "y", "yaw", "vx", "wz")
            )
            context.datasets.append(PreparedDataset(
                dataset_id=dataset_id,
                data=dataset,
                timeseries={
                    "dataset_id": dataset_id,
                    "label": dataset_id,
                    "_t0_ns": t0,
                    "t": (np.arange(len(dataset["vx"]), dtype=float) * 0.01).tolist(),
                    "a_cmd_raw": dataset["a_cmd"].tolist(),
                    "a_act_raw": dataset["a_act"].tolist(),
                    "d_cmd": dataset["d_cmd"].tolist(),
                    "d_act_raw": dataset["d_act"].tolist(),
                    "vx": dataset["vx"].tolist(),
                    "mask_dyn": dataset["gear_drive"].tolist(),
                },
            ))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            context.skipped.append(f"{dataset_id}: {exc}")

    rows = [[item.dataset_id, len(item.data["vx"])] for item in context.datasets]
    skipped = "".join(f"<li>{html.escape(reason)}</li>" for reason in context.skipped)
    section = f"""<section id="prepare"><h2>準備</h2>
<p>モデル: <code>{html.escape(model_type)}</code> / wheelbase: {_number(wheelbase)} m / 有効データセット: {len(rows)}</p>
{_table(["dataset", "samples (10 ms)"], rows, "有効データなし")}
<h3>除外理由</h3>{f'<ul>{skipped}</ul>' if skipped else '<p>なし</p>'}</section>"""
    return context, ValidationStep("prepare", {"n_valid": len(rows), "skipped": context.skipped}, section)


def validate_longitudinal(context: ValidationContext) -> ValidationStep:
    rows: list[list[str]] = []
    values: list[float] = []
    series: list[dict[str, Any]] = []
    result: dict[str, Any] = {"datasets": {}}
    for item in context.datasets:
        fit = fit_longitudinal(item.data)
        result["datasets"][item.dataset_id] = fit
        rows.append([item.dataset_id, _number(fit.get("tau") if fit else None), _number(fit.get("delay") if fit else None), _number(fit.get("rmse") if fit else None), str(fit.get("n", 0) if fit else 0)])
        if fit:
            values.append(float(fit["rmse"]))
            row = dict(item.timeseries)
            row["a_sim_cross_raw"] = _simulate_first_order(item.data["a_cmd"], fit["tau"], fit["delay"], 0.01).tolist()
            series.append(row)
    section = "<section id=\"longitudinal\"><h2>縦方向</h2>" + _table(["dataset", "τ [s]", "delay [s]", "RMSE", "n"], rows, "フィット可能なデータなし") + _figure_html(_histogram(values, "縦方向フィット RMSE"), "RMSE 分布") + _figure_html(build_fig_cross_long(_pick_longest_contiguous_timeseries_row(series)), "代表時系列") + "</section>"
    result["rmse"] = values
    return ValidationStep("longitudinal", result, section)


def validate_steering(context: ValidationContext) -> ValidationStep:
    rows: list[list[str]] = []
    values: list[float] = []
    series: list[dict[str, Any]] = []
    result: dict[str, Any] = {"datasets": {}}
    for item in context.datasets:
        fit = fit_steering(item.data)
        result["datasets"][item.dataset_id] = fit
        rows.append([item.dataset_id, _number(fit.get("tau") if fit else None), _number(fit.get("delay") if fit else None), _number(fit.get("bias") if fit else None), _number(fit.get("rmse") if fit else None), str(fit.get("n", 0) if fit else 0)])
        if fit:
            values.append(float(fit["rmse"]))
            row = dict(item.timeseries)
            row["d_sim_fit_raw"] = (_simulate_first_order(item.data["d_cmd"], fit["tau"], fit["delay"], 0.01) + fit["bias"]).tolist()
            series.append(row)
    section = "<section id=\"steering\"><h2>操舵</h2>" + _table(["dataset", "τ [s]", "delay [s]", "bias [rad]", "RMSE", "n"], rows, "フィット可能なデータなし") + _figure_html(_histogram(values, "操舵フィット RMSE"), "RMSE 分布") + _figure_html(build_fig_cross_steer(_pick_longest_contiguous_timeseries_row(series)), "代表時系列") + "</section>"
    result["rmse"] = values
    return ValidationStep("steering", result, section)


def validate_yaw(context: ValidationContext) -> ValidationStep:
    datasets = [item.data for item in context.datasets]
    fit = fit_k_us(datasets, wheelbase=context.wheelbase) if datasets else {"k_us": float("nan"), "rmse": float("nan"), "n": 0}
    rows: list[list[str]] = []
    per_dataset: dict[str, dict[str, float | int]] = {}
    for item in context.datasets:
        residual = yaw_residual(item.data, k_us=float(fit["k_us"]), wheelbase=context.wheelbase)
        metrics = {"rmse": float(np.sqrt(np.mean(residual ** 2))) if len(residual) else float("nan"), "n": int(len(residual))}
        per_dataset[item.dataset_id] = metrics
        rows.append([item.dataset_id, _number(metrics["rmse"]), str(metrics["n"])])
    section = f"<section id=\"yaw\"><h2>yaw</h2><p>横断フィット: k_us={_number(fit['k_us'])}, RMSE={_number(fit['rmse'])}, n={fit['n']}</p>{_table(['dataset', 'RMSE', 'n'], rows, '評価可能なデータなし')}</section>"
    return ValidationStep("yaw", {"fit": fit, "datasets": per_dataset}, section)


def validate_xy(context: ValidationContext) -> ValidationStep:
    datasets = [item.data for item in context.datasets]
    fit = fit_xy_heading_rate_coeff(datasets, initial=float(context.params.get("xy_heading_rate_coeff", 0.0))) if datasets else {"xy_heading_rate_coeff": float("nan"), "rmse": float("nan"), "n": 0}
    rows: list[list[str]] = []
    per_dataset: dict[str, dict[str, float | int]] = {}
    residuals: list[float] = []
    for item in context.datasets:
        rx, ry = xy_residual(item.data, float(fit["xy_heading_rate_coeff"]))
        joined = np.concatenate((rx, ry))
        metrics = {"rmse": float(np.sqrt(np.mean(joined ** 2))) if len(joined) else float("nan"), "n": int(len(joined))}
        per_dataset[item.dataset_id] = metrics
        rows.append([item.dataset_id, _number(metrics["rmse"]), str(metrics["n"])])
        residuals.extend(joined.tolist())
    section = f"<section id=\"xy\"><h2>x/y</h2><p>横断 heading-rate 係数: {_number(fit['xy_heading_rate_coeff'])}, RMSE={_number(fit['rmse'])}, n={fit['n']}</p>{_table(['dataset', 'RMSE', 'n'], rows, '評価可能なデータなし')}{_figure_html(_histogram(residuals, 'x/y 残差'), '残差分布')}</section>"
    return ValidationStep("xy", {"fit": fit, "datasets": per_dataset}, section)


def assemble_html(context: ValidationContext, steps: list[ValidationStep]) -> str:
    """Aggregate already-rendered sections without performing verification work."""
    sections = "\n".join(step.html for step in steps)
    return f"""<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>物理的妥当性検証</title>
<style>body{{font-family:sans-serif;max-width:1200px;margin:2rem auto;line-height:1.5}}table{{border-collapse:collapse;margin:.5rem 0 1.5rem}}td,th{{border:1px solid #bbb;padding:.35rem .6rem;text-align:left}}section{{margin:2rem 0}}.plotly-graph-div{{max-width:100%}}</style>
<script>{get_plotlyjs()}</script></head><body><h1>車両モデル物理的妥当性検証</h1>{sections}</body></html>"""


def run(collection_dir: Path, params_path: Path, output: Path, *, scenario: Path | None = None, case: str = "current") -> Path:
    """Run all verification steps and write one self-contained HTML report."""
    context, prepared = prepare_datasets(collection_dir, params_path, scenario=scenario, case=case)
    steps = [prepared, validate_longitudinal(context), validate_steering(context), validate_yaw(context), validate_xy(context)]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(assemble_html(context, steps), encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="物理的妥当性検証レポート生成")
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--collection-dir", "--root", dest="collection_dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--case", default="current")
    args = parser.parse_args()
    run(args.collection_dir, args.params, args.out, scenario=args.scenario, case=args.case)


if __name__ == "__main__":
    main()
