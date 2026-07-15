"""Physical-validity verification for the models selected by a scenario.

The CSV cache is read once per dataset.  Every selected model is then evaluated
with its declared parameters; only experimental models additionally get the
data-driven fitting diagnostics.
"""
from __future__ import annotations

import argparse
import html
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs
import yaml

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
from .reidentify.model_config import ModelSpec, load_model_config


@dataclass
class PreparedDataset:
    dataset_id: str
    data: dict[str, Any]
    data_by_source: dict[str, dict[str, Any]]
    timeseries: dict[str, Any]


@dataclass
class ComparedModel:
    name: str
    vehicle_model_type: str
    acceleration_source: str
    params: dict[str, Any]

    @property
    def finalized(self) -> bool:
        return self.name in {"baseline", "v1"}


@dataclass
class ValidationContext:
    params: dict[str, Any]
    model_type: str
    wheelbase: float
    models: list[ComparedModel] = field(default_factory=list)
    datasets: list[PreparedDataset] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)


@dataclass
class ValidationStep:
    name: str
    result: dict[str, Any]
    html: str


def _number(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "—"
    return f"{number:.6g}" if np.isfinite(number) else "—"


def _figure_html(fig: go.Figure, title: str) -> str:
    return f"<h3>{html.escape(title)}</h3>{fig.to_html(full_html=False, include_plotlyjs=False)}"


def _histogram_by_model(values: dict[str, list[float]], title: str) -> go.Figure:
    fig = go.Figure()
    for name, samples in values.items():
        if samples:
            fig.add_trace(go.Histogram(x=samples, name=name, opacity=0.65))
    fig.update_layout(title=title, barmode="overlay", xaxis_title="RMSE", yaxis_title="データセット数")
    return fig


def _valid_number(params: dict[str, Any], key: str, *, positive: bool = False) -> tuple[float | None, str | None]:
    value = params.get(key)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None, f"{key} がありません"
    if not np.isfinite(number) or (positive and number <= 0.0):
        return None, f"{key} は{'正の' if positive else '有限の'}数値である必要があります"
    return number, None


def _models_from_inputs(params_path: Path, scenario: Path | None, case: str) -> list[ComparedModel]:
    tuned_document = yaml.safe_load(params_path.read_text(encoding="utf-8")) or {}
    tuned_params = dict(tuned_document.get("params", tuned_document))
    if scenario is None:
        return [ComparedModel(case, str(tuned_params.get("vehicle_model_type", case)), "accel", tuned_params)]
    config = load_model_config(scenario)
    models: list[ComparedModel] = []
    for name in config.comparison_models:
        spec: ModelSpec = config.find_case(name)
        params = dict(spec.params)
        if name == "current":
            params.update(tuned_params)
        models.append(ComparedModel(name, spec.vehicle_model_type, spec.acceleration_source, params))
    return models


def _add_xy(dataset: dict[str, Any], source: dict[str, Any]) -> float:
    kin = source["kinematic"]
    if kin.empty:
        raise ValueError("kinematic が空です")
    t0 = max(float(source[topic]["t_ns"].iloc[0]) for topic in ("cmd", "accel", "steering", "velocity", "kinematic"))
    t_grid = t0 + np.arange(len(dataset["vx"]), dtype=float) * 0.01e9
    source_t = kin["t_ns"].to_numpy(dtype=float)
    dataset["xy"] = tuple(np.interp(t_grid, source_t, kin[column].to_numpy(dtype=float)) for column in ("x", "y", "yaw", "vx", "wz"))
    return t0


def prepare_datasets(collection_dir: Path, params_path: Path, *, scenario: Path | None = None, case: str = "current") -> tuple[ValidationContext, ValidationStep]:
    """Read each cache once and prepare the requested acceleration sources."""
    models = _models_from_inputs(params_path, scenario, case)
    current = next((model for model in models if model.name == "current"), models[0])
    wheelbase, _ = _valid_number(current.params, "wheelbase", positive=True)
    context = ValidationContext(
        params=current.params, model_type=current.vehicle_model_type,
        wheelbase=wheelbase if wheelbase is not None else float("nan"), models=models,
    )
    sources = tuple(dict.fromkeys(model.acceleration_source for model in models))
    for dataset_id, csv_path in discover_cached_datasets(collection_dir):
        try:
            source = read_dataset_csv(csv_path)
            by_source: dict[str, dict[str, Any]] = {}
            t0 = 0.0
            for acceleration_source in sources:
                # Keep the public legacy call shape when no scenario is used.
                # Scenario-driven reports need the model's declared source.
                if scenario is None:
                    dataset = build_resampled(source, 0.01, context=f"physical_validity:{dataset_id}")
                else:
                    dataset = build_resampled(source, 0.01, context=f"physical_validity:{dataset_id}", acceleration_source=acceleration_source)
                if dataset is None:
                    raise ValueError("共通時間範囲が短すぎるか、必須信号が空です")
                t0 = _add_xy(dataset, source)
                by_source[acceleration_source] = dataset
            data = by_source[current.acceleration_source]
            context.datasets.append(PreparedDataset(dataset_id, data, by_source, {
                "dataset_id": dataset_id, "label": dataset_id, "_t0_ns": t0,
                "t": (np.arange(len(data["vx"]), dtype=float) * 0.01).tolist(),
                "a_cmd_raw": data["a_cmd"].tolist(), "a_act_raw": data["a_act"].tolist(),
                "d_cmd": data["d_cmd"].tolist(), "d_act_raw": data["d_act"].tolist(),
                "vx": data["vx"].tolist(), "mask_dyn": data["gear_drive"].tolist(),
            }))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            context.skipped.append(f"{dataset_id}: {exc}")
    skipped = "".join(f"<li>{html.escape(reason)}</li>" for reason in context.skipped)
    model_rows = "".join(
        f"<li><code>{html.escape(model.name)}</code>: {html.escape(model.vehicle_model_type)} / accel={html.escape(model.acceleration_source)} ({'確定済み' if model.finalized else '検討中'})</li>"
        for model in models
    )
    section = f'''<section id="prepare"><h2>準備</h2><p>有効データセット: {len(context.datasets)}</p>
<h3>比較モデル</h3><ul>{model_rows}</ul><h3>除外理由</h3>{f'<ul>{skipped}</ul>' if skipped else '<p>なし</p>'}</section>'''
    return context, ValidationStep("prepare", {"n_valid": len(context.datasets), "skipped": context.skipped, "models": [m.name for m in models]}, section)


def _data(item: PreparedDataset, model: ComparedModel) -> dict[str, Any]:
    return item.data_by_source[model.acceleration_source]


def _fixed_response(data: dict[str, Any], params: dict[str, Any], *, steer: bool) -> tuple[dict[str, Any] | None, np.ndarray | None, str | None]:
    prefix = "steer" if steer else "acc"
    tau, reason = _valid_number(params, f"{prefix}_time_constant", positive=True)
    delay, delay_reason = _valid_number(params, f"{prefix}_time_delay")
    if reason or delay_reason or delay is None or delay < 0.0:
        return None, None, reason or ("%s_time_delay は0以上である必要があります" % prefix)
    cmd = data["d_cmd"] if steer else data["a_cmd"]
    actual = data["d_act"] if steer else data["a_act"]
    threshold = 0.1 if steer else 0.15
    mask = data["gear_drive"] & (data["vx"] > 0.5) & (np.abs(np.gradient(cmd, 0.01)) > threshold)
    bias = 0.0
    if steer:
        bias, bias_reason = _valid_number(params, "steer_bias")
        if bias_reason:
            return None, None, bias_reason
    prediction = _simulate_first_order(cmd, tau, delay, 0.01) + bias
    residual = prediction[mask] - actual[mask]
    return {"rmse": float(np.sqrt(np.mean(residual ** 2))) if len(residual) else float("nan"), "n": int(len(residual))}, prediction, None


def _summary_table(models: list[ComparedModel], results: dict[str, dict[str, Any]], param_keys: tuple[str, ...]) -> str:
    baseline = results.get("baseline", {}).get("rmse", float("nan"))
    rows = []
    for model in models:
        value = results[model.name]
        rmse = value.get("rmse", float("nan"))
        if model.name == "baseline" and np.isfinite(rmse):
            ratio = 1.0
        else:
            ratio = rmse / baseline if np.isfinite(rmse) and np.isfinite(baseline) and baseline != 0 else float("nan")
        params = ", ".join(f"{key}={_number(model.params.get(key))}" for key in param_keys)
        reason = value.get("reason", "")
        rows.append(f"<tr><th>{html.escape(model.name)}</th><td>{_number(rmse)}</td><td>{_number(ratio)}</td><td>{value.get('n', 0)}</td><td><code>{html.escape(params)}</code></td><td>{html.escape(reason)}</td></tr>")
    return "<table><thead><tr><th>モデル</th><th>固定評価 RMSE</th><th>baseline 比</th><th>サンプル数</th><th>使用パラメータ</th><th>評価不能理由</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table>"


def _cross_diagnostic_figure(data: dict[str, Any], *, xy: bool, value: float, wheelbase: float | None = None) -> go.Figure:
    """Show a representative residual series for an experimental cross fit."""
    if xy:
        rx, ry = xy_residual(data, value)
        residual = np.sqrt(rx * rx + ry * ry)
        title = "x/y 残差の代表時系列"
    else:
        residual = yaw_residual(data, k_us=value, wheelbase=float(wheelbase))
        title = "yaw 残差の代表時系列"
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=np.arange(len(residual)) * 0.01, y=residual, name="残差"))
    fig.update_layout(title=title, xaxis_title="有効区間の時間 [s]", yaxis_title="残差")
    return fig


def _response_step(context: ValidationContext, *, steer: bool) -> ValidationStep:
    title, section_id = ("操舵", "steering") if steer else ("縦方向", "longitudinal")
    fixed: dict[str, dict[str, Any]] = {}
    distributions: dict[str, list[float]] = {}
    diagnostics = []
    for model in context.models:
        per_dataset: dict[str, Any] = {}
        samples: list[float] = []
        required = ("steer_time_constant", "steer_time_delay", "steer_bias") if steer else ("acc_time_constant", "acc_time_delay")
        missing_reason = next((reason for key in required for _value, reason in [_valid_number(model.params, key, positive=key.endswith("constant"))] if reason), "")
        for item in context.datasets:
            metrics, prediction, reason = _fixed_response(_data(item, model), model.params, steer=steer)
            per_dataset[item.dataset_id] = metrics or {"rmse": float("nan"), "n": 0, "reason": reason}
            if metrics and np.isfinite(metrics["rmse"]):
                samples.append(metrics["rmse"])
        n = sum(value.get("n", 0) for value in per_dataset.values())
        total_squared_error = sum(value.get("rmse", float("nan")) ** 2 * value.get("n", 0) for value in per_dataset.values() if np.isfinite(value.get("rmse", float("nan"))))
        fixed[model.name] = {
            "datasets": per_dataset,
            "rmse": float(np.sqrt(total_squared_error / n)) if n else float("nan"),
            "n": n,
            "reason": missing_reason or next((value.get("reason") for value in per_dataset.values() if value.get("reason")), ""),
        }
        distributions[model.name] = samples
        if not model.finalized:
            rows = []
            for item in context.datasets:
                data = _data(item, model)
                fit = fit_steering(data) if steer else fit_longitudinal(data)
                if fit:
                    row = dict(item.timeseries)
                    # The shared display row is based on current; replace its
                    # source-dependent signals before showing another model.
                    row.update({
                        "a_cmd_raw": data["a_cmd"].tolist(), "a_act_raw": data["a_act"].tolist(),
                        "d_cmd": data["d_cmd"].tolist(), "d_act_raw": data["d_act"].tolist(),
                        "vx": data["vx"].tolist(), "mask_dyn": data["gear_drive"].tolist(),
                    })
                    key = "d_sim_fit_raw" if steer else "a_sim_cross_raw"
                    row[key] = (_simulate_first_order(data["d_cmd"] if steer else data["a_cmd"], fit["tau"], fit["delay"], 0.01) + (fit.get("bias", 0.0) if steer else 0.0)).tolist()
                    rows.append(row)
            figure = build_fig_cross_steer(_pick_longest_contiguous_timeseries_row(rows)) if steer else build_fig_cross_long(_pick_longest_contiguous_timeseries_row(rows))
            diagnostics.append(_figure_html(figure, f"{model.name}: データ駆動フィット診断"))
    keys = ("steer_time_constant", "steer_time_delay", "steer_bias") if steer else ("acc_time_constant", "acc_time_delay")
    section = f'<section id="{section_id}"><h2>{title}</h2>' + _summary_table(context.models, fixed, keys) + _figure_html(_histogram_by_model(distributions, f"{title}: モデル別固定評価 RMSE"), "固定評価 RMSE 分布") + "".join(diagnostics) + "</section>"
    return ValidationStep(section_id, {"models": fixed, "datasets": fixed.get("current", {}).get("datasets", {})}, section)


def validate_longitudinal(context: ValidationContext) -> ValidationStep:
    return _response_step(context, steer=False)


def validate_steering(context: ValidationContext) -> ValidationStep:
    return _response_step(context, steer=True)


def _cross_step(context: ValidationContext, *, xy: bool) -> ValidationStep:
    title, section_id = ("x/y", "xy") if xy else ("yaw", "yaw")
    fixed: dict[str, dict[str, Any]] = {}
    distributions: dict[str, list[float]] = {}
    diagnostics: list[str] = []
    for model in context.models:
        params = model.params
        first, reason = _valid_number(params, "xy_heading_rate_coeff" if xy else "wheelbase", positive=not xy)
        second, second_reason = (None, None) if xy else _valid_number(params, "k_us")
        per_dataset: dict[str, Any] = {}
        samples: list[float] = []
        if reason or second_reason:
            why = reason or second_reason
            per_dataset = {item.dataset_id: {"rmse": float("nan"), "n": 0, "reason": why} for item in context.datasets}
        else:
            for item in context.datasets:
                data = _data(item, model)
                if xy:
                    residual = np.concatenate(xy_residual(data, first))
                else:
                    residual = yaw_residual(data, k_us=second, wheelbase=first)
                metric = {"rmse": float(np.sqrt(np.mean(residual ** 2))) if len(residual) else float("nan"), "n": int(len(residual))}
                per_dataset[item.dataset_id] = metric
                if np.isfinite(metric["rmse"]):
                    samples.append(metric["rmse"])
        n = sum(value["n"] for value in per_dataset.values())
        total_squared_error = sum(value["rmse"] ** 2 * value["n"] for value in per_dataset.values() if np.isfinite(value["rmse"]))
        fixed[model.name] = {"datasets": per_dataset, "rmse": float(np.sqrt(total_squared_error / n)) if n else float("nan"), "n": n, "reason": reason or second_reason or ""}
        distributions[model.name] = samples
        if not model.finalized and context.datasets:
            data = [_data(item, model) for item in context.datasets]
            fit = fit_xy_heading_rate_coeff(data, initial=float(first or 0.0)) if xy else fit_k_us(data, wheelbase=float(first))
            diagnostics.append(f"<p>{html.escape(model.name)}: 横断フィット {html.escape('xy_heading_rate_coeff' if xy else 'k_us')}={_number(fit.get('xy_heading_rate_coeff', fit.get('k_us')))}, RMSE={_number(fit['rmse'])}, n={fit['n']}</p>")
            representative = _data(context.datasets[0], model)
            fit_value = float(fit.get("xy_heading_rate_coeff", fit.get("k_us", 0.0)))
            diagnostics.append(_figure_html(
                _cross_diagnostic_figure(representative, xy=xy, value=fit_value, wheelbase=float(first) if not xy else None),
                f"{model.name}: 代表時系列",
            ))
    keys = ("xy_heading_rate_coeff",) if xy else ("wheelbase", "k_us")
    section = f'<section id="{section_id}"><h2>{title}</h2>' + _summary_table(context.models, fixed, keys) + _figure_html(_histogram_by_model(distributions, f"{title}: モデル別固定評価 RMSE"), "固定評価 RMSE 分布") + "".join(diagnostics) + "</section>"
    return ValidationStep(section_id, {"models": fixed, "datasets": fixed.get("current", {}).get("datasets", {})}, section)


def validate_yaw(context: ValidationContext) -> ValidationStep:
    return _cross_step(context, xy=False)


def validate_xy(context: ValidationContext) -> ValidationStep:
    return _cross_step(context, xy=True)


def assemble_html(context: ValidationContext, steps: list[ValidationStep]) -> str:
    sections = "\n".join(step.html for step in steps)
    return f'''<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>物理的妥当性検証</title>
<style>body{{font-family:sans-serif;max-width:1200px;margin:2rem auto;line-height:1.5}}section{{margin:2rem 0}}.plotly-graph-div{{max-width:100%}}table{{border-collapse:collapse;width:100%}}th,td{{border:1px solid #bbb;padding:.35rem;text-align:left}}</style>
<script>{get_plotlyjs()}</script></head><body><h1>車両モデル物理的妥当性検証</h1>{sections}</body></html>'''


def run(collection_dir: Path, params_path: Path, output: Path, *, scenario: Path | None = None, case: str = "current") -> Path:
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
