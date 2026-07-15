"""Generate the physical-validity report from finalized reidentify artifacts."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
from typing import Any

import plotly.graph_objects as go
from plotly.offline import get_plotlyjs
import numpy as np
import yaml

from .lib._figures._physical_validity import build_fig_cross_long, build_fig_cross_steer
from .lib._physical_validity import analyze_dataset, fit_k_us, fit_xy_heading_rate_coeff
from .reidentify.load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .reidentify.model_config import load_model_config
from .reidentify.settings import TARGET_MODEL_NAME


def _model_fingerprint(model_type: str, params: dict) -> str:
    payload = json.dumps({"model_type": model_type, "params": params}, sort_keys=True, ensure_ascii=False, default=str)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def _resolve_report_models(scenario: Path | None, case: str = "current") -> tuple[str, dict, str, dict, str]:
    if scenario is None:
        return "current", {}, "baseline", {}, "baseline"
    document = yaml.safe_load(Path(scenario).read_text(encoding="utf-8")) or {}
    models = document.get("Evaluation", {}).get("Conditions", {}).get("models", {})
    current = models.get(case, {})
    baseline = models.get("baseline", {})
    current_type = str(current.get("vehicle_model_type", ""))
    baseline_type = str(baseline.get("vehicle_model_type", ""))
    allowed = {"delay_steer_acc_geared_wo_fall_guard", "delay_steer_acc_geared_for_diffusion_planner"}
    if baseline_type not in allowed:
        raise ValueError("baseline model の vehicle_model_type は 'delay_steer_acc_geared_wo_fall_guard' または 'delay_steer_acc_geared_for_diffusion_planner' である必要があります")
    return current_type, dict(current.get("params") or {}), baseline_type, dict(baseline.get("params") or {}), "baseline"


def _metrics_cache_matches(path: Path, expected: dict[str, str]) -> bool:
    if not path.is_file():
        return False
    try:
        import pandas as pd
        frame = pd.read_csv(path, nrows=1)
    except (OSError, ValueError, ImportError):
        return False
    return all(key in frame.columns and str(frame.iloc[0][key]) == str(value) for key, value in expected.items())


def _figure_html(fig: go.Figure, title: str) -> str:
    return f"<h3>{html.escape(title)}</h3>" + fig.to_html(full_html=False, include_plotlyjs=False)


def build_release_note_html(
    deviation_html: str,
    long_fig: go.Figure,
    steer_fig: go.Figure,
    long_resid_hist_fig: go.Figure,
    steer_resid_hist_fig: go.Figure,
    long_fit_rmse_html: str,
    steer_fit_rmse_html: str,
    label: str = "current",
    params_filename: str = "",
    n_dataset: int = 0,
    current_model: str = "current",
    evaluation_v2_html: str = "",
) -> str:
    deviation = deviation_html or '<section id="deviation"><div class="note">--metrics-cache 未指定のため、このレポートでは省略されました。</div></section>'
    return f"""<!DOCTYPE html><html lang="ja"><head><meta charset="utf-8"><title>物理的妥当性レポート（要約）— {html.escape(label)}</title>
<script>document.addEventListener('DOMContentLoaded',()=>new IntersectionObserver(es=>es.forEach(e=>e.isIntersecting&&(e.target.dataset.visible='1'))).observe(document.body))</script>
<style>body{{font-family:sans-serif;max-width:1200px;margin:2rem auto}} .note{{padding:1rem;background:#fff4cc}}</style></head><body>
<h1>{html.escape(current_model)}モデルリリースレポート</h1><p>生成元: {html.escape(params_filename)} / 有効データセット数: {n_dataset}</p>
{deviation}{evaluation_v2_html}<section id="rn-fit"><h2>縦横モデルのフィッティング評価</h2>
{long_fit_rmse_html}{_figure_html(long_fig, '縦方向時系列')}{long_resid_hist_fig.to_html(full_html=False, include_plotlyjs=False)}
{steer_fit_rmse_html}{_figure_html(steer_fig, '操舵時系列')}{steer_resid_hist_fig.to_html(full_html=False, include_plotlyjs=False)}</section></body></html>"""


def _histogram(values: list[float], title: str) -> go.Figure:
    fig = go.Figure()
    if values:
        fig.add_trace(go.Histogram(x=values, name=title))
    fig.update_layout(title=title)
    return fig


def build_report(collection_dir: Path, params_path: Path, output: Path, *, scenario: Path | None = None, case: str = "current") -> Path:
    params_doc = yaml.safe_load(params_path.read_text(encoding="utf-8")) or {}
    params = params_doc.get("params", params_doc)
    current_type, current_case_params, _baseline_type, _baseline_params, _ = _resolve_report_models(scenario, case)
    wheelbase = float(params.get("wheelbase", current_case_params.get("wheelbase", 4.76012)))
    rows: list[dict] = []
    datasets: list[dict] = []
    figure_rows: list[dict] = []
    failures: list[str] = []
    for dataset_id, csv_path in discover_cached_datasets(collection_dir):
        try:
            dataset = build_resampled(read_dataset_csv(csv_path), 0.01, context=f"physical_validity:{dataset_id}")
            if dataset is None:
                raise ValueError("共通時間範囲が短すぎます")
            kin = read_dataset_csv(csv_path)["kinematic"]
            if kin.empty:
                raise ValueError("kinematic が空です")
            t0 = float(kin["t_ns"].iloc[0])
            t_grid = t0 + np.arange(len(dataset["vx"]), dtype=float) * 0.01e9
            source_t = kin["t_ns"].to_numpy(dtype=float)
            dataset["xy"] = tuple(
                np.interp(t_grid, source_t, kin[column].to_numpy(dtype=float))
                for column in ("x", "y", "yaw", "vx", "wz")
            )
            datasets.append(dataset)
            figure_rows.append({
                "dataset_id": dataset_id,
                "t": (np.arange(len(dataset["vx"]), dtype=float) * 0.01).tolist(),
                "a_cmd_raw": dataset["a_cmd"].tolist(),
                "a_act_raw": dataset["a_act"].tolist(),
                "d_cmd": dataset["d_cmd"].tolist(),
                "d_act_raw": dataset["d_act"].tolist(),
                "vx": dataset["vx"].tolist(),
                "mask_dyn": dataset["gear_drive"].tolist(),
            })
            result = analyze_dataset(dataset, wheelbase=wheelbase)
            rows.append({"dataset_id": dataset_id, **result})
        except (OSError, ValueError, KeyError, TypeError) as exc:
            failures.append(f"{dataset_id}: {exc}")
    yaw = fit_k_us(datasets, wheelbase=wheelbase)
    xy = fit_xy_heading_rate_coeff(datasets, initial=float(params.get("xy_heading_rate_coeff", 0.0))) if datasets else {"xy_heading_rate_coeff": float("nan"), "rmse": float("nan"), "n": 0}
    long_values = [r["longitudinal"]["rmse"] for r in rows if r.get("longitudinal")]
    steer_values = [r["steering"]["rmse"] for r in rows if r.get("steering")]
    long_fit = {"tau": float(params.get("acc_time_constant", 0.0)), "delay": float(params.get("acc_time_delay", 0.0))}
    long_fig = build_fig_cross_long(figure_rows, long_fit)
    steer_fig = build_fig_cross_steer(figure_rows)
    def metric_rmse(result: Any) -> str:
        if not isinstance(result, dict):
            return "—"
        value = result.get("rmse")
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "—"
        return f"{number:.6g}" if np.isfinite(number) else "—"

    summary = "".join(
        f"<tr><td>{html.escape(r['dataset_id'])}</td>"
        f"<td>{metric_rmse(r.get('longitudinal'))}</td>"
        f"<td>{metric_rmse(r.get('steering'))}</td>"
        f"<td>{metric_rmse({'rmse': r.get('yaw_rmse')})}</td></tr>"
        for r in rows
    )
    failure_html = "".join(f"<li>{html.escape(item)}</li>" for item in failures)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(f"""<!DOCTYPE html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>物理的妥当性レポート</title><style>body{{font-family:sans-serif;max-width:1200px;margin:2rem auto;line-height:1.5}}table{{border-collapse:collapse}}td,th{{border:1px solid #bbb;padding:.35rem .6rem}}.metric{{display:inline-block;margin:.5rem;padding:.8rem;background:#eef5ff}}</style><script>{get_plotlyjs()}</script></head><body>
<h1>車両モデル物理的妥当性検証レポート</h1><p>モデル: <code>{html.escape(current_type)}</code> / データセット数: {len(datasets)}</p>
<h2>フィット結果</h2><div class="metric">acc τ={long_fit['tau']:.4g}s, T={long_fit['delay']:.4g}s</div><div class="metric">steer τ={float(params.get('steer_time_constant', 0.0)):.4g}s, T={float(params.get('steer_time_delay', 0.0)):.4g}s</div><div class="metric">k_us={yaw['k_us']:.6g}, RMSE={yaw['rmse']:.6g}</div><div class="metric">xy_heading_rate_coeff={xy['xy_heading_rate_coeff']:.6g}, RMSE={xy['rmse']:.6g}</div>
<h2>データセット別評価</h2><table><thead><tr><th>dataset</th><th>縦 RMSE</th><th>操舵 RMSE</th><th>yaw RMSE</th></tr></thead><tbody>{summary or '<tr><td colspan="4">有効データなし</td></tr>'}</tbody></table>
<h2>残差分布</h2>{_figure_html(_histogram(long_values, '縦方向フィット RMSE'), '縦方向')}{_figure_html(_histogram(steer_values, '操舵フィット RMSE'), '操舵')}
<h2>代表時系列</h2>{_figure_html(long_fig, '縦方向応答')}{_figure_html(steer_fig, '操舵応答')}
<h2>N-step 評価</h2><p>詳細な horizon 別評価は <code>report.html</code> を参照してください。</p>
{f'<h2>スキップ理由</h2><ul>{failure_html}</ul>' if failures else ''}</body></html>""", encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="物理的妥当性レポート生成")
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--collection-dir", "--root", dest="collection_dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--case", default="current")
    args = parser.parse_args()
    build_report(args.collection_dir, args.params, args.out, scenario=args.scenario, case=args.case)


if __name__ == "__main__":
    main()
