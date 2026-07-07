#!/usr/bin/env python3
"""
物理的妥当性レポート生成スクリプト

アンダーステア係数と操舵不感帯の物理的妥当性を、
実機ログからの独立同定と理論式の両面から検証する HTML レポートを生成する。

使用法:
  python physical_validity_report.py \\
    --params <collection>/tuned_params.yaml \\
    --collection-dir <collection> \\
    --out <collection>/physical_validity_report.html
"""
from __future__ import annotations

import argparse
import hashlib
import html as _html_stdlib
import json
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import yaml

# ---------------------------------------------------------------------------


from driving_log_replayer_v2.real_log_sim_comparison.lib._collection import DatasetEntry  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._coverage import _curvature_coverage  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_cmd, resolve_lite_bag  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._map import load_map_ways, resolve_map_osm  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._models_config import (  # noqa: E402
    load_models_doc,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (  # noqa: E402
    HORIZONS as _HORIZONS,
    acc_score as _acc_score,
    aggregate_normalized as _agg_normalized,
    robust_score as _robust_score,
    steer_score as _steer_score,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._tune_report import _build_viewer_html  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune import (  # noqa: E402
    _BASELINE_MODEL,
    _discover,
    _eval as _tune_eval,
    _filter_by_date,
    load_datasets,
)

# ---------------------------------------------------------------------------
# 定数 (物理定数・フィット定数の SSOT は lib._physical_validity。本ファイル固有の
#       レポート表示用定数のみここで定義する)
# ---------------------------------------------------------------------------
from driving_log_replayer_v2.real_log_sim_comparison.lib._figures import (  # noqa: E402
    build_fig_cross_long,
    build_fig_cross_steer,
    build_fig_equation_residual_hist,
    build_fig_kus_single,
    build_fig_nstep_error_hist,
    build_fig_nstep_error_growth,
    build_fig_perfect_tracking_box,
    build_fig_perfect_tracking_traj,
)
from scipy.optimize import least_squares, minimize_scalar  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._fit_core import (  # noqa: E402
    _moving_avg,
    delay_shift,
    equation_residual_at_params,
    savgol_derivative,
    savgol_smooth,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._figures._common import apply_base_layout  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._fig_io import fig_to_compact_json  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._inline_assets import gzip_b64  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.step_report_html import _RENDER_GLUE  # noqa: E402

from driving_log_replayer_v2.real_log_sim_comparison.lib._physical_validity import (  # noqa: E402
    DWZ_MAX,
    VX_MIN_CURVE,
    WHEELBASE,
    WZ_MIN,
    _DRIFT_A_TH,
    _FIT_DT,
    _N_CROSS_FIT_DATASET,
    _PERF_HORIZONS,
    _PERF_STRIDE,
    _extract_kus_arrays,
    compute_cross_long_rows,
    compute_cross_steer_rows,
    compute_kus_bins,
    compute_perfect_tracking_data,
    fit_long_cross_dataset_bounded,
    fit_long_single,
    fit_steer_single,
    merged_model_params,
    _DELAY_CANDIDATES_LONG,
    _DELAY_CANDIDATES_STEER,
    _TAU_BOUNDS_LONG,
    _TAU_BOUNDS_STEER,
    _SG_WINDOW_LONG,
    _SG_WINDOW_STEER,
    _SG_POLYORDER,
)

_H_SPAN = {10: "≈0.33 s", 20: "≈0.67 s", 30: "≈1.0 s", 40: "≈1.33 s"}

# 1-2/1-5. 理想追従評価に使うデータセット数（レポート固有。ホライズン・stride の SSOT は
# lib._physical_validity の _PERF_HORIZONS / _PERF_STRIDE を import して使う）
_PERF_N_DATASET = 10                                   # 1-5 横方向 box plot 用 データセット数


def _resolve_report_models(
    scenario: Path | None, current_case: str
) -> tuple[str, dict, str, dict, str]:
    """Return current type/params, baseline type/params, and baseline case name."""
    if scenario is None:
        return _BASELINE_MODEL, {}, _BASELINE_MODEL, {}, "baseline"

    doc = load_models_doc(scenario)
    if current_case not in doc.models:
        raise ValueError(
            f"{scenario}: current case {current_case!r} が Conditions.models にありません"
        )
    reference = doc.overlay.reference_tag
    if not reference or reference not in doc.models:
        raise ValueError(
            f"{scenario}: Conditions.overlay.reference_tag に有効な baseline model が必要です"
        )
    current = doc.models[current_case]
    baseline = doc.models[reference]
    if current.vehicle_model_type is None or baseline.vehicle_model_type is None:
        raise ValueError("current/baseline model の vehicle_model_type は必須です")
    if baseline.vehicle_model_type not in {"delay_steer_acc_geared_wo_fall_guard", "delay_steer_acc_geared_for_diffusion_planner"}:
        raise ValueError(
            f"baseline model の vehicle_model_type は 'delay_steer_acc_geared_wo_fall_guard' または 'delay_steer_acc_geared_for_diffusion_planner' である必要があります。"
            f"現在の値: {baseline.vehicle_model_type}"
        )
    return (
        current.vehicle_model_type,
        dict(current.params),
        baseline.vehicle_model_type,
        dict(baseline.params),
        reference,
    )


def _model_fingerprint(model_type: str, params: dict) -> str:
    payload = json.dumps(
        {"model_type": model_type, "params": params},
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    )
    return hashlib.sha256(payload.encode()).hexdigest()[:16]


def _metrics_cache_matches(path: Path, expected: dict[str, str]) -> bool:
    try:
        header = pd.read_csv(path, nrows=1)
    except Exception as exc:  # noqa: BLE001
        print(f"  [WARN] metrics cache を読めません ({exc})。再計算します")
        return False
    for key, value in expected.items():
        if key not in header.columns or header.empty or str(header.iloc[0][key]) != value:
            print(f"  [INFO] metrics cache の {key} が現在条件と不一致。再計算します")
            return False
    return True


def _to_entries(ds_list: list) -> list[DatasetEntry]:
    """`_discover` の (uuid, lite_dir) タプルリストを lib 共有関数用の DatasetEntry へ変換する。

    real_lite の解決は lib._io.resolve_lite_bag（`real.lite.mcap` 単一ファイル →
    `real.lite` rosbag2 dir の順）に委ねる。comparison/scenarios は本レポートでは不要。
    """
    entries: list[DatasetEntry] = []
    for uuid, lite_dir in ds_list:
        lite_dir = Path(lite_dir)
        entries.append(DatasetEntry(
            dataset_id=uuid,
            dir=lite_dir,
            real_lite=resolve_lite_bag(lite_dir, "real"),
            comparison_dir=None,
            scenarios_dir=None,
            status="success",
        ))
    return entries


def _fit_single_worker(args: tuple) -> tuple[str, dict | None, dict | None]:
    """プロセスワーカー: 1 データセットの縦方向 / 操舵 一次遅れモデル同定。"""
    uuid, bag_str = args
    bag = Path(bag_str)
    return uuid, fit_long_single(bag), fit_steer_single(bag)


def fit_per_dataset(
    entries: list[DatasetEntry], n_jobs: int = 8,
) -> tuple[dict[str, dict], dict[str, dict]]:
    """縦方向 / 操舵の per-dataset 実行時フィットを並列実行する。

    対象は real_lite を持つ全 entries。最長連続時系列図は全件から候補を選ぶのが
    目的のため、旧 identify_{long,steer}_dynamics.py（事前 CSV 生成）や正典
    step_cross_dataset.py と同じく全件を母集団とする。
    Returns: (per_ds_long, per_ds_steer)  ({dataset_id: fit dict})
    """
    targets = [e for e in entries if e.real_lite is not None]
    per_ds_long: dict[str, dict] = {}
    per_ds_steer: dict[str, dict] = {}
    with ProcessPoolExecutor(max_workers=min(n_jobs, max(len(targets), 1))) as pool:
        futs = [
            pool.submit(_fit_single_worker, (e.dataset_id, str(e.real_lite)))
            for e in targets
        ]
        for i, fut in enumerate(as_completed(futs), 1):
            uuid, long_fit, steer_fit = fut.result()
            if long_fit is not None:
                per_ds_long[uuid] = long_fit
            if steer_fit is not None:
                per_ds_steer[uuid] = steer_fit
            if i % 100 == 0:
                print(f"  {i}/{len(targets)} フィット済み", flush=True)
    return per_ds_long, per_ds_steer


def optimize_tau_with_equation_residual(
    per_ds: dict[str, dict],
    delay_candidates: np.ndarray,
    dt: float,
    tau_bounds: tuple[float, float],
    filter_w: int = 1,
    fixed_tau: float = None,
) -> list[dict]:
    """遅延を candidates に固定し、方程式残差の MSE を最小化する tau を最適化する（または指定された fixed_tau を用いる）。"""
    results = []
    ds_precomputed = []
    for uuid, fit in per_ds.items():
        if "cmd_arr" not in fit or "act_arr" not in fit or "mask_arr" not in fit:
            continue
        cmd = np.asarray(fit["cmd_arr"], dtype=float)
        act = np.asarray(fit["act_arr"], dtype=float)
        mask = np.asarray(fit["mask_arr"], dtype=bool)
        if len(cmd) == 0 or mask.sum() == 0:
            continue

        dot_act = np.gradient(act, dt)
        if filter_w > 1:
            cmd_f = _moving_avg(cmd, filter_w)
            act_f = _moving_avg(act, filter_w)
            dot_act_f = _moving_avg(dot_act, filter_w)
        else:
            cmd_f = cmd.copy()
            act_f = act.copy()
            dot_act_f = dot_act.copy()

        ds_precomputed.append({
            "cmd_f": cmd_f,
            "act_f": act_f,
            "dot_act_f": dot_act_f,
            "mask": mask,
        })
    if not ds_precomputed:
        return []
    for delay in delay_candidates:
        n_steps = int(round(float(delay) / dt))
        x_list = []
        y_list = []
        for dp in ds_precomputed:
            cmd_del = delay_shift(dp["cmd_f"], n_steps)
            act_del = delay_shift(dp["act_f"], n_steps)
            mask = dp["mask"]
            x_list.append(cmd_del[mask] - act_del[mask])
            y_list.append(dp["dot_act_f"][mask])
        X = np.concatenate(x_list)
        Y = np.concatenate(y_list)
        tau_inv_min = 1.0 / tau_bounds[1]
        tau_inv_max = 1.0 / tau_bounds[0]

        if fixed_tau is not None:
            best_tau = fixed_tau
            tau_inv = 1.0 / fixed_tau
            resid = tau_inv * X - Y
        else:
            def _res(x):
                tau_inv = float(x[0])
                return tau_inv * X - Y

            res = least_squares(_res, [1.0 / 0.2], bounds=([tau_inv_min], [tau_inv_max]))
            best_tau = 1.0 / float(res.x[0])
            resid = res.fun
        best_mse = float(np.mean(resid ** 2))
        best_rmse = float(np.sqrt(best_mse))
        results.append({
            "delay": float(delay),
            "tau": best_tau,
            "rmse": best_rmse,
            "mean": float(np.mean(resid)),
            "std": float(np.std(resid)),
            "resid_samples": resid.tolist(),
        })
    if results:
        min_idx = np.argmin([r["rmse"] for r in results])
        for idx, r in enumerate(results):
            r["selected"] = (idx == min_idx)
    return results


def build_fig_residual_candidates_hist(
    results: list[dict],
    channel_label: str,
    unit_label: str,
) -> go.Figure:
    """遅延固定＆時定数最適化ペアごとの方程式残差ヒストグラムを、ドロップダウン付きのgo.Figureとして構成する。"""
    fig = go.Figure()
    valid_results = [r for r in results if len(r.get("resid_samples", [])) > 0]
    if not valid_results:
        fig.add_annotation(text="データなし", showarrow=False)
        return fig
    selected_idx = 0
    for idx, r in enumerate(valid_results):
        if r.get("selected", False):
            selected_idx = idx
            break
    for idx, r in enumerate(valid_results):
        arr = np.asarray(r["resid_samples"], dtype=float)
        arr = arr[np.isfinite(arr)]
        lo, hi = (float(x) for x in np.percentile(arr, [1, 99]))
        if not (hi > lo):
            lo, hi = float(arr.min()), float(arr.max() + 1e-9)
        counts, edges = np.histogram(arr, bins=80, range=(lo, hi))
        centers = (edges[:-1] + edges[1:]) / 2.0
        is_visible = (idx == selected_idx)
        fig.add_trace(go.Bar(
            x=centers.tolist(),
            y=counts.tolist(),
            width=float(edges[1] - edges[0]),
            marker_color="teal",
            opacity=0.75,
            name=f"Delay={r['delay']:.3f}s, τ={r['tau']:.3f}s",
            visible=is_visible,
        ))
    buttons = []
    for idx, r in enumerate(valid_results):
        visibility = [False] * len(valid_results)
        visibility[idx] = True
        title_text = f"{channel_label}: 方程式残差 E[k] の分布 (遅延={r['delay']:.3f}s, τ={r['tau']:.3f}s)"
        buttons.append(dict(
            label=f"遅延 {r['delay']:.3f} s (最適 τ={r['tau']:.3f} s)",
            method="update",
            args=[
                {"visible": visibility},
                {"title": title_text}
            ]
        ))
    fig.update_layout(
        updatemenus=[
            dict(
                active=selected_idx,
                buttons=buttons,
                x=0.0,
                xanchor="left",
                y=1.15,
                yanchor="top"
            )
        ]
    )
    fig.update_xaxes(title_text=f"方程式残差 E[k] = RHS − LHS  [{unit_label}]")
    fig.update_yaxes(title_text="サンプル数")
    selected_r = valid_results[selected_idx]
    title_text = f"{channel_label}: 方程式残差 E[k] の分布 (遅延={selected_r['delay']:.3f}s, τ={selected_r['tau']:.3f}s)"
    return apply_base_layout(fig, title=title_text, height=380)


_MATHJAX_HEAD = (
    "<script>"
    r"window.MathJax={tex:{inlineMath:[['\\(','\\)']],displayMath:[['\\[','\\]']]},"
    "svg:{fontCache:'global'}};"
    "</script>"
    "<script async src='https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js'></script>"
)
_PLOTLY_CDN = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'

_STYLE = """
body { font-family: sans-serif; max-width: 1300px; margin: 0 auto; padding: 20px; color: #333; }
h1 { color: #222; }
h2 { color: #444; border-bottom: 2px solid #bbb; padding-bottom: 4px; margin-top: 36px; }
h3 { color: #555; margin-top: 20px; }
p { line-height: 1.6; }
code { background: #f0f0f0; padding: 2px 4px; border-radius: 3px; font-size: 12px; }
.param-table { border-collapse: collapse; margin: 12px 0; font-size: 13px; }
.param-table td, .param-table th { border: 1px solid #ddd; padding: 6px 12px; }
.param-table th { background: #f5f5f5; }
.meta { color: #888; font-size: 12px; margin-bottom: 16px; }
.note { background: #fff8e1; border-left: 4px solid #ffc107; padding: 8px 12px;
        margin: 8px 0; font-size: 13px; }
nav a { margin-right: 12px; }
details { margin: 8px 0; }
details > summary {
  cursor: pointer; font-weight: bold; color: #555;
  padding: 4px 0; list-style: none; display: flex; align-items: center; gap: 6px;
}
details > summary::before { content: "▶"; font-size: 10px; color: #888; transition: transform 0.15s; }
details[open] > summary::before { transform: rotate(90deg); }
details > summary::-webkit-details-marker { display: none; }
figure { margin: 0; }
figure .plotly-fig { width: 100%; min-height: 120px; border: 1px solid #ddd;
                     border-radius: 4px; background: #fff; }
.plotly-fig.pending::before {
  content: "図を描画中…"; display: block; padding: 32px;
  color: #888; font-size: 13px; text-align: center; }
.fig-error { padding: 32px; color: #b91c1c; font-size: 13px; text-align: center; }
figcaption { margin-bottom: 6px; font-weight: 600; }
figcaption .fname { font-weight: 400; color: #888; font-size: 12px; margin-left: 8px; }
"""


def _lazy_fig_html(fig: go.Figure, fig_id: str, fname: str, caption: str | None = None) -> str:
    """`go.Figure` を gzip+base64 遅延描画の `<figure>` HTML に変換する。

    step_report_html.py が cases/*.fig.json を埋め込む方式（`plotly-fig pending` +
    `<script class='figspec'>` + `_RENDER_GLUE` による IntersectionObserver 遅延描画）と
    表示スタイル・埋め込み方式を統一するためのヘルパー。plot 内 `layout.title` は
    figcaption 見出しへ移し、レイアウトからは除去する（重なって隠れるため）。
    """
    text = fig_to_compact_json(fig)
    spec_obj = json.loads(text)
    layout = spec_obj.get("layout") or {}
    height = int(layout.get("height") or 400)
    title_text = None
    t = layout.get("title")
    if isinstance(t, dict):
        title_text = t.get("text")
    if "title" in layout:
        layout.pop("title", None)
        text = json.dumps(spec_obj, separators=(",", ":"), ensure_ascii=False)
    heading = caption if caption is not None else (title_text or fname)
    return (
        f"<figure><figcaption>{heading}"
        f"<span class='fname'>{_html_stdlib.escape(fname)}</span></figcaption>"
        f"<div class='plotly-fig pending' id='{fig_id}' style='height:{height}px'></div>"
        f"<script type='application/gzip+json+base64' class='figspec' "
        f"data-target='{fig_id}'>{gzip_b64(text)}</script>"
        f"</figure>"
    )


# ---------------------------------------------------------------------------
# Phase 1: 並列 MCAP 読み込みワーカー（k_us 分析 + カーブカバレッジ）
# ---------------------------------------------------------------------------
def _load_mcap_worker(args: tuple) -> dict | None:
    """プロセスワーカー: 1 MCAP から k_us 分析用データとカーブカバレッジを抽出。"""
    # 引数は (uuid: str, lite_dir: str, steer_bias: float | None) のタプル
    uuid, lite_dir_str, steer_bias = args
    lite_dir = Path(lite_dir_str)
    mcap = lite_dir / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None

    # k_us 分析用配列 (vx/wz/steer_eff/dwz) + 付随情報 (t/yaw) は lib と共通化
    rec = _extract_kus_arrays(mcap, steer_bias=steer_bias)
    if rec is None:
        return None
    t_k = rec["t"]
    vx = rec["vx"]

    try:
        df_cmd = load_cmd(mcap, "/control/command/control_cmd")
    except Exception:
        df_cmd = pd.DataFrame()

    # カーブカバレッジ（_coverage._curvature_coverage は t 列を要求）
    t_rel = t_k - t_k[0]
    kin_cv = pd.DataFrame({"t": t_rel, "yaw": rec["yaw"]})
    vel_cv = pd.DataFrame({"t": t_rel, "lon_vel": vx})
    try:
        cov = _curvature_coverage(kin_cv, vel_cv)
    except Exception:
        cov = {"curve_count": 0, "kappa_max_abs": 0.0}

    # cmd_steer（stride=10 で間引き: 操舵信号 of 時系列表示用）
    _STRIDE = 10
    t_sub = t_k[::_STRIDE]
    if not df_cmd.empty:
        t_c = df_cmd["t_ns"].values * 1e-9
        cmd_steer_arr = np.interp(t_sub, t_c, df_cmd["cmd_steer"].values).tolist()
    else:
        cmd_steer_arr = []

    return {
        "uuid": uuid,
        "lite_dir": lite_dir_str,
        "vx": vx.tolist(),
        "wz": rec["wz"].tolist(),
        "steer_eff": rec["steer_eff"].tolist(),
        "dwz": rec["dwz"].tolist(),
        "gear_drive": rec["gear_drive"].tolist(),
        "curve_count": cov["curve_count"],
        "kappa_max_abs": cov["kappa_max_abs"],
        "cmd_steer": cmd_steer_arr,
    }


def load_all_mcap(ds_list: list, steer_bias: float | None = None, n_jobs: int = 8) -> list[dict]:
    """全データセットを並列 MCAP 読み込み。"""
    args_list = [(uuid, str(lite_dir), steer_bias) for uuid, lite_dir in ds_list]
    results: list[dict] = []
    with ProcessPoolExecutor(max_workers=n_jobs) as pool:
        futs = {pool.submit(_load_mcap_worker, a): a for a in args_list}
        for i, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            if r is not None:
                results.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(args_list)} 読み込み済み", flush=True)
    return results


# ---------------------------------------------------------------------------
# HTML セクション組み立て
# ---------------------------------------------------------------------------
def _build_sec_metrics(baseline_score: float | None, phase14_score: float, label: str = "current") -> str:
    """各種メトリクスの直感的・物理的解説セクション。"""
    if baseline_score and baseline_score > 0:
        improvement_pct = (baseline_score - phase14_score) / baseline_score * 100
        nyaw_ratio = 100 - improvement_pct
        score_bullet = (
            f"  <li>{label} の score = <b>{phase14_score:.3f}</b>"
            f"（baseline = <b>{baseline_score:.3f}</b> → 約 <b>{improvement_pct:.1f}%</b> 改善）</li>"
        )
        note_text = (
            f"{label}（{phase14_score:.3f}）が baseline（\\(k_{{\\mathrm{{us}}}}=0\\) / "
            f"<code>steer_dead_band</code>=0, score={baseline_score:.3f}）より"
            f" {improvement_pct:.1f}% 低いことは、yaw/lat 誤差が baseline の約 {nyaw_ratio:.0f}% まで縮小したことを意味する。"
        )
    else:
        score_bullet = f"  <li>{label} の score = <b>{phase14_score:.3f}</b></li>"
        note_text = f"baseline score が未計算のため比較なし（--metrics-cache を指定すると比較が有効になります）。"
    # LaTeX を含む静的部分は通常文字列。プレースホルダを後置換で動的値に差し替える
    tmpl = """
<section id="metrics">
<h2>0. 評価メトリクスの物理的意味</h2>

<details>
<summary>0-1. N-step Open Loop評価: 前向き積分誤差</summary>
<p>
車両モデルを実機ログの初期状態から N 個の制御コマンド区間だけ前向きに積分し、
実機の自己位置推定軌跡との終端誤差を評価する。
制御コマンドは 30 Hz（\\(\\Delta t = 1/30\\) 秒 \\(\\approx 33\\) ms）で記録されており、
ホライズン N=10（≈ 0.33 秒先）〜 N=40（≈ 1.33 秒先）の 4 点を等重みで集約する。
</p>
<table class="param-table">
  <tr><th>ホライズン</th><th>時間スパン（30 Hz 基準）</th><th>主に捉える現象</th></tr>
  <tr><td>N=10</td><td>≈ 0.33 秒先</td><td>アクチュエータ遅れ・一次遅れ時定数（即応性）</td></tr>
  <tr><td>N=20</td><td>≈ 0.67 秒先</td><td>中期の操舵追従・加速度変動</td></tr>
  <tr><td>N=30</td><td>≈ 1.0 秒先</td><td>ホイールベース・ステアバイアスの累積効果</td></tr>
  <tr><td>N=40</td><td>≈ 1.33 秒先</td><td>アンダーステア・カーブ全体の軌跡ドリフト</td></tr>
</table>
<div class="note">
<b>直感</b>: N=10 でいい成績でも N=40 が悪い場合、モデルは「短期の応答」は捉えているが
「カーブを曲がり続ける能力」に欠陥がある（= アンダーステア補正や累積バイアスの問題）。
</div>
</details>

<details>
<summary>0-2. 誤差の 3 成分（yaw・long・lat）</summary>
<table class="param-table">
  <tr><th>成分</th><th>単位</th><th>物理的意味</th><th>主な感度</th></tr>
  <tr>
    <td><b>yaw 誤差</b></td><td>deg</td>
    <td>N ステップ後の車両姿勢角（ヨー角）誤差。旋回量の過不足を示す。</td>
    <td>操舵一次遅れ時定数 \\(\\tau_\\delta\\)、\\(k_{{\\mathrm{{us}}}}\\)、ホイールベース \\(L\\)</td>
  </tr>
  <tr>
    <td><b>long 誤差</b></td><td>cm</td>
    <td>進行方向の位置誤差（前後方向）。加速度の積算ズレを示す。</td>
    <td>加速度一次遅れ時定数 \\(\\tau_a\\)、純粋遅延 \\(T_a\\)</td>
  </tr>
  <tr>
    <td><b>lat 誤差</b></td><td>cm</td>
    <td>横方向の位置誤差。操舵追従精度とアンダーステアの積算効果を示す。</td>
    <td>\\(k_{{\\mathrm{{us}}}}\\)、<code>steer_dead_band</code>、\\(\\tau_\\delta\\)、<code>steer_bias</code></td>
  </tr>
</table>
<div class="note">
<b>long ⊥ steer の直交性</b>: 低速・定常走行では long 誤差は加速度パラメータのみに感度を持ち、
steer 系パラメータへの感度はほぼゼロ（逆もしかり）。これを利用して2フェーズ独立チューニングを実現。
</div>
</details>

<details>
<summary>0-3. 正規化スコア（nyaw, nlong, nlat）</summary>
<p>
各データセットの誤差を <b>baseline モデル</b>（補正なし遅延モデル、\\(k_{{\\mathrm{{us}}}}=0\\)・<code>steer_dead_band</code>=0 相当）の
誤差で正規化する:
\\[
\\text{nyaw} = \\frac{\\text{yaw}_{\\mathrm{tuned}}}{\\max(\\text{yaw}_{\\mathrm{baseline}},\\; \\text{floor}_{\\mathrm{yaw}})}
\\]
</p>
<ul>
  <li>\\(\\text{nyaw} < 1\\): baseline より良い（チューニング済みが有効）</li>
  <li>\\(\\text{nyaw} = 1\\): baseline と同等</li>
  <li>\\(\\text{nyaw} > 1\\): baseline より悪い（過補正・副作用）</li>
</ul>
<p><b>なぜ正規化するか</b>:
絶対誤差のまま集約すると、大カーブ・高速など「難しいシナリオ」（baseline 誤差が大きいデータセット）が
スコアを支配してしまい、全 650 データセットで均等に改善できているかを測れない。
正規化により「baseline と比べてどれだけ改善したか」を全データセットで統一スケールで評価できる。
</p>
<table class="param-table">
  <tr><th>フロア定数</th><th>N=10</th><th>N=40</th><th>目的</th></tr>
  <tr><td>YAW_FLOOR</td><td>0.06 deg</td><td>0.24 deg</td>
    <td>低ダイナミクス（ほぼ直進）のデータセットで分母がゼロ近くになる暴発を防ぐ</td></tr>
  <tr><td>LONG_FLOOR</td><td>1.0 cm</td><td>4.5 cm</td><td>縦方向の同上</td></tr>
  <tr><td>LAT_FLOOR</td><td>0.3 cm</td><td>1.2 cm</td><td>横方向の同上</td></tr>
</table>
</details>

<details>
<summary>0-4. mean と worst</summary>
<p>
650 データセットの正規化スコアに対して 2 種類の集約を行う:
</p>
<ul>
  <li><b>mean</b>: 全データセットの平均。「全体的に良い設定」を測る。</li>
  <li><b>worst</b>: 全データセットの最大値（最悪ケース）。「どのシナリオでも崩れない頑健性」を測る。</li>
</ul>
<p>
mean だけを最小化すると、一部のデータセットに特化したパラメータが選ばれ worst が悪化することがある。
worst だけだと過度に保守的になる。両者を組み合わせることでロバストな設定を探索する。
</p>
</details>

<details>
<summary>0-5. ロバストスコア（robust_score）</summary>
<p>
最終目的関数:
\\[
\\text{score} = \\sum_{h \\in \\{10,20,30,40\\}} \\left[
  (\\overline{\\text{nyaw}} + 0.5 \\overline{\\text{nlong}} + 0.5 \\overline{\\text{nlat}})
+ 0.5 (\\hat{\\text{nyaw}} + 0.5 \\hat{\\text{nlong}} + 0.5 \\hat{\\text{nlat}})
\\right]
\\]
ここで \\(\\overline{\\cdot}\\) は mean、\\(\\hat{\\cdot}\\) は worst（全データセットの max）。
<b>スコアは小さいほど良い。</b>
</p>
<ul>
  <li>yaw の重み = 1（位置の重み 0.5 + 0.5 = 1 と均等）</li>
  <li>long と lat は各 0.5 倍（yaw : 位置 = 1 : 1 を維持）</li>
  <li>worst 項の重み 0.5 = 「mean の改善と worst の頑健性を半々で重視」</li>
__SCORE_BULLET__
</ul>
<div class="note">
<b>直感的なスケール</b>: score が 1 下がると「全 650 データセット・全 4 ホライズンで平均的に
nyaw が 1/8 改善した」相当（sum over 4 horizons × 2 terms (mean+0.5worst) でほぼ 8 で割る）。
__NOTE_TEXT__
</div>
</details>
</section>
"""
    return tmpl.replace("__SCORE_BULLET__", score_bullet).replace("__NOTE_TEXT__", note_text)


def _kus_band_table_rows(params: dict) -> str:
    """k_us パラメータの HTML テーブル行を生成。"""
    k_us = params.get("k_us", 0.0)
    return f"  <tr><td><code>k_us</code></td><td>{k_us:.6f} rad·s²/m</td><td>アンダーステア係数（全速度域一定）</td></tr>"


_ACCEL_PHASE_NAMES: dict[str, str] = {"accel": "加速中", "cruise": "巡航中", "decel": "減速中"}


def _split_resid_by_phase(e: np.ndarray, act_masked: np.ndarray) -> dict[str, np.ndarray]:
    """残差配列 e（mask 適用済み）を、対応する実測加速度 act_masked の符号で
    加速中 (> +_DRIFT_A_TH) / 巡航中 / 減速中 (< -_DRIFT_A_TH) に3分割する。

    閾値は `lib._physical_validity._long_drift_profile` の phase 分類（_DRIFT_A_TH=0.3 [m/s²]）
    と同じ規約を流用し、レポート内で「加速/巡航/減速」の定義を統一する。
    """
    return {
        "accel": e[act_masked > _DRIFT_A_TH],
        "cruise": e[(act_masked >= -_DRIFT_A_TH) & (act_masked <= _DRIFT_A_TH)],
        "decel": e[act_masked < -_DRIFT_A_TH],
    }


def _pool_resid_long_tuned_by_phase(per_ds: dict[str, dict]) -> dict[str, list[float]]:
    """縦方向 tuned: 各データセット既存の残差（各データセット自身の同定済み tau/delay で
    計算済みの `fit["resid_samples"]`）を、実測加速度の符号で加速/巡航/減速に分けて
    データセット横断で RMSE をプールする（再計算不要）。
    """
    out: dict[str, list[float]] = {name: [] for name in _ACCEL_PHASE_NAMES}
    for fit in per_ds.values():
        resid_samples = fit.get("resid_samples")
        if not resid_samples or "act_arr" not in fit or "mask_arr" not in fit:
            continue
        e = np.asarray(resid_samples, dtype=float)
        mask = np.asarray(fit["mask_arr"], dtype=bool)
        act_masked = np.asarray(fit["act_arr"], dtype=float)[mask]
        if len(act_masked) != len(e):
            continue
        for name, sub in _split_resid_by_phase(e, act_masked).items():
            if sub.size > 0:
                out[name].append(float(np.sqrt(np.mean(sub ** 2))))
    return out


def _pool_resid_long_baseline_by_phase(
    per_ds: dict[str, dict], tau: float, delay: float, window_s: float,
) -> dict[str, list[float]]:
    """縦方向 baseline: 全データセット共通の baseline (tau, delay) を実測配列に代入して
    方程式残差を再計算し、実測加速度の符号で加速/巡航/減速に分けて RMSE をプールする。
    """
    out: dict[str, list[float]] = {name: [] for name in _ACCEL_PHASE_NAMES}
    for fit in per_ds.values():
        if "cmd_arr" not in fit or "act_arr" not in fit or "mask_arr" not in fit:
            continue
        cmd = np.asarray(fit["cmd_arr"], dtype=float)
        act = np.asarray(fit["act_arr"], dtype=float)
        mask = np.asarray(fit["mask_arr"], dtype=bool)
        t_s = np.asarray(fit.get("t_s", np.arange(len(cmd)) * _FIT_DT), dtype=float)
        resid = equation_residual_at_params(
            cmd, act, mask, _FIT_DT, tau=tau, delay=delay,
            scale=1.0, bias=0.0, window_s=window_s, polyorder=_SG_POLYORDER, t_s=t_s,
        )
        e = np.asarray(resid["resid"], dtype=float)
        act_masked = act[mask]
        if len(act_masked) != len(e):
            continue
        for name, sub in _split_resid_by_phase(e, act_masked).items():
            if sub.size > 0:
                out[name].append(float(np.sqrt(np.mean(sub ** 2))))
    return out


def _build_fit_rmse_table_html(
    tuned_rmses: list[float],
    baseline_rmses: list[float],
    unit_label: str,
    label: str,
    phase_tuned: dict[str, list[float]] | None = None,
    phase_baseline: dict[str, list[float]] | None = None,
) -> str:
    """方程式残差RMSE (J) の tuned vs baseline 比較テーブル（平均・中央値・99%ile、データセット横断）。

    1-2（縦方向）・1-3（操舵）の各セクション末尾に挿入する。ここでの J は
    `equation_residual_at_params` の `rmse_resid`（同じ実測配列に tuned / baseline
    パラメータをそれぞれ代入して評価した方程式残差 RMSE）であり、各セクション末尾の
    param-table にある「同定誤差量（RMSE）」列（\\(a_{sim}-a_{real}\\) 等の出力誤差、
    数値は未表示）とは別の量である点に注意。

    `phase_tuned`/`phase_baseline`（縦方向のみ; `_pool_resid_long_*_by_phase` の戻り値）を
    渡すと、実測加速度の符号による 加速中/巡航中/減速中 の内訳行を追加する。
    """
    if not tuned_rmses or not baseline_rmses:
        return ""

    def _cell(t_val: float, b_val: float) -> str:
        ratio = t_val / b_val if b_val > 0 else 1.0
        if ratio < 0.99:
            style = ' style="color:#28a745;font-weight:bold"'
        elif ratio > 1.01:
            style = ' style="color:#dc3545"'
        else:
            style = ""
        return f"<td{style}>{t_val:.4g}</td>"

    def _stats(vals: list[float]) -> tuple[float, float, float]:
        arr = np.asarray(vals)
        return float(arr.mean()), float(np.median(arr)), float(np.quantile(arr, 0.99))

    t_mean, t_med, t_p99 = _stats(tuned_rmses)
    b_mean, b_med, b_p99 = _stats(baseline_rmses)

    phase_rows = ""
    if phase_tuned and phase_baseline:
        row_blocks = []
        for name, name_ja in _ACCEL_PHASE_NAMES.items():
            t_vals, b_vals = phase_tuned.get(name, []), phase_baseline.get(name, [])
            if not t_vals or not b_vals:
                continue
            pt_mean, pt_med, pt_p99 = _stats(t_vals)
            pb_mean, pb_med, pb_p99 = _stats(b_vals)
            row_blocks.append(
                f'<tr><td><b>{label}</b>（{name_ja}）</td>'
                f'{_cell(pt_mean, pb_mean)}{_cell(pt_med, pb_med)}{_cell(pt_p99, pb_p99)}</tr>\n'
                f'<tr><td style="color:#888">baseline（{name_ja}）</td>'
                f'<td>{pb_mean:.4g}</td><td>{pb_med:.4g}</td><td>{pb_p99:.4g}</td></tr>'
            )
        if row_blocks:
            phase_rows = (
                '<tr><td colspan="4" style="background:#f0f0f0">'
                f'<b>フェーズ別内訳</b>（実測加速度: 加速 &gt; +{_DRIFT_A_TH:g}, '
                f'減速 &lt; -{_DRIFT_A_TH:g} m/s²、それ以外は巡航中</td></tr>\n'
                + "\n".join(row_blocks)
            )

    return f"""
<h3>フィッティング精度: 方程式残差RMSE 比較（{label} vs baseline）</h3>
<p>
全データセットについて、同じ実測配列に {label} パラメータと baseline（補正なし）を
それぞれ代入し、方程式残差 \\(J\\)（<code>equation_residual_at_params</code> の
<code>rmse_resid</code>、単位 {unit_label}）をデータセット横断で集計した。
これは上の残差ヒストグラム（{label} vs baseline 重ね描画）と同じ量であり、
本節末尾の表にある「同定誤差量（RMSE）」列（\\(a_{{\\mathrm{{sim}}}} - a_{{\\mathrm{{real}}}}\\) 等の
出力誤差）とは別物である。
</p>
<table class="param-table" style="font-size:12px">
  <thead><tr><th>モデル</th><th>平均</th><th>中央値</th><th>99%ile</th></tr></thead>
  <tbody>
    <tr><td><b>{label}</b></td>{_cell(t_mean, b_mean)}{_cell(t_med, b_med)}{_cell(t_p99, b_p99)}</tr>
    <tr><td style="color:#888">baseline</td>
        <td>{b_mean:.4g}</td><td>{b_med:.4g}</td><td>{b_p99:.4g}</td></tr>
{phase_rows}
  </tbody>
</table>
<div class="note">
{label} の値が baseline より小さい場合は <b style="color:#28a745">緑（改善）</b>、
大きい場合は <span style="color:#dc3545">赤（悪化）</span> で表示。
</div>
"""


def _build_sec1(
    params: dict,
    long_fig: go.Figure,
    steer_fig: go.Figure,
    kus_fig: go.Figure,
    n_dataset: int,
    long_resid_hist_fig: go.Figure | None = None,
    steer_resid_hist_fig: go.Figure | None = None,
    long_resid_opt_html: str = "",
    steer_resid_opt_html: str = "",
    long_fit_rmse_html: str = "",
    steer_fit_rmse_html: str = "",
) -> str:
    """1-0. 座標系と主要な記号の定義 + モデルパラメータの定義を含む sec1 全体 HTML を返す。"""
    kus_rows = _kus_band_table_rows(params)

    # ---------- モデルパラメータの定義（旧 _build_sec_model_params）----------
    def _fmt_p(v) -> str:
        if isinstance(v, float):
            return f"{v:.6g}"
        return str(v)

    kus_profile_rows = (
        f"  <tr><td><code>k_us</code></td>"
        f"<td>{_fmt_p(params.get('k_us', 0.0))} rad·s²/m</td>"
        f"<td>全速度域で一定のアンダーステア係数</td></tr>"
    )


    def _fmt(v) -> str:  # noqa: E306 (ローカル _fmt。上の _fmt_p とスコープが異なる)
        if isinstance(v, float):
            return f"{v:.6g}"
        return str(v)

    tau_a = _fmt(params.get("acc_time_constant", "N/A"))
    T_a   = _fmt(params.get("acc_time_delay", "N/A"))
    tau_d = _fmt(params.get("steer_time_constant", "N/A"))
    T_d   = _fmt(params.get("steer_time_delay", "N/A"))
    DSF   = _fmt(params.get("debug_steer_scaling_factor", "N/A"))
    beta  = _fmt(params.get("steer_bias", "N/A"))
    db    = _fmt(params.get("steer_dead_band", "N/A"))
    rlim  = _fmt(params.get("steer_rate_lim", "N/A"))

    _long_html_inner = _lazy_fig_html(
        long_fig, "fig-sec-long-timeseries", "sec1-2/long_fit（dataset横断）",
    )
    _steer_html_inner = _lazy_fig_html(
        steer_fig, "fig-sec-steer-timeseries", "sec1-3/steer_fit（dataset横断）",
    )
    long_html  = f"<details><summary>時系列グラフを表示（クリックで展開）</summary>{_long_html_inner}</details>"
    steer_html = f"<details><summary>時系列グラフを表示（クリックで展開）</summary>{_steer_html_inner}</details>"
    kus_html   = kus_fig.to_html(full_html=False, include_plotlyjs=False)
    long_resid_hist_html = (
        long_resid_hist_fig.to_html(full_html=False, include_plotlyjs=False)
        if long_resid_hist_fig is not None else ""
    )
    steer_resid_hist_html = (
        steer_resid_hist_fig.to_html(full_html=False, include_plotlyjs=False)
        if steer_resid_hist_fig is not None else ""
    )
    return f"""
<section id="sec-coords">
<h2>1-0. 座標系と主要な記号の定義</h2>
<p>
本レポートの運動方程式は、以下の車体基準（body frame）および進行方向基準の座標系に基づく。
「実装ラベル」列は、シミュレータ内部の状態ベクトル（6 状態、詳細は
<a href="#sec-state-space">1-1 節</a>）の IDX か、対応する YAML 設定名を示す（該当しない記号は入力・
導出量などであり、実装ラベル欄には <code>—</code> あるいは補助注記を記す）。「ROS トピック / フィールド」列は、
実機ログ（<code>real.lite</code>）と scenario_simulator_v2 の双方で共通に使われるトピック名と
フィールドを <code>topic.field</code> 形式でまとめる。
</p>
<table class="param-table">
  <tr><th>記号</th><th>意味</th><th>単位</th><th>実装ラベル</th>
      <th>ROS トピック / フィールド</th>
  </tr>
  <tr><td>\\(x, y\\)</td><td>地図平面上の車両位置</td><td>m</td>
      <td><code>IDX::X</code>, <code>IDX::Y</code></td>
      <td><code>/localization/kinematic_state.pose.pose.position.{{x, y}}</code></td>
  </tr>
  <tr><td>\\(\\theta\\)</td><td>ヨー角（車体の向き。地図 X 軸からの反時計回り回転）</td><td>rad</td>
      <td><code>IDX::YAW</code></td>
      <td><code>/localization/kinematic_state.pose.pose.orientation</code></td>
  </tr>
  <tr><td>\\(v_x\\)</td><td>前進速度（進行方向＝車体 X 軸成分。<code>lon_vel</code>）</td><td>m/s</td>
      <td><code>IDX::VX</code></td>
      <td><code>/vehicle/status/velocity_status.longitudinal_velocity</code></td>
  </tr>
  <tr><td>\\(\\delta_{{\\mathrm{{act}}}}\\)</td><td>前輪実ステア角</td><td>rad</td>
      <td><code>IDX::STEER</code></td>
      <td><code>/vehicle/status/steering_status.steering_tire_angle</code></td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{act}}}}\\)</td>
      <td>加速度アクチュエータの内部状態（勾配重力加算前の出力）</td><td>m/s²</td>
      <td><code>IDX::PEDAL_ACCX</code></td>
      <td>—（直接の実測トピックなし。次行 \\(a_{{\\mathrm{{report}}}}\\) を実測相当として使用）</td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{report}}}}\\)</td>
      <td>実測・可視化用の縦加速度。<code>calcModel</code> の状態ではなく、
      Euler 更新後に \\((v_{{x,\\mathrm{{new}}}} - v_{{x,\\mathrm{{prev}}}}) / \\Delta t\\) として事後計算される
      出力量。モデル上は \\(a_{{\\mathrm{{report}}}} = a_{{\\mathrm{{act}}}} + a_{{\\mathrm{{slope}}}}\\)。</td>
      <td>m/s²</td><td><code>IDX::ACCX</code></td>
      <td><code>/localization/acceleration.accel.accel.linear.x</code></td>
  </tr>
  <tr><td>\\(v_y\\)</td><td>横速度</td><td>m/s</td>
      <td>—（モデルの <code>getVy()</code> は常に 0 固定）</td>
      <td><code>/localization/kinematic_state.twist.twist.linear.y</code></td>
  </tr>
  <tr><td>\\(\\omega\\)</td><td>ヨーレート（\\(\\dot\\theta\\)）</td><td>rad/s</td>
      <td>—（導出量。<code>calc_yaw_rate</code> で算出）</td>
      <td><code>/localization/kinematic_state.twist.twist.angular.z</code></td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{slope}}}}\\)</td><td>路面勾配による重力加速度成分</td><td>m/s²</td>
      <td>—（入力 <code>SLOPE_ACCX</code>）</td>
      <td>専用トピックなし（pitch から算出）</td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{cmd,des}}}}\\)</td><td>加速度指令</td><td>m/s²</td>
      <td>—（入力）</td>
      <td><code>/control/command/control_cmd.longitudinal.acceleration</code></td>
  </tr>
  <tr><td>\\(\\delta_{{\\mathrm{{cmd,des}}}}\\)</td><td>操舵指令</td><td>rad</td>
      <td>—（入力）</td>
      <td><code>/control/command/control_cmd.lateral.steering_tire_angle</code></td>
  </tr>
  <tr><td><code>gear</code></td><td>ギア状態。運動方程式の連続入力ではなく、DRIVE 系
      （<code>GearReport.report</code> の enum 値 2..19）だけを同定・評価に使うための離散マスク</td><td>—</td>
      <td>—（評価マスク）</td>
      <td><code>/vehicle/status/gear_status.report</code>（直前値で時刻対応）</td>
  </tr>
  <tr><td>\\(\\beta\\)</td><td>ステアバイアス（系統的操舵オフセット）</td><td>rad</td>
      <td><code>steer_bias</code></td>
      <td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(L\\)</td><td>ホイールベース</td><td>m</td>
      <td><code>wheelbase</code></td>
      <td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(k_{{\\mathrm{{us}}}}\\)</td>
      <td>アンダーステア係数（全速度域一定）</td><td>rad·s²/m</td>
      <td><code>k_us</code></td>
      <td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(\\tau_\\delta\\)</td><td>操舵 1 次遅れ時定数</td><td>s</td>
      <td><code>steer_time_constant</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(T_\\delta\\)</td><td>操舵純粋遅延</td><td>s</td>
      <td><code>steer_time_delay</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(\\tau_a\\)</td><td>加速度 1 次遅れ時定数</td><td>s</td>
      <td><code>acc_time_constant</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(T_a\\)</td><td>加速度純粋遅延</td><td>s</td>
      <td><code>acc_time_delay</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(K_{{\\mathrm{{steer\\_scale}}}}\\)</td><td>操舵指令スケーリング倍率（1.0 = 補正なし）</td><td>—</td>
      <td><code>debug_steer_scaling_factor</code></td><td>—（設定パラメータ）</td>
  </tr>
</table>
</section>

<section id="sec-state-space">
<h2>1-1. 状態空間モデルと数値積分（Euler 法によるアップデート）</h2>
<p>
1-2（縦方向）・1-3（操舵）・1-4（ヨー・横方向）で個別に同定する運動方程式を、実装では一つの状態ベクトルに
まとめた \\(\\dot{{\\mathbf{{x}}}} = f(\\mathbf{{x}}, \\mathbf{{u}})\\) として扱い、共通の Euler 積分ループで毎ステップ時間発展させている。
各記号の意味・単位・ROS トピックは <a href="#sec-coords">1-0 の記号表</a>を参照。
</p>
<p>
<b>同定の統一ストーリー</b>: 各状態方程式を \\(\\mathrm{{LHS}} = \\mathrm{{RHS}}(\\theta)\\) と見て、
方程式残差を
\\[
E[k] = \\mathrm{{RHS}}[k;\\theta] - \\mathrm{{LHS}}[k]
\\]
で定義する。これは <code>vehicle_model_fitting</code> と同じ符号で、0 中心・低分散なら式と実測がよく合っている。
縦・操舵ではこの残差を整合診断として示し、ヨーでは線形最小二乗の同定量として直接使う。
</p>

<h3>連続時間の状態方程式（ベクトル形式）</h3>
<p class="meta">
<span style="color:#1565c0">青字</span>が状態ベクトル \\(\\mathbf{{x}}\\)（6状態）、<span style="color:#e65100">橙字</span>が入力ベクトル \\(\\mathbf{{u}}\\)。
</p>
\\[
{{\\color{{#1565c0}} \\mathbf{{x}}}} =
\\begin{{pmatrix}}
{{\\color{{#1565c0}} x}} \\\\
{{\\color{{#1565c0}} y}} \\\\
{{\\color{{#1565c0}} \\theta}} \\\\
{{\\color{{#1565c0}} v_x}} \\\\
{{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}}
\\end{{pmatrix}}
, \\qquad
{{\\color{{#e65100}} \\mathbf{{u}}}} =
\\begin{{pmatrix}}
{{\\color{{#e65100}} a_{{\\mathrm{{cmd,des}}}}}} \\\\
{{\\color{{#e65100}} \\delta_{{\\mathrm{{cmd,des}}}}}} \\\\
{{\\color{{#e65100}} a_{{\\mathrm{{slope}}}}}}
\\end{{pmatrix}}
\\]
<p class="meta">
\\(a_{{\\mathrm{{cmd,des}}}}, \\delta_{{\\mathrm{{cmd,des}}}}\\) は純粋遅延 \\(T_a, T_\\delta\\) を経て入力される。
下の状態方程式はアクチュエータ2チャネルの右辺全体を \\(t-T\\) で評価する
<b>full-RHS 遅延</b>（<code>DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER</code>）で統一し、
指令だけでなく状態フィードバック \\(a_{{\\mathrm{{act}}}}, \\delta_{{\\mathrm{{act}}}}\\) も同じ \\((t-T_a), (t-T_\\delta)\\) で参照する
（Euler 形の \\(n_a = \\mathrm{{round}}(T_a/\\Delta t), n_\\delta = \\mathrm{{round}}(T_\\delta/\\Delta t)\\) は遅延サンプル数。
旧「指令のみ遅延」との関係は<a href="#sec-delay-mode">本節末</a>を参照）。
<code>gear</code> はこの入力ベクトルには含めず、<code>/vehicle/status/gear_status</code> の直前値から
DRIVE 系（enum 値 2..19）の時刻だけを残す評価マスクとして適用する。非 DRIVE 系
（NEUTRAL/REVERSE/PARK/LOW 等）は同定・評価窓から除外される。
</p>
\\[
\\dot{{\\mathbf{{x}}}} =
\\begin{{pmatrix}}
{{\\color{{#1565c0}} \\dot x}} \\\\
{{\\color{{#1565c0}} \\dot y}} \\\\
{{\\color{{#1565c0}} \\dot\\theta}} \\\\
{{\\color{{#1565c0}} \\dot v_x}} \\\\
{{\\color{{#1565c0}} \\dot\\delta_{{\\mathrm{{act}}}}}} \\\\
{{\\color{{#1565c0}} \\dot a_{{\\mathrm{{act}}}}}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} v_x}} \\cos{{\\color{{#1565c0}} \\theta}} \\\\
{{\\color{{#1565c0}} v_x}} \\sin{{\\color{{#1565c0}} \\theta}} \\\\
\\dfrac{{{{\\color{{#1565c0}} v_x}}\\,\\tan({{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}+\\beta)}}{{L + k_{{\\mathrm{{us}}}}\\,{{\\color{{#1565c0}} v_x}}^2}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}} + {{\\color{{#e65100}} a_{{\\mathrm{{slope}}}}}} \\\\
\\dfrac{{{{\\color{{#e65100}} \\delta_{{\\mathrm{{cmd,des}}}}}}(t-T_\\delta) - {{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}(t-T_\\delta)}}{{\\tau_\\delta}} \\\\
\\dfrac{{{{\\color{{#e65100}} a_{{\\mathrm{{cmd,des}}}}}}(t-T_a) - {{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}}(t-T_a)}}{{\\tau_a}}
\\end{{pmatrix}}
\\]
<p class="meta">
&#128279; \\(\\tau_\\delta, \\beta\\) は <a href="#sec-steer">1-3</a>、\\(\\tau_a\\) は <a href="#sec-long">1-2</a>、
\\(L, k_{{\\mathrm{{us}}}}\\) は <a href="#sec-yaw">1-4</a> で定義。
</p>

<h3>離散化：Euler 法によるアップデート</h3>
<p>
制御周期 \\(\\Delta t\\) ごとの離散時刻を \\(k = 0, 1, 2, \\dots\\) とすると、
\\[
{{\\color{{#1565c0}} \\mathbf{{x}}_{{k+1}}}} = {{\\color{{#1565c0}} \\mathbf{{x}}_k}} + f({{\\color{{#1565c0}} \\mathbf{{x}}_k}}, {{\\color{{#e65100}} \\mathbf{{u}}_k}}) \\cdot \\Delta t
\\]
で状態を更新する（前進・陽的 Euler 法、実装は
<code>SimModelInterface::updateEuler(dt, input)</code>）。\\(f\\) を展開すると、各状態は次のように更新される
（full-RHS 遅延のため、アクチュエータ2チャネルは右辺で状態を \\(k-n_a, k-n_\\delta\\) サンプル前に参照する）。
</p>
\\[
\\begin{{pmatrix}}
{{\\color{{#1565c0}} x_{{k+1}}}} \\\\
{{\\color{{#1565c0}} y_{{k+1}}}} \\\\
{{\\color{{#1565c0}} \\theta_{{k+1}}}} \\\\
{{\\color{{#1565c0}} v_{{x,k+1}}}} \\\\
{{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k+1}}}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}},k+1}}}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} x_k}} \\\\
{{\\color{{#1565c0}} y_k}} \\\\
{{\\color{{#1565c0}} \\theta_k}} \\\\
{{\\color{{#1565c0}} v_{{x,k}}}} \\\\
{{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k}}}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}},k}}}}
\\end{{pmatrix}}
+
\\begin{{pmatrix}}
{{\\color{{#1565c0}} v_{{x,k}}}} \\cos{{\\color{{#1565c0}} \\theta_k}} \\\\
{{\\color{{#1565c0}} v_{{x,k}}}} \\sin{{\\color{{#1565c0}} \\theta_k}} \\\\
\\dfrac{{{{\\color{{#1565c0}} v_{{x,k}}}}\\,\\tan({{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k}}}}+\\beta)}}{{L + k_{{\\mathrm{{us}}}}\\,{{\\color{{#1565c0}} v_{{x,k}}}}^2}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}},k}}}} + {{\\color{{#e65100}} a_{{\\mathrm{{slope}},k}}}} \\\\
\\dfrac{{{{\\color{{#e65100}} \\delta_{{\\mathrm{{cmd,des}},\\,k-n_\\delta}}}} - {{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},\\,k-n_\\delta}}}}}}{{\\tau_\\delta}} \\\\
\\dfrac{{{{\\color{{#e65100}} a_{{\\mathrm{{cmd,des}},\\,k-n_a}}}} - {{\\color{{#1565c0}} a_{{\\mathrm{{act}},\\,k-n_a}}}}}}{{\\tau_a}}
\\end{{pmatrix}}
\\cdot \\Delta t
\\]

<h3 id="sec-delay-mode">full-RHS 遅延と旧モデルとの関係</h3>
<p>
上の状態方程式は、アクチュエータ2チャネル（\\(\\dot a_{{\\mathrm{{act}}}}, \\dot\\delta_{{\\mathrm{{act}}}}\\)）の
<b>右辺全体</b>を \\(t-T\\) で評価する <b>full-RHS 遅延</b>
（<code>DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER</code>）で統一している。指令だけでなく状態
\\(a_{{\\mathrm{{act}}}}, \\delta_{{\\mathrm{{act}}}}\\) も \\(t-T\\) で参照する（C++ の <code>pedal_delayed</code> /
<code>steer_delayed</code> に対応）。\\(a_{{\\mathrm{{slope}}}}\\) は速度式 \\(\\dot v_x\\) 側の外部入力なので
遅延させず、ヨー・位置・速度式は現在状態で評価する。
</p>
<p>
旧 <code>DELAY_STEER_ACC_GEARED_WO_FALL_GUARD</code> は状態フィードバックを<b>現在時刻</b>で評価する
「指令のみ遅延」で、上式で状態の遅延を 0 とした（\\(a_{{\\mathrm{{act}}}}(t-T_a)\\to a_{{\\mathrm{{act}}}}(t)\\)、
\\(\\delta_{{\\mathrm{{act}}}}(t-T_\\delta)\\to\\delta_{{\\mathrm{{act}}}}(t)\\)）特別な場合であり、full-RHS 遅延はその厳密な一般化である。
</p>
</section>

<section id="sec-long">
<h2>1-2. 縦方向（加速度アクチュエータ）の同定</h2>
<p class="meta">
&#128279; <a href="#sec-state-space">1-1 状態方程式</a> の \\(\\dot v_x\\) 式・
\\(\\dot a_{{\\mathrm{{act}}}}\\) 式に対応。
</p>
<p>
実測加速度は勾配成分を引いたものを使う。
\\[
a_{{\\mathrm{{real}}}} := a_{{\\mathrm{{report}}}} - a_{{\\mathrm{{slope}}}}
\\approx a_{{\\mathrm{{act}}}}
\\]
</p>

<p><b>モデル式:</b></p>
\\[
\\begin{{pmatrix}}
{{\\color{{#1565c0}} \\dot v_x}} \\\\
{{\\color{{#1565c0}} \\dot a_{{\\mathrm{{act}}}}}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}} + {{\\color{{#e65100}} a_{{\\mathrm{{slope}}}}}} \\\\
\\dfrac{{{{\\color{{#e65100}} a_{{\\mathrm{{cmd,des}}}}}}(t-T_a) - {{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}}(t-T_a)}}{{\\tau_a}}
\\end{{pmatrix}}
\\]
<p class="meta">
<a href="#sec-state-space">1-1</a> と同じ <b>full-RHS 遅延</b>：\\(\\dot a_{{\\mathrm{{act}}}}\\) は指令・状態とも
\\(t-T_a\\) で評価する。\\(a_{{\\mathrm{{slope}}}}\\) は \\(\\dot v_x\\) 側の入力で非遅延。
</p>
<h3>方程式残差で見る</h3>
<p>
full-RHS 遅延を適用した残差診断。指令・状態をともに \\(t-T_a\\) で評価した
\\(\\dot a_{{\\mathrm{{act}}}}\\) 式の左辺と右辺は
\\[
\\mathrm{{LHS}}_a[k] = \\widehat{{\\dot a}}_{{\\mathrm{{real}}}}[k],
\\qquad
\\mathrm{{RHS}}_a[k;\\tau_a,T_a] =
\\frac{{u[k] - a_{{\\mathrm{{real}}}}[k-n_{{\\mathrm{{delay}}}}]}}{{\\tau_a}}
\\]
である。ここで \\(u[k] = a_{{\\mathrm{{cmd,des}}}}[k - n_{{\\mathrm{{delay}}}}]\\)。方程式残差は
\\[
E_a[k;\\tau_a,T_a]
= \\mathrm{{RHS}}_a[k;\\tau_a,T_a] - \\mathrm{{LHS}}_a[k],
\\qquad
J_a
= \\frac{{1}}{{|\\mathcal{{K}}|}} \\sum_{{k\\in\\mathcal{{K}}}}
\\left(E_a[k]\\right)^2
\\]
で評価する。0 中心で低分散なら、同定した \\((\\tau_a,T_a)\\) が運動方程式と整合している。
</p>
<p>
パラメータ同定は、<code>_fit_core.py</code> の <code>fit_first_order_delay_residual_3phase</code> による3段階交互最適化を用いた方程式残差の最小二乗法で直接行われます。
</p>
<div class="note">
  <b>3段階交互最適化 (3-Phase Alternating Optimization) のアルゴリズム (縦方向):</b>
  <ol>
    <li><b>Phase 1 (初期値固定時定数最適化):</b> 無駄時間 \(T_a\) を初期パラメータの基準値に固定した状態で、時定数 \(\tau_a\) を最小二乗法で最適化し初期パラメータを決定します（\(\tau_a\) の初期固定値指定がある場合は最適化をスキップしてその値を使用）。</li>
    <li><b>Phase 2 (遅延グリッドサーチ):</b> Phase 1 で得られたパラメータを固定したまま、事前に定義された遅延候補（グリッド）の中から方程式残差の二乗和 (MSE) を最小化する無駄時間 \(T_a\) を探索します。</li>
    <li><b>Phase 3 (再最適化スキップ):</b> 縦方向では、初期遅延による時定数決定の固定関係を維持するため、Phase 3 (再最適化) をスキップし、Phase 1/2 で得られた時定数 \(\tau_a\) と遅延時間 \(T_a\) を最終的な同定値とします。</li>
  </ol>
</div>
<p class="meta">
&#128279; この残差式は <code>_fit_core.py</code> の <code>equation_residual_at_params</code>（full-RHS 遅延）および
<a href="#sec-state-space">1-1</a> の状態方程式と完全に対応する（<code>vehicle_model_fitting</code> の残差式と同一形式）。
</p>
{long_resid_hist_html}
{long_resid_opt_html}

<p><b>主なパラメータ:</b></p>
<ul>
  <li>\\(T_a\\): 加速度純粋遅延（<code>acc_time_delay</code>）。</li>
  <li>\\(\\tau_a\\): 加速度一次遅れ時定数（<code>acc_time_constant</code>）。</li>
  <li>\\(a_{{\\mathrm{{real}}}}\\): 実車ログの勾配補正後加速度。</li>
  <li>\\(a_{{\\mathrm{{sim}}}}\\): モデルから計算した加速度。</li>
</ul>

<h3>実機ログとの比較</h3>
<p>
図では、同定した \\((\\tau_a,T_a)\\) でのモデル出力と実測加速度を重ねる。
表示時はモデル出力に \\(a_{{\\mathrm{{slope}}}}\\) を戻し、\\(a_{{\\mathrm{{report}}}}\\) と比較する。
連続するデータセットは ROS 絶対時刻で結合し、1 本の時系列として表示する。
</p>
{long_html}

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量（RMSE）</th></tr>
  <tr><td><code>acc_time_constant</code> (\\(\\tau_a\\))</td><td>{tau_a} s</td>
      <td>一次遅れ時定数：小さいほど加速応答が速い</td>
      <td rowspan="2">\\(a_{{\\mathrm{{sim}}}} - a_{{\\mathrm{{real}}}}\\)</td></tr>
  <tr><td><code>acc_time_delay</code> (\\(T_a\\))</td><td>{T_a} s</td>
      <td>純粋遅延：指令が実際に入力されるまでの遅延時間</td></tr>
</table>
{long_fit_rmse_html}
</section>

<section id="sec-steer">
<h2>1-3. 操舵アクチュエータ（追従ループ）の同定</h2>
<p class="meta">
&#128279; <a href="#sec-state-space">1-1 状態方程式</a> の
\\(\\dot\\delta_{{\\mathrm{{act}}}}\\) 式に対応。
</p>

<p><b>モデル式:</b></p>
\\[
{{\\color{{#1565c0}} \\dot\\delta_{{\\mathrm{{act}}}}}}(t)
= \\dfrac{{{{\\color{{#e65100}} \\delta_{{\\mathrm{{des}}}}}}(t) - {{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}(t-T_\\delta)}}{{\\tau_\\delta}},
\\qquad
{{\\color{{#e65100}} \\delta_{{\\mathrm{{des}}}}}}(t)
= K_{{\\mathrm{{steer\\_scale}}}} \\cdot {{\\color{{#e65100}} \\delta_{{\\mathrm{{cmd,des}}}}}}(t - T_\\delta)
\\]
<p class="meta">
<a href="#sec-state-space">1-1</a> と同じ <b>full-RHS 遅延</b>：指令遅延（\\(\\delta_{{\\mathrm{{des}}}}\\) 内の \\(t-T_\\delta\\)）に加え、
状態フィードバック \\(\\delta_{{\\mathrm{{act}}}}\\) も \\(t-T_\\delta\\) で評価する。
</p>
<h3>方程式残差で見る</h3>
<p>
full-RHS 遅延を適用した残差診断。指令・実舵角をともに \\(t-T_\\delta\\) で評価した
\\(\\dot\\delta_{{\\mathrm{{act}}}}\\) 式の左辺と右辺を
\\[
\\mathrm{{LHS}}_\\delta[k] = \\widehat{{\\dot\\delta}}_{{\\mathrm{{act}}}}[k],
\\qquad
\\mathrm{{RHS}}_\\delta[k;\\tau_\\delta,T_\\delta] =
\\frac{{\\delta_{{\\mathrm{{des}}}}[k] - \\delta_{{\\mathrm{{act}}}}[k-n_{{\\mathrm{{delay}}}}]}}{{\\tau_\\delta}},
\\]
\\[
E_\\delta[k;\\tau_\\delta,T_\\delta]
= \\mathrm{{RHS}}_\\delta[k;\\tau_\\delta,T_\\delta] - \\mathrm{{LHS}}_\\delta[k]
\\]
と置く。\\(\\delta_{{\\mathrm{{des}}}}[k] = K_{{\\mathrm{{steer\_scale}}}} \\cdot \\delta_{{\\mathrm{{cmd,des}}}}[k-n_{{\\mathrm{{delay}}}}]\\)。
残差が 0 付近に集まれば、同定した \\((\\tau_\\delta,T_\\delta)\\) は操舵の状態方程式と整合している。
</p>
<p>
パラメータ同定は、縦方向と同様に <code>_fit_core.py</code> の <code>fit_first_order_delay_residual_3phase</code> による3段階交互最適化を用いた方程式残差の最小二乗法で直接行われます。
</p>
<div class="note">
  <b>3段階交互最適化 (3-Phase Alternating Optimization) のアルゴリズム:</b>
  <ol>
    <li><b>Phase 1 (初期値固定時定数最適化):</b> 無駄時間 \(T_\delta\) を初期パラメータの基準値（Steer: 0.0315s）に固定した状態で、逆数変数 \(\tau_\delta^{-1} = 1/\tau_\delta\) に対して最小二乗法で最適化を行い、初期探索パラメータを決定します。</li>
    <li><b>Phase 2 (遅延グリッドサーチ):</b> Phase 1 で得られたパラメータを固定したまま、事前に定義された遅延候補（グリッド）の中から方程式残差の二乗和 (MSE) を最小化する無駄時間 \(T_\delta\) を探索します。</li>
    <li><b>Phase 3 (パラメータ再最適化):</b> Phase 2 で決定された最良の無駄時間 \(T_\delta\) に固定した上で、再び時定数 \(\tau_\delta\) などのパラメータを最小二乗法で再最適化し、最終的な同定値を得ます。</li>
  </ol>
</div>
<p class="meta">
&#128279; この残差式は <code>_fit_core.py</code> の <code>equation_residual_at_params</code>（full-RHS 遅延）から生成される。
<code>vehicle_model_fitting</code> の操舵残差式と同一形式。
</p>
{steer_resid_hist_html}
{steer_resid_opt_html}

<p><b>主なパラメータ:</b></p>
<ul>
  <li>\\(T_\\delta\\): 操舵純粋遅延（<code>steer_time_delay</code>）。</li>
  <li>\\(\\tau_\\delta\\): 操舵一次遅れ時定数（<code>steer_time_constant</code>）。</li>
  <li>\\(K_{{\\mathrm{{steer\\_scale}}}}\\): 操舵指令スケーリング倍率（<code>debug_steer_scaling_factor</code>）。</li>
  <li>\\(\\beta\\): 追従式の外側で 1-4 のヨー式に渡す操舵バイアス。</li>
</ul>

<h3>実機ログとの比較</h3>
<p>
図では、同定した \\((\\tau_\\delta,T_\\delta)\\) のモデル出力と実測 \\(\\delta_{{\\mathrm{{act}}}}\\) を重ねる。
チューニング値でのシミュレーション結果も比較として表示する。
連続するデータセットは ROS 絶対時刻で結合し、1 本の時系列として表示する。
</p>
{steer_html}
<div class="note">
⚠️ <b>注記</b>: <code>cmd_steer</code> は understeer converter 適用前（コントローラ出力）、
<code>delta_act</code> は converter 適用後の実舵角である。定常ゲインは速度依存の補正を含むため、
時定数 \\(\\tau_\\delta\\) と遅延 \\(T_\\delta\\) の分布は quasi-static で有効だが、ゲイン推定は低速帯（v &lt; 3 m/s）に限定して評価することが望ましい。
</div>

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量（RMSE）</th></tr>
  <tr><td><code>steer_time_constant</code> (\\(\\tau_\\delta\\))</td><td>{tau_d} s</td>
      <td>一次遅れ時定数：小さいほど操舵応答が速い</td><td rowspan="4">モデル出力 − 実測操舵角</td></tr>
  <tr><td><code>steer_time_delay</code> (\\(T_\\delta\\))</td><td>{T_d} s</td>
      <td>純粋遅延：操舵指令が実際に入力されるまでの遅延時間</td></tr>
  <tr><td><code>debug_steer_scaling_factor</code> (操舵ゲイン補正倍率)</td><td>{DSF}</td>
      <td>指令スケーリング（1.0 = 補正なし）；遅延後に乗算</td></tr>
  <tr><td><code>steer_bias</code> (\\(\\beta\\))</td><td>{beta} rad</td>
      <td>報告操舵角への加算値（\\(\\delta_{{\\mathrm{{act}}}} + \\beta\\)）；1-4 のヨー式と共有</td></tr>
  <tr><td><code>steer_dead_band</code></td><td>{db} rad</td>
      <td>不感帯幅（固定値・同定対象外）</td><td>—</td></tr>
  <tr><td><code>steer_rate_lim</code></td><td>{rlim} rad/s</td>
      <td>操舵レート飽和（固定値・同定対象外）</td><td>—</td></tr>
</table>
{steer_fit_rmse_html}
</section>
<section id="sec-yaw">
<h2>1-4. ヨー・横方向（運動学的自転車モデル）— スカラー 最小二乗法同定</h2>
<p class="meta">
&#128279; <a href="#sec-state-space">1-1 状態方程式</a> の \\(\\dot x\\) 式・
\\(\\dot y\\) 式・\\(\\dot\\theta\\) 式に対応。
</p>
<p>
1-2（縦方向）・1-3（操舵追従）が先行して確定した後、
\\(\\dot\\theta\\) 式の残差 \\(E = \\mathrm{{RHS}} - \\mathrm{{LHS}}\\) を <b>高曲率サブセット</b> で最小化して
\\(k_{{\\mathrm{{us}}}}\\) を同定する。縦・操舵と同じ方程式残差の原理であり、この残差を直接推定（最小二乗法による同定）に用います。
\\(\\dot\\theta\\) 式は \\(\\tan\\delta\\) 空間で \\(k_{{\\mathrm{{us}}}}\\) について<b>線形</b>であり、この空間では残差最小化が極めて良条件なため直接解くことができます。
直進（\\(\\delta_{{\\mathrm{{act}}}}+\\beta \\approx 0\\)）では感度がほぼゼロなので、全データセット集約
スコアだけでは \\(k_{{\\mathrm{{us}}}}\\) を同定できない。
</p>

<p><b>運動方程式（\\(\\dot x\\) 式・\\(\\dot y\\) 式・\\(\\dot\\theta\\) 式と同じ状態表記）:</b></p>
\\[
\\begin{{pmatrix}}
{{\\color{{#1565c0}} \\dot x}} \\\\
{{\\color{{#1565c0}} \\dot y}} \\\\
{{\\color{{#1565c0}} \\dot\\theta}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} v_x}} \\cos{{\\color{{#1565c0}} \\theta}} \\\\
{{\\color{{#1565c0}} v_x}} \\sin{{\\color{{#1565c0}} \\theta}} \\\\
\\dfrac{{{{\\color{{#1565c0}} v_x}}\\,\\tan({{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}+\\beta)}}{{L + k_{{\\mathrm{{us}}}}\\,{{\\color{{#1565c0}} v_x}}^2}}
\\end{{pmatrix}}
\\]
<p><b>式中の定数・補足:</b></p>
<ul>
  <li>\\(L\\)（<code>wheelbase</code>）: 自転車モデルのホイールベース。</li>
  <li>\\(k_{{\\mathrm{{us}}}}\\): アンダーステア係数（全速度域で一定のスカラー）。</li>
  <li>\\(\\beta\\)（<code>steer_bias</code>）: ヨー式の \\(\\tan(\\cdot)\\) 引数に入るバイアス。系統的なヨーオフセット成分。</li>
  <li>\\(\\delta_{{\\mathrm{{act}}}}\\): 1-3 の操舵追従結果。操舵ゲイン補正倍率による \\(k_{{\\mathrm{{us}}}}\\) の \\(v_x^2\\delta\\) 成分の部分吸収に注意。</li>
</ul>

<h3>実機ログからの独立同定（全 {n_dataset} データセット、スカラー 最小二乗法）</h3>

<details>
<summary>推定手法の詳細</summary>
<p>
定常旋回フィルタ（\\(|\\omega| > {WZ_MIN}\\) rad/s、\\(|\\dot{{\\omega}}| < {DWZ_MAX}\\) rad/s²、
\\(v_x > {VX_MIN_CURVE}\\) m/s）を通過した全タイムステップを一括して、以下の2種類の推定を行う。
</p>
<p><b>① 最小二乗法推定（青・水平線）</b>: \\(\\dot\\theta\\) 式より
\\[
\\omega_i = \\dot\\theta_i
= \\frac{{v_i\\tan(\\delta_i)}}{{L+k_{{\\mathrm{{us}}}}v_i^2}}
\\quad\\Longleftrightarrow\\quad
\\tan(\\delta_i) - \\frac{{L\\,\\omega_i}}{{v_i}} = k_{{\\mathrm{{us}}}}\\,(v_i\\,\\omega_i) .
\\]
各サンプルの補正済み操舵角を \\(\\delta_i := \\delta_{{\\mathrm{{act}},i}} + \\beta\\) と置き、
\\(x_i = v_i\\,\\omega_i\\)、\\(y_i = \\tan(\\delta_i) - L\\,\\omega_i / v_i\\) とした原点回帰の残差は
\\[
r_i(k_{{\\mathrm{{us}}}}) = y_i - k_{{\\mathrm{{us}}}}\\,x_i .
\\]
その二乗和を評価関数にすると
\\[
J(k_{{\\mathrm{{us}}}}) = \\sum_i \\bigl(y_i - k_{{\\mathrm{{us}}}}\\,x_i\\bigr)^2
\\]
であり、その最小二乗解は
\\[
\\hat{{k}}_{{\\mathrm{{us}}}} = \\frac{{\\sum_i x_i\\,y_i}}{{\\sum_i x_i^2}}
= \\frac{{\\sum_i (v_i\\omega_i)\\,\\bigl(\\tan(\\delta_i) - L\\omega_i/v_i\\bigr)}}{{\\sum_i (v_i\\omega_i)^2}}
\\]
ここで \\(L = {WHEELBASE}\\) m はホイールベース。十分統計量
\\(\\sum x_i^2,\\ \\sum x_i y_i\\) は加算的なので、データセット横断でも生サンプル再読込なしにプールできる。
</p>
<p><b>② 個別サンプル（IQR バンド）</b>: 実機運動学ログの各タイムステップで瞬時 \\(k_{{\\mathrm{{us}}}}\\) を推定し、
25〜75 パーセンタイルをバンドとして表示:
\\[
\\tilde{{k}}_{{\\mathrm{{us}}}}[i] = \\frac{{\\tan(\\delta_i) / \\omega_i - L / v_{{x,i}}}}{{v_{{x,i}}}}
\\]
</p>
</details>
{kus_html}
<div class="note">
<b>解釈</b>: 最小二乗法推定値がモデル設定値 \\(k_{{\\mathrm{{us}}}}\\)（破線）と近ければ整合している。
ただし直進（\\(\\delta_{{\\mathrm{{act}}}}+\\beta\\approx 0\\)）では感度がほぼゼロで、J6 の多くのデータセットが
低速（\\(v_x\\) mean ≈ 1.9 m/s）のため、推定誤差が大きい点に注意（凡例のサンプル数 n を参照）。
</div>

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量</th></tr>
{kus_rows}
  <tr><td><code>steer_bias</code> (\\(\\beta\\)) <i>[1-3 と共有]</i></td><td>{beta} rad</td>
      <td>\\(\\tan(\\delta_{{\\mathrm{{act}}}} + \\beta)\\) の引数：ヨーオフセット成分</td>
      <td>\\(\\widehat{{\\omega}}-\\omega\\)（間接）</td></tr>
</table>
</section>
"""


def _build_sec14(
    fig_box: go.Figure,
    fig_traj: go.Figure,
    params: dict,
    n_dataset: int,
) -> str:
    """1-5. モデル構造限界（理想追従評価）セクション HTML。"""
    box_html  = fig_box.to_html(full_html=False, include_plotlyjs=False)
    traj_html = fig_traj.to_html(full_html=False, include_plotlyjs=False)
    tau_a = f"{params.get('acc_time_constant', float('nan')):.3g}"
    T_a   = f"{params.get('acc_time_delay', float('nan')):.3g}"
    tau_d = f"{params.get('steer_time_constant', float('nan')):.3g}"
    T_d   = f"{params.get('steer_time_delay', float('nan')):.3g}"
    h_str = ", ".join(f"N={h}（{h * _FIT_DT:.2f}s）" for h in _PERF_HORIZONS)
    return f"""
<section id="sec-perf-tracking">
<h2>1-5. モデル構造限界（理想追従評価）</h2>
<p>
アクチュエータ追従が完璧だった場合（実測 \\(v_x\\) と実測 \\(\\delta_{{\\mathrm{{act}}}}\\) を
自転車モデルの直接入力として使用）に残る位置ずれを評価する。
これにより <b>アクチュエータ遅れの寄与</b> と <b>モデル構造外の寄与</b>（タイヤスリップ、路面バンク、
横速度 \\(v_y\\)、\\(k_{{\\mathrm{{us}}}}\\) キャリブレーション誤差）を分離できる。
</p>
<p>
現行スコアとの差分 ≈ アクチュエータ応答が占める誤差分
（\\(\\tau_a\\)={tau_a} s, \\(T_a\\)={T_a} s, \\(\\tau_\\delta\\)={tau_d} s, \\(T_\\delta\\)={T_d} s の合算効果）。
</p>
<div class="note">
⚠️ <b>設計上の帰結</b>:
縦方向誤差は \\(v_x\\) を実車ログから直接取得しているため積分上ほぼゼロになる。
<b>横方向誤差のみが真のモデル構造限界を表す。</b><br>
残差は「現行チューン値 \\(k_{{\\mathrm{{us}}}}\\) および \\(\\beta\\) での理想追従誤差」であるため、
パラメータのキャリブレーション誤差も一部含む（現行パラメータ前提での下限値）。
</div>

<h3>横方向誤差分布（ホライズン別、上位 {n_dataset} データセット）</h3>
<p>
カーブ走行区間（\\(v_x > {VX_MIN_CURVE}\\) m/s）を stride={_PERF_STRIDE} ステップで走査し、
N-step Open Loop評価（ロールアウト）終端の横方向誤差絶対値を集計する。ホライズン: {h_str}。
</p>
{box_html}

<h3>代表データセット の軌跡比較（実車 vs 自転車モデル）</h3>
<p>
初期状態を実車ログに合わせ、実測 \\(v_x\\) と \\(\\delta_{{\\mathrm{{act}}}}\\) を入力として積分した
自転車モデル軌跡（青破線）を実車の軌跡（黒実線）と比較する。
リセットなしの連続積分であるため、後半の乖離はモデル構造誤差の累積を示す。
座標は初期位置を原点 (0, 0) に正規化している。
</p>
{traj_html}
</section>
"""


def _build_sec2(kus_fig: go.Figure, n_dataset: int) -> str:
    kus_html = kus_fig.to_html(full_html=False, include_plotlyjs=False)
    return f"""
<section id="identification">
<h2>2. 実機ログからの独立同定</h2>

<details open>
<summary>2-1. アンダーステア係数 \\(k_{{\\mathrm{{us}}}}\\) の独立同定（全 {n_dataset} データセット）</summary>
<details>
<summary>推定手法の詳細</summary>
<p>
定常旋回フィルタ（\\(|\\omega| > {WZ_MIN}\\) rad/s、\\(|\\dot{{\\omega}}| < {DWZ_MAX}\\) rad/s²、
\\(v_x > {VX_MIN_CURVE}\\) m/s）を通過した全タイムステップを一括して、以下の2種類の推定を行う。
</p>
<p><b>① 最小二乗法推定（青・水平線）</b>: \\(\\dot\\theta\\) 式より
\\[
\\omega_i = \\dot\\theta_i
= \\frac{{v_i\\tan(\\delta_i)}}{{L+k_{{\\mathrm{{us}}}}v_i^2}}
\\quad\\Longleftrightarrow\\quad
\\tan(\\delta_i) - \\frac{{L\\,\\omega_i}}{{v_i}} = k_{{\\mathrm{{us}}}}\\,(v_i\\,\\omega_i) .
\\]
各サンプルの補正済み操舵角を \\(\\delta_i := \\delta_{{\\mathrm{{act}},i}} + \\beta\\) と置き、
\\(x_i = v_i\\,\\omega_i\\)、\\(y_i = \\tan(\\delta_i) - L\\,\\omega_i / v_i\\) とした原点回帰の
最小二乗解は
\\[
\\hat{{k}}_{{\\mathrm{{us}}}} = \\frac{{\\sum_i x_i\\,y_i}}{{\\sum_i x_i^2}}
\\]
ここで \\(L = {WHEELBASE}\\) m はホイールベース。十分統計量
\\(\\sum x_i^2,\\ \\sum x_i y_i\\) は加算的なので、データセット横断でも生サンプル再読込なしにプールできる。
</p>
<p><b>② 個別サンプル（IQR バンド）</b>: 実機運動学ログの各タイムステップで瞬時 \\(k_{{\\mathrm{{us}}}}\\) を推定し、
25〜75 パーセンタイルをバンドとして表示:
\\[
\\tilde{{k}}_{{\\mathrm{{us}}}}[i] = \\frac{{\\tan(\\delta_i) / \\omega_i - L / v_{{x,i}}}}{{v_{{x,i}}}}
\\]
チューニング済み \\(k_{{\\mathrm{{us}}}}\\)（橙色破線）と重ね描きして妥当性を確認する。
</p>
</details>
{kus_html}
<div class="note">
<b>解釈</b>: 最小二乗法推定値がモデル設定値 \\(k_{{\\mathrm{{us}}}}\\)（破線）と近ければ整合している。
ただし直進（\\(\\delta_{{\\mathrm{{act}}}}+\\beta\\approx 0\\)）では感度がほぼゼロで、J6 の多くのデータセットが
低速（\\(v_x\\) mean ≈ 1.9 m/s）のため、推定誤差が大きい点に注意（凡例のサンプル数 n を参照）。
</div>
</details>
</section>
"""


def _build_sec3(viewer_sections: list[str], label: str = "phase14") -> str:
    body = "\n".join(viewer_sections) if viewer_sections else "<p>ビューア生成対象 データセット なし</p>"
    return f"""
<section id="curve-viewer">
<h2>3. カーブ部での実機 vs モデル軌跡（インタラクティブビューア）</h2>
<p>
旋回イベント数 <code>curve_count</code>（\\(|\\kappa| > 0.02\\) m⁻¹、連続弧長 ≥ 10 m）が多い
代表データセットについて縦横モデル検証ビューアを埋め込む。
ドロップダウンで <b>{label}</b>（\\(k_{{\\mathrm{{us}}}}\\)＋<code>steer_dead_band</code> 有効）と
<b>baseline</b>（\\(k_{{\\mathrm{{us}}}}=0\\), <code>steer_dead_band</code>=0）を切り替えて実機軌跡への一致を比較できる。
</p>
{body}
</section>
"""


def _build_sec_deviation(
    df: pd.DataFrame,
    n_dataset: int,
    recomputed_score: float | None = None,
    expected_score: float | None = None,
    score_name: str = "robust_score",
    label: str = "phase14",
) -> str:
    """N-step Open Loop評価: 終端誤差テーブルセクション（tuned vs baseline）。"""
    horizons = sorted(df["h"].unique().tolist())

    stats_list = []
    for h in horizons:
        sub = df[df["h"] == h]
        stats_list.append({
            "h": h,
            "p14_yaw_mean": sub["p14_yaw"].mean(), "p14_yaw_p99": sub["p14_yaw"].quantile(0.99),
            "p14_lat_mean": sub["p14_lat"].mean(), "p14_lat_p99": sub["p14_lat"].quantile(0.99),
            "p14_long_mean": sub["p14_long"].mean(), "p14_long_p99": sub["p14_long"].quantile(0.99),
            "p14_vx_mean": sub["p14_vx"].mean(), "p14_vx_p99": sub["p14_vx"].quantile(0.99),
            "bl_yaw_mean": sub["bl_yaw"].mean(), "bl_yaw_p99": sub["bl_yaw"].quantile(0.99),
            "bl_lat_mean": sub["bl_lat"].mean(), "bl_lat_p99": sub["bl_lat"].quantile(0.99),
            "bl_long_mean": sub["bl_long"].mean(), "bl_long_p99": sub["bl_long"].quantile(0.99),
            "bl_vx_mean": sub["bl_vx"].mean(), "bl_vx_p99": sub["bl_vx"].quantile(0.99),
        })

    score_html = ""
    if recomputed_score is not None and expected_score is not None:
        diff_pct = abs(recomputed_score - expected_score) / expected_score * 100 if expected_score else 0.0
        ok = diff_pct < 2.0
        color = "#28a745" if ok else "#dc3545"
        score_html = (
            f'<div class="note" style="border-color:{color}">'
            f"スコア再現検証 (<code>{score_name}</code>): 再計算 = <b>{recomputed_score:.4f}</b>、"
            f"YAML 期待値 = <b>{expected_score:.4f}</b>（差 {diff_pct:.2f}%）"
            + (" — ✓ 整合" if ok else " — ⚠ 不整合（override/model/SUB_DT を確認）")
            + "</div>"
        )

    def _cell(p14_val: float, bl_val: float, fmt: str = ".3f") -> str:
        ratio = p14_val / bl_val if bl_val > 0 else 1.0
        if ratio < 0.99:
            style = ' style="color:#28a745;font-weight:bold"'
        elif ratio > 1.01:
            style = ' style="color:#dc3545"'
        else:
            style = ""
        return f"<td{style}>{p14_val:{fmt}}</td>"

    tbody_rows = []
    for s in stats_list:
        h = s["h"]
        span = _H_SPAN.get(h, "")
        tbody_rows.append(
            f'<tr>\n'
            f'  <td rowspan="2" style="text-align:center"><b>N={h}</b><br>'
            f'<small style="color:#888">{span}</small></td>\n'
            f'  <td><b>{label}</b></td>\n'
            f'  {_cell(s["p14_yaw_mean"], s["bl_yaw_mean"])}'
            f'{_cell(s["p14_yaw_p99"], s["bl_yaw_p99"])}\n'
            f'  {_cell(s["p14_lat_mean"], s["bl_lat_mean"])}'
            f'{_cell(s["p14_lat_p99"], s["bl_lat_p99"])}\n'
            f'  {_cell(s["p14_long_mean"], s["bl_long_mean"])}'
            f'{_cell(s["p14_long_p99"], s["bl_long_p99"])}\n'
            f'  {_cell(s["p14_vx_mean"], s["bl_vx_mean"], ".4f")}'
            f'{_cell(s["p14_vx_p99"], s["bl_vx_p99"], ".4f")}\n'
            f'</tr>\n'
            f'<tr>\n'
            f'  <td style="color:#888">baseline</td>\n'
            f'  <td>{s["bl_yaw_mean"]:.3f}</td><td>{s["bl_yaw_p99"]:.3f}</td>\n'
            f'  <td>{s["bl_lat_mean"]:.3f}</td><td>{s["bl_lat_p99"]:.3f}</td>\n'
            f'  <td>{s["bl_long_mean"]:.3f}</td><td>{s["bl_long_p99"]:.3f}</td>\n'
            f'  <td>{s["bl_vx_mean"]:.4f}</td><td>{s["bl_vx_p99"]:.4f}</td>\n'
            f'</tr>'
        )

    tbody = "\n".join(tbody_rows)

    hist_fig = build_fig_nstep_error_hist(df, label=label)
    hist_html = hist_fig.to_html(full_html=False, include_plotlyjs=False)

    growth_fig = build_fig_nstep_error_growth(df, label=label)
    growth_html = growth_fig.to_html(full_html=False, include_plotlyjs=False)

    return f"""
<section id="deviation">
<h2>N-step Open Loop評価: 終端誤差（{label} vs baseline）</h2>
<p>
全データセットに対し {label} パラメータと baseline（補正なし）で N-step Open Loop評価（ロールアウト）を実施し、
終端誤差 RMSE の データセット横断 <b>平均</b>（mean）と <b>99パーセンタイル</b>（99%ile）を N ごとに集計する。
下の<b>ドリフト成長カーブ</b>は横軸 = horizon N（ステップ数）で誤差の中央値 + IQR 帯の成長を {label} と baseline で比較する。
分布の形（裾の太さ・外れ値の割合）を確認したい場合は「詳細評価結果」のヒストグラムを参照。
</p>
{score_html}
<table class="param-table" style="font-size:12px">
  <thead>
    <tr>
      <th rowspan="2">N（時間）</th>
      <th rowspan="2">モデル</th>
      <th colspan="2">yaw 誤差 [deg]</th>
      <th colspan="2">lat 誤差 [cm]</th>
      <th colspan="2">long 誤差 [cm]</th>
      <th colspan="2">速度誤差 \\(v_x\\) [m/s]</th>
    </tr>
    <tr>
      <th>平均</th><th>99%ile</th>
      <th>平均</th><th>99%ile</th>
      <th>平均</th><th>99%ile</th>
      <th>平均</th><th>99%ile</th>
    </tr>
  </thead>
  <tbody>
{tbody}
  </tbody>
</table>
<div class="note">
{label} の値が baseline より小さい場合は <b style="color:#28a745">緑（改善）</b>、
大きい場合は <span style="color:#dc3545">赤（悪化）</span> で表示。
RMSE は各データセットの全 k0 ステップ（stride=5）の終端誤差（N ステップ先）の二乗平均平方根。
キャッシュは <code>--metrics-cache</code> で指定した CSV ファイルに保存される。
</div>
<h3>ドリフト成長カーブ（horizon 別 終端誤差の中央値 + IQR）</h3>
<p>
横軸を horizon N（ステップ数）にとり、各 N での データセット横断 終端誤差 RMSE の中央値を実線、
25–75%ile を帯で示す。誤差が N とともにどう成長するか、{label} と baseline の傾きの差を一目で比較できる。
</p>
{growth_html}
<details>
<summary>詳細評価結果: N-step Open Loop評価 誤差分布（ヒストグラム）</summary>
<p>
{label} モデルのデータセット横断 RMSE の分布を N・誤差成分ごとにヒストグラムで示す。
縦線は 中央値／90%ile／95%ile／99%ile。表の「平均」「99%ile」だけでは見えない
裾の太さ（大きく外れるデータセットがどの程度の割合あるか）を確認できる。
</p>
{hist_html}
</details>
</section>
"""


def _build_sec_closed_loop_comparison(collection_dir: Path, uuids_str: str) -> str:
    if not uuids_str:
        return ""
    
    uuids = [u.strip() for u in uuids_str.split(",") if u.strip()]
    sections = []
    
    for uuid in uuids:
        target_dirs = [
            collection_dir / uuid,
            collection_dir / "datasets" / uuid,
        ]
        
        found_dir = None
        for d in target_dirs:
            if d.exists():
                found_dir = d
                break
        
        if not found_dir:
            print(f"  [WARN] クローズドループ結果ディレクトリが見つかりません (UUID: {uuid})")
            continue
            
        metrics_json = found_dir / "metrics_closed_loop.json"
        playback_html = found_dir / "figures" / "viewer.html"
        
        if not playback_html.exists():
            playback_html_list = list(found_dir.glob("**/viewer.html"))
            if playback_html_list:
                playback_html = playback_html_list[0]
            
        metrics_tbl = ""
        if metrics_json.exists():
            try:
                import json
                m = json.loads(metrics_json.read_text(encoding="utf-8"))
                rows = []
                for run_name, run_data in m.get("runs", {}).items():
                    rows.append(f"""
                      <tr>
                        <td><b>{run_name}</b></td>
                        <td>{run_data.get('steer_rmse_deg', 'N/A')}</td>
                        <td>{run_data.get('vel_rmse_mps', 'N/A')}</td>
                      </tr>
                    """)
                if rows:
                    metrics_tbl = f"""
                    <table class="param-table" style="font-size:12px; margin-bottom:10px;">
                      <thead>
                        <tr>
                          <th>シミュレーションモデル</th>
                          <th>操舵 RMSE [deg]</th>
                          <th>速度 RMSE [m/s]</th>
                        </tr>
                      </thead>
                      <tbody>
                        {"".join(rows)}
                      </tbody>
                    </table>
                    """
            except Exception as e:
                print(f"  [WARN] metrics_closed_loop.json 読み込み失敗: {e}")
                
        iframe_html = ""
        if playback_html and playback_html.exists():
            try:
                content = playback_html.read_text(encoding="utf-8")
                srcdoc = _html_stdlib.escape(content, quote=True)
                iframe_html = f"""
                <iframe srcdoc="{srcdoc}"
                  width="100%" height="1000"
                  style="border:1px solid #ccc;border-radius:4px"
                  loading="lazy"></iframe>
                """
            except Exception as e:
                print(f"  [WARN] viewer.html 読み込み失敗: {e}")
                
        if iframe_html or metrics_tbl:
            sections.append(f"""
            <h3>Dataset: <code>{uuid}</code></h3>
            {metrics_tbl}
            {iframe_html}
            """)
            
    if not sections:
        return ""
        
    body = "\n".join(sections)
    return f"""
<section id="sec-closed-loop">
<h2>4. クローズドループシミュレーション比較</h2>
<p>
指定されたデータセットについて、実機走行ログ vs クローズドループシミュレーションによる走行結果の比較（軌跡再生ビューア）を提示します。
</p>
{body}
</section>
"""


def build_html(
    params: dict,
    long_fig: go.Figure,
    steer_fig: go.Figure,
    kus_fig: go.Figure,
    viewer_sections: list[str],
    n_dataset: int,
    baseline_score: float | None = None,
    deviation_html: str = "",
    label: str = "current",
    params_filename: str = "",
    perf_html: str = "",
    closed_loop_html: str = "",
    long_resid_hist_fig: go.Figure | None = None,
    steer_resid_hist_fig: go.Figure | None = None,
    long_resid_opt_html: str = "",
    steer_resid_opt_html: str = "",
    long_fit_rmse_html: str = "",
    steer_fit_rmse_html: str = "",
) -> str:
    score = params.get("_score", "N/A")
    phase14_score = float(score) if isinstance(score, (int, float, str)) and str(score) != "N/A" else 0.0
    sec_metrics = _build_sec_metrics(baseline_score=baseline_score, phase14_score=phase14_score, label=label)
    
    sec_tuning = f"""
<section id="sec-tuning">
<h2>2. 統合最適化（パラメータ最適化）</h2>
<p>
各モデルの独立最適化パラメータをベースにした、全データセット横断での統合最適化の結果を評価します。
</p>
{sec_metrics}
{deviation_html}
</section>
"""

    sec1 = _build_sec1(
        params, long_fig, steer_fig, kus_fig, n_dataset,
        long_resid_hist_fig=long_resid_hist_fig, steer_resid_hist_fig=steer_resid_hist_fig,
        long_resid_opt_html=long_resid_opt_html, steer_resid_opt_html=steer_resid_opt_html,
        long_fit_rmse_html=long_fit_rmse_html, steer_fit_rmse_html=steer_fit_rmse_html,
    )
    sec3 = _build_sec3(viewer_sections, label=label)

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>物理的妥当性レポート — {label}</title>
  {_MATHJAX_HEAD}
  {_PLOTLY_CDN}
  <style>{_STYLE}</style>
</head>
<body>
<h1>車両モデル物理的妥当性検証レポート — {label}</h1>
<p class="meta">
  生成元: <code>{params_filename or label}</code> &nbsp;|&nbsp;
  score: {score} &nbsp;|&nbsp;
  有効データセット数: {n_dataset}
</p>
<nav>
  <a href="#sec-coords">1-0. 座標系定義</a>
  <a href="#sec-state-space">1-1. 状態空間モデルと数値積分</a>
  <a href="#sec-long">1-2. 縦方向</a>
  <a href="#sec-steer">1-3. 操舵</a>
  <a href="#sec-yaw">1-4. ヨー・横方向</a>
  <a href="#sec-tuning">2. 統合最適化</a>
  {f'<a href="#sec-perf-tracking">1-5. モデル構造限界</a>' if perf_html else ""}
  <a href="#curve-viewer">3. カーブビューア</a>
  {f'<a href="#sec-closed-loop">4. クローズドループ比較</a>' if closed_loop_html else ""}
</nav>
{sec1}
{sec_tuning}
{perf_html}
{sec3}
{closed_loop_html}
{_RENDER_GLUE}
</body>
</html>
"""


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
) -> str:
    """N-step Open Loop評価とフィッティング評価だけを抜粋したリリースノート用の自己完結 HTML を返す。

    main() で既に計算済みの値をそのまま受け取り、MCAP 再読込・再フィットは行わない。
    """
    deviation_section = deviation_html or (
        '<section id="deviation"><h2>N-step Open Loop評価</h2>'
        '<div class="note">--metrics-cache 未指定のため、このレポートでは省略されました。</div>'
        "</section>"
    )

    long_ts_html = _lazy_fig_html(
        long_fig, "rn-fig-long-timeseries", "release-note/long_fit（dataset横断）",
    )
    steer_ts_html = _lazy_fig_html(
        steer_fig, "rn-fig-steer-timeseries", "release-note/steer_fit（dataset横断）",
    )
    long_resid_hist_html = long_resid_hist_fig.to_html(full_html=False, include_plotlyjs=False)
    steer_resid_hist_html = steer_resid_hist_fig.to_html(full_html=False, include_plotlyjs=False)

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>物理的妥当性レポート（要約）— {label}</title>
  {_MATHJAX_HEAD}
  {_PLOTLY_CDN}
  <style>{_STYLE}</style>
</head>
<body>
<h1>{current_model}モデルリリースレポート— {label}</h1>
<p class="meta">
  生成元: <code>{params_filename or label}</code> &nbsp;|&nbsp;
  有効データセット数: {n_dataset}
</p>
<nav>
  <a href="#deviation">N-step Open Loop評価</a>
  <a href="#rn-fit">フィッティング評価</a>
</nav>

{deviation_section}

<section id="rn-fit">
<h2>縦横モデルのフィッティング評価</h2>

<h3>縦方向</h3>
{long_fit_rmse_html}
<details><summary>時系列グラフを表示（クリックで展開）</summary>{long_ts_html}</details>
{long_resid_hist_html}

<h3>操舵</h3>
{steer_fit_rmse_html}
<details><summary>時系列グラフを表示（クリックで展開）</summary>{steer_ts_html}</details>
{steer_resid_hist_html}
</section>

{_RENDER_GLUE}
</body>
</html>
"""


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def _find_first_curve_t(ctx, pre_roll_s: float = 5.0) -> float:
    """DatasetCtx の運動学データから最初のカーブ開始時刻を検出し、pre_roll_s 秒前を返す。"""
    kin = ctx.data["kin"]
    if kin.empty:
        return 0.0
    t_ns = kin["t_ns"].values
    t_rel = (t_ns - ctx.t0_ns) * 1e-9
    vx = kin["vx"].values
    wz = kin["wz"].values
    with np.errstate(divide="ignore", invalid="ignore"):
        kappa = np.where(vx > 0.5, np.abs(wz / vx), 0.0)
    in_curve = kappa > 0.02
    # 5 フレーム以上連続でカーブ条件を満たす最初の点
    for i in range(len(t_rel) - 5):
        if in_curve[i : i + 5].all():
            return float(max(0.0, t_rel[i] - pre_roll_s))
    return 0.0


def main() -> None:
    ap = argparse.ArgumentParser(description="物理的妥当性レポート生成")
    ap.add_argument(
        "--params", type=Path, required=True,
        help="チューニング済みパラメータ YAML (tuned_params.yaml)",
    )
    ap.add_argument(
        "--collection-dir", type=Path, required=True,
        help="real.lite 群を収集した collection ディレクトリ",
    )
    ap.add_argument(
        "--out", type=Path, required=True,
        help="出力 HTML パス",
    )
    ap.add_argument(
        "--release-note-out", type=Path, default=None,
        help="N-step Open Loop評価とフィッティング評価のみを抜粋したリリースノート用 HTML の出力パス（省略時は生成しない）",
    )
    ap.add_argument(
        "--scenario", type=Path, default=None,
        help="current/baseline のモデル定義を含む scenario.yaml",
    )
    ap.add_argument(
        "--case", default="current",
        help="current として評価する Conditions.models の名前（既定: current）",
    )
    ap.add_argument("--n-curve-ds", type=int, default=3, help="ビューア埋め込みカーブ データセット数")
    ap.add_argument("--n-jobs", type=int, default=8)
    ap.add_argument(
        "--viewer-uuids", type=str, default=None,
        help="ビューアに使う データセット UUID をカンマ区切りで指定（省略時は curve_count 上位を自動選択）",
    )
    ap.add_argument(
        "--pinned-uuids", type=str, default="",
        help="ビューアに必ず含める データセット UUID をカンマ区切りで指定（前方一致）。--viewer-uuids や自動選択より優先して先頭に配置",
    )
    ap.add_argument(
        "--long-steer-pinned-uuids", type=str,
        default="049b35fe-6310-59b0-a778-d47ab6163beb",
        help="1-2/1-3の時系列グラフ（縦方向/操舵）に必ず含めるデータセットUUIDをカンマ区切りで"
             "指定（前方一致）。自動選択される最長連続系列に追加して表示する",
    )
    ap.add_argument(
        "--metrics-cache", type=Path, default=None,
        help=(
            "rollout メトリクス CSV キャッシュパス（指定時のみ偏差テーブルを生成）。"
            "ファイルが存在すれば読み込み、なければ全データセット rollout を実行して保存する。"
        ),
    )
    ap.add_argument(
        "--extra-ds", type=Path, nargs="*", default=[],
        help="collection-dir 外から MCAP 解析・ビューアに追加する データセット ディレクトリ（複数指定可）",
    )
    ap.add_argument(
        "--label", type=str, default=None,
        help="レポート内のモデル名ラベル（デフォルト: scenario.yamlでのモデル種別名、例: delay_steer_acc_geared_for_diffusion_planner）",
    )
    ap.add_argument(
        "--ds-after", type=str, default=None,
        help="この日付以降のデータセットのみ使用（YYYY-MM-DD形式、例: 2026-06-16）",
    )
    ap.add_argument(
        "--ds-before", type=str, default=None,
        help="この日付より前のデータセットのみ使用（YYYY-MM-DD形式）",
    )
    ap.add_argument(
        "--closed-loop-uuids", type=str, default="",
        help="クローズドループ比較をレポートに含めるデータセット UUID をカンマ区切りで指定",
    )
    args = ap.parse_args()

    with open(args.params) as f:
        yaml_data = yaml.safe_load(f)
    params: dict = yaml_data.get("params", yaml_data)
    params["_score"] = yaml_data.get("score", "N/A")
    current_model, current_base_params, baseline_model, baseline_params, baseline_case = (
        _resolve_report_models(args.scenario, args.case)
    )
    phase_label = args.label if args.label is not None else current_model
    tuned_params = {k: v for k, v in params.items() if not k.startswith("_")}
    current_eval_params = {**current_base_params, **tuned_params}

    # Resolve fully merged parameters to extract vehicle geometry and steer bias overrides
    merged_tuned = merged_model_params(current_eval_params)
    steer_bias = float(merged_tuned.get("steer_bias", 0.0005))
    wheelbase = float(merged_tuned.get("wheelbase", 4.76012))

    # 1-2/1-3 末尾の baseline 比較テーブル用: baseline（補正なしデフォルト）の縦方向・操舵パラメータ
    merged_baseline = merged_model_params(baseline_params)
    baseline_tau_a = float(merged_baseline.get("acc_time_constant"))
    baseline_T_a = float(merged_baseline.get("acc_time_delay"))
    baseline_tau_d = float(merged_baseline.get("steer_time_constant"))
    baseline_T_d = float(merged_baseline.get("steer_time_delay"))

    # baseline と current(tuned) の車両モデル種別が一致するかどうか。
    # 1-2/1-3 末尾の方程式残差RMSE比較（equation_residual_at_params に baseline の (tau, delay) を
    # 代入する方式）は、baseline_model が current_model と "同じ運動方程式形" を持つ場合にのみ有効。
    # 例えば ideal_steer_acc は縦方向に一次遅れ・遅延を持たず、taiga_x は acc_time_constant/
    # acc_time_delay という parameterization 自体を持たないため、これらが baseline_model や
    # current_model に指定された場合、同じ式へのパラメータ代入という前提が崩れる。
    # _resolve_report_models はモデル種別の一致を保証しないため、ここで明示的に確認し、
    # 一致しない場合は 1-2/1-3 の新テーブル・残差ヒストグラムの baseline 重ね描画を無効化する。
    _model_types_match = (
        current_model == baseline_model or
        ({current_model, baseline_model} <= {"delay_steer_acc_geared_wo_fall_guard", "delay_steer_acc_geared_for_diffusion_planner"})
    )
    if not _model_types_match:
        print(
            f"  [WARN] baseline モデル種別 ({baseline_model}) が current モデル種別 ({current_model}) と"
            " 異なるため、1-2/1-3 の方程式残差RMSE baseline 比較はスキップします"
            "（運動方程式形が同一である保証がないため）。",
        )

    # Override global WHEELBASE in both this script and the imported module
    global WHEELBASE
    WHEELBASE = wheelbase
    import driving_log_replayer_v2.real_log_sim_comparison.lib._physical_validity as _pv
    _pv.WHEELBASE = wheelbase

    cache_identity = {
        "current_model_type": current_model,
        "baseline_model_type": baseline_model,
        "current_fingerprint": _model_fingerprint(current_model, current_eval_params),
        "baseline_fingerprint": _model_fingerprint(baseline_model, baseline_params),
    }
    print(f"パラメータ: {args.params.name}  (label={phase_label})")
    print(f"  current: {args.case} ({current_model})")
    print(f"  baseline: {baseline_case} ({baseline_model})")
    print(f"  k_us={params.get('k_us', 0):.5f} (全速度域一定)")
    print(f"  steer_bias={steer_bias:.5f} rad")
    print(f"  wheelbase={wheelbase:.5f} m")
    print(f"  steer_dead_band={params.get('steer_dead_band',0):.5f} rad")

    # データセット列挙
    import datetime as _dt
    ds_list = _discover(args.collection_dir)
    for extra in (args.extra_ds or []):
        uuid = extra.name
        if not any(u == uuid for u, _ in ds_list):
            ds_list.append((uuid, extra))
            print(f"  [extra-ds] {uuid} を追加")
    ds_after_date  = _dt.date.fromisoformat(args.ds_after)  if args.ds_after  else None
    ds_before_date = _dt.date.fromisoformat(args.ds_before) if args.ds_before else None
    if ds_after_date or ds_before_date:
        ds_list = _filter_by_date(ds_list, ds_before_date, ds_after_date)
    print(f"\nデータセット: {len(ds_list)} 件")

    # Phase 1: 並列 MCAP 読み込み
    print("\n[Phase 1] MCAP 並列読み込み ...")
    records = load_all_mcap(ds_list, steer_bias=steer_bias, n_jobs=args.n_jobs)
    print(f"  有効: {len(records)} 件")

    # Phase 2: スカラー k_us 最小二乗法
    print("\n[Phase 2] スカラー k_us 最小二乗法 ...")
    bins = compute_kus_bins(records)
    print(f"  k_us={bins['k_us']:.5f} (曲線走行サンプル n={bins['n_pts']})")

    # Phase 2b: 縦方向 / 操舵 per-dataset 実行時フィット（旧 identify_*_dynamics.py の
    # 事前 CSV 生成を置き換え。最長連続時系列選定の母集団として全件を並列同定する）
    entries = _to_entries(ds_list)
    n_fit_target = len([e for e in entries if e.real_lite is not None])
    print(f"\n[Phase 2b] 縦方向・操舵 per-dataset フィット (全 {n_fit_target} データセット並列) ...")
    per_ds_long, per_ds_steer = fit_per_dataset(entries, n_jobs=args.n_jobs)
    print(f"  縦方向: {len(per_ds_long)}/{n_fit_target} 件、操舵: {len(per_ds_steer)}/{n_fit_target} 件 同定成功")

    # Phase 3: カーブ多データセット選定
    print("\n[Phase 3] カーブ データセット 選定 ...")
    record_by_uuid = {r["uuid"]: r for r in records}

    def _resolve_uuids(prefix_list: list[str]) -> list[dict]:
        result = []
        seen = set()
        for u in prefix_list:
            matched = [v for k, v in record_by_uuid.items() if k.startswith(u)]
            if not matched:
                print(f"  ⚠ UUID '{u}' が records に見つかりません（スキップ）")
            for m in matched:
                if m["uuid"] not in seen:
                    seen.add(m["uuid"])
                    result.append(m)
        return result

    # まず pinned UUID を先頭に確保
    pinned_prefixes = [u.strip() for u in args.pinned_uuids.split(",") if u.strip()] if args.pinned_uuids else []
    pinned_records = _resolve_uuids(pinned_prefixes)
    pinned_uuids_set = {r["uuid"] for r in pinned_records}

    if args.viewer_uuids:
        requested = [u.strip() for u in args.viewer_uuids.split(",") if u.strip()]
        extra = [r for r in _resolve_uuids(requested) if r["uuid"] not in pinned_uuids_set]
        candidate_curve = pinned_records + extra
        print(f"  --viewer-uuids 指定順モード: pinned={len(pinned_records)} + extra={len(extra)}")
    else:
        records_sorted = sorted(records, key=lambda r: r["curve_count"], reverse=True)
        auto = [r for r in records_sorted if r["uuid"] not in pinned_uuids_set]
        candidate_curve = pinned_records + auto

    top_curve = candidate_curve[: args.n_curve_ds]
    for r in top_curve:
        pinned_mark = " [pinned]" if r["uuid"] in pinned_uuids_set else ""
        print(f"  {r['uuid'][:12]}  curve_count={r['curve_count']}  kappa_max={r['kappa_max_abs']:.4f}{pinned_mark}")

    # Phase 3b / 3c: DatasetCtx 構築 & rollout メトリクス
    top_items = [(r["uuid"], Path(r["lite_dir"])) for r in top_curve]
    deviation_html = ""
    cache_is_valid = bool(
        args.metrics_cache
        and args.metrics_cache.exists()
        and _metrics_cache_matches(args.metrics_cache, cache_identity)
    )
    baseline_score = None
    if cache_is_valid:
        # キャッシュあり → viewer データセット のみ load、メトリクスは CSV から読む
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} データセット) ...")
        ctxs = load_datasets(
            top_items,
            n_jobs=min(args.n_jobs, len(top_items)),
            baseline_model_type=baseline_model,
            baseline_params=baseline_params,
        )
        print(f"\n[Phase 3c] rollout メトリクスキャッシュ読み込み ...")
        df_rollout = pd.read_csv(args.metrics_cache)
        n_dataset_cache = df_rollout["uuid"].nunique()
        n_h_cache = df_rollout["h"].nunique()
        print(f"  {len(df_rollout)} 行（{n_dataset_cache} データセット × {n_h_cache} horizons）")
        # score 再現検証（キャッシュロード時も実施）
        per_ds_arg = []
        bl_arg: dict = {}
        for uuid_key, grp in df_rollout.groupby("uuid"):
            gd = grp.set_index("h")[
                ["p14_yaw", "p14_long", "p14_lat", "bl_yaw", "bl_long", "bl_lat"]
            ].to_dict("index")
            per_ds_arg.append((
                uuid_key,
                {int(h): {"yaw": v["p14_yaw"], "long": v["p14_long"], "lat": v["p14_lat"]}
                 for h, v in gd.items()},
            ))
            bl_arg[uuid_key] = {
                int(h): {"yaw": v["bl_yaw"], "long": v["bl_long"], "lat": v["bl_lat"]}
                for h, v in gd.items()
            }
        agg = _agg_normalized(per_ds_arg, bl_arg)
        expected = float(yaml_data.get("score") or 0.0)
        candidates = [
            ("robust_score", _robust_score(agg)),
            ("steer_score",  _steer_score(agg)),
            ("acc_score",    _acc_score(agg)),
        ]
        if expected:
            best_name, recomputed = min(candidates, key=lambda kv: abs(kv[1] - expected))
        else:
            best_name, recomputed = next(kv for kv in candidates if kv[0] == "steer_score")
        diff_str = f"{abs(recomputed - expected) / expected * 100:.2f}%" if expected else "N/A"
        print(f"  再現スコア: {recomputed:.4f} ({best_name})  期待値: {expected:.4f}  差: {diff_str}")
        # baseline (k_us=0) の該当スコアを計算
        if best_name == "robust_score":
            baseline_score = _robust_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        elif best_name == "acc_score":
            baseline_score = _acc_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        else:
            baseline_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline {best_name}: {baseline_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    elif args.metrics_cache:
        # キャッシュなし → 全データセット load（ついでに viewer データセット も取り出す）
        all_items = [(r["uuid"], Path(r["lite_dir"])) for r in records]
        print(f"\n[Phase 3b+3c] 全データセット DatasetCtx 構築 ({len(all_items)} データセット) + rollout メトリクス計算 ...")
        all_ctxs = load_datasets(
            all_items,
            n_jobs=args.n_jobs,
            baseline_model_type=baseline_model,
            baseline_params=baseline_params,
        )
        # viewer データセットを all_ctxs から抽出（重複 load 回避）
        ctxs_by_id = {c.dataset_id: c for c in all_ctxs}
        ctxs = [ctxs_by_id[r["uuid"]] for r in top_curve if r["uuid"] in ctxs_by_id]
        # phase14 override: YAML の全 params から _* メタキーを除外（hand-pick より安全）
        rows = []
        for i, ctx in enumerate(all_ctxs, 1):
            try:
                p14 = _tune_eval(ctx, current_eval_params, current_model)
            except Exception as e:
                print(f"  [WARN] {ctx.dataset_id[:12]}: eval 失敗 ({e})")
                continue
            bl = ctx.base_metric
            for h in _HORIZONS:
                rows.append({
                    "uuid": ctx.dataset_id, "h": h,
                    "p14_yaw": p14[h]["yaw"], "p14_long": p14[h]["long"],
                    "p14_lat": p14[h]["lat"], "p14_vx": p14[h]["vx"],
                    "bl_yaw": bl[h]["yaw"], "bl_long": bl[h]["long"],
                    "bl_lat": bl[h]["lat"], "bl_vx": bl[h]["vx"],
                    **cache_identity,
                })
            if i % 100 == 0:
                print(f"  {i}/{len(all_ctxs)} 完了", flush=True)
        df_rollout = pd.DataFrame(rows)
        args.metrics_cache.parent.mkdir(parents=True, exist_ok=True)
        df_rollout.to_csv(args.metrics_cache, index=False)
        print(f"  キャッシュ保存: {args.metrics_cache}")
        # score 再現検証
        per_ds_arg = []
        bl_arg: dict = {}
        for uuid_key, grp in df_rollout.groupby("uuid"):
            grp_dict = grp.set_index("h")[
                ["p14_yaw", "p14_long", "p14_lat", "bl_yaw", "bl_long", "bl_lat"]
            ].to_dict("index")
            per_ds_arg.append((
                uuid_key,
                {int(h): {"yaw": v["p14_yaw"], "long": v["p14_long"], "lat": v["p14_lat"]}
                 for h, v in grp_dict.items()},
            ))
            bl_arg[uuid_key] = {
                int(h): {"yaw": v["bl_yaw"], "long": v["bl_long"], "lat": v["bl_lat"]}
                for h, v in grp_dict.items()
            }
        agg = _agg_normalized(per_ds_arg, bl_arg)
        expected = float(yaml_data.get("score") or 0.0)
        # YAML の score は tuning --phase に応じて steer/acc/robust のいずれかなので最接近を選択
        candidates = [
            ("robust_score", _robust_score(agg)),
            ("steer_score",  _steer_score(agg)),
            ("acc_score",    _acc_score(agg)),
        ]
        if expected:
            best_name, recomputed = min(candidates, key=lambda kv: abs(kv[1] - expected))
        else:
            best_name, recomputed = next(kv for kv in candidates if kv[0] == "steer_score")
        diff_str = f"{abs(recomputed - expected) / expected * 100:.2f}%" if expected else "N/A"
        print(f"  再現スコア: {recomputed:.4f} ({best_name})  期待値: {expected:.4f}  差: {diff_str}")
        # baseline (k_us=0) の該当スコアを計算
        if best_name == "robust_score":
            baseline_score = _robust_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        elif best_name == "acc_score":
            baseline_score = _acc_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        else:
            baseline_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline {best_name}: {baseline_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    else:
        pass
        # --metrics-cache 未指定 → 通常の viewer データセット のみ load
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} データセット) ...")
        ctxs = load_datasets(
            top_items,
            n_jobs=min(args.n_jobs, len(top_items)),
            baseline_model_type=baseline_model,
            baseline_params=baseline_params,
        )

    curve_count_map = {r["uuid"]: r["curve_count"] for r in top_curve}

    # Viewer はモデル種別も含む完全な設定を受け取り、異種モデル間で式を切り替える。
    configs: dict[str, dict] = {
        f"{phase_label}（{args.case} / {current_model}）": {
            **current_eval_params,
            "vehicle_model_type": current_model.upper(),
        },
        f"baseline（{baseline_case} / {baseline_model}）": {
            **baseline_params,
            "vehicle_model_type": baseline_model.upper(),
        },
    }

    # 地図ロード（デフォルトパス自動解決）
    map_osm_path = resolve_map_osm(None)
    map_ways = load_map_ways(map_osm_path) if map_osm_path else None
    if map_ways:
        print(f"  地図ロード完了: {map_osm_path} ({len(map_ways)} ways)")
    else:
        print("  地図なし（ビューアは軌跡のみ表示）")

    viewer_sections: list[str] = []
    for ctx in ctxs:
        # 最初のカーブ開始時刻を検出してプリシーク位置を決定
        initial_t = _find_first_curve_t(ctx, pre_roll_s=5.0)
        vh = _build_viewer_html(ctx, configs, ctx.base, map_ways=map_ways, initial_t=initial_t)
        if vh is None:
            print(f"  {ctx.dataset_id[:12]}: ビューア生成スキップ（データ不足）")
            continue
        srcdoc = _html_stdlib.escape(vh, quote=True)
        cc = curve_count_map.get(ctx.dataset_id, "?")
        viewer_sections.append(f"""
<h3>Dataset: <code>{ctx.dataset_id}</code>  &nbsp;（curve_count = {cc}）</h3>
<p style="font-size:11px;color:#888">
  ドロップダウンで config を切り替え、つまみでパラメータを手動調整できます。
  「最適化」ボタンで最小二乗フィットも実行できます。
</p>
<iframe srcdoc="{srcdoc}"
  width="100%" height="1300"
  style="border:1px solid #ccc;border-radius:4px"
  loading="lazy"></iframe>
""")

    # Phase 4: HTML 組み立て
    print("\n[Phase 4] plotly 図生成 & HTML 組み立て ...")
    kus_fig   = build_fig_kus_single(
        bins, {"チューニング済み": params},
        title="実機ログからのアンダーステア係数独立同定（最小二乗法回帰・スカラー）",
    )
    # 縦方向 / 操舵: 共有ライブラリの実行時フィット (per-dataset フィット結果 + 横断フィット)
    # から最長連続時系列図を生成する（旧: 事前生成 CSV 依存の build_long_figure /
    # build_steer_id_figure）。凡例にチューン値 τ/T を残すためモデル名に値を埋め込む。
    tuned_clean = current_eval_params
    merged_tuned = merged_model_params(tuned_clean)

    def _model_name(tau_key: str, delay_key: str) -> str:
        tau = merged_tuned.get(tau_key)
        delay = merged_tuned.get(delay_key)
        if tau is None or delay is None:
            return phase_label
        return f"{phase_label} τ={float(tau):.3f}s T={float(delay):.3f}s"

    models_long = {_model_name("acc_time_constant", "acc_time_delay"): SimpleNamespace(params=tuned_clean)}
    models_steer = {_model_name("steer_time_constant", "steer_time_delay"): SimpleNamespace(params=tuned_clean)}

    cross_fit_long = fit_long_cross_dataset_bounded(entries, per_ds_long)
    if np.isfinite(cross_fit_long.get("tau", float("nan"))):
        print(
            f"  縦方向 横断同定: τ={cross_fit_long['tau']:.3f}s"
            f" 遅延={cross_fit_long['delay']:.3f}s"
            f" RMSE={cross_fit_long['rmse_mps2']:.3f} m/s²"
            f" ({cross_fit_long.get('n_datasets', 0)} データセット)"
        )
    else:
        print("  縦方向 横断同定: 失敗（有効データセットなし）")

    long_steer_pinned_uuids = [
        u.strip() for u in args.long_steer_pinned_uuids.split(",") if u.strip()
    ]
    rows_long = compute_cross_long_rows(
        entries, per_ds_long, cross_fit_long, models_long, pinned_uuids=long_steer_pinned_uuids,
        n_jobs=args.n_jobs,
    )
    long_fig = build_fig_cross_long(rows_long, cross_fit_long)

    rows_steer = compute_cross_steer_rows(
        entries, per_ds_steer, models_steer, pinned_uuids=long_steer_pinned_uuids,
        n_jobs=args.n_jobs,
    )
    # 旧実装の表示要素を維持: 各 subplot タイトルに per-dataset 同定値 τ/T を付記
    for r in rows_steer:
        for ds_id, fit in per_ds_steer.items():
            if ds_id[:8] in r["label"]:
                r["label"] += f"  τ={fit['tau']:.3f}s T={fit['delay']:.3f}s"
                break
    steer_fig = build_fig_cross_steer(rows_steer)

    for prefix in long_steer_pinned_uuids:
        for name, rows in (("縦方向 (1-2)", rows_long), ("操舵 (1-3)", rows_steer)):
            hit = any(
                str(d).startswith(prefix)
                for r in rows
                for d in r.get("dataset_ids", [r.get("dataset_id", "")])
            )
            if not hit:
                print(f"  ⚠ 指定データセット '{prefix}' が {name} の時系列グラフに反映されませんでした（同定失敗 or データなし）")

    # 方程式残差の評価結果: 各データセットの3段階交互最適化で同定されたパラメータでの残差 E[k]=RHS−LHS を
    # 全データセットでプールし分布を示す。
    def _pool_resid(per_ds: dict[str, dict]) -> tuple[list[float], float, list[float]]:
        pooled: list[float] = []
        rmses: list[float] = []
        for fit in per_ds.values():
            pooled.extend(fit.get("resid_samples", []) or [])
            rr = fit.get("rmse_resid", float("nan"))
            if np.isfinite(rr):
                rmses.append(float(rr))
        med = float(np.median(rmses)) if rmses else float("nan")
        return pooled, med, rmses

    def _pool_resid_at_params(
        per_ds: dict[str, dict], tau: float, delay: float, window_s: float,
    ) -> tuple[list[float], float, list[float]]:
        """per_ds に保存済みの実測配列 (cmd_arr/act_arr/mask_arr/t_s) を使い、
        baseline など任意の (tau, delay) での方程式残差 RMSE を再計算する。
        tuned 側 (_pool_resid) と同じプール方式で pooled residuals・中央値・
        per-dataset RMSE 一覧を返す（実機ログの再読み込み・モデル再フィットは不要）。
        """
        pooled: list[float] = []
        rmses: list[float] = []
        for fit in per_ds.values():
            if "cmd_arr" not in fit or "act_arr" not in fit or "mask_arr" not in fit:
                continue
            cmd = np.asarray(fit["cmd_arr"], dtype=float)
            act = np.asarray(fit["act_arr"], dtype=float)
            mask = np.asarray(fit["mask_arr"], dtype=bool)
            t_s = np.asarray(fit.get("t_s", np.arange(len(cmd)) * _FIT_DT), dtype=float)
            resid = equation_residual_at_params(
                cmd, act, mask, _FIT_DT, tau=tau, delay=delay,
                scale=1.0, bias=0.0, window_s=window_s, polyorder=_SG_POLYORDER, t_s=t_s,
            )
            pooled.extend(resid["resid"].tolist())
            if np.isfinite(resid["rmse_resid"]):
                rmses.append(float(resid["rmse_resid"]))
        med = float(np.median(rmses)) if rmses else float("nan")
        return pooled, med, rmses

    final_tau_a = float(merged_tuned.get("acc_time_constant"))
    final_T_a = float(merged_tuned.get("acc_time_delay"))
    final_tau_d = float(merged_tuned.get("steer_time_constant"))
    final_T_d = float(merged_tuned.get("steer_time_delay"))

    _resid_long_pool, _resid_long_med, _resid_long_rmses = _pool_resid_at_params(
        per_ds_long, final_tau_a, final_T_a, _SG_WINDOW_LONG,
    )
    _resid_steer_pool, _resid_steer_med, _resid_steer_rmses = _pool_resid_at_params(
        per_ds_steer, final_tau_d, final_T_d, _SG_WINDOW_STEER,
    )

    if _model_types_match:
        _resid_long_bl_pool, _resid_long_bl_med, _resid_long_bl_rmses = _pool_resid_at_params(
            per_ds_long, baseline_tau_a, baseline_T_a, _SG_WINDOW_LONG,
        )
        _resid_steer_bl_pool, _resid_steer_bl_med, _resid_steer_bl_rmses = _pool_resid_at_params(
            per_ds_steer, baseline_tau_d, baseline_T_d, _SG_WINDOW_STEER,
        )
        _long_phase_tuned = _pool_resid_long_baseline_by_phase(
            per_ds_long, final_tau_a, final_T_a, _SG_WINDOW_LONG,
        )
        _long_phase_baseline = _pool_resid_long_baseline_by_phase(
            per_ds_long, baseline_tau_a, baseline_T_a, _SG_WINDOW_LONG,
        )
    else:
        # baseline_model と current_model の運動方程式形が同一である保証がないため、
        # baseline の (tau, delay) を代入した残差計算・比較テーブルは行わない。
        _resid_long_bl_pool, _resid_long_bl_med, _resid_long_bl_rmses = [], float("nan"), []
        _resid_steer_bl_pool, _resid_steer_bl_med, _resid_steer_bl_rmses = [], float("nan"), []
        _long_phase_tuned, _long_phase_baseline = {}, {}

    long_resid_hist_fig = build_fig_equation_residual_hist(
        _resid_long_pool, channel_label="縦 (dot a_act 式)", unit_label="m/s³",
        rmse_median=_resid_long_med,
        baseline_resid_samples=_resid_long_bl_pool or None, baseline_rmse_median=_resid_long_bl_med,
    )
    steer_resid_hist_fig = build_fig_equation_residual_hist(
        _resid_steer_pool, channel_label="操舵 (dot δ_act 式)", unit_label="rad/s",
        rmse_median=_resid_steer_med,
        baseline_resid_samples=_resid_steer_bl_pool or None, baseline_rmse_median=_resid_steer_bl_med,
    )

    if _model_types_match:
        long_fit_rmse_html = _build_fit_rmse_table_html(
            _resid_long_rmses, _resid_long_bl_rmses, "m/s³", phase_label,
            phase_tuned=_long_phase_tuned, phase_baseline=_long_phase_baseline,
        )
        steer_fit_rmse_html = _build_fit_rmse_table_html(
            _resid_steer_rmses, _resid_steer_bl_rmses, "rad/s", phase_label,
        )
    else:
        _model_mismatch_note = (
            '<div class="note" style="border-color:#dc3545">'
            f"⚠️ baseline（<code>{baseline_model}</code>）と {phase_label}（<code>{current_model}</code>）は"
            "車両モデル種別が異なるため、方程式残差RMSEの baseline 比較"
            "（同一の一次遅れ+純粋遅延式へのパラメータ代入方式）は運動方程式形の一致を前提としており、"
            "意味のある値にならないため省略しました。"
            "</div>"
        )
        long_fit_rmse_html = _model_mismatch_note
        steer_fit_rmse_html = _model_mismatch_note

    # 1-5 横方向理想追従評価には curve 上位 _PERF_N_DATASET データセットを使用（viewer 用 top_curve とは独立して選択）
    perf_records = candidate_curve[:_PERF_N_DATASET]
    print(f"  [1-5] 横方向理想追従評価図生成 ({len(perf_records)} データセット) ...")
    perf_entries = _to_entries([(r["uuid"], Path(r["lite_dir"])) for r in perf_records])
    perf_data = compute_perfect_tracking_data(perf_entries, params)
    perf_fig_box = build_fig_perfect_tracking_box(perf_data)
    perf_fig_traj = build_fig_perfect_tracking_traj(
        perf_data,
        labels={
            "title": "実車 vs 自転車モデル軌跡（実測 v_x + 実舵角入力、初期状態を実車ログに合わせたリセットなし積分）",
            "gt_name": "実車 軌跡",
            "model_name": "自転車モデル（理想追従）",
            "x_title": "Δx [m]",
            "y_title": "Δy [m]",
        },
        height=480,
    )
    perf_html = _build_sec14(perf_fig_box, perf_fig_traj, params, len(perf_records))

    # [1-2] 方程式残差による縦方向パラメータ最適化とヒストグラム生成
    print("  [1-2] 方程式残差による縦方向パラメータ最適化とヒストグラム生成...")
    long_fixed_tau = cross_fit_long.get("phase1_tau") if (cross_fit_long and "phase1_tau" in cross_fit_long) else None
    top_entries_long = sorted(
        (e for e in entries if e.real_lite is not None and e.dataset_id in per_ds_long),
        key=lambda e: per_ds_long[e.dataset_id].get("n_dyn", 0),
        reverse=True,
    )[:_N_CROSS_FIT_DATASET]
    top_dataset_ids_long = {e.dataset_id for e in top_entries_long}
    filtered_per_ds_long = {k: v for k, v in per_ds_long.items() if k in top_dataset_ids_long}
    long_opt_results = optimize_tau_with_equation_residual(
        filtered_per_ds_long,
        delay_candidates=_DELAY_CANDIDATES_LONG,
        dt=_FIT_DT,
        tau_bounds=_TAU_BOUNDS_LONG,
        filter_w=10,
        fixed_tau=long_fixed_tau,
    )
    long_resid_opt_html = ""
    if long_opt_results:
        long_opt_fig = build_fig_residual_candidates_hist(
            long_opt_results,
            channel_label="縦 (dot a_act 式)",
            unit_label="m/s³",
        )
        long_opt_rows = ""
        for r in long_opt_results:
            selected_style = "style='background-color: #e8f5e9; font-weight: bold;'" if r["selected"] else ""
            selected_text = "<b>★ 選択</b>" if r["selected"] else "—"
            long_opt_rows += f"""<tr {selected_style}>
                <td>{r['delay']:.3f} s</td>
                <td>{r['tau']:.3f} s</td>
                <td>{r['rmse']:.4f} m/s³</td>
                <td>{r['mean']:.4e} m/s³</td>
                <td>{r['std']:.4f} m/s³</td>
                <td>{selected_text}</td>
            </tr>"""
        long_opt_fig_html = long_opt_fig.to_html(full_html=False, include_plotlyjs=False)
        long_process_log = ""
        if "phase1_tau" in cross_fit_long:
            p1_t = cross_fit_long["phase1_tau"]
            p1_d = cross_fit_long.get("phase1_delay", 0.101)
            p2_d = cross_fit_long["phase2_delay"]
            p3_t = cross_fit_long["phase3_tau"]
            p3_d = cross_fit_long["phase3_delay"]
            long_process_log = f"""
            <div class="note" style="margin-top: 10px; margin-bottom: 15px; border-left: 4px solid #4caf50; background-color: #f9f9f9; padding: 10px;">
              <b>【実行ログ】縦方向・横断同定における最適化の遷移挙動:</b>
              <ul style="margin: 5px 0 0 0; padding-left: 20px;">
                <li><b>Phase 1 (時定数固定):</b> \\(\\tau_a\\) を固定値 \\({p1_t:.4f}\\) s として設定（初期遅延 \\(T_a = {p1_d:.3f}\\) s、最適化はスキップ）</li>
                <li><b>Phase 2 (遅延グリッドサーチ):</b> 固定した \\(\\tau_a = {p1_t:.4f}\\) s のまま、残差二乗和を最小化する遅延を決定 &rarr; \\(T_a = {p2_d:.3f}\\) s （下表で最小RMSEとなる行）</li>
                <li><b>Phase 3 (再最適化スキップ):</b> 縦方向では基準値での時定数決定の固定関係を維持するため、Phase 3 (再最適化) をスキップし、Phase 1/2 の結果（\\(\\tau_a = {p3_t:.4f}\\) s, \\(T_a = {p3_d:.3f}\\) s）を最終同定値として採用します。</li>
              </ul>
            </div>
            """

        long_resid_opt_html = f"""
        <details>
          <summary><b>遅延固定時の時定数最適化と方程式残差の評価結果（クリックで展開）</b></summary>
          <p>
            遅延時間 \\(T_a\\) を \\(\\Delta t\\) の整数倍に固定した上で、方程式残差の二乗和（MSE）を最小化するように時定数 \\(\\tau_a\\) を最適化した結果です。全データセットから抽出した動的サンプルをプールして評価しています。
          </p>
          {long_process_log}
          <table class="param-table">
            <tr><th>固定遅延 \\(T_a\\)</th><th>最適時定数 \\(\\tau_a\\)</th><th>方程式残差 RMSE</th><th>残差平均</th><th>残差標準偏差</th><th>判定</th></tr>
            {long_opt_rows}
          </table>
          {long_opt_fig_html}
        </details>
        """

    # [1-3] 方程式残差による操舵パラメータ最適化とヒストグラム生成
    print("  [1-3] 方程式残差による操舵パラメータ最適化とヒストグラム生成...")
    steer_opt_results = optimize_tau_with_equation_residual(
        per_ds_steer,
        delay_candidates=_DELAY_CANDIDATES_STEER,
        dt=_FIT_DT,
        tau_bounds=_TAU_BOUNDS_STEER,
        filter_w=1,
    )
    steer_resid_opt_html = ""
    if steer_opt_results:
        steer_opt_fig = build_fig_residual_candidates_hist(
            steer_opt_results,
            channel_label="操舵 (dot δ_act 式)",
            unit_label="rad/s",
        )
        steer_opt_rows = ""
        for r in steer_opt_results:
            selected_style = "style='background-color: #e8f5e9; font-weight: bold;'" if r["selected"] else ""
            selected_text = "<b>★ 選択</b>" if r["selected"] else "—"
            steer_opt_rows += f"""<tr {selected_style}>
                <td>{r['delay']:.3f} s</td>
                <td>{r['tau']:.3f} s</td>
                <td>{r['rmse']:.4f} rad/s</td>
                <td>{r['mean']:.4e} rad/s</td>
                <td>{r['std']:.4f} rad/s</td>
                <td>{selected_text}</td>
            </tr>"""
        steer_opt_fig_html = steer_opt_fig.to_html(full_html=False, include_plotlyjs=False)
        steer_resid_opt_html = f"""
        <details>
          <summary><b>遅延固定時の時定数最適化と方程式残差の評価結果（クリックで展開）</b></summary>
          <p>
            遅延時間 \\(T_\\delta\\) を \\(\\Delta t\\) の整数倍に固定した上で、方程式残差の二乗和（MSE）を最小化するように時定数 \\(\\tau_\\delta\\) を最適化した結果です（3段階交互最適化における <b>Phase 2 (遅延グリッドサーチ)</b> のプール探索プロセスに対応します。判定が「★ 選択」された行の遅延時間が、Phase 3 での時定数再最適化に用いられます）。全データセットから抽出した動的サンプルをプールして評価しています。
          </p>
          <table class="param-table">
            <tr><th>固定遅延 \\(T_\\delta\\)</th><th>最適時定数 \\(\\tau_\\delta\\)</th><th>方程式残差 RMSE</th><th>残差平均</th><th>残差標準偏差</th><th>判定</th></tr>
            {steer_opt_rows}
          </table>
          {steer_opt_fig_html}
        </details>
        """

    closed_loop_html = _build_sec_closed_loop_comparison(args.collection_dir, args.closed_loop_uuids)

    html = build_html(
        params, long_fig, steer_fig, kus_fig, viewer_sections, len(records),
        baseline_score=baseline_score,
        deviation_html=deviation_html,
        label=phase_label,
        params_filename=args.params.name,
        perf_html=perf_html,
        closed_loop_html=closed_loop_html,
        long_resid_hist_fig=long_resid_hist_fig,
        steer_resid_hist_fig=steer_resid_hist_fig,
        long_resid_opt_html=long_resid_opt_html,
        steer_resid_opt_html=steer_resid_opt_html,
        long_fit_rmse_html=long_fit_rmse_html,
        steer_fit_rmse_html=steer_fit_rmse_html,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding="utf-8")
    size_kb = args.out.stat().st_size // 1024
    print(f"\n✓ 完了: {args.out}  ({size_kb} KB)")

    if args.release_note_out:
        release_note_html = build_release_note_html(
            deviation_html=deviation_html,
            long_fig=long_fig,
            steer_fig=steer_fig,
            long_resid_hist_fig=long_resid_hist_fig,
            steer_resid_hist_fig=steer_resid_hist_fig,
            long_fit_rmse_html=long_fit_rmse_html,
            steer_fit_rmse_html=steer_fit_rmse_html,
            label=phase_label,
            params_filename=args.params.name,
            n_dataset=len(records),
            current_model=current_model,
        )
        args.release_note_out.parent.mkdir(parents=True, exist_ok=True)
        args.release_note_out.write_text(release_note_html, encoding="utf-8")
        rn_size_kb = args.release_note_out.stat().st_size // 1024
        print(f"✓ 完了（リリースノート）: {args.release_note_out}  ({rn_size_kb} KB)")


if __name__ == "__main__":
    main()
