#!/usr/bin/env python3
"""
物理的妥当性レポート生成スクリプト

速度依存 k_us ランプと操舵不感帯の物理的妥当性を、
実機ログからの独立同定と理論式の両面から検証する HTML レポートを生成する。

使用法:
  python physical_validity_report.py \\
    --params /home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_june_phase14.yaml \\
    --collection-dir /home/kotaroyoshimoto/data/openloop_j6_15_june \\
    --out /home/kotaroyoshimoto/data/openloop_j6_15/physical_validity_report.html
"""
from __future__ import annotations

import argparse
import html as _html_stdlib
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import yaml
from plotly.subplots import make_subplots

# ---------------------------------------------------------------------------
# sys.path セットアップ
# ---------------------------------------------------------------------------
_INSTALL = Path(
    "/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/"
    "driving_log_replayer_v2/local/lib/python3.10/dist-packages"
)
if _INSTALL.exists() and str(_INSTALL) not in sys.path:
    sys.path.insert(0, str(_INSTALL))

from driving_log_replayer_v2.real_log_sim_comparison.lib._coverage import _curvature_coverage  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_kinematic, load_steering  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._map import load_map_ways, resolve_map_osm  # noqa: E402
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
    load_datasets,
)

# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------
WHEELBASE = 4.76012   # [m]
STEER_BIAS = 0.0005   # [rad]
VX_MIN_CURVE = 1.5    # [m/s]
WZ_MIN = 0.02         # [rad/s]
DWZ_MAX = 0.30        # [rad/s²]
K_US_CLIP = 0.5
VX_EDGES = np.array([0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0])
_H_SPAN = {10: "≈0.33 s", 20: "≈0.67 s", 30: "≈1.0 s", 40: "≈1.33 s"}

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
"""


# ---------------------------------------------------------------------------
# Phase 1: 並列 MCAP 読み込みワーカー（k_us 分析 + カーブカバレッジ）
# ---------------------------------------------------------------------------
def _load_mcap_worker(args: tuple) -> dict | None:
    """プロセスワーカー: 1 MCAP から k_us 分析用データとカーブカバレッジを抽出。"""
    # 引数は (uuid: str, lite_dir: str) のタプル（pickle 対応のため str）
    uuid, lite_dir_str = args
    lite_dir = Path(lite_dir_str)
    mcap = lite_dir / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None
    try:
        df_kin = load_kinematic(mcap)
        df_steer = load_steering(mcap)
    except Exception:
        return None
    if df_kin.empty or df_steer.empty or len(df_kin) < 10:
        return None

    t_k = df_kin["t_ns"].values * 1e-9
    vx = df_kin["vx"].values
    wz = df_kin["wz"].values

    steer_raw = df_steer["steer"].values
    t_s = df_steer["t_ns"].values * 1e-9
    steer = np.interp(t_k, t_s, steer_raw)
    steer_eff = steer - STEER_BIAS

    # 角加速度（定常旋回フィルタ用）
    dt = np.diff(t_k)
    dt_safe = np.where(dt > 0, dt, 1e-3)
    dwz_mid = np.diff(wz) / dt_safe
    dwz = np.empty_like(wz)
    dwz[0] = dwz_mid[0] if len(dwz_mid) > 0 else 0.0
    dwz[-1] = dwz_mid[-1] if len(dwz_mid) > 0 else 0.0
    dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])

    # カーブカバレッジ（_coverage._curvature_coverage は t 列を要求）
    t0_ns = df_kin["t_ns"].values[0]
    t_rel = (df_kin["t_ns"].values - t0_ns) * 1e-9
    kin_cv = pd.DataFrame({"t": t_rel, "yaw": df_kin["yaw"].values})
    vel_cv = pd.DataFrame({"t": t_rel, "lon_vel": vx})
    try:
        cov = _curvature_coverage(kin_cv, vel_cv)
    except Exception:
        cov = {"curve_count": 0, "kappa_max_abs": 0.0}

    return {
        "uuid": uuid,
        "lite_dir": lite_dir_str,
        "vx": vx.tolist(),
        "wz": wz.tolist(),
        "steer_eff": steer_eff.tolist(),
        "dwz": dwz.tolist(),
        "curve_count": cov["curve_count"],
        "kappa_max_abs": cov["kappa_max_abs"],
    }


def load_all_mcap(ds_list: list, n_jobs: int = 8) -> list[dict]:
    """全データセットを並列 MCAP 読み込み。"""
    args_list = [(uuid, str(lite_dir)) for uuid, lite_dir in ds_list]
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
# Phase 2: k_us 速度ビン別 OLS 回帰
# ---------------------------------------------------------------------------
def compute_kus_bins(records: list[dict]) -> dict:
    """速度ビン別 OLS 回帰で k_us(v) を推定。モデル: tan(δ_eff) = (L/v + k_us·v)·ω"""
    all_vx = np.concatenate([np.asarray(r["vx"]) for r in records])
    all_wz = np.concatenate([np.asarray(r["wz"]) for r in records])
    all_steer_eff = np.concatenate([np.asarray(r["steer_eff"]) for r in records])
    all_dwz = np.concatenate([np.asarray(r["dwz"]) for r in records])

    mask_ok = (
        (np.abs(all_wz) > WZ_MIN)
        & (np.abs(all_dwz) < DWZ_MAX)
        & (all_vx > VX_MIN_CURVE)
    )
    vx_f = all_vx[mask_ok]
    wz_f = all_wz[mask_ok]
    steer_f = all_steer_eff[mask_ok]
    tan_steer = np.tan(np.clip(steer_f, -0.8, 0.8))

    n_bins = len(VX_EDGES) - 1
    vx_mid = np.empty(n_bins)
    kus_ols = np.full(n_bins, np.nan)
    kus_p25 = np.full(n_bins, np.nan)
    kus_p75 = np.full(n_bins, np.nan)
    n_pts = np.zeros(n_bins, dtype=int)

    for i in range(n_bins):
        lo, hi = VX_EDGES[i], VX_EDGES[i + 1]
        mask_bin = (vx_f >= lo) & (vx_f < hi)
        n = int(mask_bin.sum())
        n_pts[i] = n
        vx_mid[i] = (lo + hi) / 2
        if n < 10:
            continue
        vx_b = vx_f[mask_bin]
        wz_b = wz_f[mask_bin]
        ts_b = tan_steer[mask_bin]
        vm = float(np.median(vx_b))
        vx_mid[i] = vm
        # OLS 原点回帰: tan(steer) = C * wz => k_us = (C - L/v) / v
        C_ols = float(np.sum(wz_b * ts_b) / np.sum(wz_b ** 2))
        kus_ols[i] = (C_ols - WHEELBASE / vm) / vm
        # 個別サンプル percentile（外れ値確認用）
        with np.errstate(divide="ignore", invalid="ignore"):
            kus_each = (
                ts_b / np.where(np.abs(wz_b) > 1e-6, wz_b, np.nan) - WHEELBASE / vx_b
            ) / vx_b
        kus_each = kus_each[np.isfinite(kus_each)]
        kus_each = np.clip(kus_each, -K_US_CLIP, K_US_CLIP)
        if len(kus_each) > 10:
            kus_p25[i] = float(np.percentile(kus_each, 25))
            kus_p75[i] = float(np.percentile(kus_each, 75))

    return {
        "vx_mid": vx_mid,
        "kus_ols": kus_ols,
        "kus_p25": kus_p25,
        "kus_p75": kus_p75,
        "n_pts": n_pts,
    }


# ---------------------------------------------------------------------------
# plotly 図生成
# ---------------------------------------------------------------------------
def _kus_step_profile(vx: np.ndarray, params: dict) -> np.ndarray:
    """params から N 段ステップの k_us プロファイルを計算して返す。

    新形式: k_us_bands (list) + k_us_thresholds (list) を使用。
    後方互換: k_us_lo / k_us_vx_thresh / k_us_mid / k_us_vx_thresh2 を自動変換。
    """
    # 新形式 k_us_bands / k_us_thresholds
    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")

    # 後方互換: k_us_lo / k_us_mid 形式から変換
    if bands is None and "k_us_lo" in params:
        thresh1 = params.get("k_us_vx_thresh", 0.0)
        if thresh1 > 0.0:
            thresh2 = params.get("k_us_vx_thresh2", 0.0)
            if "k_us_mid" in params and thresh2 > thresh1:
                bands = [params["k_us_lo"], params["k_us_mid"], params.get("k_us", 0.0)]
                thresholds = [thresh1, thresh2]
            else:
                bands = [params["k_us_lo"], params.get("k_us", 0.0)]
                thresholds = [thresh1]

    if bands is not None and thresholds is not None and len(bands) > 0:
        result = np.full_like(vx, bands[-1], dtype=float)
        for i, thr in enumerate(thresholds):
            result = np.where(vx < thr, bands[i], result)
        return result

    # レガシー: 単一 k_us（ランプなし）
    k_us = params.get("k_us", 0.0)
    return np.full_like(vx, k_us, dtype=float)


def _kus_band_label(params: dict) -> str:
    """速度帯パラメータを凡例文字列に変換。"""
    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")

    if bands is None and "k_us_lo" in params:
        thresh1 = params.get("k_us_vx_thresh", 0.0)
        if thresh1 > 0.0:
            thresh2 = params.get("k_us_vx_thresh2", 0.0)
            if "k_us_mid" in params and thresh2 > thresh1:
                bands = [params["k_us_lo"], params["k_us_mid"], params.get("k_us", 0.0)]
                thresholds = [thresh1, thresh2]
            else:
                bands = [params["k_us_lo"], params.get("k_us", 0.0)]
                thresholds = [thresh1]

    if bands is not None and thresholds is not None:
        parts = []
        for i, b in enumerate(bands):
            if i < len(thresholds):
                parts.append(f"<{thresholds[i]:.1f}m/s: {b:.4f}")
            else:
                parts.append(f"≥{thresholds[-1]:.1f}m/s: {b:.4f}")
        return " | ".join(parts)

    return f"k_us={params.get('k_us', 0.0):.4f}"


def build_kus_figure(bins: dict, params: dict) -> go.Figure:
    """k_us vs 速度の plotly 図（実測推定 + チューニング済み速度帯プロファイル重ね描き）。"""
    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=["k_us 推定値（速度ビン別 OLS）", "速度ビン別 曲線走行サンプル数"],
        horizontal_spacing=0.12,
    )

    vx_mid = bins["vx_mid"]
    kus_ols = bins["kus_ols"]
    kus_p25 = bins["kus_p25"]
    kus_p75 = bins["kus_p75"]
    n_pts = bins["n_pts"]
    valid = np.isfinite(kus_ols)

    # 実測 IQR バンド
    valid_iqr = valid & np.isfinite(kus_p25)
    if np.any(valid_iqr):
        fig.add_trace(go.Scatter(
            x=list(vx_mid[valid_iqr]) + list(vx_mid[valid_iqr][::-1]),
            y=list(kus_p25[valid_iqr]) + list(kus_p75[valid_iqr][::-1]),
            fill="toself", fillcolor="rgba(70,130,180,0.15)",
            line=dict(color="rgba(255,255,255,0)"),
            showlegend=True, name="25–75%ile（個別サンプル）",
        ), row=1, col=1)

    # OLS 推定値
    fig.add_trace(go.Scatter(
        x=vx_mid[valid].tolist(), y=kus_ols[valid].tolist(),
        mode="markers+lines",
        marker=dict(color="steelblue", size=7),
        line=dict(color="steelblue", width=2),
        name="OLS 推定 k_us(v)",
    ), row=1, col=1)

    # チューニング済み速度帯プロファイル（ステップ関数）
    vx_dense = np.linspace(0.0, 12.0, 600)
    kus_tune = _kus_step_profile(vx_dense, params)
    label = _kus_band_label(params)
    fig.add_trace(go.Scatter(
        x=vx_dense.tolist(), y=kus_tune.tolist(),
        mode="lines",
        line=dict(color="darkorange", width=2.5, dash="dash"),
        name=f"チューニング済み速度帯 ({label})",
    ), row=1, col=1)

    # k_us=0 水平線
    fig.add_hline(y=0.0, line=dict(color="gray", width=1, dash="dot"), row=1, col=1)

    # 閾値縦線
    thresholds = params.get("k_us_thresholds")
    if thresholds is None and "k_us_lo" in params:
        thresh1 = params.get("k_us_vx_thresh", 0.0)
        thresh2 = params.get("k_us_vx_thresh2", 0.0)
        if thresh1 > 0.0:
            thresholds = [thresh1]
            if thresh2 > thresh1:
                thresholds.append(thresh2)
    colors = ["green", "purple", "brown", "teal"]
    for i, thr in enumerate(thresholds or []):
        clr = colors[i % len(colors)]
        fig.add_vline(
            x=thr, line=dict(color=clr, width=1.2, dash="dash"),
            annotation_text=f"thr{i+1}={thr:.1f}", annotation_position="top right",
            row=1, col=1,
        )

    # サンプル数棒グラフ
    bin_widths = np.diff(VX_EDGES)
    fig.add_trace(go.Bar(
        x=vx_mid.tolist(), y=n_pts.tolist(),
        width=(bin_widths * 0.8).tolist(),
        marker_color="steelblue", opacity=0.6,
        name="サンプル数",
        showlegend=False,
    ), row=1, col=2)

    fig.update_xaxes(title_text="車速 vx [m/s]")
    fig.update_yaxes(title_text="k_us [rad·s²/m]", range=[-0.05, 0.12], row=1, col=1)
    fig.update_yaxes(title_text="サンプル数", row=1, col=2)
    fig.update_layout(
        height=430,
        title_text="実機ログからの k_us 独立同定（速度ビン別 OLS 回帰）",
        title_x=0.0,
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
        margin=dict(l=60, r=20, t=70, b=40),
    )
    return fig


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
            f"{label}（{phase14_score:.3f}）が baseline（k_us=0 / deadband=0, score={baseline_score:.3f}）より"
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
<summary>0-1. N-step 前向き積分誤差</summary>
<p>
車両モデルを実機ログの初期状態から N 個の制御コマンド区間だけ前向きに積分し、
実機の自己位置推定軌跡との終端誤差を評価する。
制御コマンドは 30 Hz（\\(\\Delta t = 1/30\\) 秒 \\(\\approx 33\\) ms）で記録されており、
ホライズン N=10（≈ 0.33 秒先）〜 N=40（≈ 1.33 秒先）の 4 点を等重みで集約する。
</p>
<table class="param-table">
  <tr><th>ホライズン</th><th>時間スパン（30 Hz 基準）</th><th>主に捉える現象</th></tr>
  <tr><td>N=10</td><td>≈ 0.33 秒先</td><td>アクチュエータ遅れ・1次遅れ時定数（即応性）</td></tr>
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
    <td>操舵時定数 τ_δ、k_us、ホイールベース</td>
  </tr>
  <tr>
    <td><b>long 誤差</b></td><td>cm</td>
    <td>進行方向の位置誤差（前後方向）。加速度の積算ズレを示す。</td>
    <td>加速度時定数 τ_a、遅延 T_a</td>
  </tr>
  <tr>
    <td><b>lat 誤差</b></td><td>cm</td>
    <td>横方向の位置誤差。操舵追従精度とアンダーステアの積算効果を示す。</td>
    <td>k_us、steer_dead_band、τ_δ、steer_bias</td>
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
各データセットの誤差を <b>baseline モデル</b>（補正なし delay モデル、k_us=0・deadband=0 相当）の
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
絶対誤差のまま集約すると、大カーブ・高速など「難しいシナリオ」（baseline 誤差が大きい DS）が
スコアを支配してしまい、全 650 DS で均等に改善できているかを測れない。
正規化により「baseline と比べてどれだけ改善したか」を全 DS で統一スケールで評価できる。
</p>
<table class="param-table">
  <tr><th>フロア定数</th><th>N=10</th><th>N=40</th><th>目的</th></tr>
  <tr><td>YAW_FLOOR</td><td>0.06 deg</td><td>0.24 deg</td>
    <td>低ダイナミクス（ほぼ直進）の DS で分母がゼロ近くになる暴発を防ぐ</td></tr>
  <tr><td>LONG_FLOOR</td><td>1.0 cm</td><td>4.5 cm</td><td>縦方向の同上</td></tr>
  <tr><td>LAT_FLOOR</td><td>0.3 cm</td><td>1.2 cm</td><td>横方向の同上</td></tr>
</table>
</details>

<details>
<summary>0-4. mean と worst</summary>
<p>
650 DS の正規化スコアに対して 2 種類の集約を行う:
</p>
<ul>
  <li><b>mean</b>: 全 DS の平均。「全体的に良い設定」を測る。</li>
  <li><b>worst</b>: 全 DS の最大値（最悪ケース）。「どのシナリオでも崩れない頑健性」を測る。</li>
</ul>
<p>
mean だけを最小化すると、一部の DS に特化したパラメータが選ばれ worst が悪化することがある。
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
ここで \\(\\overline{\\cdot}\\) は mean、\\(\\hat{\\cdot}\\) は worst（全 DS の max）。
<b>スコアは小さいほど良い。</b>
</p>
<ul>
  <li>yaw の重み = 1（位置の重み 0.5 + 0.5 = 1 と均等）</li>
  <li>long と lat は各 0.5 倍（yaw : 位置 = 1 : 1 を維持）</li>
  <li>worst 項の重み 0.5 = 「mean の改善と worst の頑健性を半々で重視」</li>
__SCORE_BULLET__
</ul>
<div class="note">
<b>直感的なスケール</b>: score が 1 下がると「全 650 DS・全 4 ホライズンで平均的に
nyaw が 1/8 改善した」相当（sum over 4 horizons × 2 terms (mean+0.5worst) でほぼ 8 で割る）。
__NOTE_TEXT__
</div>
</details>
</section>
"""
    return tmpl.replace("__SCORE_BULLET__", score_bullet).replace("__NOTE_TEXT__", note_text)


def _kus_band_table_rows(params: dict) -> str:
    """k_us 速度帯パラメータの HTML テーブル行を生成。"""
    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")

    if bands is None and "k_us_lo" in params:
        thresh1 = params.get("k_us_vx_thresh", 0.0)
        if thresh1 > 0.0:
            thresh2 = params.get("k_us_vx_thresh2", 0.0)
            if "k_us_mid" in params and thresh2 > thresh1:
                bands = [params["k_us_lo"], params["k_us_mid"], params.get("k_us", 0.0)]
                thresholds = [thresh1, thresh2]
            else:
                bands = [params["k_us_lo"], params.get("k_us", 0.0)]
                thresholds = [thresh1]

    if bands is not None and thresholds is not None:
        rows = []
        for i, b in enumerate(bands):
            if i == 0:
                speed_range = f"vx &lt; {thresholds[0]:.2f} m/s"
            elif i < len(thresholds):
                speed_range = f"{thresholds[i-1]:.2f} ≤ vx &lt; {thresholds[i]:.2f} m/s"
            else:
                speed_range = f"vx ≥ {thresholds[-1]:.2f} m/s"
            rows.append(
                f"  <tr><td><code>k_us_band[{i}]</code></td>"
                f"<td>{b:.6f} rad·s²/m</td>"
                f"<td>{speed_range} の k_us_eff</td></tr>"
            )
        for i, thr in enumerate(thresholds):
            rows.append(
                f"  <tr><td><code>k_us_threshold[{i}]</code></td>"
                f"<td>{thr:.3f} m/s</td>"
                f"<td>速度帯 {i} → {i+1} の切替閾値</td></tr>"
            )
        return "\n".join(rows)

    k_us = params.get("k_us", 0.0)
    return f"  <tr><td><code>k_us</code></td><td>{k_us:.6f} rad·s²/m</td><td>アンダーステア係数（速度依存なし）</td></tr>"


def _build_sec_model_intro(params: dict, label: str, params_filename: str = "") -> str:
    """レポート冒頭: モデルパラメータ一覧テーブル。"""

    def _fmt(v) -> str:
        if isinstance(v, float):
            return f"{v:.6g}"
        return str(v)

    # k_us 速度帯プロファイル行
    thresh1 = params.get("k_us_vx_thresh", 0.0)
    thresh2 = params.get("k_us_vx_thresh2", 0.0)
    if thresh1 > 0 or thresh2 > 0:
        kus_profile_rows = f"""\
  <tr><td><code>k_us_lo</code></td><td>{_fmt(params.get('k_us_lo', 0.0))}</td>
    <td>アンダーステア係数（vx &lt; {_fmt(thresh1)} m/s）</td></tr>
  <tr><td><code>k_us_mid</code></td><td>{_fmt(params.get('k_us_mid', 0.0))}</td>
    <td>アンダーステア係数（{_fmt(thresh1)} ≤ vx &lt; {_fmt(thresh2)} m/s）</td></tr>
  <tr><td><code>k_us</code></td><td>{_fmt(params.get('k_us', 0.0))}</td>
    <td>アンダーステア係数（vx ≥ {_fmt(thresh2)} m/s）</td></tr>
  <tr><td><code>k_us_vx_thresh</code></td><td>{_fmt(thresh1)} m/s</td>
    <td>k_us_lo → k_us_mid の切り替え速度</td></tr>
  <tr><td><code>k_us_vx_thresh2</code></td><td>{_fmt(thresh2)} m/s</td>
    <td>k_us_mid → k_us の切り替え速度</td></tr>"""
    else:
        kus_profile_rows = f"""\
  <tr><td><code>k_us</code></td><td>{_fmt(params.get('k_us', 0.0))}</td>
    <td>アンダーステア係数（全速度域）</td></tr>"""

    score = params.get("_score", "N/A")
    score_str = f"{float(score):.4f}" if str(score) != "N/A" else "N/A"
    src_str = f"<code>{params_filename}</code>" if params_filename else f"<code>{label}</code>"

    return f"""
<section id="model-intro">
<h2>モデルパラメータ一覧</h2>
<p>本レポートで検証するモデル（<b>{label}</b>）のパラメータを示す。
ソースファイル: {src_str} &nbsp;|&nbsp; スコア（steer_score）: <b>{score_str}</b></p>

<h3>アンダーステア補正（k_us 速度帯ステップ）</h3>
<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>説明</th></tr>
{kus_profile_rows}
</table>

<h3>操舵系</h3>
<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>説明</th></tr>
  <tr><td><code>steer_time_constant</code></td><td>{_fmt(params.get('steer_time_constant', 'N/A'))} s</td>
    <td>操舵アクチュエータ 1 次遅れ時定数 τ_δ</td></tr>
  <tr><td><code>steer_time_delay</code></td><td>{_fmt(params.get('steer_time_delay', 'N/A'))} s</td>
    <td>操舵指令の純粋遅延 T_δ</td></tr>
  <tr><td><code>steer_bias</code></td><td>{_fmt(params.get('steer_bias', 'N/A'))} rad</td>
    <td>系統的操舵オフセット β（左右対称バイアス補正）</td></tr>
  <tr><td><code>steer_rate_lim</code></td><td>{_fmt(params.get('steer_rate_lim', 'N/A'))} rad/s</td>
    <td>操舵レート制限（飽和速度）</td></tr>
  <tr><td><code>steer_dead_band</code></td><td>{_fmt(params.get('steer_dead_band', 'N/A'))} rad</td>
    <td>操舵不感帯幅（±dead_band 以内の指令は無視）</td></tr>
  <tr><td><code>debug_steer_scaling_factor</code></td><td>{_fmt(params.get('debug_steer_scaling_factor', 'N/A'))}</td>
    <td>操舵指令スケーリング係数（1.0 = 補正なし）</td></tr>
</table>

<h3>加速度系</h3>
<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>説明</th></tr>
  <tr><td><code>acc_time_constant</code></td><td>{_fmt(params.get('acc_time_constant', 'N/A'))} s</td>
    <td>加速度アクチュエータ 1 次遅れ時定数 τ_a</td></tr>
  <tr><td><code>acc_time_delay</code></td><td>{_fmt(params.get('acc_time_delay', 'N/A'))} s</td>
    <td>加速度指令の純粋遅延 T_a</td></tr>
</table>
</section>
"""


def _build_sec1(params: dict) -> str:
    kus_rows = _kus_band_table_rows(params)

    return f"""
<section id="theory">
<h2>1. モデル式要素の理論的説明</h2>

<details>
<summary>1-1. 速度依存アンダーステア補正（k_us 速度帯ステップ）</summary>
<p>
横方向の運動方程式（運動学的自転車モデル拡張）:
\\[
\\omega = \\frac{{v \\cdot \\tan(\\delta + \\beta)}}{{L + k_{{\\mathrm{{us,eff}}}} \\cdot v^2}},
\\qquad \\dot{{\\theta}} = \\omega
\\]
アンダーステア係数は速度帯ごとに定数を切り替えるステップ形状をとる:
\\[
k_{{\\mathrm{{us,eff}}}}(v) = k_{{\\mathrm{{us,band[i]}}}} \\quad \\text{{where }} i = \\max\\{{j : v \\geq \\mathrm{{threshold}}_j\\}}
\\]
</p>
<p><b>物理的意味</b>:
低速では慣性力が小さくタイヤ横力は無視できる（純運動学：\\(k_{{\\mathrm{{us,eff}}}}\\approx0\\)）。
高速になるとタイヤの横すべり剛性に対して慣性力が大きくなり、旋回半径が広がる
<em>アンダーステア</em>が現れる。分母の \\(k_{{\\mathrm{{us}}}} v^2\\) が大回り量を補正する。
速度帯ごとに異なる係数を割り当てることで、速度域別の特性差を表現する。
</p>
<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>意味</th></tr>
{kus_rows}
</table>
</details>
</section>
"""


def _build_sec2(kus_fig: go.Figure, n_ds: int) -> str:
    kus_html = kus_fig.to_html(full_html=False, include_plotlyjs=False)
    return f"""
<section id="identification">
<h2>2. 実機ログからの独立同定</h2>

<details open>
<summary>2-1. アンダーステア係数 k_us の速度依存性（全 {n_ds} データセット）</summary>
<details>
<summary>推定手法の詳細</summary>
<p>
定常旋回フィルタ（\\(|\\omega| > {WZ_MIN}\\) rad/s、\\(|\\dot{{\\omega}}| < {DWZ_MAX}\\) rad/s²、
\\(v_x > {VX_MIN_CURVE}\\) m/s）を通過した各タイムステップを速度ビンに割り当て、
ビン内で以下の2種類の推定を行う。
</p>
<p><b>① 最小二乗法推定（青丸・実線）</b>: 原点回帰 \\(\\tan(\\delta_{{\\mathrm{{eff}}}}) = C \\cdot \\omega\\) の最小二乗解
\\[
C_{{\\mathrm{{OLS}}}} = \\frac{{\\sum \\omega_i \\, \\tan(\\delta_i)}}{{\\sum \\omega_i^2}},
\\qquad
\\hat{{k}}_{{\\mathrm{{us}}}} = \\frac{{C_{{\\mathrm{{OLS}}}} - L / \\bar{{v}}_x}}{{\\bar{{v}}_x}}
\\]
ここで \\(\\bar{{v}}_x\\) はビン内の速度中央値、\\(L = {WHEELBASE}\\) m はホイールベース。
</p>
<p><b>② 個別サンプル（IQR バンド）</b>: 実機運動学ログの各タイムステップで瞬時 k_us を推定し、
ビン内の 25〜75 パーセンタイルをバンドとして表示:
\\[
\\tilde{{k}}_{{\\mathrm{{us}}}}[i] = \\frac{{\\tan(\\delta_i) / \\omega_i - L / v_{{x,i}}}}{{v_{{x,i}}}}
\\]
チューニング済みランプ曲線（橙色破線）と重ね描きして形状の妥当性を確認する。
</p>
</details>
{kus_html}
<div class="note">
<b>解釈</b>: 最小二乗法推定値が低速ビンでほぼ 0、高速ビンで正の値に推移していれば、
ランプ形状は物理的実態と整合している。ただし J6 の多くの DS が低速（vx_mean ≈ 1.9 m/s）
のため、高速ビンのサンプル数は少なく推定誤差が大きい点に注意（右パネルのサンプル数を参照）。
</div>
</details>
</section>
"""


def _build_sec3(viewer_sections: list[str], label: str = "phase14") -> str:
    body = "\n".join(viewer_sections) if viewer_sections else "<p>ビューア生成対象 DS なし</p>"
    return f"""
<section id="curve-viewer">
<h2>3. カーブ部での実機 vs モデル軌跡（インタラクティブビューア）</h2>
<p>
旋回イベント数 <code>curve_count</code>（\\(|\\kappa| > 0.02\\) m⁻¹、連続弧長 ≥ 10 m）が多い
代表データセットについて縦横モデル検証ビューアを埋め込む。
ドロップダウンで <b>{label}</b>（k_us ランプ＋deadband 有効）と
<b>baseline</b>（k_us=0, deadband=0）を切り替えて実機軌跡への一致を比較できる。
</p>
<div class="note">
⚠️ <code>debug_steer_scaling_factor</code> は JS モデル非対応（ステア挙動は近似）。ビューアは探索ツールとして利用すること。
</div>
{body}
</section>
"""


def _build_sec_deviation(
    df: pd.DataFrame,
    n_ds: int,
    recomputed_score: float | None = None,
    expected_score: float | None = None,
    score_name: str = "robust_score",
    label: str = "phase14",
) -> str:
    """N-step 終端誤差の最大乖離テーブルセクション（tuned vs baseline）。"""
    horizons = sorted(df["h"].unique().tolist())

    stats_list = []
    for h in horizons:
        sub = df[df["h"] == h]
        stats_list.append({
            "h": h,
            "p14_yaw_mean": sub["p14_yaw"].mean(), "p14_yaw_max": sub["p14_yaw"].max(),
            "p14_lat_mean": sub["p14_lat"].mean(), "p14_lat_max": sub["p14_lat"].max(),
            "p14_long_mean": sub["p14_long"].mean(), "p14_long_max": sub["p14_long"].max(),
            "p14_vx_mean": sub["p14_vx"].mean(), "p14_vx_max": sub["p14_vx"].max(),
            "bl_yaw_mean": sub["bl_yaw"].mean(), "bl_yaw_max": sub["bl_yaw"].max(),
            "bl_lat_mean": sub["bl_lat"].mean(), "bl_lat_max": sub["bl_lat"].max(),
            "bl_long_mean": sub["bl_long"].mean(), "bl_long_max": sub["bl_long"].max(),
            "bl_vx_mean": sub["bl_vx"].mean(), "bl_vx_max": sub["bl_vx"].max(),
        })

    score_html = ""
    if recomputed_score is not None and expected_score is not None:
        diff_pct = abs(recomputed_score - expected_score) / expected_score * 100
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
            f'{_cell(s["p14_yaw_max"], s["bl_yaw_max"])}\n'
            f'  {_cell(s["p14_lat_mean"], s["bl_lat_mean"])}'
            f'{_cell(s["p14_lat_max"], s["bl_lat_max"])}\n'
            f'  {_cell(s["p14_long_mean"], s["bl_long_mean"])}'
            f'{_cell(s["p14_long_max"], s["bl_long_max"])}\n'
            f'  {_cell(s["p14_vx_mean"], s["bl_vx_mean"], ".4f")}'
            f'{_cell(s["p14_vx_max"], s["bl_vx_max"], ".4f")}\n'
            f'</tr>\n'
            f'<tr>\n'
            f'  <td style="color:#888">baseline</td>\n'
            f'  <td>{s["bl_yaw_mean"]:.3f}</td><td>{s["bl_yaw_max"]:.3f}</td>\n'
            f'  <td>{s["bl_lat_mean"]:.3f}</td><td>{s["bl_lat_max"]:.3f}</td>\n'
            f'  <td>{s["bl_long_mean"]:.3f}</td><td>{s["bl_long_max"]:.3f}</td>\n'
            f'  <td>{s["bl_vx_mean"]:.4f}</td><td>{s["bl_vx_max"]:.4f}</td>\n'
            f'</tr>'
        )

    tbody = "\n".join(tbody_rows)

    return f"""
<section id="deviation">
<h2>0-6. 全 {n_ds} DS の N-step 終端誤差（{label} vs baseline）</h2>
<p>
全データセットに対し {label} パラメータと baseline（補正なし）で N-step ロールアウトを実施し、
終端誤差 RMSE の DS 横断 <b>平均</b>（mean）と <b>最大</b>（worst-case DS）を N ごとに集計する。
「最大」は「最も誤差が大きかった DS の RMSE」を指す（単一ステップの瞬間値ではない）。
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
      <th colspan="2">速度誤差 vx [m/s]</th>
    </tr>
    <tr>
      <th>平均</th><th>最大</th>
      <th>平均</th><th>最大</th>
      <th>平均</th><th>最大</th>
      <th>平均</th><th>最大</th>
    </tr>
  </thead>
  <tbody>
{tbody}
  </tbody>
</table>
<div class="note">
{label} の値が baseline より小さい場合は <b style="color:#28a745">緑（改善）</b>、
大きい場合は <span style="color:#dc3545">赤（悪化）</span> で表示。
RMSE は各 DS の全 k0 ステップ（stride=5）の終端誤差（N ステップ先）の二乗平均平方根。
キャッシュは <code>--metrics-cache</code> で指定した CSV ファイルに保存される。
</div>
</section>
"""


def build_html(
    params: dict,
    kus_fig: go.Figure,
    viewer_sections: list[str],
    n_ds: int,
    baseline_score: float | None = None,
    deviation_html: str = "",
    label: str = "current",
    params_filename: str = "",
) -> str:
    score = params.get("_score", "N/A")
    phase14_score = float(score) if isinstance(score, (int, float, str)) and str(score) != "N/A" else 0.0
    sec_intro = _build_sec_model_intro(params, label=label, params_filename=params_filename)
    sec0 = _build_sec_metrics(baseline_score=baseline_score, phase14_score=phase14_score, label=label)
    sec1 = _build_sec1(params)
    sec2 = _build_sec2(kus_fig, n_ds)
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
  有効データセット数: {n_ds}
</p>
<nav>
  <a href="#model-intro">モデルパラメータ</a>
  <a href="#metrics">0. メトリクス解説</a>
  {'<a href="#deviation">0-6. N-step 最大乖離</a>' if deviation_html else ""}
  <a href="#theory">1. 理論</a>
  <a href="#identification">2. 実機同定</a>
  <a href="#curve-viewer">3. カーブビューア</a>
</nav>
{sec_intro}
{sec0}
{deviation_html}
{sec1}
{sec2}
{sec3}
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
        "--params", type=Path,
        default=Path("/home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_june_phase14.yaml"),
    )
    ap.add_argument(
        "--collection-dir", type=Path,
        default=Path("/home/kotaroyoshimoto/data/openloop_j6_15_june"),
    )
    ap.add_argument(
        "--out", type=Path,
        default=Path("/home/kotaroyoshimoto/data/openloop_j6_15/physical_validity_report.html"),
    )
    ap.add_argument("--n-curve-ds", type=int, default=3, help="ビューア埋め込みカーブ DS 数")
    ap.add_argument("--n-jobs", type=int, default=8)
    ap.add_argument(
        "--viewer-uuids", type=str, default=None,
        help="ビューアに使う DS UUID をカンマ区切りで指定（省略時は curve_count 上位を自動選択）",
    )
    ap.add_argument(
        "--pinned-uuids", type=str, default="ce063d43",
        help="ビューアに必ず含める DS UUID をカンマ区切りで指定（前方一致）。--viewer-uuids や自動選択より優先して先頭に配置",
    )
    ap.add_argument(
        "--metrics-cache", type=Path, default=None,
        help=(
            "rollout メトリクス CSV キャッシュパス（指定時のみ偏差テーブルを生成）。"
            "ファイルが存在すれば読み込み、なければ全 DS rollout を実行して保存する。"
        ),
    )
    ap.add_argument(
        "--extra-ds", type=Path, nargs="*", default=[],
        help="collection-dir 外から MCAP 解析・ビューアに追加する DS ディレクトリ（複数指定可）",
    )
    ap.add_argument(
        "--label", type=str, default="current",
        help="レポート内のモデル名ラベル（デフォルト: current）",
    )
    args = ap.parse_args()

    phase_label = args.label

    with open(args.params) as f:
        yaml_data = yaml.safe_load(f)
    params: dict = yaml_data.get("params", yaml_data)
    params["_score"] = yaml_data.get("score", "N/A")
    print(f"パラメータ: {args.params.name}  (label={phase_label})")
    # k_us 速度帯表示（新形式 / 後方互換形式 両対応）
    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")
    if bands is None and "k_us_lo" in params:
        thresh1 = params.get("k_us_vx_thresh", 0.0)
        thresh2 = params.get("k_us_vx_thresh2", 0.0)
        if thresh1 > 0.0:
            if "k_us_mid" in params and thresh2 > thresh1:
                bands = [params["k_us_lo"], params["k_us_mid"], params.get("k_us", 0.0)]
                thresholds = [thresh1, thresh2]
            else:
                bands = [params["k_us_lo"], params.get("k_us", 0.0)]
                thresholds = [thresh1]
    if bands is not None and thresholds is not None:
        band_str = " | ".join(
            f"band[{i}]={b:.5f}" for i, b in enumerate(bands)
        )
        thr_str = " | ".join(f"thr[{i}]={t:.2f}" for i, t in enumerate(thresholds))
        print(f"  k_us 速度帯: {band_str}")
        print(f"  閾値: {thr_str} m/s")
    else:
        print(f"  k_us={params.get('k_us', 0):.5f} (速度依存なし)")
    print(f"  steer_dead_band={params.get('steer_dead_band',0):.5f} rad")

    # データセット列挙
    ds_list = _discover(args.collection_dir)
    for extra in (args.extra_ds or []):
        uuid = extra.name
        if not any(u == uuid for u, _ in ds_list):
            ds_list.append((uuid, extra))
            print(f"  [extra-ds] {uuid} を追加")
    print(f"\nデータセット: {len(ds_list)} 件")

    # Phase 1: 並列 MCAP 読み込み
    print("\n[Phase 1] MCAP 並列読み込み ...")
    records = load_all_mcap(ds_list, n_jobs=args.n_jobs)
    print(f"  有効: {len(records)} 件")

    # Phase 2: k_us 速度ビン別 OLS
    print("\n[Phase 2] k_us 速度ビン別 OLS ...")
    bins = compute_kus_bins(records)
    n_valid = int(np.isfinite(bins["kus_ols"]).sum())
    print(f"  有効速度ビン: {n_valid}/{len(bins['kus_ols'])}")

    # Phase 3: カーブ多 DS 選定
    print("\n[Phase 3] カーブ DS 選定 ...")
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
    if args.metrics_cache and args.metrics_cache.exists():
        # キャッシュあり → viewer DS のみ load、メトリクスは CSV から読む
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} DS) ...")
        ctxs = load_datasets(top_items, n_jobs=min(args.n_jobs, len(top_items)))
        print(f"\n[Phase 3c] rollout メトリクスキャッシュ読み込み ...")
        df_rollout = pd.read_csv(args.metrics_cache)
        n_ds_cache = df_rollout["uuid"].nunique()
        n_h_cache = df_rollout["h"].nunique()
        print(f"  {len(df_rollout)} 行（{n_ds_cache} DS × {n_h_cache} horizons）")
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
        expected = float(yaml_data.get("score", 0.0))
        candidates = [
            ("robust_score", _robust_score(agg)),
            ("steer_score",  _steer_score(agg)),
            ("acc_score",    _acc_score(agg)),
        ]
        best_name, recomputed = min(candidates, key=lambda kv: abs(kv[1] - expected))
        diff_pct = abs(recomputed - expected) / expected * 100 if expected else float("inf")
        print(f"  再現スコア: {recomputed:.4f} ({best_name})  期待値: {expected:.4f}  差: {diff_pct:.2f}%")
        # baseline (k_us=0) の steer_score を計算
        baseline_steer_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline steer_score: {baseline_steer_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    elif args.metrics_cache:
        # キャッシュなし → 全 DS load（ついでに viewer DS も取り出す）
        all_items = [(r["uuid"], Path(r["lite_dir"])) for r in records]
        print(f"\n[Phase 3b+3c] 全 DS DatasetCtx 構築 ({len(all_items)} DS) + rollout メトリクス計算 ...")
        all_ctxs = load_datasets(all_items, n_jobs=args.n_jobs)
        # viewer DS を all_ctxs から抽出（重複 load 回避）
        ctxs_by_id = {c.dataset_id: c for c in all_ctxs}
        ctxs = [ctxs_by_id[r["uuid"]] for r in top_curve if r["uuid"] in ctxs_by_id]
        # phase14 override: YAML の全 params から _* メタキーを除外（hand-pick より安全）
        override = {k: v for k, v in params.items() if not k.startswith("_")}
        rows = []
        for i, ctx in enumerate(all_ctxs, 1):
            try:
                p14 = _tune_eval(ctx, override, _BASELINE_MODEL)
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
        expected = float(yaml_data.get("score", 0.0))
        # YAML の score は tuning --phase に応じて steer/acc/robust のいずれかなので最接近を選択
        candidates = [
            ("robust_score", _robust_score(agg)),
            ("steer_score",  _steer_score(agg)),
            ("acc_score",    _acc_score(agg)),
        ]
        best_name, recomputed = min(candidates, key=lambda kv: abs(kv[1] - expected))
        diff_pct = abs(recomputed - expected) / expected * 100 if expected else float("inf")
        print(f"  再現スコア: {recomputed:.4f} ({best_name})  期待値: {expected:.4f}  差: {diff_pct:.2f}%")
        # baseline (k_us=0) の steer_score を計算
        baseline_steer_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline steer_score: {baseline_steer_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    else:
        baseline_steer_score = None
        # --metrics-cache 未指定 → 通常の viewer DS のみ load
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} DS) ...")
        ctxs = load_datasets(top_items, n_jobs=min(args.n_jobs, len(top_items)))

    curve_count_map = {r["uuid"]: r["curve_count"] for r in top_curve}

    # tuned 設定と baseline 設定（configs = {ラベル: override_params}）
    tuned_keys = [
        "k_us", "k_us_vx_lo", "k_us_vx_hi",
        "k_us_lo", "k_us_mid", "k_us_vx_thresh", "k_us_vx_thresh2",
        "steer_dead_band", "steer_bias",
        "steer_time_constant", "steer_time_delay",
        "acc_time_constant", "acc_time_delay",
        "debug_steer_scaling_factor", "steer_rate_lim",
    ]
    configs: dict[str, dict] = {
        f"{phase_label}（k_us ランプ＋deadband）": {
            k: params[k] for k in tuned_keys if k in params
        },
        "baseline（k_us=0 / deadband=0）": {
            "k_us": 0.0, "k_us_vx_lo": 0.0, "k_us_vx_hi": 0.0, "steer_dead_band": 0.0,
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
    kus_fig = build_kus_figure(bins, params)
    html = build_html(
        params, kus_fig, viewer_sections, len(records),
        baseline_score=baseline_steer_score,
        deviation_html=deviation_html,
        label=phase_label,
        params_filename=args.params.name,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding="utf-8")
    size_kb = args.out.stat().st_size // 1024
    print(f"\n✓ 完了: {args.out}  ({size_kb} KB)")


if __name__ == "__main__":
    main()
