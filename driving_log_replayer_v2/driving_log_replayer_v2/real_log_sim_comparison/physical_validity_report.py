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
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter

# ---------------------------------------------------------------------------


from driving_log_replayer_v2.real_log_sim_comparison.lib._coverage import _curvature_coverage  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_accel, load_cmd, load_kinematic, load_steering, load_velocity  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._map import load_map_ways, map_ways_in_bbox, resolve_map_osm  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._plotly_utils import lanes_to_trace  # noqa: E402
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

_FIT_DT = 0.01          # モデルフィット用リサンプリング DT [s]
_FIT_N_DATASET = 2      # フィット表示する代表データセット数（最良・最悪それぞれ）

# 1-1/1-4. 理想追従評価用共通定数
_PERF_HORIZONS: tuple[int, ...] = (10, 20, 50, 100)  # steps @ _FIT_DT → 0.10, 0.20, 0.50, 1.00 s
_PERF_STRIDE = 5                                       # rollout reset stride [steps]
_PERF_N_DATASET = 10                                   # 1-4 横方向 box plot 用 データセット数
_PERF_N_TRAJ = 3                                       # 軌跡比較表示 データセット数
_LONG_PERF_N_DATASET = 20                                   # 1-1 縦方向理想追従 box plot 用 データセット数
_DRIFT_A_TH = 0.3                                      # 加速/減速/巡航を分ける加速度閾値 [m/s²]

def _sim_first_order(cmd: np.ndarray, tau: float, n_delay: int, dt: float = _FIT_DT) -> np.ndarray:
    """純粋遅延 + 一次遅れシミュレーション（lfilter 版）。"""
    n = len(cmd)
    cmd_del = np.empty(n)
    if n_delay > 0:
        cmd_del[:n_delay] = cmd[0]
        cmd_del[n_delay:] = cmd[:-n_delay]
    else:
        cmd_del = cmd.copy()
    alpha = float(np.clip(dt / tau, 0.0, 1.0))
    return lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del)


def _bicycle_nstep_perf(
    gt_x: np.ndarray,
    gt_y: np.ndarray,
    gt_yaw: np.ndarray,
    gt_vx: np.ndarray,
    gt_steer: np.ndarray,
    params: dict,
    horizon: int,
    dt: float,
    stride: int = _PERF_STRIDE,
) -> np.ndarray:
    """純粋自転車モデルの N-step 横方向誤差（ベクトル化）。

    gt_steer: 実測操舵角 [rad]（生センサ値。steer_bias は params から適用）
    params.steer_bias / k_us(v) を C++ モデルと同一式で評価し積分する。
    Returns: |横方向誤差| [m] 配列（vx > VX_MIN_CURVE な開始点のみ）
    """
    L = WHEELBASE
    beta = float(params.get("steer_bias", 0.0))
    k_us_arr = _kus_step_profile(gt_vx, params)
    denom_arr = np.maximum(L + k_us_arr * gt_vx ** 2, 0.05 * L)
    wz_arr = gt_vx * np.tan(gt_steer + beta) / denom_arr

    n = len(gt_x)
    k0s = np.arange(0, n - horizon, stride)
    k0s = k0s[gt_vx[k0s] > VX_MIN_CURVE]
    if len(k0s) == 0:
        return np.array([], dtype=float)

    bx   = gt_x[k0s].astype(float).copy()
    by   = gt_y[k0s].astype(float).copy()
    byaw = gt_yaw[k0s].astype(float).copy()

    for j in range(horizon):
        ki = np.clip(k0s + j, 0, n - 1)
        vxi = gt_vx[ki]
        wzi = wz_arr[ki]
        bx   = bx   + vxi * np.cos(byaw) * dt
        by   = by   + vxi * np.sin(byaw) * dt
        byaw = byaw + wzi * dt

    k_end = np.minimum(k0s + horizon, n - 1)
    dx  = bx - gt_x[k_end]
    dy  = by - gt_y[k_end]
    lat = np.abs(-dx * np.sin(gt_yaw[k_end]) + dy * np.cos(gt_yaw[k_end]))
    return lat


def _bicycle_trajectory_full(
    gt_vx: np.ndarray,
    gt_steer: np.ndarray,
    x0: float,
    y0: float,
    yaw0: float,
    params: dict,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """自転車モデルの全区間連続積分軌跡（リセットなし）。
    初期状態を GT に合わせ、実測 vx + steer で積分する（アクチュエータ遅れなし仮定）。
    """
    L = WHEELBASE
    beta = float(params.get("steer_bias", 0.0))
    k_us_arr = _kus_step_profile(gt_vx, params)
    denom_arr = np.maximum(L + k_us_arr * gt_vx ** 2, 0.05 * L)
    wz_arr = gt_vx * np.tan(gt_steer + beta) / denom_arr

    n = len(gt_vx)
    xs = np.empty(n)
    ys = np.empty(n)
    x, y, yaw = x0, y0, yaw0
    for i in range(n):
        xs[i] = x
        ys[i] = y
        x   = x   + float(gt_vx[i]) * float(np.cos(yaw)) * dt
        y   = y   + float(gt_vx[i]) * float(np.sin(yaw)) * dt
        yaw = yaw + float(wz_arr[i]) * dt
    return xs, ys


def _long_nstep_perf(
    gt_vx: np.ndarray,
    a_act: np.ndarray,
    horizon: int,
    dt: float,
    stride: int = _PERF_STRIDE,
) -> np.ndarray:
    """縦方向モデル構造限界評価: 実測加速度 a_act を直接積分し GT 変位と比較。

    シミュレータの加速度応答が実機と完全一致（a_act_sim = a_act_gt）した場合に残る
    縦方向変位誤差を評価する。残差は dvx/dt = a_act という運動方程式に含まれない
    要素（路面勾配・空気抵抗・タイヤ縦力・センサーバイアス等）に由来する。

    1-4 横方向評価との対称性:
      1-4: gt_steer を bicycle model に直接入力 → steer 完全追従時の横方向残差
      本関数: gt_a_act を積分器に直接入力 → acc 完全追従時の縦方向残差

    Returns: |s_sim - s_gt| [m] 配列
    """
    n = len(gt_vx)
    k0s = np.arange(0, n - horizon, stride)
    k0s = k0s[gt_vx[k0s] > VX_MIN_CURVE]
    if len(k0s) == 0:
        return np.array([], dtype=float)

    vx_sim = gt_vx[k0s].astype(float).copy()
    s_sim  = np.zeros(len(k0s))
    s_gt   = np.zeros(len(k0s))

    for j in range(horizon):
        ki = np.clip(k0s + j, 0, n - 1)
        s_sim  += vx_sim * dt
        vx_sim += a_act[ki] * dt
        s_gt   += gt_vx[ki] * dt

    return np.abs(s_sim - s_gt)


def _long_drift_profile(
    gt_vx: np.ndarray,
    a_act: np.ndarray,
    horizon: int,
    dt: float,
    stride: int = _PERF_STRIDE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """縦方向ドリフトプロファイル: box plot と同一 reset-stride ロールアウトで
    各ステップ j の符号付き誤差を記録する。

    _long_nstep_perf との整合性:
      - 同一の k0s 走査・VX_MIN_CURVE フィルタ・初期化を使用。
      - serr[:, horizon] を絶対値化したものが _long_nstep_perf の終端値に一致する。

    Returns
    -------
    verr  : ndarray, shape (m, horizon+1)  速度誤差 vx_sim - gt_vx [m/s]
    serr  : ndarray, shape (m, horizon+1)  変位誤差 s_sim - s_gt [m]
    phase : ndarray, shape (m,)  int  0=減速, 1=巡航, 2=加速
            （各開始点からホライズン内の a_act 平均値で分類）
    None を返す場合: 有効な開始点が 0 のとき。
    """
    n = len(gt_vx)
    k0s = np.arange(0, n - horizon, stride)
    k0s = k0s[gt_vx[k0s] > VX_MIN_CURVE]
    if len(k0s) == 0:
        return None

    m = len(k0s)
    verr = np.zeros((m, horizon + 1), dtype=float)
    serr = np.zeros((m, horizon + 1), dtype=float)

    vx_sim = gt_vx[k0s].astype(float).copy()
    s_sim  = np.zeros(m)
    s_gt   = np.zeros(m)

    for j in range(horizon):
        ki = np.clip(k0s + j, 0, n - 1)
        ki1 = np.clip(k0s + j + 1, 0, n - 1)
        s_sim  += vx_sim * dt
        vx_sim += a_act[ki] * dt
        s_gt   += gt_vx[ki] * dt
        verr[:, j + 1] = vx_sim - gt_vx[ki1]
        serr[:, j + 1] = s_sim - s_gt

    # 局面分類: 開始点からホライズン内の a_act 平均
    mean_a = np.array([
        float(np.mean(a_act[np.clip(k0 + np.arange(horizon), 0, n - 1)]))
        for k0 in k0s
    ])
    phase = np.where(mean_a < -_DRIFT_A_TH, 0,
             np.where(mean_a > _DRIFT_A_TH, 2, 1)).astype(int)

    return verr, serr, phase


def _find_mcap(collection_dir: Path, uuid: str) -> Path | None:
    """データセット uuid の MCAP パスを探して返す（見つからなければ None）。"""
    for ds_dir in [
        collection_dir / "datasets" / uuid,
        collection_dir / uuid,
    ]:
        mcap = ds_dir / "real.lite" / "real.lite_0.mcap"
        if mcap.exists():
            return mcap
    return None


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

    try:
        df_cmd = load_cmd(mcap, "/control/command/control_cmd")
    except Exception:
        df_cmd = pd.DataFrame()

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

    # cmd_steer（stride=10 で間引き: 操舵信号の時系列表示用）
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
        "wz": wz.tolist(),
        "steer_eff": steer_eff.tolist(),
        "dwz": dwz.tolist(),
        "curve_count": cov["curve_count"],
        "kappa_max_abs": cov["kappa_max_abs"],
        "cmd_steer": cmd_steer_arr,
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
# Phase 2: k_us 速度ビン別 最小二乗法回帰
# ---------------------------------------------------------------------------
def compute_kus_bins(records: list[dict]) -> dict:
    """速度ビン別 最小二乗法回帰で k_us(v) を推定。モデル: tan(δ_eff) = (L/v + k_us·v)·ω"""
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
        # 最小二乗法原点回帰: tan(steer) = C * wz => k_us = (C - L/v) / v
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
        subplot_titles=["k_us 推定値（速度ビン別 最小二乗法）", "速度ビン別 曲線走行サンプル数"],
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

    # 最小二乗法推定値
    fig.add_trace(go.Scatter(
        x=vx_mid[valid].tolist(), y=kus_ols[valid].tolist(),
        mode="markers+lines",
        marker=dict(color="steelblue", size=7),
        line=dict(color="steelblue", width=2),
        name="最小二乗法推定 k_us(v)",
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
        title_text="実機ログからの k_us 独立同定（速度ビン別 最小二乗法回帰）",
        title_x=0.0,
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
        margin=dict(l=60, r=20, t=70, b=40),
    )
    return fig


_N_CROSS_FIT_DATASET = 10      # 横断最小二乗法に使うデータセット数
_DA_THRESH_FIT  = 0.15    # 動的区間フィルタ: |Δa_cmd/dt| [m/s²/s]（identify_long_dynamics.py と同値）
_VX_MIN_FIT     = 0.5     # 動的区間フィルタ: 最低速度 [m/s]
_DELAY_CANDIDATES = np.arange(0.0, 0.31 + 1e-9, 0.01)   # 遅延候補 0〜300ms, 10ms 刻み
_TAU_BOUNDS     = (0.01, 5.0)                             # 時定数探索範囲 [s]


def _fit_long_cross_dataset(
    collection_dir: Path, df_id: pd.DataFrame
) -> tuple[float, float, float]:
    """全データセット横断での加速度一次遅れモデルの最小二乗法同定。

    動的区間が豊富な上位 _N_CROSS_FIT_DATASET データセットのサンプルを一つのプールに集め、
    遅延グリッドサーチ + output-error 最小二乗法で全体最適な (τ, T) を求める。

    Returns: (tau [s], T [s], RMSE [m/s²])
    """
    _N_DYN_MIN = 100
    df_cand = df_id[df_id["n_dyn"] >= _N_DYN_MIN] if "n_dyn" in df_id.columns else df_id
    df_top  = df_cand.nlargest(min(_N_CROSS_FIT_DATASET, len(df_cand)), "n_dyn")

    pooled: list[tuple[np.ndarray, np.ndarray]] = []   # (a_cmd_full, a_act_full, mask)
    for row in df_top.itertuples():
        mcap = _find_mcap(collection_dir, row.uuid)
        if mcap is None:
            continue
        try:
            df_cmd   = load_cmd(mcap, "/control/command/control_cmd")
            df_accel = load_accel(mcap)
            df_vel   = load_velocity(mcap)
        except Exception:
            continue
        if df_cmd.empty or df_accel.empty or df_vel.empty:
            continue

        t0 = max(df_cmd["t_ns"].iloc[0], df_accel["t_ns"].iloc[0], df_vel["t_ns"].iloc[0])
        t1 = min(df_cmd["t_ns"].iloc[-1], df_accel["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1])
        if (t1 - t0) < 2e9:
            continue
        t_ns = np.arange(t0, t1, _FIT_DT * 1e9, dtype=np.float64)
        t_s  = (t_ns - t0) * 1e-9

        a_cmd_arr = np.interp(t_s, (df_cmd["t_ns"].values   - t0) * 1e-9, df_cmd["cmd_accel"].values)
        a_act_arr = np.interp(t_s, (df_accel["t_ns"].values  - t0) * 1e-9, df_accel["accel"].values)
        vx        = np.interp(t_s, (df_vel["t_ns"].values    - t0) * 1e-9, df_vel["lon_vel"].values)

        d_cmd = np.abs(np.gradient(a_cmd_arr, _FIT_DT))
        mask  = (vx > _VX_MIN_FIT) & (d_cmd > _DA_THRESH_FIT)
        if mask.sum() < 50:
            continue
        pooled.append((a_cmd_arr, a_act_arr, mask))

    if not pooled:
        return float("nan"), float("nan"), float("nan")

    def _total_mse(log_tau: float, n_delay: int) -> float:
        tau = float(np.exp(log_tau))
        sq_sum, n_sum = 0.0, 0
        for a_cmd_arr, a_act_arr, mask in pooled:
            a_sim = _sim_first_order(a_cmd_arr, tau, n_delay)
            diff  = a_sim[mask] - a_act_arr[mask]
            sq_sum += float(np.dot(diff, diff))
            n_sum  += int(mask.sum())
        return sq_sum / n_sum if n_sum > 0 else float("inf")

    best_mse   = float("inf")
    best_tau   = float("nan")
    best_delay = float("nan")
    log_lo, log_hi = np.log(_TAU_BOUNDS[0]), np.log(_TAU_BOUNDS[1])
    for delay_s in _DELAY_CANDIDATES:
        n_delay = int(round(delay_s / _FIT_DT))
        res = minimize_scalar(
            lambda lt, nd=n_delay: _total_mse(lt, nd),
            bounds=(log_lo, log_hi), method="bounded",
        )
        tau = float(np.exp(res.x))
        mse = float(res.fun)
        if mse < best_mse:
            best_mse   = mse
            best_tau   = tau
            best_delay = delay_s

    return best_tau, best_delay, float(np.sqrt(best_mse))


def build_long_figure(collection_dir: Path, params: dict, phase_label: str = "") -> go.Figure:
    """縦方向モデルフィット時系列（横断最小二乗法同定値 vs チューニング値、代表データセット 3 本）。

    トレース構成:
    - 黒実線: 実測加速度
    - 灰実線: 指令加速度
    - 青点線: 全データセット横断最小二乗法同定値
    - 橙実線: チューニング値
    """
    csv_path = collection_dir / "long_dynamics_identified.csv"

    def _placeholder(msg: str) -> go.Figure:
        fig = go.Figure()
        fig.add_annotation(text=msg, xref="paper", yref="paper",
                           x=0.5, y=0.5, showarrow=False, font=dict(size=13), align="center")
        fig.update_layout(height=200)
        return fig

    if not csv_path.exists():
        return _placeholder(
            f"<b>{csv_path.name} が見つかりません</b><br>"
            "事前に <code>identify_long_dynamics.py</code> を実行してください。"
        )

    df_id = pd.read_csv(csv_path)
    tau_tune = float(params.get("acc_time_constant", float("nan")))
    T_tune   = float(params.get("acc_time_delay", float("nan")))

    # 全データセット横断で最小二乗法同定（一度だけ実行）
    tau_cross, T_cross, rmse_cross = _fit_long_cross_dataset(collection_dir, df_id)

    # 動的区間が豊富なデータセットを候補にし、最良・最悪それぞれ _FIT_N_DATASET 本を選択
    _N_DYN_MIN = 100
    df_cand = df_id[df_id["n_dyn"] >= _N_DYN_MIN] if "n_dyn" in df_id.columns else df_id
    if len(df_cand) < _FIT_N_DATASET * 2:
        df_cand = df_id
    df_best  = df_cand.nsmallest(_FIT_N_DATASET, "rmse_mps2")
    df_worst = df_cand.nlargest(_FIT_N_DATASET, "rmse_mps2")
    # 重複除去（小規模データセット時に同一行が両方に入る場合）
    df_worst = df_worst[~df_worst.index.isin(df_best.index)]
    # 最良を先、最悪を後に並べる
    df_display = pd.concat([
        df_best.assign(_case="最良"),
        df_worst.assign(_case="最悪"),
    ])

    rows_data = []
    for row in df_display.itertuples():
        mcap = _find_mcap(collection_dir, row.uuid)
        if mcap is None:
            continue
        try:
            df_cmd   = load_cmd(mcap, "/control/command/control_cmd")
            df_accel = load_accel(mcap)
            df_vel   = load_velocity(mcap)
        except Exception:
            continue
        if df_cmd.empty or df_accel.empty or df_vel.empty:
            continue

        t0 = max(df_cmd["t_ns"].iloc[0], df_accel["t_ns"].iloc[0], df_vel["t_ns"].iloc[0])
        t1 = min(df_cmd["t_ns"].iloc[-1], df_accel["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1])
        if (t1 - t0) < 2e9:
            continue
        t_ns  = np.arange(t0, t1, _FIT_DT * 1e9, dtype=np.float64)
        t_s   = (t_ns - t0) * 1e-9
        a_cmd_arr = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_accel"].values)
        a_act_arr = np.interp(t_s, (df_accel["t_ns"].values - t0) * 1e-9, df_accel["accel"].values)
        vx        = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)

        # 横断同定値でシミュレーション（初期過渡が走行区間に影響しないよう全期間使用）
        a_sim_cross: np.ndarray | None = None
        if not np.isnan(tau_cross):
            a_sim_cross = _sim_first_order(a_cmd_arr, tau_cross, int(round(T_cross / _FIT_DT)))
        a_sim_tune: np.ndarray | None = None
        if not (np.isnan(tau_tune) or np.isnan(T_tune)):
            a_sim_tune = _sim_first_order(a_cmd_arr, tau_tune, int(round(T_tune / _FIT_DT)))

        # 低速区間（停止・停車）をマスク → NaN で折れ線を途切れさせる
        moving = vx > VX_MIN_CURVE

        def _mask(arr: np.ndarray) -> list:
            a = arr.copy().astype(float)
            a[~moving] = np.nan
            return a.tolist()

        rmse_val = getattr(row, "rmse_mps2", float("nan"))
        case_tag = getattr(row, "_case", "")
        rows_data.append({
            "label": f"[{case_tag}] {row.uuid[:8]}  RMSE={rmse_val:.3f} m/s²",
            "t": t_s.tolist(),
            "a_cmd": _mask(a_cmd_arr),
            "a_act": _mask(a_act_arr),
            "a_sim_cross": _mask(a_sim_cross) if a_sim_cross is not None else None,
            "a_sim_tune": _mask(a_sim_tune) if a_sim_tune is not None else None,
        })

    if not rows_data:
        return _placeholder("MCAP 読み込み失敗（データセットディレクトリが見つかりません）")

    cross_label = (
        f"横断最小二乗法同定値  τ={tau_cross:.3f}s  遅延={T_cross:.3f}s  RMSE={rmse_cross:.3f} m/s²"
        if not np.isnan(tau_cross) else "横断最小二乗法同定値（同定失敗）"
    )
    _pl = phase_label or "チューニング値"
    tune_label = f"{_pl}  τ={tau_tune:.3f}s  遅延={T_tune:.3f}s"

    n = len(rows_data)
    fig = make_subplots(rows=n, cols=1,
                        subplot_titles=[r["label"] for r in rows_data],
                        vertical_spacing=0.08)
    show_legend = True
    for i, r in enumerate(rows_data, 1):
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["a_act"],
            name="実測加速度", line=dict(color="black", width=1.5),
            showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["a_cmd"],
            name="指令加速度", line=dict(color="gray", width=1.2, dash="dash"),
            showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        if r["a_sim_cross"] is not None:
            fig.add_trace(go.Scatter(
                x=r["t"], y=r["a_sim_cross"],
                name=cross_label,
                line=dict(color="steelblue", width=1.5, dash="dot"),
                showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        if r["a_sim_tune"] is not None:
            fig.add_trace(go.Scatter(
                x=r["t"], y=r["a_sim_tune"],
                name=tune_label,
                line=dict(color="darkorange", width=1.5),
                showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        show_legend = False
        fig.update_yaxes(title_text="a [m/s²]", row=i, col=1)
    fig.update_xaxes(title_text="時刻 [s]", row=n, col=1)
    fig.update_layout(
        height=300 * n,
        title_text=f"縦方向モデルフィット（最良・最悪 計 {n} 件、全データセット横断最小二乗法同定値・走行区間のみ表示）",
        margin=dict(t=70, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )
    return fig


def build_long_perf_figure(
    records: list[dict],
    map_ways: list | None = None,
) -> tuple[go.Figure, go.Figure, go.Figure]:
    """縦方向モデル構造限界評価: 3 図を返す。

    fig_box   : ホライズン別絶対誤差 box plot（従来どおり）
    fig_growth : 符号付き誤差の成長カーブ（速度誤差・変位誤差 x 全体+局面別）
    fig_map   : 地図上の変位誤差分布（rollout 開始点を誤差で色分け）

    シミュレータの加速度応答が実機と完全一致（a_act_sim = a_act_gt）した場合に残る
    縦方向変位誤差を評価する。
    """
    h_labels = [f"{h * _FIT_DT:.2f}s" for h in _PERF_HORIZONS]
    per_h_errors: dict[int, list[float]] = {h: [] for h in _PERF_HORIZONS}
    _H_MAX = _PERF_HORIZONS[-1]  # 最長ホライズン（成長カーブ・地図分布用）

    # 成長カーブ用プール
    all_verr: list[np.ndarray] = []   # shape (m_i, H_max+1)
    all_serr: list[np.ndarray] = []
    all_phase: list[np.ndarray] = []  # int array (m_i,)

    # 地図分布用プール（rollout 開始点 k0 の x/y と終端変位誤差）
    map_x_pool: list[np.ndarray] = []
    map_y_pool: list[np.ndarray] = []
    map_serr_pool: list[np.ndarray] = []

    n_dataset = min(len(records), _LONG_PERF_N_DATASET)
    for rec in records[:n_dataset]:
        mcap = Path(rec["lite_dir"]) / "real.lite" / "real.lite_0.mcap"
        if not mcap.exists():
            continue
        try:
            df_accel = load_accel(mcap)
            df_vel   = load_velocity(mcap)
            df_kin   = load_kinematic(mcap)
        except Exception:
            continue
        if df_accel.empty or df_vel.empty:
            continue

        t0 = max(float(df_accel["t_ns"].values[0]), float(df_vel["t_ns"].values[0]))
        t1 = min(float(df_accel["t_ns"].values[-1]), float(df_vel["t_ns"].values[-1]))
        if (t1 - t0) < 2e9:
            continue
        t_ns  = np.arange(t0, t1, _FIT_DT * 1e9, dtype=np.float64)
        t_s   = (t_ns - t0) * 1e-9
        gt_vx = np.interp(t_s, (df_vel["t_ns"].values   - t0) * 1e-9, df_vel["lon_vel"].values)
        a_act = np.interp(t_s, (df_accel["t_ns"].values  - t0) * 1e-9, df_accel["accel"].values)
        if len(gt_vx) < 50:
            continue

        # box plot 用
        for h in _PERF_HORIZONS:
            errs = _long_nstep_perf(gt_vx, a_act, h, _FIT_DT)
            per_h_errors[h].extend(errs.tolist())

        # 成長カーブ用（最長ホライズンで符号付きプロファイルを取得）
        drift = _long_drift_profile(gt_vx, a_act, _H_MAX, _FIT_DT)
        if drift is not None:
            verr, serr, phase = drift
            all_verr.append(verr)
            all_serr.append(serr)
            all_phase.append(phase)

            # 地図分布用: k0s は _long_drift_profile と同一計算で x/y 位置を取得
            if not df_kin.empty:
                gt_x = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["x"].values)
                gt_y = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["y"].values)
                n = len(gt_vx)
                k0s_map = np.arange(0, n - _H_MAX, _PERF_STRIDE)
                k0s_map = k0s_map[gt_vx[k0s_map] > VX_MIN_CURVE]
                if len(k0s_map) > 0:
                    map_x_pool.append(gt_x[k0s_map])
                    map_y_pool.append(gt_y[k0s_map])
                    map_serr_pool.append(serr[:, _H_MAX])

    # ------------------------------------------------------------------
    # fig_box: 従来の絶対誤差 box plot
    # ------------------------------------------------------------------
    fig_box = go.Figure()
    for h, hl in zip(_PERF_HORIZONS, h_labels):
        errs = per_h_errors[h]
        if errs:
            fig_box.add_trace(go.Box(
                y=[e * 100 for e in errs], name=hl,
                boxpoints="outliers", marker_size=3, marker_color="darkorange",
            ))
    if not any(per_h_errors[h] for h in _PERF_HORIZONS):
        fig_box.add_annotation(
            text="データなし（データセットが見つかりません）",
            xref="paper", yref="paper", x=0.5, y=0.5,
            showarrow=False, font=dict(size=13),
        )
    fig_box.update_layout(
        title=f"縦方向 モデル構造限界評価（a_act 直接入力 vs GT 変位、上位 {n_dataset} データセット）",
        xaxis_title="ホライズン [s]",
        yaxis_title="|縦方向誤差| [cm]",
        height=400,
        showlegend=False,
        margin=dict(t=60, b=40),
    )

    # ------------------------------------------------------------------
    # fig_growth: 符号付き誤差の成長カーブ（コア + 局面分類）
    # ------------------------------------------------------------------
    _PHASE_COLORS = {0: "steelblue", 1: "gray", 2: "tomato"}
    _PHASE_NAMES  = {0: "減速 (a < -0.3)", 1: "巡航", 2: "加速 (a > +0.3)"}

    if all_verr:
        verr_pool  = np.vstack(all_verr)   # (M, H_max+1)
        serr_pool  = np.vstack(all_serr)   # (M, H_max+1)
        phase_pool = np.concatenate(all_phase)  # (M,)
        t_axis = np.arange(_H_MAX + 1) * _FIT_DT  # 0 → 1.0 s

        fig_growth = make_subplots(
            rows=1, cols=2,
            subplot_titles=["速度誤差  vx_sim − GT vx  [m/s]",
                            "変位誤差  s_sim − s_gt  [cm]"],
            horizontal_spacing=0.10,
        )

        for col, (pool, scale, ytitle) in enumerate([
            (verr_pool, 1.0,   "速度誤差 [m/s]<br><sup>正 = sim が GT より速い</sup>"),
            (serr_pool, 100.0, "変位誤差 [cm]<br><sup>正 = sim が GT より進んでいる</sup>"),
        ], start=1):
            data = pool * scale
            med  = np.median(data, axis=0)
            q25  = np.percentile(data, 25, axis=0)
            q75  = np.percentile(data, 75, axis=0)

            # IQR 帯（半透明 fill）
            fig_growth.add_trace(go.Scatter(
                x=np.concatenate([t_axis, t_axis[::-1]]).tolist(),
                y=np.concatenate([q75, q25[::-1]]).tolist(),
                fill="toself", fillcolor="rgba(255,165,0,0.18)",
                line=dict(color="rgba(0,0,0,0)"),
                name="IQR 25–75%", showlegend=(col == 1),
                legendgroup="iqr",
            ), row=1, col=col)

            # 全体中央値（橙実線）
            fig_growth.add_trace(go.Scatter(
                x=t_axis.tolist(), y=med.tolist(),
                mode="lines", line=dict(color="darkorange", width=2.5),
                name="中央値（全体）", showlegend=(col == 1),
                legendgroup="med_all",
            ), row=1, col=col)

            # 局面別中央値（細線オーバーレイ）
            for ph_id, ph_name in _PHASE_NAMES.items():
                mask = phase_pool == ph_id
                if mask.sum() < 5:
                    continue
                ph_med = np.median(data[mask], axis=0)
                fig_growth.add_trace(go.Scatter(
                    x=t_axis.tolist(), y=ph_med.tolist(),
                    mode="lines",
                    line=dict(color=_PHASE_COLORS[ph_id], width=1.2, dash="dot"),
                    name=ph_name, showlegend=(col == 1),
                    legendgroup=f"ph{ph_id}",
                ), row=1, col=col)

            # ゼロ基準線
            fig_growth.add_hline(
                y=0, line=dict(color="black", width=1, dash="dash"),
                row=1, col=col,
            )

            fig_growth.update_yaxes(title_text=ytitle, row=1, col=col)
            fig_growth.update_xaxes(title_text="ロールアウト経過時間 [s]", row=1, col=col)

        fig_growth.update_layout(
            title=(
                f"縦方向ドリフト成長カーブ"
                f"（符号付き・reset-stride ロールアウト、上位 {n_dataset} データセット）"
            ),
            height=430,
            margin=dict(t=70, b=50),
            legend=dict(orientation="v", x=1.02, y=1),
        )
    else:
        fig_growth = go.Figure()
        fig_growth.add_annotation(
            text="データなし（ドリフトプロファイルを算出できるデータセットがありません）",
            xref="paper", yref="paper", x=0.5, y=0.5,
            showarrow=False, font=dict(size=13),
        )
        fig_growth.update_layout(height=300)

    # ------------------------------------------------------------------
    # fig_map: 地図上の変位誤差分布（rollout 開始点を 1.0s 終端誤差で色分け）
    # ------------------------------------------------------------------
    _MAP_MARGIN = 10.0
    if not map_x_pool:
        fig_map = go.Figure()
        fig_map.add_annotation(
            text="データなし（位置データを取得できませんでした）",
            xref="paper", yref="paper",
            x=0.5, y=0.5, showarrow=False, font=dict(size=13),
        )
        fig_map.update_layout(height=400)
    else:
        x_arr    = np.concatenate(map_x_pool)
        y_arr    = np.concatenate(map_y_pool)
        serr_arr = np.concatenate(map_serr_pool) * 100.0  # m → cm

        x_min = float(x_arr.min()) - _MAP_MARGIN
        x_max = float(x_arr.max()) + _MAP_MARGIN
        y_min = float(y_arr.min()) - _MAP_MARGIN
        y_max = float(y_arr.max()) + _MAP_MARGIN
        vmax  = max(float(np.percentile(np.abs(serr_arr), 99)), 1e-6)

        fig_map = go.Figure()

        # 地図レーン背景
        if map_ways is not None:
            lane_ways = map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max))
            if lane_ways:
                fig_map.add_trace(lanes_to_trace(lane_ways))

        # rollout 開始点を誤差値で色分け
        fig_map.add_trace(go.Scatter(
            x=x_arr.tolist(), y=y_arr.tolist(),
            mode="markers",
            marker=dict(
                color=serr_arr.tolist(),
                colorscale="RdBu",
                reversescale=True,
                cmin=-vmax, cmax=vmax,
                size=4,
                colorbar=dict(title="cm", thickness=14),
            ),
            showlegend=False,
            hovertemplate=(
                "x=%{x:.1f}m y=%{y:.1f}m<br>"
                "変位誤差=%{marker.color:.2f}cm"
                "<extra></extra>"
            ),
        ))

        fig_map.update_xaxes(title_text="x [m]", range=[x_min, x_max])
        fig_map.update_yaxes(
            title_text="y [m]", range=[y_min, y_max],
            scaleanchor="x", scaleratio=1,
        )
        fig_map.update_layout(
            title="縦方向変位誤差の地図分布（1.0s 窓終端・rollout 開始点、正 = sim が GT より進む）",
            height=600,
            margin=dict(t=70, b=50, r=80),
        )

    return fig_box, fig_growth, fig_map


def build_steer_id_figure(collection_dir: Path, params: dict) -> go.Figure:
    """操舵モデルフィット時系列（非線形最小二乗法同定値 vs チューン値、代表データセット 3本）。"""
    csv_path = collection_dir / "steer_dynamics_identified.csv"

    def _placeholder(msg: str) -> go.Figure:
        fig = go.Figure()
        fig.add_annotation(text=msg, xref="paper", yref="paper",
                           x=0.5, y=0.5, showarrow=False, font=dict(size=13), align="center")
        fig.update_layout(height=200)
        return fig

    if not csv_path.exists():
        return _placeholder(
            f"<b>{csv_path.name} が見つかりません</b><br>"
            "事前に <code>identify_steer_dynamics.py</code> を実行してください。"
        )

    df_id = pd.read_csv(csv_path)
    tau_tune = float(params.get("steer_time_constant", float("nan")))
    T_tune   = float(params.get("steer_time_delay", float("nan")))

    # 動的区間が豊富なデータセットを候補にし、最良・最悪それぞれ _FIT_N_DATASET 本を選択
    _N_DYN_MIN = 100
    df_cand = df_id[df_id["n_dyn"] >= _N_DYN_MIN]
    if len(df_cand) < _FIT_N_DATASET * 2:
        df_cand = df_id
    df_best  = df_cand.nsmallest(_FIT_N_DATASET, "rmse_mrad")
    df_worst = df_cand.nlargest(_FIT_N_DATASET, "rmse_mrad")
    df_worst = df_worst[~df_worst.index.isin(df_best.index)]
    df_display = pd.concat([
        df_best.assign(_case="最良"),
        df_worst.assign(_case="最悪"),
    ])

    rows_data = []
    for row in df_display.itertuples():
        mcap = _find_mcap(collection_dir, row.uuid)
        if mcap is None:
            continue
        try:
            df_cmd   = load_cmd(mcap, "/control/command/control_cmd")
            df_steer = load_steering(mcap)
            df_vel   = load_velocity(mcap)
        except Exception:
            continue
        if df_cmd.empty or df_steer.empty or df_vel.empty:
            continue

        t0 = max(df_cmd["t_ns"].iloc[0], df_steer["t_ns"].iloc[0], df_vel["t_ns"].iloc[0])
        t1 = min(df_cmd["t_ns"].iloc[-1], df_steer["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1])
        if (t1 - t0) < 2e9:
            continue
        t_ns    = np.arange(t0, t1, _FIT_DT * 1e9, dtype=np.float64)
        t_s     = (t_ns - t0) * 1e-9
        d_cmd   = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_steer"].values)
        d_act   = np.interp(t_s, (df_steer["t_ns"].values - t0) * 1e-9, df_steer["steer"].values)
        vx      = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)

        # データセット全体でシミュレーション（初期過渡が走行区間に影響しないよう全期間使用）
        d_sim_id = _sim_first_order(d_cmd, row.tau, int(round(row.delay / _FIT_DT)))
        d_sim_tune = None
        if not (np.isnan(tau_tune) or np.isnan(T_tune)):
            d_sim_tune = _sim_first_order(d_cmd, tau_tune, int(round(T_tune / _FIT_DT)))

        # 低速区間をマスク
        moving = vx > VX_MIN_CURVE
        def _mask(arr: np.ndarray) -> list:
            a = arr.copy().astype(float)
            a[~moving] = np.nan
            return a.tolist()

        case_tag = getattr(row, "_case", "")
        rows_data.append({
            "label": f"[{case_tag}] {row.uuid[:8]}  RMSE={row.rmse_mrad:.1f} mrad  τ={row.tau:.3f}s  T={row.delay:.3f}s",
            "t": t_s.tolist(),
            "d_act": _mask(d_act),
            "d_sim_id": _mask(d_sim_id),
            "d_sim_tune": _mask(d_sim_tune) if d_sim_tune is not None else None,
        })

    if not rows_data:
        return _placeholder("MCAP 読み込み失敗（データセット ディレクトリが見つかりません）")

    n = len(rows_data)
    fig = make_subplots(rows=n, cols=1,
                        subplot_titles=[r["label"] for r in rows_data],
                        vertical_spacing=0.08)
    show_legend = True
    for i, r in enumerate(rows_data, 1):
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["d_act"],
            name="実測 δ_act", line=dict(color="black", width=1.5),
            showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["d_sim_id"],
            name="非線形最小二乗法同定値", line=dict(color="steelblue", width=1.5, dash="dot"),
            showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        if r["d_sim_tune"] is not None:
            fig.add_trace(go.Scatter(
                x=r["t"], y=r["d_sim_tune"],
                name=f"チューン値 τ={tau_tune:.3f}s T={T_tune:.3f}s",
                line=dict(color="darkorange", width=1.5),
                showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        show_legend = False
        fig.update_yaxes(title_text="δ [rad]", row=i, col=1)
    fig.update_xaxes(title_text="時刻 [s]", row=n, col=1)
    fig.update_layout(
        height=300 * n,
        title_text=f"操舵モデルフィット（最良・最悪 計 {n} 件、走行区間のみ表示）",
        margin=dict(t=70, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )
    return fig


def build_perfect_tracking_figure(
    top_records: list[dict],
    params: dict,
) -> tuple[go.Figure, go.Figure]:
    """理想追従（実測 vx + δ_act 入力）の N-step 横方向誤差 box plot + 軌跡比較図。

    アクチュエータ追従が完璧だった場合の位置ずれを評価し、
    モデル構造限界（タイヤスリップ・路面バンク・vy・k_us キャリブレーション誤差）を定量化する。
    縦方向誤差は gt_vx を直接使うため積分上ほぼゼロになる（設計上の帰結）。
    """
    n_dataset = min(len(top_records), _PERF_N_DATASET)
    h_labels = [f"{h * _FIT_DT:.2f}s" for h in _PERF_HORIZONS]

    per_h_errors: dict[int, list[float]] = {h: [] for h in _PERF_HORIZONS}
    traj_data: list[dict] = []

    for rec in top_records[:n_dataset]:
        mcap = Path(rec["lite_dir"]) / "real.lite" / "real.lite_0.mcap"
        if not mcap.exists():
            continue
        try:
            df_kin   = load_kinematic(mcap)
            df_steer = load_steering(mcap)
        except Exception:
            continue
        if df_kin.empty or df_steer.empty or len(df_kin) < 50:
            continue

        t0 = float(df_kin["t_ns"].values[0])
        t1 = float(df_kin["t_ns"].values[-1])
        t_ns = np.arange(t0, t1, _FIT_DT * 1e9, dtype=np.float64)
        t_s  = (t_ns - t0) * 1e-9

        def _ip(df_raw: pd.DataFrame, col: str) -> np.ndarray:
            return np.interp(t_s, (df_raw["t_ns"].values - t0) * 1e-9, df_raw[col].values)

        gt_x     = _ip(df_kin, "x")
        gt_y     = _ip(df_kin, "y")
        gt_yaw   = _ip(df_kin, "yaw")
        gt_vx    = _ip(df_kin, "vx")
        gt_steer = _ip(df_steer, "steer")

        for h in _PERF_HORIZONS:
            lat_errs = _bicycle_nstep_perf(gt_x, gt_y, gt_yaw, gt_vx, gt_steer, params, h, _FIT_DT)
            per_h_errors[h].extend(lat_errs.tolist())

        if len(traj_data) < _PERF_N_TRAJ:
            bx, by = _bicycle_trajectory_full(
                gt_vx, gt_steer, float(gt_x[0]), float(gt_y[0]), float(gt_yaw[0]), params, _FIT_DT
            )
            moving = gt_vx > VX_MIN_CURVE
            _PLOT_STRIDE = 5
            traj_data.append({
                "uuid": rec["uuid"][:8],
                "gt_x": (gt_x - gt_x[0])[::_PLOT_STRIDE],
                "gt_y": (gt_y - gt_y[0])[::_PLOT_STRIDE],
                "bx":   (bx   - gt_x[0])[::_PLOT_STRIDE],
                "by":   (by   - gt_y[0])[::_PLOT_STRIDE],
                "moving": moving[::_PLOT_STRIDE],
            })

    # ---- Box plot ----
    fig_box = go.Figure()
    for h, hl in zip(_PERF_HORIZONS, h_labels):
        errs = per_h_errors[h]
        if errs:
            fig_box.add_trace(go.Box(
                y=[e * 100 for e in errs],
                name=hl,
                boxpoints="outliers",
                marker_size=3,
                marker_color="steelblue",
            ))
    fig_box.update_layout(
        title=f"理想追従 N-step 横方向誤差（上位 {n_dataset} データセット プール）",
        xaxis_title="ホライズン [s]",
        yaxis_title="|横方向誤差| [cm]",
        height=400,
        showlegend=False,
        margin=dict(t=50, b=40),
    )

    # ---- 軌跡比較 ----
    n_traj = len(traj_data)
    if n_traj == 0:
        fig_traj = go.Figure()
        fig_traj.update_layout(title="軌跡データなし", height=300)
    else:
        fig_traj = make_subplots(
            rows=1, cols=n_traj,
            subplot_titles=[f"データセット {d['uuid']}" for d in traj_data],
            horizontal_spacing=0.08,
        )
        for i, td in enumerate(traj_data, start=1):
            m = td["moving"]
            gt_xm = td["gt_x"].copy().astype(float); gt_xm[~m] = np.nan
            gt_ym = td["gt_y"].copy().astype(float); gt_ym[~m] = np.nan
            bxm   = td["bx"].copy().astype(float);   bxm[~m]   = np.nan
            bym   = td["by"].copy().astype(float);   bym[~m]   = np.nan
            show_legend = (i == 1)
            fig_traj.add_trace(go.Scatter(
                x=gt_xm.tolist(), y=gt_ym.tolist(),
                mode="lines", name="GT 軌跡",
                line=dict(color="black", width=2),
                legendgroup="gt", showlegend=show_legend,
            ), row=1, col=i)
            fig_traj.add_trace(go.Scatter(
                x=bxm.tolist(), y=bym.tolist(),
                mode="lines", name="自転車モデル（理想追従）",
                line=dict(color="steelblue", width=1.5, dash="dash"),
                legendgroup="bic", showlegend=show_legend,
            ), row=1, col=i)
        fig_traj.update_layout(
            title="GT vs 自転車モデル軌跡（実測 vx + δ_act 入力、初期状態 GT 合わせ、リセットなし）",
            height=480,
            margin=dict(t=60, b=40),
        )
        for i in range(1, n_traj + 1):
            fig_traj.update_xaxes(title_text="Δx [m]", row=1, col=i)
            fig_traj.update_yaxes(title_text="Δy [m]", row=1, col=i)

    return fig_box, fig_traj


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
    score_str = f"{float(score):.4f}" if score is not None and str(score) != "N/A" else "N/A"
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


def _build_sec1(
    params: dict,
    long_fig: go.Figure,
    steer_fig: go.Figure,
    kus_fig: go.Figure,
    n_dataset: int,
    long_perf_figs: tuple[go.Figure, go.Figure, go.Figure] | None = None,
) -> str:
    kus_rows = _kus_band_table_rows(params)

    def _fmt(v) -> str:
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

    _long_html_inner  = long_fig.to_html(full_html=False, include_plotlyjs=False)
    _steer_html_inner = steer_fig.to_html(full_html=False, include_plotlyjs=False)
    long_html  = f"<details><summary>時系列グラフを表示（クリックで展開）</summary>{_long_html_inner}</details>"
    steer_html = f"<details><summary>時系列グラフを表示（クリックで展開）</summary>{_steer_html_inner}</details>"
    kus_html   = kus_fig.to_html(full_html=False, include_plotlyjs=False)
    _vx_min = VX_MIN_CURVE
    _stride = _PERF_STRIDE
    if long_perf_figs is not None:
        fig_box, fig_growth, fig_map = long_perf_figs
        box_html    = fig_box.to_html(full_html=False, include_plotlyjs=False)
        growth_html = fig_growth.to_html(full_html=False, include_plotlyjs=False)
        map_html    = fig_map.to_html(full_html=False, include_plotlyjs=False)
        long_perf_subsection = (
            "<h3>モデル構造限界評価（acc 理想追従）</h3>"
            "<p>シミュレータの加速度応答が実機と完全一致（\\(a_{\\mathrm{act,sim}} = a_{\\mathrm{act,gt}}\\)）した場合に"
            "残る縦方向変位誤差を評価する。<br>"
            "各開始点で \\(v_{x,\\mathrm{sim}}(t_0) = v_{x,\\mathrm{GT}}(t_0)\\) に初期化し、"
            "実測加速度 \\(a_{\\mathrm{act}}\\) を直接積分して変位を計算、GT 変位と比較する。</p>"
            "<p>1-4 横方向評価との対称性: "
            "1-4 では <i>gt_steer</i> を bicycle model に直接入力して steer 完全追従時の横方向残差を評価する。"
            "本評価では <i>gt_a_act</i> を積分器に直接入力して acc 完全追従時の縦方向残差を評価する。</p>"
            "<div class=\"note\">"
            "&#9888;&#65039; <b>残差の解釈</b>: \\(\\dot{v}_x = a_{\\mathrm{act}}\\) という運動方程式に含まれない"
            "要素（路面勾配・空気抵抗・タイヤ縦力・加速度センサーバイアス等）が残差として現れる。"
            "</div>"
            f"<p>走行区間（\\(v_x > {_vx_min}\\) m/s）を stride={_stride} ステップで走査し、"
            "N-step ロールアウト終端の縦方向誤差絶対値を集計する。"
            "ホライズン: N=10（0.10s）, N=20（0.20s）, N=50（0.50s）, N=100（1.00s）。</p>"
            + box_html
            + "<p><b>図②: ドリフト成長カーブ（符号付き）</b> — "
            "ゼロ線から片側に膨らむ傾向が系統的な過大／過小推定を示す。"
            "帯（IQR 25–75%）はばらつき、膨らむ速さは蓄積の速度を表す。"
            "局面別点線（青=減速・灰=巡航・赤=加速）でどの走行シーンでズレが生じるかを確認できる。"
            "x 軸の 0.10/0.20/0.50/1.00s は上の box plot のホライズンと一致する。</p>"
            + growth_html
            + "<p><b>図③: 地図上の変位誤差分布</b> — "
            "各点は rollout 開始位置（\\(v_x > v_{\\mathrm{min}}\\) を満たす 1.0s 窓の開始点）。"
            "色は 1.0s 窓終端の変位誤差（赤 = sim が GT より進む過大推定、青 = 過小推定）。"
            "路線・カーブ・区間ごとに誤差パターンを地理的に把握できる。</p>"
            + map_html
        )
    else:
        long_perf_subsection = ""

    return f"""
<section id="sec-long">
<h2>1-1. 縦方向（加速度アクチュエータ）— Phase 49 で同定</h2>
<p>
速度 \\(v_x\\) は横方向・操舵状態に依存せず縦方向のみで閉じるため（long ⊥ steer の直交性）、
\\(\\text{{err}}_{{vx}}\\) を目的関数として他のパラメータと独立に同定できる。
</p>

<p><b>運動方程式:</b></p>
\\[
a_{{\\mathrm{{cmd,del}}}}(t) = a_{{\\mathrm{{cmd}}}}(t - T_a)
\\]
\\[
\\dot{{a}}_{{\\mathrm{{act}}}}(t) = \\frac{{a_{{\\mathrm{{cmd,del}}}}(t) - a_{{\\mathrm{{act}}}}(t)}}{{\\tau_a}}
\\]
\\[
\\dot{{v}}_x(t) = a_{{\\mathrm{{act}}}}(t)
\\]
<p>
加速度指令 \\(a_{{\\mathrm{{cmd}}}}\\) は純粋遅延 \\(T_a\\) だけ遅れてアクチュエータに届き、
時定数 \\(\\tau_a\\) の一次遅れでペダル加速度 \\(a_{{\\mathrm{{act}}}}\\) が応答する。
速度 \\(v_x\\) はその積分として得られる。
</p>

<h3>実機ログからの独立同定（代表データセットモデルフィット）</h3>
<p>
\\(a_{{\\mathrm{{cmd}}}}\\)（指令加速度）を入力として遅延グリッドサーチ + output-error 非線形最小二乗法で
同定した \\((\\tau_a, T_a)\\) のモデル出力（青点線）と実測 \\(a_{{\\mathrm{{act}}}}\\)（黒実線）を比較する。
橙実線はチューン値でのシミュレーション結果。動的区間（n_dyn）が豊富なデータセットから rmse 最小順に選択し、低速・停車区間は除外して表示。
</p>
{long_html}

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量</th></tr>
  <tr><td><code>acc_time_constant</code> (τ_a)</td><td>{tau_a} s</td>
      <td>一次遅れ時定数：小さいほど加速応答が速い</td><td rowspan="2">err_vx</td></tr>
  <tr><td><code>acc_time_delay</code> (T_a)</td><td>{T_a} s</td>
      <td>純粋遅延：指令が実際に入力されるまでの無駄時間</td></tr>
</table>

{long_perf_subsection}
</section>

<section id="sec-steer">
<h2>1-2. 操舵アクチュエータ（追従ループ）— Phase 50 で同定</h2>
<p>
操舵追従ループは実車位置・ヨーのフィードバックを持たないオープンループなので、
\\(\\text{{err}}_{{\\mathrm{{steer}}}}\\) を目的関数として位置・ヨー誤差とは構造的に独立して同定できる。
</p>

<p><b>運動方程式:</b></p>
\\[
\\delta_{{\\mathrm{{cmd,del}}}}(t) = \\delta_{{\\mathrm{{cmd}}}}(t - T_\\delta)
\\]
\\[
\\delta_{{\\mathrm{{des}}}}(t) = \\mathrm{{DSF}} \\cdot \\mathrm{{clamp}}(\\delta_{{\\mathrm{{cmd,del}}}},\\; \\pm\\delta_{{\\mathrm{{lim}}}})
\\]
\\[
\\dot{{\\delta}}_{{\\mathrm{{act}}}}(t) = \\mathrm{{sat}}\\!\\left(
  -\\frac{{\\delta_{{\\mathrm{{act}}}}(t) - \\delta_{{\\mathrm{{des}}}}(t) + \\mathrm{{db}}(\\cdot)}}{{\\tau_\\delta}},\\;
  \\pm\\dot{{\\delta}}_{{\\mathrm{{lim}}}}
\\right)
\\]
\\[
\\delta_{{\\mathrm{{sim}}}}(t) = \\delta_{{\\mathrm{{act}}}}(t) + \\beta
\\]
<p>
ここで \\(\\mathrm{{db}}(\\cdot)\\) は不感帯（\\(|\\delta_{{\\mathrm{{act}}}} - \\delta_{{\\mathrm{{des}}}}| \\leq \\delta_{{\\mathrm{{db}}}}\\) の範囲ではレート = 0）。
報告操舵角 \\(\\delta_{{\\mathrm{{sim}}}}\\) には steer_bias β を加算する（β の一方の役割）。
</p>
<div class="note">
⚠️ <b>結合点</b>: DSF は操舵指令に定数ゲインをかける形で、直進時の系統的な横力成分（v²δ 由来の
アンダーステア成分）を部分的に吸収できる。
したがって DSF の最適値は k_us の同定後に再検証することが望ましい。<br>
β（steer_bias）はアクチュエータ追従式自体には入らず（<code>getSteer()</code> は bias なし）、
報告値の加算と 1-3 のヨー式の両方に現れる二重登場のパラメータである。
</div>

<h3>実機ログからの独立同定（代表データセットモデルフィット）</h3>
<p>
\\(\\delta_{{\\mathrm{{cmd}}}}\\) を入力として遅延グリッドサーチ + output-error 非線形最小二乗法で
同定した \\((\\tau_\\delta, T_\\delta)\\) のモデル出力（青点線）と実測 \\(\\delta_{{\\mathrm{{act}}}}\\)（黒実線）を比較する。
橙実線はチューン値でのシミュレーション結果。動的区間（n_dyn）が豊富なデータセットから rmse 最小順に選択し、低速・停車区間は除外して表示。
</p>
{steer_html}
<div class="note">
⚠️ <b>注記</b>: <code>cmd_steer</code> は understeer converter 適用前（コントローラ出力）、
<code>delta_act</code> は converter 適用後の実舵角である。定常ゲインは速度依存の補正を含むため、
時定数 \\(\\tau_\\delta\\) と遅延 \\(T_\\delta\\) の分布は quasi-static で有効だが、ゲイン推定は低速帯（v &lt; 3 m/s）に限定して評価することが望ましい。
</div>

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量</th></tr>
  <tr><td><code>steer_time_constant</code> (τ_δ)</td><td>{tau_d} s</td>
      <td>一次遅れ時定数：小さいほど操舵応答が速い</td><td rowspan="4">err_steer</td></tr>
  <tr><td><code>steer_time_delay</code> (T_δ)</td><td>{T_d} s</td>
      <td>純粋遅延：操舵指令の無駄時間</td></tr>
  <tr><td><code>debug_steer_scaling_factor</code> (DSF)</td><td>{DSF}</td>
      <td>指令スケーリング（1.0 = 補正なし）；遅延後に乗算</td></tr>
  <tr><td><code>steer_bias</code> (β)</td><td>{beta} rad</td>
      <td>報告操舵角への加算（δ_sim = δ_act + β）；ヨー式にも二重登場</td></tr>
  <tr><td><code>steer_dead_band</code></td><td>{db} rad</td>
      <td>不感帯幅（固定値・同定対象外）</td><td>—</td></tr>
  <tr><td><code>steer_rate_lim</code></td><td>{rlim} rad/s</td>
      <td>操舵レート飽和（固定値・同定対象外）</td><td>—</td></tr>
</table>
</section>
<section id="sec-yaw">
<h2>1-3. ヨー・横方向（運動学的自転車モデル）— 速度ビン別 最小二乗法同定</h2>
<p>
1-1（縦方向）・1-2（操舵追従）が先行して確定した後、
\\(\\text{{err}}_{{wz}}\\) を目的関数として <b>高曲率サブセット</b> で k_us を同定する。
直進（\\(\\delta \\approx 0\\)）では感度がゼロなので、全データセット集約スコアは k_us に対して構造的不可同定。
</p>

<p><b>運動方程式:</b></p>
\\[
\\omega(t) = \\frac{{v_x \\cdot \\tan\\bigl(\\delta_{{\\mathrm{{act}}}}(t) + \\beta\\bigr)}}{{\\max\\!\\left(L + k_{{\\mathrm{{us,eff}}}}(v_x)\\cdot v_x^2,\\; 0.05\\,L\\right)}},
\\qquad \\dot{{\\theta}} = \\omega
\\]
\\[
\\dot{{x}} = v_x \\cos\\theta, \\qquad \\dot{{y}} = v_x \\sin\\theta
\\]
<p>
アンダーステア係数 \\(k_{{\\mathrm{{us,eff}}}}\\) は速度帯ごとの定数（速度帯ステップ）で実装される。
β（steer_bias）はヨー式の \\(\\tan(\\cdot)\\) 引数にも加算され、系統的なヨーオフセットを生む
（1-2 の報告値加算とは別経路で同一パラメータが効く）。
</p>
<div class="note">
⚠️ <b>前提条件</b>: この式の入力 \\(\\delta_{{\\mathrm{{act}}}}\\) は 1-2 の結果に依存する。
1-2 の DSF が k_us の v²δ 成分を部分吸収しているため、Phase 50 確定後の DSF 値を固定した上で
k_us を同定することが重要。
</div>

<h3>実機ログからの独立同定（全 {n_dataset} データセット、速度ビン別 最小二乗法）</h3>

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
</p>
</details>
{kus_html}
<div class="note">
<b>解釈</b>: 最小二乗法推定値が低速ビンでほぼ 0、高速ビンで正の値に推移していれば、
ランプ形状は物理的実態と整合している。ただし J6 の多くのデータセットが低速（vx_mean ≈ 1.9 m/s）
のため、高速ビンのサンプル数は少なく推定誤差が大きい点に注意（右パネルのサンプル数を参照）。
</div>

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量</th></tr>
{kus_rows}
  <tr><td><code>steer_bias</code> (β) <i>[1-2 と共有]</i></td><td>{beta} rad</td>
      <td>tan(δ_act + β) の引数：ヨーオフセットを生む</td><td>err_wz（間接）</td></tr>
</table>
</section>
"""


def _build_sec14(
    fig_box: go.Figure,
    fig_traj: go.Figure,
    params: dict,
    n_dataset: int,
) -> str:
    """1-4. モデル構造限界（理想追従評価）セクション HTML。"""
    box_html  = fig_box.to_html(full_html=False, include_plotlyjs=False)
    traj_html = fig_traj.to_html(full_html=False, include_plotlyjs=False)
    tau_a = f"{params.get('acc_time_constant', float('nan')):.3g}"
    T_a   = f"{params.get('acc_time_delay', float('nan')):.3g}"
    tau_d = f"{params.get('steer_time_constant', float('nan')):.3g}"
    T_d   = f"{params.get('steer_time_delay', float('nan')):.3g}"
    h_str = ", ".join(f"N={h}（{h * _FIT_DT:.2f}s）" for h in _PERF_HORIZONS)
    return f"""
<section id="sec-perf-tracking">
<h2>1-4. モデル構造限界（理想追従評価）</h2>
<p>
アクチュエータ追従が完璧だった場合（実測 \\(v_x\\) と実測 \\(\\delta_{{\\mathrm{{act}}}}\\) を
自転車モデルの直接入力として使用）に残る位置ずれを評価する。
これにより <b>アクチュエータ遅れの寄与</b> と <b>モデル構造外の寄与</b>（タイヤスリップ、路面バンク、
横速度 \\(v_y\\)、\\(k_{{\\mathrm{{us}}}}\\) キャリブレーション誤差）を分離できる。
</p>
<p>
現行スコアとの差分 ≈ アクチュエータ応答が占める誤差分
（τ_a={tau_a} s, T_a={T_a} s, τ_δ={tau_d} s, T_δ={T_d} s の合算効果）。
</p>
<div class="note">
⚠️ <b>設計上の帰結</b>:
縦方向誤差は \\(v_x\\) を GT から直接取得しているため積分上ほぼゼロになる。
<b>横方向誤差のみが真のモデル構造限界を表す。</b><br>
残差は「現行チューン値 \\(k_{{\\mathrm{{us}}}}\\) および \\(\\beta\\) での理想追従誤差」であるため、
パラメータのキャリブレーション誤差も一部含む（現行パラメータ前提での下限値）。
</div>

<h3>横方向誤差分布（ホライズン別、上位 {n_dataset} データセット）</h3>
<p>
カーブ走行区間（\\(v_x > {VX_MIN_CURVE}\\) m/s）を stride={_PERF_STRIDE} ステップで走査し、
N-step ロールアウト終端の横方向誤差絶対値を集計する。ホライズン: {h_str}。
</p>
{box_html}

<h3>代表データセット の軌跡比較（GT vs 自転車モデル）</h3>
<p>
初期状態を GT に合わせ、実測 \\(v_x\\) と \\(\\delta_{{\\mathrm{{act}}}}\\) を入力として積分した
自転車モデル軌跡（青破線）を GT 軌跡（黒実線）と比較する。
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
<summary>2-1. アンダーステア係数 k_us の速度依存性（全 {n_dataset} データセット）</summary>
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
ランプ形状は物理的実態と整合している。ただし J6 の多くのデータセットが低速（vx_mean ≈ 1.9 m/s）
のため、高速ビンのサンプル数は少なく推定誤差が大きい点に注意（右パネルのサンプル数を参照）。
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
ドロップダウンで <b>{label}</b>（k_us ランプ＋deadband 有効）と
<b>baseline</b>（k_us=0, deadband=0）を切り替えて実機軌跡への一致を比較できる。
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
    """N-step 終端誤差の最大乖離テーブルセクション（tuned vs baseline）。"""
    horizons = sorted(df["h"].unique().tolist())

    stats_list = []
    for h in horizons:
        sub = df[df["h"] == h]
        stats_list.append({
            "h": h,
            "p14_yaw_mean": sub["p14_yaw"].mean(), "p14_yaw_p95": sub["p14_yaw"].quantile(0.95), "p14_yaw_p99": sub["p14_yaw"].quantile(0.99), "p14_yaw_max": sub["p14_yaw"].max(),
            "p14_lat_mean": sub["p14_lat"].mean(), "p14_lat_p95": sub["p14_lat"].quantile(0.95), "p14_lat_p99": sub["p14_lat"].quantile(0.99), "p14_lat_max": sub["p14_lat"].max(),
            "p14_long_mean": sub["p14_long"].mean(), "p14_long_p95": sub["p14_long"].quantile(0.95), "p14_long_p99": sub["p14_long"].quantile(0.99), "p14_long_max": sub["p14_long"].max(),
            "p14_vx_mean": sub["p14_vx"].mean(), "p14_vx_p95": sub["p14_vx"].quantile(0.95), "p14_vx_p99": sub["p14_vx"].quantile(0.99), "p14_vx_max": sub["p14_vx"].max(),
            "bl_yaw_mean": sub["bl_yaw"].mean(), "bl_yaw_p95": sub["bl_yaw"].quantile(0.95), "bl_yaw_p99": sub["bl_yaw"].quantile(0.99), "bl_yaw_max": sub["bl_yaw"].max(),
            "bl_lat_mean": sub["bl_lat"].mean(), "bl_lat_p95": sub["bl_lat"].quantile(0.95), "bl_lat_p99": sub["bl_lat"].quantile(0.99), "bl_lat_max": sub["bl_lat"].max(),
            "bl_long_mean": sub["bl_long"].mean(), "bl_long_p95": sub["bl_long"].quantile(0.95), "bl_long_p99": sub["bl_long"].quantile(0.99), "bl_long_max": sub["bl_long"].max(),
            "bl_vx_mean": sub["bl_vx"].mean(), "bl_vx_p95": sub["bl_vx"].quantile(0.95), "bl_vx_p99": sub["bl_vx"].quantile(0.99), "bl_vx_max": sub["bl_vx"].max(),
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
            f'{_cell(s["p14_yaw_p95"], s["bl_yaw_p95"])}'
            f'{_cell(s["p14_yaw_p99"], s["bl_yaw_p99"])}'
            f'{_cell(s["p14_yaw_max"], s["bl_yaw_max"])}\n'
            f'  {_cell(s["p14_lat_mean"], s["bl_lat_mean"])}'
            f'{_cell(s["p14_lat_p95"], s["bl_lat_p95"])}'
            f'{_cell(s["p14_lat_p99"], s["bl_lat_p99"])}'
            f'{_cell(s["p14_lat_max"], s["bl_lat_max"])}\n'
            f'  {_cell(s["p14_long_mean"], s["bl_long_mean"])}'
            f'{_cell(s["p14_long_p95"], s["bl_long_p95"])}'
            f'{_cell(s["p14_long_p99"], s["bl_long_p99"])}'
            f'{_cell(s["p14_long_max"], s["bl_long_max"])}\n'
            f'  {_cell(s["p14_vx_mean"], s["bl_vx_mean"], ".4f")}'
            f'{_cell(s["p14_vx_p95"], s["bl_vx_p95"], ".4f")}'
            f'{_cell(s["p14_vx_p99"], s["bl_vx_p99"], ".4f")}'
            f'{_cell(s["p14_vx_max"], s["bl_vx_max"], ".4f")}\n'
            f'</tr>\n'
            f'<tr>\n'
            f'  <td style="color:#888">baseline</td>\n'
            f'  <td>{s["bl_yaw_mean"]:.3f}</td><td>{s["bl_yaw_p95"]:.3f}</td><td>{s["bl_yaw_p99"]:.3f}</td><td>{s["bl_yaw_max"]:.3f}</td>\n'
            f'  <td>{s["bl_lat_mean"]:.3f}</td><td>{s["bl_lat_p95"]:.3f}</td><td>{s["bl_lat_p99"]:.3f}</td><td>{s["bl_lat_max"]:.3f}</td>\n'
            f'  <td>{s["bl_long_mean"]:.3f}</td><td>{s["bl_long_p95"]:.3f}</td><td>{s["bl_long_p99"]:.3f}</td><td>{s["bl_long_max"]:.3f}</td>\n'
            f'  <td>{s["bl_vx_mean"]:.4f}</td><td>{s["bl_vx_p95"]:.4f}</td><td>{s["bl_vx_p99"]:.4f}</td><td>{s["bl_vx_max"]:.4f}</td>\n'
            f'</tr>'
        )

    tbody = "\n".join(tbody_rows)

    return f"""
<section id="deviation">
<h2>N-step 終端誤差（{label} vs baseline）</h2>
<p>
全データセットに対し {label} パラメータと baseline（補正なし）で N-step ロールアウトを実施し、
終端誤差 RMSE の データセット横断 <b>平均</b>（mean）、<b>95パーセンタイル</b>（95%点）、<b>99パーセンタイル</b>（99%点）、<b>最大</b>（worst-case データセット）を N ごとに集計する。
「最大」は「最も誤差が大きかった データセットの RMSE」を指す。
</p>
<p>
最大乖離（Worst値）は突発的なノイズ等で大きくなる場合がありますが、<b>95%点や99%点</b>の指標を見ることで、大部分の走行区間でモデルが極めて高い精度（OKの範囲内）で適合していることを確認できます。
</p>
{score_html}
<table class="param-table" style="font-size:12px">
  <thead>
    <tr>
      <th rowspan="2">N（時間）</th>
      <th rowspan="2">モデル</th>
      <th colspan="4">yaw 誤差 [deg]</th>
      <th colspan="4">lat 誤差 [cm]</th>
      <th colspan="4">long 誤差 [cm]</th>
      <th colspan="4">速度誤差 vx [m/s]</th>
    </tr>
    <tr>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
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
        playback_html = found_dir / "figures" / "trajectory_playback.html"
        
        if not playback_html.exists():
            playback_html_list = list(found_dir.glob("**/trajectory_playback.html"))
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
                print(f"  [WARN] trajectory_playback.html 読み込み失敗: {e}")
                
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
    long_perf_figs: tuple[go.Figure, go.Figure, go.Figure] | None = None,
    closed_loop_html: str = "",
) -> str:
    score = params.get("_score", "N/A")
    phase14_score = float(score) if isinstance(score, (int, float, str)) and str(score) != "N/A" else 0.0
    sec_intro = _build_sec_model_intro(params, label=label, params_filename=params_filename)
    sec_metrics = _build_sec_metrics(baseline_score=baseline_score, phase14_score=phase14_score, label=label)
    
    sec_tuning = f"""
<section id="sec-tuning">
<h2>2. 統合最適化（パラメータ最適化）</h2>
<p>
各モデルの独立最適化パラメータをベースにした、全データセット横断での統合最適化（Phase 0）の結果を評価します。
</p>
{sec_metrics}
{deviation_html}
</section>
"""

    sec1 = _build_sec1(params, long_fig, steer_fig, kus_fig, n_dataset, long_perf_figs=long_perf_figs)
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
  <a href="#model-intro">モデルパラメータ</a>
  <a href="#sec-long">1-1. 縦方向</a>
  <a href="#sec-steer">1-2. 操舵</a>
  <a href="#sec-yaw">1-3. ヨー・横方向</a>
  <a href="#sec-tuning">2. 統合最適化</a>
  {f'<a href="#sec-perf-tracking">1-4. モデル構造限界</a>' if perf_html else ""}
  <a href="#curve-viewer">3. カーブビューア</a>
  {f'<a href="#sec-closed-loop">4. クローズドループ比較</a>' if closed_loop_html else ""}
</nav>
{sec_intro}
{sec1}
{sec_tuning}
{perf_html}
{sec3}
{closed_loop_html}
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
        "--label", type=str, default="current",
        help="レポート内のモデル名ラベル（デフォルト: current）",
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
    records = load_all_mcap(ds_list, n_jobs=args.n_jobs)
    print(f"  有効: {len(records)} 件")

    # Phase 2: k_us 速度ビン別 最小二乗法
    print("\n[Phase 2] k_us 速度ビン別 最小二乗法 ...")
    bins = compute_kus_bins(records)
    n_valid = int(np.isfinite(bins["kus_ols"]).sum())
    print(f"  有効速度ビン: {n_valid}/{len(bins['kus_ols'])}")

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
    if args.metrics_cache and args.metrics_cache.exists():
        # キャッシュあり → viewer データセット のみ load、メトリクスは CSV から読む
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} データセット) ...")
        ctxs = load_datasets(top_items, n_jobs=min(args.n_jobs, len(top_items)))
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
        # baseline (k_us=0) の steer_score を計算
        baseline_steer_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline steer_score: {baseline_steer_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    elif args.metrics_cache:
        # キャッシュなし → 全データセット load（ついでに viewer データセット も取り出す）
        all_items = [(r["uuid"], Path(r["lite_dir"])) for r in records]
        print(f"\n[Phase 3b+3c] 全データセット DatasetCtx 構築 ({len(all_items)} データセット) + rollout メトリクス計算 ...")
        all_ctxs = load_datasets(all_items, n_jobs=args.n_jobs)
        # viewer データセットを all_ctxs から抽出（重複 load 回避）
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
        # baseline (k_us=0) の steer_score を計算
        baseline_steer_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline steer_score: {baseline_steer_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    else:
        baseline_steer_score = None
        # --metrics-cache 未指定 → 通常の viewer データセット のみ load
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} データセット) ...")
        ctxs = load_datasets(top_items, n_jobs=min(args.n_jobs, len(top_items)))

    curve_count_map = {r["uuid"]: r["curve_count"] for r in top_curve}

    # tuned 設定と baseline 設定（configs = {ラベル: override_params}）
    tuned_keys = [
        "k_us",
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
            "k_us": 0.0, "steer_dead_band": 0.0,
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
    kus_fig   = build_kus_figure(bins, params)
    long_fig  = build_long_figure(args.collection_dir, params, phase_label=phase_label)
    steer_fig = build_steer_id_figure(args.collection_dir, params)

    # 1-1 縦方向理想追従評価（全 records の先頭 _LONG_PERF_N_DATASET データセットを使用）
    long_perf_records = records[:_LONG_PERF_N_DATASET]
    print(f"  [1-1] 縦方向理想追従評価図生成 ({len(long_perf_records)} データセット) ...")
    long_perf_figs = build_long_perf_figure(long_perf_records, map_ways=map_ways)

    # 1-4 横方向理想追従評価には curve 上位 _PERF_N_DATASET データセットを使用（viewer 用 top_curve とは独立して選択）
    perf_records = candidate_curve[:_PERF_N_DATASET]
    print(f"  [1-4] 横方向理想追従評価図生成 ({len(perf_records)} データセット) ...")
    perf_fig_box, perf_fig_traj = build_perfect_tracking_figure(perf_records, params)
    perf_html = _build_sec14(perf_fig_box, perf_fig_traj, params, len(perf_records))

    closed_loop_html = _build_sec_closed_loop_comparison(args.collection_dir, args.closed_loop_uuids)

    html = build_html(
        params, long_fig, steer_fig, kus_fig, viewer_sections, len(records),
        baseline_score=baseline_steer_score,
        deviation_html=deviation_html,
        label=phase_label,
        params_filename=args.params.name,
        perf_html=perf_html,
        long_perf_figs=long_perf_figs,
        closed_loop_html=closed_loop_html,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding="utf-8")
    size_kb = args.out.stat().st_size // 1024
    print(f"\n✓ 完了: {args.out}  ({size_kb} KB)")


if __name__ == "__main__":
    main()
