"""実機走行ログの特性カバレッジ集計 (純関数・ROS 非依存).

1 データセットの実機ログから「どんな走行条件を含むか」(速度域・加減速・曲率域・走行距離)
を集計する。step4 が real ログ 1 本分を計算して metrics_closed_loop.json の real.coverage に
格納し、step13 がデータセット横断で並べて評価データの偏り (低速ばかり・直線ばかり等) を
可視化する。

速度域ビンは step5 の速度域別 RMSE 集計 ((0,2),(2,5),(5,8),(8,∞) m/s) と同一にし、
誤差解析とカバレッジ解析の軸を揃える。
"""

from __future__ import annotations

import numpy as np
import pandas as pd

# 速度域ビン [m/s] (step5_analyze_nstep の速度域別 RMSE と同一)
SPEED_BINS: list[tuple[float, float]] = [(0, 2), (2, 5), (5, 8), (8, 100)]

# 曲率半径ビン [m]。|κ| 閾値は 1/R。カーブイベント判定 (R<50 相当) と整合させる。
CURVATURE_BINS: list[tuple[str, float, float]] = [
    ("R>100", 0.0, 1 / 100),
    ("R=50-100", 1 / 100, 1 / 50),
    ("R=25-50", 1 / 50, 1 / 25),
    ("R<25", 1 / 25, np.inf),
]

_CURVE_KAPPA = 1 / 50  # カーブイベント閾値 |κ| > 0.02 (R<50)
_CURVE_MIN_ARC = 10.0  # カーブイベントの最小連続弧長 [m]
_V_STOP = 0.3          # 停止判定速度 [m/s]
_V_KAPPA_MIN = 0.5     # 曲率計算の低速マスク (κ=wz/v が低速で暴発するため)

_AX_PERCENTILES = (5, 25, 50, 75, 95)


def _speed_bin_label(lo: float, hi: float) -> str:
    return f"{lo}-{hi}" if hi < 100 else f"{lo}+"


def _time_weights(t: np.ndarray) -> np.ndarray:
    """各サンプルの滞在時間重み [s] (前進差分、末尾は直前値を流用)。"""
    if len(t) < 2:
        return np.ones(len(t))
    dt = np.diff(t)
    return np.append(dt, dt[-1])


def _stop_count(v: np.ndarray) -> int:
    """停止 (v≤_V_STOP) → 走行 の遷移回数 = 発進イベント数。"""
    stopped = v <= _V_STOP
    if len(stopped) < 2:
        return 0
    return int(np.sum(stopped[:-1] & ~stopped[1:]))


def compute_coverage(
    kin: pd.DataFrame,
    vel: pd.DataFrame,
    acc: pd.DataFrame,
    steer: pd.DataFrame,  # noqa: ARG001  (将来のステア域集計用に契約へ含める)
    *,
    wheelbase: float,  # noqa: ARG001
) -> dict:
    """実機ログ 1 本の走行特性カバレッジを集計する。

    kin/vel/acc/steer は align_time 済み DataFrame (kin は filter_localization 済み)。
    返り値はすべて JSON 化可能なプリミティブ (NaN は生成しない)。
    """
    cov: dict = {}

    # --- 走行距離・時間・停止 ---
    if not kin.empty:
        x, y = kin["x"].to_numpy(float), kin["y"].to_numpy(float)
        seg = np.hypot(np.diff(x), np.diff(y))
        cov["dist_m"] = float(seg.sum())
    else:
        cov["dist_m"] = 0.0

    if not vel.empty:
        t_v = vel["t"].to_numpy(float)
        v = vel["lon_vel"].to_numpy(float)
        w = _time_weights(t_v)
        cov["duration_s"] = float(t_v.max() - t_v.min())
        cov["stop_count"] = _stop_count(v)
        cov["v_max_mps"] = float(np.nanmax(v))
        total_w = float(w.sum()) or 1.0
        cov["speed_bins"] = {
            _speed_bin_label(lo, hi): float(w[(v >= lo) & (v < hi)].sum() / total_w)
            for lo, hi in SPEED_BINS
        }
    else:
        cov["duration_s"] = 0.0
        cov["stop_count"] = 0
        cov["v_max_mps"] = 0.0
        cov["speed_bins"] = {_speed_bin_label(lo, hi): 0.0 for lo, hi in SPEED_BINS}

    # --- 加減速分布 ---
    if not acc.empty:
        ax = acc["accel"].to_numpy(float)
        ax = ax[np.isfinite(ax)]
    else:
        ax = np.empty(0)
    cov["ax_percentiles"] = {
        f"p{p}": (float(np.percentile(ax, p)) if len(ax) else 0.0) for p in _AX_PERCENTILES
    }

    # --- 曲率域 (kinematic yaw 由来。κ = yaw_rate / v、低速マスク) ---
    cov["curvature"] = _curvature_coverage(kin, vel)

    return cov


def _curvature_coverage(kin: pd.DataFrame, vel: pd.DataFrame) -> dict:
    """曲率半径ビン別の走行距離・カーブイベント数を集計する。"""
    empty = {
        "kappa_max_abs": 0.0,
        "bins_dist_m": {label: 0.0 for label, _, _ in CURVATURE_BINS},
        "curve_count": 0,
    }
    if kin.empty or vel.empty or len(kin) < 3:
        return empty

    t_k = kin["t"].to_numpy(float)
    yaw_rate = np.gradient(np.unwrap(kin["yaw"].to_numpy(float)), t_k)
    v_i = np.interp(t_k, vel["t"].to_numpy(float), vel["lon_vel"].to_numpy(float))
    valid = v_i > _V_KAPPA_MIN
    kappa = np.zeros_like(t_k)
    kappa[valid] = yaw_rate[valid] / v_i[valid]
    abs_k = np.abs(kappa)

    # サンプルごとの走行弧長 (v·dt)。低速マスク外は直進扱い (κ=0 で R>100 ビンに入る)
    ds = v_i * _time_weights(t_k)
    bins_dist = {
        label: float(ds[(abs_k >= lo) & (abs_k < hi)].sum())
        for label, lo, hi in CURVATURE_BINS
    }

    # カーブイベント: |κ|>閾値 の連続区間で弧長 ≥ _CURVE_MIN_ARC のもの
    curve_count = 0
    in_curve = abs_k > _CURVE_KAPPA
    arc = 0.0
    for i in range(len(t_k)):
        if in_curve[i]:
            arc += ds[i]
        elif arc > 0:
            if arc >= _CURVE_MIN_ARC:
                curve_count += 1
            arc = 0.0
    if arc >= _CURVE_MIN_ARC:
        curve_count += 1

    return {
        "kappa_max_abs": float(abs_k.max()),
        "bins_dist_m": bins_dist,
        "curve_count": curve_count,
    }
