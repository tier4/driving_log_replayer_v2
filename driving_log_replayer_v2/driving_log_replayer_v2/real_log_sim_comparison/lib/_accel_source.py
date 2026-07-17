"""Acceleration source helpers used by reidentify."""

from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


VALID_ACCEL_SOURCES: frozenset[str] = frozenset({
    "accel",
    "kinematic_diff",
    "velocity_diff",
    "kinematic_savgol",
    "velocity_savgol",
    "kinematic_rts",
})

ACCEL_DELAY_MAP: dict[str, float] = {
    "accel": 0.080,
    "kinematic_diff": 0.0,
    "velocity_diff": 0.0,
    "kinematic_savgol": 0.0,
    "velocity_savgol": 0.0,
    "kinematic_rts": 0.0,
}

# kinematic_rts: [v, a] 状態のカルマン往復 (RTS) 平滑による加速度 GT。
# GT 品質診断 (2026-07-17, dev 292 datasets, analysis/gt_quality.csv) で q=0.2 が
# 現行 kinematic_savgol (窓 0.4 s) を「積分整合性 (brake T=0.5/1 s で −13〜15%) ×
# ノイズフロア (同等 0.0144 vs 0.0142 m/s²)」の両軸で支配したため採用。
# savgol 微分と異なり群遅延・窓端減衰がなく、ブレーキ過渡のなまり損失が小さい。
KINEMATIC_RTS_Q_JERK = 0.2
KINEMATIC_RTS_R_V_STD = 0.03  # 速度観測ノイズ std [m/s]

# savgol 微分の窓長。0.4 s (polyorder=2, 100 Hz) の実効カットオフは理想微分器比 -3dB で
# 約 1.4 Hz。旧 0.2 s (カットオフ 2.76 Hz) + 2 Hz ゼロ位相 LPF のカスケードと同等の
# 雑音フロア低減 (ax 残差 RMS 0.215→0.201, 12 dataset 実測) を別フィルタなしで窓に統合した値。
# acc アクチュエータ帯域 1/(2πτ)≈0.53 Hz の ~2.7 倍を確保し過渡同定 (fit_lon) を損なわない。
ACCEL_SAVGOL_WINDOW_S = 0.4
ACCEL_SAVGOL_POLYORDER = 2


def normalize_accel_source(value: Any, *, default: str = "accel") -> str:
    source = str(value or default)
    if source not in VALID_ACCEL_SOURCES:
        raise ValueError(f"Unknown acceleration_source={source!r}; supported={sorted(VALID_ACCEL_SOURCES)}")
    return source


def _savgol_window(n: int, dt: float, window_s: float, polyorder: int) -> int | None:
    win = int(round(window_s / dt))
    if win % 2 == 0:
        win += 1
    if win <= polyorder:
        win = polyorder + 1 if (polyorder + 1) % 2 == 1 else polyorder + 2
    if win > n:
        win = n if n % 2 == 1 else n - 1
    if win <= polyorder:
        return None
    return win


def savgol_derivative(values: np.ndarray, dt: float) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    win = _savgol_window(len(values), dt, ACCEL_SAVGOL_WINDOW_S, ACCEL_SAVGOL_POLYORDER)
    if win is None:
        return np.gradient(values, dt)
    return savgol_filter(
        values,
        window_length=win,
        polyorder=ACCEL_SAVGOL_POLYORDER,
        deriv=1,
        delta=dt,
    )


def rts_smooth_accel(
    vx: np.ndarray,
    dt: float,
    q_jerk: float = KINEMATIC_RTS_Q_JERK,
    r_v_std: float = KINEMATIC_RTS_R_V_STD,
) -> np.ndarray:
    """速度系列から [v, a] 状態のカルマン往復 (RTS) 平滑で加速度を推定する。

    process: v' = a, a' = w (ジャーク白色雑音、PSD q_jerk)。観測: vx = v + n。
    等間隔 dt を仮定する (非一様な場合は呼び出し側で補間する)。
    """
    vx = np.asarray(vx, dtype=float)
    n = len(vx)
    if n < 3:
        return np.gradient(vx, dt) if n > 1 else np.zeros_like(vx)

    f_mat = np.array([[1.0, dt], [0.0, 1.0]])
    q_mat = q_jerk * np.array([
        [dt**3 / 3.0, dt**2 / 2.0],
        [dt**2 / 2.0, dt],
    ])
    h_mat = np.array([[1.0, 0.0]])
    r_var = r_v_std**2

    x_pred = np.zeros((n, 2))
    p_pred = np.zeros((n, 2, 2))
    x_filt = np.zeros((n, 2))
    p_filt = np.zeros((n, 2, 2))

    x = np.array([vx[0], 0.0])
    p = np.diag([r_var, 4.0])
    for k in range(n):
        if k > 0:
            x = f_mat @ x
            p = f_mat @ p @ f_mat.T + q_mat
        x_pred[k], p_pred[k] = x, p
        innovation = vx[k] - x[0]
        s = p[0, 0] + r_var
        gain = p[:, 0] / s
        x = x + gain * innovation
        p = p - np.outer(gain, p[0, :])
        x_filt[k], p_filt[k] = x, p

    x_smooth = x_filt.copy()
    for k in range(n - 2, -1, -1):
        c = p_filt[k] @ f_mat.T @ np.linalg.inv(p_pred[k + 1])
        x_smooth[k] = x_filt[k] + c @ (x_smooth[k + 1] - x_pred[k + 1])
    return x_smooth[:, 1]


def _native_rts_df(df: pd.DataFrame, *, value_col: str, out_col: str) -> pd.DataFrame:
    """ネイティブ時刻列の速度から RTS 平滑加速度の DataFrame を作る。

    RTS は等間隔前提のため、中央値 dt の一様グリッドへ補間して平滑し、
    元の時刻へ補間して返す。
    """
    t_ns = df["t_ns"].to_numpy(dtype=np.int64)
    values = df[value_col].to_numpy(dtype=float)
    if len(values) < 3:
        return _native_savgol_df(df, value_col=value_col, out_col=out_col)
    t_s = (t_ns - t_ns[0]).astype(float) * 1e-9
    dt = float(np.median(np.diff(t_s)))
    if not np.isfinite(dt) or dt <= 0.0:
        return _native_savgol_df(df, value_col=value_col, out_col=out_col)
    t_grid = np.arange(t_s[0], t_s[-1] + dt * 0.5, dt)
    accel_grid = rts_smooth_accel(np.interp(t_grid, t_s, values), dt)
    return pd.DataFrame({"t_ns": t_ns, out_col: np.interp(t_s, t_grid, accel_grid)})


def _native_diff_df(
    df: pd.DataFrame,
    *,
    value_col: str,
    out_col: str,
    rolling_window: int = 10,
) -> pd.DataFrame:
    t_ns = df["t_ns"].to_numpy(dtype=np.int64)
    values = df[value_col].to_numpy(dtype=float)
    t_s = t_ns.astype(float) * 1e-9
    sample_dt = np.diff(t_s)
    dv = np.diff(values)
    raw_accel = np.zeros_like(values, dtype=float)
    raw_accel[1:] = dv / np.maximum(sample_dt, 1e-6)
    smooth_accel = (
        pd.Series(raw_accel)
        .rolling(window=rolling_window, min_periods=1, center=True)
        .mean()
        .to_numpy(dtype=float)
    )
    return pd.DataFrame({"t_ns": t_ns, out_col: smooth_accel})


def _native_savgol_df(df: pd.DataFrame, *, value_col: str, out_col: str) -> pd.DataFrame:
    t_ns = df["t_ns"].to_numpy(dtype=np.int64)
    values = df[value_col].to_numpy(dtype=float)
    if len(values) < 2:
        deriv = np.zeros_like(values, dtype=float)
    else:
        t_s = t_ns.astype(float) * 1e-9
        dt = float(np.median(np.diff(t_s)))
        deriv = savgol_derivative(values, max(dt, 1e-6))
    return pd.DataFrame({"t_ns": t_ns, out_col: deriv})


def accel_dataframe_from_source(
    source: str,
    *,
    df_accel: pd.DataFrame,
    df_vel: pd.DataFrame,
    df_kin: pd.DataFrame,
    accel_col: str = "accel",
    velocity_col: str = "lon_vel",
    kin_vx_col: str = "vx",
    out_col: str = "accel",
) -> pd.DataFrame:
    """Return a two-column acceleration dataframe: t_ns and out_col."""
    source = normalize_accel_source(source)
    if source == "accel":
        return df_accel[["t_ns", accel_col]].rename(columns={accel_col: out_col}).reset_index(drop=True)
    if source == "kinematic_diff":
        return _native_diff_df(df_kin, value_col=kin_vx_col, out_col=out_col)
    if source == "velocity_diff":
        return _native_diff_df(df_vel, value_col=velocity_col, out_col=out_col)
    if source == "kinematic_savgol":
        return _native_savgol_df(df_kin, value_col=kin_vx_col, out_col=out_col)
    if source == "velocity_savgol":
        return _native_savgol_df(df_vel, value_col=velocity_col, out_col=out_col)
    if source == "kinematic_rts":
        return _native_rts_df(df_kin, value_col=kin_vx_col, out_col=out_col)
    raise AssertionError(source)


def accel_on_grid(
    source: str,
    *,
    df_accel: pd.DataFrame,
    df_vel: pd.DataFrame,
    df_kin: pd.DataFrame,
    t_s: np.ndarray,
    t0_ns: int | float,
    dt: float,
    accel_col: str = "accel",
    velocity_col: str = "lon_vel",
    kin_vx_col: str = "vx",
) -> np.ndarray:
    """Evaluate the selected acceleration source on an existing relative time grid."""
    source = normalize_accel_source(source)
    if source in {"kinematic_savgol", "velocity_savgol"}:
        src_df = df_kin if source == "kinematic_savgol" else df_vel
        value_col = kin_vx_col if source == "kinematic_savgol" else velocity_col
        vx_grid = np.interp(t_s, (src_df["t_ns"].to_numpy(dtype=float) - t0_ns) * 1e-9, src_df[value_col].to_numpy(dtype=float))
        return savgol_derivative(vx_grid, dt)
    if source == "kinematic_rts":
        vx_grid = np.interp(
            t_s, (df_kin["t_ns"].to_numpy(dtype=float) - t0_ns) * 1e-9,
            df_kin[kin_vx_col].to_numpy(dtype=float),
        )
        return rts_smooth_accel(vx_grid, dt)

    acc_df = accel_dataframe_from_source(
        source,
        df_accel=df_accel,
        df_vel=df_vel,
        df_kin=df_kin,
        accel_col=accel_col,
        velocity_col=velocity_col,
        kin_vx_col=kin_vx_col,
        out_col="accel",
    )
    return np.interp(
        t_s,
        (acc_df["t_ns"].to_numpy(dtype=float) - t0_ns) * 1e-9,
        acc_df["accel"].to_numpy(dtype=float),
    )
