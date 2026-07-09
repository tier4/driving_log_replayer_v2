"""Acceleration source helpers shared by reidentify and report evaluation."""

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
})

ACCEL_DELAY_MAP: dict[str, float] = {
    "accel": 0.080,
    "kinematic_diff": 0.0,
    "velocity_diff": 0.0,
    "kinematic_savgol": 0.0,
    "velocity_savgol": 0.0,
}

ACCEL_SAVGOL_WINDOW_S = 0.2
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


def _savgol_derivative(values: np.ndarray, dt: float) -> np.ndarray:
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
        deriv = _savgol_derivative(values, max(dt, 1e-6))
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
        return _savgol_derivative(vx_grid, dt)

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


def rollout_data_with_accel_source(data: dict[str, pd.DataFrame], source: str) -> dict[str, pd.DataFrame]:
    """Return rollout data with data['acc'] replaced by the selected source."""
    source = normalize_accel_source(source)
    if source == "accel":
        return data
    acc = accel_dataframe_from_source(
        source,
        df_accel=data["acc"],
        df_vel=data["vel"],
        df_kin=data["kin"],
        accel_col="ax",
        velocity_col="vx",
        kin_vx_col="vx",
        out_col="ax",
    )
    acc["ay"] = 0.0
    out = dict(data)
    out["acc"] = acc[["t_ns", "ax", "ay"]]
    return out
