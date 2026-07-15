"""Physical-validity calculations on the current reidentify cache contract.

The old report consumed ``real.lite`` MCAPs directly.  This module deliberately
works on the resampled dictionaries returned by :mod:`reidentify.load_data`,
so the numerical definitions remain usable without ROS or rosbag imports.
"""
from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
from scipy.signal import savgol_filter

_FIT_DT = 0.01
WHEELBASE = 4.76012
_XY_HEADING_RATE_COEFF_BOUNDS = (-5.0, 5.0)


def _finite_mask(*arrays: np.ndarray) -> np.ndarray:
    mask = np.ones(len(arrays[0]), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(array)
    return mask


def _savgol_derivative(values: np.ndarray, dt: float = _FIT_DT) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if len(values) < 5:
        return np.gradient(values, dt) if len(values) > 1 else np.zeros_like(values)
    window = min(len(values) if len(values) % 2 else len(values) - 1, 21)
    window = max(window, 5)
    return savgol_filter(values, window, 2, deriv=1, delta=dt, mode="interp")


def _simulate_first_order(cmd: np.ndarray, tau: float, delay: float, dt: float) -> np.ndarray:
    delayed = np.asarray(cmd, dtype=float).copy()
    samples = max(0, int(round(float(delay) / dt)))
    if samples:
        delayed[samples:] = delayed[:-samples]
        delayed[:samples] = delayed[0]
    alpha = min(1.0, dt / max(float(tau), 1e-9))
    out = np.empty_like(delayed)
    if len(out) == 0:
        return out
    out[0] = alpha * delayed[0]
    for i in range(1, len(out)):
        out[i] = out[i - 1] + alpha * (delayed[i] - out[i - 1])
    return out


def yaw_residual(dataset: dict, *, k_us: float, wheelbase: float = WHEELBASE) -> np.ndarray:
    vx, wz, steer = dataset["vx"], dataset["wz"], dataset["d_act"]
    rhs = vx * np.tan(np.clip(steer, -0.8, 0.8)) / (wheelbase + k_us * vx * vx)
    mask = dataset["gear_drive"] & (vx > 1.5) & np.isfinite(rhs) & np.isfinite(wz)
    return (rhs - wz)[mask]


def build_xy_columns(
    dataset: dict[str, Any], source: dict[str, pd.DataFrame], *, dt: float = _FIT_DT,
) -> float:
    """kinematic トピックから x,y,yaw,vx,wz を dataset の時間グリッドへ補間し、``dataset["xy"]`` に積む。

    ``dataset`` は :func:`reidentify.load_data.build_resampled` の戻り値 (等間隔グリッド上の
    a_cmd/vx/... を持つ dict)、``source`` は :func:`reidentify.load_data.read_dataset_csv` の
    戻り値 (topic 別 raw DataFrame) を想定する。戻り値は共通開始時刻 ``t0`` [ns]。
    """
    kin = source["kinematic"]
    if kin.empty:
        raise ValueError("kinematic が空です")
    t0 = max(
        float(source[topic]["t_ns"].iloc[0])
        for topic in ("cmd", "accel", "steering", "velocity", "kinematic")
    )
    t_grid = t0 + np.arange(len(dataset["vx"]), dtype=float) * dt * 1e9
    source_t = kin["t_ns"].to_numpy(dtype=float)
    dataset["xy"] = tuple(
        np.interp(t_grid, source_t, kin[column].to_numpy(dtype=float))
        for column in ("x", "y", "yaw", "vx", "wz")
    )
    return t0


def xy_residual(dataset: dict, coeff: float) -> tuple[np.ndarray, np.ndarray]:
    x, y, yaw, vx, wz = (np.asarray(value, dtype=float) for value in dataset["xy"])
    lhs_x, lhs_y = _savgol_derivative(x), _savgol_derivative(y)
    effective_yaw = yaw - coeff * vx * wz
    mask = dataset["gear_drive"][:len(x)] & (vx > 0.5) & _finite_mask(lhs_x, lhs_y, yaw, vx, wz)
    return (vx * np.cos(effective_yaw) - lhs_x)[mask], (vx * np.sin(effective_yaw) - lhs_y)[mask]


def fit_xy_heading_rate_coeff(datasets: list[dict], initial: float = 0.0) -> dict:
    def objective(coeff: float) -> float:
        parts = [np.concatenate(xy_residual(ds, coeff)) for ds in datasets]
        values = np.concatenate([part for part in parts if len(part)]) if any(len(p) for p in parts) else np.empty(0)
        return float(np.mean(values * values)) if len(values) else float("inf")
    result = minimize_scalar(objective, bounds=_XY_HEADING_RATE_COEFF_BOUNDS, method="bounded")
    coeff = float(result.x) if result.success else float(initial)
    parts = [np.concatenate(xy_residual(ds, coeff)) for ds in datasets]
    values = np.concatenate([part for part in parts if len(part)]) if any(len(p) for p in parts) else np.empty(0)
    return {"xy_heading_rate_coeff": coeff, "rmse": float(np.sqrt(np.mean(values ** 2))) if len(values) else float("nan"), "n": int(len(values))}
