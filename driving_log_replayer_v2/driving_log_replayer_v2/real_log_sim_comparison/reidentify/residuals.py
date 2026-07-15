"""Model-equation residuals evaluated on resampled reidentify cache datasets.

These helpers work on the dictionaries returned by :mod:`reidentify.load_data`,
so the numerical definitions remain usable without ROS or rosbag imports. They
are shared between the direct-fit stages and the report's fixed evaluation.
"""
from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd

from ..lib._accel_source import savgol_derivative
from ..lib._nstep_common import rms
from .physical_constants import VX_MIN_CURVE
from .settings import KINEMATIC_STEER_VX_MIN, RESAMPLE_DT, STEER_CLIP_RAD


def rmse(values: np.ndarray) -> float:
    """Root mean square of ``values``; NaN when empty."""
    return rms(values) if len(values) else float("nan")


def _finite_mask(*arrays: np.ndarray) -> np.ndarray:
    mask = np.ones(len(arrays[0]), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(array)
    return mask


def yaw_residual(dataset: dict, *, k_us: float, wheelbase: float) -> np.ndarray:
    vx, wz, steer = dataset["vx"], dataset["wz"], dataset["d_act"]
    rhs = vx * np.tan(np.clip(steer, -STEER_CLIP_RAD, STEER_CLIP_RAD)) / (wheelbase + k_us * vx * vx)
    mask = dataset["gear_drive"] & (vx > VX_MIN_CURVE) & np.isfinite(rhs) & np.isfinite(wz)
    return (rhs - wz)[mask]


def build_xy_columns(
    dataset: dict[str, Any], source: dict[str, pd.DataFrame], *, dt: float = RESAMPLE_DT,
) -> None:
    """kinematic トピックから x,y,yaw,vx,wz を dataset の時間グリッドへ補間し、``dataset["xy"]`` に積む。

    ``dataset`` は :func:`reidentify.load_data.build_resampled` の戻り値 (等間隔グリッド上の
    a_cmd/vx/... を持つ dict)、``source`` は :func:`reidentify.load_data.read_dataset_csv` の
    戻り値 (topic 別 raw DataFrame) を想定する。
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


def xy_residual(dataset: dict, coeff: float) -> tuple[np.ndarray, np.ndarray]:
    x, y, yaw, vx, wz = (np.asarray(value, dtype=float) for value in dataset["xy"])
    lhs_x, lhs_y = savgol_derivative(x, RESAMPLE_DT), savgol_derivative(y, RESAMPLE_DT)
    effective_yaw = yaw - coeff * vx * wz
    mask = dataset["gear_drive"][:len(x)] & (vx > KINEMATIC_STEER_VX_MIN) & _finite_mask(lhs_x, lhs_y, yaw, vx, wz)
    return (vx * np.cos(effective_yaw) - lhs_x)[mask], (vx * np.sin(effective_yaw) - lhs_y)[mask]
