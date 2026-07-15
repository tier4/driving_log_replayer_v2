"""Small numerical helpers for reidentify rollout evaluation."""
from __future__ import annotations

import numpy as np
import pandas as pd

# N-step rollout error metrics, in the column order used by metrics.csv.
METRIC_KEYS: tuple[str, ...] = ("pos", "long", "lat", "yaw", "steer", "vx", "ax")


def to_seconds(frame: pd.DataFrame, t0_ns: int) -> pd.DataFrame:
    result = frame.copy()
    result["t"] = (result["t_ns"].astype(np.float64) - float(t0_ns)) * 1.0e-9
    return result


def interp_or_zeros(
    target_t: np.ndarray,
    source_t: np.ndarray,
    source_values: np.ndarray,
) -> np.ndarray:
    if len(source_t) == 0:
        return np.zeros_like(target_t, dtype=float)
    return np.interp(target_t, source_t, source_values)


def local_ds(
    dx: np.ndarray,
    dy: np.ndarray,
    cos_yaw: np.ndarray,
    sin_yaw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    return dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.asarray(values, dtype=float) ** 2)))
