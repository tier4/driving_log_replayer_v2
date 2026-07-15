"""AUTONOMOUS driving start detection for reidentify rollouts."""
from __future__ import annotations

import warnings

import numpy as np
import pandas as pd

AUTONOMOUS_MODE = 2


def _moving_start(
    velocity: pd.DataFrame,
    threshold: float,
    minimum_duration: float,
) -> int:
    timestamps = velocity["t_ns"].to_numpy()
    moving = velocity["lon_vel"].to_numpy() > threshold
    window_ns = minimum_duration * 1.0e9
    for index in np.flatnonzero(moving):
        in_window = (timestamps >= timestamps[index]) & (
            timestamps < timestamps[index] + window_ns
        )
        if in_window.sum() >= 2 and moving[in_window].all():
            return int(timestamps[index])
    if moving.any():
        return int(timestamps[int(np.argmax(moving))])
    return int(timestamps[0])


def find_autonomous_start(
    operation_mode: pd.DataFrame,
    velocity: pd.DataFrame,
    *,
    vel_threshold: float = 0.1,
    min_moving_duration: float = 0.3,
) -> int:
    """Return the later of first sustained motion and first AUTONOMOUS mode."""
    if velocity.empty:
        raise ValueError("velocity data is empty; cannot determine rollout start")
    motion_start = _moving_start(velocity, vel_threshold, min_moving_duration)
    if operation_mode.empty:
        mode_start = int(velocity["t_ns"].iloc[0])
    else:
        autonomous = operation_mode[operation_mode["mode"] == AUTONOMOUS_MODE]
        if autonomous.empty:
            warnings.warn(
                "AUTONOMOUS mode was not found; using velocity start only",
                stacklevel=2,
            )
            mode_start = int(velocity["t_ns"].iloc[0])
        else:
            mode_start = int(autonomous["t_ns"].iloc[0])
    return max(motion_start, mode_start)
