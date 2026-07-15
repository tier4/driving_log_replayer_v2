"""Steering GT source helpers used by reidentify.

acceleration_source と対になる steering_source を提供する。"steer" は
steering_status の生値、"steer_savgol" はゼロ位相の savgol 平滑化 (deriv=0) を
掛けた値。プラトー分析 (2026-07) より steer 残差は低周波支配で平滑化の効果は
小さい (-2% 程度) が、p 系比較モデルの GT を ax 側 (kinematic_savgol) と
一貫させるために設ける。
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

from ._accel_source import _savgol_window

VALID_STEER_SOURCES: frozenset[str] = frozenset({"steer", "steer_savgol"})

# 平滑化窓。0.4 s (polyorder=2, steering 50 Hz) の -3dB カットオフは約 2.55 Hz で、
# steer アクチュエータ帯域 1/(2πτ)≈1.06 Hz の ~2.4 倍を確保 (ax 側の窓 0.4 s と同じ設計比)。
STEER_SAVGOL_WINDOW_S = 0.4
STEER_SAVGOL_POLYORDER = 2


def normalize_steer_source(value: Any, *, default: str = "steer") -> str:
    source = str(value or default)
    if source not in VALID_STEER_SOURCES:
        raise ValueError(
            f"Unknown steering_source={source!r}; supported={sorted(VALID_STEER_SOURCES)}"
        )
    return source


def steer_dataframe_from_source(
    source: str,
    *,
    df_steer: pd.DataFrame,
    steer_col: str = "steer",
) -> pd.DataFrame:
    """Return the steering dataframe for the selected source (t_ns + steer_col)."""
    source = normalize_steer_source(source)
    if source == "steer":
        return df_steer
    values = df_steer[steer_col].to_numpy(dtype=float)
    if len(values) < 2:
        return df_steer
    t_s = df_steer["t_ns"].to_numpy(dtype=np.int64).astype(float) * 1e-9
    dt = max(float(np.median(np.diff(t_s))), 1e-6)
    win = _savgol_window(len(values), dt, STEER_SAVGOL_WINDOW_S, STEER_SAVGOL_POLYORDER)
    if win is None:
        return df_steer
    smoothed = df_steer.copy()
    smoothed[steer_col] = savgol_filter(values, win, STEER_SAVGOL_POLYORDER)
    return smoothed
