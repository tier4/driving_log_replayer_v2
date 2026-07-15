"""steering_source (steer | steer_savgol) の単体テスト。"""
from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib._steer_source import (
    VALID_STEER_SOURCES,
    normalize_steer_source,
    steer_dataframe_from_source,
)


def _steer_df(n: int = 500, dt_s: float = 0.02, noise: float = 0.01) -> pd.DataFrame:
    rng = np.random.default_rng(7)
    t_ns = (np.arange(n) * dt_s * 1e9).astype(np.int64)
    t = np.arange(n) * dt_s
    signal = 0.1 * np.sin(2 * np.pi * 0.2 * t)  # 帯域内 (0.2 Hz) の実操舵
    return pd.DataFrame({"t_ns": t_ns, "steer": signal + rng.normal(0.0, noise, n)})


def test_normalize_steer_source_accepts_valid_and_rejects_unknown() -> None:
    assert normalize_steer_source(None) == "steer"
    assert normalize_steer_source("steer_savgol") == "steer_savgol"
    with pytest.raises(ValueError, match="steering_source"):
        normalize_steer_source("lowpass")
    assert VALID_STEER_SOURCES == {"steer", "steer_savgol"}


def test_raw_source_returns_dataframe_unchanged() -> None:
    df = _steer_df()
    assert steer_dataframe_from_source("steer", df_steer=df) is df


def test_savgol_source_smooths_noise_and_preserves_signal() -> None:
    df = _steer_df()
    smoothed = steer_dataframe_from_source("steer_savgol", df_steer=df)

    assert smoothed is not df  # 入力は改変しない
    assert (smoothed["t_ns"].values == df["t_ns"].values).all()
    t = df["t_ns"].values * 1e-9
    truth = 0.1 * np.sin(2 * np.pi * 0.2 * t)
    rms_raw = float(np.sqrt(np.mean((df["steer"].values - truth) ** 2)))
    rms_smooth = float(np.sqrt(np.mean((smoothed["steer"].values - truth) ** 2)))
    # ゼロ位相平滑化: 雑音を大きく低減しつつ帯域内信号を保つ。
    assert rms_smooth < rms_raw * 0.5


def test_savgol_source_passes_through_short_series() -> None:
    df = _steer_df(n=1)
    result = steer_dataframe_from_source("steer_savgol", df_steer=df)
    assert (result["steer"].values == df["steer"].values).all()
