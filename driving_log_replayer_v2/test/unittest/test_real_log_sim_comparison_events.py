# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison._events` の純関数テスト."""

from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib._events import (
    AUTONOMOUS_MODE,
    find_autonomous_start,
    find_curve2_exit,
    find_curve2_launch,
    find_sim_launch,
)


def _vel_df(times_ns: list[int], vels: list[float]) -> pd.DataFrame:
    df = pd.DataFrame({"t_ns": times_ns, "lon_vel": vels})
    df["t"] = (df["t_ns"] - df["t_ns"].iloc[0]) / 1e9
    return df


def test_find_autonomous_start_with_mode_transition() -> None:
    df_mode = pd.DataFrame(
        {
            "t_ns": [0, 100, 200, 300],
            "mode": [0, 0, AUTONOMOUS_MODE, AUTONOMOUS_MODE],
        }
    )
    df_vel = pd.DataFrame({"t_ns": [0, 100, 200, 300], "lon_vel": [0.0, 0.0, 1.0, 2.0]})
    assert find_autonomous_start(df_mode, df_vel) == 200


def test_find_autonomous_start_fallback_to_velocity() -> None:
    df_mode = pd.DataFrame({"t_ns": [0, 100], "mode": [0, 1]})
    df_vel = pd.DataFrame({"t_ns": [0, 100, 200], "lon_vel": [0.0, 0.05, 1.0]})
    with pytest.warns(UserWarning):
        assert find_autonomous_start(df_mode, df_vel) == 200


def test_find_autonomous_start_raises_on_empty() -> None:
    with pytest.raises(ValueError):
        find_autonomous_start(pd.DataFrame(), pd.DataFrame())


def test_find_autonomous_start_velocity_fallback_debounce_ignores_spike() -> None:
    # 単点ノイズスパイク (t=0.5s) を採らず、継続する発進 (t=2.0s) を採るデバウンス。
    df_mode = pd.DataFrame(columns=["t_ns", "mode"])  # mode 無 → 速度 fallback
    ts = [i * 0.1 for i in range(40)]
    vs = [0.0] * 40
    vs[5] = 5.0  # 単点スパイク (t=0.5s)
    for i in range(20, 40):
        vs[i] = 1.0  # t=2.0s 以降は継続
    df_vel = pd.DataFrame({"t_ns": [int(t * 1e9) for t in ts], "lon_vel": vs})
    t0 = find_autonomous_start(df_mode, df_vel, min_moving_duration=0.3)
    assert abs(t0 / 1e9 - 2.0) < 0.05


def test_find_curve2_launch_picks_stop_inside_window() -> None:
    # t=10s 付近に 3s 停止 (window 外) と、t=50s 付近に 2s 停止 (window 内) があるケース
    times = np.linspace(0, 70, 1401)  # 50ms 刻み
    vels = np.full_like(times, 5.0)
    vels[(times >= 9.0) & (times <= 12.0)] = 0.0  # window 外停止
    vels[(times >= 48.0) & (times <= 50.0)] = 0.0  # window 内停止
    df = pd.DataFrame({"t_ns": (times * 1e9).astype(np.int64), "t": times, "lon_vel": vels})
    t_launch = find_curve2_launch(df, window=(20.0, 120.0))
    assert t_launch is not None
    assert abs(t_launch - 50.0) < 0.1


def test_find_curve2_launch_returns_none_when_no_stop_in_window() -> None:
    times = np.linspace(0, 30, 301)
    vels = np.full_like(times, 5.0)
    df = pd.DataFrame({"t_ns": (times * 1e9).astype(np.int64), "t": times, "lon_vel": vels})
    assert find_curve2_launch(df, window=(20.0, 120.0)) is None


def test_find_curve2_exit_first_outside_after_inside() -> None:
    # 円内に居て、ある時刻で円外に出る軌跡
    t = np.linspace(0, 10, 101)
    cx, cy = 0.0, 0.0
    # 半径2の円内→円外へ移動
    x = np.linspace(0.0, 40.0, len(t))
    y = np.zeros_like(t)
    df = pd.DataFrame({"t": t, "x": x, "y": y})
    t_exit = find_curve2_exit(df, (cx, cy), t_launch=0.0, radius=10.0)
    assert t_exit is not None
    assert 2.5 <= t_exit <= 2.6  # x が radius を超える時刻


def test_find_curve2_exit_returns_none_if_never_inside() -> None:
    t = np.linspace(0, 10, 11)
    x = np.linspace(100.0, 200.0, 11)
    y = np.zeros_like(t)
    df = pd.DataFrame({"t": t, "x": x, "y": y})
    assert find_curve2_exit(df, (0.0, 0.0), t_launch=0.0, radius=10.0) is None


def test_find_sim_launch_threshold_and_min_t() -> None:
    times = np.linspace(0, 30, 301)
    vels = np.where(times < 7.0, 0.2, 1.0)  # 7s 以降 > 0.5
    df = pd.DataFrame({"t": times, "lon_vel": vels})
    t = find_sim_launch(df, threshold=0.5, min_t=5.0)
    assert t is not None
    assert abs(t - 7.0) < 0.2


def test_find_sim_launch_returns_none_when_never_moving() -> None:
    df = pd.DataFrame({"t": [0.0, 1.0, 2.0], "lon_vel": [0.0, 0.1, 0.2]})
    assert find_sim_launch(df) is None
