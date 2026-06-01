# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison.lib._io` の純関数テスト (arc-length / localization 有効性ゲート)."""

from __future__ import annotations

import numpy as np
import pandas as pd

from driving_log_replayer_v2.real_log_sim_comparison.lib._io import (
    cumulative_arc_length,
    filter_localization,
)


def _kin(xs: list[float], ys: list[float], dt: float = 0.02) -> pd.DataFrame:
    n = len(xs)
    return pd.DataFrame(
        {"t_ns": [int(i * dt * 1e9) for i in range(n)], "x": xs, "y": ys}
    )


# --- cumulative_arc_length ---


def test_arc_length_straight_line() -> None:
    s = cumulative_arc_length([0.0, 1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(s, [0.0, 1.0, 2.0, 3.0])


def test_arc_length_empty() -> None:
    assert len(cumulative_arc_length([], [])) == 0


def test_arc_length_diagonal() -> None:
    s = cumulative_arc_length([0.0, 3.0], [0.0, 4.0])
    np.testing.assert_allclose(s, [0.0, 5.0])


# --- filter_localization ---


def test_filter_localization_clean_noop() -> None:
    # 実機相当のクリーンな連続軌跡 → 1 件も除外しない。
    xs = [90000.0 + i * 0.1 for i in range(50)]
    ys = [43000.0 + i * 0.05 for i in range(50)]
    clean, dropped = filter_localization(_kin(xs, ys))
    assert dropped == 0
    assert len(clean) == 50


def test_filter_localization_drops_origin_frames() -> None:
    # 先頭に原点(0,0) フレームが数個混入 → それらを除外。
    xs = [0.0, 0.0, 0.0] + [90000.0 + i * 0.1 for i in range(20)]
    ys = [0.0, 0.0, 0.0] + [43000.0 + i * 0.1 for i in range(20)]
    clean, dropped = filter_localization(_kin(xs, ys))
    assert dropped == 3
    assert not ((clean["x"].abs() < 1.0) & (clean["y"].abs() < 1.0)).any()


def test_filter_localization_drops_teleport_with_dt() -> None:
    # 途中に 1 点だけ遠方へ瞬間移動 (dt>0, 速度ゲートで除外)。
    xs = [90000.0 + i for i in range(10)]
    ys = [43000.0] * 10
    xs[5] = 200000.0  # teleport
    clean, dropped = filter_localization(_kin(xs, ys, dt=0.1))
    assert dropped == 1
    assert 200000.0 not in clean["x"].to_numpy()


def test_filter_localization_drops_same_timestamp_jump() -> None:
    # 同一タイムスタンプ (dt==0) の teleport は max_step_jump で除外。
    df = pd.DataFrame({
        "t_ns": [0, 0, int(0.02e9), int(0.04e9)],
        "x": [90000.0, 95000.0, 90000.1, 90000.2],  # index1 が同時刻 5000m 跳躍
        "y": [43000.0, 43000.0, 43000.0, 43000.0],
    })
    clean, dropped = filter_localization(df)
    assert dropped == 1
    assert 95000.0 not in clean["x"].to_numpy()


def test_filter_localization_empty() -> None:
    clean, dropped = filter_localization(pd.DataFrame(columns=["t_ns", "x", "y"]))
    assert dropped == 0
    assert clean.empty


def test_filter_localization_single_point() -> None:
    clean, dropped = filter_localization(_kin([90000.0], [43000.0]))
    assert dropped == 0
    assert len(clean) == 1
