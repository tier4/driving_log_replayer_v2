# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison.step2_bag_to_scenario._select_apex_indices` のテスト (D0 周回 waypoint)."""

from __future__ import annotations

from driving_log_replayer_v2.real_log_sim_comparison.step2_bag_to_scenario import (
    _build_initial_signal_actions,
    _build_signal_story_acts,
    _select_apex_indices,
)


def _triangle_loop() -> tuple[list[float], list[float]]:
    # (0,0)→(10,0) の弦に対し中央 (idx5) で y=5 まで膨らむ三角経路。
    xs = [float(i) for i in range(11)]
    ys = [float(min(i, 10 - i)) for i in range(11)]
    return xs, ys


def test_apex_n1_picks_bulge() -> None:
    xs, ys = _triangle_loop()
    idxs = _select_apex_indices(xs, ys, 0.0, 0.0, 10.0, 0.0, 1)
    assert idxs == [5]  # 弦からの垂直距離が最大の中央点


def test_apex_n2_monotonic_order() -> None:
    xs, ys = _triangle_loop()
    idxs = _select_apex_indices(xs, ys, 0.0, 0.0, 10.0, 0.0, 2)
    assert len(idxs) == 2
    assert idxs == sorted(idxs)  # 経路順 (区間分割なので単調)


def test_apex_zero_n_returns_empty() -> None:
    xs, ys = _triangle_loop()
    assert _select_apex_indices(xs, ys, 0.0, 0.0, 10.0, 0.0, 0) == []


def test_apex_too_few_points() -> None:
    assert _select_apex_indices([0.0, 1.0], [0.0, 1.0], 0.0, 0.0, 1.0, 1.0, 1) == []


def test_apex_degenerate_chord_returns_empty() -> None:
    xs, ys = _triangle_loop()
    # start == goal (弦長ゼロ) → 解決不能で []
    assert _select_apex_indices(xs, ys, 0.0, 0.0, 0.0, 0.0, 1) == []


# ---------------------------------------------------------------------------
# traffic_signals: replay / green モード (D0 真因=赤信号 replay の修正)
# ---------------------------------------------------------------------------

_SIGNALS = {
    100: [(0.0, "red solidOn circle"), (5.0, "green solidOn circle")],
    200: [(0.0, "green solidOn circle")],
}


def test_initial_signal_actions_replay_uses_first_state() -> None:
    # replay (force_all_green=False): bag の各信号の開始時刻状態をそのまま採用。
    acts = _build_initial_signal_actions(_SIGNALS, force_all_green=False)
    states = {
        a["InfrastructureAction"]["TrafficSignalAction"]["TrafficSignalStateAction"]["name"]:
        a["InfrastructureAction"]["TrafficSignalAction"]["TrafficSignalStateAction"]["state"]
        for a in acts
    }
    assert states == {"100": "red solidOn circle", "200": "green solidOn circle"}


def test_initial_signal_actions_green_forces_all_green() -> None:
    # green (force_all_green=True): 全信号を常時 green に固定 (赤信号 replay 由来の D0 回避)。
    acts = _build_initial_signal_actions(_SIGNALS, force_all_green=True)
    states = [
        a["InfrastructureAction"]["TrafficSignalAction"]["TrafficSignalStateAction"]["state"]
        for a in acts
    ]
    assert states and all(s == "green solidOn circle" for s in states)


def test_signal_story_empty_in_green_mode() -> None:
    # green モードでは Story 側の信号切替 Act は空 (Init で全 green 固定するため)。
    assert _build_signal_story_acts(_SIGNALS, {100: 1, 200: 2}, force_all_green=True) == []
