"""正規化スコアの分母フロアの回帰テスト。"""
from __future__ import annotations

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (
    LAT_FLOOR_PER_STEP,
    LONG_FLOOR_PER_STEP,
    YAW_FLOOR_PER_STEP,
    normalize_components,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import HORIZONS

# per-step 化前の固定辞書 (N=10..300)。傾き置換がこの値を再現し続けることを固定する。
_LEGACY_FLOORS = {
    "yaw": {10: 0.06, 30: 0.18, 70: 0.42, 150: 0.90, 300: 1.80},
    "long": {10: 1.0, 30: 3.0, 70: 7.0, 150: 15.0, 300: 30.0},
    "lat": {10: 0.3, 30: 0.9, 70: 2.1, 150: 4.5, 300: 9.0},
}


@pytest.mark.parametrize("h", sorted(_LEGACY_FLOORS["yaw"]))
def test_per_step_floors_reproduce_legacy_values(h: int) -> None:
    assert YAW_FLOOR_PER_STEP * h == pytest.approx(_LEGACY_FLOORS["yaw"][h])
    assert LONG_FLOOR_PER_STEP * h == pytest.approx(_LEGACY_FLOORS["long"][h])
    assert LAT_FLOOR_PER_STEP * h == pytest.approx(_LEGACY_FLOORS["lat"][h])


def test_normalize_components_works_for_any_horizon() -> None:
    """旧辞書では KeyError だった任意 horizon でもフロアが機能すること。"""
    m = {"yaw": 1.0, "long": 1.0, "lat": 1.0}
    tiny_baseline = {"yaw": 1e-9, "long": 1e-9, "lat": 1e-9}
    h = 42  # HORIZONS に含まれない horizon
    assert h not in HORIZONS
    normalized = normalize_components(m, tiny_baseline, h)
    assert normalized["nyaw"] == pytest.approx(1.0 / (YAW_FLOOR_PER_STEP * h))
    assert normalized["nlong"] == pytest.approx(1.0 / (LONG_FLOOR_PER_STEP * h))
    assert normalized["nlat"] == pytest.approx(1.0 / (LAT_FLOOR_PER_STEP * h))
