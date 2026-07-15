"""正規化スコアの分母フロアの回帰テスト。"""
from __future__ import annotations

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (
    ACT_W,
    AX_FLOOR_MPS2,
    LAT_FLOOR_PER_STEP,
    LONG_FLOOR_PER_STEP,
    STEER_FLOOR_DEG,
    YAW_FLOOR_PER_STEP,
    aggregate_normalized,
    normalize_components,
    robust_score,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import (
    ACT_SCORE_HORIZONS,
    HORIZONS,
)

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


def test_normalize_components_without_actuator_keys_is_backward_compatible() -> None:
    """yaw/long/lat の 3 キー dict では nsteer/nax を出さない (旧呼び出し互換)。"""
    m = {"yaw": 1.0, "long": 1.0, "lat": 1.0}
    baseline = {"yaw": 1.0, "long": 1.0, "lat": 1.0}
    normalized = normalize_components(m, baseline, 10)
    assert "nsteer" not in normalized
    assert "nax" not in normalized


@pytest.mark.parametrize("h", [10, 300])
def test_actuator_floors_are_horizon_independent(h: int) -> None:
    """steer/ax のフロアはプラトー特性により h 非依存の定数であること。"""
    m = {"yaw": 1.0, "long": 1.0, "lat": 1.0, "steer": 1.0, "ax": 1.0}
    tiny_baseline = {"yaw": 1e-9, "long": 1e-9, "lat": 1e-9, "steer": 1e-9, "ax": 1e-9}
    normalized = normalize_components(m, tiny_baseline, h)
    assert normalized["nsteer"] == pytest.approx(1.0 / STEER_FLOOR_DEG)
    assert normalized["nax"] == pytest.approx(1.0 / AX_FLOOR_MPS2)


def test_act_score_horizons_are_subset_of_horizons() -> None:
    """アクチュエータ項の horizon は rollout 追加評価が不要なよう HORIZONS ⊆ であること。"""
    assert set(ACT_SCORE_HORIZONS) <= set(HORIZONS)


def _make_metric(yaw: float, long: float, lat: float, steer: float, ax: float) -> dict:
    return {"yaw": yaw, "long": long, "lat": lat, "steer": steer, "ax": ax}


def _uniform_by_h(metric: dict, horizons: tuple[int, ...]) -> dict:
    return {h: dict(metric) for h in horizons}


def test_robust_score_without_act_horizons_matches_legacy() -> None:
    """act_horizons=() (デフォルト) は旧目的関数 (yaw/long/lat のみ) と一致すること。"""
    horizons = (10, 30)
    baseline = _make_metric(yaw=1.0, long=10.0, lat=3.0, steer=0.5, ax=0.3)
    m = _make_metric(yaw=0.5, long=5.0, lat=1.5, steer=1.0, ax=0.6)
    baselines = {"ds": _uniform_by_h(baseline, horizons)}
    agg = aggregate_normalized([("ds", _uniform_by_h(m, horizons))], baselines, horizons)

    # steer/ax が倍悪化していても legacy スコアには影響しない。
    legacy = robust_score(agg, horizons)
    # 1 dataset なので mean = worst。per-horizon: (nyaw + 0.5(nlong+nlat))·1.5 = 0.5·1.5 + ...
    expected_per_h = (0.5 + 0.5 * (0.5 + 0.5)) * 1.5
    assert legacy == pytest.approx(expected_per_h * len(horizons))


def test_robust_score_adds_actuator_terms_for_act_horizons() -> None:
    """act_horizons を与えると nsteer/nax の mean+worst 項が ACT_W 重みで加算されること。"""
    horizons = (10, 30)
    baseline = _make_metric(yaw=1.0, long=10.0, lat=3.0, steer=0.5, ax=0.3)
    m = _make_metric(yaw=0.5, long=5.0, lat=1.5, steer=1.0, ax=0.6)
    baselines = {"ds": _uniform_by_h(baseline, horizons)}
    agg = aggregate_normalized([("ds", _uniform_by_h(m, horizons))], baselines, horizons)

    legacy = robust_score(agg, horizons)
    score = robust_score(agg, horizons, act_horizons=(10,))
    # nsteer = 1.0/0.5 = 2.0, nax = 0.6/0.3 = 2.0。1 dataset なので mean = worst。
    expected_act = ACT_W * (2.0 + 2.0) + 0.5 * ACT_W * (2.0 + 2.0)
    assert score == pytest.approx(legacy + expected_act)


def test_aggregate_normalized_reports_actuator_mean_and_worst() -> None:
    """複数 dataset の nsteer/nax mean・worst 集約の手計算一致。"""
    horizons = (10,)
    baseline = _make_metric(yaw=1.0, long=10.0, lat=3.0, steer=1.0, ax=1.0)
    baselines = {
        "a": _uniform_by_h(baseline, horizons),
        "b": _uniform_by_h(baseline, horizons),
    }
    per_ds = [
        ("a", _uniform_by_h(_make_metric(1.0, 10.0, 3.0, steer=1.0, ax=1.0), horizons)),
        ("b", _uniform_by_h(_make_metric(1.0, 10.0, 3.0, steer=3.0, ax=0.5), horizons)),
    ]
    agg = aggregate_normalized(per_ds, baselines, horizons)
    b = agg["by_h"][10]
    assert b["nsteer_mean"] == pytest.approx(2.0)  # (1.0 + 3.0) / 2
    assert b["nsteer_worst"] == pytest.approx(3.0)
    assert b["nax_mean"] == pytest.approx(0.75)  # (1.0 + 0.5) / 2
    assert b["nax_worst"] == pytest.approx(1.0)
