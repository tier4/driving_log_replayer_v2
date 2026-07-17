"""正規化スコアの分母フロアの回帰テスト。"""
from __future__ import annotations

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (
    ACT_W,
    AX_FLOOR_MPS2,
    CVAR_Q,
    FLOOR_TABLE,
    STEER_FLOOR_DEG,
    aggregate_normalized,
    cvar_worst,
    normalize_components,
    robust_score,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import (
    ACT_SCORE_HORIZONS,
    HORIZONS,
)

def test_floor_table_covers_score_horizons_and_is_monotonic() -> None:
    """v3 フロアテーブルは score horizon を全被覆し、h について単調増加であること。"""
    assert set(HORIZONS) <= set(FLOOR_TABLE)
    ordered = sorted(FLOOR_TABLE)
    for key in ("yaw", "long", "lat"):
        values = [FLOOR_TABLE[h][key] for h in ordered]
        assert values == sorted(values)
        assert all(v > 0 for v in values)


def test_normalize_components_rejects_horizon_outside_floor_table() -> None:
    """v3 (デフォルト) フロアはテーブル外の horizon で fail fast すること。"""
    m = {"yaw": 1.0, "long": 1.0, "lat": 1.0}
    baseline = {"yaw": 1.0, "long": 1.0, "lat": 1.0}
    h = 42  # HORIZONS / FLOOR_TABLE に含まれない horizon
    assert h not in HORIZONS
    with pytest.raises(ValueError, match="FLOOR_TABLE"):
        normalize_components(m, baseline, h)


def test_v3_floor_clips_tiny_baseline() -> None:
    """v3 (デフォルト) フロアはテーブル値で分母クリップすること。"""
    m = {"yaw": 1.0, "long": 1.0, "lat": 1.0}
    tiny_baseline = {"yaw": 1e-9, "long": 1e-9, "lat": 1e-9}
    h = 30
    normalized = normalize_components(m, tiny_baseline, h)
    assert normalized["nyaw"] == pytest.approx(1.0 / FLOOR_TABLE[h]["yaw"])
    assert normalized["nlong"] == pytest.approx(1.0 / FLOOR_TABLE[h]["long"])
    assert normalized["nlat"] == pytest.approx(1.0 / FLOOR_TABLE[h]["lat"])


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


def test_cvar_worst_matches_max_for_small_n() -> None:
    """n ≤ 1/(1-CVAR_Q) では CVaR は max に一致する (少数 dataset の後方互換)。"""
    assert CVAR_Q == pytest.approx(0.90)
    values = [0.5, 3.0, 1.0, 2.0]  # n=4 ≤ 10
    assert cvar_worst(values) == pytest.approx(max(values))


def test_cvar_worst_averages_top_tail() -> None:
    """n=20 では上位 ceil(20·0.1)=2 件の平均になること。"""
    values = [float(i) for i in range(1, 21)]  # 1..20
    assert cvar_worst(values) == pytest.approx((20.0 + 19.0) / 2)


def _make_metric(yaw: float, long: float, lat: float, steer: float, ax: float) -> dict:
    return {"yaw": yaw, "long": long, "lat": lat, "steer": steer, "ax": ax}


def _uniform_by_h(metric: dict, horizons: tuple[int, ...]) -> dict:
    return {h: dict(metric) for h in horizons}


def test_robust_score_without_act_horizons_uses_motion_components_only() -> None:
    """act_horizons=() は yaw/long/lat のみを集約すること。"""
    horizons = (10, 30)
    baseline = _make_metric(yaw=1.0, long=10.0, lat=3.0, steer=0.5, ax=0.3)
    m = _make_metric(yaw=0.5, long=5.0, lat=1.5, steer=1.0, ax=0.6)
    baselines = {"ds": _uniform_by_h(baseline, horizons)}
    agg = aggregate_normalized([("ds", _uniform_by_h(m, horizons))], baselines, horizons)

    # steer/ax が倍悪化していても、act_horizons 未指定のスコアには影響しない。
    motion_only = robust_score(agg, horizons)
    # 1 dataset なので mean = worst。per-horizon: (nyaw + 0.5(nlong+nlat))·1.5 = 0.5·1.5 + ...
    expected_per_h = (0.5 + 0.5 * (0.5 + 0.5)) * 1.5
    assert motion_only == pytest.approx(expected_per_h * len(horizons))


def test_robust_score_adds_actuator_terms_for_act_horizons() -> None:
    """act_horizons を与えると nsteer/nax の mean+worst 項が ACT_W 重みで加算されること。"""
    horizons = (10, 30)
    baseline = _make_metric(yaw=1.0, long=10.0, lat=3.0, steer=0.5, ax=0.3)
    m = _make_metric(yaw=0.5, long=5.0, lat=1.5, steer=1.0, ax=0.6)
    baselines = {"ds": _uniform_by_h(baseline, horizons)}
    agg = aggregate_normalized([("ds", _uniform_by_h(m, horizons))], baselines, horizons)

    motion_only = robust_score(agg, horizons)
    score = robust_score(agg, horizons, act_horizons=(10,))
    # nsteer = 1.0/0.5 = 2.0, nax = 0.6/0.3 = 2.0。1 dataset なので mean = worst。
    expected_act = ACT_W * (2.0 + 2.0) + 0.5 * ACT_W * (2.0 + 2.0)
    assert score == pytest.approx(motion_only + expected_act)


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


def test_aggregate_normalized_reports_cvar_for_all_components() -> None:
    """集約は全成分に *_cvar を出力し、少数 dataset では *_worst と一致すること。"""
    horizons = (10,)
    baseline = _make_metric(yaw=1.0, long=10.0, lat=3.0, steer=1.0, ax=1.0)
    baselines = {
        "a": _uniform_by_h(baseline, horizons),
        "b": _uniform_by_h(baseline, horizons),
    }
    per_ds = [
        ("a", _uniform_by_h(_make_metric(1.0, 10.0, 3.0, steer=1.0, ax=1.0), horizons)),
        ("b", _uniform_by_h(_make_metric(2.0, 20.0, 6.0, steer=3.0, ax=0.5), horizons)),
    ]
    agg = aggregate_normalized(per_ds, baselines, horizons)
    b = agg["by_h"][10]
    for key in ("nyaw", "nlong", "nlat", "nsteer", "nax"):
        assert b[f"{key}_cvar"] == pytest.approx(b[f"{key}_worst"])


def test_robust_score_cvar_reads_cvar_keys() -> None:
    """worst_stat="cvar" は *_worst でなく *_cvar を読むこと (手計算一致)。"""
    horizons = (10,)
    by_h = {
        10: {
            "nyaw_mean": 1.0, "nyaw_worst": 5.0, "nyaw_cvar": 2.0,
            "nlong_mean": 1.0, "nlong_worst": 5.0, "nlong_cvar": 2.0,
            "nlat_mean": 1.0, "nlat_worst": 5.0, "nlat_cvar": 2.0,
            "nsteer_mean": 1.0, "nsteer_worst": 5.0, "nsteer_cvar": 2.0,
            "nax_mean": 1.0, "nax_worst": 5.0, "nax_cvar": 2.0,
        }
    }
    agg = {"by_h": by_h}
    # mean 項: 1 + 0.5·(1+1) = 2、cvar 項: 0.5·(2 + 0.5·(2+2)) = 2
    assert robust_score(agg, horizons, worst_stat="cvar") == pytest.approx(4.0)
    # act 項: 0.5·(1+1) + 0.5·0.5·(2+2) = 2
    assert robust_score(
        agg, horizons, act_horizons=(10,), worst_stat="cvar"
    ) == pytest.approx(6.0)
    # max は従来どおり *_worst を読む: 1 + 0.5·2 + 0.5·(5 + 0.5·10) = 7
    assert robust_score(agg, horizons) == pytest.approx(7.0)


def test_robust_score_rejects_unknown_worst_stat() -> None:
    with pytest.raises(ValueError, match="worst_stat"):
        robust_score({"by_h": {}}, (), worst_stat="p95")
