from __future__ import annotations

import copy

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import (
    add_evaluation_v2,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import (
    build_evaluation_v2,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import (
    horizon_steps_from_timestamps,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import HORIZONS_S
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import METRIC_WEIGHTS
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import (
    render_evaluation_v2_html,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import (
    select_time_horizon_metrics,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.evaluation_v2 import worst_tail_cvar


def _dataset_metrics(scales: list[float]) -> dict[str, dict[float, dict[str, float]]]:
    return {
        f"dataset-{index:03d}": {
            horizon: {
                metric: (index + 1) * (metric_index + 1) * horizon * scale
                for metric_index, metric in enumerate(METRIC_WEIGHTS)
            }
            for horizon in HORIZONS_S
        }
        for index, scale in enumerate(scales)
    }


def test_worst_tail_cvar_uses_ceil_for_five_percent_tail() -> None:
    assert worst_tail_cvar(range(1, 22)) == pytest.approx(20.5)


def test_time_horizons_are_mapped_per_dataset_with_half_up_rounding() -> None:
    mapping = horizon_steps_from_timestamps([index * 0.1 for index in range(40)])

    assert mapping == {0.25: 3, 0.5: 5, 1.0: 10, 2.0: 20, 3.0: 30}
    selected = select_time_horizon_metrics(
        {steps: {metric: float(steps) for metric in METRIC_WEIGHTS} for steps in mapping.values()},
        mapping,
    )
    assert selected[0.25]["yaw"] == pytest.approx(3.0)


def test_v2_horizons_weights_and_baseline_normalized_score() -> None:
    baseline = _dataset_metrics([1.0] * 20)
    tuned = _dataset_metrics([0.5] * 20)

    result = build_evaluation_v2({"baseline": baseline, "tuned": tuned})

    assert result["horizons_s"] == [0.25, 0.5, 1.0, 2.0, 3.0]
    assert result["weights"] == {
        "yaw": 0.20,
        "lat": 0.15,
        "long": 0.15,
        "vx": 0.15,
        "wz": 0.15,
        "steer": 0.10,
        "ax": 0.10,
    }
    assert result["comparison"]["baseline"]["score"] == pytest.approx(2.0)
    assert result["comparison"]["tuned"]["score"] == pytest.approx(1.0)
    assert result["comparison"]["tuned"]["score_components"] == pytest.approx(
        {"mean": 0.5, "worst_5pct_cvar": 0.5}
    )


def test_v2_worst_five_percent_cvar_is_aggregated_per_metric() -> None:
    baseline = _dataset_metrics([1.0] * 40)
    scales = [1.0] * 38 + [9.0, 10.0]
    candidate = _dataset_metrics(scales)

    result = build_evaluation_v2({"baseline": baseline, "candidate": candidate})
    horizon = result["comparison"]["candidate"]["by_h"][0.25]
    yaw = horizon["metrics"]["yaw"]

    assert yaw["normalized_mean"] == pytest.approx((38.0 + 9.0 + 10.0) / 40.0)
    assert yaw["normalized_worst_5pct_cvar"] == pytest.approx(9.5)
    assert horizon["score_components"]["worst_5pct_cvar"] == pytest.approx(9.5)


def test_additive_document_keeps_all_legacy_fields_unchanged() -> None:
    baseline = _dataset_metrics([1.0] * 2)
    legacy = {
        "params": {"old": 1.0},
        "score": 12.5,
        "comparison": {"baseline": {"score": 10.0, "by_h": {10: {"nyaw_mean": 1.0}}}},
        "metadata": {"n_valid": 2},
    }
    before = copy.deepcopy(legacy)

    result = add_evaluation_v2(legacy, {"baseline": baseline})

    assert legacy == before
    for key, value in before.items():
        assert result[key] == value
    assert result["metrics_schema_version"] == 2
    assert "evaluation_v2" in result


def test_v2_requires_all_seven_metrics() -> None:
    baseline = _dataset_metrics([1.0])
    del baseline["dataset-000"][0.25]["wz"]

    with pytest.raises(ValueError, match="wz"):
        build_evaluation_v2({"baseline": baseline})


def test_zero_baseline_compared_with_itself_has_unit_normalized_score() -> None:
    baseline = _dataset_metrics([1.0])
    baseline["dataset-000"][0.25]["wz"] = 0.0

    result = build_evaluation_v2({"baseline": baseline})

    wz = result["comparison"]["baseline"]["by_h"][0.25]["metrics"]["wz"]
    assert wz["normalized_mean"] == pytest.approx(1.0)
    assert wz["normalized_worst_5pct_cvar"] == pytest.approx(1.0)
    assert result["comparison"]["baseline"]["score"] == pytest.approx(2.0)


def test_html_lists_legacy_compatible_v2_and_escapes_model_names() -> None:
    baseline = _dataset_metrics([1.0] * 2)
    evaluation = build_evaluation_v2(
        {"baseline<script>": baseline}, baseline_name="baseline<script>"
    )

    rendered = render_evaluation_v2_html(evaluation)

    assert 'id="evaluation-v2"' in rendered
    assert "legacy" in rendered
    assert "worst 5% CVaR" in rendered
    assert "baseline&lt;script&gt;" in rendered
    assert "baseline<script>" not in rendered
