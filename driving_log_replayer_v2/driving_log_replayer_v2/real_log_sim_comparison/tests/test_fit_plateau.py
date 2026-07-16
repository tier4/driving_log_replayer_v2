"""fit_plateau (プラトー直接同定の共有コア) の単体テスト。"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_plateau
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.parameter_constraints import (
    PARAMETER_CONSTRAINTS,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import HORIZONS


def test_channels_have_search_bounds_in_ssot() -> None:
    """同定対象は SSOT (PARAMETER_CONSTRAINTS) に search_bounds を持つこと。"""
    for _metric, key in fit_plateau.CHANNELS:
        assert key in PARAMETER_CONSTRAINTS
        assert PARAMETER_CONSTRAINTS[key].search_bounds is not None
    assert fit_plateau.THETA_KEYS == tuple(key for _m, key in fit_plateau.CHANNELS)


def test_channels_are_steer_and_ax() -> None:
    """系統別分離: steer は steer scaling、ax は acc scaling のみを同定すること。"""
    assert dict(fit_plateau.CHANNELS) == {
        "steer": "debug_steer_scaling_factor",
        "ax": "debug_acc_scaling_factor",
    }


def test_default_horizon_is_in_score_horizons() -> None:
    """既定 horizon は base_metric を流用できるよう HORIZONS に含まれること。"""
    assert fit_plateau.DEFAULT_HORIZON in HORIZONS


def test_plateau_channel_values_extracts_single_metric() -> None:
    metrics = [
        {30: {"steer": 0.4, "ax": 0.2}},
        {30: {"steer": 0.6, "ax": 0.3}},
    ]
    assert fit_plateau._plateau_channel_values(metrics, 30, "steer") == pytest.approx([0.4, 0.6])
    assert fit_plateau._plateau_channel_values(metrics, 30, "ax") == pytest.approx([0.2, 0.3])


@pytest.mark.parametrize(
    ("metrics", "metric_key"),
    [
        ([None], "steer"),
        ([{30: {"steer": math.inf, "ax": 0.2}}], "steer"),
        ([{30: {"steer": 0.4, "ax": math.nan}}], "ax"),
    ],
)
def test_plateau_channel_values_rejects_invalid_metrics(metrics, metric_key) -> None:
    assert fit_plateau._plateau_channel_values(metrics, 30, metric_key) is None


def test_plateau_channel_values_ignores_other_channel() -> None:
    """一方のチャネルが無効でも、他方のチャネルのフィットは進められること。"""
    metrics = [{30: {"steer": math.inf, "ax": 0.2}}]
    assert fit_plateau._plateau_channel_values(metrics, 30, "ax") == pytest.approx([0.2])


def test_fit_scaling_channels_rejects_horizon_outside_score_horizons(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="horizon"):
        fit_plateau.fit_scaling_channels(
            tmp_path, tmp_path / "scenario.yaml", horizon=42,
        )
