"""fit_plateau (プラトー直接同定) の単体テスト。"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_plateau
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.parameter_constraints import (
    PARAMETER_CONSTRAINTS,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import HORIZONS


def test_theta_keys_have_search_bounds_in_ssot() -> None:
    """同定対象は SSOT (PARAMETER_CONSTRAINTS) に search_bounds を持つこと。"""
    for key in fit_plateau.THETA_KEYS:
        assert key in PARAMETER_CONSTRAINTS
        assert PARAMETER_CONSTRAINTS[key].search_bounds is not None


def test_default_horizon_is_in_score_horizons() -> None:
    """既定 horizon は base_metric を流用できるよう HORIZONS に含まれること。"""
    assert fit_plateau.DEFAULT_HORIZON in HORIZONS


def test_plateau_terms_collects_steer_and_ax() -> None:
    metrics = [
        {30: {"steer": 0.4, "ax": 0.2}},
        {30: {"steer": 0.6, "ax": 0.3}},
    ]
    terms = fit_plateau._plateau_terms(metrics, 30)
    assert terms is not None
    steer_vals, ax_vals = terms
    assert steer_vals == pytest.approx([0.4, 0.6])
    assert ax_vals == pytest.approx([0.2, 0.3])


@pytest.mark.parametrize(
    "metrics",
    [
        [None],
        [{30: {"steer": math.inf, "ax": 0.2}}],
        [{30: {"steer": 0.4, "ax": math.nan}}],
    ],
)
def test_plateau_terms_rejects_invalid_metrics(metrics) -> None:
    assert fit_plateau._plateau_terms(metrics, 30) is None


def test_fit_plateau_rejects_horizon_outside_score_horizons(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="horizon"):
        fit_plateau.fit_plateau(
            tmp_path, tmp_path / "scenario.yaml", horizon=42,
        )


def test_write_diagnostics_csv_schema(tmp_path: Path) -> None:
    metric = {
        h: {
            "steer": 0.4, "ax": 0.2, "steer_mean": 0.1, "ax_mean": -0.05,
            "yaw": 0.3, "long": 5.0, "lat": 1.0, "vx": 0.2, "pos": 5.1,
        }
        for h in (1, 10, 30)
    }
    result = {
        "diag_horizons": (1, 10, 30),
        "dataset_ids": ["ds-a", "ds-b"],
        "diagnostics": {"v1": [metric, None], "v1_p": [metric, metric]},
    }
    out = tmp_path / "plateau_diagnostics.csv"
    fit_plateau._write_diagnostics_csv(out, result)

    with out.open() as stream:
        rows = list(csv.DictReader(stream))
    # v1 は ds-b が None なので 3 horizons、v1_p は 2 datasets × 3 horizons。
    assert len(rows) == 3 + 6
    assert set(rows[0]) == {
        "dataset_id", "model", "horizon",
        "steer", "ax", "steer_mean", "ax_mean", "yaw", "long", "lat", "vx", "pos",
    }
    assert {row["model"] for row in rows} == {"v1", "v1_p"}
    assert float(rows[0]["steer"]) == pytest.approx(0.4)
    assert float(rows[0]["ax_mean"]) == pytest.approx(-0.05)
