from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_xy
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.fit_xy import (
    _fit_xy_heading_rate_coeff,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.parameter_constraints import (
    FIT_XY,
    PARAMETER_CONSTRAINTS,
)


def _synthetic_trajectory(coeff: float, *, dt: float = 0.01, n: int = 4000) -> dict:
    """既知の coeff で C++ 状態方程式を積分した合成軌道を作る。

    d_state(X) = vel*cos(yaw - coeff*vel*yaw_rate)
    d_state(Y) = vel*sin(yaw - coeff*vel*yaw_rate)
    """
    t = np.arange(n) * dt
    wz = 0.3 * np.sin(2.0 * np.pi * t / 5.0)
    vx = 5.0 * np.ones(n)
    yaw = np.cumsum(wz) * dt
    effective_yaw = yaw - coeff * vx * wz
    vx_state = vx * np.cos(effective_yaw)
    vy_state = vx * np.sin(effective_yaw)
    x = np.cumsum(vx_state) * dt
    y = np.cumsum(vy_state) * dt
    return {
        "gear_drive": np.ones(n, dtype=bool),
        "vx": vx.astype(np.float32),
        "xy": (x, y, yaw, vx, wz),
    }


@pytest.mark.parametrize("known_coeff", [0.0, 0.02, -0.015])
def test_fit_xy_heading_rate_coeff_recovers_known_value(known_coeff: float) -> None:
    dataset = _synthetic_trajectory(known_coeff)

    result = _fit_xy_heading_rate_coeff([dataset])

    assert result["n"] > 0
    assert result["xy_heading_rate_coeff"] == pytest.approx(known_coeff, abs=5e-3)
    assert result["rmse"] < 0.05


def test_fit_xy_heading_rate_coeff_merges_multiple_datasets() -> None:
    single = _fit_xy_heading_rate_coeff([_synthetic_trajectory(0.02)])
    merged = _fit_xy_heading_rate_coeff(
        [_synthetic_trajectory(0.02), _synthetic_trajectory(0.02)]
    )

    assert merged["n"] == 2 * single["n"]
    assert merged["xy_heading_rate_coeff"] == pytest.approx(0.02, abs=5e-3)


def test_fit_xy_skips_optimization_when_target_disabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(
        PARAMETER_CONSTRAINTS,
        "xy_heading_rate_coeff",
        replace(PARAMETER_CONSTRAINTS["xy_heading_rate_coeff"], optimization_stages=frozenset()),
    )

    result = fit_xy.fit_xy(
        Path("/nonexistent"),
        phase2_params={"xy_heading_rate_coeff": 0.0},
    )

    assert result["params"]["xy_heading_rate_coeff"] == pytest.approx(0.0)
    assert result["metadata"]["optimized_parameters"] == []
    assert result["metadata"]["phase"] == 3


def test_fit_xy_requires_scenario_fallback_when_target_disabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(
        PARAMETER_CONSTRAINTS,
        "xy_heading_rate_coeff",
        replace(PARAMETER_CONSTRAINTS["xy_heading_rate_coeff"], optimization_stages=frozenset()),
    )

    with pytest.raises(ValueError, match="scenario 初期値がありません"):
        fit_xy.fit_xy(Path("/nonexistent"), phase2_params={})


def test_optimization_targets_include_fit_xy() -> None:
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.parameter_constraints import (
        stage_targets,
    )

    assert stage_targets(FIT_XY) == {"xy_heading_rate_coeff"}


def test_run_rejects_incomplete_steer_artifact(tmp_path: Path) -> None:
    phase2 = tmp_path / "phase2.yaml"
    phase2.write_text("params: {}\nmetadata:\n  phase: 2\n", encoding="utf-8")

    with pytest.raises(ValueError, match="invalid fit_steer result"):
        fit_xy.run(
            tmp_path,
            tmp_path / "phase3.yaml",
            phase2_params_path=phase2,
            scenario=tmp_path / "scenario.yaml",
        )


def test_run_rejects_wrong_phase_metadata(tmp_path: Path) -> None:
    phase2 = tmp_path / "phase2.yaml"
    phase2.write_text(
        yaml.safe_dump(
            {
                "params": {
                    "acc_time_constant": 0.2,
                    "acc_time_delay": 0.1,
                    "debug_acc_scaling_factor": 1.0,
                    "steer_time_constant": 0.2,
                    "steer_time_delay": 0.05,
                    "debug_steer_scaling_factor": 1.0,
                    "steer_bias": 0.0,
                    "k_us": 0.01,
                },
                "metadata": {"phase": 1},
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="metadata.phase"):
        fit_xy.run(
            tmp_path,
            tmp_path / "phase3.yaml",
            phase2_params_path=phase2,
            scenario=tmp_path / "scenario.yaml",
        )
