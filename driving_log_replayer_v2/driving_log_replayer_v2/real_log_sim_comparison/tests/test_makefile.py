from __future__ import annotations

import os
from pathlib import Path
import subprocess


MAKE_DIR = Path(__file__).parents[1]


def _make_help(**overrides: str) -> str:
    env = os.environ.copy()
    for name in (
        "ROOT",
        "SCENARIO",
        "INPUT_PARAM",
        "N_JOBS",
        "MULTI_BATCH_ROOT",
        "LOCAL_SAMPLE_DIR",
        "LOCAL_SCENARIO",
        "REAL_LOG_SIM_COMPARISON_JOBS",
    ):
        env.pop(name, None)
    env.update(overrides)
    return subprocess.run(
        ["make", "--no-print-directory", "help"],
        cwd=MAKE_DIR,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def test_make_defaults_preserve_previous_local_inputs() -> None:
    output = _make_help(WS_ROOT="/tmp/ws")

    assert "ROOT=/home/kotaroyoshimoto/data/openloop_j6_16_onwards" in output
    assert f"SCENARIO={MAKE_DIR}/sample/scenario.yaml" in output
    assert "INPUT_PARAM=/tmp/ws/src/description/vehicle/j6_gen2_description/" in output
    assert "N_JOBS=32" in output


def test_make_accepts_legacy_environment_variables() -> None:
    output = _make_help(
        MULTI_BATCH_ROOT="/tmp/legacy-root",
        LOCAL_SCENARIO="/tmp/legacy-scenario.yaml",
        REAL_LOG_SIM_COMPARISON_JOBS="7",
    )

    assert "ROOT=/tmp/legacy-root" in output
    assert "SCENARIO=/tmp/legacy-scenario.yaml" in output
    assert "N_JOBS=7" in output


def test_make_resolves_scenario_from_legacy_sample_directory() -> None:
    output = _make_help(LOCAL_SAMPLE_DIR="/tmp/legacy-sample")

    assert "SCENARIO=/tmp/legacy-sample/scenario.yaml" in output


def test_new_make_variables_take_precedence_over_legacy_environment() -> None:
    output = _make_help(
        ROOT="/tmp/new-root",
        SCENARIO="/tmp/new-scenario.yaml",
        N_JOBS="3",
        MULTI_BATCH_ROOT="/tmp/legacy-root",
        LOCAL_SCENARIO="/tmp/legacy-scenario.yaml",
        REAL_LOG_SIM_COMPARISON_JOBS="7",
    )

    assert "ROOT=/tmp/new-root" in output
    assert "SCENARIO=/tmp/new-scenario.yaml" in output
    assert "N_JOBS=3" in output
