#!/usr/bin/env python3
# Copyright (c) 2025 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Orchestration node for real_log_sim_comparison.

Runs the full analysis pipeline inside a cloud DLR2 job:
  1. Filter real vehicle MCAP to lite/real.lite.mcap  (make_lite.py)
  2. Run scenario_test_runner (normal sim)  → lite/sim_normal.lite.mcap
  3. Run scenario_test_runner (Godot sim)   → lite/sim_godot.lite.mcap  (skipped if binary missing)
  4. Generate comparison figures and report  (compare_logs.py)

Outputs are written to result_archive_path, which is collected by
`logging.additional_log_archive` in .webauto-ci.yml.
"""

import json
import os
import subprocess
import sys
import traceback
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


SIM_OUT_ROOT = Path("/tmp/scenario_test_runner")


class RealLogSimComparisonEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("real_log_sim_comparison_evaluator")

        for param in [
            "t4_dataset_path",
            "result_jsonl_path",
            "result_archive_path",
            "vehicle_model_normal",
            "vehicle_model_godot",
            "sensor_model_sim",
            "godot_executable",
            "scenario_test_runner_scenario",
        ]:
            self.declare_parameter(param, "")

        # Fire once after ROS init to avoid blocking the spin loop.
        self._timer = self.create_timer(0.1, self._run_once)

    def _run_once(self) -> None:
        self._timer.cancel()

        t4_dataset_path = Path(self.get_parameter("t4_dataset_path").value)
        result_jsonl_path = Path(self.get_parameter("result_jsonl_path").value)
        result_archive_path = Path(self.get_parameter("result_archive_path").value)
        vehicle_model_normal = self.get_parameter("vehicle_model_normal").value
        vehicle_model_godot = self.get_parameter("vehicle_model_godot").value
        sensor_model_sim = self.get_parameter("sensor_model_sim").value
        godot_executable = self.get_parameter("godot_executable").value
        scenario_test_runner_scenario = self.get_parameter("scenario_test_runner_scenario").value

        result_archive_path.mkdir(parents=True, exist_ok=True)
        lite_dir = result_archive_path / "lite"
        comparison_dir = result_archive_path / "comparison"

        try:
            run_pipeline(
                t4_dataset_path,
                lite_dir,
                comparison_dir,
                vehicle_model_normal,
                vehicle_model_godot,
                sensor_model_sim,
                Path(godot_executable),
                Path(scenario_test_runner_scenario),
                self.get_logger(),
            )
            success = True
            summary = "Success"
        except Exception:
            success = False
            summary = traceback.format_exc()
            self.get_logger().error(f"Pipeline failed:\n{summary}")

        _write_result_jsonl(result_jsonl_path, success, summary)
        rclpy.shutdown()


# ── Pipeline steps ────────────────────────────────────────────────────────────

def run_pipeline(
    t4_dataset_path: Path,
    lite_dir: Path,
    comparison_dir: Path,
    vehicle_model_normal: str,
    vehicle_model_godot: str,
    sensor_model_sim: str,
    godot_executable: Path,
    scenario_test_runner_scenario: Path,
    logger,
) -> None:
    analysis_share = Path(get_package_share_directory("real_log_sim_comparison"))
    make_lite = analysis_share / "make_lite.py"
    compare_logs = analysis_share / "compare_logs.py"

    # Locate the real vehicle MCAP inside input_bag/
    input_bag_dir = t4_dataset_path / "input_bag"
    real_mcap = _find_mcap(input_bag_dir)
    logger.info(f"Real MCAP: {real_mcap}")

    # Step 1 – real lite
    logger.info("Step 1: generating real.lite.mcap")
    _run([
        sys.executable, str(make_lite),
        "--kind", "real",
        "--input", str(real_mcap),
        "--output", str(lite_dir / "real.lite.mcap"),
    ])

    # Step 2 – sim_normal
    logger.info("Step 2: sim_normal")
    _run_sim(
        vehicle_model=vehicle_model_normal,
        sensor_model=sensor_model_sim,
        output_lite=lite_dir / "sim_normal.lite.mcap",
        make_lite=make_lite,
        scenario_path=scenario_test_runner_scenario,
        extra_args=[],
        logger=logger,
    )

    # Step 3 – sim_godot (skip when binary is absent)
    if godot_executable.exists():
        logger.info("Step 3: sim_godot")
        _run_sim(
            vehicle_model=vehicle_model_godot,
            sensor_model=sensor_model_sim,
            output_lite=lite_dir / "sim_godot.lite.mcap",
            make_lite=make_lite,
            scenario_path=scenario_test_runner_scenario,
            extra_args=[f"godot_executable:={godot_executable}"],
            logger=logger,
        )
    else:
        logger.warn(f"Godot executable not found ({godot_executable}); skipping sim_godot step.")

    # Step 4 – compare
    logger.info("Step 4: compare_logs")
    env = os.environ.copy()
    env["REAL_LOG_SIM_BASE_DIR"] = str(comparison_dir.parent)  # lite/ and comparison/ live here
    _run(
        [sys.executable, str(compare_logs)],
        cwd=str(analysis_share),
        env=env,
    )


def _run_sim(
    vehicle_model: str,
    sensor_model: str,
    output_lite: Path,
    make_lite: Path,
    scenario_path: Path,
    extra_args: list[str],
    logger,
) -> None:
    SIM_OUT_ROOT.mkdir(parents=True, exist_ok=True)
    # Remove stale outputs from a previous run.
    for f in SIM_OUT_ROOT.rglob("*.mcap"):
        f.unlink()

    launch_args = [
        f"vehicle_model:={vehicle_model}",
        f"sensor_model:={sensor_model}",
        "record:=true",
        "record_storage_id:=mcap",
        "architecture_type:=awf/universe/20250130",
        "initialize_duration:=100",
        f"scenario:={scenario_path}",
        *extra_args,
    ]
    _run(["ros2", "launch", "scenario_test_runner", "scenario_test_runner.launch.py", *launch_args])

    mcap_files = list(SIM_OUT_ROOT.rglob("*.mcap"))
    if not mcap_files:
        msg = f"No .mcap found in {SIM_OUT_ROOT} after sim run"
        raise RuntimeError(msg)

    _run([
        sys.executable, str(make_lite),
        "--kind", "sim",
        "--input", str(mcap_files[0]),
        "--output", str(output_lite),
    ])

    # Remove full sim bag to save disk space.
    for f in SIM_OUT_ROOT.rglob("*.mcap"):
        try:
            f.unlink()
        except OSError:
            pass


def _find_mcap(bag_dir: Path) -> Path:
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        msg = f"No .mcap file found in {bag_dir}"
        raise FileNotFoundError(msg)
    return mcap_files[0]


def _run(cmd: list[str], cwd: str | None = None, env: dict | None = None) -> None:
    result = subprocess.run(cmd, check=False, cwd=cwd, env=env)
    if result.returncode != 0:
        msg = f"Command failed (rc={result.returncode}): {' '.join(str(c) for c in cmd)}"
        raise RuntimeError(msg)


def _write_result_jsonl(path: Path, success: bool, summary: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {
        "Result": {"Success": success, "Summary": summary},
        "Stamp": {"System": 0.0},
        "Frame": {},
    }
    with path.open("w") as f:
        json.dump(record, f)
        f.write("\n")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = RealLogSimComparisonEvaluator()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
