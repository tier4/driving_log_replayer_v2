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

Runs the per-step analysis pipeline inside a cloud DLR2 job:
  1. Filter real vehicle MCAP to lite/real.lite.mcap  (make_lite.py)
  2. Generate per-step figures and report from real log  (compare_logs.py)

Outputs are written to result_archive_path, which is collected by
`logging.additional_log_archive` in .webauto-ci.yml.
"""

import json
import os
import subprocess
import sys
import traceback
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


class RealLogSimComparisonEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("real_log_sim_comparison_evaluator")

        for param in [
            "t4_dataset_path",
            "result_jsonl_path",
            "result_archive_path",
            "scenario_path",
            "map_path",
        ]:
            self.declare_parameter(param, "")

        # Fire once after ROS init to avoid blocking the spin loop.
        self._timer = self.create_timer(0.1, self._run_once)

    def _run_once(self) -> None:
        self._timer.cancel()

        t4_dataset_path = Path(self.get_parameter("t4_dataset_path").value)
        result_jsonl_path = Path(self.get_parameter("result_jsonl_path").value)
        result_archive_path = Path(self.get_parameter("result_archive_path").value)
        map_path = self.get_parameter("map_path").value

        result_archive_path.mkdir(parents=True, exist_ok=True)
        lite_dir = result_archive_path / "lite"
        comparison_dir = result_archive_path / "comparison"

        # scenario_path は use_case.py が共通で渡す。scenario.yaml から Conditions を読む。
        scenario_path_str = self.get_parameter("scenario_path").value
        compare_cfg = _load_compare_config(scenario_path_str)

        try:
            run_pipeline(
                t4_dataset_path,
                lite_dir,
                comparison_dir,
                map_path,
                compare_cfg,
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
    map_path: str,
    compare_cfg: dict[str, Any],
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

    # Step 2 – compare (real log only; sim logs are skipped when absent)
    logger.info("Step 2: compare_logs")
    env = os.environ.copy()
    env["BEST_MODEL_BASE_DIR"] = str(comparison_dir.parent)  # lite/ and comparison/ live here

    # 地図: t4_dataset_path/map/lanelet2_map.osm を自動解決
    map_osm = Path(map_path) / "lanelet2_map.osm" if map_path else Path("")
    if map_osm.exists():
        env["MAP_OSM_PATH"] = str(map_osm)
        logger.info(f"Map OSM: {map_osm}")
    else:
        # 空文字を明示することで compare_logs.py の後方互換フォールバック（x2_dev 地図の glob）を抑制する
        env["MAP_OSM_PATH"] = ""
        logger.warn(f"lanelet2_map.osm not found at {map_osm}; map background will be omitted")

    # シナリオ名
    if compare_cfg.get("scenario_name"):
        env["SCENARIO_NAME"] = compare_cfg["scenario_name"]

    # カーブ設定 YAML（scenario.yaml と同じディレクトリを基準に解決済み）
    # キーがなければ "" を明示設定し、後方互換フォールバック（x2_dev CURVE_CENTERS）を抑制する
    env["CURVE_CONFIG_YAML"] = compare_cfg.get("curve_config_yaml", "")

    _run(
        [sys.executable, str(compare_logs)],
        cwd=str(analysis_share),
        env=env,
    )


def _load_compare_config(scenario_path_str: str) -> dict[str, Any]:
    """scenario.yaml の Conditions から compare_logs 用設定を抽出する。

    Conditions に以下のキーを認識する（すべて任意）:
      - scenario_name (str): 図タイトル用シナリオ名
      - curve_config_yaml (str): カーブ設定 YAML の scenario.yaml からの相対パス or 絶対パス
                                 空文字を明示するとカーブ別解析スキップ
    """
    if not scenario_path_str:
        return {}
    scenario_path = Path(scenario_path_str)
    if not scenario_path.exists():
        return {}
    try:
        import yaml as _yaml
        with scenario_path.open(encoding="utf-8") as f:
            doc = _yaml.safe_load(f) or {}
        conditions: dict = (doc.get("Evaluation") or {}).get("Conditions") or {}

        cfg: dict[str, Any] = {}

        if "scenario_name" in conditions:
            cfg["scenario_name"] = str(conditions["scenario_name"])
        elif "ScenarioName" in doc:
            cfg["scenario_name"] = str(doc["ScenarioName"])

        if "curve_config_yaml" in conditions:
            raw = str(conditions["curve_config_yaml"])
            if raw == "":
                # 空文字はカーブ別解析スキップを明示
                cfg["curve_config_yaml"] = ""
            else:
                p = Path(raw)
                if not p.is_absolute():
                    p = scenario_path.parent / p
                cfg["curve_config_yaml"] = str(p) if p.exists() else ""

        return cfg
    except Exception:
        return {}


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
