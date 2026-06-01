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

Runs the 7-stage comparison pipeline inside a cloud DLR2 job:
  1. step1_make_lite       実機 input_bag → lite/real.lite/
  2. step2_bag_to_scenario lite/real.lite → scenarios/auto_scenario.yaml
  3. step3_run_sims        sim_runs.yaml の各 run を closed-loop 実行 → lite/<tag>.lite/
  4. step4_compare_logs    real + 全 sim を N-way 比較 (figures/, report.md)
  5. step5_analyze_per_step cases.yaml の各 case で per-step delta 解析
  6. step6_analyze_cases   per_step/*.csv を集約 (overlay/, cases_summary.md)

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

import rclpy
from rclpy.node import Node


class RealLogSimComparisonEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("real_log_sim_comparison_evaluator")

        for param in [
            "t4_dataset_path",
            "result_jsonl_path",
            "result_archive_path",
            "result_bag_path",
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
        result_bag_path = Path(self.get_parameter("result_bag_path").value)
        map_path = self.get_parameter("map_path").value

        result_archive_path.mkdir(parents=True, exist_ok=True)
        result_bag_path.mkdir(parents=True, exist_ok=True)
        lite_dir = result_archive_path / "lite"
        comparison_dir = result_archive_path / "comparison"

        # post_process の create_metadata_yaml は result_bag_path に rosbag ファイルがないとエラーになる。
        # pipeline が途中で失敗しても post_process が通るよう、事前に空の MCAP を置く。
        _write_placeholder_result_bag(result_bag_path)

        # scenario_path は use_case.py が共通で渡す。scenario.yaml から Conditions を読む。
        scenario_path_str = self.get_parameter("scenario_path").value
        compare_cfg = _load_compare_config(scenario_path_str)

        try:
            counts = run_pipeline(
                t4_dataset_path,
                lite_dir,
                result_bag_path,
                comparison_dir,
                map_path,
                compare_cfg,
                self.get_logger(),
            )
            sim_p, sim_e = counts["sim_runs_produced"], counts["sim_runs_expected"]
            case_p, case_e = counts["cases_produced"], counts["cases_expected"]
            # sim run / case が 1 件も出力されなければ比較は成立しない → INCOMPLETE (失敗扱い)。
            # 例外が出なくても「有意な出力ゼロ」を Success と誤報しないための E1 ガード。
            degenerate = (sim_p == 0) or (case_p == 0)
            success = not degenerate
            counts_str = (
                f"sim_runs {sim_p}/{sim_e}, cases {case_p}/{case_e}, "
                f"report={counts['report_ok']}, cases_summary={counts['cases_summary_ok']}"
            )
            summary = f"{'Success' if success else 'INCOMPLETE'} ({counts_str})"
            if degenerate:
                self.get_logger().error(f"Pipeline produced no comparison: {counts_str}")
        except Exception:
            success = False
            summary = traceback.format_exc()
            self.get_logger().error(f"Pipeline failed:\n{summary}")

        _write_result_jsonl(result_jsonl_path, success, summary)
        os._exit(0)


# ── Pipeline steps ────────────────────────────────────────────────────────────

def run_pipeline(
    t4_dataset_path: Path,
    lite_dir: Path,
    result_bag_path: Path,
    comparison_dir: Path,
    map_path: str,
    compare_cfg: dict[str, Any],
    logger,
) -> dict[str, int]:
    """7 段階パイプラインを実行し、各段の生成物カウントを返す。

    Stage 3/5/6 は個別 try/except で失敗継続する設計のため、例外が出ないことと
    「有意な出力が出たこと」は別物。呼び出し側 (_run_once) が本カウントを使って
    成否 (sim run / case が 0 件なら INCOMPLETE) を判定し、result.jsonl に計上する。
    """
    # Locate the real vehicle bag inside input_bag/ (db3 or mcap, auto-detected by step1_make_lite)
    input_bag_dir = t4_dataset_path / "input_bag"
    _validate_bag_dir(input_bag_dir)
    logger.info(f"Input bag: {input_bag_dir}")

    # ---- 共通 env ----
    env = os.environ.copy()
    env["BEST_MODEL_BASE_DIR"] = str(comparison_dir.parent)  # lite/ and comparison/ live here

    map_osm = Path(map_path) / "lanelet2_map.osm" if map_path else Path("")
    if map_osm.exists():
        env["MAP_OSM_PATH"] = str(map_osm)
        logger.info(f"Map OSM: {map_osm}")
    else:
        env["MAP_OSM_PATH"] = ""
        logger.warning(f"lanelet2_map.osm not found at {map_osm}; map background will be omitted")

    if compare_cfg.get("scenario_name"):
        env["SCENARIO_NAME"] = compare_cfg["scenario_name"]
    env["CURVE_CONFIG_YAML"] = compare_cfg.get("curve_config_yaml", "")

    # ---- Stage 1: real lite bag ----
    logger.info("Stage 1: generating real lite bag")
    lite_dir.mkdir(parents=True, exist_ok=True)
    lite_bag = lite_dir / "real.lite"
    _run([
        sys.executable, "-m",
        "driving_log_replayer_v2.real_log_sim_comparison.step1_make_lite",
        "--kind", "real",
        "--input", str(input_bag_dir),
        "--output", str(lite_bag),
    ], timeout=300)

    # ---- Stage 2: bag → scenario yaml 自動生成 ----
    logger.info("Stage 2: step2_bag_to_scenario (auto-generate OpenSCENARIO yaml)")
    scenarios_dir = comparison_dir.parent / "scenarios"
    scenarios_dir.mkdir(parents=True, exist_ok=True)
    auto_scenario = scenarios_dir / "auto_scenario.yaml"
    if map_osm.exists():
        try:
            _run([
                sys.executable, "-m",
                "driving_log_replayer_v2.real_log_sim_comparison.step2_bag_to_scenario",
                "--input-bag", str(input_bag_dir),
                "--map", str(map_osm),
                "--output", str(auto_scenario),
            ], env=env, timeout=300)
        except RuntimeError as exc:
            logger.warning(f"Stage 2 (step2_bag_to_scenario) failed: {exc}")
    else:
        logger.warning("Stage 2: map OSM が無いため scenario 自動生成スキップ")

    # ---- Stage 3: sim runs ループ (cases.yaml 同様、sim_runs.yaml 必須) ----
    sim_runs_yaml = compare_cfg.get("sim_runs_config", "")
    if not sim_runs_yaml or not Path(sim_runs_yaml).exists():
        raise RuntimeError(
            "sim_runs.yaml が見つかりません。scenario.yaml の Conditions.sim_runs_config に "
            "sim run 定義 YAML への (絶対 or 相対) パスを指定してください。"
            f" got: {sim_runs_yaml!r}"
        )

    from driving_log_replayer_v2.real_log_sim_comparison.lib._sim_runs_config import (  # noqa: PLC0415
        load_sim_runs_config,
    )

    sim_cfg = load_sim_runs_config(sim_runs_yaml)
    env["SIM_RUNS_CONFIG_YAML"] = sim_runs_yaml

    logger.info(f"Stage 3: step3_run_sims over {len(sim_cfg.runs)} run(s)")
    if auto_scenario.exists():
        base_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        for i, run in enumerate(sim_cfg.runs):
            sim_lite = lite_dir / f"{run.tag}.lite"
            env_run = env.copy()
            env_run["SIM_RUN_TAG"] = run.tag
            # nested ros2 launch: DDS 衝突回避のため domain id を切替
            env_run["ROS_DOMAIN_ID"] = str(base_domain_id + 10 + i)
            logger.info(f"  run: tag={run.tag}, vehicle_model={run.vehicle_model}, "
                        f"ROS_DOMAIN_ID={env_run['ROS_DOMAIN_ID']}")
            try:
                _run([
                    sys.executable, "-m",
                    "driving_log_replayer_v2.real_log_sim_comparison.step3_run_sims",
                    "--run-tag", run.tag,
                    "--scenario", str(auto_scenario),
                    "--sim-runs-config", sim_runs_yaml,
                    "--output-lite", str(sim_lite),
                ], env=env_run, timeout=run.timeout_s)
            except RuntimeError as exc:
                logger.warning(f"Stage 3 (run={run.tag}) failed but continuing: {exc}")
    else:
        logger.warning("Stage 3: auto_scenario.yaml が無いため sim 実行をスキップ")

    # ---- Stage 4: step4_compare_logs (real + 全 sim、sim_runs.yaml 連動で N-way) ----
    logger.info("Stage 4: step4_compare_logs (real + sim N-way)")
    _run([
        sys.executable, "-m",
        "driving_log_replayer_v2.real_log_sim_comparison.step4_compare_logs",
    ], env=env, timeout=1800)

    # ---- Stage 5: VehicleModel per-step 解析 (cases.yaml 必須) ----
    cases_yaml = compare_cfg.get("cases_config", "")
    if not cases_yaml or not Path(cases_yaml).exists():
        raise RuntimeError(
            "cases.yaml が見つかりません。scenario.yaml の Conditions.cases_config に "
            "VehicleModel ケース定義 YAML への (絶対 or 相対) パスを指定してください。"
            f" got: {cases_yaml!r}"
        )

    from driving_log_replayer_v2.real_log_sim_comparison.lib._cases_config import (  # noqa: PLC0415
        load_cases_config,
    )

    cases_cfg = load_cases_config(cases_yaml)
    env["CASES_CONFIG_YAML"] = cases_yaml

    logger.info(f"Stage 5: step5_analyze_per_step over {len(cases_cfg.cases)} case(s)")
    for case in cases_cfg.cases:
        logger.info(f"  case: tag={case.tag}, vehicle_model={case.vehicle_model}")
        env_case = env.copy()
        env_case["CASE_TAG"] = case.tag
        try:
            _run([
                sys.executable, "-m",
                "driving_log_replayer_v2.real_log_sim_comparison.step5_analyze_per_step",
                "--case-tag", case.tag,
                "--cases-config", cases_yaml,
            ], env=env_case, timeout=1800)
        except RuntimeError as exc:
            logger.warning(f"Stage 5 (case={case.tag}) failed but continuing: {exc}")

    # ---- Stage 6: ケース集約解析 (overlay 図 + cases_summary.md) ----
    logger.info("Stage 6: step6_analyze_cases (cross-case aggregation)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step6_analyze_cases",
            "--cases-config", cases_yaml,
        ], env=env, timeout=600)
    except RuntimeError as exc:
        logger.warning(f"Stage 6 (step6_analyze_cases) failed but continuing: {exc}")

    # ---- Stage 7: k_us 同定 (rollout sweep, 追加設定不要) ----
    # 実機 lite (Stage 1 出力) のみを使い、free-running rollout で実効 k_us を同定する独立ステージ。
    logger.info("Stage 7: step7_identify_kus (k_us identification via rollout sweep)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step7_identify_kus",
        ], env=env, timeout=900)
    except RuntimeError as exc:
        logger.warning(f"Stage 7 (step7_identify_kus) failed but continuing: {exc}")

    # ---- Stage 8: DiffusionPlanner 計画軌跡比較 (planner レベルの乖離分離) ----
    logger.info("Stage 8: step8_compare_dp_trajectory (planner trajectory real vs sim)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step8_compare_dp_trajectory",
        ], env=env, timeout=900)
    except RuntimeError as exc:
        logger.warning(f"Stage 8 (step8_compare_dp_trajectory) failed but continuing: {exc}")

    # ---- 生成物カウント (E1: 沈黙の失敗対策) ----
    # Stage 3/5 は失敗継続するため、実際に出力が出た数を数えて成否判定の材料にする。
    def _lite_exists(tag: str) -> bool:
        return (lite_dir / f"{tag}.lite").exists() or (lite_dir / f"{tag}.lite.mcap").exists()

    per_step_dir = comparison_dir / "per_step"
    sim_produced = sum(1 for r in sim_cfg.runs if _lite_exists(r.tag))
    cases_produced = sum(
        1 for c in cases_cfg.cases if (per_step_dir / c.tag / "per_step_delta.csv").exists()
    )
    counts = {
        "sim_runs_expected": len(sim_cfg.runs),
        "sim_runs_produced": sim_produced,
        "cases_expected": len(cases_cfg.cases),
        "cases_produced": cases_produced,
        "report_ok": int((comparison_dir / "report.md").exists()),
        "cases_summary_ok": int((comparison_dir / "cases" / "cases_summary.md").exists()),
        "kus_sweep_ok": int((comparison_dir / "kus_sweep" / "kus_sweep.csv").exists()),
    }
    logger.info(
        f"Pipeline outputs: sim_runs {sim_produced}/{len(sim_cfg.runs)}, "
        f"cases {cases_produced}/{len(cases_cfg.cases)}, "
        f"report={counts['report_ok']}, cases_summary={counts['cases_summary_ok']}, "
        f"kus_sweep={counts['kus_sweep_ok']}"
    )
    return counts


def _load_compare_config(scenario_path_str: str) -> dict[str, Any]:
    """scenario.yaml の Conditions から step4_compare_logs 用設定を抽出する。

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

        # cases_config (必須): Stage 5/6 のケース定義 YAML
        if "cases_config" in conditions:
            raw = str(conditions["cases_config"])
            p = Path(raw)
            if not p.is_absolute():
                p = scenario_path.parent / p
            cfg["cases_config"] = str(p)

        # sim_runs_config (必須): Stage 3/4 の sim run 定義 YAML
        if "sim_runs_config" in conditions:
            raw = str(conditions["sim_runs_config"])
            p = Path(raw)
            if not p.is_absolute():
                p = scenario_path.parent / p
            cfg["sim_runs_config"] = str(p)

        return cfg
    except Exception:
        return {}


def _write_placeholder_result_bag(result_bag_path: Path) -> None:
    """result_bag_path に空の mcap ファイルを置いて post_process を通す。

    rosbag2_py.SequentialWriter は既存ディレクトリへの上書き不可のため、
    一時ディレクトリに書いて .mcap ファイルだけ移動する。
    """
    import shutil
    import tempfile

    import rosbag2_py

    with tempfile.TemporaryDirectory(dir=result_bag_path.parent) as tmp:
        bag_uri = Path(tmp) / "placeholder"
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        del writer
        for mcap in bag_uri.glob("*.mcap"):
            shutil.move(str(mcap), str(result_bag_path / mcap.name))


def _validate_bag_dir(bag_dir: Path) -> None:
    """input_bag ディレクトリに db3 または mcap ファイルが存在するか確認する。"""
    if not any(bag_dir.glob("*.mcap")) and not any(bag_dir.glob("*.db3")):
        raise FileNotFoundError(f"No .mcap or .db3 file found in {bag_dir}")


def _run(cmd: list[str], cwd: str | None = None, env: dict | None = None, timeout: int | None = None) -> None:
    try:
        result = subprocess.run(cmd, check=False, cwd=cwd, env=env, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"Command timed out after {timeout}s: {' '.join(str(c) for c in cmd)}")
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
