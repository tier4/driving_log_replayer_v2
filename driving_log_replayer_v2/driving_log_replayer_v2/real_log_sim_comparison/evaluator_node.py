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

Runs the 10-stage comparison pipeline inside a cloud DLR2 job:
  1. step1_make_lite       実機 input_bag → lite/real.lite/
  2. step2_bag_to_scenario lite/real.lite → scenarios/auto_scenario.yaml
  3. step3_run_sims        sim_runs.yaml の各 run を closed-loop 実行 → lite/<tag>.lite/
  4. step4_compare_logs    real + 全 sim を N-way 比較 (figures/, report.md)
  5. step5_analyze_nstep   cases.yaml の各 case で N-step オープンループ解析
  6. step6_analyze_cases   nstep/*.csv を集約 (overlay/, cases_summary.md)
  7. step7_sweep_params    real.lite で車両モデルパラメータを rollout sweep 同定 (param_sweep/)
  8. step8_compare_dp_trajectory DiffusionPlanner 出力軌跡 real vs sim 比較 (figures/dp_*.svg)
  11. step11_build_html_report comparison/ 配下の全プロットを集約 (result_archive/real_log_sim_comparison/index.html)

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
        # Web.Auto は result_archive/ の中身をそのまま zip 化するため、ラッパーフォルダが無いと
        # 展開時に comparison/lite/scenarios/report.html が散らばる。本ユースケースの成果物は
        # 単一バンドルフォルダ配下にまとめ、zip 展開で 1 フォルダにまとまるようにする。
        # post_process が後段で書く result_jsonl.png のみ result_archive/ 直下に残る
        # (= 展開時は「バンドルフォルダ 1 つ + result_jsonl.png」)。
        bundle_dir = result_archive_path / "real_log_sim_comparison"
        lite_dir = bundle_dir / "lite"
        comparison_dir = bundle_dir / "comparison"

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
            # skip_sim (closed-loop sim を意図的に省略) のときは sim 0 件を失敗にしない。
            sim_skipped = bool(counts["sim_skipped"])
            degenerate = (sim_p == 0 and not sim_skipped) or (case_p == 0)
            success = not degenerate
            counts_str = (
                f"sim_runs {'skipped' if sim_skipped else f'{sim_p}/{sim_e}'}, "
                f"cases {case_p}/{case_e}, "
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
    """10 段階パイプラインを実行し、各段の生成物カウントを返す。

    Stage 3/5/6 は個別 try/except で失敗継続する設計のため、例外が出ないことと
    「有意な出力が出たこと」は別物。呼び出し側 (_run_once) が本カウントを使って
    成否 (sim run / case が 0 件なら INCOMPLETE) を判定し、result.jsonl に計上する。
    """
    # Locate the real vehicle bag inside input_bag/ (db3 or mcap, auto-detected by step1_make_lite)
    input_bag_dir = t4_dataset_path / "input_bag"
    _validate_bag_dir(input_bag_dir)
    logger.info(f"Input bag: {input_bag_dir}")

    # ---- 共通 env ----
    env = build_common_env(comparison_dir, map_path, compare_cfg, logger)
    # Stage 2 (scenario 自動生成) が使う地図パス。build_common_env が解決済みの
    # MAP_OSM_PATH (存在しない場合は空文字) から復元する。
    map_osm = Path(env["MAP_OSM_PATH"]) if env["MAP_OSM_PATH"] else Path("")
    # perception 再生 (既定 false; step3 が読む)。true で実機 input_bag の先行車を ego-pose 同期で
    # 各 sim に注入し、実機の先行車追従 (停止・加減速) を再現する。
    env["REPRODUCE_BAG"] = (
        str(input_bag_dir) if compare_cfg.get("reproduce_perception", False) else ""
    )

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

    # ---- Stage 3: sim runs ループ (scenario.yaml の Conditions.sim_runs 必須) ----
    scenario_config = compare_cfg.get("scenario_config", "")
    if not scenario_config or not Path(scenario_config).exists():
        raise RuntimeError(
            "scenario.yaml が見つかりません。Conditions.models / sim_runs / cases を "
            "scenario.yaml に直接記述してください。"
            f" got: {scenario_config!r}"
        )

    from driving_log_replayer_v2.real_log_sim_comparison.lib._sim_runs_config import (  # noqa: PLC0415
        load_sim_runs_config,
    )

    sim_cfg = load_sim_runs_config(scenario_config)
    env["SCENARIO_CONFIG_YAML"] = scenario_config

    # closed-loop sim のスキップ (open-loop 解析だけ欲しいとき・マルチ DS バッチの時間短縮)。
    # scenario.yaml の Conditions.skip_sim (クラウド) か env SKIP_SIM=1 (make 変数) で指定する。
    # Stage 2 (scenario 生成) は軽量で、collect_datasets の dataset_id 推定が auto_scenario.yaml
    # に依存するためスキップしない。Stage 4 以降は sim lite 欠損時に実機のみで動く。
    skip_sim = bool(compare_cfg.get("skip_sim", False)) or env.get("SKIP_SIM") == "1"

    if skip_sim:
        logger.info(
            f"Stage 3: skipped (skip_sim) — {len(sim_cfg.runs)} run(s) defined but not executed"
        )
    elif auto_scenario.exists():
        logger.info(f"Stage 3: step3_run_sims over {len(sim_cfg.runs)} run(s)")
        base_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        # 各 sim run の実行ログ (ros2 launch / autoware / make_lite 等) は run ごとに分離保存する
        # (集約ログを汚さず後から個別に追えるように)。result_archive 配下なのでアーカイブされる。
        sim_logs_dir = comparison_dir / "sim_logs"
        for i, run in enumerate(sim_cfg.runs):
            sim_lite = lite_dir / f"{run.tag}.lite"
            env_run = env.copy()
            env_run["SIM_RUN_TAG"] = run.tag
            # nested ros2 launch: DDS 衝突回避のため domain id を切替
            env_run["ROS_DOMAIN_ID"] = str(base_domain_id + 10 + i)
            run_log = sim_logs_dir / f"{run.tag}.log"
            logger.info(f"  run: tag={run.tag}, vehicle_model={run.vehicle_model}, "
                        f"ROS_DOMAIN_ID={env_run['ROS_DOMAIN_ID']}, log={run_log}")
            try:
                _run([
                    sys.executable, "-m",
                    "driving_log_replayer_v2.real_log_sim_comparison.step3_run_sims",
                    "--run-tag", run.tag,
                    "--scenario", str(auto_scenario),
                    "--config-scenario", scenario_config,
                    "--output-lite", str(sim_lite),
                ], env=env_run, timeout=run.timeout_s, log_file=run_log)
            except RuntimeError as exc:
                logger.warning(
                    f"Stage 3 (run={run.tag}) failed but continuing: {exc} (log: {run_log})"
                )
    else:
        logger.warning("Stage 3: auto_scenario.yaml が無いため sim 実行をスキップ")

    counts = run_analysis(lite_dir, comparison_dir, env, compare_cfg, logger)
    counts["sim_skipped"] = int(skip_sim)
    return counts


def build_common_env(
    comparison_dir: Path, map_path: str, compare_cfg: dict[str, Any], logger
) -> dict[str, str]:
    """Stage 2〜11 が共有する env を構築する。"""
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
    # 実機データ取得時の版・重み (外部記録)。step4 が provenance 掲載に使う。
    env["REAL_PROVENANCE"] = compare_cfg.get("real_provenance", "")
    # route-shaping 実験オプション (既定 0=start+goal のみ; step2 が読む)。D0 の修正ではない。
    env["LOOP_WAYPOINTS"] = str(compare_cfg.get("loop_waypoints", 0))
    # 信号の扱い (既定 replay; step2 が読む)。green で赤信号 replay 由来の D0 早期停止を回避。
    env["TRAFFIC_SIGNALS"] = str(compare_cfg.get("traffic_signals", "replay"))
    return env


def run_analysis(
    lite_dir: Path,
    comparison_dir: Path,
    env: dict[str, str],
    compare_cfg: dict[str, Any],
    logger,
) -> dict[str, int]:
    """解析ステージ (Stage 4〜11) を実行し、生成物カウントを返す。

    lite/ (real + sim run) が揃っている前提で動く純解析部。run_pipeline (フルパイプライン)
    と run_analysis.py CLI (sim 実行済みバンドルの再解析) の両方から呼ばれる。
    """
    scenario_config = compare_cfg.get("scenario_config", "")
    if not scenario_config or not Path(scenario_config).exists():
        raise RuntimeError(
            "scenario.yaml が見つかりません。Conditions.models / sim_runs / cases を "
            "scenario.yaml に直接記述してください。"
            f" got: {scenario_config!r}"
        )
    from driving_log_replayer_v2.real_log_sim_comparison.lib._sim_runs_config import (  # noqa: PLC0415
        load_sim_runs_config,
    )

    sim_cfg = load_sim_runs_config(scenario_config)
    env = env.copy()
    env["SCENARIO_CONFIG_YAML"] = scenario_config

    # ---- Stage 4: step4_compare_logs (real + 全 sim、sim_runs.yaml 連動で N-way) ----
    logger.info("Stage 4: step4_compare_logs (real + sim N-way)")
    _run([
        sys.executable, "-m",
        "driving_log_replayer_v2.real_log_sim_comparison.step4_compare_logs",
    ], env=env, timeout=1800)

    # ---- Stage 5: VehicleModel N-step オープンループ解析 (Conditions.cases 必須) ----
    from driving_log_replayer_v2.real_log_sim_comparison.lib._cases_config import (  # noqa: PLC0415
        load_cases_config,
    )

    cases_cfg = load_cases_config(scenario_config)

    logger.info(f"Stage 5: step5_analyze_nstep over {len(cases_cfg.cases)} case(s)")
    for case in cases_cfg.cases:
        logger.info(f"  case: tag={case.tag}, vehicle_model_type={case.vehicle_model_type}")
        env_case = env.copy()
        env_case["CASE_TAG"] = case.tag
        try:
            _run([
                sys.executable, "-m",
                "driving_log_replayer_v2.real_log_sim_comparison.step5_analyze_nstep",
                "--case-tag", case.tag,
                "--scenario", scenario_config,
            ], env=env_case, timeout=1800)
        except RuntimeError as exc:
            logger.warning(f"Stage 5 (case={case.tag}) failed but continuing: {exc}")

    # ---- Stage 6: ケース集約解析 (overlay 図 + cases_summary.md) ----
    logger.info("Stage 6: step6_analyze_cases (cross-case aggregation)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step6_analyze_cases",
            "--scenario", scenario_config,
        ], env=env, timeout=600)
    except RuntimeError as exc:
        logger.warning(f"Stage 6 (step6_analyze_cases) failed but continuing: {exc}")

    # ---- Stage 7: 車両モデルパラメータ sweep 同定 (rollout, 追加設定不要) ----
    # 実機 lite (Stage 1 出力) のみを使い、k_us/ステア・加速度時定数等を
    # free-running rollout sweep で同定する独立ステージ (2D ペア sweep 含む)。
    logger.info("Stage 7: step7_sweep_params (vehicle model parameter sweep via rollout)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step7_sweep_params",
        ], env=env, timeout=1800)
    except RuntimeError as exc:
        logger.warning(f"Stage 7 (step7_sweep_params) failed but continuing: {exc}")

    # ---- Stage 8: DiffusionPlanner 計画軌跡比較 (planner レベルの乖離分離) ----
    logger.info("Stage 8: step8_compare_dp_trajectory (planner trajectory real vs sim)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step8_compare_dp_trajectory",
        ], env=env, timeout=900)
    except RuntimeError as exc:
        logger.warning(f"Stage 8 (step8_compare_dp_trajectory) failed but continuing: {exc}")

    # ---- Stage 11: comparison/ 配下の全アセットを 1 枚に埋め込んだ自己完結 HTML 生成 ----
    logger.info("Stage 11: step11_build_html_report (result_archive/real_log_sim_comparison/report.html)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step11_build_html_report",
        ], env=env, timeout=300)
    except RuntimeError as exc:
        logger.warning(f"Stage 11 (step11_build_html_report) failed but continuing: {exc}")

    # ---- Stage 12: 開発者向け notebook (report.ipynb) 生成 ----
    logger.info("Stage 12: step12_build_notebook (result_archive/real_log_sim_comparison/report.ipynb)")
    try:
        _run([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step12_build_notebook",
        ], env=env, timeout=120)
    except RuntimeError as exc:
        logger.warning(f"Stage 12 (step12_build_notebook) failed but continuing: {exc}")

    # ---- 生成物カウント (E1: 沈黙の失敗対策) ----
    # Stage 3/5 は失敗継続するため、実際に出力が出た数を数えて成否判定の材料にする。
    def _lite_exists(tag: str) -> bool:
        return (lite_dir / f"{tag}.lite").exists() or (lite_dir / f"{tag}.lite.mcap").exists()

    nstep_dir = comparison_dir / "nstep"
    sim_produced = sum(1 for r in sim_cfg.runs if _lite_exists(r.tag))
    cases_produced = sum(
        1 for c in cases_cfg.cases if (nstep_dir / c.tag / "nstep_delta.csv").exists()
    )
    counts = {
        "sim_runs_expected": len(sim_cfg.runs),
        "sim_runs_produced": sim_produced,
        "cases_expected": len(cases_cfg.cases),
        "cases_produced": cases_produced,
        "report_ok": int((comparison_dir / "report.md").exists()),
        "cases_summary_ok": int((comparison_dir / "cases" / "cases_summary.md").exists()),
        "param_sweep_ok": int((comparison_dir / "param_sweep" / "param_sweep_summary.md").exists()),
        "dp_compare_ok": int(
            (comparison_dir / "figures" / "dp_real_vs_sim.svg").exists()
            or (comparison_dir / "figures" / "dp_real_vs_sim.fig.json").exists()
        ),
        # report.html / report.ipynb は comparison/ の親 (result_archive/) 直下に生成される。
        "report_html_ok": int((comparison_dir.parent / "report.html").exists()),
        "report_ipynb_ok": int((comparison_dir.parent / "report.ipynb").exists()),
    }
    logger.info(
        f"Pipeline outputs: sim_runs {sim_produced}/{len(sim_cfg.runs)}, "
        f"cases {cases_produced}/{len(cases_cfg.cases)}, "
        f"report={counts['report_ok']}, cases_summary={counts['cases_summary_ok']}, "
        f"param_sweep={counts['param_sweep_ok']}, dp_compare={counts['dp_compare_ok']}, "
        f"report_html={counts['report_html_ok']}, report_ipynb={counts['report_ipynb_ok']}"
    )
    return counts


def _load_compare_config(scenario_path_str: str) -> dict[str, Any]:
    """scenario.yaml の Conditions から step4_compare_logs 用設定を抽出する。

    Conditions に以下のキーを認識する（すべて任意）:
      - scenario_name (str): 図タイトル用シナリオ名
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

        # real_provenance (任意): 実機データ取得時の pilot-auto.x2 / DP 重みの自由記述
        # (例 "autoware v0.48.x / DP exp neighbor320_xxx")。版・重み差の解釈用に provenance 掲載。
        if "real_provenance" in conditions:
            cfg["real_provenance"] = str(conditions["real_provenance"])

        # loop_waypoints (任意, 既定 0): 実走軌跡形状を強制する route-shaping 実験オプション。
        # 注: D0 (sim 早期停止) の真因は赤信号 replay であり routing ではない (live sim で確定)
        # ため、loop_waypoints は D0 の修正ではない。route は start+goal でも周回全体を引く。
        if "loop_waypoints" in conditions:
            try:
                cfg["loop_waypoints"] = int(conditions["loop_waypoints"])
            except (TypeError, ValueError):
                cfg["loop_waypoints"] = 0

        # traffic_signals (任意, 既定 replay): 信号の扱い。replay=実機 bag の信号を再現、
        # green=全信号常時 green。replay は到達時刻 desync で実機が green 通過した信号に
        # sim ego が赤で当たり永久停止する (D0 の真因) ことがあり、green で周回を完走できる。
        if "traffic_signals" in conditions:
            ts = str(conditions["traffic_signals"]).strip().lower()
            cfg["traffic_signals"] = ts if ts in ("replay", "green", "none") else "replay"

        # reproduce_perception (任意, 既定 false): true で実機 input_bag の先行車を ego-pose 同期で
        # 各 sim に注入 (perception_reproducer_node)。NPC 無の auto-scenario に実機の先行車追従
        # (停止・加減速) を再現させる。step3 が REPRODUCE_BAG 経由で起動。
        if "reproduce_perception" in conditions:
            cfg["reproduce_perception"] = bool(conditions["reproduce_perception"])

        # skip_sim (任意, 既定 false): true で Stage 3 (closed-loop sim 実行) をスキップし、
        # 実機ログのみの解析 (open-loop N-step / param sweep / カバレッジ等) を実行する。
        # closed-loop 比較が不要なとき・マルチ DS バッチの時間短縮用。ローカルは env SKIP_SIM=1
        # でも指定できる (run_pipeline 参照)。
        if "skip_sim" in conditions:
            cfg["skip_sim"] = bool(conditions["skip_sim"])

        # scenario_config: models / cases / sim_runs はすべて scenario.yaml 自身に統合済み。
        # scenario_path をそのまま保持する (cases_config / sim_runs_config の外部ファイル参照は廃止)。
        cfg["scenario_config"] = str(scenario_path)

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


def _run(
    cmd: list[str],
    cwd: str | None = None,
    env: dict | None = None,
    timeout: int | None = None,
    log_file: Path | None = None,
) -> None:
    """subprocess.run の薄いラッパ。非 0 終了で RuntimeError。

    log_file 指定時はサブプロセスの stdout/stderr をそのファイルへリダイレクトし
    (標準出力には出さず分離保存する)。sim run ごとのログ分離に使う。
    """
    try:
        if log_file is not None:
            log_file.parent.mkdir(parents=True, exist_ok=True)
            with log_file.open("w") as f:
                result = subprocess.run(
                    cmd, check=False, cwd=cwd, env=env, timeout=timeout,
                    stdout=f, stderr=subprocess.STDOUT,
                )
        else:
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
