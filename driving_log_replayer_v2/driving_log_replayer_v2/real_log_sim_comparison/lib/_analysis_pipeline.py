"""ROS-free orchestration helpers for real_log_sim_comparison analysis stages."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
from typing import Any


def run_command(
    cmd: list[str],
    cwd: str | None = None,
    env: dict | None = None,
    timeout: int | None = None,
    log_file: Path | None = None,
) -> None:
    """Run a subprocess and raise RuntimeError on failure."""
    try:
        if log_file is not None:
            log_file.parent.mkdir(parents=True, exist_ok=True)
            with log_file.open("w", encoding="utf-8", errors="replace") as f:
                result = subprocess.run(
                    cmd, check=False, cwd=cwd, env=env, timeout=timeout,
                    stdout=f, stderr=subprocess.STDOUT,
                )
        else:
            result = subprocess.run(cmd, check=False, cwd=cwd, env=env, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"Command timed out after {timeout}s: {' '.join(str(c) for c in cmd)}") from exc
    if result.returncode != 0:
        msg = f"Command failed (rc={result.returncode}): {' '.join(str(c) for c in cmd)}"
        raise RuntimeError(msg)


def build_common_env(
    comparison_dir: Path, map_path: str, compare_cfg: dict[str, Any], logger
) -> dict[str, str]:
    """Build env shared by analysis stages without importing ROS modules."""
    env = os.environ.copy()
    env["BEST_MODEL_BASE_DIR"] = str(comparison_dir.parent)

    map_osm = Path(map_path) / "lanelet2_map.osm" if map_path else None
    if map_osm is not None and map_osm.is_file():
        env["MAP_OSM_PATH"] = str(map_osm)
        logger.info(f"Map OSM: {map_osm}")
    else:
        env["MAP_OSM_PATH"] = ""
        logger.warning(f"lanelet2_map.osm not found at {map_osm}; map background will be omitted")

    if compare_cfg.get("scenario_name"):
        env["SCENARIO_NAME"] = compare_cfg["scenario_name"]
    env["REAL_PROVENANCE"] = compare_cfg.get("real_provenance", "")
    env["LOOP_WAYPOINTS"] = str(compare_cfg.get("loop_waypoints", 0))

    reproduce_perception = bool(compare_cfg.get("reproduce_perception", False))
    default_signals = "none" if reproduce_perception else "replay"
    env["TRAFFIC_SIGNALS"] = str(compare_cfg.get("traffic_signals", default_signals))
    env["EGO_REPLAY_DURATION"] = str(compare_cfg.get("ego_replay_duration", 0.0))
    env["REPLAY_PREROLL"] = str(compare_cfg.get("replay_preroll", 0.0))
    env["REPLAY_POSITION_BASED"] = "1" if compare_cfg.get("replay_position_based", False) else ""
    return env


def run_analysis(
    lite_dir: Path,
    comparison_dir: Path,
    env: dict[str, str],
    compare_cfg: dict[str, Any],
    logger,
) -> dict[str, int]:
    """Run analysis stages for an existing lite/ bundle."""
    scenario_config = compare_cfg.get("scenario_config", "")
    if not scenario_config or not Path(scenario_config).exists():
        raise RuntimeError(
            "scenario.yaml が見つかりません。Conditions.models / sim_runs / cases を "
            "scenario.yaml に直接記述してください。"
            f" got: {scenario_config!r}"
        )

    from driving_log_replayer_v2.real_log_sim_comparison.lib._models_config import (  # noqa: PLC0415
        load_models_doc,
        load_run_models,
    )

    sim_cfg = load_run_models(scenario_config)
    cases_cfg = load_models_doc(scenario_config)

    skip_ol = env.get("SKIP_OL") == "1"
    if skip_ol:
        logger.info("SKIP_OL is enabled. Skipping all analysis stages.")
        return {
            "sim_runs_produced": 0,
            "sim_runs_expected": len(sim_cfg.runs),
            "cases_produced": 0,
            "cases_expected": len(cases_cfg.cases),
            "report_ok": 0,
            "cases_summary_ok": 0,
        }

    env = env.copy()
    env["SCENARIO_CONFIG_YAML"] = scenario_config

    logger.info(f"Stage OL1: step_ol1_analyze_nstep over {len(cases_cfg.cases)} case(s)")
    for case in cases_cfg.cases:
        logger.info(f"  case: tag={case.tag}, vehicle_model_type={case.vehicle_model_type}")
        env_case = env.copy()
        env_case["CASE_TAG"] = case.tag
        try:
            run_command([
                sys.executable, "-m",
                "driving_log_replayer_v2.real_log_sim_comparison.step_ol1_analyze_nstep",
                "--case-tag", case.tag,
                "--scenario", scenario_config,
            ], env=env_case, timeout=1800)
        except RuntimeError as exc:
            logger.warning(f"Stage OL1 (case={case.tag}) failed but continuing: {exc}")

    logger.info("Stage OL2: step_ol2_analyze_cases (cross-case aggregation)")
    try:
        run_command([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step_ol2_analyze_cases",
            "--scenario", scenario_config,
        ], env=env, timeout=600)
    except RuntimeError as exc:
        logger.warning(f"Stage OL2 (step_ol2_analyze_cases) failed but continuing: {exc}")

    logger.info("Stage CL3: step_cl3_compare_logs (real + sim N-way)")
    try:
        run_command([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step_cl3_compare_logs",
        ], env=env, timeout=1800)
    except RuntimeError as exc:
        logger.warning(f"Stage CL3 (step_cl3_compare_logs) failed but continuing: {exc}")

    logger.info("Stage CL4: step_cl4_compare_dp_trajectory (planner trajectory real vs sim)")
    try:
        run_command([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step_cl4_compare_dp_trajectory",
        ], env=env, timeout=900)
    except RuntimeError as exc:
        logger.warning(f"Stage CL4 (step_cl4_compare_dp_trajectory) failed but continuing: {exc}")

    logger.info("Stage Report HTML: step_report_html (result_archive/real_log_sim_comparison/report.html)")
    try:
        run_command([
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step_report_html",
        ], env=env, timeout=300)
    except RuntimeError as exc:
        logger.warning(f"Stage Report HTML (step_report_html) failed but continuing: {exc}")

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
        "dp_compare_ok": int(
            (comparison_dir / "figures" / "dp_real_vs_sim.svg").exists()
            or (comparison_dir / "figures" / "dp_real_vs_sim.fig.json").exists()
        ),
        "report_html_ok": int((comparison_dir.parent / "report.html").exists()),
    }
    logger.info(
        f"Pipeline outputs: sim_runs {sim_produced}/{len(sim_cfg.runs)}, "
        f"cases {cases_produced}/{len(cases_cfg.cases)}, "
        f"report={counts['report_ok']}, cases_summary={counts['cases_summary_ok']}, "
        f"dp_compare={counts['dp_compare_ok']}, "
        f"report_html={counts['report_html_ok']}"
    )
    return counts


def load_compare_config(scenario_path_str: str) -> dict[str, Any]:
    """Load real_log_sim_comparison settings from Evaluation.Conditions."""
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

        if "real_provenance" in conditions:
            cfg["real_provenance"] = str(conditions["real_provenance"])

        if "loop_waypoints" in conditions:
            try:
                cfg["loop_waypoints"] = int(conditions["loop_waypoints"])
            except (TypeError, ValueError):
                cfg["loop_waypoints"] = 0

        if "traffic_signals" in conditions:
            ts = str(conditions["traffic_signals"]).strip().lower()
            cfg["traffic_signals"] = ts if ts in ("replay", "green", "none") else "replay"

        if "reproduce_perception" in conditions:
            cfg["reproduce_perception"] = bool(conditions["reproduce_perception"])

        if "ego_replay_duration" in conditions:
            try:
                cfg["ego_replay_duration"] = float(conditions["ego_replay_duration"])
            except (TypeError, ValueError):
                cfg["ego_replay_duration"] = 0.0

        if "replay_preroll" in conditions:
            try:
                cfg["replay_preroll"] = float(conditions["replay_preroll"])
            except (TypeError, ValueError):
                cfg["replay_preroll"] = 0.0

        if "replay_position_based" in conditions:
            cfg["replay_position_based"] = bool(conditions["replay_position_based"])

        if "skip_sim" in conditions:
            cfg["skip_sim"] = bool(conditions["skip_sim"])

        cfg["scenario_config"] = str(scenario_path)
        return cfg
    except Exception:
        return {}
