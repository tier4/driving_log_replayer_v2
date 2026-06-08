#!/usr/bin/env python3
"""sim 実行済みバンドルに対して解析ステージ (Stage 4〜11) だけを再実行する CLI.

`make local_cloud_run` のフルパイプライン (Stage 1〜3: lite 抽出・scenario 生成・sim 実行)
は重いため、解析コード (step4〜step11) を変更して結果を作り直したいときに、既存の
出力バンドル (lite/ + comparison/ を含むディレクトリ) を入力として解析だけを回す。
ROS launch 不要の純 Python CLI (オーケストレーションは evaluator_node.run_analysis を共用)。

使い方 (Makefile 経由が簡単):
    make -C real_log_sim_comparison local_analysis_run                # sample/out/latest を再解析
    make -C real_log_sim_comparison local_analysis_run OUT_DIR=sample/out/20260603_211156

直接実行:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.run_analysis \
        --bundle-dir <out_dir or .../result_archive/real_log_sim_comparison> \
        --scenario sample/scenario.yaml \
        [--map-osm /path/to/lanelet2_map.osm]

注意: comparison/ 配下の解析成果物と report.html は上書き再生成される
(lite/ と scenarios/ には触れない)。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys

from .evaluator_node import _load_compare_config, build_common_env, run_analysis


def _resolve_bundle_dir(raw: Path) -> Path:
    """lite/ を含むバンドルディレクトリを解決する。

    渡されたパスが以下のいずれでも受け付ける:
      - バンドル自体 (.../result_archive/real_log_sim_comparison)
      - out_dir (.../out/<timestamp>; 配下の result_archive/real_log_sim_comparison を探索)
    """
    candidates = [
        raw,
        raw / "result_archive" / "real_log_sim_comparison",
        raw / "real_log_sim_comparison",
    ]
    for c in candidates:
        if (c / "lite").is_dir():
            return c.resolve()
    raise FileNotFoundError(
        f"lite/ を含むバンドルが見つかりません: {raw} "
        f"(探索: {[str(c) for c in candidates]})"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="sim 実行済みバンドルの解析ステージ (Stage 4〜11) 再実行"
    )
    parser.add_argument(
        "--bundle-dir",
        required=True,
        help="lite/ + comparison/ を含むバンドル、または out/<timestamp> ディレクトリ",
    )
    parser.add_argument(
        "--scenario",
        required=True,
        help="scenario.yaml (Conditions の cases_config / sim_runs_config / curve_config_yaml を読む)",
    )
    parser.add_argument(
        "--map-osm",
        default="",
        help="lanelet2_map.osm のパス (省略時は地図背景なし)",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("run_analysis")

    bundle_dir = _resolve_bundle_dir(Path(args.bundle_dir))
    lite_dir = bundle_dir / "lite"
    comparison_dir = bundle_dir / "comparison"
    logger.info(f"Bundle: {bundle_dir}")

    compare_cfg = _load_compare_config(args.scenario)
    if not compare_cfg:
        print(f"ERROR: scenario.yaml が読めません: {args.scenario}", file=sys.stderr)
        sys.exit(2)

    # build_common_env は <map_path>/lanelet2_map.osm を解決するため親ディレクトリを渡す
    map_path = str(Path(args.map_osm).parent) if args.map_osm else ""
    env = build_common_env(comparison_dir, map_path, compare_cfg, logger)

    counts = run_analysis(lite_dir, comparison_dir, env, compare_cfg, logger)

    sim_p, sim_e = counts["sim_runs_produced"], counts["sim_runs_expected"]
    case_p, case_e = counts["cases_produced"], counts["cases_expected"]
    print(
        f"\n解析完了: sim_runs {sim_p}/{sim_e}, cases {case_p}/{case_e}, "
        f"report={counts['report_ok']}, cases_summary={counts['cases_summary_ok']}, "
        f"param_sweep={counts['param_sweep_ok']}, report_html={counts['report_html_ok']}, "
        f"report_ipynb={counts['report_ipynb_ok']}"
    )
    print(f"レポート: {bundle_dir / 'report.html'}")
    print(f"notebook: {bundle_dir / 'report.ipynb'}")
    if case_p == 0:
        print("WARNING: case 出力が 0 件です (lite/ に real.lite があるか確認してください)",
              file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
