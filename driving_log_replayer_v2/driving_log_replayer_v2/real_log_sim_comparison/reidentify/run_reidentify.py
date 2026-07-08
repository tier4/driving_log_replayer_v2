#!/usr/bin/env python3
"""reidentify v2 パイプラインのオーケストレーションエントリポイント。"""
from __future__ import annotations

import argparse
from pathlib import Path

from ..lib._parallel import default_parallel_jobs, normalize_parallel_jobs
from .settings import DEFAULT_INPUT_PARAM, DEFAULT_OUTPUT_DIR_NAME

ALL_STEPS = ("extract", "fit_lon", "fit_steer", "fit_merge", "report", "release")


def _step_extract(collection_dir: Path) -> None:
    from . import extract  # noqa: PLC0415 (ROS 依存を --only で回避可能にするため遅延 import)

    extract.extract_collection(collection_dir)


def _step_fit_lon(collection_dir: Path, out_dir: Path, n_jobs: int) -> Path:
    from . import fit_lon  # noqa: PLC0415

    out = out_dir / "phase1_acc.yaml"
    fit_lon.run(collection_dir, out, n_jobs=n_jobs)
    return out


def _step_fit_steer(
    collection_dir: Path, out_dir: Path, phase1_out: Path, scenario: Path, case: str, n_jobs: int,
) -> Path:
    from . import fit_steer  # noqa: PLC0415

    out = out_dir / "phase2_steer.yaml"
    fit_steer.run(
        collection_dir, out, phase1_params_path=phase1_out, scenario=scenario, case=case, n_jobs=n_jobs,
    )
    return out


def _step_fit_merge(
    collection_dir: Path, scenario: Path, out_dir: Path, phase2_out: Path, case: str, n_trials: int, n_jobs: int,
) -> Path:
    from . import fit_merge  # noqa: PLC0415

    out = out_dir / "tuned_params.yaml"
    fit_merge.run(
        collection_dir, scenario, out, case=case, phase2_params_path=phase2_out, n_trials=n_trials, n_jobs=n_jobs,
    )
    return out


def _step_report(collection_dir: Path, scenario: Path, case: str, out_dir: Path, tuned_out: Path) -> None:
    from . import report  # noqa: PLC0415

    report.build_report(
        collection_dir, tuned_out, out_dir / "validity_report.png", out_dir / "validity_report.txt",
        scenario=scenario, case=case,
    )


def _step_release(tuned_out: Path, out_dir: Path, input_param: Path) -> None:
    from . import release_params  # noqa: PLC0415

    release_params.release(input_param, tuned_out, out_dir)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="reidentify v2 パイプライン (extract→fit_lon→fit_steer→fit_merge→report→release)"
    )
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--case", default="current")
    ap.add_argument(
        "--out-dir", type=Path, default=None,
        help=f"中間/最終成果物の出力先 (既定: <collection-dir>/{DEFAULT_OUTPUT_DIR_NAME})",
    )
    ap.add_argument("--input-param", type=Path, default=DEFAULT_INPUT_PARAM, help="リリース用ベース simulator_model.param.yaml")
    ap.add_argument("--n-trials", type=int, default=50)
    ap.add_argument("--n-jobs", type=int, default=default_parallel_jobs())
    ap.add_argument(
        "--only", default="",
        help=f"実行するステップをカンマ区切りで指定 (既定: 全ステップ)。選択肢: {','.join(ALL_STEPS)}",
    )
    args = ap.parse_args()
    args.n_jobs = normalize_parallel_jobs(args.n_jobs)

    out_dir = args.out_dir or (args.collection_dir / DEFAULT_OUTPUT_DIR_NAME)
    out_dir.mkdir(parents=True, exist_ok=True)

    steps = [s.strip() for s in args.only.split(",") if s.strip()] if args.only else list(ALL_STEPS)
    unknown = set(steps) - set(ALL_STEPS)
    if unknown:
        raise SystemExit(f"未知のステップ: {sorted(unknown)} (選択肢: {ALL_STEPS})")

    phase1_out = out_dir / "phase1_acc.yaml"
    phase2_out = out_dir / "phase2_steer.yaml"
    tuned_out = out_dir / "tuned_params.yaml"

    if "extract" in steps:
        print("\n[run_reidentify] === Step1: extract ===")
        _step_extract(args.collection_dir)
    if "fit_lon" in steps:
        print("\n[run_reidentify] === Step2: fit_lon ===")
        phase1_out = _step_fit_lon(args.collection_dir, out_dir, args.n_jobs)
    if "fit_steer" in steps:
        print("\n[run_reidentify] === Step3: fit_steer ===")
        phase2_out = _step_fit_steer(args.collection_dir, out_dir, phase1_out, args.scenario, args.case, args.n_jobs)
    if "fit_merge" in steps:
        print("\n[run_reidentify] === Step4a: fit_merge ===")
        tuned_out = _step_fit_merge(
            args.collection_dir, args.scenario, out_dir, phase2_out, args.case, args.n_trials, args.n_jobs,
        )
    if "report" in steps:
        print("\n[run_reidentify] === Step4b: report ===")
        _step_report(args.collection_dir, args.scenario, args.case, out_dir, tuned_out)
    if "release" in steps:
        print("\n[run_reidentify] === Step4c: release ===")
        _step_release(tuned_out, out_dir, args.input_param)

    print(f"\n[run_reidentify] 完了。成果物: {out_dir}")


if __name__ == "__main__":
    main()
