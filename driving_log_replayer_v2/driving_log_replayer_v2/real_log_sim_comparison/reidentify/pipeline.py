#!/usr/bin/env python3
"""Main reidentify pipeline orchestrator."""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

from ..lib._parallel import default_parallel_jobs, normalize_parallel_jobs
from .settings import DEFAULT_INPUT_PARAM, DEFAULT_OUTPUT_DIR_NAME

ALL_STEPS = ("extract", "fit_lon", "fit_steer", "fit_merge", "report", "release")


def _step_extract(collection_dir: Path, *, force: bool = False) -> None:
    from . import extract  # noqa: PLC0415 (ROS dependencies stay behind this step)

    extract.extract_collection(collection_dir, force=force)


def _step_fit_lon(collection_dir: Path, out_dir: Path, n_jobs: int) -> Path:
    from . import fit_lon  # noqa: PLC0415

    out = out_dir / "phase1_acc.yaml"
    fit_lon.run(collection_dir, out, n_jobs=n_jobs)
    return out


def _step_fit_steer(
    collection_dir: Path,
    out_dir: Path,
    phase1_out: Path,
    scenario: Path,
    case: str,
    n_jobs: int,
) -> Path:
    from . import fit_steer  # noqa: PLC0415

    out = out_dir / "phase2_steer.yaml"
    fit_steer.run(
        collection_dir,
        out,
        phase1_params_path=phase1_out,
        scenario=scenario,
        case=case,
        n_jobs=n_jobs,
    )
    return out


def _step_fit_merge(
    collection_dir: Path,
    scenario: Path,
    out_dir: Path,
    phase2_out: Path,
    case: str,
    n_trials: int,
    n_jobs: int,
) -> Path:
    from . import fit_merge  # noqa: PLC0415

    out = out_dir / "tuned_params.yaml"
    fit_merge.run(
        collection_dir,
        scenario,
        out,
        case=case,
        phase2_params_path=phase2_out,
        n_trials=n_trials,
        n_jobs=n_jobs,
    )
    return out


def _step_report(
    collection_dir: Path,
    scenario: Path,
    case: str,
    out_dir: Path,
    tuned_out: Path,
    n_jobs: int,
    closed_loop_uuids: str,
    n_curve_ds: int,
) -> None:
    cmd = [
        sys.executable,
        "-m",
        "driving_log_replayer_v2.real_log_sim_comparison.physical_validity_report",
        "--params",
        str(tuned_out),
        "--collection-dir",
        str(collection_dir),
        "--out",
        str(out_dir / "physical_validity_report.html"),
        "--release-note-out",
        str(out_dir / "physical_validity_release_note.html"),
        "--scenario",
        str(scenario),
        "--case",
        case,
        "--metrics-cache",
        str(out_dir / "metrics_cache.csv"),
        "--closed-loop-uuids",
        closed_loop_uuids,
        "--n-curve-ds",
        str(n_curve_ds),
        "--n-jobs",
        str(n_jobs),
    ]
    subprocess.run(cmd, check=True)


def _step_release(tuned_out: Path, out_dir: Path, input_param: Path) -> Path:
    from . import release_params  # noqa: PLC0415

    return release_params.release(input_param, tuned_out, out_dir)


def _parse_steps(value: str) -> list[str]:
    steps = [s.strip() for s in value.split(",") if s.strip()] if value else list(ALL_STEPS)
    unknown = set(steps) - set(ALL_STEPS)
    if unknown:
        raise SystemExit(f"未知のステップ: {sorted(unknown)} (選択肢: {ALL_STEPS})")
    return steps


def main() -> None:
    parser = argparse.ArgumentParser(
        description="reidentify pipeline (extract→fit_lon→fit_steer→fit_merge→report→release)"
    )
    parser.add_argument("--collection-dir", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--case", default="current")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help=f"中間/最終成果物の出力先 (既定: <collection-dir>/{DEFAULT_OUTPUT_DIR_NAME})",
    )
    parser.add_argument("--input-param", type=Path, default=DEFAULT_INPUT_PARAM)
    parser.add_argument("--n-trials", type=int, default=50)
    parser.add_argument("--n-jobs", type=int, default=default_parallel_jobs())
    parser.add_argument("--only", default="", help=f"実行ステップのカンマ区切り指定: {','.join(ALL_STEPS)}")
    parser.add_argument("--force-extract", action="store_true", help="既存 reidentify_cache.csv も再生成する")
    parser.add_argument("--closed-loop-uuids", default="")
    parser.add_argument("--n-curve-ds", type=int, default=0)
    args = parser.parse_args()

    n_jobs = normalize_parallel_jobs(args.n_jobs)
    out_dir = args.out_dir or (args.collection_dir / DEFAULT_OUTPUT_DIR_NAME)
    out_dir.mkdir(parents=True, exist_ok=True)
    steps = _parse_steps(args.only)

    phase1_out = out_dir / "phase1_acc.yaml"
    phase2_out = out_dir / "phase2_steer.yaml"
    tuned_out = out_dir / "tuned_params.yaml"

    if "extract" in steps:
        print("\n[pipeline] === Step1: extract ===")
        _step_extract(args.collection_dir, force=args.force_extract)
    if "fit_lon" in steps:
        print("\n[pipeline] === Step2: fit_lon ===")
        phase1_out = _step_fit_lon(args.collection_dir, out_dir, n_jobs)
    if "fit_steer" in steps:
        print("\n[pipeline] === Step3: fit_steer ===")
        phase2_out = _step_fit_steer(args.collection_dir, out_dir, phase1_out, args.scenario, args.case, n_jobs)
    if "fit_merge" in steps:
        print("\n[pipeline] === Step4a: fit_merge ===")
        tuned_out = _step_fit_merge(
            args.collection_dir,
            args.scenario,
            out_dir,
            phase2_out,
            args.case,
            args.n_trials,
            n_jobs,
        )
    if "report" in steps:
        print("\n[pipeline] === Step4b: report ===")
        _step_report(
            args.collection_dir,
            args.scenario,
            args.case,
            out_dir,
            tuned_out,
            n_jobs,
            args.closed_loop_uuids,
            args.n_curve_ds,
        )
    if "release" in steps:
        print("\n[pipeline] === Step4c: release ===")
        _step_release(tuned_out, out_dir, args.input_param)

    print(f"\n[pipeline] 完了。成果物: {out_dir}")


if __name__ == "__main__":
    main()
