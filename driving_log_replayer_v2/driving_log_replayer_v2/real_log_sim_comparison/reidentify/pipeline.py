#!/usr/bin/env python3
"""Main reidentify pipeline orchestrator."""
from __future__ import annotations

import argparse
from pathlib import Path

from ..lib._parallel import normalize_parallel_jobs
from .settings import DEFAULT_OUTPUT_DIR_NAME


def _step_extract(collection_dir: Path, *, force: bool = False) -> dict:
    from . import extract  # noqa: PLC0415 (ROS dependencies stay behind this step)

    return extract.extract_collection(collection_dir, force=force)


def _step_fit_lon(
    collection_dir: Path, out_dir: Path, scenario: Path, n_jobs: int,
) -> Path:
    from . import fit_lon  # noqa: PLC0415

    out = out_dir / "phase1_acc.yaml"
    fit_lon.run(collection_dir, out, n_jobs=n_jobs, scenario=scenario)
    return out


def _step_fit_steer(
    collection_dir: Path,
    out_dir: Path,
    phase1_out: Path,
    scenario: Path,
    n_jobs: int,
) -> Path:
    from . import fit_steer  # noqa: PLC0415

    out = out_dir / "phase2_steer.yaml"
    fit_steer.run(
        collection_dir,
        out,
        phase1_params_path=phase1_out,
        scenario=scenario,
        n_jobs=n_jobs,
    )
    return out


def _step_fit_xy(
    collection_dir: Path, out_dir: Path, phase2_out: Path, scenario: Path, n_jobs: int,
) -> Path:
    from . import fit_xy  # noqa: PLC0415

    out = out_dir / "phase3_xy.yaml"
    fit_xy.run(
        collection_dir,
        out,
        phase2_params_path=phase2_out,
        scenario=scenario,
        n_jobs=n_jobs,
    )
    return out


def _step_fit_merge(
    collection_dir: Path,
    scenario: Path,
    out_dir: Path,
    phase3_out: Path,
    n_trials: int,
    n_jobs: int,
) -> tuple[Path, Path, dict]:
    from . import fit_merge  # noqa: PLC0415

    out = out_dir / "tuned_params.yaml"
    metrics_out = out_dir / "metrics.csv"
    result = fit_merge.run(
        collection_dir,
        scenario,
        out,
        phase3_params_path=phase3_out,
        n_trials=n_trials,
        n_jobs=n_jobs,
        metrics_out=metrics_out,
    )
    return out, metrics_out, result


def _step_report(
    out_dir: Path,
    phase1_out: Path,
    phase2_out: Path,
    phase3_out: Path,
    tuned_out: Path,
    metrics_out: Path,
    release_out: Path,
    extraction_summary: dict,
    fit_result: dict,
    scenario: Path,
    n_jobs: int | None = None,
) -> None:
    from . import report  # noqa: PLC0415

    summary = dict(extraction_summary)
    fit_skipped = (fit_result.get("metadata") or {}).get("skipped") or []
    summary["skipped"] = [*(summary.get("skipped") or []), *fit_skipped]
    summary["n_skipped"] = int(summary.get("n_skipped", 0)) + len(fit_skipped)
    if n_jobs is None:
        report.run(
            tuned_out,
            metrics_out,
            out_dir / "report.html",
            failures=summary,
            collection_dir=out_dir.parent,
            scenario=scenario,
            extraction_summary=extraction_summary,
            phase1_params=phase1_out,
            phase2_params=phase2_out,
            phase3_params=phase3_out,
            release_params=release_out,
        )
    else:
        report.run(
            tuned_out,
            metrics_out,
            out_dir / "report.html",
            failures=summary,
            collection_dir=out_dir.parent,
            scenario=scenario,
            extraction_summary=extraction_summary,
            phase1_params=phase1_out,
            phase2_params=phase2_out,
            phase3_params=phase3_out,
            release_params=release_out,
            n_jobs=n_jobs,
        )


def _step_release(
    tuned_out: Path, out_dir: Path, input_param: Path, scenario: Path | None = None,
) -> Path:
    from . import release_params  # noqa: PLC0415

    return release_params.release(input_param, tuned_out, out_dir, scenario=scenario)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="reidentify pipeline (extract→fit_lon→fit_steer→fit_xy→fit_merge→release→report)"
    )
    parser.add_argument("--root", type=Path, required=True, help="datasets/ を持つ collection root")
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--input-param", type=Path, required=True)
    # Makefile の N_TRIALS 既定値と揃える。
    parser.add_argument("--n-trials", type=int, default=160)
    parser.add_argument("--n-jobs", type=int, default=32)
    parser.add_argument("--force-extract", action="store_true", help="既存 reidentify_cache.csv も再生成する")
    args = parser.parse_args()

    if args.n_trials < 1:
        parser.error("--n-trials must be at least 1")
    if args.n_jobs < 1:
        parser.error("--n-jobs must be at least 1")
    if not (args.root / "datasets").is_dir():
        parser.error(f"datasets directory not found: {args.root / 'datasets'}")
    try:
        from .model_config import load_model_config  # noqa: PLC0415
        from .release_params import validate_input  # noqa: PLC0415

        load_model_config(args.scenario)
        validate_input(args.input_param)
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))

    n_jobs = normalize_parallel_jobs(args.n_jobs)
    out_dir = args.root / DEFAULT_OUTPUT_DIR_NAME
    out_dir.mkdir(parents=True, exist_ok=True)

    print("\n[pipeline] === 1/7 extract ===")
    extraction_summary = _step_extract(args.root, force=args.force_extract)

    print("\n[pipeline] === 2/7 fit_lon ===")
    phase1_out = _step_fit_lon(args.root, out_dir, args.scenario, n_jobs)

    print("\n[pipeline] === 3/7 fit_steer ===")
    phase2_out = _step_fit_steer(
        args.root, out_dir, phase1_out, args.scenario, n_jobs
    )

    print("\n[pipeline] === 4/7 fit_xy ===")
    phase3_out = _step_fit_xy(args.root, out_dir, phase2_out, args.scenario, n_jobs)

    print("\n[pipeline] === 5/7 fit_merge ===")
    tuned_out, metrics_out, fit_result = _step_fit_merge(
        args.root,
        args.scenario,
        out_dir,
        phase3_out,
        args.n_trials,
        n_jobs,
    )

    print("\n[pipeline] === 6/7 release ===")
    release_out = _step_release(tuned_out, out_dir, args.input_param, args.scenario)

    print("\n[pipeline] === 7/7 report ===")
    _step_report(
        out_dir, phase1_out, phase2_out, phase3_out, tuned_out, metrics_out, release_out,
        extraction_summary, fit_result, args.scenario, n_jobs,
    )

    print(f"\n[pipeline] 完了。成果物: {out_dir}")


if __name__ == "__main__":
    main()
