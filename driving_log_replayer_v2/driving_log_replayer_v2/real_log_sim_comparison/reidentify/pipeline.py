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
    collection_dir: Path, out_dir: Path, scenario: Path, n_jobs: int, target: str,
) -> Path:
    from . import fit_lon  # noqa: PLC0415

    out = out_dir / "phase1_acc.yaml"
    fit_lon.run(collection_dir, out, n_jobs=n_jobs, scenario=scenario, target=target)
    return out


def _step_fit_steer(
    collection_dir: Path,
    out_dir: Path,
    phase1_out: Path,
    scenario: Path,
    n_jobs: int,
    target: str,
) -> Path:
    from . import fit_steer  # noqa: PLC0415

    out = out_dir / "phase2_steer.yaml"
    fit_steer.run(
        collection_dir,
        out,
        phase1_params_path=phase1_out,
        scenario=scenario,
        n_jobs=n_jobs,
        target=target,
    )
    return out


def _step_fit_xy(
    collection_dir: Path, out_dir: Path, phase2_out: Path, scenario: Path, n_jobs: int,
    target: str,
) -> Path:
    from . import fit_xy  # noqa: PLC0415

    out = out_dir / "phase3_xy.yaml"
    fit_xy.run(
        collection_dir,
        out,
        phase2_params_path=phase2_out,
        scenario=scenario,
        n_jobs=n_jobs,
        target=target,
    )
    return out


def _step_fit_merge(
    collection_dir: Path,
    scenario: Path,
    out_dir: Path,
    warm_start_out: Path | None,
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
        phase3_params_path=warm_start_out,
        n_trials=n_trials,
        n_jobs=n_jobs,
        metrics_out=metrics_out,
    )
    return out, metrics_out, result


def _step_evaluate(
    collection_dir: Path,
    scenario: Path,
    out_dir: Path,
    direct_fit_out: Path | None,
    n_jobs: int,
) -> tuple[Path, Path, dict]:
    """merge を実行しない評価専用ステージ (fit 完全スキップ / direct-fit のみ)。"""
    from . import fit_merge  # noqa: PLC0415

    out = out_dir / "tuned_params.yaml"
    metrics_out = out_dir / "metrics.csv"
    result = fit_merge.run_evaluate(
        collection_dir,
        scenario,
        out,
        direct_fit_params_path=direct_fit_out,
        metrics_out=metrics_out,
        n_jobs=n_jobs,
    )
    return out, metrics_out, result


def _step_report(
    out_dir: Path,
    phase1_out: Path,
    phase2_out: Path,
    phase3_out: Path,
    tuned_out: Path,
    metrics_out: Path,
    release_out: Path | None,
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
    parser.add_argument(
        "--input-param", type=Path, default=None,
        help="リリース基準の simulator_model.param.yaml。release 指定がある場合のみ必須",
    )
    # Makefile の N_TRIALS 既定値と揃える。
    parser.add_argument("--n-trials", type=int, default=160)
    parser.add_argument("--n-jobs", type=int, default=32)
    parser.add_argument("--force-extract", action="store_true", help="既存 reidentify_cache.csv も再生成する")
    parser.add_argument(
        "--report-only", action="store_true",
        help="extract/fit/release を行わず、既存の成果物から report.html だけを再生成する",
    )
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

        cfg = load_model_config(args.scenario)
        # report-only は release 済み成果物を読むだけなので input-param を必要としない。
        if cfg.release is not None and not args.report_only:
            if args.input_param is None:
                parser.error("--input-param is required when the scenario declares a release")
            validate_input(args.input_param)
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))

    n_jobs = normalize_parallel_jobs(args.n_jobs)
    out_dir = args.root / DEFAULT_OUTPUT_DIR_NAME
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.report_only:
        # 高価な fit を再実行せず、既存成果物 (tuned_params.yaml / metrics.csv 等) から
        # report.html だけを作り直す。report は ROS 非依存なので単体で回収できる。
        tuned_out = out_dir / "tuned_params.yaml"
        metrics_out = out_dir / "metrics.csv"
        for required in (tuned_out, metrics_out):
            if not required.is_file():
                parser.error(f"--report-only requires an existing artifact: {required}")
        release_out = out_dir / "simulator_model.param.yaml" if cfg.release is not None else None
        print("\n[pipeline] === report-only ===")
        _step_report(
            out_dir,
            out_dir / "phase1_acc.yaml",
            out_dir / "phase2_steer.yaml",
            out_dir / "phase3_xy.yaml",
            tuned_out, metrics_out, release_out,
            {}, {"metadata": {"skipped": []}}, args.scenario, n_jobs,
        )
        print(f"\n[pipeline] report-only 完了。成果物: {out_dir / 'report.html'}")
        return

    direct_stages = cfg.fit.direct_stages
    target = cfg.fit.target
    phase_paths = {
        "lon": out_dir / "phase1_acc.yaml",
        "steer": out_dir / "phase2_steer.yaml",
        "xy": out_dir / "phase3_xy.yaml",
    }
    # 実行しない direct-fit ステージの古い成果物はレポートの誤表示を避けるため削除する。
    for stage_name, path in phase_paths.items():
        if stage_name not in direct_stages and path.exists():
            path.unlink()
    # release をスキップする場合、前回実行の simulator_model.param.yaml が残っていると
    # レポート 8 章が古いリリース値を今回のものとして表示してしまうため削除する。
    released_path = out_dir / "simulator_model.param.yaml"
    if cfg.release is None and released_path.exists():
        released_path.unlink()

    # ステージ番号: extract + 実行する fit ステージ + evaluate/merge + [release] + report。
    total = 1 + len(cfg.fit.direct_stages) + 1 + (1 if cfg.release is not None else 0) + 1
    step = 0

    def banner(name: str) -> None:
        nonlocal step
        step += 1
        print(f"\n[pipeline] === {step}/{total} {name} ===")

    banner("extract")
    extraction_summary = _step_extract(args.root, force=args.force_extract)

    if "lon" in direct_stages:
        banner("fit_lon")
        _step_fit_lon(args.root, out_dir, args.scenario, n_jobs, target)
    if "steer" in direct_stages:
        banner("fit_steer")
        _step_fit_steer(
            args.root, out_dir, phase_paths["lon"], args.scenario, n_jobs, target,
        )
    if "xy" in direct_stages:
        banner("fit_xy")
        _step_fit_xy(args.root, out_dir, phase_paths["steer"], args.scenario, n_jobs, target)

    # direct-fit の完全なプレフィックス (lon,steer,xy) が走ったときだけ phase3 を warm-start に使う。
    last_direct_out = (
        phase_paths["xy"] if direct_stages == ("lon", "steer", "xy") else None
    )
    if cfg.fit.has_merge:
        banner("fit_merge")
        tuned_out, metrics_out, fit_result = _step_fit_merge(
            args.root, args.scenario, out_dir, last_direct_out, args.n_trials, n_jobs,
        )
    else:
        # merge を実行しない: direct-fit の最終成果物 (あれば) を tuned として評価する。
        direct_fit_out = phase_paths[direct_stages[-1]] if direct_stages else None
        banner("evaluate")
        tuned_out, metrics_out, fit_result = _step_evaluate(
            args.root, args.scenario, out_dir, direct_fit_out, n_jobs,
        )

    release_out = None
    if cfg.release is not None:
        banner("release")
        release_out = _step_release(tuned_out, out_dir, args.input_param, args.scenario)

    banner("report")
    _step_report(
        out_dir, phase_paths["lon"], phase_paths["steer"], phase_paths["xy"],
        tuned_out, metrics_out, release_out,
        extraction_summary, fit_result, args.scenario, n_jobs,
    )

    print(f"\n[pipeline] 完了。成果物: {out_dir}")


if __name__ == "__main__":
    main()
