"""直接同定ステージ (fit_lon / fit_steer / fit_xy / fit_merge) が共有する骨格処理。

各ステージは「CSV キャッシュを並列ロード → データセット単位で同定 → 前段成果物
YAML の検証と scenario 初期値の継承 → 成果物 YAML の書き出し」という同じ骨格を
持つ。ここではその骨格だけを提供し、同定のドメインロジックは各ステージに残す。
"""
from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from concurrent.futures import ProcessPoolExecutor, as_completed
import math
from pathlib import Path
import sys
from typing import Any, TypeVar

import numpy as np
import yaml

from ..lib._parallel import normalize_parallel_jobs
from .load_data import build_resampled, read_dataset_csv
from .model_config import load_model_config
from .parameter_constraints import PARAMETER_CONSTRAINTS, validate_parameters
from .settings import RESAMPLE_DT, TARGET_MODEL_NAME

R = TypeVar("R")

TUNING_TYPE_DIRECT_FIT = "physical_direct_fit"


def load_resampled(task: tuple[str, Path], *, stage: str) -> dict | None:
    """CSV キャッシュを読み込み、同定用の共通グリッドへ再サンプルする。"""
    ds_id, csv_path = task
    dfs = read_dataset_csv(csv_path)
    return build_resampled(dfs, RESAMPLE_DT, context=f"{stage}:{ds_id}")


def map_datasets(
    tasks: Sequence[tuple[str, Path]],
    worker: Callable[[tuple[str, Path]], R],
    *,
    stage: str,
    n_jobs: int,
) -> list[R]:
    """データセット並列処理の共通ループ。worker の例外は SKIP ログを出して除外する。"""
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    print(f"[{stage}] データセット並列ロード ({len(tasks)} 件)...")
    results: list[R] = []
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futures = {pool.submit(worker, task): task[0] for task in tasks}
        for future in as_completed(futures):
            try:
                results.append(future.result())
            except Exception as exc:  # noqa: BLE001 (dataset-local input failure)
                print(f"[{stage}] SKIP {futures[future]}: {exc}", file=sys.stderr)
    return results


def stage_metadata(
    collection_dir: Path,
    *,
    n_datasets: int,
    n_valid: int,
    phase: int,
    optimized: Iterable[str],
    **extra: Any,
) -> dict:
    """直接同定ステージ成果物の共通 metadata を組み立てる。"""
    return {
        "collection_dir": str(collection_dir),
        "n_datasets": n_datasets,
        "n_valid": n_valid,
        **extra,
        "tuning_type": TUNING_TYPE_DIRECT_FIT,
        "phase": phase,
        "optimized_parameters": sorted(optimized),
    }


def clamped_medians(
    rows: Sequence[tuple[float, ...]], keys: Sequence[str],
) -> dict[str, float]:
    """データセット別フィット結果の列 median を制約域へ clamp して返す。"""
    columns = list(map(np.asarray, zip(*rows)))
    return {
        key: PARAMETER_CONSTRAINTS[key].clamp(float(np.median(column)))
        for key, column in zip(keys, columns)
    }


def require_disabled_fallbacks(
    params: dict, *, managed_keys: frozenset[str], targets: frozenset[str], stage: str,
) -> None:
    """最適化を無効化したキーに scenario 由来のフォールバック値があるか検証する。"""
    missing = (managed_keys - targets) - params.keys()
    if missing:
        raise ValueError(
            f"{stage} で最適化を無効化した params の scenario 初期値がありません: "
            f"{sorted(missing)}"
        )
    fallback_keys = (managed_keys - targets) & params.keys()
    if fallback_keys:
        validate_parameters(
            params, frozenset(fallback_keys), source=f"{stage} の scenario 初期値",
        )


def read_phase_artifact(
    path: Path, *, expected_phase: int, required_keys: frozenset[str], producer: str,
) -> dict:
    """前段成果物 YAML を検証付きで読み込み、params dict を返す。"""
    if not path.is_file():
        raise FileNotFoundError(f"{producer} result not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("params"), dict):
        raise ValueError(f"invalid {producer} result: {path}")
    params = dict(data["params"])
    missing = required_keys - params.keys()
    invalid = [
        key
        for key in required_keys & params.keys()
        if not (
            isinstance(params[key], (int, float))
            and not isinstance(params[key], bool)
            and math.isfinite(float(params[key]))
        )
    ]
    metadata = data.get("metadata")
    if missing or invalid or not isinstance(metadata, dict) or metadata.get("phase") != expected_phase:
        raise ValueError(
            f"invalid {producer} result: missing={sorted(missing)}, invalid={sorted(invalid)}, "
            f"metadata.phase={metadata.get('phase') if isinstance(metadata, dict) else None}"
        )
    return params


def inherit_scenario_defaults(
    params: dict, scenario: Path, *, stage: str, target: str = TARGET_MODEL_NAME,
) -> dict:
    """scenario.yaml の fit 対象ケース params を setdefault で継承し、ケース params を返す。"""
    case_params = dict(load_model_config(scenario).find_case(target).params)
    for key, value in case_params.items():
        params.setdefault(key, value)
    print(f"[{stage}] scenario.yaml ({target}) からパラメータを引き継ぎました: {list(case_params)}")
    return case_params


def write_phase_artifact(out: Path, result: dict) -> dict:
    """ステージ成果物 YAML を書き出す。"""
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f"✓ パラメータ保存完了: {out}")
    return result
