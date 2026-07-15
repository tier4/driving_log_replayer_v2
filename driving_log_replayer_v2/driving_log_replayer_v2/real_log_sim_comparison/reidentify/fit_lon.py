#!/usr/bin/env python3
"""縦方向モデルの直接同定。"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import sys

import numpy as np
import yaml

from . import fit_core
from ..lib._accel_source import ACCEL_DELAY_MAP
from ..lib._parallel import normalize_parallel_jobs
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .settings import (
    ACCEL_SOURCE,
    ACC_SCALE_BOUNDS,
    LONG_DA_THRESH,
    LONG_DELAY_GRID,
    LONG_RESULT_DELAY_BOUNDS,
    LONG_RESULT_TAU_BOUNDS,
    LONG_TAU_BOUNDS,
    LONG_VX_MIN,
    MIN_FIT_SAMPLES,
    RESAMPLE_DT,
)



def _long_mask(ds: dict) -> np.ndarray:
    """縦方向同定の動的マスクを返す。"""
    d_cmd_acc = np.abs(np.gradient(ds["a_cmd"], RESAMPLE_DT))
    return ds["gear_drive"] & (ds["vx"] > LONG_VX_MIN) & (d_cmd_acc > LONG_DA_THRESH)


def _load_one(task: tuple[str, Path]) -> dict | None:
    ds_id, csv_path = task
    dfs = read_dataset_csv(csv_path)
    return build_resampled(dfs, RESAMPLE_DT, context=f"fit_lon:{ds_id}")


def _fit_one(ds: dict) -> tuple[float, float, float] | None:
    mask = _long_mask(ds)
    if mask.sum() < MIN_FIT_SAMPLES:
        return None
    fit = fit_core.fit_first_order_delay(
        ds["a_cmd"], ds["a_act"], mask, RESAMPLE_DT,
        tau_bounds=LONG_TAU_BOUNDS, delay_candidates=LONG_DELAY_GRID, fit_scale=True,
    )
    if fit is None:
        return None
    corrected_delay = max(0.0, fit["delay"] - ACCEL_DELAY_MAP.get(ACCEL_SOURCE, 0.0))
    return fit["tau"], corrected_delay, fit["scale"]


def _process_one(task: tuple[str, Path]) -> tuple[bool, tuple[float, float, float] | None]:
    """CSV load・resample・直接同定を同じ worker 内で完結させる。"""
    dataset = _load_one(task)
    if dataset is None:
        return False, None
    return True, _fit_one(dataset)



def fit_lon(collection_dir: Path, *, n_jobs: int = 1) -> dict:
    """collection 配下の全 CSV キャッシュから縦方向モデルを直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    print(f"[fit_lon] データセット並列ロード ({len(tasks)} 件)...")

    n_valid = 0
    fit_results: list[tuple[float, float, float]] = []
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futures = {pool.submit(_process_one, task): task[0] for task in tasks}
        for future in as_completed(futures):
            try:
                loaded, result = future.result()
            except Exception as exc:  # noqa: BLE001 (dataset-local input failure)
                print(f"[fit_lon] SKIP {futures[future]}: {exc}", file=sys.stderr)
                continue
            if loaded:
                n_valid += 1
            if result is not None:
                fit_results.append(result)

    print(f"有効データセット数: {n_valid}")
    if n_valid == 0:
        raise RuntimeError("同定に使えるデータセットが 0 件です。")

    if not fit_results:
        raise RuntimeError(
            "縦方向の動的条件を満たすデータがありません "
            f"(各 dataset {MIN_FIT_SAMPLES} samples 以上が必要)。"
        )
    taus, delays, scales = map(np.asarray, zip(*fit_results))

    params = {
        "acc_time_constant": float(np.clip(np.median(taus), *LONG_RESULT_TAU_BOUNDS)),
        "acc_time_delay": float(np.clip(np.median(delays), *LONG_RESULT_DELAY_BOUNDS)),
        "debug_acc_scaling_factor": float(np.clip(np.median(scales), *ACC_SCALE_BOUNDS)),
    }
    print(f"  同定結果: acc_time_constant = {params['acc_time_constant']:.4f} s")
    print(f"            acc_time_delay    = {params['acc_time_delay']:.4f} s")
    print(f"            acc_scaling       = {params['debug_acc_scaling_factor']:.4f}")

    return {
        "params": params,
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": n_valid,
            "tuning_type": "physical_direct_fit",
            "phase": 1,
        },
    }


def run(collection_dir: Path, out: Path, *, n_jobs: int = 1) -> dict:
    result = fit_lon(collection_dir, n_jobs=n_jobs)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f"✓ パラメータ保存完了: {out}")
    return result
