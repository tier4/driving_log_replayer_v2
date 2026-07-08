#!/usr/bin/env python3
"""[Step2] 縦方向モデルの直接同定 → phase1_acc.yaml。"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import yaml

from . import fit_core
from ..lib._parallel import default_parallel_jobs, normalize_parallel_jobs
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .settings import (
    ACC_SCALE_BOUNDS,
    LONG_DA_THRESH,
    LONG_DELAY_GRID,
    LONG_RESULT_DELAY_BOUNDS,
    LONG_RESULT_TAU_BOUNDS,
    LONG_TAU_BOUNDS,
    LONG_VX_MIN,
    MIN_FIT_SAMPLES,
    RESAMPLE_DT,
    ACCEL_SOURCE,
    ACCEL_DELAY_MAP,
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



def fit_lon(collection_dir: Path, *, n_jobs: int = 1) -> dict:
    """collection 配下の全 CSV キャッシュから縦方向モデルを直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    print(f"[fit_lon] データセット並列ロード ({len(tasks)} 件)...")

    datasets: list[dict] = []
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futs = [pool.submit(_load_one, t) for t in tasks]
        for fut in as_completed(futs):
            r = fut.result()
            if r is not None:
                datasets.append(r)

    print(f"有効データセット数: {len(datasets)}")
    if not datasets:
        raise RuntimeError("有効なデータセットが 0 件です (先に extract.py を実行してください)。")

    taus: list[float] = []
    delays: list[float] = []
    scales: list[float] = []
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(datasets))
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futs = [pool.submit(_fit_one, ds) for ds in datasets]
        for fut in as_completed(futs):
            res = fut.result()
            if res is not None:
                tau, delay, scale = res
                taus.append(tau)
                delays.append(delay)
                scales.append(scale)

    if not taus:
        raise RuntimeError(
            "縦方向の動的条件を満たすデータがありません "
            f"(各 dataset {MIN_FIT_SAMPLES} samples 以上が必要)。"
        )

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
        "score": 0.0,
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": len(datasets),
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


def main() -> None:
    ap = argparse.ArgumentParser(description="縦方向モデルの直接同定 (Step2)")
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n-jobs", type=int, default=default_parallel_jobs())
    args = ap.parse_args()
    run(args.collection_dir, args.out, n_jobs=normalize_parallel_jobs(args.n_jobs))


if __name__ == "__main__":
    main()
