#!/usr/bin/env python3
"""[Step2] 縦方向モデルの直接同定 → phase1_acc.yaml (ROS フリー)。

physical_tuning.py の phase1 ロジック(`_long_mask`/`_fit_one_long_worker`)を、
データ読込元を CSV キャッシュ (load_data.py) に差し替えて移植したもの。
出力 YAML スキーマは旧 `phase1_acc.yaml` と同一に保つ (fit_merge.py・旧
パイプラインとの互換性のため)。

旧 physical_tuning.py にあった --skip-lon (scenario.yaml の値をそのまま
引き継ぐ) は、新パイプラインでは常に直接同定する方針としてここでは持たない
(必要な場合は旧パイプラインを使う)。
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import os
from pathlib import Path

import numpy as np
import yaml

from . import fit_core
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv

DT = 0.01  # 10ms サンプリング (遅延グリッド分解能を上げるため)
VX_MIN = 0.5
DA_THRESH = 0.15
TAU_BOUNDS_LONG = (0.01, 5.0)
DELAY_GRID_LONG = np.arange(0.0, 0.31, 0.01)


def _long_mask(ds: dict) -> np.ndarray:
    """縦方向同定の動的マスク: DRIVE 中 & vx>VX_MIN & 指令加速度変化が大きい区間。"""
    d_cmd_acc = np.abs(np.gradient(ds["a_cmd"], DT))
    return ds["gear_drive"] & (ds["vx"] > VX_MIN) & (d_cmd_acc > DA_THRESH)


def _load_one(task: tuple[str, Path]) -> dict | None:
    ds_id, csv_path = task
    dfs = read_dataset_csv(csv_path)
    return build_resampled(dfs, DT, context=f"fit_lon:{ds_id}")


def _fit_one(ds: dict) -> tuple[float, float, float] | None:
    mask = _long_mask(ds)
    if mask.sum() < 50:
        return None
    fit = fit_core.fit_first_order_delay(
        ds["a_cmd"], ds["a_act"], mask, DT,
        tau_bounds=TAU_BOUNDS_LONG, delay_candidates=DELAY_GRID_LONG, fit_scale=True,
    )
    if fit is None:
        return None
    return fit["tau"], fit["delay"], fit["scale"]


def fit_lon(collection_dir: Path, *, n_jobs: int = 1) -> dict:
    """collection 配下の全 CSV キャッシュから縦方向モデルを直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    print(f"[fit_lon] データセット並列ロード ({len(tasks)} 件)...")

    datasets: list[dict] = []
    with ProcessPoolExecutor(max_workers=n_jobs) as pool:
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
    with ProcessPoolExecutor(max_workers=n_jobs) as pool:
        futs = [pool.submit(_fit_one, ds) for ds in datasets]
        for fut in as_completed(futs):
            res = fut.result()
            if res is not None:
                tau, delay, scale = res
                taus.append(tau)
                delays.append(delay)
                scales.append(scale)

    if taus:
        params = {
            "acc_time_constant": float(np.clip(np.median(taus), 0.1, 3.0)),
            "acc_time_delay": float(np.clip(np.median(delays), 0.0, 0.3)),
            "debug_acc_scaling_factor": float(np.clip(np.median(scales), 0.8, 1.2)),
        }
        print(f"  同定結果: acc_time_constant = {params['acc_time_constant']:.4f} s")
        print(f"            acc_time_delay    = {params['acc_time_delay']:.4f} s")
        print(f"            acc_scaling       = {params['debug_acc_scaling_factor']:.4f}")
    else:
        print("[WARN] 縦方向の動的条件を満たすデータがないため、デフォルト値を設定します。")
        params = {"acc_time_constant": 0.2, "acc_time_delay": 0.1, "debug_acc_scaling_factor": 1.0}

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
    ap.add_argument("--n-jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()
    run(args.collection_dir, args.out, n_jobs=args.n_jobs)


if __name__ == "__main__":
    main()
