#!/usr/bin/env python3
"""xy_heading_rate_coeff の直接同定。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.optimize import minimize_scalar

from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .parameter_constraints import FIT_XY, PARAMETER_CONSTRAINTS, stage_targets
from .residuals import build_xy_columns, rmse, xy_residual
from .settings import MIN_FIT_SAMPLES, RESAMPLE_DT
from .stage_common import (
    inherit_scenario_defaults,
    map_datasets,
    read_phase_artifact,
    require_disabled_fallbacks,
    stage_metadata,
    write_phase_artifact,
)

_XY_KEYS = frozenset({"xy_heading_rate_coeff"})
_XY_HEADING_RATE_COEFF_BOUNDS = (-5.0, 5.0)
_PHASE2_KEYS = frozenset(
    {
        "acc_time_constant",
        "acc_time_delay",
        "debug_acc_scaling_factor",
        "steer_time_constant",
        "steer_time_delay",
        "debug_steer_scaling_factor",
        "steer_bias",
        "k_us",
    }
)


def _fit_xy_heading_rate_coeff(datasets: list[dict], initial: float = 0.0) -> dict:
    """全 dataset をプールした残差二乗平均を最小化する係数を直接同定する。"""
    def objective(coeff: float) -> float:
        parts = [np.concatenate(xy_residual(ds, coeff)) for ds in datasets]
        values = np.concatenate([part for part in parts if len(part)]) if any(len(p) for p in parts) else np.empty(0)
        return float(np.mean(values * values)) if len(values) else float("inf")
    result = minimize_scalar(objective, bounds=_XY_HEADING_RATE_COEFF_BOUNDS, method="bounded")
    coeff = float(result.x) if result.success else float(initial)
    parts = [np.concatenate(xy_residual(ds, coeff)) for ds in datasets]
    values = np.concatenate([part for part in parts if len(part)]) if any(len(p) for p in parts) else np.empty(0)
    return {"xy_heading_rate_coeff": coeff, "rmse": rmse(values), "n": int(len(values))}


def _load_one(task: tuple[str, Path]) -> dict | None:
    """CSV load・resample・xy 列付与を同じ worker 内で完結させ、フィットに使う列だけを返す。"""
    ds_id, csv_path = task
    dfs = read_dataset_csv(csv_path)
    dataset = build_resampled(dfs, RESAMPLE_DT, context=f"fit_xy:{ds_id}")
    if dataset is None:
        return None
    build_xy_columns(dataset, dfs, dt=RESAMPLE_DT)
    return {"xy": dataset["xy"], "gear_drive": dataset["gear_drive"]}


def fit_xy(
    collection_dir: Path, *, n_jobs: int = 1, phase2_params: dict,
) -> dict:
    """collection 配下の全 CSV キャッシュから xy_heading_rate_coeff を直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    targets = stage_targets(FIT_XY) & _XY_KEYS
    params = dict(phase2_params)
    require_disabled_fallbacks(
        params, managed_keys=_XY_KEYS, targets=targets, stage="fit_xy",
    )
    if not targets:
        return {
            "params": params,
            "metadata": stage_metadata(
                collection_dir, n_datasets=len(tasks), n_valid=0, phase=3, optimized=(),
            ),
        }

    datasets = [
        dataset
        for dataset in map_datasets(tasks, _load_one, stage="fit_xy", n_jobs=n_jobs)
        if dataset is not None
    ]
    n_valid = len(datasets)

    print(f"有効データセット数: {n_valid}")
    if n_valid == 0:
        raise RuntimeError("同定に使えるデータセットが 0 件です。")

    fit = _fit_xy_heading_rate_coeff(datasets)
    if fit["n"] < MIN_FIT_SAMPLES:
        raise RuntimeError(
            f"xy_heading_rate_coeff 直接同定に十分なサンプルがありません (n={fit['n']})。"
        )

    coeff = PARAMETER_CONSTRAINTS["xy_heading_rate_coeff"].clamp(fit["xy_heading_rate_coeff"])
    params["xy_heading_rate_coeff"] = coeff
    print(f"  同定結果: xy_heading_rate_coeff = {coeff:.6f} (rmse={fit['rmse']:.4f}, n={fit['n']})")

    return {
        "params": params,
        "metadata": stage_metadata(
            collection_dir, n_datasets=len(tasks), n_valid=n_valid, phase=3,
            optimized=targets, fit_rmse=fit["rmse"], fit_n=fit["n"],
        ),
    }


def run(
    collection_dir: Path,
    out: Path,
    *,
    phase2_params_path: Path,
    scenario: Path,
    n_jobs: int = 1,
) -> dict:
    phase2_params = read_phase_artifact(
        phase2_params_path, expected_phase=2, required_keys=_PHASE2_KEYS, producer="fit_steer",
    )
    print(f"[fit_xy] fit_steer のパラメータを引き継ぎました: {list(phase2_params)}")

    inherit_scenario_defaults(phase2_params, scenario, stage="fit_xy")

    result = fit_xy(collection_dir, phase2_params=phase2_params, n_jobs=n_jobs)
    return write_phase_artifact(out, result)
