#!/usr/bin/env python3
"""xy_heading_rate_coeff の直接同定。"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import minimize_scalar
import yaml

from ..lib._parallel import normalize_parallel_jobs
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .residuals import build_xy_columns, rmse, xy_residual
from .model_config import load_model_config
from .parameter_constraints import (
    FIT_XY,
    PARAMETER_CONSTRAINTS,
    stage_targets,
    validate_parameters,
)
from .settings import MIN_FIT_SAMPLES, RESAMPLE_DT, TARGET_MODEL_NAME

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
    if not targets:
        missing_fallback = _XY_KEYS - params.keys()
        if missing_fallback:
            raise ValueError(
                "fit_xy で最適化を無効化した params の scenario 初期値がありません: "
                f"{sorted(missing_fallback)}"
            )
        validate_parameters(params, _XY_KEYS, source="fit_xy の scenario 初期値")
        return {
            "params": params,
            "metadata": {
                "collection_dir": str(collection_dir), "n_datasets": len(tasks), "n_valid": 0,
                "tuning_type": "physical_direct_fit", "phase": 3,
                "optimized_parameters": [],
            },
        }

    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    print(f"[fit_xy] データセット並列ロード ({len(tasks)} 件)...")

    n_valid = 0
    datasets: list[dict] = []
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futures = {pool.submit(_load_one, task): task[0] for task in tasks}
        for future in as_completed(futures):
            try:
                dataset = future.result()
            except Exception as exc:  # noqa: BLE001 (dataset-local input failure)
                print(f"[fit_xy] SKIP {futures[future]}: {exc}", file=sys.stderr)
                continue
            if dataset is not None:
                n_valid += 1
                datasets.append(dataset)

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
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": n_valid,
            "tuning_type": "physical_direct_fit",
            "phase": 3,
            "optimized_parameters": sorted(targets),
            "fit_rmse": fit["rmse"],
            "fit_n": fit["n"],
        },
    }


def run(
    collection_dir: Path,
    out: Path,
    *,
    phase2_params_path: Path,
    scenario: Path,
    n_jobs: int = 1,
) -> dict:
    if not phase2_params_path.is_file():
        raise FileNotFoundError(f"fit_steer result not found: {phase2_params_path}")
    phase2_data = yaml.safe_load(phase2_params_path.read_text(encoding="utf-8"))
    if not isinstance(phase2_data, dict) or not isinstance(phase2_data.get("params"), dict):
        raise ValueError(f"invalid fit_steer result: {phase2_params_path}")
    phase2_params = dict(phase2_data["params"])
    missing = _PHASE2_KEYS - phase2_params.keys()
    invalid = [
        key
        for key in _PHASE2_KEYS & phase2_params.keys()
        if not (
            isinstance(phase2_params[key], (int, float))
            and not isinstance(phase2_params[key], bool)
            and math.isfinite(float(phase2_params[key]))
        )
    ]
    metadata = phase2_data.get("metadata")
    if missing or invalid or not isinstance(metadata, dict) or metadata.get("phase") != 2:
        raise ValueError(
            f"invalid fit_steer result: missing={sorted(missing)}, invalid={sorted(invalid)}, "
            f"metadata.phase={metadata.get('phase') if isinstance(metadata, dict) else None}"
        )
    print(f"[fit_xy] fit_steer のパラメータを引き継ぎました: {list(phase2_params)}")

    case_params = dict(load_model_config(scenario).find_case(TARGET_MODEL_NAME).params)
    for key, value in case_params.items():
        phase2_params.setdefault(key, value)
    print(f"[fit_xy] scenario.yaml からパラメータを引き継ぎました: {list(case_params)}")

    result = fit_xy(collection_dir, phase2_params=phase2_params, n_jobs=n_jobs)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f"✓ パラメータ保存完了: {out}")
    return result
