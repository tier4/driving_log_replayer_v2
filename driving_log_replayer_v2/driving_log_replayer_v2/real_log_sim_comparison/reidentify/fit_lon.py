#!/usr/bin/env python3
"""縦方向モデルの直接同定。"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import fit_core
from ..lib._accel_source import ACCEL_DELAY_MAP
from .load_data import discover_cached_datasets
from .model_config import load_model_config
from .parameter_constraints import (
    FIT_LON,
    PARAMETER_CONSTRAINTS,
    stage_targets,
    validate_parameters,
)
from .settings import (
    ACCEL_SOURCE,
    LONG_DA_THRESH,
    LONG_VX_MIN,
    MIN_FIT_SAMPLES,
    RESAMPLE_DT,
    TARGET_MODEL_NAME,
)
from .stage_common import (
    clamped_medians,
    load_resampled,
    map_datasets,
    require_disabled_fallbacks,
    stage_metadata,
    write_phase_artifact,
)

# このステージが責任を持つキー集合。実行時に最適化を無効化しても不変なので import 時に確定する。
_LONG_KEYS = stage_targets(FIT_LON)
_ACC_TAU_FIT_BOUNDS = PARAMETER_CONSTRAINTS["acc_time_constant"].direct_fit_bounds
_ACC_DELAY_FIT_CANDIDATES = PARAMETER_CONSTRAINTS["acc_time_delay"].direct_fit_candidates
assert _ACC_TAU_FIT_BOUNDS is not None
assert _ACC_DELAY_FIT_CANDIDATES is not None


def _long_mask(ds: dict) -> np.ndarray:
    """縦方向同定の動的マスクを返す。"""
    d_cmd_acc = np.abs(np.gradient(ds["a_cmd"], RESAMPLE_DT))
    return ds["gear_drive"] & (ds["vx"] > LONG_VX_MIN) & (d_cmd_acc > LONG_DA_THRESH)


def _fit_one(ds: dict) -> tuple[float, float, float] | None:
    mask = _long_mask(ds)
    if mask.sum() < MIN_FIT_SAMPLES:
        return None
    fit = fit_core.fit_first_order_delay(
        ds["a_cmd"], ds["a_act"], mask, RESAMPLE_DT,
        tau_bounds=_ACC_TAU_FIT_BOUNDS, delay_candidates=_ACC_DELAY_FIT_CANDIDATES, fit_scale=True,
    )
    if fit is None:
        return None
    corrected_delay = max(0.0, fit["delay"] - ACCEL_DELAY_MAP.get(ACCEL_SOURCE, 0.0))
    return fit["tau"], corrected_delay, fit["scale"]


def _process_one(task: tuple[str, Path]) -> tuple[bool, tuple[float, float, float] | None]:
    """CSV load・resample・直接同定を同じ worker 内で完結させる。"""
    dataset = load_resampled(task, stage="fit_lon")
    if dataset is None:
        return False, None
    return True, _fit_one(dataset)



def fit_lon(
    collection_dir: Path, *, n_jobs: int = 1, initial_params: dict | None = None,
    scenario: Path | None = None,
) -> dict:
    """collection 配下の全 CSV キャッシュから縦方向モデルを直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    targets = stage_targets(FIT_LON) & _LONG_KEYS
    params = {
        key: value for key, value in (initial_params or {}).items() if key in _LONG_KEYS
    }
    require_disabled_fallbacks(
        params, managed_keys=_LONG_KEYS, targets=targets, stage="fit_lon",
    )
    validate_parameters(params, frozenset(params), source="fit_lon の scenario 初期値")
    if not targets:
        return {
            "params": params,
            "metadata": stage_metadata(
                collection_dir, n_datasets=len(tasks), n_valid=0, phase=1, optimized=(),
            ),
        }

    results = map_datasets(tasks, _process_one, stage="fit_lon", n_jobs=n_jobs)
    n_valid = sum(1 for loaded, _result in results if loaded)
    fit_results = [result for _loaded, result in results if result is not None]

    print(f"有効データセット数: {n_valid}")
    if n_valid == 0:
        raise RuntimeError("同定に使えるデータセットが 0 件です。")

    if not fit_results:
        raise RuntimeError(
            "縦方向の動的条件を満たすデータがありません "
            f"(各 dataset {MIN_FIT_SAMPLES} samples 以上が必要)。"
        )
    fitted_params = clamped_medians(
        fit_results, ("acc_time_constant", "acc_time_delay", "debug_acc_scaling_factor"),
    )
    params.update({key: value for key, value in fitted_params.items() if key in targets})
    print(f"  同定結果: acc_time_constant = {params['acc_time_constant']:.4f} s")
    print(f"            acc_time_delay    = {params['acc_time_delay']:.4f} s")

    # τ/delay 確定後、scaling はプラトー (N-step rollout の定常誤差) で決め直す。
    # 動的励起マスク上の同時推定値 (dynamic_scale) は τ/delay の同定精度のために
    # 残すが採用しない。同定コアは fit_plateau と共通の rollout 正式実装。
    plateau_meta: dict[str, float] = {}
    if "debug_acc_scaling_factor" in targets:
        if scenario is None:
            print("  [WARN] scenario 未指定のため plateau scaling 同定をスキップ (動的フィット値を使用)")
        else:
            from .fit_plateau import fit_scaling_channels  # noqa: PLC0415 (rollout 系の重い import を遅延)

            dynamic_scale = params["debug_acc_scaling_factor"]
            print("[fit_lon] プラトー scaling 同定 (rollout, τ/delay 固定)...")
            plateau = fit_scaling_channels(
                collection_dir, scenario,
                case_name=TARGET_MODEL_NAME,
                override_params=dict(params),
                channels=(("ax", "debug_acc_scaling_factor"),),
                n_jobs=n_jobs,
            )
            scale = plateau["fitted"]["debug_acc_scaling_factor"]
            params["debug_acc_scaling_factor"] = scale
            objective = plateau["objectives"]["ax"]
            plateau_meta = {
                "plateau_scale": scale,
                "dynamic_scale": dynamic_scale,
                "plateau_rmse_initial": objective["initial"],
                "plateau_rmse_final": objective["final"],
                "plateau_n_valid": plateau["metadata"]["n_valid"],
            }
            print(
                f"  同定結果: acc_scaling       = {scale:.4f} "
                f"(動的フィット値 {dynamic_scale:.4f} を置換, "
                f"plateau mean RMSE {objective['initial']:.4f}"
                f" -> {objective['final']:.4f} m/s^2)"
            )

    return {
        "params": params,
        "metadata": stage_metadata(
            collection_dir, n_datasets=len(tasks), n_valid=n_valid, phase=1,
            optimized=targets, **plateau_meta,
        ),
    }


def run(
    collection_dir: Path, out: Path, *, n_jobs: int = 1, scenario: Path | None = None,
) -> dict:
    initial_params = None
    if scenario is not None:
        initial_params = dict(load_model_config(scenario).find_case(TARGET_MODEL_NAME).params)
    result = fit_lon(
        collection_dir, n_jobs=n_jobs, initial_params=initial_params, scenario=scenario,
    )
    return write_phase_artifact(out, result)
