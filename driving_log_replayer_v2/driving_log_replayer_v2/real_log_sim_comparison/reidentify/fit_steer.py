#!/usr/bin/env python3
"""操舵モデルと ``k_us`` の直接同定。"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from . import fit_core
from ..lib._parallel import normalize_parallel_jobs
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .model_config import load_model_config
from .parameter_constraints import FIT_STEER, PARAMETER_CONSTRAINTS, stage_targets
from .physical_constants import DWZ_MAX, VX_MIN_CURVE, WZ_MIN
from .settings import (
    MIN_FIT_SAMPLES,
    MIN_K_US_SAMPLES,
    RESAMPLE_DT,
    STEER_BIAS_MIN_SAMPLES,
    STEER_CLIP_RAD,
    STEER_CMD_ENERGY_MIN,
    STEER_DSTEER_MIN,
    STEER_STRAIGHT_VX_MIN,
    STEER_STRAIGHT_WZ_MAX,
    STEER_VX_MIN,
    TARGET_MODEL_NAME,
)

_PHASE1_KEYS = frozenset(
    {"acc_time_constant", "acc_time_delay", "debug_acc_scaling_factor"}
)
_STEER_RESPONSE_KEYS = frozenset(
    {"steer_time_constant", "steer_time_delay", "steer_bias", "debug_steer_scaling_factor"}
)
_STEER_TAU_FIT_BOUNDS = PARAMETER_CONSTRAINTS["steer_time_constant"].direct_fit_bounds
_STEER_DELAY_FIT_CANDIDATES = PARAMETER_CONSTRAINTS["steer_time_delay"].direct_fit_candidates
assert _STEER_TAU_FIT_BOUNDS is not None
assert _STEER_DELAY_FIT_CANDIDATES is not None


def _load_one(task: tuple[str, Path]) -> dict | None:
    ds_id, csv_path = task
    dfs = read_dataset_csv(csv_path)
    return build_resampled(dfs, RESAMPLE_DT, context=f"fit_steer:{ds_id}")


def _fit_one_steer(ds: dict) -> tuple[float, float, float, float] | None:
    d_cmd = ds["d_cmd"]
    d_act = ds["d_act"]
    vx = ds["vx"]
    wz = ds["wz"]
    gear_drive = ds["gear_drive"]
    d_cmd_steer = np.abs(np.gradient(d_cmd, RESAMPLE_DT))
    mask_dyn = gear_drive & (vx > STEER_VX_MIN) & (d_cmd_steer > STEER_DSTEER_MIN / RESAMPLE_DT)
    if mask_dyn.sum() < MIN_FIT_SAMPLES:
        return None

    mask_straight = gear_drive & (vx > STEER_STRAIGHT_VX_MIN) & (np.abs(wz) < STEER_STRAIGHT_WZ_MAX)
    if mask_straight.sum() < STEER_BIAS_MIN_SAMPLES:
        return None
    bias = float(np.mean(d_act[mask_straight]))

    d_act_nobias = d_act - bias
    sum_cmd2 = np.sum(d_cmd[mask_dyn] ** 2)
    if sum_cmd2 <= STEER_CMD_ENERGY_MIN:
        return None
    dsf = float(np.sum(d_cmd[mask_dyn] * d_act_nobias[mask_dyn]) / sum_cmd2)
    dsf = PARAMETER_CONSTRAINTS["debug_steer_scaling_factor"].clamp(dsf)

    fit = fit_core.fit_first_order_delay(
        dsf * d_cmd, d_act_nobias, mask_dyn, RESAMPLE_DT,
        tau_bounds=_STEER_TAU_FIT_BOUNDS, delay_candidates=_STEER_DELAY_FIT_CANDIDATES,
    )
    if fit is None:
        return None
    return fit["tau"], fit["delay"], bias, dsf


def _steady_turn_samples(ds: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """k_us 回帰に必要な定常旋回サンプルだけを抽出する。"""
    wz = ds["wz"]
    dwz_mid = np.diff(wz) / RESAMPLE_DT
    dwz = np.empty_like(wz)
    dwz[0] = dwz_mid[0] if len(dwz_mid) > 0 else 0.0
    dwz[-1] = dwz_mid[-1] if len(dwz_mid) > 0 else 0.0
    dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])
    mask = (
        ds["gear_drive"]
        & (np.abs(wz) > WZ_MIN)
        & (np.abs(dwz) < DWZ_MAX)
        & (ds["vx"] > VX_MIN_CURVE)
    )
    return ds["vx"][mask], wz[mask], ds["d_act"][mask]


def _process_one(
    task: tuple[str, Path],
) -> tuple[tuple[float, float, float, float] | None, tuple[np.ndarray, np.ndarray, np.ndarray]] | None:
    """CSV load・resample・直接同定・定常旋回抽出を同じ worker で行う。"""
    dataset = _load_one(task)
    if dataset is None:
        return None
    return _fit_one_steer(dataset), _steady_turn_samples(dataset)


def fit_steer(
    collection_dir: Path,
    *,
    phase1_params: dict,
    wheelbase: float,
    n_jobs: int = 1,
) -> dict:
    """collection 配下の全 CSV キャッシュから操舵モデル + k_us を直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    targets = stage_targets(FIT_STEER)
    params = dict(phase1_params)
    if not targets:
        return {
            "params": params,
            "metadata": {
                "collection_dir": str(collection_dir), "n_datasets": len(tasks), "n_valid": 0,
                "wheelbase": wheelbase, "tuning_type": "physical_direct_fit", "phase": 2,
                "optimized_parameters": [],
            },
        }
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    print(f"[fit_steer] データセット並列ロード ({len(tasks)} 件)...")

    n_valid = 0
    fit_results: list[tuple[float, float, float, float]] = []
    steady_samples: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futures = {pool.submit(_process_one, task): task[0] for task in tasks}
        for future in as_completed(futures):
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001 (dataset-local input failure)
                print(f"[fit_steer] SKIP {futures[future]}: {exc}", file=sys.stderr)
                continue
            if result is None:
                continue
            fit_result, samples = result
            n_valid += 1
            steady_samples.append(samples)
            if fit_result is not None:
                fit_results.append(fit_result)

    print(f"有効データセット数: {n_valid}")
    if n_valid == 0:
        raise RuntimeError("同定に使えるデータセットが 0 件です。")

    response_targets = targets & _STEER_RESPONSE_KEYS
    if response_targets and not fit_results:
        raise RuntimeError(
            "操舵の動的条件を満たすデータがありません "
            f"(各 dataset {MIN_FIT_SAMPLES} samples 以上が必要)。"
        )
    if response_targets:
        taus, delays, biases, dsfs = map(np.asarray, zip(*fit_results))
        fitted_params = {
            "steer_time_constant": PARAMETER_CONSTRAINTS["steer_time_constant"].clamp(float(np.median(taus))),
            "steer_time_delay": PARAMETER_CONSTRAINTS["steer_time_delay"].clamp(float(np.median(delays))),
            "steer_bias": PARAMETER_CONSTRAINTS["steer_bias"].clamp(float(np.median(biases))),
            "debug_steer_scaling_factor": PARAMETER_CONSTRAINTS["debug_steer_scaling_factor"].clamp(float(np.median(dsfs))),
        }
        params.update({key: value for key, value in fitted_params.items() if key in targets})
    params.setdefault("steer_dead_band", PARAMETER_CONSTRAINTS["steer_dead_band"].default)
    params.setdefault("steer_rate_lim", PARAMETER_CONSTRAINTS["steer_rate_lim"].default)
    if response_targets:
        print(f"  同定結果: steer_time_constant = {params['steer_time_constant']:.4f} s")
        print(f"            steer_time_delay    = {params['steer_time_delay']:.4f} s")
        print(f"            steer_bias          = {params['steer_bias']:.6f} rad")
        print(f"            steer_scaling       = {params['debug_steer_scaling_factor']:.4f}")

    if "k_us" in targets:
        print("[fit_steer] k_us 直接同定を実行中...")
        vx_f = np.concatenate([samples[0] for samples in steady_samples])
        wz_f = np.concatenate([samples[1] for samples in steady_samples])
        steer_f = np.concatenate([samples[2] for samples in steady_samples]) - params["steer_bias"]
        tan_steer = np.tan(np.clip(steer_f, -STEER_CLIP_RAD, STEER_CLIP_RAD))
        x = vx_f * wz_f
        y = tan_steer - wheelbase * wz_f / vx_f if len(vx_f) else np.empty(0)
        sum_x2 = float(np.sum(x * x))
        sum_xy = float(np.sum(x * y))
        n_pts = int(len(vx_f))
        if n_pts < MIN_K_US_SAMPLES or sum_x2 <= 0:
            raise RuntimeError(f"k_us 回帰に十分な定常旋回サンプルがありません (n={n_pts})。")
        k_us = PARAMETER_CONSTRAINTS["k_us"].clamp(sum_xy / sum_x2)
        params["k_us"] = k_us
        print(f"  同定結果: k_us = {k_us:.5f} (曲線走行サンプル n={n_pts})")

    return {
        "params": params,
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": n_valid,
            "wheelbase": wheelbase,
            "tuning_type": "physical_direct_fit",
            "phase": 2,
            "optimized_parameters": sorted(targets),
        },
    }


def run(
    collection_dir: Path,
    out: Path,
    *,
    phase1_params_path: Path,
    scenario: Path,
    n_jobs: int = 1,
) -> dict:
    if not phase1_params_path.is_file():
        raise FileNotFoundError(f"fit_lon result not found: {phase1_params_path}")
    phase1_data = yaml.safe_load(phase1_params_path.read_text(encoding="utf-8"))
    if not isinstance(phase1_data, dict) or not isinstance(phase1_data.get("params"), dict):
        raise ValueError(f"invalid fit_lon result: {phase1_params_path}")
    phase1_params = dict(phase1_data["params"])
    missing = _PHASE1_KEYS - phase1_params.keys()
    invalid = [
        key
        for key in _PHASE1_KEYS & phase1_params.keys()
        if not (
            isinstance(phase1_params[key], (int, float))
            and not isinstance(phase1_params[key], bool)
            and math.isfinite(float(phase1_params[key]))
        )
    ]
    metadata = phase1_data.get("metadata")
    if missing or invalid or not isinstance(metadata, dict) or metadata.get("phase") != 1:
        raise ValueError(
            f"invalid fit_lon result: missing={sorted(missing)}, invalid={sorted(invalid)}, "
            f"metadata.phase={metadata.get('phase') if isinstance(metadata, dict) else None}"
        )
    print(f"[fit_steer] fit_lon のパラメータを引き継ぎました: {list(phase1_params)}")

    case_params = dict(load_model_config(scenario).find_case(TARGET_MODEL_NAME).params)
    for key, value in case_params.items():
        phase1_params.setdefault(key, value)
    wheelbase_value = case_params.get("wheelbase", case_params.get("wheel_base"))
    if wheelbase_value is None:
        raise ValueError(f"{scenario}: models.{TARGET_MODEL_NAME}.params.wheelbase が必要です")
    wheelbase = float(wheelbase_value)
    print(f"[fit_steer] scenario.yaml からパラメータを引き継ぎました: {list(case_params)}")
    print(f"[fit_steer] wheelbase = {wheelbase}")

    result = fit_steer(collection_dir, phase1_params=phase1_params, wheelbase=wheelbase, n_jobs=n_jobs)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f"✓ パラメータ保存完了: {out}")
    return result
