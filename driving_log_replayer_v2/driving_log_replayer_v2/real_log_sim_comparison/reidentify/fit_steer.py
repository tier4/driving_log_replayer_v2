#!/usr/bin/env python3
"""[Step3] 操舵 + k_us の直接同定 → phase2_steer.yaml。"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import os
from pathlib import Path

import numpy as np
import yaml

from . import fit_core
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .physical_constants import DWZ_MAX, VX_MIN_CURVE, WZ_MIN
from .scenario_params import resolve_wheelbase
from .settings import (
    DEFAULT_WHEELBASE,
    K_US_CLIP,
    MIN_FIT_SAMPLES,
    MIN_K_US_SAMPLES,
    RESAMPLE_DT,
    STEER_BIAS_MIN_SAMPLES,
    STEER_CLIP_RAD,
    STEER_CMD_ENERGY_MIN,
    STEER_DEFAULT_DEAD_BAND,
    STEER_DEFAULT_RATE_LIM,
    STEER_DELAY_GRID,
    STEER_DSTEER_MIN,
    STEER_RESULT_DELAY_BOUNDS,
    STEER_RESULT_TAU_BOUNDS,
    STEER_SCALE_BOUNDS,
    STEER_STRAIGHT_VX_MIN,
    STEER_STRAIGHT_WZ_MAX,
    STEER_TAU_BOUNDS,
    STEER_VX_MIN,
)


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
    dsf = float(np.clip(dsf, *STEER_SCALE_BOUNDS))

    fit = fit_core.fit_first_order_delay(
        dsf * d_cmd, d_act_nobias, mask_dyn, RESAMPLE_DT,
        tau_bounds=STEER_TAU_BOUNDS, delay_candidates=STEER_DELAY_GRID,
    )
    if fit is None:
        return None
    return fit["tau"], fit["delay"], bias, dsf


def fit_steer(
    collection_dir: Path,
    *,
    phase1_params: dict | None = None,
    wheelbase: float = DEFAULT_WHEELBASE,
    n_jobs: int = 1,
) -> dict:
    """collection 配下の全 CSV キャッシュから操舵モデル + k_us を直接同定する。"""
    tasks = discover_cached_datasets(collection_dir)
    print(f"[fit_steer] データセット並列ロード ({len(tasks)} 件)...")

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

    params = dict(phase1_params or {})

    taus: list[float] = []
    delays: list[float] = []
    biases: list[float] = []
    dsfs: list[float] = []
    with ProcessPoolExecutor(max_workers=n_jobs) as pool:
        futs = [pool.submit(_fit_one_steer, ds) for ds in datasets]
        for fut in as_completed(futs):
            res = fut.result()
            if res is not None:
                tau, delay, bias, dsf = res
                taus.append(tau)
                delays.append(delay)
                biases.append(bias)
                dsfs.append(dsf)

    if not taus:
        raise RuntimeError(
            "操舵の動的条件を満たすデータがありません "
            f"(各 dataset {MIN_FIT_SAMPLES} samples 以上が必要)。"
        )

    params["steer_time_constant"] = float(np.clip(np.median(taus), *STEER_RESULT_TAU_BOUNDS))
    params["steer_time_delay"] = float(np.clip(np.median(delays), *STEER_RESULT_DELAY_BOUNDS))
    params["steer_bias"] = float(np.median(biases))
    params["debug_steer_scaling_factor"] = float(np.median(dsfs))
    params.setdefault("steer_dead_band", STEER_DEFAULT_DEAD_BAND)
    params.setdefault("steer_rate_lim", STEER_DEFAULT_RATE_LIM)
    print(f"  同定結果: steer_time_constant = {params['steer_time_constant']:.4f} s")
    print(f"            steer_time_delay    = {params['steer_time_delay']:.4f} s")
    print(f"            steer_bias          = {params['steer_bias']:.6f} rad")
    print(f"            steer_scaling       = {params['debug_steer_scaling_factor']:.4f}")

    print("[fit_steer] k_us 直接同定を実行中...")
    all_vx, all_wz, all_steer, all_dwz = [], [], [], []
    for ds in datasets:
        wz = ds["wz"]
        dwz_mid = np.diff(wz) / RESAMPLE_DT
        dwz = np.empty_like(wz)
        dwz[0] = dwz_mid[0] if len(dwz_mid) > 0 else 0.0
        dwz[-1] = dwz_mid[-1] if len(dwz_mid) > 0 else 0.0
        dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])
        all_vx.append(ds["vx"])
        all_wz.append(wz)
        all_steer.append(ds["d_act"])
        all_dwz.append(dwz)

    vx_all = np.concatenate(all_vx)
    wz_all = np.concatenate(all_wz)
    steer_all = np.concatenate(all_steer)
    dwz_all = np.concatenate(all_dwz)
    gear_all = np.concatenate([ds["gear_drive"] for ds in datasets])

    mask_ok = gear_all & (np.abs(wz_all) > WZ_MIN) & (np.abs(dwz_all) < DWZ_MAX) & (vx_all > VX_MIN_CURVE)
    vx_f = vx_all[mask_ok]
    wz_f = wz_all[mask_ok]
    steer_f = steer_all[mask_ok] - params["steer_bias"]
    tan_steer = np.tan(np.clip(steer_f, -STEER_CLIP_RAD, STEER_CLIP_RAD))

    x = vx_f * wz_f
    y = tan_steer - wheelbase * wz_f / vx_f if len(vx_f) else np.empty(0)
    sum_x2 = float(np.sum(x * x))
    sum_xy = float(np.sum(x * y))
    n_pts = int(len(vx_f))
    if n_pts < MIN_K_US_SAMPLES or sum_x2 <= 0:
        raise RuntimeError(f"k_us 回帰に十分な定常旋回サンプルがありません (n={n_pts})。")
    k_us = sum_xy / sum_x2
    k_us = float(np.clip(k_us, 0.0, K_US_CLIP))
    params["k_us"] = k_us
    print(f"  同定結果: k_us = {k_us:.5f} (曲線走行サンプル n={n_pts})")

    return {
        "params": params,
        "score": 0.0,
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": len(datasets),
            "wheelbase": wheelbase,
            "tuning_type": "physical_direct_fit",
            "phase": 2,
        },
    }


def run(
    collection_dir: Path,
    out: Path,
    *,
    phase1_params_path: Path | None = None,
    scenario: Path | None = None,
    case: str = "current",
    n_jobs: int = 1,
) -> dict:
    phase1_params: dict = {}
    if phase1_params_path is not None and phase1_params_path.exists():
        with phase1_params_path.open("r") as f:
            phase1_data = yaml.safe_load(f)
        phase1_params = dict(phase1_data.get("params", phase1_data))
        print(f"[fit_steer] Step2 (fit_lon) のパラメータを引き継ぎました: {list(phase1_params.keys())}")

    if scenario is not None and scenario.exists():
        try:
            from ..lib._models_config import load_models_doc
            cfg = load_models_doc(scenario)
            case_params = dict(cfg.find_case(case).params)
            for k, v in case_params.items():
                phase1_params.setdefault(k, v)
            print(f"[fit_steer] scenario.yaml の '{case}' からパラメータを引き継ぎました: {list(case_params.keys())}")
        except Exception as e:
            print(f"[fit_steer] Warning: {case} のパラメータを scenario.yaml からロードできませんでした: {e}")

    wheelbase = resolve_wheelbase(scenario, case)
    print(f"[fit_steer] wheelbase = {wheelbase}")

    result = fit_steer(collection_dir, phase1_params=phase1_params, wheelbase=wheelbase, n_jobs=n_jobs)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f"✓ パラメータ保存完了: {out}")
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description="操舵+k_us の直接同定 (Step3)")
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--phase1-params", type=Path, default=None, help="Step2 (fit_lon) の出力 YAML")
    ap.add_argument("--scenario", type=Path, default=None, help="wheelbase 解決用の scenario.yaml (任意)")
    ap.add_argument("--case", type=str, default="current")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n-jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()
    run(
        args.collection_dir, args.out,
        phase1_params_path=args.phase1_params, scenario=args.scenario, case=args.case,
        n_jobs=args.n_jobs,
    )


if __name__ == "__main__":
    main()
