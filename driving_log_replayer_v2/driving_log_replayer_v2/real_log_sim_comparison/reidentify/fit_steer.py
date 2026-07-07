#!/usr/bin/env python3
"""[Step3] 操舵 + k_us (アンダーステア勾配) の直接同定 → phase2_steer.yaml (ROS フリー)。

physical_tuning.py の phase2 ロジック(`_fit_one_steer_worker` + k_us 速度ビン別
回帰)を、データ読込元を CSV キャッシュ (load_data.py) に差し替えて移植したもの。
出力 YAML には Step2 (fit_lon) の params を継承した上で操舵系パラメータを追加する
(旧 `phase2_steer.yaml` と同じ「縦+横 全パラメータ入り」のスキーマ)。
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
from .physical_constants import DWZ_MAX, VX_MIN_CURVE, WZ_MIN
from .scenario_params import resolve_wheelbase

DT = 0.01
VX_MIN = 0.5
DSTEER_MIN = 0.001
TAU_BOUNDS_STEER = (0.01, 2.0)
DELAY_GRID_STEER = np.arange(0.0, 0.16, 0.01)
# per-bin OLS 推定値のクリップ上限 (physical_tuning.py 固有値。lib._physical_validity
# の K_US_CLIP=0.5 は per-sample IQR 表示用で役割が異なるため意図的に別値)。
K_US_CLIP = 0.05


def _load_one(task: tuple[str, Path]) -> dict | None:
    ds_id, csv_path = task
    dfs = read_dataset_csv(csv_path)
    return build_resampled(dfs, DT, context=f"fit_steer:{ds_id}")


def _fit_one_steer(ds: dict) -> tuple[float, float, float, float] | None:
    d_cmd = ds["d_cmd"]
    d_act = ds["d_act"]
    vx = ds["vx"]
    wz = ds["wz"]
    gear_drive = ds["gear_drive"]
    d_cmd_steer = np.abs(np.gradient(d_cmd, DT))
    mask_dyn = gear_drive & (vx > VX_MIN) & (d_cmd_steer > DSTEER_MIN / DT)
    if mask_dyn.sum() < 50:
        return None

    mask_straight = gear_drive & (vx > 3.0) & (np.abs(wz) < 0.005)
    bias = float(np.mean(d_act[mask_straight])) if mask_straight.sum() > 20 else 0.0005

    d_act_nobias = d_act - bias
    sum_cmd2 = np.sum(d_cmd[mask_dyn] ** 2)
    dsf = float(np.sum(d_cmd[mask_dyn] * d_act_nobias[mask_dyn]) / sum_cmd2) if sum_cmd2 > 1e-5 else 1.0
    dsf = float(np.clip(dsf, 0.8, 1.2))

    fit = fit_core.fit_first_order_delay(
        dsf * d_cmd, d_act_nobias, mask_dyn, DT,
        tau_bounds=TAU_BOUNDS_STEER, delay_candidates=DELAY_GRID_STEER,
    )
    if fit is None:
        return None
    return fit["tau"], fit["delay"], bias, dsf


def fit_steer(
    collection_dir: Path,
    *,
    phase1_params: dict | None = None,
    wheelbase: float = 4.76012,
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

    if taus:
        params["steer_time_constant"] = float(np.clip(np.median(taus), 0.05, 0.8))
        params["steer_time_delay"] = float(np.clip(np.median(delays), 0.0, 0.15))
        params["steer_bias"] = float(np.median(biases))
        params["debug_steer_scaling_factor"] = float(np.median(dsfs))
        params["steer_dead_band"] = 0.001
        params["steer_rate_lim"] = 5.0
        print(f"  同定結果: steer_time_constant = {params['steer_time_constant']:.4f} s")
        print(f"            steer_time_delay    = {params['steer_time_delay']:.4f} s")
        print(f"            steer_bias          = {params['steer_bias']:.6f} rad")
        print(f"            steer_scaling       = {params['debug_steer_scaling_factor']:.4f}")
    else:
        print("[WARN] 操舵の動的条件を満たすデータがないため、デフォルト値を設定します。")
        params["steer_time_constant"] = 0.12
        params["steer_time_delay"] = 0.05
        params["steer_bias"] = 0.0005
        params["debug_steer_scaling_factor"] = 1.0

    # k_us (アンダーステア勾配) の速度ビン別直接同定: 全データセットの定常旋回サンプルを
    # プールしてスカラー原点回帰 y = k_us·x (x=v·ω, y=tan(δ)-L·ω/v) を解く。
    print("[fit_steer] k_us 直接同定を実行中...")
    all_vx, all_wz, all_steer, all_dwz = [], [], [], []
    for ds in datasets:
        wz = ds["wz"]
        dwz_mid = np.diff(wz) / DT
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
    tan_steer = np.tan(np.clip(steer_f, -0.8, 0.8))

    x = vx_f * wz_f
    y = tan_steer - wheelbase * wz_f / vx_f if len(vx_f) else np.empty(0)
    sum_x2 = float(np.sum(x * x))
    sum_xy = float(np.sum(x * y))
    n_pts = int(len(vx_f))
    k_us = sum_xy / sum_x2 if (n_pts >= 10 and sum_x2 > 0) else 0.008
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
