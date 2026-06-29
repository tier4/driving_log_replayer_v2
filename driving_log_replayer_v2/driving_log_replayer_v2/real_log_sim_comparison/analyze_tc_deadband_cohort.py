"""TC / dead_band コホートテスト.

advisor の指示: C++ 変更前に仮説を実測で検証する。
- 低速コホート (vx_mean < 2.5 m/s) vs 高速コホート (≥ 2.5 m/s) で
  steer_time_constant と steer_dead_band のスイープを行い、
  どちらが nyaw40 worst/mean に効くかを特定する。

実験 A: TC を変えて dead_band=0.00244 固定
実験 B: dead_band を変えて TC=0.142 固定
比較: baseline (TC=0.15, dead_band=0) との差

vx 統計は前スクリプトと同じ方法 (kin["t"] ≥ t0_sec でマスク)。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent.parent.parent.parent))

import driving_log_replayer_v2.real_log_sim_comparison.step5_analyze_nstep as s5  # noqa: E402
import driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune as mdt  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import aggregate_normalized  # noqa: E402

COLLECTION_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june")
BEST_PARAMS_YAML = Path("/home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_june_steer_params.yaml")
HORIZONS = (10, 20, 30, 40)
N_JOBS = 8
VX_SPLIT = 2.5  # low: <2.5 m/s, high: ≥2.5 m/s


def load_best_params() -> dict:
    with open(BEST_PARAMS_YAML) as f:
        return yaml.safe_load(f)["params"]


def get_vx_mean(ctx: mdt.DatasetCtx) -> float:
    kin = ctx.data.get("kin")
    if kin is None or len(kin) == 0:
        return float("nan")
    vx_col = "vx" if "vx" in kin.columns else None
    if vx_col is None:
        return float("nan")
    t_col = "t" if "t" in kin.columns else None
    if t_col is not None:
        t0_sec = ctx.t0_ns * 1e-9
        mask = kin[t_col].values >= t0_sec
        vx = kin[vx_col].values[mask] if mask.any() else kin[vx_col].values
    else:
        vx = kin[vx_col].values
    return float(np.mean(vx)) if len(vx) > 0 else float("nan")


def eval_override(ctxs, override, vx_means):
    """override で全 ctx を評価し、low/high コホートの nyaw40 統計を返す。"""
    per_ds_metrics = [(ctx.dataset_id, mdt._eval(ctx, override, mdt._BASELINE_MODEL)) for ctx in ctxs]
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    agg = aggregate_normalized(per_ds_metrics, baselines, HORIZONS)

    low_nyaw40, high_nyaw40 = [], []
    for ds_rec in agg["per_ds"]:
        ds_id = ds_rec["dataset_id"]
        v = vx_means.get(ds_id, float("nan"))
        nyaw40 = ds_rec["by_h"][40]["nyaw"]
        if np.isnan(v):
            continue
        if v < VX_SPLIT:
            low_nyaw40.append(nyaw40)
        else:
            high_nyaw40.append(nyaw40)

    def stats(vals):
        if not vals:
            return {"mean": float("nan"), "worst": float("nan"), "n": 0}
        return {"mean": float(np.mean(vals)), "worst": float(max(vals)), "n": len(vals)}

    return {
        "by_h40": agg["by_h"][40],
        "low": stats(low_nyaw40),
        "high": stats(high_nyaw40),
    }


def print_row(label, res):
    b = res["by_h40"]
    l, h = res["low"], res["high"]
    print(
        f"  {label:40s}  "
        f"all: mean={b['nyaw_mean']:.4f} worst={b['nyaw_worst']:.4f}  "
        f"low(n={l['n']}): mean={l['mean']:.4f} w={l['worst']:.4f}  "
        f"high(n={h['n']}): mean={h['mean']:.4f} w={h['worst']:.4f}"
    )


def main():
    print("=== TC / dead_band コホートテスト ===")
    print(f"VX_SPLIT={VX_SPLIT} m/s: low=<{VX_SPLIT}, high=>={VX_SPLIT}\n")

    best = load_best_params()
    default = s5._build_params()

    # 1. データセットロード
    lite_dirs = mdt._discover(COLLECTION_DIR)
    print(f"[load] {len(lite_dirs)} datasets (n_jobs={N_JOBS})...", flush=True)
    ctxs = mdt.load_datasets(lite_dirs, n_jobs=N_JOBS)
    print(f"[load] 有効: {len(ctxs)}", flush=True)

    # vx_mean を事前計算
    vx_means = {ctx.dataset_id: get_vx_mean(ctx) for ctx in ctxs}
    low_n = sum(1 for v in vx_means.values() if not np.isnan(v) and v < VX_SPLIT)
    high_n = sum(1 for v in vx_means.values() if not np.isnan(v) and v >= VX_SPLIT)
    print(f"  コホート分布: low={low_n}, high={high_n}\n")

    # ---- ベースライン ----
    print("--- ベースライン (TC=0.15, dead_band=0.0, k_us=0) ---")
    res_baseline = eval_override(ctxs, {}, vx_means)
    print_row("baseline (default)", res_baseline)

    # ---- チューニング済みベスト ----
    print("\n--- チューニング済みベスト ---")
    res_tuned = eval_override(ctxs, best, vx_means)
    print_row(f"tuned (TC={best['steer_time_constant']:.3f} db={best['steer_dead_band']:.4f})", res_tuned)

    # ---- 実験 A: TC スイープ (dead_band=0.00244 固定) ----
    print("\n--- 実験 A: TC スイープ (dead_band=best, 他=best) ---")
    tc_candidates = [0.05, 0.08, 0.10, 0.12, 0.142, 0.15, 0.20]
    for tc in tc_candidates:
        override = {**best, "steer_time_constant": tc}
        res = eval_override(ctxs, override, vx_means)
        print_row(f"TC={tc:.3f}", res)

    # ---- 実験 B: dead_band スイープ (TC=0.142 固定) ----
    print("\n--- 実験 B: dead_band スイープ (TC=best, 他=best) ---")
    db_candidates = [0.0, 0.0005, 0.001, 0.00244, 0.005]
    for db in db_candidates:
        override = {**best, "steer_dead_band": db}
        res = eval_override(ctxs, override, vx_means)
        print_row(f"dead_band={db:.4f}", res)

    # ---- 実験 C: TC × dead_band 2D (promising 領域) ----
    print("\n--- 実験 C: TC×dead_band 組み合わせ (key points) ---")
    combos = [
        ("TC=0.05, db=0.0",    0.05, 0.0),
        ("TC=0.05, db=0.001",  0.05, 0.001),
        ("TC=0.08, db=0.0",    0.08, 0.0),
        ("TC=0.08, db=0.001",  0.08, 0.001),
        ("TC=0.10, db=0.0",    0.10, 0.0),
        ("TC=0.10, db=0.001",  0.10, 0.001),
        ("TC=0.15, db=0.0",    0.15, 0.0),  # baseline 相当 (k_us あり)
    ]
    for label, tc, db in combos:
        override = {**best, "steer_time_constant": tc, "steer_dead_band": db}
        res = eval_override(ctxs, override, vx_means)
        print_row(label, res)

    print("\n完了。")


if __name__ == "__main__":
    main()
