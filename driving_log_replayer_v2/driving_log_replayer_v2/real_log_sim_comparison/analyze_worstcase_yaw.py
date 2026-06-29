"""worst-case yaw の per-dataset 分析スクリプト.

best params での N40 nyaw を dataset 別に計算し、
正規化スコアが高い (≥1.0) データセットを特定・分析する。

advisor の指示に従い:
  1. per-dataset N40 nyaw を列挙 → worst 10 を特定
  2. 絶対ヨー誤差 vs 正規化値を比較 (floor clip artifact 判定)
  3. vx_mean/max を確認 (高速域仮説の検証)
  4. tuned vs baseline の比較 (どこで tuned が baseline を上回るか)
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

# パスを通す
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent.parent.parent.parent))

import driving_log_replayer_v2.real_log_sim_comparison.step5_analyze_nstep as s5  # noqa: E402
import driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune as mdt  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (  # noqa: E402
    aggregate_normalized,
    YAW_FLOOR,
)

COLLECTION_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june")
BEST_PARAMS_YAML = Path("/home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_june_phase4.yaml")
HORIZONS = (10, 20, 30, 40)
N_JOBS = 8
TOP_N = 20  # worst-case 上位何件を詳細表示するか


def load_best_params() -> dict:
    with open(BEST_PARAMS_YAML) as f:
        d = yaml.safe_load(f)
    return d["params"]


def get_vx_stats(ctx: mdt.DatasetCtx) -> dict:
    """ctx.data["kin"] から速度統計を返す."""
    kin = ctx.data.get("kin")
    if kin is None or len(kin) == 0:
        return {"vx_mean": float("nan"), "vx_max": float("nan"), "vx_std": float("nan")}
    vx = kin["vx"].values if hasattr(kin, "columns") else np.array([])
    if len(vx) == 0:
        return {"vx_mean": float("nan"), "vx_max": float("nan"), "vx_std": float("nan")}
    # 自律走行開始以降のみ
    t_kin = kin["t"].values if "t" in (kin.columns if hasattr(kin, "columns") else []) else np.arange(len(vx))
    t0_sec = ctx.t0_ns * 1e-9
    mask = t_kin >= t0_sec
    vx_auto = vx[mask] if mask.any() else vx
    return {
        "vx_mean": float(np.mean(vx_auto)),
        "vx_max": float(np.max(vx_auto)),
        "vx_std": float(np.std(vx_auto)),
    }


def main() -> None:
    print("=== Worst-case N40 yaw 分析 ===", flush=True)
    print(f"COLLECTION_DIR : {COLLECTION_DIR}", flush=True)
    print(f"BEST_PARAMS    : {BEST_PARAMS_YAML}", flush=True)

    # 1. best params ロード
    best = load_best_params()
    print(f"\n[best params] k_us={best.get('k_us'):.5f} "
          f"vx_lo={best.get('k_us_vx_lo'):.2f} vx_hi={best.get('k_us_vx_hi'):.2f} "
          f"steer_tc={best.get('steer_time_constant'):.3f}", flush=True)

    # 2. データセットロード
    lite_dirs = mdt._discover(COLLECTION_DIR)
    print(f"\n[load] {len(lite_dirs)} datasets を読み込み中 (n_jobs={N_JOBS})...", flush=True)
    ctxs = mdt.load_datasets(lite_dirs, n_jobs=N_JOBS)
    print(f"[load] 有効: {len(ctxs)} datasets", flush=True)

    # 3. best params で評価
    print("\n[eval] best params で評価中...", flush=True)
    per_ds_metrics = [(ctx.dataset_id, mdt._eval(ctx, best, mdt._BASELINE_MODEL)) for ctx in ctxs]
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    agg = aggregate_normalized(per_ds_metrics, baselines, HORIZONS)

    # 4. per-dataset N40 nyaw + 絶対誤差 + vx 統計を収集
    print("[vx] 速度統計を収集中...", flush=True)
    ctx_by_id = {ctx.dataset_id: ctx for ctx in ctxs}
    vx_stats = {ctx.dataset_id: get_vx_stats(ctx) for ctx in ctxs}

    records = []
    for ds_rec in agg["per_ds"]:
        ds_id = ds_rec["dataset_id"]
        h40 = ds_rec["by_h"][40]
        h20 = ds_rec["by_h"][20]
        base_h40 = baselines[ds_id][40]
        floor_h40 = YAW_FLOOR[40]
        effective_denom = max(base_h40["yaw"], floor_h40)
        is_floor_clipped = base_h40["yaw"] < floor_h40
        vx = vx_stats.get(ds_id, {})
        records.append({
            "dataset_id": ds_id,
            # N40 yaw
            "nyaw40": h40["nyaw"],
            "yaw40_abs": h40["yaw"],          # tuned の絶対誤差 [deg]
            "base_yaw40": base_h40["yaw"],    # baseline の絶対誤差 [deg]
            "denom40": effective_denom,       # 正規化分母 (floor clip 後)
            "floor_clipped": is_floor_clipped,
            # N20 yaw
            "nyaw20": h20["nyaw"],
            "yaw20_abs": h20["yaw"],
            "base_yaw20": base_h40["yaw"],
            # 速度
            "vx_mean": vx.get("vx_mean", float("nan")),
            "vx_max": vx.get("vx_max", float("nan")),
        })

    records.sort(key=lambda r: r["nyaw40"], reverse=True)

    # 5. 集計サマリ
    by_h = agg["by_h"]
    print("\n--- 集計結果 (best params) ---")
    for h in HORIZONS:
        b = by_h[h]
        print(f"  N{h}: nyaw_mean={b['nyaw_mean']:.4f} nyaw_worst={b['nyaw_worst']:.4f} "
              f"nlat_mean={b['nlat_mean']:.4f} nlat_worst={b['nlat_worst']:.4f}")

    # nyaw > 1.0 の数
    n_over1 = sum(1 for r in records if r["nyaw40"] > 1.0)
    n_floor = sum(1 for r in records if r["floor_clipped"])
    print(f"\n  N40 nyaw > 1.0 : {n_over1}/{len(records)} datasets")
    print(f"  floor-clip 発生 : {n_floor}/{len(records)} datasets (baseline < {YAW_FLOOR[40]} deg)")

    # 6. worst-case TOP N
    print(f"\n--- Worst-case TOP {TOP_N} (N40 nyaw 降順) ---")
    header = (f"{'dataset_id':36s}  nyaw40  yaw40  base40  denom  floor?  "
              f"nyaw20  vx_mean  vx_max")
    print(header)
    print("-" * len(header))
    for r in records[:TOP_N]:
        flag = "Y" if r["floor_clipped"] else " "
        print(
            f"{r['dataset_id']:36s}  "
            f"{r['nyaw40']:6.3f}  "
            f"{r['yaw40_abs']:5.3f}  "
            f"{r['base_yaw40']:6.3f}  "
            f"{r['denom40']:5.3f}  "
            f"{flag:5s}  "
            f"{r['nyaw20']:6.3f}  "
            f"{r['vx_mean']:7.2f}  "
            f"{r['vx_max']:6.2f}"
        )

    # 7. nyaw40 > 1.0 かつ floor-clip なし (genuine failure) を抽出
    genuine = [r for r in records if r["nyaw40"] > 1.0 and not r["floor_clipped"]]
    artifact = [r for r in records if r["nyaw40"] > 1.0 and r["floor_clipped"]]
    print(f"\n--- nyaw40 > 1.0 分析 ---")
    print(f"  genuine failure (base_yaw40 >= {YAW_FLOOR[40]}): {len(genuine)}")
    print(f"  floor-clip artifact (base_yaw40 < {YAW_FLOOR[40]}):  {len(artifact)}")

    if genuine:
        print(f"\n--- Genuine failures (vx_mean 降順) ---")
        genuine.sort(key=lambda r: r["vx_mean"], reverse=True)
        for r in genuine[:min(10, len(genuine))]:
            print(f"  {r['dataset_id']:36s}  nyaw40={r['nyaw40']:.3f}  "
                  f"yaw40={r['yaw40_abs']:.3f}deg  base={r['base_yaw40']:.3f}deg  "
                  f"vx_mean={r['vx_mean']:.2f} vx_max={r['vx_max']:.2f}")

    if artifact:
        print(f"\n--- Floor-clip artifacts (nyaw40 降順) ---")
        for r in sorted(artifact, key=lambda r: r["nyaw40"], reverse=True)[:min(10, len(artifact))]:
            print(f"  {r['dataset_id']:36s}  nyaw40={r['nyaw40']:.3f}  "
                  f"yaw40={r['yaw40_abs']:.3f}deg  base={r['base_yaw40']:.3f}deg  "
                  f"vx_mean={r['vx_mean']:.2f}")

    # 8. nyaw40 ≥ 1.0 の vx 統計 vs 全体統計
    all_vx_means = [r["vx_mean"] for r in records if not np.isnan(r["vx_mean"])]
    bad_vx_means = [r["vx_mean"] for r in genuine if not np.isnan(r["vx_mean"])]
    if all_vx_means:
        print(f"\n--- 速度分布比較 ---")
        print(f"  全 datasets vx_mean: avg={np.mean(all_vx_means):.2f} median={np.median(all_vx_means):.2f} "
              f"max={np.max(all_vx_means):.2f}")
        if bad_vx_means:
            print(f"  genuine failure vx_mean: avg={np.mean(bad_vx_means):.2f} "
                  f"median={np.median(bad_vx_means):.2f} max={np.max(bad_vx_means):.2f}")

    # 9. nyaw40 分布
    nyaw40_all = [r["nyaw40"] for r in records]
    print(f"\n--- nyaw40 分布 ---")
    for thresh in [0.5, 0.7, 0.9, 1.0, 1.1, 1.2, 1.5]:
        cnt = sum(1 for v in nyaw40_all if v >= thresh)
        print(f"  nyaw40 >= {thresh:.1f}: {cnt:4d} / {len(nyaw40_all)}")


if __name__ == "__main__":
    main()
