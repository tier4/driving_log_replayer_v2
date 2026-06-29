#!/usr/bin/env python3
"""
k_us スコア感度診断スクリプト

仮説1（高速カーブデータ不足）vs 仮説2（δ_cmd クローズドループバイアス）の切り分け。

実験A: k_us スイープ（全 DS vs 高速カーブ DS）
  - 全パラメータを phase44 固定、k_us_bands 全体を同一値でスイープ
  - スコア最小点の位置と感度曲線の形状を比較
  - 全 DS で低 k_us が最適 → 高速カーブ DS でも低 k_us が最適なら仮説2（バイアス）
  - 高速カーブ DS で高 k_us に最小点が移動するなら仮説1（データ不足）

実験B: δ_actual 使用時の k_us スイープ
  - 操舵入力を δ_cmd → δ_actual に差し替えて同じ k_us スイープ
  - 最小点が高 k_us 側に移動するなら仮説2（δ_cmd バイアス）が主因
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

_INSTALL = Path(
    "/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/"
    "driving_log_replayer_v2/local/lib/python3.10/dist-packages"
)
if _INSTALL.exists() and str(_INSTALL) not in sys.path:
    sys.path.insert(0, str(_INSTALL))

from driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune import (
    _BASELINE_MODEL,
    _discover,
    _eval as _tune_eval,
    load_datasets,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (
    HORIZONS,
    aggregate_normalized,
    steer_score,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_kinematic, load_steering
from driving_log_replayer_v2.real_log_sim_comparison.lib._collection import datasets_root

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
COLLECTION_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june")
PARAMS_YAML    = Path("/home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_june_phase44.yaml")
OUT_CSV        = Path("/home/kotaroyoshimoto/data/openloop_j6_15/kus_sensitivity.csv")
N_CURVE_DS     = 30   # 高速カーブ DS として使う上位 N 件
N_JOBS         = 8
VX_THRESH      = 2.71  # k_us_mid 以上が有効になる速度 [m/s]
KAPPA_THRESH   = 0.02  # カーブ判定曲率 [1/m]
KUS_SWEEP      = np.array([0.001, 0.002, 0.003, 0.004, 0.006, 0.008, 0.010,
                            0.012, 0.015, 0.018, 0.022, 0.028])

# ---------------------------------------------------------------------------
# ユーティリティ
# ---------------------------------------------------------------------------
def _compute_curve_fraction(lite_dir: Path, vx_thresh: float, kappa_thresh: float) -> float:
    """DS の走行中で「高速カーブ」に該当する時間割合を返す。"""
    mcap = lite_dir / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return 0.0
    try:
        df_kin = load_kinematic(mcap)
        if df_kin.empty or len(df_kin) < 10:
            return 0.0
        vx = df_kin["vx"].values
        wz = df_kin["wz"].values
        kappa = np.where(vx > 0.5, np.abs(wz / vx), 0.0)
        in_curve_highspeed = (vx > vx_thresh) & (kappa > kappa_thresh)
        return float(in_curve_highspeed.mean())
    except Exception:
        return 0.0


def _eval_score(ctx, params_override: dict) -> dict | None:
    """1 DS の steer_score 用メトリクスを返す（失敗時 None）。"""
    try:
        p14 = _tune_eval(ctx, params_override, _BASELINE_MODEL)
        bl  = ctx.base_metric
        return {
            int(h): {
                "p14_yaw": p14[h]["yaw"], "p14_lat": p14[h]["lat"],
                "bl_yaw":  bl[h]["yaw"],  "bl_lat":  bl[h]["lat"],
            }
            for h in HORIZONS
        }
    except Exception as e:
        print(f"  [WARN] {ctx.dataset_id[:12]}: {e}")
        return None


def _score_from_metrics(metrics_by_uuid: dict) -> float:
    """metrics_by_uuid: {uuid: {h: {p14_yaw, p14_lat, bl_yaw, bl_lat}}} → steer_score"""
    per_ds = []
    bl_map = {}
    for uuid, hmap in metrics_by_uuid.items():
        per_ds.append((uuid, {h: {"yaw": v["p14_yaw"], "lat": v["p14_lat"], "long": 0.0}
                               for h, v in hmap.items()}))
        bl_map[uuid] = {h: {"yaw": v["bl_yaw"], "lat": v["bl_lat"], "long": 0.0}
                        for h, v in hmap.items()}
    agg = aggregate_normalized(per_ds, bl_map)
    return steer_score(agg)


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def main() -> None:
    # パラメータ読み込み
    with open(PARAMS_YAML) as f:
        yaml_data = yaml.safe_load(f)
    base_params: dict = {k: v for k, v in yaml_data.get("params", yaml_data).items()
                         if not k.startswith("_")}
    print(f"基準パラメータ: {PARAMS_YAML.name}")
    print(f"  k_us_lo={base_params.get('k_us_lo', 'N/A'):.5f}  "
          f"k_us_mid={base_params.get('k_us_mid', 'N/A'):.5f}  "
          f"k_us={base_params.get('k_us', 'N/A'):.5f}")

    # DS 発見
    ds_root = datasets_root(COLLECTION_DIR)
    ds_list = _discover(COLLECTION_DIR)
    print(f"\nデータセット: {len(ds_list)} 件")

    # --- 高速カーブ割合を計算して DS をランク付け ---
    print(f"\n[Step 1] 高速カーブ割合を計算 (vx>{VX_THRESH} m/s かつ κ>{KAPPA_THRESH} 1/m) ...")
    curve_fracs = {}
    for i, (uuid, lite_dir) in enumerate(ds_list):
        frac = _compute_curve_fraction(lite_dir, VX_THRESH, KAPPA_THRESH)
        curve_fracs[uuid] = frac
        if (i + 1) % 100 == 0:
            print(f"  {i+1}/{len(ds_list)} 処理済み")

    sorted_ds = sorted(ds_list, key=lambda x: curve_fracs[x[0]], reverse=True)
    curve_ds  = sorted_ds[:N_CURVE_DS]
    print(f"  高速カーブ上位 {N_CURVE_DS} DS:")
    for uuid, _ in curve_ds[:10]:
        print(f"    {uuid[:12]}  curve_frac={curve_fracs[uuid]:.3f}")

    # 統計: 全 DS のカーブ割合分布
    fracs = np.array(list(curve_fracs.values()))
    print(f"\n  高速カーブ割合 (全 DS):")
    print(f"    p50={np.median(fracs):.4f}  p90={np.percentile(fracs,90):.4f}  "
          f"p99={np.percentile(fracs,99):.4f}  max={fracs.max():.4f}")
    print(f"    > 0.01 の DS: {(fracs > 0.01).sum()} 件")
    print(f"    > 0.05 の DS: {(fracs > 0.05).sum()} 件")
    print(f"    > 0.10 の DS: {(fracs > 0.10).sum()} 件")

    # --- DatasetCtx ロード ---
    print(f"\n[Step 2] DatasetCtx ロード ({N_CURVE_DS} DS) ...")
    ctxs = load_datasets(curve_ds, n_jobs=N_JOBS)
    print(f"  ロード完了: {len(ctxs)} DS")

    # --- 実験A: k_us スイープ（全方向を同一値で変化） ---
    print(f"\n[Step 3] k_us スイープ ({len(KUS_SWEEP)} 値 × {len(ctxs)} DS) ...")
    rows = []
    for kus_val in KUS_SWEEP:
        # k_us_lo / k_us_mid / k_us をすべて kus_val に設定（速度帯の影響を除外して純粋なk_us感度を見る）
        override_same = {**base_params,
                         "k_us": kus_val, "k_us_lo": kus_val, "k_us_mid": kus_val}
        # k_us のみ変化（速度帯は維持）
        override_high = {**base_params, "k_us": kus_val}
        override_mid  = {**base_params, "k_us_mid": kus_val}

        metrics_same, metrics_high, metrics_mid = {}, {}, {}
        for ctx in ctxs:
            r_same  = _eval_score(ctx, override_same)
            r_high  = _eval_score(ctx, override_high)
            r_mid   = _eval_score(ctx, override_mid)
            if r_same:  metrics_same[ctx.dataset_id]  = r_same
            if r_high:  metrics_high[ctx.dataset_id]  = r_high
            if r_mid:   metrics_mid[ctx.dataset_id]   = r_mid

        sc_same = _score_from_metrics(metrics_same) if metrics_same else float("nan")
        sc_high = _score_from_metrics(metrics_high) if metrics_high else float("nan")
        sc_mid  = _score_from_metrics(metrics_mid)  if metrics_mid  else float("nan")

        print(f"  k_us={kus_val:.4f}  score(全帯同値)={sc_same:.4f}  "
              f"score(k_us_high変化)={sc_high:.4f}  score(k_us_mid変化)={sc_mid:.4f}")
        rows.append({"k_us_val": kus_val,
                     "score_all_bands": sc_same,
                     "score_kus_only":  sc_high,
                     "score_mid_only":  sc_mid})

    df = pd.DataFrame(rows)
    df.to_csv(OUT_CSV, index=False)
    print(f"\n結果 CSV: {OUT_CSV}")

    # --- 実験B: δ_actual 使用時の感度（簡易版: phase44 vs k_us=0.015 の ωz 誤差比較） ---
    print(f"\n[Step 4] δ_cmd vs δ_actual の乖離分析 ...")
    _analyze_delta_gap(curve_ds)


def _analyze_delta_gap(ds_list_subset: list) -> None:
    """
    δ_cmd と δ_actual の差を速度帯別に集計。
    もし δ_cmd > δ_actual（コントローラがオーバーコマンド）なら仮説2（δ_cmd バイアス）を支持。
    """
    from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_steering

    # /control/command/control_cmd (δ_cmd) は load_steering と別トピック
    # lite MCAP にある steering_status が δ_actual
    # δ_cmd は control_cmd トピック
    results = []
    for uuid, lite_dir in ds_list_subset:
        mcap = lite_dir / "real.lite" / "real.lite_0.mcap"
        if not mcap.exists():
            continue
        try:
            df_kin    = load_kinematic(mcap)
            df_steer  = load_steering(mcap)   # δ_actual (/vehicle/status/steering_status)
            df_cmd    = _load_steer_cmd(mcap) # δ_cmd (/control/command/control_cmd)
            if df_kin.empty or df_steer.empty or df_cmd is None or df_cmd.empty:
                continue

            t_k    = df_kin["t_ns"].values * 1e-9
            vx     = df_kin["vx"].values
            wz     = df_kin["wz"].values
            kappa  = np.where(vx > 0.5, np.abs(wz / vx), 0.0)

            t_s    = df_steer["t_ns"].values * 1e-9
            delta_actual = np.interp(t_k, t_s, df_steer["steer"].values)

            t_c    = df_cmd["t_ns"].values * 1e-9
            delta_cmd    = np.interp(t_k, t_c, df_cmd["steer"].values)

            for vx_lo, vx_hi in [(2.71, 5.91), (5.91, 20.0)]:
                mask = (vx >= vx_lo) & (vx < vx_hi) & (kappa > KAPPA_THRESH)
                if mask.sum() < 5:
                    continue
                diff = delta_cmd[mask] - delta_actual[mask]
                results.append({
                    "uuid":   uuid[:12],
                    "vx_band": f"{vx_lo:.1f}-{vx_hi:.1f}",
                    "n":       int(mask.sum()),
                    "diff_mean": float(diff.mean()),   # 正→δ_cmd > δ_actual（オーバーコマンド）
                    "diff_p50":  float(np.median(diff)),
                    "diff_p90":  float(np.percentile(np.abs(diff), 90)),
                })
        except Exception as e:
            print(f"  [WARN] {uuid[:12]}: {e}")

    if not results:
        print("  ⚠ δ_cmd トピックが取得できませんでした")
        return

    df_gap = pd.DataFrame(results)
    for band, grp in df_gap.groupby("vx_band"):
        print(f"\n  速度帯 {band} m/s (カーブ中, κ>{KAPPA_THRESH}):")
        print(f"    DS 数: {len(grp)}")
        print(f"    δ_cmd - δ_actual  mean={grp['diff_mean'].mean()*1000:.2f} mrad  "
              f"p50={grp['diff_p50'].median()*1000:.2f} mrad  "
              f"|diff| p90={grp['diff_p90'].median()*1000:.2f} mrad")
        pos_frac = (grp["diff_mean"] > 0.001).mean()
        print(f"    δ_cmd > δ_actual の DS 割合: {pos_frac:.1%}")

    gap_csv = OUT_CSV.parent / "delta_gap_analysis.csv"
    df_gap.to_csv(gap_csv, index=False)
    print(f"\n  δ乖離 CSV: {gap_csv}")


def _load_steer_cmd(mcap_path: Path):
    """control_cmd から steering_tire_angle を読み込む。"""
    try:
        from driving_log_replayer_v2.real_log_sim_comparison.lib._io import iter_bag_messages
        from rosidl_runtime_py.utilities import get_message

        TOPIC = "/control/command/control_cmd"
        rows = []
        for t_ns, msg in iter_bag_messages(mcap_path, [TOPIC]):
            try:
                steer = msg.lateral.steering_tire_angle
            except AttributeError:
                steer = msg.steering_tire_angle
            rows.append({"t_ns": t_ns, "steer": steer})
        if not rows:
            return None
        return pd.DataFrame(rows)
    except Exception:
        return None


if __name__ == "__main__":
    main()
