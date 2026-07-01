#!/usr/bin/env python3
"""
k_us_eff yaw レート層同定スクリプト（Phase 48 新前提版）

【前提】
2026-06-16 以降、実車の raw_vehicle_cmd_converter が understeer 補正
  desired_steer = δ_cmd * (1 + K_us * v²/L)  (K_us=0.020, j6_gen2)
を適用する。open-loop ハーネスはこの補正を含まないため、シム実効 k_us として
  k_us_eff ≈ k_us_phys - K_us
を同定する（負許容: curve ツリー floor guard により数値的安全）。

【N=1 禁止】
run_rollout のリセット時に steer_init = atan(gt_wz * L / vx)（k_us=0 bicycle 逆算）
を使うため、N=1 では k_us=0 が構造的に最小化され真の k_us_eff を検出できない
（step_ol1_analyze_nstep.py 警告 参照）。
N=20-40 では actuator lag + コマンド系列がステアをシードから引き離し感度が出る。

【クロス確認】
N=10,20,30,40 で最小点位置を並べ、N 減少で 0 に引き戻されないか確認する。
N↓→0: シードアーティファクト残存（大 N の最小点を採用）
N↓→安定: シードウォッシュアウト完了、信頼できる k_us_eff。

【使い方】
  python diagnose_kus_wz_id.py
出力:
  kus_wz_id_sweep.csv   — k_us スイープ結果 (各ホライズン wz_rmse / alat_rmse)
  kus_wz_delta_gap.csv  — δ_cmd vs δ_actual 乖離分析
"""
from __future__ import annotations

import datetime
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

# ---------------------------------------------------------------------------
# インストールパス（curve ツリービルド）
# ---------------------------------------------------------------------------
_INSTALL = Path(
    "/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/"
    "driving_log_replayer_v2/local/lib/python3.10/dist-packages"
)
if _INSTALL.exists() and str(_INSTALL) not in sys.path:
    sys.path.insert(0, str(_INSTALL))

import driving_log_replayer_v2.real_log_sim_comparison.step_ol1_analyze_nstep as s5
from driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune import (
    _BASELINE_MODEL,
    DatasetCtx,
    _discover,
    _eval as _tune_eval,
    _filter_by_date,
    load_datasets,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (
    HORIZONS as POS_HORIZONS,
    aggregate_normalized,
    steer_score,
)
from driving_log_replayer_v2.real_log_sim_comparison.diagnose_kus_sensitivity import (
    KAPPA_THRESH,
    VX_THRESH,
    _compute_curve_fraction,
    _load_steer_cmd,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_kinematic, load_steering

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
COLLECTION_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june")
PARAMS_YAML    = Path("/home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_phase48.yaml")
OUT_DIR        = Path("/home/kotaroyoshimoto/data/openloop_j6_15")
OUT_CSV        = OUT_DIR / "kus_wz_id_sweep.csv"

# post-6/16: understeer 補正適用後のデータのみ使用（pre/post を confound しない）
DS_AFTER   = datetime.date(2026, 6, 16)
N_CURVE_DS = 30    # 高速カーブ DS 上位 N 件
N_JOBS     = 8
STRIDE     = 5     # run_rollout ストライド（速度と網羅性のトレードオフ）

# N=10: シードアーティファクト確認用（主評価に使わない）
# N=20-40: 主評価ホライズン（actuator lag がシードを洗い出す）
WZ_HORIZONS: tuple[int, ...] = (10, 20, 30, 40)

# k_us_eff 掃引: 負を許容（converter 過補償 k_us_eff < 0 を検出可能にする）
KUS_SWEEP = np.linspace(-0.03, 0.03, 25)

# DSF 吸収バイアス対策
# True: DSF=1.0 / steer_bias=0.0 に固定してスイープ（推奨）
#   Phase 48: DSF=1.0494 (4.9% 増) は v²-shape ヨー誤差を部分吸収し |k_us_eff| を過小評価する
# False: base_params の DSF/steer_bias をそのまま使用（(k_us, DSF) 同時確認時に使う）
FIX_DSF = True


# ---------------------------------------------------------------------------
# yaw レート + 横加速度 RMSE 評価（メイン目的関数）
# ---------------------------------------------------------------------------
def _eval_wz_rmse(
    ctx: DatasetCtx,
    params_override: dict,
    horizons: tuple[int, ...] = WZ_HORIZONS,
    stride: int = STRIDE,
) -> dict[int, dict] | None:
    """1 DS の per-horizon yaw rate RMSE と横加速度 RMSE を返す。

    WARNING: N=1 は禁止（step_ol1_analyze_nstep.py 警告）。
    リセット steer_init = atan(gt_wz * L / vx) は k_us=0 bicycle 逆算のため、
    N=1 では k_us=0 が構造的に最小化され真の k_us_eff を検出できない。
    N≥20 のホライズンのみ主評価に使うこと。N=10 はクロス確認専用。

    Returns:
        {h: {"wz_rmse": float, "alat_rmse": float, "n": int}} for h in horizons,
        or None on failure.
    """
    params = dict(ctx.base)
    params.update(params_override)

    try:
        df = s5.run_rollout(
            ctx.data, ctx.t0_ns, params, _BASELINE_MODEL,
            horizons=horizons, stride=stride,
        )
    except Exception as e:
        print(f"  [WARN] {ctx.dataset_id[:12]}: run_rollout 失敗: {e}")
        return None

    if df is None or df.empty:
        return None

    result: dict[int, dict] = {}
    for h in horizons:
        df_h = df[df["horizon"] == h]
        if len(df_h) < 3:
            continue

        # wz RMSE: err_wz = real_wz[k_end] - sim_wz[k_end] (run_rollout:1160)
        wz_rmse = float(np.sqrt((df_h["err_wz"].values ** 2).mean()))

        # a_lat RMSE: a_lat = vx * wz
        #   sim_wz = real_wz - err_wz  (run_rollout の定義より)
        #   real_alat = real_vx_kend * real_wz
        #   sim_alat  = sim_vx * sim_wz
        sim_wz_vals = df_h["real_wz"].values - df_h["err_wz"].values
        real_alat   = df_h["real_vx_kend"].values * df_h["real_wz"].values
        sim_alat    = df_h["sim_vx"].values * sim_wz_vals
        alat_rmse   = float(np.sqrt(((real_alat - sim_alat) ** 2).mean()))

        result[h] = {
            "wz_rmse":   wz_rmse,
            "alat_rmse": alat_rmse,
            "n":         int(len(df_h)),
        }

    return result or None


# ---------------------------------------------------------------------------
# override 組み立て（Phase 47 バンディング無効化パターン）
# ---------------------------------------------------------------------------
def _build_kus_override(
    base_params: dict,
    kus_val: float,
    fix_dsf: bool = FIX_DSF,
) -> dict:
    """Phase 47 と同じ手順で banding を完全無効化し、k_us をスカラーとして設定する。

    Phase 48 パラメータは k_us_vx_lo/hi/thresh* を持ち n_kus_bands_>0 になりうるため、
    k_us スカラーを override するだけでは calc_yaw_rate に届かない
    （banding が k_us_ を完全に隠蔽する。C++: if (n_kus_bands_ > 0) k_us_eff = band_values_[last]）。
    multi_dataset_tune.py Phase 47 (lines 688-698) と同一処理で banding を無効化する。

    thresh=0 → Python 側で bands/thresholds ベクタが空 → n_kus_bands_=0 → scalar mode に切替。
    """
    override = dict(base_params)
    # thresh=0 → scalar mode（Phase 47 line 691-692 と同一）
    override["k_us_vx_lo"]     = 0.0
    override["k_us_vx_hi"]     = 0.0
    override["k_us_vx_thresh"]  = 0.0
    override["k_us_vx_thresh2"] = 0.0
    # 旧式バンドキー（phase44 系 k_us_lo/mid）を除去
    override.pop("k_us_lo", None)
    override.pop("k_us_mid", None)
    # k_us_bands / k_us_thresholds を空リストで上書き → n_kus_bands=0 → scalar mode
    # 重要: pop() では ctx.base 由来の同名キー（simulator_model.param.yaml の k_us_bands=[0,0.02,0.016]）
    # が params.update(override) 後に残存する。空リストで明示的に上書きすることで消去する。
    override["k_us_bands"]      = []
    override["k_us_thresholds"] = []
    # sweep 対象: k_us スカラーを設定（Phase 47 では探索対象として pop; ここでは sweep 値を注入）
    override["k_us"] = kus_val
    if fix_dsf:
        # Phase 48 DSF=1.0494 は v²-shape ヨー誤差を部分吸収 → |k_us_eff| 過小評価。
        # DSF=1.0 / steer_bias=0.0 に固定して吸収バイアスを排除する。
        override["debug_steer_scaling_factor"] = 1.0
        override["steer_bias"] = 0.0
    return override


# ---------------------------------------------------------------------------
# 2-point プローブ: override が calc_yaw_rate に届くか事前確認
# ---------------------------------------------------------------------------
def _probe_override(
    ctxs: list[DatasetCtx],
    base_params: dict,
    fix_dsf: bool = FIX_DSF,
    horizon: int = 40,
    stride: int = STRIDE,
) -> bool:
    """k_us ∈ {-0.03, +0.03} で wz_rmse に差異があるか確認する。

    フラット（差≈0）→ banding が scalar k_us を隠蔽 → override 不到達 → フルスイープを中断すべき。
    差あり         → override が calc_yaw_rate に到達 → フルスイープ続行可。

    最もカーブ割合の高い DS（ctxs[0]）を使用する。

    Returns:
        True:  override 有効（続行可）
        False: フラット（override 不到達・中断推奨）
    """
    ctx_probe = ctxs[0]
    probe_vals = (-0.03, +0.03)
    wz_rmses: list[float] = []

    print(f"\n[プローブ] k_us override 到達確認 (DS={ctx_probe.dataset_id[:12]}, N={horizon}) ...")
    for kv in probe_vals:
        ov = _build_kus_override(base_params, kv, fix_dsf)
        r = _eval_wz_rmse(ctx_probe, ov, horizons=(horizon,), stride=stride)
        if r and horizon in r:
            wz_rmses.append(r[horizon]["wz_rmse"])
            print(f"  k_us={kv:+.3f} → wz_rmse={r[horizon]['wz_rmse']:.5f}")
        else:
            wz_rmses.append(float("nan"))
            print(f"  k_us={kv:+.3f} → 評価失敗 (None)")

    if len(wz_rmses) == 2 and all(not np.isnan(x) for x in wz_rmses):
        diff = abs(wz_rmses[0] - wz_rmses[1])
        print(f"  |rmse(-0.03) - rmse(+0.03)| = {diff:.5f}")
        if diff < 1e-6:
            print("  ⛔ FLAT: override が calc_yaw_rate に届いていない。")
            print("     banding キー（k_us_bands/k_us_thresholds/_build_params の挙動）を確認してください。")
            return False
        if diff > 5e-4:
            print("  ✓ 差異確認 → override 有効。フルスイープを続行します。")
        else:
            print("  ⚠ 差異が微小 (< 5e-4)。カーブ感度が低い DS の可能性あり。結果を慎重に解釈してください。")
        return True

    # 評価失敗時は警告を出して続行（別途エラーログで確認）
    print("  ⚠ プローブ評価失敗。続行しますが override 到達を保証できません。")
    return True


# ---------------------------------------------------------------------------
# k_us スイープ
# ---------------------------------------------------------------------------
def _sweep_kus_wz(
    ctxs: list[DatasetCtx],
    base_params: dict,
    kus_sweep: np.ndarray,
    horizons: tuple[int, ...] = WZ_HORIZONS,
    stride: int = STRIDE,
    fix_dsf: bool = FIX_DSF,
) -> pd.DataFrame:
    """k_us スカラを掃引し各ホライズンで wz_rmse / alat_rmse を DS 横断集計する。

    fix_dsf=True:  DSF=1.0 / steer_bias=0.0 に固定（DSF 吸収バイアス排除、推奨）
    fix_dsf=False: base_params の DSF/steer_bias をそのまま使用
    """
    rows = []
    for kus_val in kus_sweep:
        # Phase 47 バンディング無効化 + DSF 吸収バイアス排除（_build_kus_override 参照）
        override = _build_kus_override(base_params, kus_val, fix_dsf)

        ds_results = []
        for ctx in ctxs:
            r = _eval_wz_rmse(ctx, override, horizons=horizons, stride=stride)
            if r is not None:
                ds_results.append(r)

        row: dict = {"k_us": kus_val, "n_ds": len(ds_results)}
        for h in horizons:
            h_wz   = [r[h]["wz_rmse"]   for r in ds_results if h in r]
            h_alat = [r[h]["alat_rmse"] for r in ds_results if h in r]
            row[f"wz_rmse_N{h}"]   = float(np.mean(h_wz))   if h_wz   else float("nan")
            row[f"alat_rmse_N{h}"] = float(np.mean(h_alat)) if h_alat else float("nan")
        rows.append(row)

        parts = [
            f"N{h}:wz={row.get(f'wz_rmse_N{h}', float('nan')):.4f}"
            for h in horizons
        ]
        print(f"  k_us={kus_val:+.4f}  n_ds={len(ds_results)}  " + "  ".join(parts))

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# クロス確認: 最小点の N 安定性
# ---------------------------------------------------------------------------
def _crosscheck_min_stability(df: pd.DataFrame, horizons: tuple[int, ...]) -> None:
    """各ホライズンの wz_rmse 最小点の k_us 値を表示する。

    N 減少で最小点が 0 に引き戻される → シードアーティファクト残存。
    大 N の最小点と一致                → シードウォッシュアウト完了。

    Note: N=10 のアーティファクト判定には N=20/30/40 の最小点が必要なので、
    Pass 1 で全ホライズンの mins を計算してから Pass 2 で表示する（逐次比較では不可）。
    """
    print("\n[クロス確認] ホライズン別 wz_rmse 最小点:")
    print(f"  {'N':>6}  {'k_us @min':>10}  {'wz_rmse_min':>13}  判定")

    # Pass 1: 全ホライズンの最小点位置を先に収集（Pass 2 でのクロス比較に必要）
    mins: dict[int, float] = {}
    rmse_mins: dict[int, float] = {}
    for h in sorted(horizons):
        col = f"wz_rmse_N{h}"
        if col not in df.columns:
            continue
        valid = df[~df[col].isna() & (df["n_ds"] > 0)]
        if valid.empty:
            continue
        idx_min     = valid[col].idxmin()
        mins[h]     = float(valid.loc[idx_min, "k_us"])
        rmse_mins[h] = float(valid.loc[idx_min, col])

    # Pass 2: 表示（全 mins 利用可能なのでクロス比較が正しく動く）
    large_n_set = {20, 30, 40}
    for h in sorted(horizons):
        if h not in mins:
            print(f"  N={h:3d}  {'N/A':>10}  {'N/A':>13}")
            continue
        kus_min  = mins[h]
        rmse_min = rmse_mins[h]
        artifact_warn = ""
        if h == 10:
            large_n_vals = [mins[hh] for hh in large_n_set if hh in mins]
            if large_n_vals and abs(kus_min) < 0.003 and any(
                abs(lv - kus_min) > 0.003 for lv in large_n_vals
            ):
                artifact_warn = "  ⚠ N=10→0 引き戻し (シードアーティファクト想定通り)"
            else:
                artifact_warn = "  ← N=10 は seeding artifact で 0 寄りになりやすい"
        print(f"  N={h:3d}  {kus_min:>+10.4f}  {rmse_min:>13.5f}{artifact_warn}")

    # N=20/30/40 コンセンサス
    large_n_mins = [mins[h] for h in [20, 30, 40] if h in mins]
    if len(large_n_mins) >= 2:
        spread    = max(large_n_mins) - min(large_n_mins)
        consensus = float(np.mean(large_n_mins))
        print(f"\n  N=20/30/40 最小点の散布: {spread:.4f} rad  (< 0.005 なら安定)")
        print(f"  コンセンサス k_us_eff ≈ {consensus:+.4f} rad  (spread={spread:.4f})")
        if spread < 0.005:
            label = (
                "converter 過補償 (k_us_eff < 0)" if consensus < -0.002
                else ("良好整定 (k_us_eff ≈ 0)" if abs(consensus) <= 0.002
                      else "過小補償 (k_us_eff > 0)")
            )
            print(f"  → {label}")


# ---------------------------------------------------------------------------
# 位置レベルクロスチェック（wz 最小点の独立確認）
# ---------------------------------------------------------------------------
def _pos_score_from_metrics(metrics_by_uuid: dict[str, dict]) -> float:
    """metrics_by_uuid: {uuid: {h: {p14_yaw, p14_lat, bl_yaw, bl_lat}}} → steer_score。

    diagnose_kus_sensitivity.py の _score_from_metrics と同じロジック。
    共通 horizon がない場合は nan を返す。
    """
    per_ds: list[tuple[str, dict]] = []
    bl_map: dict[str, dict] = {}
    for uuid, hmap in metrics_by_uuid.items():
        per_ds.append((uuid, {
            h: {"yaw": v["p14_yaw"], "lat": v["p14_lat"], "long": 0.0}
            for h, v in hmap.items()
        }))
        bl_map[uuid] = {
            h: {"yaw": v["bl_yaw"], "lat": v["bl_lat"], "long": 0.0}
            for h, v in hmap.items()
        }
    if not per_ds:
        return float("nan")
    common_h = tuple(sorted(set.intersection(*[set(m.keys()) for _, m in per_ds])))
    if not common_h:
        return float("nan")
    agg = aggregate_normalized(per_ds, bl_map, horizons=common_h)
    return steer_score(agg, horizons=common_h)


def _sweep_position_score(
    ctxs: list[DatasetCtx],
    base_params: dict,
    kus_sweep: np.ndarray,
    fix_dsf: bool = FIX_DSF,
) -> pd.DataFrame:
    """k_us スイープを位置レベル steer_score でも評価する（wz 最小点との一致確認用）。

    位置レベルは構造的不可同定（直進主体の集約では k_us 勾配≈0）なので「フラット」が予想結果。
    フラット → 不可同定の確認（期待通り）。
    wz 最小点と概ね一致 → 信頼できる k_us_eff（追加根拠）。
    wz=-0.01 vs pos=+0.01 の最小が不一致 → シードリーク等の問題を示唆。

    Override 組み立ては _build_kus_override() で wz スイープと同一（バンディング無効化済み）。
    """
    print(f"\n[位置スコアクロスチェック] k_us スイープ ({len(kus_sweep)} 値 × {len(ctxs)} DS) ...")
    rows = []
    for kus_val in kus_sweep:
        ov = _build_kus_override(base_params, kus_val, fix_dsf)
        metrics_by_uuid: dict[str, dict] = {}
        for ctx in ctxs:
            try:
                p14 = _tune_eval(ctx, ov, _BASELINE_MODEL)
                bl  = ctx.base_metric
                hmap = {
                    int(h): {
                        "p14_yaw": p14[h]["yaw"],
                        "p14_lat": p14[h]["lat"],
                        "bl_yaw":  bl[h]["yaw"],
                        "bl_lat":  bl[h]["lat"],
                    }
                    for h in POS_HORIZONS
                    if h in p14 and h in bl
                }
                if hmap:
                    metrics_by_uuid[ctx.dataset_id] = hmap
            except Exception as e:
                print(f"  [WARN pos] {ctx.dataset_id[:12]}: {e}")

        pos_sc = _pos_score_from_metrics(metrics_by_uuid) if metrics_by_uuid else float("nan")
        rows.append({"k_us": kus_val, "pos_steer_score": pos_sc, "n_ds": len(metrics_by_uuid)})
        print(f"  k_us={kus_val:+.4f}  pos_steer_score={pos_sc:.5f}  n_ds={len(metrics_by_uuid)}")

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# δ_cmd vs δ_actual 乖離分析（クローズドループバイアス bound）
# ---------------------------------------------------------------------------
def _analyze_delta_gap_local(
    ds_list_subset: list[tuple[str, Path]],
    out_csv: Path,
) -> None:
    """post-6/16 高カーブ中の δ_cmd - δ_actual 乖離を速度帯別に集計する。

    k_us 同定をカーブに集中させた分、クローズドループバイアス（δ_cmd vs δ_actual 差）も
    最大化するので、その大きさを bound しておく。
    """
    results = []
    for uuid, lite_dir in ds_list_subset:
        mcap = lite_dir / "real.lite" / "real.lite_0.mcap"
        if not mcap.exists():
            continue
        try:
            df_kin   = load_kinematic(mcap)
            df_steer = load_steering(mcap)
            df_cmd   = _load_steer_cmd(mcap)
            if df_kin.empty or df_steer.empty or df_cmd is None or df_cmd.empty:
                continue

            t_k   = df_kin["t_ns"].values * 1e-9
            vx    = df_kin["vx"].values
            wz    = df_kin["wz"].values
            kappa = np.where(vx > 0.5, np.abs(wz / vx), 0.0)

            t_s          = df_steer["t_ns"].values * 1e-9
            delta_actual = np.interp(t_k, t_s, df_steer["steer"].values)
            t_c          = df_cmd["t_ns"].values * 1e-9
            delta_cmd    = np.interp(t_k, t_c, df_cmd["steer"].values)

            for vx_lo, vx_hi in [(1.0, 2.71), (2.71, 5.91), (5.91, 20.0)]:
                mask = (vx >= vx_lo) & (vx < vx_hi) & (kappa > KAPPA_THRESH)
                if mask.sum() < 5:
                    continue
                diff = delta_cmd[mask] - delta_actual[mask]
                results.append({
                    "uuid":         uuid[:12],
                    "vx_band":      f"{vx_lo:.1f}-{vx_hi:.1f}",
                    "n":            int(mask.sum()),
                    "diff_mean":    float(diff.mean()),
                    "diff_p50":     float(np.median(diff)),
                    "diff_abs_p90": float(np.percentile(np.abs(diff), 90)),
                })
        except Exception as e:
            print(f"  [WARN] {uuid[:12]}: {e}")

    if not results:
        print("  ⚠ δ_cmd トピックを取得できませんでした")
        return

    df_gap = pd.DataFrame(results)
    for band, grp in df_gap.groupby("vx_band"):
        print(f"\n  速度帯 {band} m/s (カーブ中, κ>{KAPPA_THRESH}):")
        print(f"    DS 数: {len(grp)}")
        print(f"    δ_cmd - δ_actual  mean={grp['diff_mean'].mean()*1000:.2f} mrad  "
              f"p50={grp['diff_p50'].median()*1000:.2f} mrad  "
              f"|diff| p90={grp['diff_abs_p90'].median()*1000:.2f} mrad")
        pos_frac = (grp["diff_mean"] > 0.001).mean()
        print(f"    δ_cmd > δ_actual の DS 割合: {pos_frac:.1%}")
        if band == "2.71-5.91" or band == "5.91-20.0":
            # understeer 補正が有効な速度帯: diff > 0 ならコントローラがオーバーコマンドしている
            note = "← understeer 補正で δ_cmd 増幅されている可能性"
            print(f"    {note}")

    df_gap.to_csv(out_csv, index=False)
    print(f"\n  δ乖離 CSV: {out_csv}")


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def main() -> None:
    # パラメータ読み込み
    with open(PARAMS_YAML) as f:
        yaml_data = yaml.safe_load(f)
    base_params = {
        k: v
        for k, v in yaml_data.get("params", yaml_data).items()
        if not k.startswith("_")
    }

    print("=" * 70)
    print("k_us_eff yaw レート層同定 (post-2026-06-16, 曲率サブセット)")
    print("=" * 70)
    print(f"基準パラメータ: {PARAMS_YAML.name}")
    print(f"  k_us          = {base_params.get('k_us', 'N/A')}")
    dsf = base_params.get("debug_steer_scaling_factor", 1.0)
    sb  = base_params.get("steer_bias", 0.0)
    print(f"  DSF           = {dsf:.5f}  (中立=1.0, 差={dsf-1.0:+.4f})")
    print(f"  steer_bias    = {sb:.6f} rad")
    print(f"FIX_DSF={FIX_DSF}: {'DSF=1.0/steer_bias=0.0 に固定' if FIX_DSF else 'base_params のまま'}")
    if FIX_DSF and abs(dsf - 1.0) > 0.01:
        print(f"  ⚠ Phase48 DSF={dsf:.4f} は 4.9% 偏位あり → FIX_DSF=True で吸収バイアスを排除")

    # DS 発見 + post-6/16 日付フィルタ
    ds_list = _discover(COLLECTION_DIR)
    print(f"\n全データセット: {len(ds_list)} 件")
    ds_list = _filter_by_date(ds_list, before=None, after=DS_AFTER)
    print(f"post-{DS_AFTER} フィルタ後: {len(ds_list)} 件\n")

    if not ds_list:
        print("[ERROR] post-6/16 DS が 0 件です。COLLECTION_DIR を確認してください。")
        return

    # --- 高速カーブ割合でランク付け ---
    print(f"[Step 1] 高速カーブ割合を計算 (vx>{VX_THRESH} m/s かつ κ>{KAPPA_THRESH} 1/m) ...")
    curve_fracs: dict[str, float] = {}
    for i, (uuid, lite_dir) in enumerate(ds_list):
        curve_fracs[uuid] = _compute_curve_fraction(lite_dir, VX_THRESH, KAPPA_THRESH)
        if (i + 1) % 50 == 0:
            print(f"  {i+1}/{len(ds_list)} 処理済み")

    sorted_ds = sorted(ds_list, key=lambda x: curve_fracs[x[0]], reverse=True)
    curve_ds  = sorted_ds[:N_CURVE_DS]

    fracs = np.array(list(curve_fracs.values()))
    print(f"\n  高速カーブ割合 (post-6/16 DS, n={len(ds_list)}):")
    print(f"    p50={np.median(fracs):.4f}  "
          f"p90={np.percentile(fracs, 90):.4f}  "
          f"max={fracs.max():.4f}")
    print(f"    > 0.01: {(fracs > 0.01).sum()} 件  "
          f"> 0.05: {(fracs > 0.05).sum()} 件  "
          f"> 0.10: {(fracs > 0.10).sum()} 件")
    print(f"\n  高速カーブ上位 {N_CURVE_DS} DS (先頭10件):")
    for uuid, _ in curve_ds[:10]:
        print(f"    {uuid[:12]}  curve_frac={curve_fracs[uuid]:.3f}")

    # --- DatasetCtx ロード ---
    print(f"\n[Step 2] DatasetCtx ロード ({N_CURVE_DS} DS, n_jobs={N_JOBS}) ...")
    ctxs = load_datasets(curve_ds, n_jobs=N_JOBS)
    print(f"  ロード完了: {len(ctxs)} DS")

    if not ctxs:
        print("[ERROR] ロード可能な DS が 0 件です")
        return

    # --- 2-point プローブ: override が calc_yaw_rate に届くか事前確認 ---
    ok = _probe_override(ctxs, base_params, FIX_DSF)
    if not ok:
        print("\n[ABORT] プローブが FLAT を検出しました。banding 無効化キーを確認してください。")
        print("  _build_kus_override() と step5._build_params() の k_us_bands/k_us_thresholds を確認してください。")
        return

    # --- Step 3: k_us スイープ（yaw レート層）---
    print(f"\n[Step 3] k_us スイープ ({len(KUS_SWEEP)} 値 × {len(ctxs)} DS)")
    print(f"  horizons={WZ_HORIZONS}  stride={STRIDE}  fix_dsf={FIX_DSF}")
    print(f"  ※ N=10 はシードアーティファクト確認用、N=20/30/40 が主評価\n")

    df_sweep = _sweep_kus_wz(ctxs, base_params, KUS_SWEEP, WZ_HORIZONS, STRIDE, FIX_DSF)
    df_sweep.to_csv(OUT_CSV, index=False)
    print(f"\n結果 CSV: {OUT_CSV}")

    # --- 最小点の N 安定性クロス確認 ---
    _crosscheck_min_stability(df_sweep, WZ_HORIZONS)

    # --- 感度サマリ ---
    print("\n[感度サマリ] 各ホライズンの wz_rmse 範囲（広いほど k_us 感度あり）:")
    for h in sorted(WZ_HORIZONS):
        col = f"wz_rmse_N{h}"
        if col not in df_sweep.columns:
            continue
        valid = df_sweep[~df_sweep[col].isna()]
        if valid.empty:
            continue
        span = float(valid[col].max() - valid[col].min())
        rmse_at_zero = (
            df_sweep.loc[df_sweep["k_us"].abs().idxmin(), col]
            if not df_sweep.empty else float("nan")
        )
        print(f"  N={h:3d}: rmse 範囲={span:.5f} rad/s  rmse@k_us≈0={rmse_at_zero:.5f}")

    # --- Step 3b: 位置レベルクロスチェック（独立した最小点確認）---
    pos_csv = OUT_DIR / "kus_wz_pos_crosscheck.csv"
    df_pos = _sweep_position_score(ctxs, base_params, KUS_SWEEP, FIX_DSF)
    df_pos.to_csv(pos_csv, index=False)

    # wz 最小点 vs 位置最小点の比較サマリ
    print("\n[クロスチェックサマリ] wz(N=40) vs 位置レベルの k_us 最小点:")
    for label, df_v, col in [
        ("wz N=40", df_sweep, "wz_rmse_N40"),
        ("pos score", df_pos, "pos_steer_score"),
    ]:
        if col not in df_v.columns:
            continue
        valid = df_v[~df_v[col].isna() & (df_v["n_ds"] > 0)]
        if valid.empty:
            continue
        span = float(valid[col].max() - valid[col].min())
        kmin = float(valid.loc[valid[col].idxmin(), "k_us"])
        flat_note = " (フラット = 位置は構造的不可同定、期待通り)" if span < 1e-4 else ""
        print(f"  {label:12s}: 最小点 k_us={kmin:+.4f}  範囲={span:.5f}{flat_note}")

    # --- Step 4: δ_cmd vs δ_actual 乖離分析 ---
    print(f"\n[Step 4] δ_cmd vs δ_actual 乖離分析 ({len(curve_ds)} DS) ...")
    delta_gap_csv = OUT_DIR / "kus_wz_delta_gap.csv"
    _analyze_delta_gap_local(curve_ds, delta_gap_csv)

    print("\n" + "=" * 70)
    print("完了。")
    print(f"  主結果 CSV:        {OUT_CSV}")
    print(f"  位置クロスチェック: {pos_csv}")
    print(f"  δ乖離 CSV:         {delta_gap_csv}")
    print("\n次のステップ:")
    print("  1. kus_wz_id_sweep.csv で N=20/30/40 の wz_rmse 最小点を確認")
    print("  2. kus_wz_pos_crosscheck.csv の最小点と一致するか確認（不一致はシードリーク等を示唆）")
    print("  3. クロス確認: N=10→0, N=40→別値 ならシードウォッシュアウト済み → 大 N 採用")
    print("  4. k_us_eff の符号: < 0 = converter 過補償, ≈ 0 = 良好整定, > 0 = 過小補償")
    print("  5. k_us_eff を multi_dataset_tune.py Phase 47 に反映し position score で確認")


if __name__ == "__main__":
    main()
