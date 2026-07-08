#!/usr/bin/env python3
import argparse
import sys
import glob
import yaml
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

# 依存パッケージロード用のパス追加
# workspace の install dist-packages ディレクトリを走査してロード可能にする
for _p in glob.glob("/home/kotaroyoshimoto/workspace/x2_e2e_44/install/*/local/lib/python3.10/dist-packages"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from driving_log_replayer_v2.real_log_sim_comparison.lib._fit_core import (
    fit_first_order_delay,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._parallel import (
    default_parallel_jobs,
    normalize_parallel_jobs,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import (
    load_accel,
    load_cmd,
    load_gear_status,
    load_steering,
    load_velocity,
    load_kinematic,
    require_drive_gear_mask,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._validation import (
    require_non_empty_df,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import (
    ACCEL_SOURCE,
    ACCEL_DELAY_MAP,
)
import pandas as pd

# 物理定数の SSOT は lib._physical_validity。チューニング(本モジュール)と検証
# (physical_validity_report) で同じ車両ジオメトリ・同定条件を使うため、そこから import する。
# k_us の定常旋回フィルタ (VX_MIN_CURVE / WZ_MIN / DWZ_MAX) は「同定した母集団」と
# 「検証する母集団」を一致させるため必ず共有する (かつては tuning=(0.5,0.01,0.1) と
# validation=(1.5,0.02,0.30) で別母集団を見ていた)。
from driving_log_replayer_v2.real_log_sim_comparison.lib._physical_validity import (
    WHEELBASE as WHEELBASE_SSOT,
    VX_MIN_CURVE,
    WZ_MIN,
    DWZ_MAX,
)

CMD_TOPIC = "/control/command/control_cmd"
DT = 0.01           # 10msサンプリング (本モジュール固有: 遅延グリッド分解能を上げるため
                    # 検証側 _physical_validity の 1/30 とは意図的に異なる。A-2 参照)
# 縦/操舵フィットの動的マスク閾値。_physical_validity の _VX_MIN_FIT / _DA_THRESH_FIT /
# _DSTEER_MIN と同値 (同一役割・現状一致)。将来ドリフトさせないこと。
VX_MIN = 0.5
DA_THRESH = 0.15
DSTEER_MIN = 0.001
# K_US_CLIP: 本モジュールでは「速度ビン別 OLS 推定値 k_val」のクリップ (per-bin, sim へ渡す
# パラメータの安全上限)。_physical_validity の同名 K_US_CLIP=0.5 は「per-sample の IQR 表示用
# クリップ」で役割・粒度が異なるため意図的に別値。再チューニング後、k_us がこの 0.05 上限に
# 張り付く場合は妥当性を再検討すること (wheelbase 変更で k_val の大きさが変わるため)。
K_US_CLIP = 0.05
# 車両ホイールベース [m]。権威ある源は j6_gen2_description/config/vehicle_info.param.yaml
# (= 4.76012)。ここではその値を再エクスポートする _physical_validity の SSOT を既定値とする。
# scenario に明示があれば main() でそれを優先する。
# 注意: wheelbase は「同定して反映する値」ではなく固定ジオメトリ。tuning 出力を源にして
# 循環させないこと (常に車両記述由来の値を使う)。
WHEELBASE = WHEELBASE_SSOT

# 縦/操舵フィットの遅延グリッド (production 固有: DT=0.01 刻みで検証側 1/30 より細かい)。
# 一次遅れ + 純粋遅延の同定本体は lib._fit_core.fit_first_order_delay (SSOT) に集約。
DELAY_GRID_LONG = np.arange(0.0, 0.31, 0.01)
DELAY_GRID_STEER = np.arange(0.0, 0.16, 0.01)
TAU_BOUNDS_LONG = (0.01, 5.0)
TAU_BOUNDS_STEER = (0.01, 2.0)


def _load_light_worker(args: tuple) -> dict | None:
    uuid, ds_dir = args
    mcap = Path(ds_dir) / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None
    df_cmd = load_cmd(mcap, CMD_TOPIC)
    df_accel = load_accel(mcap)
    df_steer = load_steering(mcap)
    df_vel = load_velocity(mcap)
    df_kin = load_kinematic(mcap)
    df_gear = load_gear_status(mcap)
    context = f"physical_tuning:{uuid}"
    require_non_empty_df(df_cmd, name="/control/command/control_cmd", context=context)
    require_non_empty_df(df_accel, name="/localization/acceleration", context=context)
    require_non_empty_df(df_steer, name="/vehicle/status/steering_status", context=context)
    require_non_empty_df(df_vel, name="/vehicle/status/velocity_status", context=context)
    require_non_empty_df(df_kin, name="/localization/kinematic_state", context=context)

    t0 = max(df_cmd["t_ns"].iloc[0], df_accel["t_ns"].iloc[0], df_steer["t_ns"].iloc[0], df_vel["t_ns"].iloc[0], df_kin["t_ns"].iloc[0])
    t1 = min(df_cmd["t_ns"].iloc[-1], df_accel["t_ns"].iloc[-1], df_steer["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1], df_kin["t_ns"].iloc[-1])
    if (t1 - t0) < 2e9:
        return None

    t_ns = np.arange(t0, t1, DT * 1e9, dtype=np.float64)
    t_s = (t_ns - t0) * 1e-9

    a_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_accel"].values)
    
    if ACCEL_SOURCE == "accel":
        accel_t_ns = df_accel["t_ns"].values
        accel_val = df_accel["accel"].values
    elif ACCEL_SOURCE == "kinematic_diff":
        t_s_kin = df_kin["t_ns"].values * 1e-9
        vx = df_kin["vx"].values
        dt = np.diff(t_s_kin)
        dv = np.diff(vx)
        raw_accel = np.zeros_like(vx)
        raw_accel[1:] = dv / np.maximum(dt, 1e-6)
        smooth_accel = pd.Series(raw_accel).rolling(window=10, min_periods=1, center=True).mean().values
        accel_t_ns = df_kin["t_ns"].values
        accel_val = smooth_accel
    elif ACCEL_SOURCE == "velocity_diff":
        t_s_vel = df_vel["t_ns"].values * 1e-9
        lon_vel = df_vel["lon_vel"].values
        dt = np.diff(t_s_vel)
        dv = np.diff(lon_vel)
        raw_accel = np.zeros_like(lon_vel)
        raw_accel[1:] = dv / np.maximum(dt, 1e-6)
        smooth_accel = pd.Series(raw_accel).rolling(window=10, min_periods=1, center=True).mean().values
        accel_t_ns = df_vel["t_ns"].values
        accel_val = smooth_accel
    else:
        raise ValueError(f"Unknown ACCEL_SOURCE: {ACCEL_SOURCE}")

    a_act = np.interp(t_s, (accel_t_ns - t0) * 1e-9, accel_val)

    d_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_steer"].values)
    d_act = np.interp(t_s, (df_steer["t_ns"].values - t0) * 1e-9, df_steer["steer"].values)
    vx = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)
    wz = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["wz"].values)
    gear_drive = require_drive_gear_mask(
        df_gear,
        t_ns.astype(np.int64),
        context=context,
        allow_leading_gap=True,
    )

    return {
        "uuid": uuid,
        "a_cmd": a_cmd.astype(np.float32),
        "a_act": a_act.astype(np.float32),
        "d_cmd": d_cmd.astype(np.float32),
        "d_act": d_act.astype(np.float32),
        "vx": vx.astype(np.float32),
        "wz": wz.astype(np.float32),
        "gear_drive": gear_drive,
    }

def _long_mask(ds: dict) -> np.ndarray:
    """縦方向同定の動的マスク: DRIVE 中 & vx>VX_MIN & 指令加速度変化が大きい区間。"""
    d_cmd_acc = np.abs(np.gradient(ds["a_cmd"], DT))
    return ds["gear_drive"] & (ds["vx"] > VX_MIN) & (d_cmd_acc > DA_THRESH)


def _fit_one_long_worker(ds: dict) -> tuple[float, float, float] | None:
    a_cmd = ds["a_cmd"]
    a_act = ds["a_act"]
    mask = _long_mask(ds)
    if mask.sum() < 50:
        return None

    # 遅延グリッド × log-τ 同定 + 射影スケール (debug_acc_scaling_factor) = 共通カーネル。
    fit = fit_first_order_delay(
        a_cmd, a_act, mask, DT,
        tau_bounds=TAU_BOUNDS_LONG, delay_candidates=DELAY_GRID_LONG, fit_scale=True,
    )
    if fit is None:
        return None
    corrected_delay = max(0.0, fit["delay"] - ACCEL_DELAY_MAP.get(ACCEL_SOURCE, 0.0))
    return fit["tau"], corrected_delay, fit["scale"]


def _fit_one_steer_worker(ds: dict) -> tuple[float, float, float, float] | None:
    d_cmd = ds["d_cmd"]
    d_act = ds["d_act"]
    vx = ds["vx"]
    wz = ds["wz"]
    gear_drive = ds["gear_drive"]
    d_cmd_steer = np.abs(np.gradient(d_cmd, DT))
    mask_dyn = gear_drive & (vx > VX_MIN) & (d_cmd_steer > DSTEER_MIN / DT)
    if mask_dyn.sum() < 50:
        return None

    # bias (直進区間の実操舵中点) と dsf (射影スケール) は操舵 production 固有の前処理。
    mask_straight = gear_drive & (vx > 3.0) & (np.abs(wz) < 0.005)
    bias = float(np.mean(d_act[mask_straight])) if mask_straight.sum() > 20 else 0.0005

    d_act_nobias = d_act - bias
    sum_cmd2 = np.sum(d_cmd[mask_dyn] ** 2)
    dsf = float(np.sum(d_cmd[mask_dyn] * d_act_nobias[mask_dyn]) / sum_cmd2) if sum_cmd2 > 1e-5 else 1.0
    dsf = float(np.clip(dsf, 0.8, 1.2))

    # dsf を cmd に前掛けし、bias 除去済み実操舵に対して τ・遅延を同定 (共通カーネル、scale なし)。
    fit = fit_first_order_delay(
        dsf * d_cmd, d_act_nobias, mask_dyn, DT,
        tau_bounds=TAU_BOUNDS_STEER, delay_candidates=DELAY_GRID_STEER,
    )
    if fit is None:
        return None
    return fit["tau"], fit["delay"], bias, dsf

def main() -> None:
    ap = argparse.ArgumentParser(description="物理直接同定チューニング")
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--phase", type=int, choices=[1, 2, 12], default=2,
                    help="1: 縦方向のみ, 2: 縦＋横＋アンダーステア, 12: 両フェーズ一括（ロード1回）")
    ap.add_argument("--phase-params", type=str, default="", help="前フェーズの YAML パス")
    ap.add_argument("--out", type=Path, required=True,
                    help="出力 YAML パス（--phase 12 の場合は Phase2 出力）")
    ap.add_argument("--out-phase1", type=Path, default=None,
                    help="--phase 12 のときの Phase1 出力 YAML パス（省略時は --out と同じディレクトリに phase1_acc.yaml）")
    ap.add_argument("--n-jobs", type=int, default=default_parallel_jobs())
    ap.add_argument("--case", type=str, default="current",
                    help="ベースとなる scenario.yaml 内の models エントリ名 (既定: current)")
    ap.add_argument("--skip-lon", action="store_true", help="縦方向モデルの直接同定をスキップし、scenario から取得した値を設定する")
    ap.add_argument("--skip-steer", action="store_true", help="操舵・横方向モデルの直接同定をスキップし、scenario から取得した値を設定する")
    args = ap.parse_args()

    # scenario.yaml から wheelbase およびパラメータ群をロード
    global WHEELBASE
    wb_found = None
    scenario_params = {}
    try:
        with args.scenario.open("r") as f:
            scen = yaml.safe_load(f)
            models = scen.get("Evaluation", {}).get("Conditions", {}).get("models", {})
            for case_key in [args.case, "current", "best_normal", "case_normal", "normal", "baseline"]:
                if case_key in models:
                    wb = models[case_key].get("wheelbase")
                    if wb:
                        wb_found = float(wb)
                        break
            case_data = models.get(args.case, {})
            scenario_params = case_data.get("params", {})
            print(f"[INFO] scenario.yaml の '{args.case}' からパラメータを取得しました: {list(scenario_params.keys())}")
    except Exception as e:
        print(f"[WARN] scenario.yaml のパースまたはパラメータ取得に失敗しました: {e}")

    if wb_found is not None:
        WHEELBASE = wb_found
        print(f"[INFO] scenario.yaml から wheelbase を取得しました: {WHEELBASE}")
    else:
        # サイレントフォールバック禁止: SSOT 既定値を使うことを必ず明示する。
        print(f"[WARN] scenario.yaml に wheelbase の記載がないため、SSOT 既定値を使用します: "
              f"{WHEELBASE} (source: lib._physical_validity.WHEELBASE = j6_gen2_description)")

    # 前フェーズのパラメータの読み込み（継承用）
    inherited_params = {}
    if args.phase_params and Path(args.phase_params).exists():
        try:
            with Path(args.phase_params).open("r") as f:
                inherited_data = yaml.safe_load(f)
                inherited_params = inherited_data.get("params", inherited_data)
                print(f"[INFO] 前フェーズのパラメータを引き継ぎました: {list(inherited_params.keys())}")
        except Exception as e:
            print(f"[WARN] phase-params の読み込みに失敗しました: {e}")

    # データセットスキャン
    ds_root = args.collection_dir / "datasets"
    if not ds_root.exists():
        ds_root = args.collection_dir
    ds_dirs = sorted(ds_root.iterdir()) if ds_root.is_dir() else []
    tasks = [(d.name, str(d)) for d in ds_dirs if (d / "real.lite" / "real.lite_0.mcap").exists()]
    phase_label = "1+2" if args.phase == 12 else str(args.phase)
    args.n_jobs = normalize_parallel_jobs(args.n_jobs)
    print(f"\n[Phase {phase_label}] データセット並列ロード ({len(tasks)} 件)...")

    datasets = []
    n_workers = normalize_parallel_jobs(args.n_jobs, n_tasks=len(tasks))
    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        futs = {pool.submit(_load_light_worker, t): t for t in tasks}
        for i, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            if r is not None:
                datasets.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(tasks)} 完了", flush=True)

    print(f"有効データセット数: {len(datasets)}")
    if not datasets:
        print("ERROR: 有効なデータセットが 0 件です。")
        sys.exit(1)

    params = inherited_params.copy()

    # --phase 12 の場合: Phase1 同定 → phase1 YAML 出力 → Phase2 同定 → phase2 YAML 出力
    # をこのプロセス内で連続実行し、データセットロードは1回だけで済ませる。

    # 1. 縦方向モデルパラメータの直接同定 (Phase 1 または Phase 2 で必要)
    if args.skip_lon:
        print("\n=== 縦方向モデルの直接同定をスキップ（元の値を引き継ぎ） ===")
        params["acc_time_constant"] = float(scenario_params.get("acc_time_constant", 0.30))
        params["acc_time_delay"] = float(scenario_params.get("acc_time_delay", 0.1))
        params["debug_acc_scaling_factor"] = float(scenario_params.get("debug_acc_scaling_factor", 1.0))
        print(f"  設定パラメータ: acc_time_constant = {params['acc_time_constant']:.4f} s")
        print(f"                  acc_time_delay    = {params['acc_time_delay']:.4f} s")
        print(f"                  acc_scaling       = {params['debug_acc_scaling_factor']:.4f}")
    elif args.phase in (1, 12) or "acc_time_constant" not in params:
        print("\n=== 縦方向モデルの直接同定を実行中 ===")
        taus_long = []
        delays_long = []
        asfs_long = []
        n_workers = normalize_parallel_jobs(args.n_jobs, n_tasks=len(datasets))
        with ProcessPoolExecutor(max_workers=n_workers) as pool:
            futs = {pool.submit(_fit_one_long_worker, ds): ds["uuid"] for ds in datasets}
            for fut in as_completed(futs):
                res = fut.result()
                if res is not None:
                    tau, delay, asf = res
                    taus_long.append(tau)
                    delays_long.append(delay)
                    asfs_long.append(asf)

        if taus_long:
            # τ / delay / scale はいずれも per-dataset median を採る。縦の drag(v) 多項式・throttle/brake
            # 非対称 τ は Step 3 で pooled 同定を試作したが、c0=0 制約下で aero-only (c2·v²) の寄与は
            # ~0%、改善は c1>0 の非物理な bias 誤当てはめ由来と判明したため撤去 (実測ログ参照)。
            params["acc_time_constant"] = float(np.clip(np.median(taus_long), 0.1, 3.0))
            params["acc_time_delay"] = float(np.clip(np.median(delays_long), 0.0, 0.3))
            params["debug_acc_scaling_factor"] = float(np.clip(np.median(asfs_long), 0.8, 1.2))
            print(f"  同定結果: acc_time_constant = {params['acc_time_constant']:.4f} s")
            print(f"            acc_time_delay    = {params['acc_time_delay']:.4f} s")
            print(f"            acc_scaling       = {params['debug_acc_scaling_factor']:.4f}")
        else:
            print("[WARN] 縦方向の動的条件を満たすデータがないため、デフォルト値を設定します。")
            params["acc_time_constant"] = 0.2
            params["acc_time_delay"] = 0.1
            params["debug_acc_scaling_factor"] = 1.0

    # --phase 12 の場合: Phase1 結果を YAML として中間保存してから Phase2 へ進む
    if args.phase == 12:
        out_phase1 = args.out_phase1 or (args.out.parent / "phase1_acc.yaml")
        out_phase1.parent.mkdir(parents=True, exist_ok=True)
        yaml_phase1 = {
            "params": params.copy(),
            "score": 0.0,
            "metadata": {
                "collection_dir": str(args.collection_dir),
                "n_datasets": len(tasks),
                "n_valid": len(datasets),
                "scenario": str(args.scenario),
                "tuning_type": "physical_direct_fit",
                "phase": 1,
            }
        }
        with out_phase1.open("w") as f:
            yaml.safe_dump(yaml_phase1, f, allow_unicode=True, sort_keys=False)
        print(f"\n✓ Phase1 中間パラメータ保存完了: {out_phase1}")

    # 2. 操舵およびアンダーステア勾配の直接同定 (Phase 2)
    if args.phase in (2, 12):
        if args.skip_steer:
            print("\n=== 操舵モデルおよびアンダーステアの直接同定をスキップ（元の値を引き継ぎ） ===")
            params["steer_time_constant"] = float(scenario_params.get("steer_time_constant", 0.15))
            params["steer_time_delay"] = float(scenario_params.get("steer_time_delay", 0.17))
            params["steer_bias"] = float(scenario_params.get("steer_bias", 0.0))
            params["debug_steer_scaling_factor"] = float(scenario_params.get("debug_steer_scaling_factor", 1.0))
            params["steer_dead_band"] = float(scenario_params.get("steer_dead_band", 0.0))
            params["steer_rate_lim"] = float(scenario_params.get("steer_rate_lim", 5.0))
            params["k_us"] = float(scenario_params.get("k_us", 0.018))
            print(f"  設定パラメータ: steer_time_constant = {params['steer_time_constant']:.4f} s")
            print(f"                  steer_time_delay    = {params['steer_time_delay']:.4f} s")
            print(f"                  steer_bias          = {params['steer_bias']:.6f} rad")
            print(f"                  steer_scaling       = {params['debug_steer_scaling_factor']:.4f}")
            print(f"                  k_us                = {params['k_us']:.5f}")
        else:
            print("\n=== 操舵モデルおよびアンダーステアの直接同定を実行中 ===")
            taus_steer = []
            delays_steer = []
            biases = []
            dsfs = []
            n_workers = normalize_parallel_jobs(args.n_jobs, n_tasks=len(datasets))
            with ProcessPoolExecutor(max_workers=n_workers) as pool:
                futs = {pool.submit(_fit_one_steer_worker, ds): ds["uuid"] for ds in datasets}
                for fut in as_completed(futs):
                    res = fut.result()
                    if res is not None:
                        tau, delay, bias, dsf = res
                        taus_steer.append(tau)
                        delays_steer.append(delay)
                        biases.append(bias)
                        dsfs.append(dsf)

            if taus_steer:
                params["steer_time_constant"] = float(np.clip(np.median(taus_steer), 0.05, 0.8))
                params["steer_time_delay"] = float(np.clip(np.median(delays_steer), 0.0, 0.15))
                params["steer_bias"] = float(np.median(biases))
                params["debug_steer_scaling_factor"] = float(np.median(dsfs))
                params["steer_dead_band"] = 0.001  # 固定初期値
                params["steer_rate_lim"] = 5.0      # 固定値
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

            # 3. アンダーステア勾配 (k_us) の速度依存プロファイル同定
            print("\n=== アンダーステア勾配 (k_us) の速度ビン別直接同定を実行中 ===")
            # 各データセットから定常旋回フィルタを通した全体点群を作成
            all_vx, all_wz, all_steer, all_dwz = [], [], [], []
            for ds in datasets:
                vx = ds["vx"]
                wz = ds["wz"]
                d_act = ds["d_act"]
                gear_drive = ds["gear_drive"]
                # ヨー角加速度の算出
                dwz_mid = np.diff(wz) / DT
                dwz = np.empty_like(wz)
                dwz[0] = dwz_mid[0] if len(dwz_mid) > 0 else 0.0
                dwz[-1] = dwz_mid[-1] if len(dwz_mid) > 0 else 0.0
                dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])

                all_vx.append(vx)
                all_wz.append(wz)
                all_steer.append(d_act)
                all_dwz.append(dwz)
                # 非 DRIVE 区間は後段の定常旋回フィルタに入れない。
                ds["gear_drive"] = gear_drive

            vx_all = np.concatenate(all_vx)
            wz_all = np.concatenate(all_wz)
            steer_all = np.concatenate(all_steer)
            dwz_all = np.concatenate(all_dwz)
            gear_all = np.concatenate([ds["gear_drive"] for ds in datasets])

            # 定常旋回フィルタ
            mask_ok = gear_all & (np.abs(wz_all) > WZ_MIN) & (np.abs(dwz_all) < DWZ_MAX) & (vx_all > VX_MIN_CURVE)
            vx_f = vx_all[mask_ok]
            wz_f = wz_all[mask_ok]
            steer_f = steer_all[mask_ok] - params["steer_bias"]
            tan_steer = np.tan(np.clip(steer_f, -0.8, 0.8))

            # スカラー原点回帰 y = k_us·x,  x = v·ω,  y = tan(δ) − L·ω/v
            x = vx_f * wz_f
            y = tan_steer - WHEELBASE * wz_f / vx_f if len(vx_f) else np.empty(0)
            sum_x2 = float(np.sum(x * x))
            sum_xy = float(np.sum(x * y))
            n_pts = int(len(vx_f))
            k_us = sum_xy / sum_x2 if (n_pts >= 10 and sum_x2 > 0) else 0.008
            # 下限クリップ (アンダーステア補正として正の値を担保)
            k_us = float(np.clip(k_us, 0.0, K_US_CLIP))

            params["k_us"] = k_us
            print(f"  同定結果: k_us = {k_us:.5f} (曲線走行サンプル n={n_pts})")

    # 結果を YAML 出力
    out_dir = args.out.parent
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    # multi_dataset_tune と同様の yaml_data 構造にする
    yaml_data = {
        "params": params,
        "score": 0.0,  # ダミープレースホルダー
        "metadata": {
            "collection_dir": str(args.collection_dir),
            "n_datasets": len(tasks),
            "n_valid": len(datasets),
            "scenario": str(args.scenario),
            "tuning_type": "physical_direct_fit",
            "phase": args.phase,
        }
    }
    with args.out.open("w") as f:
        yaml.safe_dump(yaml_data, f, allow_unicode=True, sort_keys=False)
    print(f"\n✓ パラメータ保存完了: {args.out}")

if __name__ == "__main__":
    main()
