#!/usr/bin/env python3
import argparse
import sys
import os
import glob
import yaml
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter

# 依存パッケージロード用のパス追加
# workspace の install dist-packages ディレクトリを走査してロード可能にする
for _p in glob.glob("/home/kotaroyoshimoto/workspace/x2_e2e_44/install/*/local/lib/python3.10/dist-packages"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from driving_log_replayer_v2.real_log_sim_comparison.lib._io import (
    load_accel,
    load_cmd,
    load_steering,
    load_velocity,
    load_kinematic,
)

CMD_TOPIC = "/control/command/control_cmd"
DT = 0.01           # 10msサンプリング
VX_MIN = 0.5
DA_THRESH = 0.15
DSTEER_MIN = 0.001
WZ_MIN = 0.01
DWZ_MAX = 0.1
VX_MIN_CURVE = 0.5
K_US_CLIP = 0.05
WHEELBASE = 2.74  # x2 の標準値。scenario.yaml から動的取得を試みる

VX_EDGES = np.array([0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0])

def _simulate_first_order(cmd: np.ndarray, tau: float, n_delay: int) -> np.ndarray:
    n = len(cmd)
    del_arr = np.empty(n)
    if n_delay > 0:
        del_arr[:n_delay] = cmd[0]
        del_arr[n_delay:] = cmd[:-n_delay]
    else:
        del_arr = cmd.copy()
    alpha = float(np.clip(DT / tau, 0.0, 1.0))
    return lfilter([alpha], [1.0, -(1.0 - alpha)], del_arr)

def _fit_tau_nls_long(a_cmd: np.ndarray, a_act: np.ndarray, n_delay: int, mask: np.ndarray) -> tuple[float, float, float]:
    def _mse_and_asf(log_tau: float) -> tuple[float, float]:
        tau = float(np.exp(log_tau))
        a_sim_base = _simulate_first_order(a_cmd, tau, n_delay)
        sum_sim2 = np.sum(a_sim_base[mask] ** 2)
        asf = float(np.sum(a_sim_base[mask] * a_act[mask]) / sum_sim2) if sum_sim2 > 1e-5 else 1.0
        asf = float(np.clip(asf, 0.8, 1.2))
        a_sim = asf * a_sim_base
        mse = float(np.mean((a_sim[mask] - a_act[mask]) ** 2))
        return mse, asf

    def _objective(log_tau: float) -> float:
        return _mse_and_asf(log_tau)[0]

    res = minimize_scalar(_objective, bounds=(np.log(0.01), np.log(5.0)), method="bounded")
    tau = float(np.exp(res.x))
    _, asf = _mse_and_asf(res.x)
    return tau, asf, _objective(res.x)

def _fit_tau_nls_steer(d_cmd: np.ndarray, d_act: np.ndarray, n_delay: int, mask: np.ndarray) -> tuple[float, float]:
    def _mse(log_tau: float) -> float:
        tau = float(np.exp(log_tau))
        d_sim = _simulate_first_order(d_cmd, tau, n_delay)
        return float(np.mean((d_sim[mask] - d_act[mask]) ** 2))
    res = minimize_scalar(_mse, bounds=(np.log(0.01), np.log(2.0)), method="bounded")
    tau = float(np.exp(res.x))
    return tau, float(res.fun)

def _load_light_worker(args: tuple) -> dict | None:
    uuid, ds_dir = args
    mcap = Path(ds_dir) / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None
    try:
        df_cmd = load_cmd(mcap, CMD_TOPIC)
        df_accel = load_accel(mcap)
        df_steer = load_steering(mcap)
        df_vel = load_velocity(mcap)
        df_kin = load_kinematic(mcap)
    except Exception:
        return None
    if df_cmd.empty or df_accel.empty or df_steer.empty or df_vel.empty or df_kin.empty:
        return None

    t0 = max(df_cmd["t_ns"].iloc[0], df_accel["t_ns"].iloc[0], df_steer["t_ns"].iloc[0], df_vel["t_ns"].iloc[0], df_kin["t_ns"].iloc[0])
    t1 = min(df_cmd["t_ns"].iloc[-1], df_accel["t_ns"].iloc[-1], df_steer["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1], df_kin["t_ns"].iloc[-1])
    if (t1 - t0) < 2e9:
        return None

    t_ns = np.arange(t0, t1, DT * 1e9, dtype=np.float64)
    t_s = (t_ns - t0) * 1e-9

    a_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_accel"].values)
    a_act = np.interp(t_s, (df_accel["t_ns"].values - t0) * 1e-9, df_accel["accel"].values)
    d_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_steer"].values)
    d_act = np.interp(t_s, (df_steer["t_ns"].values - t0) * 1e-9, df_steer["steer"].values)
    vx = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)
    wz = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["wz"].values)

    return {
        "uuid": uuid,
        "a_cmd": a_cmd.astype(np.float32),
        "a_act": a_act.astype(np.float32),
        "d_cmd": d_cmd.astype(np.float32),
        "d_act": d_act.astype(np.float32),
        "vx": vx.astype(np.float32),
        "wz": wz.astype(np.float32),
    }

def _fit_one_long_worker(ds: dict) -> tuple[float, float, float] | None:
    a_cmd = ds["a_cmd"]
    a_act = ds["a_act"]
    vx = ds["vx"]
    d_cmd_acc = np.abs(np.gradient(a_cmd, DT))
    mask = (vx > VX_MIN) & (d_cmd_acc > DA_THRESH)
    if mask.sum() < 50:
        return None

    best_mse = np.inf
    best_tau = np.nan
    best_delay = np.nan
    best_asf = np.nan
    for delay_s in np.arange(0.0, 0.31, 0.01):
        n_delay = int(round(delay_s / DT))
        tau, asf, mse = _fit_tau_nls_long(a_cmd, a_act, n_delay, mask)
        if mse < best_mse:
            best_mse = mse
            best_tau = tau
            best_delay = delay_s
            best_asf = asf
    if np.isnan(best_tau):
        return None
    return best_tau, best_delay, best_asf

def _fit_one_steer_worker(ds: dict) -> tuple[float, float, float, float] | None:
    d_cmd = ds["d_cmd"]
    d_act = ds["d_act"]
    vx = ds["vx"]
    wz = ds["wz"]
    d_cmd_steer = np.abs(np.gradient(d_cmd, DT))
    mask_dyn = (vx > VX_MIN) & (d_cmd_steer > DSTEER_MIN / DT)
    if mask_dyn.sum() < 50:
        return None

    mask_straight = (vx > 3.0) & (np.abs(wz) < 0.005)
    bias = float(np.mean(d_act[mask_straight])) if mask_straight.sum() > 20 else 0.0005

    d_act_nobias = d_act - bias
    sum_cmd2 = np.sum(d_cmd[mask_dyn] ** 2)
    dsf = float(np.sum(d_cmd[mask_dyn] * d_act_nobias[mask_dyn]) / sum_cmd2) if sum_cmd2 > 1e-5 else 1.0
    dsf = float(np.clip(dsf, 0.8, 1.2))

    best_mse = np.inf
    best_tau = np.nan
    best_delay = np.nan
    for delay_s in np.arange(0.0, 0.16, 0.01):
        n_delay = int(round(delay_s / DT))
        d_cmd_scaled = dsf * d_cmd
        tau, mse = _fit_tau_nls_steer(d_cmd_scaled, d_act_nobias, n_delay, mask_dyn)
        if mse < best_mse:
            best_mse = mse
            best_tau = tau
            best_delay = delay_s
    if np.isnan(best_tau):
        return None
    return best_tau, best_delay, bias, dsf

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
    ap.add_argument("--n-jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    # scenario.yaml から wheelbase をロード
    global WHEELBASE
    try:
        with args.scenario.open("r") as f:
            scen = yaml.safe_load(f)
            # models -> best_normal -> wheelbase などがあれば取得
            models = scen.get("Conditions", {}).get("models", {})
            for case_key in ["best_normal", "case_normal", "normal"]:
                if case_key in models:
                    wb = models[case_key].get("wheelbase")
                    if wb:
                        WHEELBASE = float(wb)
                        print(f"[INFO] scenario.yaml から wheelbase を取得しました: {WHEELBASE}")
                        break
    except Exception as e:
        print(f"[WARN] scenario.yaml のパースに失敗しました (既定値 {WHEELBASE} を使用): {e}")

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
    print(f"\n[Phase {phase_label}] データセット並列ロード ({len(tasks)} 件)...")

    datasets = []
    with ProcessPoolExecutor(max_workers=args.n_jobs) as pool:
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
    if args.phase in (1, 12) or "acc_time_constant" not in params:
        print("\n=== 縦方向モデルの直接同定を実行中 ===")
        taus_long = []
        delays_long = []
        asfs_long = []
        with ProcessPoolExecutor(max_workers=args.n_jobs) as pool:
            futs = {pool.submit(_fit_one_long_worker, ds): ds["uuid"] for ds in datasets}
            for fut in as_completed(futs):
                res = fut.result()
                if res is not None:
                    tau, delay, asf = res
                    taus_long.append(tau)
                    delays_long.append(delay)
                    asfs_long.append(asf)

        if taus_long:
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
        print("\n=== 操舵モデルおよびアンダーステアの直接同定を実行中 ===")
        taus_steer = []
        delays_steer = []
        biases = []
        dsfs = []
        with ProcessPoolExecutor(max_workers=args.n_jobs) as pool:
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

        vx_all = np.concatenate(all_vx)
        wz_all = np.concatenate(all_wz)
        steer_all = np.concatenate(all_steer)
        dwz_all = np.concatenate(all_dwz)

        # 定常旋回フィルタ
        mask_ok = (np.abs(wz_all) > WZ_MIN) & (np.abs(dwz_all) < DWZ_MAX) & (vx_all > VX_MIN_CURVE)
        vx_f = vx_all[mask_ok]
        wz_f = wz_all[mask_ok]
        steer_f = steer_all[mask_ok] - params["steer_bias"]
        tan_steer = np.tan(np.clip(steer_f, -0.8, 0.8))

        n_bins = len(VX_EDGES) - 1
        vx_mid = []
        kus_ols = []
        for i in range(n_bins):
            lo, hi = VX_EDGES[i], VX_EDGES[i + 1]
            mask_bin = (vx_f >= lo) & (vx_f < hi)
            n = int(mask_bin.sum())
            if n < 10:
                continue
            vx_b = vx_f[mask_bin]
            wz_b = wz_f[mask_bin]
            ts_b = tan_steer[mask_bin]
            vm = float(np.median(vx_b))

            C_ols = float(np.sum(wz_b * ts_b) / np.sum(wz_b ** 2))
            k_val = (C_ols - WHEELBASE / vm) / vm
            k_val = float(np.clip(k_val, -K_US_CLIP, K_US_CLIP))
            vx_mid.append(vm)
            kus_ols.append(k_val)

        # 速度閾値 [3.0, 6.0] m/s で区切ってステップ段を決定
        thresh1 = 3.0
        thresh2 = 6.0
        lo_vals = [k for v, k in zip(vx_mid, kus_ols) if v < thresh1]
        mid_vals = [k for v, k in zip(vx_mid, kus_ols) if thresh1 <= v < thresh2]
        hi_vals = [k for v, k in zip(vx_mid, kus_ols) if v >= thresh2]

        k_us_lo = float(np.median(lo_vals)) if lo_vals else 0.008
        k_us_mid = float(np.median(mid_vals)) if mid_vals else 0.008
        k_us_hi = float(np.median(hi_vals)) if hi_vals else 0.008

        # 下限クリップ (アンダーステア補正として正の値を担保)
        k_us_lo = max(0.0, k_us_lo)
        k_us_mid = max(0.0, k_us_mid)
        k_us_hi = max(0.0, k_us_hi)

        params["k_us"] = k_us_hi
        params["k_us_bands"] = [k_us_lo, k_us_mid, k_us_hi]
        params["k_us_thresholds"] = [thresh1, thresh2]

        print(f"  同定結果: k_us_lo  (< {thresh1}m/s)  = {k_us_lo:.5f}")
        print(f"            k_us_mid (3.0-6.0m/s) = {k_us_mid:.5f}")
        print(f"            k_us_hi  (>= {thresh2}m/s) = {k_us_hi:.5f}")

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
