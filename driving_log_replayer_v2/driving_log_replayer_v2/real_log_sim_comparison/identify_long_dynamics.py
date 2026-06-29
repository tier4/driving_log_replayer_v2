#!/usr/bin/env python3
"""
縦方向一次遅れモデルの直接同定。

a_cmd (control_cmd.longitudinal.acceleration) と a_actual (VehicleReport.accel) を使い、
モデル  da/dt = (a_cmd(t-T) - a) / τ  の τ と T を各 DS で同定する。

各遅延候補 T に対して output-error NLS で τ を推定し、MSE が最小の (τ, T) を採用する。

使い方:
  python3 identify_long_dynamics.py \\
    --collection-dir /home/kotaroyoshimoto/data/openloop_j6_15_june \\
    --n-jobs 8
"""
import argparse
import glob
import sys
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter

warnings.filterwarnings("ignore")

for _p in glob.glob("/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/*/local/lib/python3.10/dist-packages"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from driving_log_replayer_v2.real_log_sim_comparison.lib._io import (
    load_accel,
    load_cmd,
    load_velocity,
)

CMD_TOPIC = "/control/command/control_cmd"
DT = 0.01           # リサンプリング間隔 [s] (10ms: 遅延分解能 10ms)
VX_MIN = 0.5        # 低速除外 [m/s]
DA_THRESH = 0.15    # 動的区間フィルタ: |Δa_cmd/dt| [m/s²/s]

DELAY_CANDIDATES = np.arange(0.0, 0.31 + 1e-9, 0.01)  # 0〜300ms, 10ms 刻み
TAU_MIN, TAU_MAX = 0.01, 5.0


def _simulate_long(a_cmd: np.ndarray, tau: float, n_delay: int) -> np.ndarray:
    """一次遅れ+純粋遅延モデル（lfilter 版）。"""
    n = len(a_cmd)
    a_del = np.empty(n)
    if n_delay > 0:
        a_del[:n_delay] = a_cmd[0]
        a_del[n_delay:] = a_cmd[:-n_delay]
    else:
        a_del = a_cmd.copy()
    alpha = float(np.clip(DT / tau, 0.0, 1.0))
    return lfilter([alpha], [1.0, -(1.0 - alpha)], a_del)


def _fit_tau_nls(
    a_cmd: np.ndarray, a_act: np.ndarray, n_delay: int, mask: np.ndarray
) -> tuple[float, float]:
    """output-error NLS で τ を推定（T はグリッド固定）。

    目的関数: MSE(a_sim(τ; a_cmd)[mask] - a_act[mask])
    """
    def _mse(log_tau: float) -> float:
        tau = float(np.exp(log_tau))
        a_sim = _simulate_long(a_cmd, tau, n_delay)
        return float(np.mean((a_sim[mask] - a_act[mask]) ** 2))

    res = minimize_scalar(_mse, bounds=(np.log(TAU_MIN), np.log(TAU_MAX)), method="bounded")
    tau = float(np.exp(res.x))
    return tau, float(res.fun)


def _fit_one_dataset(args: tuple) -> dict | None:
    uuid, ds_dir = args
    mcap = Path(ds_dir) / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None
    try:
        df_cmd = load_cmd(mcap, CMD_TOPIC)
        df_accel = load_accel(mcap)
        df_vel = load_velocity(mcap)
    except Exception:
        return None
    if df_cmd.empty or df_accel.empty or df_vel.empty:
        return None

    t0 = max(df_cmd["t_ns"].iloc[0], df_accel["t_ns"].iloc[0], df_vel["t_ns"].iloc[0])
    t1 = min(df_cmd["t_ns"].iloc[-1], df_accel["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1])
    if (t1 - t0) < 2e9:
        return None

    t_ns = np.arange(t0, t1, DT * 1e9, dtype=np.float64)
    t_s = (t_ns - t0) * 1e-9

    a_cmd_arr = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_accel"].values)
    a_act_arr = np.interp(t_s, (df_accel["t_ns"].values - t0) * 1e-9, df_accel["accel"].values)
    vx = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)

    d_cmd = np.abs(np.gradient(a_cmd_arr, DT))
    mask_dyn = (vx > VX_MIN) & (d_cmd > DA_THRESH)
    if mask_dyn.sum() < 50:
        return None

    # グリッドサーチ: 各 delay 候補で output-error NLS → τ → MSE
    best_mse = np.inf
    best_tau = np.nan
    best_delay = np.nan
    for delay_s in DELAY_CANDIDATES:
        n_delay = int(round(delay_s / DT))
        tau, mse = _fit_tau_nls(a_cmd_arr, a_act_arr, n_delay, mask_dyn)
        if mse < best_mse:
            best_mse = mse
            best_tau = tau
            best_delay = delay_s

    if np.isnan(best_tau):
        return None

    return {
        "uuid": uuid,
        "tau": best_tau,
        "delay": best_delay,
        "mse": best_mse,
        "rmse_mps2": float(np.sqrt(best_mse)),
        "n_dyn": int(mask_dyn.sum()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="縦方向一次遅れモデル直接同定")
    ap.add_argument("--collection-dir", type=Path,
                    default=Path("/home/kotaroyoshimoto/data/openloop_j6_15_june"))
    ap.add_argument("--n-jobs", type=int, default=8)
    args = ap.parse_args()

    ds_root = args.collection_dir / "datasets"
    if not ds_root.exists():
        ds_root = args.collection_dir
    ds_dirs = sorted(ds_root.iterdir()) if ds_root.is_dir() else []
    tasks = [(d.name, str(d)) for d in ds_dirs if (d / "real.lite" / "real.lite_0.mcap").exists()]
    print(f"対象 DS: {len(tasks)} 件")

    results = []
    with ProcessPoolExecutor(max_workers=args.n_jobs) as pool:
        futs = {pool.submit(_fit_one_dataset, t): t for t in tasks}
        for i, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            if r is not None:
                results.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(tasks)} 処理済み", flush=True)

    print(f"\n有効 DS: {len(results)} 件")
    if not results:
        print("ERROR: 有効結果なし")
        return

    df = pd.DataFrame(results)

    print("\n=== acc_time_constant (τ) [s] ===")
    print(f"  mean:  {df['tau'].mean():.4f}")
    print(f"  std:   {df['tau'].std():.4f}")
    for q, label in [(0.10, "p10"), (0.25, "p25"), (0.50, "p50"), (0.75, "p75"), (0.90, "p90")]:
        print(f"  {label}:   {df['tau'].quantile(q):.4f}")

    print("\n=== acc_time_delay (T) [s] ===")
    print(f"  mean:  {df['delay'].mean():.4f}")
    print(f"  std:   {df['delay'].std():.4f}")
    for q, label in [(0.10, "p10"), (0.25, "p25"), (0.50, "p50"), (0.75, "p75"), (0.90, "p90")]:
        print(f"  {label}:   {df['delay'].quantile(q):.4f}")

    print("\n=== 残差 RMSE [m/s²] ===")
    print(f"  mean:  {df['rmse_mps2'].mean():.4f}")
    print(f"  p50:   {df['rmse_mps2'].quantile(0.50):.4f}")
    print(f"  p90:   {df['rmse_mps2'].quantile(0.90):.4f}")

    print("\n=== 推奨値（中央値） ===")
    tau_med = df["tau"].median()
    delay_med = df["delay"].median()
    print(f"  acc_time_constant: {tau_med:.4f} s")
    print(f"  acc_time_delay:    {delay_med:.4f} s")

    out_csv = args.collection_dir / "long_dynamics_identified.csv"
    df.to_csv(out_csv, index=False)
    print(f"\n結果 CSV: {out_csv}")


if __name__ == "__main__":
    main()
