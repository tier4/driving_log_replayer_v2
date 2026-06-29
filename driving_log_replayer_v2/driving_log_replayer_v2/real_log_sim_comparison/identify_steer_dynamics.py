#!/usr/bin/env python3
"""
操舵1次遅れモデルの直接同定。

δ_cmd (control_cmd) と δ_actual (steering_status) を使い、
モデル  dδ/dt = (δ_cmd(t-T) - δ) / τ  の τ と T を各 DS で同定する。

使い方:
  python3 identify_steer_dynamics.py \
    --collection-dir /home/kotaroyoshimoto/data/openloop_j6_15_june \
    --n-jobs 8
"""
import argparse
import sys
import glob
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd

warnings.filterwarnings("ignore")

for _p in glob.glob("/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/*/local/lib/python3.10/dist-packages"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from driving_log_replayer_v2.real_log_sim_comparison.lib._io import (
    load_cmd,
    load_steering,
    load_velocity,
)

CMD_TOPIC = "/control/command/control_cmd"
DT = 0.01          # リサンプリング間隔 [s] (10ms: 遅延分解能 10ms)
VX_MIN = 0.5       # 低速除外 [m/s]
DSTEER_MIN = 0.001 # 動的区間フィルタ: |Δδ_cmd/dt| [rad/s]

# 探索範囲
DELAY_CANDIDATES = np.arange(0.0, 0.15 + 1e-9, 0.01)   # 0〜150ms, 10ms刻み
TAU_MIN, TAU_MAX = 0.01, 2.0


def _fit_tau_given_delay(delta_cmd_delayed: np.ndarray, delta_act: np.ndarray) -> float:
    """遅延量固定のとき OLS で τ を求める。

    モデル(離散): δ[k+1] - δ[k] = (dt/τ) * (δ_cmd_delayed[k] - δ[k])
    α = dt/τ を OLS 推定: α = Σ(dδ · e) / Σ(e²), e = δ_cmd_delayed - δ
    """
    e = delta_cmd_delayed - delta_act
    ddelta = np.diff(delta_act)
    e_mid = 0.5 * (e[:-1] + e[1:])  # 中間評価
    denom = float(np.sum(e_mid ** 2))
    if denom < 1e-12:
        return np.nan
    alpha = float(np.sum(ddelta * e_mid) / denom)
    if alpha <= 0:
        return np.nan
    tau = DT / alpha
    return float(np.clip(tau, TAU_MIN, TAU_MAX))


def _simulate_steer(delta_cmd: np.ndarray, tau: float, n_delay: int) -> np.ndarray:
    """1次遅れ+遅延モデルでδをシミュレート。"""
    n = len(delta_cmd)
    delta = np.empty(n)
    delta[0] = delta_cmd[0]
    alpha = DT / tau
    for k in range(n - 1):
        ki = max(0, k - n_delay)
        delta[k + 1] = delta[k] + alpha * (delta_cmd[ki] - delta[k])
    return delta


def _fit_one_dataset(args: tuple) -> dict | None:
    uuid, ds_dir = args
    mcap = Path(ds_dir) / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None
    try:
        df_cmd = load_cmd(mcap, CMD_TOPIC)
        df_steer = load_steering(mcap)
        df_vel = load_velocity(mcap)
    except Exception:
        return None
    if df_cmd.empty or df_steer.empty or df_vel.empty:
        return None

    # 共通タイムベースにリサンプリング
    t0 = max(df_cmd["t_ns"].iloc[0], df_steer["t_ns"].iloc[0], df_vel["t_ns"].iloc[0])
    t1 = min(df_cmd["t_ns"].iloc[-1], df_steer["t_ns"].iloc[-1], df_vel["t_ns"].iloc[-1])
    if (t1 - t0) < 2e9:  # 2秒未満は除外
        return None
    t_ns = np.arange(t0, t1, DT * 1e9, dtype=np.float64)
    t_s = (t_ns - t0) * 1e-9

    delta_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_steer"].values)
    delta_act = np.interp(t_s, (df_steer["t_ns"].values - t0) * 1e-9, df_steer["steer"].values)
    vx = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)

    # 動的・走行中フィルタ
    d_cmd = np.abs(np.gradient(delta_cmd, DT))
    mask_dyn = (vx > VX_MIN) & (d_cmd > DSTEER_MIN / DT)
    if mask_dyn.sum() < 50:
        return None

    # グリッドサーチ: 各 delay 候補で OLS → τ → MSE
    best_mse = np.inf
    best_tau = np.nan
    best_delay = np.nan
    for delay_s in DELAY_CANDIDATES:
        n_delay = int(round(delay_s / DT))
        # δ_cmd を n_delay ステップ前にシフト（遅延の適用）
        delta_cmd_delayed = np.empty_like(delta_cmd)
        if n_delay > 0:
            delta_cmd_delayed[:n_delay] = delta_cmd[0]
            delta_cmd_delayed[n_delay:] = delta_cmd[:-n_delay]
        else:
            delta_cmd_delayed = delta_cmd.copy()

        tau = _fit_tau_given_delay(delta_cmd_delayed[mask_dyn], delta_act[mask_dyn])
        if np.isnan(tau):
            continue

        delta_sim = _simulate_steer(delta_cmd, tau, n_delay)
        mse = float(np.mean((delta_sim[mask_dyn] - delta_act[mask_dyn]) ** 2))
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
        "rmse_mrad": np.sqrt(best_mse) * 1000,
        "n_dyn": int(mask_dyn.sum()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="操舵1次遅れモデル直接同定")
    ap.add_argument("--collection-dir", type=Path,
                    default=Path("/home/kotaroyoshimoto/data/openloop_j6_15_june"))
    ap.add_argument("--n-jobs", type=int, default=8)
    args = ap.parse_args()

    ds_root = args.collection_dir / "datasets"
    if not ds_root.exists():
        ds_root = args.collection_dir  # fallback
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

    print("\n=== steer_time_constant (τ) [s] ===")
    print(f"  mean:  {df['tau'].mean():.4f}")
    print(f"  std:   {df['tau'].std():.4f}")
    print(f"  p10:   {df['tau'].quantile(0.10):.4f}")
    print(f"  p25:   {df['tau'].quantile(0.25):.4f}")
    print(f"  p50:   {df['tau'].quantile(0.50):.4f}")
    print(f"  p75:   {df['tau'].quantile(0.75):.4f}")
    print(f"  p90:   {df['tau'].quantile(0.90):.4f}")

    print("\n=== steer_time_delay (T) [s] ===")
    print(f"  mean:  {df['delay'].mean():.4f}")
    print(f"  std:   {df['delay'].std():.4f}")
    print(f"  p10:   {df['delay'].quantile(0.10):.4f}")
    print(f"  p25:   {df['delay'].quantile(0.25):.4f}")
    print(f"  p50:   {df['delay'].quantile(0.50):.4f}")
    print(f"  p75:   {df['delay'].quantile(0.75):.4f}")
    print(f"  p90:   {df['delay'].quantile(0.90):.4f}")

    print("\n=== 残差 RMSE [mrad] ===")
    print(f"  mean:  {df['rmse_mrad'].mean():.2f}")
    print(f"  p50:   {df['rmse_mrad'].quantile(0.50):.2f}")
    print(f"  p90:   {df['rmse_mrad'].quantile(0.90):.2f}")

    print("\n=== 推奨値（中央値） ===")
    tau_med = df["tau"].median()
    delay_med = df["delay"].median()
    print(f"  steer_time_constant: {tau_med:.4f} s")
    print(f"  steer_time_delay:    {delay_med:.4f} s")

    out_csv = args.collection_dir / "steer_dynamics_identified.csv"
    df.to_csv(out_csv, index=False)
    print(f"\n結果 CSV: {out_csv}")


if __name__ == "__main__":
    main()
