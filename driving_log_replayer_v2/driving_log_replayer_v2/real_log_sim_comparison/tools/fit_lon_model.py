#!/usr/bin/env python3
"""縦横モデル検証ビューア (lib/_model_viewer.py lon_lat_model) の縦方向モデルを実機ログに
プログラムで最小二乗フィットし、C++ 拡張モデルの新パラメータ
(acc_time_constant=throttle τ, brake_time_constant=brake τ, lon_drag_c0/c1/c2, lon_lat_coupling)
を同定する。

モデル (C++ calcModel と一致):
    a_target(t) = a_cmd(t - T) + (c0 + c1·v + c2·v²) + coupling·(v·ω)²
    ȧ = -(a - a_target) / τ,   τ = τ_thr (a_cmd>=0) / τ_brk (a_cmd<0)
    v̇ = a   (= 観測。ここでは a の応答を当てはめる)

フィットは前方 Euler 積分による出力 (a) 誤差最小化。線形パラメータ (poly, coupling) は τ 固定で
最小二乗、τ_thr/τ_brk はグリッド探索。dead-time T は C++ が単一キューのため spec 値固定。

使い方: python3 tools/fit_lon_model.py LITE_DIR [LITE_DIR ...]   (real.lite ディレクトリ)
"""
from __future__ import annotations

import glob
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

DT = 0.02          # 50 Hz リサンプル
T_DEADTIME = 0.10  # acc_time_delay spec (C++ は単一キュー)


def _read(litedir: str) -> dict:
    mc = glob.glob(litedir + "/*.mcap")[0]
    r = SequentialReader()
    r.open(StorageOptions(uri=mc, storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out: dict = {k: ([], []) for k in ("a", "v", "cmd", "wz")}
    topic_map = {
        "/localization/acceleration": ("a", lambda m: m.accel.accel.linear.x),
        "/vehicle/status/velocity_status": ("v", lambda m: m.longitudinal_velocity),
        "/control/command/control_cmd": ("cmd", lambda m: m.longitudinal.acceleration),
        "/localization/kinematic_state": ("wz", lambda m: m.twist.twist.angular.z),
    }
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in topic_map:
            key, fn = topic_map[topic]
            try:
                val = fn(deserialize_message(data, get_message(types[topic])))
            except Exception:
                continue
            out[key][0].append(t / 1e9)
            out[key][1].append(val)
    return {k: (np.asarray(ts), np.asarray(vs)) for k, (ts, vs) in out.items()}


def _resample(series: dict) -> dict:
    """共通時間グリッド (DT) に線形補間。全系列が存在する重なり区間に限定。"""
    t0 = max(series[k][0].min() for k in series)
    t1 = min(series[k][0].max() for k in series)
    grid = np.arange(t0, t1, DT)
    res = {"t": grid}
    for k, (ts, vs) in series.items():
        res[k] = np.interp(grid, ts, vs)
    return res


def _integrate(target: np.ndarray, tau: np.ndarray, a0: float) -> np.ndarray:
    """ȧ = -(a - target)/tau を前方 Euler 積分 (時変 tau)。"""
    a = np.empty_like(target)
    a[0] = a0
    for i in range(len(target) - 1):
        a[i + 1] = a[i] + DT * (-(a[i] - target[i]) / tau[i])
    return a


def fit(datasets: list[dict]) -> dict:
    # プール: 各データセットの基底応答を連結して 1 回の LS で解く
    shift = int(round(T_DEADTIME / DT))
    grid_thr = np.arange(0.10, 0.81, 0.05)
    grid_brk = np.arange(0.10, 0.81, 0.05)
    best = None
    for tau_thr in grid_thr:
        for tau_brk in grid_brk:
            A_blocks, y_blocks = [], []
            for d in datasets:
                cmd = d["cmd"]
                cmd_del = np.concatenate([np.full(shift, cmd[0]), cmd[:-shift]]) if shift else cmd
                tau = np.where(cmd_del >= 0.0, tau_thr, tau_brk)
                v, wz, a = d["v"], d["wz"], d["a"]
                # 指令応答 (IC = 観測初期 a)
                a_cmd_resp = _integrate(cmd_del, tau, a[0])
                # 基底応答 (IC=0): poly1,poly2,coupling
                # poly0(定数)は除外: 停止時(v=0)に spurious 加速を生むと closed-loop で誤発進する。
                # 走行抵抗は v=0 でゼロが物理的に正しい (poly(v)=c1·v+c2·v²)。
                bases = [v, v * v, (v * wz) ** 2]
                basis_resp = [_integrate(b, tau, 0.0) for b in bases]
                A_blocks.append(np.column_stack(basis_resp))
                y_blocks.append(a - a_cmd_resp)
            A = np.vstack(A_blocks)
            y = np.concatenate(y_blocks)
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            resid = y - A @ coef
            rmse = float(np.sqrt(np.mean(resid ** 2)))
            if best is None or rmse < best["rmse"]:
                best = {"rmse": rmse, "tau_thr": float(tau_thr), "tau_brk": float(tau_brk),
                        "c0": 0.0, "c1": float(coef[0]),
                        "c2": float(coef[1]), "coupling": float(coef[2])}
    return best


def baseline_rmse(datasets: list[dict], tau_single: float) -> float:
    """単一 τ・poly/coupling なし (現 best_normal acc_time_constant=0.30 相当) の当てはめ RMSE。"""
    shift = int(round(T_DEADTIME / DT))
    errs = []
    for d in datasets:
        cmd = d["cmd"]
        cmd_del = np.concatenate([np.full(shift, cmd[0]), cmd[:-shift]]) if shift else cmd
        tau = np.full_like(cmd_del, tau_single)
        a_sim = _integrate(cmd_del, tau, d["a"][0])
        errs.append((a_sim - d["a"]) ** 2)
    return float(np.sqrt(np.mean(np.concatenate(errs))))


def main() -> None:
    labels = ["odaiba", "takanawa", "ds3", "ds4"]
    datasets = []
    for i, lite in enumerate(sys.argv[1:]):
        s = _resample(_read(lite))
        datasets.append(s)
        lab = labels[i] if i < len(labels) else f"ds{i}"
        print(f"[{lab}] {lite.split('/out/')[-1].split('/')[0]}: {len(s['t'])} samples "
              f"({s['t'][-1]-s['t'][0]:.0f}s), a∈[{s['a'].min():.2f},{s['a'].max():.2f}] "
              f"cmd∈[{s['cmd'].min():.2f},{s['cmd'].max():.2f}]")
    print()
    base = baseline_rmse(datasets, 0.30)
    res = fit(datasets)
    print(f"baseline (単一τ=0.30, poly/coupling なし) accel-fit RMSE = {base:.4f} m/s²")
    print(f"fitted  RMSE = {res['rmse']:.4f} m/s²  (改善 {100*(1-res['rmse']/base):.1f}%)")
    print()
    print("=== 同定パラメータ (best_normal の縦方向に設定) ===")
    print(f"  acc_time_constant (throttle τ): {res['tau_thr']:.3f}")
    print(f"  brake_time_constant (brake τ) : {res['tau_brk']:.3f}")
    print(f"  lon_drag_c0  : {res['c0']:+.4f}")
    print(f"  lon_drag_c1  : {res['c1']:+.5f}")
    print(f"  lon_drag_c2  : {res['c2']:+.6f}")
    print(f"  lon_lat_coupling: {res['coupling']:+.5f}")


if __name__ == "__main__":
    main()
