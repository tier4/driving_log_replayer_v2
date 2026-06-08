#!/usr/bin/env python3
"""Stage 9: 縦方向パラメータ (brake_time_constant) を発進フィットで同定する.

Stage 7 (k_us, 横/yaw) の縦方向版。実機 lite の post-gate control_cmd をオープンループ入力
として縦方向モデル (DELAY_STEER_ACC_BRAKE_GEARED_WO_FALL_GUARD の簡易実装) を
brake_time_constant グリッドで回し、発進直後 (t=0〜5s) の実機 actual velocity への
フィット RMSE を最小化する brake_time_constant を同定する (グリッド最小 + 放物線サブグリッド)。

brake 応答は発進・停止時に最も表れるため、評価窓は「発進 (curve_config があればカーブ②、
無ければ最初の発進)」前後に絞る。real lite のみを使う自己完結ステージで、evaluator_node が
Stage 8 の後に env のみで実行する (追加設定不要)。

注: 縦方向モデルは Python 簡易実装 (C++ 実モデルは brake 変種を wrapper 未 export)。実シムの
brake_time_constant 既定は simulator_model.param.yaml (load_sim_params) 参照。

**ill-posed 性に注意**: brake_time_constant は本来「減速・停止」時に支配的なパラメータで、
発進 (加速) 窓では弱くしか拘束されない。実データでは launch 区間の RMSE が brake_tc に対し
単調減少し、最小が非物理的な大値 (>1s) に張り付くことがある (= sim が発進速度を過大予測する
分を brake_tc が誤って吸収しているだけで、真の brake_tc 同定ではない)。その場合は端最小として
警告する。robust な brake_tc 同定には減速・停止イベントを含む窓での解析が必要 (将来拡張)。
mean_err > 0 は sim が実機より発進が速い (過大予測) ことを示す診断値として併読すること。
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .lib._events import find_autonomous_start, find_curve2_launch, find_initial_launch
from .lib._fig_io import write_fig_json
from .lib._figures import (
    build_fig_brake_sweep,
    build_fig_departure_brake_sensitivity,
    build_fig_real_cmd_acc,
)
from .lib._io import align_time, load_cmd, load_operation_mode, load_velocity, resolve_lite_bag
from .lib._params_utils import add_params_annotation, load_sim_params, setup_jp_font
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config
from .lib._nstep_common import parabolic_min as _parabolic_min

setup_jp_font()


REAL_CMD_TOPIC_CANDIDATES = [
    "/control/command/control_cmd",
    "/sub/control/command/control_cmd",
]


class BrakeAccVehicleModel:
    """DELAY_STEER_ACC_BRAKE_GEARED_WO_FALL_GUARD の縦方向モデル簡易実装."""

    def __init__(
        self,
        acc_time_delay,
        acc_time_constant,
        brake_delay,
        brake_time_constant,
        departure_vx_threshold,
        vel_rate_lim,
        dt,
    ):
        self.acc_tc = acc_time_constant
        self.brake_tc = brake_time_constant
        self.departure_threshold = departure_vx_threshold
        self.vel_rate_lim = vel_rate_lim
        self.dt = dt

        acc_queue_len = max(1, round(acc_time_delay / dt))
        brake_queue_len = max(1, round(brake_delay / dt))
        self.acc_queue = deque([0.0] * acc_queue_len)
        self.brake_queue = deque([0.0] * brake_queue_len)

        self.vx = 0.0
        self.pedal_acc = 0.0
        self.pedal_brake = 0.0
        self.in_departure = True
        self.delayed_accel = 0.0
        self.delayed_brake = 0.0

    def step(self, cmd_acc):
        accel_des = max(0.0, cmd_acc)
        brake_des = min(0.0, cmd_acc)

        self.acc_queue.append(accel_des)
        self.delayed_accel = self.acc_queue.popleft()
        self.brake_queue.append(brake_des)
        self.delayed_brake = self.brake_queue.popleft()

        if self.vx == 0.0:
            self.in_departure = True
        if self.vx > self.departure_threshold:
            self.in_departure = False

        eff_brake_tc = self.brake_tc if self.in_departure else self.acc_tc

        net = self.pedal_acc + self.pedal_brake
        d_vx = net
        d_pacc = -(self.pedal_acc - min(self.delayed_accel, self.vel_rate_lim)) / self.acc_tc
        d_pbrake = -(self.pedal_brake - max(self.delayed_brake, -self.vel_rate_lim)) / eff_brake_tc

        prev_vx = self.vx
        self.vx += d_vx * self.dt
        self.pedal_acc += d_pacc * self.dt
        self.pedal_brake += d_pbrake * self.dt

        self.pedal_acc = max(0.0, self.pedal_acc)
        self.pedal_brake = min(0.0, self.pedal_brake)
        self.vx = max(-50.0, min(self.vx, 50.0))

        if prev_vx * self.vx <= 0.0 and self.delayed_brake < 0.0:
            if -(self.pedal_acc + self.pedal_brake) >= 0.0:
                self.vx = 0.0

        if self.vx < 0.0 and self.delayed_brake >= 0.0:
            net_ss = self.delayed_accel
            if net_ss >= 0.0:
                self.vx = 0.0

        return self.vx


def simulate_departure(
    cmd_series: np.ndarray,
    brake_tc_variants: list[float],
    params_base: dict,
    dt: float = 1.0 / 40,
) -> dict[float, np.ndarray]:
    """同一 cmd_series に対して複数の brake_time_constant でシミュレーション."""
    results = {}
    for btc in brake_tc_variants:
        model = BrakeAccVehicleModel(
            acc_time_delay=params_base["acc_time_delay"],
            acc_time_constant=params_base["acc_time_constant"],
            brake_delay=params_base["brake_delay"],
            brake_time_constant=btc,
            departure_vx_threshold=params_base["departure_vx_threshold"],
            vel_rate_lim=params_base["vel_rate_lim"],
            dt=dt,
        )
        results[btc] = np.array([model.step(c) for c in cmd_series])
    return results


def _resolve_t_launch(
    df_vel_aligned: pd.DataFrame,
    window: tuple[float, float],
    *,
    curve2_configured: bool,
) -> float:
    """発進検出。curve② が設定されている時のみ curve② 発進窓を使い、それ以外は初回発進で整列。

    curve② 非設定 (curve_config_yaml="") では curve2_window 既定 (20,120) が走行の実態と無関係で、
    初期停止が窓外・ゴール停止終端が窓内に入る走行 (停止開始の完全 start→goal) では goal 停止を
    発進と誤検出してしまう。よって curve② 非設定時は real/sim とも `find_initial_launch` で
    初回発進に整列する (整列の非対称性を排除)。
    """
    if curve2_configured:
        t = find_curve2_launch(df_vel_aligned, window=window)
        if t is not None:
            return float(t)
    t = find_initial_launch(df_vel_aligned)
    return float(t) if t is not None else 0.0


def main() -> None:
    parser = argparse.ArgumentParser(description="縦方向パラメータ (brake_time_constant) 同定")
    parser.add_argument(
        "--brake-tc-values",
        default="0.03,0.05,0.08,0.10,0.15,0.20,0.30,0.40",
        help="カンマ区切りの brake_time_constant グリッド [s]",
    )
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    real_bag = resolve_lite_bag(cfg.lite_dir, "real")
    if real_bag is None:
        print(f"ERROR: real lite bag が見つかりません: {cfg.lite_dir}", file=sys.stderr)
        sys.exit(1)

    print("=== 発進時車両モデル応答診断 ===")
    print(f"bag: {real_bag}\n")

    df_mode = load_operation_mode(real_bag)
    df_vel_raw = load_velocity(real_bag)
    t0_ns = find_autonomous_start(df_mode, df_vel_raw)
    df_vel = align_time(df_vel_raw, t0_ns)

    t_launch = _resolve_t_launch(
        df_vel, cfg.curve2_window, curve2_configured=cfg.curve2 is not None
    )
    print(f"実機 t_launch = {t_launch:.2f} s")

    cmd_topic = (
        cfg.topic_overrides.get("real", {}).get("cmd")
        if cfg.topic_overrides
        else None
    ) or REAL_CMD_TOPIC_CANDIDATES
    df_cmd_raw = load_cmd(real_bag, cmd_topic)
    if df_cmd_raw.empty:
        print("ERROR: control_cmd が空です。lite mcap を確認してください。", file=sys.stderr)
        sys.exit(1)

    df_cmd = align_time(df_cmd_raw, t0_ns)
    df_cmd["t"] = df_cmd["t"] - t_launch
    df_vel = df_vel.copy()
    df_vel["t"] = df_vel["t"] - t_launch

    window = (-2.0, 15.0)
    df_cmd = df_cmd[(df_cmd["t"] >= window[0]) & (df_cmd["t"] <= window[1])].reset_index(drop=True)
    df_vel = df_vel[(df_vel["t"] >= window[0]) & (df_vel["t"] <= window[1])].reset_index(drop=True)
    print(f"control_cmd サンプル数: {len(df_cmd)}")
    print(f"velocity サンプル数: {len(df_vel)}")

    dt = 1.0 / 40.0
    t_sim = np.arange(window[0], window[1], dt)
    cmd_resampled = np.interp(t_sim, df_cmd["t"].values, df_cmd["cmd_accel"].values)

    params_base = {
        "acc_time_delay": 0.101,
        "acc_time_constant": 0.2589,
        "brake_delay": 0.0685,
        "departure_vx_threshold": 1.0,
        "vel_rate_lim": 7.0,
    }
    brake_tc_variants = sorted(float(x) for x in args.brake_tc_values.split(",") if x.strip())
    cmap = plt.get_cmap("viridis")
    colors = [cmap(i / max(1, len(brake_tc_variants) - 1)) for i in range(len(brake_tc_variants))]
    labels = [f"brake_tc={v:.4f}s" for v in brake_tc_variants]

    print("\nbrake_time_constant バリアントでシミュレーション中...")
    sim_results = simulate_departure(cmd_resampled, brake_tc_variants, params_base, dt=dt)

    print(
        f"\n{'t[s]':>6} | {'実機actual':>10} | "
        + " | ".join(f"btc={v:.4f}" for v in brake_tc_variants)
    )
    print("-" * (6 + 10 + 16 * len(brake_tc_variants) + 10))
    for t_chk in [-0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0]:
        idx = np.argmin(np.abs(t_sim - t_chk))
        real_v = (
            float(np.interp(t_chk, df_vel["t"].values, df_vel["lon_vel"].values))
            if not df_vel.empty
            else float("nan")
        )
        row = f"{t_chk:>6.1f} | {real_v:>10.3f} | "
        row += " | ".join(f"{sim_results[v][idx]:>10.3f}   " for v in brake_tc_variants)
        print(row)

    cfg.figs_dir.mkdir(parents=True, exist_ok=True)
    anno_params = {**load_sim_params(), **params_base}

    fig = build_fig_departure_brake_sensitivity(
        df_vel["t"].values, df_vel["lon_vel"].values, t_sim, sim_results,
        list(brake_tc_variants), labels,
        scenario_name=cfg.scenario_name, params=anno_params,
    )
    write_fig_json(fig, cfg.figs_dir / "departure_brake_tc_sensitivity")

    fig2 = build_fig_real_cmd_acc(
        df_cmd["t"].values, df_cmd["cmd_accel"].values,
        scenario_name=cfg.scenario_name, params=anno_params,
    )
    write_fig_json(fig2, cfg.figs_dir / "real_cmd_acc_departure")

    print("\n--- 実機 actual速度 vs FMU シム 発進誤差 RMSE (t=0~5s) ---")
    t_mask = (t_sim >= 0.0) & (t_sim <= 5.0)
    real_interp = np.interp(t_sim[t_mask], df_vel["t"].values, df_vel["lon_vel"].values)
    rmses: list[float] = []
    mean_errs: list[float] = []
    for btc in brake_tc_variants:
        sim_v = sim_results[btc][t_mask]
        rmse = float(np.sqrt(np.mean((sim_v - real_interp) ** 2)))
        mean_err = float(np.mean(sim_v - real_interp))
        rmses.append(rmse)
        mean_errs.append(mean_err)
        print(f"  brake_tc={btc:.4f}:  RMSE={rmse:.4f} m/s, mean_err={mean_err:+.4f} m/s")

    # --- 同定 (発進フィット RMSE 最小化, グリッド + 放物線サブグリッド) ---
    out_dir = cfg.out_dir / "brake_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    pd.DataFrame({
        "brake_time_constant": brake_tc_variants,
        "rmse_mps": rmses,
        "mean_err_mps": mean_errs,
    }).to_csv(out_dir / "brake_sweep.csv", index=False)

    best_i = int(np.argmin(rmses))
    btc_grid = float(brake_tc_variants[best_i])
    btc_parab = _parabolic_min(list(brake_tc_variants), rmses)

    btc_id = btc_parab if btc_parab is not None else btc_grid
    figb = build_fig_brake_sweep(list(brake_tc_variants), rmses, btc_id, params=anno_params)
    write_fig_json(figb, out_dir / "brake_sweep")

    print("\n=== 同定結果 (発進フィット RMSE 最小化, t=0〜5s) ===")
    print(f"  グリッド最小: brake_time_constant = {btc_grid:.4f} s "
          f"(RMSE = {rmses[best_i]:.4f} m/s)")
    if btc_parab is not None:
        print(f"  放物線サブグリッド推定: brake_time_constant ≈ {btc_parab:.4f} s")
    else:
        print("  放物線サブグリッド推定: 最小がグリッド端のため範囲外 (--brake-tc-values を広げて再試行)")
    if best_i in (0, len(brake_tc_variants) - 1):
        print(f"  [WARN] 最小がグリッド端 (brake_tc={btc_grid:.4f}s)。")
        if best_i == len(brake_tc_variants) - 1 and btc_grid > 0.6:
            print("         brake_tc が非物理的な大値に張り付いています。brake_tc は減速時に支配的で、"
                  "発進窓では弱く拘束されるため、これは真の同定ではなく launch 過大予測の代理です "
                  "(mean_err>0 を併読)。減速・停止窓での解析が必要。")
        else:
            print("         範囲を広げて再実行を推奨。")
    print(f"  Saved: {out_dir / 'brake_sweep.csv'}, {out_dir / 'brake_sweep.svg'}")


if __name__ == "__main__":
    main()
