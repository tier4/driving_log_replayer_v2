#!/usr/bin/env python3
"""発進時の車両モデル応答診断スクリプト.

実機 lite mcap の control_cmd をオープンループ入力として
DELAY_STEER_ACC_BRAKE_GEARED_WO_FALL_GUARD モデルを様々な brake_time_constant で
シミュレーションし、実機 actual velocity との比較から適切なパラメータを診断する。
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

from ..lib._events import find_autonomous_start, find_curve2_launch, find_sim_launch
from ..lib._io import align_time, load_cmd, load_operation_mode, load_velocity
from ..lib._params_utils import add_params_annotation, load_sim_params, setup_jp_font
from ..lib._runtime_config import add_common_cli_arguments, build_runtime_config

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


def _resolve_real_bag(lite_dir: Path) -> Path:
    """real lite bag のパスを単一ファイル / ディレクトリ両方の形式で解決する。"""
    candidates = [
        lite_dir / "real.lite.mcap",
        lite_dir / "real.lite",
    ]
    for p in candidates:
        if p.exists():
            return p
    msg = "real lite bag が見つかりません: " + " or ".join(str(c) for c in candidates)
    raise FileNotFoundError(msg)


def _resolve_t_launch(df_vel_aligned: pd.DataFrame, window: tuple[float, float]) -> float:
    """カーブ② 発進検出。見つからなければ速度ベースのフォールバック (t>=5s で v>0.5)."""
    t = find_curve2_launch(df_vel_aligned, window=window)
    if t is None:
        t = find_sim_launch(df_vel_aligned, threshold=0.5, min_t=5.0)
    return float(t) if t is not None else 0.0


def main() -> None:
    parser = argparse.ArgumentParser(description="発進時車両モデル応答診断")
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    real_bag = _resolve_real_bag(cfg.lite_dir)

    print("=== 発進時車両モデル応答診断 ===")
    print(f"bag: {real_bag}\n")

    df_mode = load_operation_mode(real_bag)
    df_vel_raw = load_velocity(real_bag)
    t0_ns = find_autonomous_start(df_mode, df_vel_raw)
    df_vel = align_time(df_vel_raw, t0_ns)

    t_launch = _resolve_t_launch(df_vel, cfg.curve2_window)
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
    brake_tc_variants = [0.0301, 0.10, 0.20, 0.40]
    colors = ["#d62728", "#ff7f0e", "#2ca02c", "#9467bd"]
    labels = [f"brake_tc={v:.4f}s" for v in brake_tc_variants]
    labels[0] += " (現在)"

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
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(
        f"{cfg.scenario_name}\n発進動作：brake_time_constant 感度分析\n"
        "(実機 post-gate cmd をオープンループ入力→シミュレーション速度 vs 実機 actual速度)",
        fontsize=11,
    )

    for ax, xlim, ylim, title in [
        (axes[0], (-1, 10), (-0.5, 8), "速度プロファイル比較（発進前後）"),
        (axes[1], (-0.5, 3.0), (-0.2, 2.5), "発進直後ズーム (t=-0.5~3s)"),
    ]:
        ax.plot(
            df_vel["t"].values,
            df_vel["lon_vel"].values,
            "k-",
            lw=3,
            label="実機 actual速度",
            zorder=10,
        )
        for btc, col, lbl in zip(brake_tc_variants, colors, labels):
            ax.plot(t_sim, sim_results[btc], color=col, lw=1.8, ls="--", label=f"FMU {lbl}")
        ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_xlabel("発進後 t [s]")
        ax.set_ylabel("速度 [m/s]")
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, lw=0.4)

    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 4))
    fig2.suptitle(
        f"{cfg.scenario_name}\n実機 post-gate control_cmd (acc) — 発進前後",
        fontsize=11,
    )
    ax2.plot(
        df_cmd["t"].values,
        df_cmd["cmd_accel"].values,
        "b-",
        lw=1.5,
        label="実機 cmd_acc (post-gate)",
    )
    ax2.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    ax2.set_xlim(-2, 10)
    ax2.set_xlabel("発進後 t [s]")
    ax2.set_ylabel("加速度指令 [m/s²]")
    ax2.legend(fontsize=9)
    ax2.grid(True, lw=0.4)

    fig.tight_layout()
    fig2.tight_layout()

    anno_params = {**load_sim_params(), **params_base}
    add_params_annotation(fig, anno_params)
    add_params_annotation(fig2, anno_params)

    out1 = cfg.figs_dir / "departure_brake_tc_sensitivity.png"
    out2 = cfg.figs_dir / "real_cmd_acc_departure.png"
    fig.savefig(str(out1), dpi=150, bbox_inches="tight")
    fig2.savefig(str(out2), dpi=150, bbox_inches="tight")
    plt.close("all")
    print(f"\n保存: {out1}")
    print(f"保存: {out2}")

    print("\n--- 実機 actual速度 vs FMU シム 発進誤差 RMSE (t=0~5s) ---")
    t_mask = (t_sim >= 0.0) & (t_sim <= 5.0)
    real_interp = np.interp(t_sim[t_mask], df_vel["t"].values, df_vel["lon_vel"].values)
    for btc in brake_tc_variants:
        sim_v = sim_results[btc][t_mask]
        rmse = np.sqrt(np.mean((sim_v - real_interp) ** 2))
        mean_err = np.mean(sim_v - real_interp)
        print(f"  brake_tc={btc:.4f}:  RMSE={rmse:.4f} m/s, mean_err={mean_err:+.4f} m/s")


if __name__ == "__main__":
    main()
