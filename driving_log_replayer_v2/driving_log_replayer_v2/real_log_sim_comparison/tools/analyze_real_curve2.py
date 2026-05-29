#!/usr/bin/env python3
"""実機ログのカーブ②詳細解析 — アンダーステア検証.

Output: comparison/figures/real_curve2_detail.{png,pdf}
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ..lib._events import find_autonomous_start, find_curve2_launch
from ..lib._io import (
    align_time,
    load_accel,
    load_cmd,
    load_kinematic,
    load_operation_mode,
    load_steering,
    load_velocity,
)
from ..lib._map import load_map_ways, map_ways_in_bbox
from ..lib._params_utils import add_params_annotation, setup_jp_font
from ..lib._runtime_config import RuntimeConfig, add_common_cli_arguments, build_runtime_config

setup_jp_font()


REAL_CMD_TOPIC_CANDIDATES = [
    "/control/command/control_cmd",
    "/sub/control/command/control_cmd",
]

T_PRE = -3.0
T_POST = 30.0


def _resolve_real_bag(lite_dir: Path) -> Path:
    for cand in (lite_dir / "real.lite.mcap", lite_dir / "real.lite"):
        if cand.exists():
            return cand
    raise FileNotFoundError(f"real lite bag が見つかりません: {lite_dir}")


def _clip_to_window(df, t_launch: float):
    mask = (df["t"] >= t_launch + T_PRE) & (df["t"] <= t_launch + T_POST)
    sub = df[mask].copy()
    sub["tr"] = sub["t"] - t_launch
    return sub


def _load_curve2_window(cfg: RuntimeConfig) -> tuple[dict, float]:
    """curve_config 由来のカーブ② 中心情報と margin を返す。"""
    curve2 = cfg.curve2
    if curve2 is None:
        # curve_config 未指定 / カーブ別解析スキップ → デフォルトを使う
        curve2 = {"label": "カーブ②", "cx": 89301, "cy": 43085, "margin": 60}
    # margin はこのスクリプトでは少し広めを既定とする (元コードは 60 だった)
    margin = max(int(curve2.get("margin", 20)), 60)
    return curve2, float(margin)


def _build_figure(  # noqa: PLR0915
    *,
    cfg: RuntimeConfig,
    kin,
    vel,
    accel,
    cmd,
    steer,
    map_ways: list,
    curve2: dict,
    bbox_margin: float,
) -> tuple[plt.Figure, dict]:
    """7軸の解析プロットを作成。出力に必要な統計値も dict で返す。"""
    cx = float(curve2["cx"])
    cy = float(curve2["cy"])

    # ステア追従誤差
    steer_cmd_interp = np.interp(
        steer["tr"].values, cmd["tr"].values, np.degrees(cmd["cmd_steer"].values)
    )
    steer_resp_deg = np.degrees(steer["steer"].values)
    steer_error = steer_resp_deg - steer_cmd_interp  # 正=オーバー / 負=アンダー

    # ステア変化率
    dt_steer = np.diff(steer["t"].values)
    dt_steer = np.where(dt_steer > 0, dt_steer, np.nan)
    dsteer_dt = np.diff(steer_resp_deg) / dt_steer
    dt_cmd = np.diff(cmd["t"].values)
    dt_cmd = np.where(dt_cmd > 0, dt_cmd, np.nan)
    dcmd_dt = np.diff(np.degrees(cmd["cmd_steer"].values)) / dt_cmd

    fig = plt.figure(figsize=(18, 16))
    fig.suptitle(
        f"{cfg.scenario_name}\n実機 {curve2['label']} 詳細解析（アンダーステア / オーバーステア検証）",
        fontsize=12,
    )
    gs = fig.add_gridspec(3, 3, height_ratios=[1.5, 1.0, 1.0], hspace=0.45, wspace=0.35)
    ax_map = fig.add_subplot(gs[0, :])
    ax_steer = fig.add_subplot(gs[1, 0])
    ax_err = fig.add_subplot(gs[1, 1])
    ax_rate = fig.add_subplot(gs[1, 2])
    ax_vel = fig.add_subplot(gs[2, 0])
    ax_acc = fig.add_subplot(gs[2, 1])
    ax_cumul = fig.add_subplot(gs[2, 2])

    # --- 軌跡 ---
    x_range = (cx - bbox_margin, cx + bbox_margin)
    y_range = (cy - bbox_margin, cy + bbox_margin)
    for pts in map_ways_in_bbox(map_ways, x_range, y_range):
        ax_map.plot(pts[:, 0], pts[:, 1], color="#cccccc", lw=0.5, zorder=1)

    ax_map.plot(
        kin["x"].values, kin["y"].values, color="black", lw=2.5, label="実機 軌跡", zorder=3
    )
    for t_mark in [0, 5, 10, 15, 20, 25]:
        row = kin.iloc[(kin["tr"] - t_mark).abs().argsort().iloc[:1]]
        if row.empty:
            continue
        ax_map.scatter(
            row["x"], row["y"], c="red" if t_mark == 0 else "gray", s=50, zorder=5
        )
        ax_map.annotate(
            f"t={t_mark}s",
            (row["x"].values[0], row["y"].values[0]),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=7,
            color="gray",
        )
    ax_map.set_xlim(*x_range)
    ax_map.set_ylim(*y_range)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.set_title(f"実機 {curve2['label']} 付近の軌跡（赤★=発進点、t=0〜25s）", fontsize=10)
    ax_map.legend(fontsize=9)
    ax_map.grid(True, lw=0.4, alpha=0.5)

    # --- ステア指令 vs 応答 ---
    ax_steer.plot(
        cmd["tr"].values,
        np.degrees(cmd["cmd_steer"].values),
        color="tab:blue",
        lw=1.5,
        ls="--",
        label="指令 cmd_steer",
    )
    ax_steer.plot(
        steer["tr"].values, steer_resp_deg, color="black", lw=2.0, label="応答 steering_tire_angle"
    )
    ax_steer.axvline(0, color="gray", lw=0.8, ls="--")
    ax_steer.set_title("ステアリング角: 指令 vs 応答", fontsize=10)
    ax_steer.set_xlabel("発進からの時刻 [s]")
    ax_steer.set_ylabel("deg")
    ax_steer.legend(fontsize=8)
    ax_steer.grid(True, lw=0.4, alpha=0.5)

    # --- 追従誤差 ---
    tr_s = steer["tr"].values
    ax_err.fill_between(
        tr_s, steer_error, 0, where=(steer_error >= 0),
        color="tab:red", alpha=0.3, label="オーバーステア（+）",
    )
    ax_err.fill_between(
        tr_s, steer_error, 0, where=(steer_error < 0),
        color="tab:blue", alpha=0.3, label="アンダーステア（−）",
    )
    ax_err.plot(tr_s, steer_error, color="black", lw=1.0)
    ax_err.axhline(0, color="gray", lw=0.8)
    ax_err.axvline(0, color="gray", lw=0.8, ls="--")
    ax_err.set_title("ステア追従誤差 (応答 − 指令)", fontsize=10)
    ax_err.set_xlabel("発進からの時刻 [s]")
    ax_err.set_ylabel("deg")
    ax_err.legend(fontsize=8)
    ax_err.grid(True, lw=0.4, alpha=0.5)

    rmse_err = float(np.sqrt(np.nanmean(steer_error**2)))
    mean_err = float(np.nanmean(steer_error))
    ax_err.text(
        0.02, 0.95,
        f"RMSE={rmse_err:.3f}°\n平均={mean_err:.3f}°",
        transform=ax_err.transAxes, fontsize=8, va="top",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7),
    )

    # --- ステア変化率 ---
    ax_rate.plot(steer["tr"].values[1:], dsteer_dt, color="black", lw=1.5, label="応答 steer rate")
    ax_rate.plot(
        cmd["tr"].values[1:], dcmd_dt, color="tab:blue", lw=1.2, ls="--", label="指令 steer rate"
    )
    ax_rate.axhline(0, color="gray", lw=0.5)
    ax_rate.axvline(0, color="gray", lw=0.8, ls="--")
    ax_rate.set_title("ステア変化率 [deg/s]", fontsize=10)
    ax_rate.set_xlabel("発進からの時刻 [s]")
    ax_rate.set_ylabel("deg/s")
    ax_rate.legend(fontsize=8)
    ax_rate.grid(True, lw=0.4, alpha=0.5)
    ax_rate.set_ylim(-30, 30)

    # --- 速度 ---
    ax_vel.plot(vel["tr"].values, vel["lon_vel"].values, color="black", lw=2.0, label="実機 速度")
    if not cmd.empty:
        ax_vel.plot(
            cmd["tr"].values, cmd["cmd_vel"].values,
            color="tab:blue", lw=1.5, ls="--", label="速度指令",
        )
    ax_vel.axvline(0, color="gray", lw=0.8, ls="--")
    ax_vel.set_title("速度", fontsize=10)
    ax_vel.set_xlabel("発進からの時刻 [s]")
    ax_vel.set_ylabel("m/s")
    ax_vel.legend(fontsize=8)
    ax_vel.grid(True, lw=0.4, alpha=0.5)

    # --- 加速度 ---
    ax_acc.plot(
        accel["tr"].values, accel["accel"].values, color="black", lw=2.0, label="実機 加速度 (IMU)"
    )
    if not cmd.empty:
        ax_acc.plot(
            cmd["tr"].values, cmd["cmd_accel"].values,
            color="tab:blue", lw=1.5, ls="--", label="加速度指令",
        )
    ax_acc.axhline(0, color="gray", lw=0.5)
    ax_acc.axvline(0, color="gray", lw=0.8, ls="--")
    ax_acc.set_title("加速度", fontsize=10)
    ax_acc.set_xlabel("発進からの時刻 [s]")
    ax_acc.set_ylabel("m/s²")
    ax_acc.legend(fontsize=8)
    ax_acc.grid(True, lw=0.4, alpha=0.5)

    # --- 累積絶対誤差 ---
    dt_arr = np.diff(steer["t"].values, prepend=steer["t"].values[0])
    dt_arr[0] = dt_arr[1] if len(dt_arr) > 1 else 0.02
    steer_err_abs_cumul_t = np.nancumsum(np.abs(steer_error) * dt_arr)
    ax_cumul.plot(steer["tr"].values, steer_err_abs_cumul_t, color="black", lw=2.0)
    ax_cumul.axvline(0, color="gray", lw=0.8, ls="--")
    ax_cumul.set_title("ステア追従誤差 累積絶対値 [deg·s]", fontsize=10)
    ax_cumul.set_xlabel("発進からの時刻 [s]")
    ax_cumul.set_ylabel("deg·s")
    ax_cumul.grid(True, lw=0.4, alpha=0.5)

    total_cumul = (
        float(steer_err_abs_cumul_t[-1]) if len(steer_err_abs_cumul_t) else 0.0
    )
    ax_cumul.text(
        0.02, 0.95,
        f"合計={total_cumul:.2f}°·s",
        transform=ax_cumul.transAxes, fontsize=8, va="top",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7),
    )

    stats = {
        "rmse": rmse_err,
        "mean": mean_err,
        "max": float(np.nanmax(steer_error)),
        "min": float(np.nanmin(steer_error)),
        "cumul": total_cumul,
        "steer_error": steer_error,
        "steer": steer,
    }
    return fig, stats


def main() -> None:
    parser = argparse.ArgumentParser(description="実機 カーブ② 詳細解析")
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    real_bag = _resolve_real_bag(cfg.lite_dir)
    print(f"Loading: {real_bag}")

    df_mode = load_operation_mode(real_bag)
    df_vel_raw = load_velocity(real_bag)
    t0_ns = find_autonomous_start(df_mode, df_vel_raw)

    df_vel = align_time(df_vel_raw, t0_ns)
    df_steer = align_time(load_steering(real_bag), t0_ns)
    df_kin = align_time(load_kinematic(real_bag), t0_ns)
    df_accel = align_time(load_accel(real_bag), t0_ns)
    df_cmd = align_time(load_cmd(real_bag, REAL_CMD_TOPIC_CANDIDATES), t0_ns)

    t_launch = find_curve2_launch(df_vel, window=cfg.curve2_window)
    if t_launch is None:
        print("ERROR: カーブ② 前の停止を検出できませんでした", file=sys.stderr)
        sys.exit(1)
    print(f"カーブ② 発進 t={t_launch:.1f}s")

    vel = _clip_to_window(df_vel, t_launch)
    steer = _clip_to_window(df_steer, t_launch)
    kin = _clip_to_window(df_kin, t_launch)
    accel = _clip_to_window(df_accel, t_launch)
    cmd = _clip_to_window(df_cmd, t_launch)

    map_ways = load_map_ways(cfg.map_osm_path) if cfg.map_osm_path else []
    print(f"地図 way 数: {len(map_ways)}")

    curve2, bbox_margin = _load_curve2_window(cfg)
    fig, stats = _build_figure(
        cfg=cfg,
        kin=kin, vel=vel, accel=accel, cmd=cmd, steer=steer,
        map_ways=map_ways, curve2=curve2, bbox_margin=bbox_margin,
    )

    cfg.figs_dir.mkdir(parents=True, exist_ok=True)
    add_params_annotation(fig)
    for ext in ("png", "pdf"):
        p = cfg.figs_dir / f"real_curve2_detail.{ext}"
        fig.savefig(str(p), dpi=150, bbox_inches="tight")
        print(f"Saved: {p}")
    plt.close(fig)

    # 統計サマリ
    print("\n=== カーブ② ステア追従誤差サマリ ===")
    print(f"  RMSE:   {stats['rmse']:.4f} deg")
    print(f"  平均誤差: {stats['mean']:.4f} deg (正=オーバーステア / 負=アンダーステア)")
    print(f"  最大誤差: {stats['max']:.4f} deg")
    print(f"  最小誤差: {stats['min']:.4f} deg")
    print(f"  累積誤差: {stats['cumul']:.2f} deg·s")

    # フェーズ別誤差
    steer = stats["steer"]
    steer_error = stats["steer_error"]
    if not steer.empty:
        base = steer.index[0]
        for label, lo, hi in [("t=0〜5s", 0.0, 5.0), ("t=5〜15s", 5.0, 15.0), ("t=15〜25s", 15.0, 25.0)]:
            mask = (steer["tr"] > lo) & (steer["tr"] <= hi) if lo > 0 else (steer["tr"] >= lo) & (steer["tr"] <= hi)
            sel = steer[mask]
            if sel.empty:
                continue
            i0 = sel.index[0] - base
            i1 = sel.index[-1] - base + 1
            ph_err = steer_error[i0:i1]
            if len(ph_err) == 0:
                continue
            print(f"  [{label}] 平均誤差={float(np.nanmean(ph_err)):.3f}°")

    print("\nDone.")


if __name__ == "__main__":
    main()
