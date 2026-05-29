#!/usr/bin/env python3
"""カーブ② 乖離詳細診断スクリプト.

軌跡乖離の原因を特定するため、1秒分解能で位置・速度・ステア・yaw の比較を行う。
乖離の縦横成分分解と、プランナー速度差の寄与を定量化する。

入力: real.lite(.mcap) + sim_curve2.lite(.mcap) (どちらかが無い場合は中断)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ..lib._events import find_autonomous_start, find_curve2_launch, find_sim_launch
from ..lib._io import align_time, load_cmd, load_kinematic, load_operation_mode, load_steering, load_velocity
from ..lib._params_utils import add_params_annotation, setup_jp_font
from ..lib._runtime_config import RuntimeConfig, add_common_cli_arguments, build_runtime_config

setup_jp_font()


REAL_CMD_TOPIC_CANDIDATES = [
    "/control/command/control_cmd",
    "/sub/control/command/control_cmd",
]
SIM_CMD_TOPIC_CANDIDATES = [
    "/control/trajectory_follower/control_cmd",
    "/control/command/control_cmd",
]


def _resolve_bag(lite_dir: Path, name_stem: str) -> Path | None:
    """`<name_stem>.lite.mcap` または `<name_stem>.lite` をこの順で解決。"""
    for cand in (lite_dir / f"{name_stem}.lite.mcap", lite_dir / f"{name_stem}.lite"):
        if cand.exists():
            return cand
    return None


def _load_one(
    bag: Path,
    *,
    is_real: bool,
    cmd_topic: list[str],
    curve2_window: tuple[float, float],
) -> dict:
    """1つの bag から velocity/kinematic/cmd/steer を読み、tr 列を付与した dict を返す。"""
    df_mode = load_operation_mode(bag)
    df_vel_raw = load_velocity(bag)
    if is_real:
        t0_ns = find_autonomous_start(df_mode, df_vel_raw)
    elif not df_vel_raw.empty:
        t0_ns = int(df_vel_raw["t_ns"].iloc[0])
    else:
        raise ValueError(f"{bag} の velocity が空")
    df_vel = align_time(df_vel_raw, t0_ns)

    t_launch = find_curve2_launch(df_vel, window=curve2_window)
    if t_launch is None:
        t_launch = find_sim_launch(df_vel, threshold=0.5, min_t=5.0)
        if t_launch is None:
            t_launch = 0.0

    df_kin = align_time(load_kinematic(bag), t0_ns)
    df_cmd = align_time(load_cmd(bag, cmd_topic), t0_ns)
    df_steer = align_time(load_steering(bag), t0_ns)

    for df in (df_vel, df_kin, df_cmd, df_steer):
        df["tr"] = df["t"] - t_launch

    return {
        "t_launch": float(t_launch),
        "vel": df_vel,
        "kin": df_kin,
        "cmd": df_cmd,
        "steer": df_steer,
    }


def _at(df: pd.DataFrame, tr_target: float, col: str) -> float:
    """指定 tr に最も近い行の col 値を返す。"""
    if df.empty:
        return float("nan")
    idx = (df["tr"] - tr_target).abs().idxmin()
    return float(df.loc[idx, col])


def decompose_deviation(
    real_kin: pd.DataFrame, sim_kin: pd.DataFrame, tr_target: float
) -> tuple[float, float, float, float, float, float, float, float]:
    """実機 yaw 方向を基準に縦/横方向の乖離を分解する。"""
    idx_r = (real_kin["tr"] - tr_target).abs().idxmin()
    rx, ry, ryaw = real_kin.loc[idx_r, ["x", "y", "yaw"]]
    idx_s = (sim_kin["tr"] - tr_target).abs().idxmin()
    sx, sy = sim_kin.loc[idx_s, ["x", "y"]]
    dx, dy = sx - rx, sy - ry
    dist = math.sqrt(dx**2 + dy**2)
    lon = dx * math.cos(ryaw) + dy * math.sin(ryaw)
    lat = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
    return dist, lon, lat, rx, ry, ryaw, sx, sy


def print_diagnosis(real: dict, sim: dict) -> None:
    print("\n" + "=" * 80)
    print("カーブ② 乖離詳細診断（1s分解能）")
    print("=" * 80)
    print(
        f"{'t[s]':>5} | {'実機速度':>8} {'シム速度':>8} {'速度差':>7} | "
        f"{'実機cmd':>8} {'シムcmd':>8} {'cmd差':>7} | "
        f"{'実機steer':>9} {'シムsteer':>9} | "
        f"{'乖離[m]':>7} {'縦[m]':>7} {'横[m]':>7}"
    )
    print("-" * 110)

    for tr in range(-2, 28):
        r_v = _at(real["vel"], tr, "lon_vel")
        s_v = _at(sim["vel"], tr, "lon_vel")
        r_cmd = _at(real["cmd"], tr, "cmd_vel")
        s_cmd = _at(sim["cmd"], tr, "cmd_vel")
        r_str = math.degrees(_at(real["steer"], tr, "steer"))
        s_str = math.degrees(_at(sim["steer"], tr, "steer"))
        dist, lon, lat, *_ = decompose_deviation(real["kin"], sim["kin"], tr)

        dv = s_v - r_v
        dc = s_cmd - r_cmd
        print(
            f"{tr:>5} | {r_v:>8.3f} {s_v:>8.3f} {dv:>+7.3f} | "
            f"{r_cmd:>8.3f} {s_cmd:>8.3f} {dc:>+7.3f} | "
            f"{r_str:>9.2f} {s_str:>9.2f} | "
            f"{dist:>7.3f} {lon:>+7.3f} {lat:>+7.3f}"
        )


def plot_detailed(real: dict, sim: dict, cfg: RuntimeConfig) -> None:
    t_vec = np.linspace(-2, 27, 300)

    dists, lons, lats = [], [], []
    for tr in t_vec:
        d, lon, lat, *_ = decompose_deviation(real["kin"], sim["kin"], tr)
        dists.append(d)
        lons.append(lon)
        lats.append(lat)
    dists = np.array(dists)
    lons = np.array(lons)
    lats = np.array(lats)

    yaw_diff = []
    for tr in t_vec:
        ry = _at(real["kin"], tr, "yaw")
        sy = _at(sim["kin"], tr, "yaw")
        diff = math.degrees(sy - ry)
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        yaw_diff.append(diff)
    yaw_diff = np.array(yaw_diff)

    rv = np.interp(t_vec, real["vel"]["tr"].values, real["vel"]["lon_vel"].values, left=np.nan, right=np.nan)
    sv = np.interp(t_vec, sim["vel"]["tr"].values, sim["vel"]["lon_vel"].values, left=np.nan, right=np.nan)
    rc = np.interp(t_vec, real["cmd"]["tr"].values, real["cmd"]["cmd_vel"].values, left=np.nan, right=np.nan)
    sc = np.interp(t_vec, sim["cmd"]["tr"].values, sim["cmd"]["cmd_vel"].values, left=np.nan, right=np.nan)
    rs = np.degrees(
        np.interp(t_vec, real["steer"]["tr"].values, real["steer"]["steer"].values, left=np.nan, right=np.nan)
    )
    ss = np.degrees(
        np.interp(t_vec, sim["steer"]["tr"].values, sim["steer"]["steer"].values, left=np.nan, right=np.nan)
    )

    fig, axes = plt.subplots(5, 1, figsize=(14, 22), sharex=True)
    fig.suptitle(f"{cfg.scenario_name}\nカーブ② 乖離詳細診断", fontsize=13)

    ax = axes[0]
    ax.plot(t_vec, rv, "k-", lw=2, label="実機 actual")
    ax.plot(t_vec, sv, color="#e05c00", lw=2, ls="--", label="シム actual")
    ax.plot(t_vec, rc, "k:", lw=1.2, alpha=0.6, label="実機 cmd")
    ax.plot(t_vec, sc, color="#e05c00", lw=1.2, ls=":", alpha=0.6, label="シム cmd")
    ax.fill_between(t_vec, rv, sv, alpha=0.15, color="red", label="速度差")
    ax.set_ylabel("速度 [m/s]")
    ax.set_title("速度（actual/cmd）")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    ax = axes[1]
    ax.plot(t_vec, sv - rv, color="#e05c00", lw=2, ls="--", label="actual差 (sim-real)")
    ax.plot(t_vec, sc - rc, color="#e05c00", lw=1.5, ls=":", alpha=0.7, label="cmd差 (sim-real)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("m/s")
    ax.set_title("速度差（シム − 実機）")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    ax = axes[2]
    ax.plot(t_vec, rs, "k-", lw=2, label="実機 actual")
    ax.plot(t_vec, ss, color="#e05c00", lw=2, ls="--", label="シム actual")
    ax.set_ylabel("deg")
    ax.set_title("ステアリング角応答")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    ax = axes[3]
    ax.plot(t_vec, yaw_diff, color="purple", lw=2, label="yaw差 (sim-real) [deg]")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("deg")
    ax.set_title("ヨー角差（シム − 実機）")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    ax = axes[4]
    ax.plot(t_vec, dists, "r-", lw=2, label="総乖離距離 [m]")
    ax.plot(t_vec, lons, color="blue", lw=1.5, ls="--", label="縦方向 (実機前方正) [m]")
    ax.plot(t_vec, lats, color="green", lw=1.5, ls="-.", label="横方向 (左正) [m]")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("m")
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_title("軌跡乖離の縦横分解（実機進行方向基準）")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    fig.tight_layout()
    add_params_annotation(fig)
    cfg.figs_dir.mkdir(parents=True, exist_ok=True)
    out = cfg.figs_dir / "c2_diagnosis.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  保存: {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description="カーブ② 乖離詳細診断")
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)

    real_bag = _resolve_bag(cfg.lite_dir, "real")
    sim_bag = _resolve_bag(cfg.lite_dir, "sim_curve2")
    if real_bag is None:
        print(f"ERROR: real lite bag が見つかりません: {cfg.lite_dir}", file=sys.stderr)
        sys.exit(1)
    if sim_bag is None:
        print(
            f"ERROR: sim_curve2 lite bag が見つかりません: {cfg.lite_dir}\n"
            "  `make sim_curve2` でシム出力を生成してください。",
            file=sys.stderr,
        )
        sys.exit(1)

    print("=== データ読み込み ===")
    print(f"  実機: {real_bag}")
    real = _load_one(real_bag, is_real=True, cmd_topic=REAL_CMD_TOPIC_CANDIDATES,
                     curve2_window=cfg.curve2_window)
    print(f"  [実機] カーブ② 発進 t={real['t_launch']:.1f}s")

    print(f"  シム: {sim_bag}")
    sim = _load_one(sim_bag, is_real=False, cmd_topic=SIM_CMD_TOPIC_CANDIDATES,
                    curve2_window=cfg.curve2_window)
    print(f"  [シム (Curve2)] 発進 t={sim['t_launch']:.1f}s")

    print_diagnosis(real, sim)
    plot_detailed(real, sim, cfg)


if __name__ == "__main__":
    main()
