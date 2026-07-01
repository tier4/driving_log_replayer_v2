#!/usr/bin/env python3
"""Stage 8: DiffusionPlanner 計画軌跡の比較.

「計画」レベルの乖離 (sim の DiffusionPlanner が実機と同じ軌跡を出すか) を、車両応答
(Stage 4)・dynamics (Stage 5/7) から分離して可視化する独立ステージ。Stage 1 が生成した
実機 lite と Stage 3 が生成した sim lite の両方が持つ DP 出力軌跡を比較する。

`--target` で出力を切り替える:
  - `real_actual`     : 実機 DP出力 vs シム DP出力 を直接比較
                        (交通信号・追跡物体数の補助情報を含む)
  - `final_planning`  : 実機 DP出力 vs 最終 planning/trajectory を比較
                        (trajectory_velocity_optimizer の影響を可視化)
  - `both` (既定)     : 両方

比較の時刻基準は「発進 (find_sim_launch による発進検出)」。
evaluator_node は Stage 7 の後に env のみで実行する (追加設定不要)。比較対象の sim run は
lite/ 配下の sim_*.lite を自動検出する (sim_normal を優先)。
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from .lib._events import find_autonomous_start, find_sim_launch
from .lib._io import (
    DP_TRAJ_TOPIC as DP_TOPIC,
    iter_bag_messages,
    load_velocity,
    resolve_lite_bag,
    resolve_primary_sim_bag,
    sim_tag_from_bag,
)
from .lib._fig_io import write_fig_json
from .lib._figures import (
    build_fig_dp_real_vs_sim,
    build_fig_dp_vs_actual,
    build_fig_dp_vs_final_traj,
)
from .lib._provenance import format_provenance_line, read_provenance
from .lib._runtime_config import RuntimeConfig, add_common_cli_arguments, build_runtime_config


FINAL_TOPIC = "/planning/trajectory"
SIG_TOPIC = "/perception/traffic_light_recognition/traffic_signals"
TRACKED_OBJECTS_TOPIC = "/perception/object_recognition/tracking/objects"


def _resolve_t0_and_launch(
    bag: Path, *, is_real: bool
) -> tuple[int, float]:
    """bag から t0 (ns) と発進時刻 t_launch (s) を返す。"""
    from .lib._io import load_operation_mode

    df_vel = load_velocity(bag)
    if is_real:
        df_mode = load_operation_mode(bag)
        t0_ns = find_autonomous_start(df_mode, df_vel)
    elif not df_vel.empty:
        t0_ns = int(df_vel["t_ns"].iloc[0])
    else:
        raise ValueError(f"{bag} の velocity が空")

    df_vel = df_vel.copy()
    df_vel["t"] = (df_vel["t_ns"] - t0_ns) / 1e9
    df_vel = df_vel[df_vel["t"] >= 0].reset_index(drop=True)

    t_launch = find_sim_launch(df_vel, threshold=0.5, min_t=5.0)
    if t_launch is None:
        t_launch = 0.0
    return t0_ns, float(t_launch)


def load_traj_frames(
    bag: Path,
    topic: str,
    t0_ns: int,
    t_launch: float,
    *,
    pre_s: float = 1.0,
    window_s: float = 15.0,
    max_dist: float = 100.0,
) -> list[dict]:
    """発進前後の軌跡フレーム (時刻ごとの points → distances/velocities) を返す。"""
    t_launch_ns = t0_ns + int(t_launch * 1e9)
    t_start_ns = t_launch_ns - int(pre_s * 1e9)
    t_end_ns = t_launch_ns + int(window_s * 1e9)

    frames = []
    for t_msg, ros in iter_bag_messages(bag, [topic]):
        if t_msg < t_start_ns or t_msg > t_end_ns:
            continue
        t_rel = (t_msg - t0_ns) / 1e9 - t_launch
        pts = ros.points
        if not pts:
            continue
        x0, y0 = pts[0].pose.position.x, pts[0].pose.position.y
        dists, vels = [], []
        for pt in pts:
            dx = pt.pose.position.x - x0
            dy = pt.pose.position.y - y0
            d = math.sqrt(dx**2 + dy**2)
            if d > max_dist:
                break
            dists.append(d)
            vels.append(pt.longitudinal_velocity_mps)
        frames.append({
            "t_rel": t_rel,
            "t_ns": t_msg,
            "dists": np.array(dists),
            "vels": np.array(vels),
        })
    return frames


def frames_to_series(frames: list[dict], t_vec: np.ndarray, d_target: float) -> np.ndarray:
    """各 t_vec 時刻における d_target 地点の計画速度（最近傍フレーム + 距離補間）。"""
    if not frames:
        return np.full(len(t_vec), np.nan)
    sorted_f = sorted(frames, key=lambda x: x["t_rel"])
    result = []
    for t in t_vec:
        closest = min(sorted_f, key=lambda f: abs(f["t_rel"] - t), default=None)
        if closest is None or abs(closest["t_rel"] - t) > 0.15:
            result.append(np.nan)
            continue
        fr = closest
        if len(fr["dists"]) > 0 and d_target <= fr["dists"][-1]:
            v = float(np.interp(d_target, fr["dists"], fr["vels"]))
        else:
            v = np.nan
        result.append(v)
    return np.array(result)


def _value_at(frames: list[dict], t_target: float, d_target: float, *, tol: float = 0.2) -> float:
    """単一時刻 + 距離の計画速度を取得。許容時刻差 `tol` 以内のフレームがなければ NaN。"""
    if not frames:
        return float("nan")
    sorted_f = sorted(frames, key=lambda x: x["t_rel"])
    closest = min(sorted_f, key=lambda f: abs(f["t_rel"] - t_target), default=None)
    if closest is None or abs(closest["t_rel"] - t_target) > tol:
        return float("nan")
    fr = closest
    if len(fr["dists"]) > 0 and d_target <= fr["dists"][-1]:
        return float(np.interp(d_target, fr["dists"], fr["vels"]))
    return float("nan")


# ---------------------------------------------------------------------------
# real_actual target — 実機 DP vs シム DP
# ---------------------------------------------------------------------------


def _count_tracked_objects(
    bag: Path, t0_ns: int, t_launch: float, *, pre_s: float = 5.0, window_s: float = 20.0
) -> pd.DataFrame:
    """発進前後の追跡物体数を時系列で取得。"""
    t_start_ns = t0_ns + int((t_launch - pre_s) * 1e9)
    t_end_ns = t0_ns + int((t_launch + window_s) * 1e9)
    rows = []
    for t_msg, ros in iter_bag_messages(bag, [TRACKED_OBJECTS_TOPIC]):
        if t_msg < t_start_ns or t_msg > t_end_ns:
            continue
        t_rel = (t_msg - t0_ns) / 1e9 - t_launch
        rows.append({"t_rel": t_rel, "n_objects": len(ros.objects)})
    return pd.DataFrame(rows)


def _load_traffic_signals(
    bag: Path, t0_ns: int, t_launch: float, *, window_s: float = 50.0
) -> pd.DataFrame:
    t_end_ns = t0_ns + int((t_launch + window_s) * 1e9)
    rows = []
    for t_msg, ros in iter_bag_messages(bag, [SIG_TOPIC]):
        if t_msg > t_end_ns:
            continue
        t_rel = (t_msg - t0_ns) / 1e9
        for group in ros.traffic_light_groups:
            for elem in group.elements:
                rows.append({
                    "t": t_rel,
                    "group_id": group.traffic_light_group_id,
                    "color": elem.color,
                    "shape": elem.shape,
                    "status": elem.status,
                })
    return pd.DataFrame(rows)


def _run_real_actual(  # noqa: PLR0915
    cfg: RuntimeConfig, real_bag: Path, sim_bag: Path
) -> None:
    sim_name = sim_tag_from_bag(sim_bag)  # 凡例・タイトルに表示するシミュレータ名 (例: sim_normal)
    print("\n=== DiffusionPlanner 計画軌跡 実機 vs シム 直接比較 ===\n")

    t0_sim, tl_sim = _resolve_t0_and_launch(sim_bag, is_real=False)
    t0_real, tl_real = _resolve_t0_and_launch(real_bag, is_real=True)
    print(f"シム  : t_launch={tl_sim:.1f}s")
    print(f"実機  : t_launch={tl_real:.1f}s")

    print("\n--- シム 交通信号10583 状態推移 ---")
    df_sig = _load_traffic_signals(sim_bag, t0_sim, tl_sim)
    if df_sig.empty:
        print("  [警告] 交通信号データなし")
    else:
        sig_target = df_sig[df_sig["group_id"] == 10583]
        if not sig_target.empty:
            prev = None
            for _, row in sig_target.iterrows():
                state = (row["color"], row["status"])
                if state != prev:
                    color_name = {1: "RED", 2: "AMBER", 3: "GREEN", 0: "UNKNOWN"}.get(
                        row["color"], "?"
                    )
                    print(f"  t={row['t']:6.1f}s: 色={color_name} status={row['status']}")
                    prev = state
        else:
            print(f"  信号10583なし。存在グループ: {sorted(df_sig['group_id'].unique())[:10]}")

    print("\n--- 追跡物体数（発進前後） ---")
    df_obj_real = _count_tracked_objects(real_bag, t0_real, tl_real)
    df_obj_sim = _count_tracked_objects(sim_bag, t0_sim, tl_sim)
    for label, df in [("実機", df_obj_real), ("シム", df_obj_sim)]:
        if not df.empty:
            print(
                f"  {label}: 平均{df['n_objects'].mean():.1f}物体, "
                f"最大{df['n_objects'].max()}, 最小{df['n_objects'].min()}"
            )
        else:
            print(f"  {label}: 追跡物体データなし")

    print("\n--- DiffusionPlanner計画軌跡ロード ---")
    frames_sim = load_traj_frames(sim_bag, DP_TOPIC, t0_sim, tl_sim, window_s=15.0)
    frames_real = load_traj_frames(real_bag, DP_TOPIC, t0_real, tl_real, window_s=15.0)
    print(f"  シム: {len(frames_sim)} フレーム")
    print(f"  実機: {len(frames_real)} フレーム")

    print("\n--- DP計画速度 比較テーブル（t_rel=-1〜+10s） ---")
    print(f"{'':>4} | {'------シム------':^42} | {'------実機------':^42}")
    print(
        f"{'t[s]':>4} | {'d=0':>6} {'d=5':>6} {'d=10':>6} {'d=20':>6} {'d=30':>6} {'d=50':>6} | "
        f"{'d=0':>6} {'d=5':>6} {'d=10':>6} {'d=20':>6} {'d=30':>6} {'d=50':>6}"
    )
    print("  " + "-" * 95)
    for t_val in [-1.0, 0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0]:
        sim_row = [_value_at(frames_sim, t_val, d) for d in [0, 5, 10, 20, 30, 50]]
        real_row = [_value_at(frames_real, t_val, d) for d in [0, 5, 10, 20, 30, 50]]
        sim_str = " ".join(f"{v:>6.2f}" if not np.isnan(v) else f"{'nan':>6}" for v in sim_row)
        real_str = " ".join(f"{v:>6.2f}" if not np.isnan(v) else f"{'nan':>6}" for v in real_row)
        print(f"{t_val:>4.1f} | {sim_str} | {real_str}")

    cfg.figs_dir.mkdir(parents=True, exist_ok=True)
    t_vec = np.linspace(-1, 12, 200)
    d_targets = [0, 5, 10, 20]
    colors_d = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    plan_series = [
        {
            "d": d, "color": col,
            "v_sim": frames_to_series(frames_sim, t_vec, d),
            "v_real": frames_to_series(frames_real, t_vec, d),
        }
        for d, col in zip(d_targets, colors_d)
    ]

    def _profile_frames(frames):
        return [
            {"t_rel": fr["t_rel"], "dists": fr["dists"], "vels": fr["vels"]}
            for fr in sorted(frames, key=lambda x: x["t_rel"])
            if -1.0 <= fr["t_rel"] <= 10.0
        ]

    profiles = {"sim": _profile_frames(frames_sim), "real": _profile_frames(frames_real)}
    objects = {
        "real_t": df_obj_real["t_rel"].values if not df_obj_real.empty else None,
        "real_n": df_obj_real["n_objects"].values if not df_obj_real.empty else None,
        "sim_t": df_obj_sim["t_rel"].values if not df_obj_sim.empty else None,
        "sim_n": df_obj_sim["n_objects"].values if not df_obj_sim.empty else None,
    }
    fig = build_fig_dp_real_vs_sim(
        t_vec, plan_series, profiles, objects,
        sim_name=sim_name, scenario_name=cfg.scenario_name,
    )
    write_fig_json(fig, cfg.figs_dir / "dp_real_vs_sim")

    # actual 速度との比較
    print("\n--- DP計画速度(d=0) vs actual速度 ---")
    df_vel_sim = load_velocity(sim_bag)
    df_vel_real = load_velocity(real_bag)
    df_vel_sim["t_rel"] = (df_vel_sim["t_ns"] - t0_sim) / 1e9 - tl_sim
    df_vel_real["t_rel"] = (df_vel_real["t_ns"] - t0_real) / 1e9 - tl_real
    df_vel_sim = df_vel_sim[df_vel_sim["t_rel"] >= -1].reset_index(drop=True)
    df_vel_real = df_vel_real[df_vel_real["t_rel"] >= -1].reset_index(drop=True)

    panels = [
        {
            "label": label,
            "dp_v": frames_to_series(frames, t_vec, 0),
            "actual_t": df_v["t_rel"].values,
            "actual_v": df_v["lon_vel"].values,
        }
        for frames, df_v, label in [
            (frames_sim, df_vel_sim, sim_name),
            (frames_real, df_vel_real, "実機"),
        ]
    ]
    fig2 = build_fig_dp_vs_actual(t_vec, panels, scenario_name=cfg.scenario_name)
    cfg.figs_dir.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig2, cfg.figs_dir / "dp_vs_actual")


# ---------------------------------------------------------------------------
# final_planning target — 実機 DP vs 最終 planning/trajectory
# ---------------------------------------------------------------------------


def _run_final_planning(  # noqa: PLR0915
    cfg: RuntimeConfig, real_bag: Path, sim_bag: Path | None
) -> None:
    print("\n=== DP出力 vs 最終trajectory 速度比較 ===\n")
    sim_name = sim_tag_from_bag(sim_bag) if sim_bag is not None else "シム"

    t0_real, tl_real = _resolve_t0_and_launch(real_bag, is_real=True)
    print(f"実機 t_launch={tl_real:.1f}s")

    frames_dp = load_traj_frames(real_bag, DP_TOPIC, t0_real, tl_real)
    frames_final = load_traj_frames(real_bag, FINAL_TOPIC, t0_real, tl_real)
    print(f"  DP出力フレーム: {len(frames_dp)}")
    print(f"  最終軌跡フレーム: {len(frames_final)}")

    frames_sim_dp: list[dict] = []
    t0_sim = tl_sim = None
    if sim_bag is not None:
        t0_sim, tl_sim = _resolve_t0_and_launch(sim_bag, is_real=False)
        frames_sim_dp = load_traj_frames(sim_bag, DP_TOPIC, t0_sim, tl_sim)
        print(f"  シム t_launch={tl_sim:.1f}s, DP出力フレーム: {len(frames_sim_dp)}")
    else:
        print("  シム lite bag が無いためシム DP は比較対象外")

    t_vals = [-1.0, 0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 12.0]
    d_list = [0, 5, 10, 20, 30]

    print("\n--- 実機: DP出力 vs 最終trajectory（各距離の速度）---")
    print(f"{'t[s]':>5} | {'------ DP出力 ------':^35} | {'--- 最終traj ---':^35}")
    print(
        f"{'':>5} | "
        + " ".join(f"d={d}m" for d in d_list)
        + " | "
        + " ".join(f"d={d}m" for d in d_list)
    )
    print("-" * 90)
    for t_val in t_vals:
        dp_row = [_value_at(frames_dp, t_val, d) for d in d_list]
        fin_row = [_value_at(frames_final, t_val, d) for d in d_list]

        def fmt(v: float) -> str:
            return f"{v:>6.2f}" if not np.isnan(v) else f"{'nan':>6}"

        print(
            f"{t_val:>5.1f} | "
            + " ".join(fmt(v) for v in dp_row)
            + " | "
            + " ".join(fmt(v) for v in fin_row)
        )

    cfg.figs_dir.mkdir(parents=True, exist_ok=True)
    t_vec = np.linspace(-1, 13, 250)
    d_targets = [0, 5, 10, 20, 30]

    def _series(d):
        return {
            "d": d,
            "v_dp": frames_to_series(frames_dp, t_vec, d),
            "v_final": frames_to_series(frames_final, t_vec, d),
            "v_sim": frames_to_series(frames_sim_dp, t_vec, d),
        }

    top_panels = [_series(d) for d in d_targets[:3]]
    diff_series = [_series(d) for d in d_targets]

    df_vel_real = load_velocity(real_bag)
    df_vel_real["tr"] = (df_vel_real["t_ns"] - t0_real) / 1e9 - tl_real
    df_vel_real = df_vel_real[df_vel_real["tr"] >= -1]
    summary = {
        "real_t": df_vel_real["tr"].values, "real_v": df_vel_real["lon_vel"].values,
        "sim_t": None, "sim_v": None, "dp0_sim": None,
        "dp0_real": frames_to_series(frames_dp, t_vec, 0),
        "fin0_real": frames_to_series(frames_final, t_vec, 0),
    }
    if sim_bag is not None and t0_sim is not None:
        df_vel_sim = load_velocity(sim_bag)
        df_vel_sim["tr"] = (df_vel_sim["t_ns"] - t0_sim) / 1e9 - tl_sim
        df_vel_sim = df_vel_sim[df_vel_sim["tr"] >= -1]
        summary["sim_t"] = df_vel_sim["tr"].values
        summary["sim_v"] = df_vel_sim["lon_vel"].values
        summary["dp0_sim"] = frames_to_series(frames_sim_dp, t_vec, 0)

    fig = build_fig_dp_vs_final_traj(
        t_vec, top_panels, diff_series, summary,
        sim_name=sim_name, scenario_name=cfg.scenario_name,
    )
    write_fig_json(fig, cfg.figs_dir / "dp_vs_final_traj")

    print("\n--- optimizer補正量の要約（実機）---")
    print(f"{'t[s]':>5} | " + " ".join(f"{'d=' + str(d) + 'm':>7}" for d in d_list))
    print("-" * 50)
    for t_val in [0.0, 1.0, 3.0, 5.0, 7.0, 10.0]:
        corr = [
            _value_at(frames_final, t_val, d) - _value_at(frames_dp, t_val, d)
            for d in d_list
        ]

        def fmt(v: float) -> str:
            return f"{v:>+7.2f}" if not np.isnan(v) else f"{'nan':>7}"

        print(f"{t_val:>5.1f} | " + " ".join(fmt(v) for v in corr))


# ---------------------------------------------------------------------------
# entry
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="DiffusionPlanner 計画軌跡の比較")
    parser.add_argument(
        "--target",
        choices=("real_actual", "final_planning", "both"),
        default="both",
        help="出力切替。real_actual=実機DPvsシムDP, final_planning=実機DPvs最終traj, both=両方",
    )
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    real_bag = resolve_lite_bag(cfg.lite_dir, "real")
    sim_bag = resolve_primary_sim_bag(cfg.lite_dir)
    if real_bag is None:
        print(f"ERROR: real lite bag が見つかりません: {cfg.lite_dir}", file=sys.stderr)
        sys.exit(1)
    if sim_bag is not None:
        print(f"比較対象 sim lite: {sim_bag.name}")

    if args.target in ("real_actual", "both"):
        if sim_bag is None:
            print(
                "WARN: sim lite bag (sim_*.lite) が無いため real_actual target をスキップ",
                file=sys.stderr,
            )
        else:
            _run_real_actual(cfg, real_bag, sim_bag)

    if args.target in ("final_planning", "both"):
        _run_final_planning(cfg, real_bag, sim_bag)


if __name__ == "__main__":
    main()
