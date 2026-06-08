#!/usr/bin/env python3
"""実機ログ vs Godotシム vs 通常シム の三方比較プロット生成スクリプト.

Outputs: comparison/figures/*.svg (軌跡比較のみ *.html), comparison/report.md
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import warnings

import numpy as np
import pandas as pd

from .lib._events import (
    AUTONOMOUS_MODE as _AUTONOMOUS_MODE,
    find_autonomous_start as _find_autonomous_start,
    find_curve2_exit as _find_curve2_exit_pure,
    find_curve2_launch as _find_curve2_launch_pure,
    find_initial_launch as _find_initial_launch,
)
from .lib._io import (
    DP_TRAJ_TOPIC,
    align_time,
    cumulative_arc_length,
    filter_localization,
    iter_bag_messages,
    load_accel,
    load_cmd,
    load_kinematic,
    load_operation_mode,
    load_steering,
    load_velocity,
    nearest_point_distance,
    resolve_topic,
)
from .lib._fig_io import write_fig_json
from .lib._figures import (
    build_fig_curve_analysis,
    build_fig_curve_steering_detail,
    build_fig_curve_yaw_steer,
    build_fig_curves_closeup,
    build_fig_steer_response,
    build_fig_timeseries_resp_cmd,
    build_fig_vs_distance,
)
from .lib._map import load_map_ways, resolve_map_osm
from .lib._playback_viewer import plot_trajectory_playback
from .lib._provenance import format_provenance_line, read_provenance
from .lib._runtime_config import (
    RuntimeConfig,
    add_common_cli_arguments,
    build_runtime_config,
)

# ---------------------------------------------------------------------------
# 設定 — main() で `_apply_runtime_config()` から上書きされる
# ---------------------------------------------------------------------------

BASE = Path(os.environ.get("BEST_MODEL_BASE_DIR") or Path(__file__).parent)
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison"
FIGS_DIR = OUT_DIR / "figures"
SCENARIO_NAME = "real_log_sim_comparison"

# `_apply_runtime_config()` で書き換えられる
_CURVE2_INDEX = 1
_CURVE2_WINDOW: tuple[float, float] = (20.0, 120.0)

WHEELBASE = 5.15  # m — kinematic_state × steering から実データで推定

# 地図座標系でのカーブ中心（後方互換デフォルト: x2_dev/2231）
# None に設定するとカーブ別解析プロットをすべてスキップする。
CURVE_CENTERS: list | None = [
    {"label": "カーブ①（右折）", "cx": 89440, "cy": 43200, "margin": 20},
    {"label": "カーブ②（左折）", "cx": 89301, "cy": 43085, "margin": 20},
    {"label": "カーブ③（右折）", "cx": 89372, "cy": 42830, "margin": 40},
]

# ログ定義（path は main() が LITE_DIR を確定した後に `_rebuild_logs()` で補完）
# kinematic / accel は sub-less / sub-prefixed の両方を試す候補リスト形式。
# `_io.iter_to_df` がリストを受け取って bag に存在する最初の候補を使う。
_REAL_KIN_CANDIDATES = [
    "/localization/kinematic_state",
    "/sub/localization/kinematic_state",
]
_REAL_ACC_CANDIDATES = [
    "/localization/acceleration",
    "/sub/localization/acceleration",
]
_REAL_CMD_CANDIDATES = [
    "/control/command/control_cmd",
    "/sub/control/command/control_cmd",
]

_DEFAULT_LOG_SPECS: dict = {
    "実機": {
        "bag_dir": "real.lite",
        "kinematic": _REAL_KIN_CANDIDATES,
        "accel": _REAL_ACC_CANDIDATES,
        "cmd": _REAL_CMD_CANDIDATES,
        "color": "black",
        "lw": 2.5,
        "ls": "-",
        "marker": "o",
        "ms": 5,
    },
}

# sim run entry のテンプレ (color/ls/marker は _rebuild_logs で動的割当)
_SIM_LOG_SPEC_TEMPLATE: dict = {
    "kinematic": "/localization/kinematic_state",
    "accel": "/localization/acceleration",
    "cmd": "/control/trajectory_follower/control_cmd",
    "lw": 2.0,
    "ms": 5,
}

# sim run に巡回的に割り当てる視覚スタイル
_SIM_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
               "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]
_SIM_LINESTYLES = ["--", (0, (4, 2)), "-.", ":", (0, (3, 1, 1, 1))]
_SIM_MARKERS = ["^", "s", "D", "v", "P", "*", "X"]

# 時間軸重ね描きへの注記: t=0 の基準がログ間で異なり (real=AUTONOMOUS 遷移 / sim=速度
# fallback)、pacing も違うため、同一 t が同一地点を意味しない。走行距離基準は
# velocity_vs_distance.svg / steering_vs_distance.svg を参照。
_TIME_AXIS_NOTE = (
    "注: t=0 は各ログの初回発進 (initial launch)。実機の初期停止時間が長くても全ログが発進で揃う"
    "（pacing 差により同一 t は同一地点を意味しない。走行距離基準は *_vs_distance.svg を参照）"
)


def _rebuild_logs(
    lite_dir: Path,
    topic_overrides: dict | None = None,
    sim_runs_cfg=None,
) -> dict:
    """LITE_DIR とオプショナルなトピック上書き辞書、sim_runs_cfg から LOGS dict を生成する。

    bag_dir 名は `<stem>.lite.mcap` (単一ファイル) を優先し、なければ
    `<stem>.lite` (rosbag2 ディレクトリ) にフォールバック。`_io._iter_msgs`
    がどちらの形式も読めるが、main() の `path.exists()` チェックがあるため
    実体のあるパスを返す必要がある。

    sim_runs_cfg (SimRunsConfig) が渡された場合、各 run.tag を <tag>.lite として
    LOGS dict に動的追加する (色 / 線スタイル / マーカーは順番に自動割当)。
    """
    result = {}
    for label, spec in _DEFAULT_LOG_SPECS.items():
        stem = spec["bag_dir"]
        candidates = [lite_dir / f"{stem}.mcap", lite_dir / stem]
        path = next((c for c in candidates if c.exists()), candidates[0])
        entry = {**spec, "path": path}
        entry.setdefault("perfect", False)
        entry.setdefault("vehicle_model", "real")
        if topic_overrides and label in topic_overrides:
            entry.update(topic_overrides[label])
        result[label] = entry

    # sim_runs.yaml の各 tag を動的追加
    if sim_runs_cfg is not None:
        for i, run in enumerate(sim_runs_cfg.runs):
            stem = f"{run.tag}.lite"
            candidates = [lite_dir / f"{stem}.mcap", lite_dir / stem]
            path = next((c for c in candidates if c.exists()), candidates[0])
            entry = {
                **_SIM_LOG_SPEC_TEMPLATE,
                "bag_dir": stem,
                "color": _SIM_COLORS[i % len(_SIM_COLORS)],
                "ls": _SIM_LINESTYLES[i % len(_SIM_LINESTYLES)],
                "marker": _SIM_MARKERS[i % len(_SIM_MARKERS)],
                "path": path,
                "vehicle_model": run.vehicle_model,
                # PERFECT_TRAJECTORY_TRACKER は control_cmd を追従しないため
                # 操舵の cmd-vs-response 比較が意味を持たない（build_report で注記）。
                "perfect": run.vehicle_model.endswith("perfect_tracker"),
            }
            if topic_overrides and run.tag in topic_overrides:
                entry.update(topic_overrides[run.tag])
            result[run.tag] = entry

    return result


LOGS = _rebuild_logs(LITE_DIR)


# ---------------------------------------------------------------------------
# bag ローダー
# ---------------------------------------------------------------------------


# ローダー / 時刻整列 / 地図 OSM は `_io.py` と `_map.py` に移管済み。
# 互換性のため、既存呼び出し名を使い続ける補助ラッパだけ残す。


def _resolve_map_osm() -> Path | None:
    """main() で `_apply_runtime_config()` が cfg.map_osm_path を返すので不要だが、
    既存テスト互換用に env から再解決する。"""
    return resolve_map_osm(os.environ.get("MAP_OSM_PATH"))


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------


def _tr(d: dict, df: pd.DataFrame) -> np.ndarray:
    """発進相対時刻 (t - t_launch)。全ログ同一検出 (find_initial_launch) で整列するため、
    実機の初期停止が長くても比較時系列が発進時刻で揃う (実機だけ遅れる問題の一般対策)。
    `t_launch` は main で全ログ共通に算出し loaded[label] に格納済み (未設定なら 0)。"""
    return df["t"].to_numpy(dtype=float) - float(d.get("t_launch", 0.0))


def _resp_cmd_runs(data: dict, resp_key: str, resp_col: str, cmd_col: str, *, deg=False):
    """応答/指令 2 段時系列の run dict 列を整形する（_tr で発進整列）。"""
    runs: list[dict] = []
    for label, d in data.items():
        resp, cmd = d[resp_key], d["cmd"]
        if resp.empty:
            continue
        y_resp = np.degrees(resp[resp_col]) if deg else np.asarray(resp[resp_col])
        r = {
            "label": label, "color": d["color"], "lw": d["lw"], "ls": d["ls"],
            "t_resp": _tr(d, resp), "y_resp": y_resp, "t_cmd": None, "y_cmd": None,
        }
        if not cmd.empty:
            r["t_cmd"] = _tr(d, cmd)
            r["y_cmd"] = np.degrees(cmd[cmd_col]) if deg else np.asarray(cmd[cmd_col])
        runs.append(r)
    return runs


def plot_velocity(data: dict):
    fig = build_fig_timeseries_resp_cmd(
        _resp_cmd_runs(data, "velocity", "lon_vel", "cmd_vel"),
        title=f"{SCENARIO_NAME}<br>速度比較（指令 vs 応答）",
        resp_title="実応答 (VelocityReport.longitudinal_velocity)",
        cmd_title="指令 (control_cmd.longitudinal.velocity) ─ 点線・薄色",
        resp_ylabel="速度 [m/s]", cmd_ylabel="速度指令 [m/s]",
        note=_TIME_AXIS_NOTE,
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / "velocity")


def plot_acceleration(data: dict):
    fig = build_fig_timeseries_resp_cmd(
        _resp_cmd_runs(data, "accel", "accel", "cmd_accel"),
        title=f"{SCENARIO_NAME}<br>加速度比較（指令 vs 応答）",
        resp_title="実応答 (localization/acceleration.accel.accel.linear.x)",
        cmd_title="指令 (control_cmd.longitudinal.acceleration) ─ 点線・薄色",
        resp_ylabel="加速度 [m/s²]", cmd_ylabel="加速度指令 [m/s²]",
        note=_TIME_AXIS_NOTE,
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / "acceleration")


def plot_steering(data: dict):
    fig = build_fig_timeseries_resp_cmd(
        _resp_cmd_runs(data, "steering", "steer", "cmd_steer", deg=True),
        title=f"{SCENARIO_NAME}<br>ステアリング比較（指令 vs 応答）",
        resp_title="実応答 (SteeringReport.steering_tire_angle)",
        cmd_title="指令 (control_cmd.lateral.steering_tire_angle) ─ 点線・薄色",
        resp_ylabel="ステア角 [deg]", cmd_ylabel="ステア指令 [deg]",
        note=_TIME_AXIS_NOTE,
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / "steering")


def _distance_of(t_query, df_kin: pd.DataFrame):
    """align_time 済み kinematic から累積走行距離 s(t) を作り、t_query における距離を返す。

    kinematic は filter_localization 済み (無効フレーム除去済み) を前提とする。
    範囲外は端点にクランプ (np.interp 既定)。kinematic 空なら None。
    """
    if df_kin.empty:
        return None
    s = cumulative_arc_length(df_kin["x"].to_numpy(), df_kin["y"].to_numpy())
    return np.interp(np.asarray(t_query, dtype=float), df_kin["t"].to_numpy(), s)


def plot_velocity_vs_distance(data: dict):
    """速度応答を走行距離 (arc-length) 基準で重ね描き。

    時間軸では pacing 差・初期化オフセットが「時間ずれ」に見えるが、距離基準にすると
    各ログが経路上のどこまで進んだか・どこで停止したかが露出する (例: sim の早期停止)。
    """
    runs: list[dict] = []
    for label, d in data.items():
        vel, kin = d["velocity"], d["kinematic"]
        if vel.empty or kin.empty:
            continue
        s = _distance_of(vel["t"], kin)
        if s is None:
            continue
        runs.append({
            "label": label, "color": d["color"], "lw": d["lw"], "ls": d["ls"],
            "s": s, "y": np.asarray(vel["lon_vel"]),
        })
    fig = build_fig_vs_distance(
        runs,
        title=f"{SCENARIO_NAME}<br>速度比較（走行距離基準）",
        subplot_title="実応答速度 vs 走行距離（早期停止・初期化オフセットを露出）",
        ylabel="速度 [m/s]",
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / "velocity_vs_distance")


def plot_steering_vs_distance(data: dict):
    """操舵応答を走行距離 (arc-length) 基準で重ね描き（pacing 差を除いた形状比較）。"""
    runs: list[dict] = []
    for label, d in data.items():
        steer, kin = d["steering"], d["kinematic"]
        if steer.empty or kin.empty:
            continue
        s = _distance_of(steer["t"], kin)
        if s is None:
            continue
        runs.append({
            "label": label, "color": d["color"], "lw": d["lw"], "ls": d["ls"],
            "s": s, "y": np.degrees(np.asarray(steer["steer"])),
        })
    fig = build_fig_vs_distance(
        runs,
        title=f"{SCENARIO_NAME}<br>ステアリング比較（走行距離基準）",
        subplot_title="実応答ステア角 vs 走行距離",
        ylabel="ステア角 [deg]",
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / "steering_vs_distance")


def plot_curves(data: dict, map_ways: list | None):
    """カーブ別の軌跡比較（横 N 列サブプロット）。"""
    if not CURVE_CENTERS:
        return
    runs = [
        {
            "label": label, "color": d["color"], "lw": d["lw"], "ls": d["ls"],
            "marker": d["marker"], "ms": d["ms"],
            "x": np.asarray(d["kinematic"]["x"]), "y": np.asarray(d["kinematic"]["y"]),
        }
        for label, d in data.items() if not d["kinematic"].empty
    ]
    fig = build_fig_curves_closeup(CURVE_CENTERS, runs, map_ways, scenario_name=SCENARIO_NAME)
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / "curves_closeup")


# ---------------------------------------------------------------------------
# カーブ②発進分析
# ---------------------------------------------------------------------------


def _find_curve_launch(
    df_vel: pd.DataFrame, launch_window: tuple[float, float]
) -> float | None:
    """指定 window 内で「停止 → 発進」を検出する `_events.find_curve2_launch` ラッパー。"""
    return _find_curve2_launch_pure(df_vel, window=launch_window)


def _find_curve_exit(
    df_kinematic: pd.DataFrame,
    curve_idx: int,
    t_launch: float,
    radius: float = 30.0,
) -> float | None:
    """`curve_centers[curve_idx]` を中心とした退出時刻検出 (`_events.find_curve2_exit` ラッパー)."""
    if not CURVE_CENTERS or not (0 <= curve_idx < len(CURVE_CENTERS)):
        return None
    c = CURVE_CENTERS[curve_idx]
    return _find_curve2_exit_pure(
        df_kinematic, (float(c["cx"]), float(c["cy"])), t_launch, radius=radius
    )


_CURVE_PRE = -2.0  # 表示窓の発進前余白 [s]
_CURVE_MG = 80  # カーブ軌跡表示の半幅 [m]


def _curve_launch_map(data: dict, launch_window: tuple[float, float]) -> dict[str, float]:
    """各ログのカーブ発進 t を検出した {label: t_launch}（検出不能はスキップ）。"""
    out: dict[str, float] = {}
    for label, d in data.items():
        t = _find_curve_launch(d["velocity"], launch_window)
        if t is not None:
            out[label] = t
    return out


def _curve_window(d: dict, curve_idx: int, t_l: float) -> tuple[float, float]:
    """発進前2s〜カーブ退出後2s の絶対時刻窓 (t_start, t_end) を返す。"""
    t_exit = _find_curve_exit(d["kinematic"], curve_idx, t_l)
    t_post = (t_exit - t_l + 2.0) if t_exit is not None else 25.0
    return t_l + _CURVE_PRE, t_l + t_post


def _clip_rel(df: pd.DataFrame, t0: float, t1: float, t_l: float) -> pd.DataFrame:
    """[t0, t1] で切り出し、発進相対時刻 tr=t-t_l を付けたコピーを返す。"""
    sub = df[(df["t"] >= t0) & (df["t"] <= t1)].copy()
    sub["tr"] = sub["t"] - t_l
    return sub


def _style(d: dict, label: str) -> dict:
    return {"label": label, "color": d["color"], "lw": d["lw"], "ls": d["ls"],
            "marker": d["marker"], "ms": d["ms"]}


def _curve_traj_runs(data: dict, launch_t: dict, curve_idx: int) -> list[dict]:
    """カーブ周辺軌跡 run（bbox 窓の seg + ★発進点）を整形する。"""
    runs: list[dict] = []
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t0, t1 = _curve_window(d, curve_idx, t_l)
        df_k = d["kinematic"]
        seg = df_k[(df_k["t"] >= t0) & (df_k["t"] <= t1)]
        if seg.empty:
            continue
        lr = df_k.iloc[(df_k["t"] - t_l).abs().argsort().iloc[0]]
        runs.append({**_style(d, label), "seg_x": np.asarray(seg["x"]), "seg_y": np.asarray(seg["y"]),
                     "launch_x": float(lr["x"]), "launch_y": float(lr["y"])})
    return runs


def _curve_obj(curve_idx: int) -> dict:
    c = CURVE_CENTERS[curve_idx]
    return {"cx": c["cx"], "cy": c["cy"], "mg": _CURVE_MG,
            "label": c.get("label", f"カーブ{curve_idx + 1}")}


def plot_curve_analysis(
    data: dict,
    map_ways: list | None,
    curve_idx: int,
    launch_window: tuple[float, float],
):
    """指定カーブの一時停止発進からの挙動を軌跡＋時系列で比較（上段全幅=軌跡, 下段3列）。"""
    if not CURVE_CENTERS or not (0 <= curve_idx < len(CURVE_CENTERS)):
        warnings.warn(f"plot_curve_analysis: curve_idx={curve_idx} が範囲外")
        return
    launch_t = _curve_launch_map(data, launch_window)
    if not launch_t:
        warnings.warn(f"発進時刻を検出できないため plot_curve_analysis(curve{curve_idx + 1}) をスキップ")
        return

    ts_runs: list[dict] = []
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t0, t1 = _curve_window(d, curve_idx, t_l)
        vel = _clip_rel(d["velocity"], t0, t1, t_l)
        acc = _clip_rel(d["accel"], t0, t1, t_l)
        steer = _clip_rel(d["steering"], t0, t1, t_l)
        cmd = _clip_rel(d["cmd"], t0, t1, t_l)
        r = {**_style(d, label),
             "t_vel": np.asarray(vel["tr"]), "vel": np.asarray(vel["lon_vel"]),
             "t_acc": np.asarray(acc["tr"]), "acc": np.asarray(acc["accel"]),
             "t_steer": np.asarray(steer["tr"]), "steer_deg": np.degrees(steer["steer"]),
             "t_cmd": None}
        if not cmd.empty:
            r.update({"t_cmd": np.asarray(cmd["tr"]), "cmd_vel": np.asarray(cmd["cmd_vel"]),
                      "cmd_acc": np.asarray(cmd["cmd_accel"]),
                      "cmd_steer_deg": np.degrees(cmd["cmd_steer"])})
        ts_runs.append(r)

    fig = build_fig_curve_analysis(
        _curve_obj(curve_idx), map_ways, _curve_traj_runs(data, launch_t, curve_idx), ts_runs,
        scenario_name=SCENARIO_NAME,
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / f"curve{curve_idx + 1}_analysis")


def plot_curve_steering_detail(
    data: dict,
    map_ways: list | None,
    curve_idx: int,
    launch_window: tuple[float, float],
):
    """
    指定カーブの一時停止発進からのステアリング詳細分析。

    レイアウト（2列構成）:
        左列 上段: 軌跡（対象カーブ付近）
        左列 下段: 指令 vs 応答 の重ね描き（同一軸）
        右列 上段: ステアリング角速度 [deg/s]
        右列 中段: 指令追従誤差（応答 − 指令）[deg]
        右列 下段: ステアリング角 累積絶対値（操舵量の積分）
    """
    if not CURVE_CENTERS or not (0 <= curve_idx < len(CURVE_CENTERS)):
        warnings.warn(f"plot_curve_steering_detail: curve_idx={curve_idx} が範囲外")
        return
    launch_t = _curve_launch_map(data, launch_window)
    if not launch_t:
        warnings.warn(f"発進時刻を検出できないため plot_curve_steering_detail(curve{curve_idx + 1}) をスキップ")
        return

    steer_runs: list[dict] = []
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t0, t1 = _curve_window(d, curve_idx, t_l)
        s = _clip_rel(d["steering"], t0, t1, t_l)
        c = _clip_rel(d["cmd"], t0, t1, t_l)
        t_s = np.asarray(s["tr"])
        s_deg = np.degrees(s["steer"].values)
        r = {**_style(d, label), "t_s": t_s, "steer_deg": s_deg, "t_cmd": None,
             "rate_t": None, "err_t": None, "integ": np.zeros_like(t_s)}
        if not c.empty:
            r["t_cmd"] = np.asarray(c["tr"])
            r["cmd_steer_deg"] = np.degrees(c["cmd_steer"].values)
        if len(t_s) > 1:
            dt = np.diff(t_s)
            rate = np.diff(s_deg) / np.where(dt > 1e-6, dt, np.nan)
            r["rate_t"], r["rate"] = t_s[1:], np.where(np.abs(rate) < 200, rate, np.nan)
            r["integ"] = np.concatenate([[0], np.cumsum(np.abs(np.diff(s_deg)) * dt)])
        if not c.empty and len(t_s) > 0:
            cmd_interp = np.interp(t_s, c["tr"].values, np.degrees(c["cmd_steer"].values))
            r["err_t"], r["err"] = t_s, s_deg - cmd_interp
        steer_runs.append(r)

    fig = build_fig_curve_steering_detail(
        _curve_obj(curve_idx), map_ways, _curve_traj_runs(data, launch_t, curve_idx), steer_runs,
        scenario_name=SCENARIO_NAME,
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / f"curve{curve_idx + 1}_steering_detail")


def plot_curve_yaw_steer(
    data: dict,
    curve_idx: int,
    launch_window: tuple[float, float],
):
    """
    ステア角と実際に進んだ方向（ヨーレート換算）の差分を可視化。

    自転車モデル: yaw_rate_pred = v * tan(steer) / L
    等価ステア角:  steer_equiv  = atan(L * yaw_rate_actual / v)

    レイアウト（縦4段、共通t軸）:
        段1: ステア角（実測）vs 等価ステア角（実ヨーレートから逆算） [deg]
        段2: ステア角 − 等価ステア角 = 「進行方向とのズレ」[deg]
        段3: 実ヨーレート vs 自転車モデル予測ヨーレート [deg/s]
        段4: yaw角の累積変化量（カーブの深さ比較）[deg]
    """
    if not CURVE_CENTERS or not (0 <= curve_idx < len(CURVE_CENTERS)):
        warnings.warn(f"plot_curve_yaw_steer: curve_idx={curve_idx} が範囲外")
        return
    launch_t = _curve_launch_map(data, launch_window)
    if not launch_t:
        return

    yaw_runs: list[dict] = []
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t0, t1 = _curve_window(d, curve_idx, t_l)
        kin = _clip_rel(d["kinematic"], t0, t1, t_l)
        vel = _clip_rel(d["velocity"], t0, t1, t_l)
        steer = _clip_rel(d["steering"], t0, t1, t_l)
        if kin.empty or vel.empty or steer.empty:
            continue
        t_k = kin["tr"].values
        yaw_u = np.unwrap(kin["yaw"].values)
        yaw_rate = np.gradient(yaw_u, t_k)  # rad/s
        v_i = np.interp(t_k, vel["tr"].values, vel["lon_vel"].values)
        s_i = np.interp(t_k, steer["tr"].values, steer["steer"].values)  # rad
        v_safe = np.where(v_i > 0.3, v_i, np.nan)  # 低速は等価ステア角が不安定
        steer_equiv = np.arctan2(WHEELBASE * yaw_rate, v_safe)  # rad
        yaw_rate_pred = v_i * np.tan(s_i) / WHEELBASE  # rad/s
        diff_deg = np.clip(np.degrees(s_i - steer_equiv), -30, 30)
        t0_idx = int(np.argmin(np.abs(t_k)))
        yaw_runs.append({**_style(d, label), "t": t_k,
                         "steer_deg": np.degrees(s_i), "equiv_deg": np.degrees(steer_equiv),
                         "diff_deg": diff_deg, "yaw_rate_deg": np.degrees(yaw_rate),
                         "yaw_rate_pred_deg": np.degrees(yaw_rate_pred),
                         "yaw_cum_deg": np.degrees(yaw_u - yaw_u[t0_idx])})

    fig = build_fig_curve_yaw_steer(
        yaw_runs, scenario_name=SCENARIO_NAME,
        curve_label=CURVE_CENTERS[curve_idx].get("label", f"カーブ{curve_idx + 1}"),
        wheelbase=float(WHEELBASE),
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / f"curve{curve_idx + 1}_yaw_steer")


def plot_steer_response(
    data: dict,
    curve_idx: int,
    launch_window: tuple[float, float],
):
    """
    ステアリング応答性能の比較図。

    ステア指令が 1deg を超えた瞬間を各ログの t=0 に揃え、
    指令 vs 応答の「追従の遅れ・形の違い」を正規化して一目でわかるようにする。

    レイアウト:
        上段(大): 正規化ステア角（指令 vs 応答）— ピーク指令値で正規化して
                  指令波形の形が揃った状態で応答のズレを比較
        中段左: 追従誤差（応答 − 指令）の時系列
        中段右: 立ち上がり時間（指令ピーク90%到達）の棒グラフ
        下段左: RMSE [deg]の棒グラフ
        下段右: ピーク追従率（応答ピーク / 指令ピーク）の棒グラフ
    """
    T_ONSET_PRE = -0.5  # onset 前の余白 [s]
    T_ONSET_POST = 12.0  # onset 後の表示幅 [s]
    ONSET_THRESH_DEG = 1.0

    runs: list[dict] = []
    bar_labels: list[str] = []
    bar_colors: list[str] = []
    rise_ms: list[float] = []
    rmse_vals: list[float] = []
    peak_vals: list[float] = []

    for label, d in data.items():
        t_l = _find_curve_launch(d["velocity"], launch_window)
        if t_l is None:
            continue
        steer_r = _clip_rel(d["steering"], t_l - 3, t_l + 20, t_l)
        cmd_r = _clip_rel(d["cmd"], t_l - 3, t_l + 20, t_l)
        if steer_r.empty or cmd_r.empty:
            continue
        cmd_deg = np.degrees(cmd_r["cmd_steer"].values)
        steer_deg = np.degrees(steer_r["steer"].values)

        onset_mask = (cmd_r["tr"] >= 0) & (np.abs(cmd_deg) > ONSET_THRESH_DEG)
        if not onset_mask.any():
            continue
        t_onset_cmd = float(cmd_r["tr"].values[onset_mask][0])
        onset_mask_act = (steer_r["tr"] >= 0) & (np.abs(steer_deg) > ONSET_THRESH_DEG)
        t_onset_act = (
            float(steer_r["tr"].values[onset_mask_act][0]) if onset_mask_act.any() else t_onset_cmd
        )
        onset_delay = t_onset_act - t_onset_cmd

        # onset t=0 で揃えた窓
        t_c = cmd_r["tr"].values - t_onset_cmd
        t_a = steer_r["tr"].values - t_onset_cmd
        wc = (t_c >= T_ONSET_PRE) & (t_c <= T_ONSET_POST)
        wa = (t_a >= T_ONSET_PRE) & (t_a <= T_ONSET_POST)
        t_c_w, c_w = t_c[wc], cmd_deg[wc]
        t_a_w, a_w = t_a[wa], steer_deg[wa]
        if len(c_w) == 0 or len(a_w) == 0:
            continue

        peak_cmd = np.abs(c_w).max()
        peak_act = np.abs(a_w).max()
        c_norm = c_w / peak_cmd if peak_cmd > 0 else c_w
        a_norm = a_w / peak_cmd if peak_cmd > 0 else a_w

        dt = 0.02
        t_grid = np.arange(T_ONSET_PRE, T_ONSET_POST, dt)
        c_i = np.interp(t_grid, t_c_w, c_w, left=np.nan, right=np.nan)
        a_i = np.interp(t_grid, t_a_w, a_w, left=np.nan, right=np.nan)
        valid = ~(np.isnan(c_i) | np.isnan(a_i))
        rmse = float(np.sqrt(np.mean((a_i[valid] - c_i[valid]) ** 2))) if valid.any() else np.nan

        runs.append({**_style(d, label), "t_c": t_c_w, "c_norm": c_norm,
                     "t_a": t_a_w, "a_norm": a_norm, "onset_delay": onset_delay,
                     "err_t": t_grid, "err": a_i - c_i, "is_real": label == "実機"})
        bar_labels.append(label)
        bar_colors.append(d["color"])
        rise_ms.append(onset_delay * 1000)
        rmse_vals.append(rmse if not np.isnan(rmse) else 0.0)
        peak_vals.append(peak_act / peak_cmd if peak_cmd > 0 else 0.0)

    if not runs:
        warnings.warn(f"発進検出ログがないため plot_steer_response(curve{curve_idx + 1}) をスキップ")
        return
    bars = {"labels": bar_labels, "colors": bar_colors,
            "rise": rise_ms, "rmse": rmse_vals, "peak": peak_vals}
    fig = build_fig_steer_response(runs, bars, scenario_name=SCENARIO_NAME)
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, FIGS_DIR / f"curve{curve_idx + 1}_steer_response")


# ---------------------------------------------------------------------------
# 数値レポート
# ---------------------------------------------------------------------------


def nearest_point_distance(ref_xy: np.ndarray, query_xy: np.ndarray) -> np.ndarray:
    """各query点に対するref_xyの最近傍距離を返す。"""
    from scipy.spatial import cKDTree

    tree = cKDTree(ref_xy)
    dists, _ = tree.query(query_xy)
    return dists


def _log_summary(d: dict) -> dict:
    """1 ログの走行サマリ (kinematic odometry 基準 + velocity_status 参考)。"""
    kin, vel = d["kinematic"], d["velocity"]
    if not kin.empty:
        dist = float(cumulative_arc_length(kin["x"].to_numpy(), kin["y"].to_numpy())[-1])
    else:
        dist = float("nan")
    if not vel.empty:
        t = vel["t"].to_numpy()
        elapsed = float(t.max() - t.min())
        v = vel["lon_vel"].to_numpy()
        stopped = float(np.mean(v <= 0.3) * 100.0)
        cruise = float(v[v > 0.3].mean()) if np.any(v > 0.3) else 0.0
        vmean, vmax, vstd = float(v.mean()), float(v.max()), float(v.std())
    else:
        elapsed = stopped = cruise = vmean = vmax = vstd = float("nan")
    mean_speed = dist / elapsed if (elapsed and elapsed > 0) else float("nan")
    return {
        "dist": dist, "elapsed": elapsed, "stopped": stopped, "cruise": cruise,
        "mean_speed": mean_speed, "vmean": vmean, "vmax": vmax, "vstd": vstd,
    }


def _prov_text(label: str, d: dict) -> str:
    """1 ログの provenance 表示文字列 (real は外部記録 / sim は記録済 onnx)。"""
    prov = d.get("prov") or {}
    if label == "実機":
        return prov.get("real_note") or "取得時バージョン不明 (要記録: scenario.yaml Conditions.real_provenance)"
    return format_provenance_line(prov)


def build_report(data: dict) -> str:
    lines = [f"# 比較レポート\n\nシナリオ: {SCENARIO_NAME}\n"]

    # モデル重み / バージョン provenance (版・重み差が乖離の原因になり得るため明示)。
    lines.append("## モデル重み / バージョン provenance\n")
    lines.append(
        "> 実機データ取得時と sim 実行時で pilot-auto.x2 / DiffusionPlanner の重みが異なり得る。"
        "観測された乖離は車両モデル忠実度ではなく版・重み差由来の可能性があるため、各ログの provenance を記載する。"
    )
    lines.append("")
    lines.append("| ログ | DP モデル重み / autoware バージョン |")
    lines.append("|---|---|")
    for label, d in data.items():
        lines.append(f"| {label} | {_prov_text(label, d)} |")
    lines.append("")

    real = data.get("実機")
    real_kin = real["kinematic"] if real is not None else pd.DataFrame()
    real_dist = (
        float(cumulative_arc_length(real_kin["x"].to_numpy(), real_kin["y"].to_numpy())[-1])
        if not real_kin.empty
        else float("nan")
    )
    stats = {label: _log_summary(d) for label, d in data.items()}

    # 診断 (A4): AUTONOMOUS 窓・t0 基準・localization 除外数を明示。
    lines.append("## 診断（AUTONOMOUS 区間・localization）\n")
    lines.append("| ログ | AUTONOMOUS窓 [s] | t0基準 | localization除外/総数 |")
    lines.append("|---|---|---|---|")
    for label, d in data.items():
        vel = d["velocity"]
        if not vel.empty:
            t = vel["t"].to_numpy()
            win = f"{t.min():.1f}〜{t.max():.1f}"
        else:
            win = "—"
        lines.append(
            f"| {label} | {win} | {d.get('t0_method', '?')} | "
            f"{d.get('kin_dropped', 0)}/{d.get('kin_total', 0)} |"
        )
    lines.append("")

    # 走行サマリ (A1): kinematic odometry 基準の信頼できる比較指標。
    lines.append("## 走行サマリ（kinematic odometry 基準）\n")
    lines.append(
        "| ログ | 経過時間[s] | 走行距離[m] | 平均速度(距離/経過)[m/s] | "
        "巡航平均(v>0.3)[m/s] | 停止割合(v≤0.3)[%] |"
    )
    lines.append("|---|---|---|---|---|---|")
    for label in data:
        s = stats[label]
        lines.append(
            f"| {label} | {s['elapsed']:.1f} | {s['dist']:.1f} | {s['mean_speed']:.3f} | "
            f"{s['cruise']:.3f} | {s['stopped']:.1f} |"
        )
    lines.append("")
    lines.append("> 走行距離・平均速度は /localization/kinematic_state (odometry) 由来 (無効フレーム除外後)。")
    lines.append("")

    # 速度統計 (参考: velocity_status — log 間で非可比なため参考扱い)。
    lines.append("## 速度統計（参考: velocity_status）\n")
    lines.append(
        "> velocity_status はトピックの sampling 周波数・記録区間が log 間で異なり、"
        "平均/最大は log 間で非可比。比較には上の「走行サマリ」を用いること。"
    )
    lines.append("")
    lines.append("| ログ | 平均 [m/s] | 最大 [m/s] | 標準偏差 |")
    lines.append("|---|---|---|---|")
    for label in data:
        s = stats[label]
        lines.append(f"| {label} | {s['vmean']:.3f} | {s['vmax']:.3f} | {s['vstd']:.3f} |")
    lines.append("")

    # 速度指令 RMSE（指令 vs 応答）
    lines.append("## 速度 RMSE（指令 vs 応答）\n")
    lines.append("| ログ | RMSE [m/s] |")
    lines.append("|---|---|")
    for label, d in data.items():
        cmd = d["cmd"]
        vel = d["velocity"]
        if cmd.empty or vel.empty:
            lines.append(f"| {label} | N/A |")
            continue
        cmd_i = np.interp(vel["t"], cmd["t"], cmd["cmd_vel"])
        rmse = np.sqrt(np.mean((vel["lon_vel"].values - cmd_i) ** 2))
        lines.append(f"| {label} | {rmse:.4f} |")
    lines.append("")

    # ステア RMSE（指令 vs 応答） + perfect_tracker 注記 (B2)
    lines.append("## ステアリング RMSE（指令 vs 応答）\n")
    lines.append("| ログ | RMSE [deg] | 備考 |")
    lines.append("|---|---|---|")
    has_perfect = False
    for label, d in data.items():
        cmd = d["cmd"]
        steer = d["steering"]
        if cmd.empty or steer.empty:
            lines.append(f"| {label} | N/A | |")
            continue
        cmd_i = np.interp(steer["t"], cmd["t"], cmd["cmd_steer"])
        rmse = np.degrees(np.sqrt(np.mean((steer["steer"].values - cmd_i) ** 2)))
        if d.get("perfect"):
            has_perfect = True
            lines.append(f"| {label} | {rmse:.4f} | ※参考(無効) |")
        else:
            lines.append(f"| {label} | {rmse:.4f} | |")
    lines.append("")
    if has_perfect:
        lines.append(
            "> ※ perfect_tracker (PERFECT_TRAJECTORY_TRACKER) は control_cmd を追従せず "
            "planner 軌跡を直接追従するため、cmd-vs-response の操舵 RMSE は指令追従誤差として"
            "意味を持たない (参考値)。挙動評価は下記「軌跡乖離・完走率」を用いること。"
        )
        lines.append("")

    # 軌跡乖離・完走率 (A3): 走行距離/完走率を主指標化し、双方向最近傍で未完走を露出。
    lines.append("## 軌跡乖離・完走率\n")
    lines.append("> A0 有効性ゲート後の kinematic で算出。完走率 = 走行距離 / 実機走行距離。")
    lines.append(
        "> sim→実機 = sim 各点の実機軌跡への最近傍 (経路追従精度)。"
        "実機→sim = 実機各点の sim 軌跡への最近傍 (sim 未走行区間で増大し未完走を露出)。"
    )
    lines.append("")
    lines.append(
        "| ログ | 走行距離[m] | 完走率[%] | sim→実機 平均/最大[m] | "
        "実機→sim 平均/最大[m] | 状態 |"
    )
    lines.append("|---|---|---|---|---|---|")
    if real is None or real_kin.empty:
        lines.append("| (実機 kinematic なし) | — | — | — | — | データ不足 |")
    else:
        ref_xy = real_kin[["x", "y"]].to_numpy()
        for label, d in data.items():
            if label == "実機":
                lines.append(f"| {label} | {real_dist:.1f} | 100.0 (基準) | — | — | 基準 |")
                continue
            dist = stats[label]["dist"]
            comp = (dist / real_dist * 100.0) if (real_dist and real_dist > 0) else float("nan")
            q_xy = (
                d["kinematic"][["x", "y"]].to_numpy()
                if not d["kinematic"].empty
                else np.empty((0, 2))
            )
            if len(q_xy) < 10:
                lines.append(f"| {label} | {dist:.1f} | {comp:.1f} | — | — | データ不足 |")
                continue
            try:
                s2r = nearest_point_distance(ref_xy, q_xy)
                r2s = nearest_point_distance(q_xy, ref_xy)
                status = "OK" if comp >= 85.0 else "早期停止/未完走"
                lines.append(
                    f"| {label} | {dist:.1f} | {comp:.1f} | "
                    f"{s2r.mean():.3f}/{s2r.max():.3f} | {r2s.mean():.3f}/{r2s.max():.3f} | {status} |"
                )
            except Exception as e:  # noqa: BLE001
                lines.append(f"| {label} | {dist:.1f} | {comp:.1f} | ERR: {e} | — | — |")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------


def _apply_runtime_config(cfg: RuntimeConfig) -> None:
    """`RuntimeConfig` をモジュールレベルの module-level state に反映する。

    advisor 助言に従い `main()` 内の `global` 宣言を本関数に集約し、
    `main()` は cfg を渡すだけにしている (本体の `plot_*` は module-level 参照で動くため)。
    """
    global BASE, LITE_DIR, OUT_DIR, FIGS_DIR  # noqa: PLW0603
    global SCENARIO_NAME, WHEELBASE, CURVE_CENTERS, LOGS  # noqa: PLW0603
    global _CURVE2_WINDOW, _CURVE2_INDEX  # noqa: PLW0603

    BASE = cfg.base_dir
    LITE_DIR = cfg.lite_dir
    OUT_DIR = cfg.out_dir
    FIGS_DIR = cfg.figs_dir

    SCENARIO_NAME = cfg.scenario_name
    WHEELBASE = float(cfg.wheelbase_validation)
    CURVE_CENTERS = cfg.curve_centers
    _CURVE2_INDEX = cfg.curve2_index
    _CURVE2_WINDOW = cfg.curve2_window

    # sim_runs.yaml 連動: cfg.sim_runs_config が指定されていれば全 run を LOGS dict に追加
    sim_runs_cfg = None
    if cfg.sim_runs_config:
        try:
            from .lib._sim_runs_config import load_sim_runs_config  # noqa: PLC0415
            sim_runs_cfg = load_sim_runs_config(cfg.sim_runs_config)
        except Exception as exc:  # noqa: BLE001
            warnings.warn(f"sim_runs.yaml 読み込み失敗: {exc} (sim 重ね描きスキップ)")

    LOGS = _rebuild_logs(LITE_DIR, cfg.topic_overrides or None, sim_runs_cfg)


def _load_dp_trajectories(bag: Path, t0_ns: int) -> list[dict]:
    """DiffusionPlanner 出力軌跡の全フレームを読み込む (playback ビューア用)。

    各フレーム = {"t": t0 基準の発行時刻 [s], "x": [...], "y": [...]} (world 座標)。
    間引き・丸めは _playback_viewer 側 (payload 構築時) で行う。
    トピックが無い bag (旧ログ等) は空リストを返す。
    """
    topic = resolve_topic(bag, [DP_TRAJ_TOPIC, "/sub" + DP_TRAJ_TOPIC])
    if topic is None:
        return []
    frames: list[dict] = []
    for t_ns, ros in iter_bag_messages(bag, [topic]):
        pts = ros.points
        if not pts:
            continue
        frames.append({
            "t": (t_ns - t0_ns) / 1e9,
            "x": [p.pose.position.x for p in pts],
            "y": [p.pose.position.y for p in pts],
        })
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description="実機 vs シム 比較プロット生成")
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    _apply_runtime_config(cfg)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIGS_DIR.mkdir(parents=True, exist_ok=True)

    print("=== データ読み込み中 ===")
    loaded: dict = {}
    for label, lcfg in LOGS.items():
        mcap_path: Path = lcfg["path"]
        if not mcap_path.exists():
            warnings.warn(f"[{label}] {mcap_path} が見つからないためスキップ")
            continue
        print(f"  [{label}] {mcap_path.name}")

        df_mode = load_operation_mode(mcap_path)
        df_vel = load_velocity(mcap_path)
        t0 = _find_autonomous_start(df_mode, df_vel)
        # t0 の決定根拠を明示 (real は mode 遷移 / sim は operation_mode 僅少で速度 fallback に
        # なりがちで、両者の基準差が時間軸重ね描きの「見かけの時間ずれ」を生む)。
        has_auto = (not df_mode.empty) and bool((df_mode["mode"] == _AUTONOMOUS_MODE).any())
        t0_method = "operation_mode(AUTONOMOUS)" if has_auto else "velocity_threshold(fallback)"
        print(f"    → t0 = {t0} ns [{t0_method}]")

        # localization 有効性ゲート: 原点(0,0)/瞬間移動の無効フレームを除外 (A0)。
        df_kin_raw = load_kinematic(mcap_path, lcfg["kinematic"])
        df_kin_clean, n_drop = filter_localization(df_kin_raw)
        if n_drop:
            print(f"    → localization 無効フレーム除外: {n_drop}/{len(df_kin_raw)}")

        loaded[label] = {
            "velocity": align_time(df_vel, t0),
            "steering": align_time(load_steering(mcap_path), t0),
            "kinematic": align_time(df_kin_clean, t0),
            # DP 計画軌跡フレーム (playback ビューアで現在時刻の計画経路を薄く表示)
            "dp_traj": _load_dp_trajectories(mcap_path, t0),
            "accel": align_time(load_accel(mcap_path, lcfg["accel"]), t0),
            "cmd": align_time(load_cmd(mcap_path, lcfg["cmd"]), t0),
            "color": lcfg["color"],
            "lw": lcfg["lw"],
            "ls": lcfg["ls"],
            "marker": lcfg["marker"],
            "ms": lcfg["ms"],
            "perfect": lcfg.get("perfect", False),
            "vehicle_model": lcfg.get("vehicle_model", ""),
            "t0_method": t0_method,
            "kin_dropped": int(n_drop),
            "kin_total": int(len(df_kin_raw)),
            # この run が使った DP 重み / autoware バージョン (step3 が sim lite に記録)。
            # 実機は外部 (車両デプロイ) のため REAL_PROVENANCE env (scenario.yaml) を使う。
            "prov": (
                {"real_note": os.environ.get("REAL_PROVENANCE", "").strip() or None}
                if label == "実機"
                else read_provenance(LITE_DIR / lcfg["bag_dir"])
            ),
        }

    if not loaded:
        warnings.warn("有効なログが1つも読み込めませんでした")
        return

    # 各ログの初回発進時刻 (t_launch) を **全ログ共通の検出器** (find_initial_launch) で算出する。
    # 比較時系列プロット (velocity/acceleration/steering) と再生ビューアはこれを基準に発進相対へ
    # 整列する (_tr / build_playback_payload)。実機の初期停止が長くても発進で揃い「実機だけ遅れる」
    # を防ぐ。real / sim を同一検出器で処理するのが要点 (整列の非対称性を作らない)。
    # 即発進するログ (例 takanawa) は t_launch≈0 となり従来の t0 基準と実質同じ (無影響)。
    for d in loaded.values():
        tl = _find_initial_launch(d["velocity"]) if not d["velocity"].empty else None
        d["t_launch"] = float(tl) if tl is not None else 0.0

    # Lanelet2 地図読み込み (RuntimeConfig が解決済みのパスを保持)
    print("\n=== 地図読み込み中 ===")
    map_ways: list | None = None
    if cfg.map_osm_path is not None:
        try:
            map_ways = load_map_ways(cfg.map_osm_path)
            print(f"  {len(map_ways)} ways をロード ({cfg.map_osm_path})")
        except Exception as e:  # noqa: BLE001
            warnings.warn(f"地図ロード失敗: {e}")
    else:
        warnings.warn("地図ファイルが見つかりません。軌跡プロットは地図背景なしで描画します")

    print("\n=== プロット生成中 ===")
    # 軌跡再生ビューア (時刻同期/位置同期シークバー付き自己完結 HTML)
    plot_trajectory_playback(loaded, map_ways, FIGS_DIR, title=SCENARIO_NAME)
    plot_velocity(loaded)
    plot_acceleration(loaded)
    plot_steering(loaded)
    # 走行距離 (arc-length) 基準の重ね描き (B1): pacing 差を除き早期停止を露出。
    plot_velocity_vs_distance(loaded)
    plot_steering_vs_distance(loaded)

    if CURVE_CENTERS:
        plot_curves(loaded, map_ways)
        # plot_curves (curve_centers の全カーブ一覧) は1枚で全カーブを概観する図。
        # 個別カーブの詳細プロット(analysis/steering_detail/yaw_steer/steer_response)は
        # cfg.plot_curves で対象を切り替える。
        for spec in cfg.plot_curves:
            idx = spec["index"]
            win = spec["launch_window"]
            plot_curve_analysis(loaded, map_ways, idx, win)
            plot_curve_steering_detail(loaded, map_ways, idx, win)
            plot_curve_yaw_steer(loaded, idx, win)
            plot_steer_response(loaded, idx, win)

    print("\n=== レポート生成中 ===")
    try:
        report = build_report(loaded)
        report_path = OUT_DIR / "report.md"
        report_path.write_text(report, encoding="utf-8")
        print(f"  保存: {report_path}")
    except Exception as e:  # noqa: BLE001
        warnings.warn(f"レポート生成失敗: {e}")

    print("\n完了。出力先:", FIGS_DIR)


if __name__ == "__main__":
    main()
