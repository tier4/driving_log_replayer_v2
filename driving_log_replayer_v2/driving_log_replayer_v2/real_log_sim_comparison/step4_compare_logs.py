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

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: F401 — 旧コード参照
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go

from .lib._events import (
    AUTONOMOUS_MODE as _AUTONOMOUS_MODE,
    find_autonomous_start as _find_autonomous_start,
    find_curve2_exit as _find_curve2_exit_pure,
    find_curve2_launch as _find_curve2_launch_pure,
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
from .lib._map import load_map_ways, map_ways_in_bbox, resolve_map_osm
from .lib._params_utils import add_params_annotation, setup_jp_font
from .lib._playback_viewer import plot_trajectory_playback
from .lib._plotly_utils import (
    FIG_HEIGHTS,
    add_params_annotation_plotly,
    lanes_to_trace,
    write_plotly_html,
)
from .lib._provenance import format_provenance_line, read_provenance
from .lib._runtime_config import (
    RuntimeConfig,
    add_common_cli_arguments,
    build_runtime_config,
)

setup_jp_font()

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
    "注: t=0 は各ログの AUTONOMOUS/発進基準。pacing 差により同一 t は同一地点を意味しない"
    "（走行距離基準は *_vs_distance.svg を参照）"
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


def _setup_fig(title: str, nrows=1, ncols=1, figsize=(12, 5)):
    fig, axes = plt.subplots(nrows, ncols, figsize=figsize, squeeze=False)
    fig.suptitle(f"{SCENARIO_NAME}\n{title}", fontsize=11)
    return fig, axes


def _save(fig, name: str):
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    add_params_annotation(fig)
    path = FIGS_DIR / f"{name}.svg"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  保存: {name}.svg")


def _traj_plot(ax, df_xy, d, label, markevery=20, zorder=3):
    """軌跡を線スタイル＋等間隔マーカーで描画するヘルパー。"""
    n = len(df_xy)
    me = max(1, n // markevery) if n > 0 else 1
    ax.plot(
        np.asarray(df_xy["x"]),
        np.asarray(df_xy["y"]),
        color=d["color"],
        lw=d["lw"],
        ls=d["ls"],
        marker=d["marker"],
        markersize=d["ms"],
        markevery=me,
        markerfacecolor=d["color"],
        markeredgecolor="white",
        markeredgewidth=0.5,
        label=label,
        zorder=zorder,
    )


# matplotlib スタイル → plotly スタイルの対応表（軌跡プロットの plotly 化用）。
# _SIM_LINESTYLES の on-off タプルは近い dash enum へ割り当てる。
_PLOTLY_DASH = {"-": "solid", "--": "dash", "-.": "dashdot", ":": "dot"}
_PLOTLY_DASH_TUPLES = {(4, 2): "longdash", (3, 1, 1, 1): "longdashdot"}
_PLOTLY_MARKER = {
    "o": "circle", "^": "triangle-up", "s": "square", "D": "diamond",
    "v": "triangle-down", "P": "cross", "*": "star", "X": "x",
}


def _plotly_dash(ls) -> str:
    """matplotlib linestyle（文字列 or (offset, on-off タプル)）を plotly dash enum へ。"""
    if isinstance(ls, tuple):
        return _PLOTLY_DASH_TUPLES.get(tuple(ls[1]), "dash")
    return _PLOTLY_DASH.get(ls, "solid")


def plot_trajectory(data: dict, map_ways: list | None):
    """地図背景あり軌跡プロット（plotly インタラクティブ HTML）。

    表示範囲は3軌跡の bbox + マージン に限定。ズーム・パン・ホバー・凡例トグル可能。
    """
    # 3軌跡の bounding box を算出
    all_xy_list = [
        d["kinematic"][["x", "y"]].values for d in data.values() if not d["kinematic"].empty
    ]
    if not all_xy_list:
        warnings.warn("kinematic データなし。軌跡プロットをスキップ")
        return
    all_xy = np.concatenate(all_xy_list)
    margin = 30  # m
    x_min, y_min = all_xy.min(axis=0) - margin
    x_max, y_max = all_xy.max(axis=0) + margin

    fig = go.Figure()
    if map_ways:
        ways = map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max))
        fig.add_trace(lanes_to_trace(ways))

    for label, d in data.items():
        df = d["kinematic"]
        if df.empty:
            continue
        x = np.asarray(df["x"])
        y = np.asarray(df["y"])
        hover_extra = ""
        customdata = None
        if "t" in df.columns:
            customdata = np.asarray(df["t"])
            hover_extra = "<br>t=%{customdata:.1f}s"
        # 線 trace（全点 hover 可能）
        fig.add_trace(go.Scatter(
            x=x, y=y,
            mode="lines",
            name=label,
            legendgroup=label,
            line=dict(color=d["color"], width=d["lw"], dash=_plotly_dash(d["ls"])),
            customdata=customdata,
            hovertemplate=f"{label}<br>x=%{{x:.1f}}m y=%{{y:.1f}}m{hover_extra}<extra></extra>",
        ))
        # 等間隔マーカー trace（matplotlib markevery 相当）
        me = max(1, len(x) // 15)
        fig.add_trace(go.Scatter(
            x=x[::me], y=y[::me],
            mode="markers",
            legendgroup=label,
            showlegend=False,
            marker=dict(
                symbol=_PLOTLY_MARKER.get(d["marker"], "circle"),
                size=d["ms"] + 3,
                color=d["color"],
                line=dict(color="white", width=0.5),
            ),
            hoverinfo="skip",
        ))

    # provenance フットノート: 各ログの DP 重み / autoware バージョン (版差での乖離解釈用)。
    prov_lines = [f"{lbl}: {_prov_text(lbl, d)}" for lbl, d in data.items()]
    fig.add_annotation(
        xref="paper", yref="paper", x=0.0, y=0.0,
        xanchor="left", yanchor="bottom", align="left", showarrow=False,
        text="provenance —<br>" + "<br>".join(prov_lines),
        font=dict(family="monospace", size=9, color="#555555"),
        bgcolor="rgba(255,255,255,0.7)",
    )
    add_params_annotation_plotly(fig)

    name = "trajectory_with_map" if map_ways else "trajectory_xy"
    fig.update_layout(
        title=dict(text=f"{SCENARIO_NAME}<br>軌跡比較", font=dict(size=14)),
        xaxis=dict(title="x [m]", range=[x_min, x_max]),
        yaxis=dict(title="y [m]", range=[y_min, y_max], scaleanchor="x", scaleratio=1),
        height=FIG_HEIGHTS[name],
        autosize=True,
        template="plotly_white",
        legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.7)"),
        margin=dict(l=60, r=20, t=60, b=40),
    )
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    write_plotly_html(fig, FIGS_DIR / f"{name}.html", BASE)


def _ts_plot(ax, t, y, d, label, cmd=False):
    """時系列プロット用ヘルパー。応答=各ログのスタイル、指令=薄い細線。"""
    t = np.asarray(t)
    y = np.asarray(y)
    if cmd:
        ax.plot(t, y, color=d["color"], lw=1.2, ls=":", alpha=0.65, label=label)
    else:
        ax.plot(t, y, color=d["color"], lw=d["lw"], ls=d["ls"], label=label)


def plot_velocity(data: dict):
    fig, axes = _setup_fig("速度比較（指令 vs 応答）", nrows=2, figsize=(14, 8))

    for label, d in data.items():
        vel = d["velocity"]
        cmd = d["cmd"]
        _ts_plot(axes[0, 0], vel["t"], vel["lon_vel"], d, label)
        if not cmd.empty:
            _ts_plot(axes[1, 0], cmd["t"], cmd["cmd_vel"], d, label, cmd=True)

    axes[0, 0].set_title(
        "実応答 (VelocityReport.longitudinal_velocity)  ─ 実線/破線/点鎖線は各ログ"
    )
    axes[0, 0].set_ylabel("速度 [m/s]")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, lw=0.5)

    axes[1, 0].set_title("指令 (control_cmd.longitudinal.velocity)  ─ 点線・薄色")
    axes[1, 0].set_ylabel("速度指令 [m/s]")
    axes[1, 0].set_xlabel("経過時間 [s]")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, lw=0.5)

    fig.text(0.5, 0.005, _TIME_AXIS_NOTE, ha="center", fontsize=8, color="#555555")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    _save(fig, "velocity")


def plot_acceleration(data: dict):
    fig, axes = _setup_fig("加速度比較（指令 vs 応答）", nrows=2, figsize=(14, 8))

    for label, d in data.items():
        acc = d["accel"]
        cmd = d["cmd"]
        _ts_plot(axes[0, 0], acc["t"], acc["accel"], d, label)
        if not cmd.empty:
            _ts_plot(axes[1, 0], cmd["t"], cmd["cmd_accel"], d, label, cmd=True)

    axes[0, 0].set_title("実応答 (localization/acceleration.accel.accel.linear.x)")
    axes[0, 0].set_ylabel("加速度 [m/s²]")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, lw=0.5)

    axes[1, 0].set_title("指令 (control_cmd.longitudinal.acceleration)  ─ 点線・薄色")
    axes[1, 0].set_ylabel("加速度指令 [m/s²]")
    axes[1, 0].set_xlabel("経過時間 [s]")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, lw=0.5)

    fig.text(0.5, 0.005, _TIME_AXIS_NOTE, ha="center", fontsize=8, color="#555555")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    _save(fig, "acceleration")


def plot_steering(data: dict):
    fig, axes = _setup_fig("ステアリング比較（指令 vs 応答）", nrows=2, figsize=(14, 8))

    for label, d in data.items():
        steer = d["steering"]
        cmd = d["cmd"]
        _ts_plot(axes[0, 0], steer["t"], np.degrees(steer["steer"]), d, label)
        if not cmd.empty:
            _ts_plot(axes[1, 0], cmd["t"], np.degrees(cmd["cmd_steer"]), d, label, cmd=True)

    axes[0, 0].set_title("実応答 (SteeringReport.steering_tire_angle)")
    axes[0, 0].set_ylabel("ステア角 [deg]")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, lw=0.5)

    axes[1, 0].set_title("指令 (control_cmd.lateral.steering_tire_angle)  ─ 点線・薄色")
    axes[1, 0].set_ylabel("ステア指令 [deg]")
    axes[1, 0].set_xlabel("経過時間 [s]")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, lw=0.5)

    fig.text(0.5, 0.005, _TIME_AXIS_NOTE, ha="center", fontsize=8, color="#555555")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    _save(fig, "steering")


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
    fig, axes = _setup_fig("速度比較（走行距離基準）", figsize=(14, 5))
    ax = axes[0, 0]
    for label, d in data.items():
        vel, kin = d["velocity"], d["kinematic"]
        if vel.empty or kin.empty:
            continue
        s = _distance_of(vel["t"], kin)
        if s is None:
            continue
        ax.plot(s, np.asarray(vel["lon_vel"]), color=d["color"], lw=d["lw"], ls=d["ls"], label=label)
    ax.set_title("実応答速度 vs 走行距離（早期停止・初期化オフセットを露出）")
    ax.set_xlabel("走行距離 [m]")
    ax.set_ylabel("速度 [m/s]")
    ax.legend(fontsize=9)
    ax.grid(True, lw=0.5)
    fig.tight_layout()
    _save(fig, "velocity_vs_distance")


def plot_steering_vs_distance(data: dict):
    """操舵応答を走行距離 (arc-length) 基準で重ね描き（pacing 差を除いた形状比較）。"""
    fig, axes = _setup_fig("ステアリング比較（走行距離基準）", figsize=(14, 5))
    ax = axes[0, 0]
    for label, d in data.items():
        steer, kin = d["steering"], d["kinematic"]
        if steer.empty or kin.empty:
            continue
        s = _distance_of(steer["t"], kin)
        if s is None:
            continue
        ax.plot(s, np.degrees(np.asarray(steer["steer"])), color=d["color"], lw=d["lw"], ls=d["ls"], label=label)
    ax.set_title("実応答ステア角 vs 走行距離")
    ax.set_xlabel("走行距離 [m]")
    ax.set_ylabel("ステア角 [deg]")
    ax.legend(fontsize=9)
    ax.grid(True, lw=0.5)
    fig.tight_layout()
    _save(fig, "steering_vs_distance")


def plot_curves(data: dict, map_ways: list | None):
    """3つのカーブにフォーカスした軌跡比較（横3列サブプロット）。"""
    if not CURVE_CENTERS:
        return
    n = len(CURVE_CENTERS)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 7))
    fig.suptitle(f"{SCENARIO_NAME}\nカーブ別軌跡比較", fontsize=11)

    for col, curve in enumerate(CURVE_CENTERS):
        ax = axes[col]
        cx, cy, mg = curve["cx"], curve["cy"], curve["margin"]
        x_min, x_max = cx - mg, cx + mg
        y_min, y_max = cy - mg, cy + mg

        # 地図背景
        if map_ways:
            for pts in map_ways:
                wx, wy = pts[:, 0], pts[:, 1]
                if wx.max() < x_min or wx.min() > x_max:
                    continue
                if wy.max() < y_min or wy.min() > y_max:
                    continue
                ax.plot(wx, wy, color="#cccccc", lw=0.6, zorder=1)

        # 各ログの軌跡（bbox内の点のみ）
        for log_label, d in data.items():
            df = d["kinematic"]
            mask = (df["x"] >= x_min) & (df["x"] <= x_max) & (df["y"] >= y_min) & (df["y"] <= y_max)
            seg = df[mask]
            if seg.empty:
                continue
            _traj_plot(ax, seg, d, log_label, markevery=8)

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal")
        ax.set_title(curve["label"], fontsize=10)
        ax.set_xlabel("x [m]")
        if col == 0:
            ax.set_ylabel("y [m]")
        ax.grid(True, lw=0.5, alpha=0.5)

        # 凡例は最初のサブプロットのみ
        if col == 0:
            ax.legend(fontsize=9, loc="best")

    fig.tight_layout()
    _save(fig, "curves_closeup")


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


def plot_curve_analysis(
    data: dict,
    map_ways: list | None,
    curve_idx: int,
    launch_window: tuple[float, float],
):
    """
    指定カーブの一時停止発進からの挙動を軌跡＋時系列で比較。

    レイアウト:
        上段(高め): 対象カーブ付近の軌跡図
        下段3列: 速度 | 加速度 | ステアリング
    """
    if not CURVE_CENTERS or not (0 <= curve_idx < len(CURVE_CENTERS)):
        warnings.warn(f"plot_curve_analysis: curve_idx={curve_idx} が範囲外")
        return
    curve_label = CURVE_CENTERS[curve_idx].get("label", f"カーブ{curve_idx + 1}")

    # ---- 各ログの発進t=0を検出 ----
    launch_t: dict[str, float] = {}
    for label, d in data.items():
        t_launch = _find_curve_launch(d["velocity"], launch_window)
        if t_launch is None:
            warnings.warn(f"{label}: {curve_label}前の停止が見つからないためスキップ")
            continue
        launch_t[label] = t_launch
        print(f"  [{label}] {curve_label}発進 t={t_launch:.1f}s")

    if not launch_t:
        warnings.warn(
            f"発進時刻を検出できるログがないため plot_curve_analysis({curve_label}) をスキップ"
        )
        return

    # ---- 図のセットアップ（2行: 上=軌跡, 下=時系列3列）----
    fig = plt.figure(figsize=(16, 13))
    fig.suptitle(f"{SCENARIO_NAME}\n{curve_label} 一時停止発進からの挙動比較", fontsize=12)
    gs = fig.add_gridspec(2, 3, height_ratios=[1.6, 1.0], hspace=0.38, wspace=0.32)
    ax_map = fig.add_subplot(gs[0, :])  # 上段全幅: 軌跡
    ax_vel = fig.add_subplot(gs[1, 0])  # 下段左: 速度
    ax_acc = fig.add_subplot(gs[1, 1])  # 下段中: 加速度
    ax_str = fig.add_subplot(gs[1, 2])  # 下段右: ステアリング

    # 表示時間範囲（発進前2s ～ カーブ退出後2s）
    T_PRE = -2.0

    # 対象カーブ表示範囲
    c2 = CURVE_CENTERS[curve_idx]
    cx, cy, mg = c2["cx"], c2["cy"], 80

    # ---- 軌跡図 ----
    if map_ways:
        for pts in map_ways:
            wx, wy = pts[:, 0], pts[:, 1]
            if wx.max() < cx - mg or wx.min() > cx + mg:
                continue
            if wy.max() < cy - mg or wy.min() > cy + mg:
                continue
            ax_map.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t_exit = _find_curve_exit(d["kinematic"], curve_idx, t_l)
        t_post = (t_exit - t_l + 2.0) if t_exit is not None else 25.0
        df_k = d["kinematic"]
        # 発進前2s〜カーブ退出後2s の軌跡
        seg = df_k[(df_k["t"] >= t_l + T_PRE) & (df_k["t"] <= t_l + t_post)]
        if seg.empty:
            continue
        _traj_plot(ax_map, seg, d, label, markevery=8)
        # 発進点マーカー（★）
        launch_row = df_k.iloc[(df_k["t"] - t_l).abs().argsort().iloc[0]]
        ax_map.plot(
            launch_row["x"],
            launch_row["y"],
            "*",
            color=d["color"],
            ms=14,
            zorder=6,
            markeredgecolor="white",
            markeredgewidth=0.5,
        )

    ax_map.set_xlim(cx - mg, cx + mg)
    ax_map.set_ylim(cy - mg, cy + mg)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.grid(True, lw=0.5, alpha=0.5)
    ax_map.set_title(
        f"軌跡（★=発進点、表示範囲: 発進前2s〜{curve_label}退出後2s）", fontsize=10
    )
    ax_map.legend(fontsize=10, loc="best")

    # ---- 時系列プロット（発進をt=0に揃える）----
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t_exit = _find_curve_exit(d["kinematic"], curve_idx, t_l)
        t_post = (t_exit - t_l + 2.0) if t_exit is not None else 25.0

        def clip(df, col, _t_l=t_l, _t_post=t_post):
            mask = (df["t"] >= _t_l + T_PRE) & (df["t"] <= _t_l + _t_post)
            sub = df[mask].copy()
            sub["tr"] = sub["t"] - _t_l  # 相対時刻
            return sub

        vel = clip(d["velocity"], "lon_vel")
        acc = clip(d["accel"], "accel")
        steer = clip(d["steering"], "steer")
        cmd = clip(d["cmd"], "cmd_vel")

        # 速度
        _ts_plot(ax_vel, vel["tr"], vel["lon_vel"], d, label)
        if not cmd.empty:
            _ts_plot(ax_vel, cmd["tr"], cmd["cmd_vel"], d, label + "（指令）", cmd=True)

        # 加速度
        _ts_plot(ax_acc, acc["tr"], acc["accel"], d, label)
        if not cmd.empty:
            _ts_plot(ax_acc, cmd["tr"], cmd["cmd_accel"], d, label + "（指令）", cmd=True)

        # ステアリング
        _ts_plot(ax_str, steer["tr"], np.degrees(steer["steer"]), d, label)
        if not cmd.empty:
            _ts_plot(
                ax_str, cmd["tr"], np.degrees(cmd["cmd_steer"]), d, label + "（指令）", cmd=True
            )

    # t=0 縦線
    for ax in (ax_vel, ax_acc, ax_str):
        ax.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)

    ax_vel.set_title("速度", fontsize=10)
    ax_vel.set_ylabel("m/s")
    ax_vel.set_xlabel("発進からの時刻 [s]")
    ax_vel.grid(True, lw=0.5)
    ax_vel.legend(fontsize=7, loc="best")

    ax_acc.set_title("加速度", fontsize=10)
    ax_acc.set_ylabel("m/s²")
    ax_acc.set_xlabel("発進からの時刻 [s]")
    ax_acc.grid(True, lw=0.5)
    ax_acc.legend(fontsize=7, loc="best")

    ax_str.set_title("ステアリング角", fontsize=10)
    ax_str.set_ylabel("deg")
    ax_str.set_xlabel("発進からの時刻 [s]")
    ax_str.grid(True, lw=0.5)
    ax_str.legend(fontsize=7, loc="best")

    _save(fig, f"curve{curve_idx + 1}_analysis")


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
    curve_label = CURVE_CENTERS[curve_idx].get("label", f"カーブ{curve_idx + 1}")

    # 発進時刻を検出
    launch_t: dict[str, float] = {}
    for label, d in data.items():
        t_l = _find_curve_launch(d["velocity"], launch_window)
        if t_l is not None:
            launch_t[label] = t_l

    if not launch_t:
        warnings.warn(
            f"発進時刻を検出できないため plot_curve_steering_detail({curve_label}) をスキップ"
        )
        return

    T_PRE = -2.0

    fig = plt.figure(figsize=(16, 14))
    fig.suptitle(
        f"{SCENARIO_NAME}\n{curve_label} ステアリング詳細分析　─　一時停止発進 t=0", fontsize=12
    )
    gs = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.30, height_ratios=[1.4, 1.0, 1.0])
    ax_map = fig.add_subplot(gs[0, 0])  # 左列上: 軌跡
    ax_ovl = fig.add_subplot(gs[1:, 0])  # 左列下: 指令 vs 応答（大きく）
    ax_rate = fig.add_subplot(gs[0, 1])  # 右列上: 角速度
    ax_err = fig.add_subplot(gs[1, 1])  # 右列中: 追従誤差
    ax_integ = fig.add_subplot(gs[2, 1])  # 右列下: 累積操舵量

    # 対象カーブ付近の軌跡
    c2 = CURVE_CENTERS[curve_idx]
    cx, cy, mg = c2["cx"], c2["cy"], 80
    if map_ways:
        for pts in map_ways:
            wx, wy = pts[:, 0], pts[:, 1]
            if wx.max() < cx - mg or wx.min() > cx + mg:
                continue
            if wy.max() < cy - mg or wy.min() > cy + mg:
                continue
            ax_map.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t_exit = _find_curve_exit(d["kinematic"], curve_idx, t_l)
        t_post = (t_exit - t_l + 2.0) if t_exit is not None else 25.0
        df_k = d["kinematic"]
        seg = df_k[(df_k["t"] >= t_l + T_PRE) & (df_k["t"] <= t_l + t_post)]
        if not seg.empty:
            _traj_plot(ax_map, seg, d, label, markevery=8)
        if df_k.empty:
            continue
        lr = df_k.iloc[(df_k["t"] - t_l).abs().argsort().iloc[0]]
        ax_map.plot(
            lr["x"],
            lr["y"],
            "*",
            color=d["color"],
            ms=13,
            zorder=6,
            markeredgecolor="white",
            markeredgewidth=0.5,
        )

    ax_map.set_xlim(cx - mg, cx + mg)
    ax_map.set_ylim(cy - mg, cy + mg)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.grid(True, lw=0.5, alpha=0.5)
    ax_map.legend(fontsize=9, loc="best")
    ax_map.set_title("軌跡（★=発進点）", fontsize=10)

    # 各ログの時系列ステアリング処理
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t_exit = _find_curve_exit(d["kinematic"], curve_idx, t_l)
        t_post = (t_exit - t_l + 2.0) if t_exit is not None else 25.0
        steer_df = d["steering"]
        cmd_df = d["cmd"]

        mask_s = (steer_df["t"] >= t_l + T_PRE) & (steer_df["t"] <= t_l + t_post)
        mask_c = (cmd_df["t"] >= t_l + T_PRE) & (cmd_df["t"] <= t_l + t_post)
        s = steer_df[mask_s].copy()
        s["tr"] = s["t"] - t_l
        c = cmd_df[mask_c].copy()
        c["tr"] = c["t"] - t_l

        s_deg = np.degrees(s["steer"].values)
        t_s = s["tr"].values

        # --- 左列下: 指令 vs 応答の重ね描き ---
        ax_ovl.plot(t_s, s_deg, color=d["color"], lw=d["lw"], ls=d["ls"], label=f"{label} 応答")
        if not c.empty:
            ax_ovl.plot(
                np.asarray(c["tr"]),
                np.degrees(np.asarray(c["cmd_steer"])),
                color=d["color"],
                lw=1.2,
                ls=":",
                alpha=0.65,
                label=f"{label} 指令",
            )

        # --- 右列上: 角速度 (deg/s) ---
        if len(t_s) > 1:
            dt = np.diff(t_s)
            rate = np.diff(s_deg) / np.where(dt > 1e-6, dt, np.nan)
            # 外れ値を除去
            rate = np.where(np.abs(rate) < 200, rate, np.nan)
            ax_rate.plot(t_s[1:], rate, color=d["color"], lw=d["lw"] * 0.9, ls=d["ls"], label=label)

        # --- 右列中: 追従誤差（応答 − 指令）---
        if not c.empty and len(t_s) > 0:
            cmd_interp = np.interp(t_s, c["tr"].values, np.degrees(c["cmd_steer"].values))
            err = s_deg - cmd_interp
            ax_err.plot(t_s, err, color=d["color"], lw=d["lw"] * 0.9, ls=d["ls"], label=label)

        # --- 右列下: 累積絶対操舵量 ---
        if len(t_s) > 1:
            dt_full = np.diff(t_s)
            incr = np.abs(np.diff(s_deg))
            cumsum = np.concatenate([[0], np.cumsum(incr * dt_full)])
            ax_integ.plot(t_s, cumsum, color=d["color"], lw=d["lw"], ls=d["ls"], label=label)

    # t=0 縦線
    for ax in (ax_ovl, ax_rate, ax_err, ax_integ):
        ax.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)

    ax_ovl.set_title("ステアリング角　指令（点線）vs 応答（実線/破線）", fontsize=10)
    ax_ovl.set_ylabel("ステア角 [deg]")
    ax_ovl.set_xlabel("発進からの時刻 [s]")
    ax_ovl.grid(True, lw=0.5)
    ax_ovl.legend(fontsize=8, ncol=2)

    ax_rate.set_title("ステアリング角速度", fontsize=10)
    ax_rate.set_ylabel("deg/s")
    ax_rate.set_xlabel("発進からの時刻 [s]")
    ax_rate.axhline(0, color="gray", lw=0.6, ls="-")
    ax_rate.grid(True, lw=0.5)
    ax_rate.legend(fontsize=9)

    ax_err.set_title("指令追従誤差（応答 − 指令）", fontsize=10)
    ax_err.set_ylabel("誤差 [deg]")
    ax_err.set_xlabel("発進からの時刻 [s]")
    ax_err.axhline(0, color="gray", lw=0.6, ls="-")
    ax_err.grid(True, lw=0.5)
    ax_err.legend(fontsize=9)

    ax_integ.set_title("累積絶対操舵量（∫|dθ/dt|dt）", fontsize=10)
    ax_integ.set_ylabel("deg")
    ax_integ.set_xlabel("発進からの時刻 [s]")
    ax_integ.grid(True, lw=0.5)
    ax_integ.legend(fontsize=9)

    _save(fig, f"curve{curve_idx + 1}_steering_detail")


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
    curve_label = CURVE_CENTERS[curve_idx].get("label", f"カーブ{curve_idx + 1}")

    launch_t: dict[str, float] = {}
    for label, d in data.items():
        t_l = _find_curve_launch(d["velocity"], launch_window)
        if t_l is not None:
            launch_t[label] = t_l

    if not launch_t:
        return

    T_PRE = -2.0

    fig, axes = plt.subplots(4, 1, figsize=(13, 16), sharex=True)
    fig.suptitle(
        f"{SCENARIO_NAME}\n{curve_label} ステア角 vs 実際の進行方向"
        f"（自転車モデル L={WHEELBASE:.2f}m）",
        fontsize=12,
    )

    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        t_exit = _find_curve_exit(d["kinematic"], curve_idx, t_l)
        t_post = (t_exit - t_l + 2.0) if t_exit is not None else 25.0

        # ---- 時系列データを発進t=0で切り出し ----
        def clip(df, _t_l=t_l, _t_post=t_post):
            m = (df["t"] >= _t_l + T_PRE) & (df["t"] <= _t_l + _t_post)
            sub = df[m].copy()
            sub["tr"] = sub["t"] - _t_l
            return sub

        kin = clip(d["kinematic"])
        vel = clip(d["velocity"])
        steer = clip(d["steering"])

        if kin.empty or vel.empty or steer.empty:
            continue

        t_k = kin["tr"].values

        # yaw unwrap → yaw_rate (rad/s)
        yaw_u = np.unwrap(kin["yaw"].values)
        yaw_rate = np.gradient(yaw_u, t_k)  # rad/s

        # 速度・ステアを kinematic の時刻に補間
        v_i = np.interp(t_k, vel["tr"].values, vel["lon_vel"].values)
        s_i = np.interp(t_k, steer["tr"].values, steer["steer"].values)  # rad

        # 等価ステア角: atan(L * yaw_rate / v)  [rad]
        # v が小さいと不安定 → 0.3m/s 以下はマスク
        v_safe = np.where(v_i > 0.3, v_i, np.nan)
        steer_equiv = np.arctan2(WHEELBASE * yaw_rate, v_safe)  # rad

        # 予測ヨーレート: v * tan(steer) / L  [rad/s]
        yaw_rate_pred = v_i * np.tan(s_i) / WHEELBASE

        # 差分: ステア角 − 等価ステア角  [deg]
        diff_deg = np.degrees(s_i - steer_equiv)
        # 外れ値（v≈0 由来）をクリップ
        diff_deg = np.clip(diff_deg, -30, 30)

        # yaw累積変化量（t=0の値を基準）
        t0_idx = int(np.argmin(np.abs(t_k)))
        yaw_cum = np.degrees(yaw_u - yaw_u[t0_idx])

        # ---- 描画 ----
        kw = dict(color=d["color"], lw=d["lw"], ls=d["ls"])

        # 段1: ステア角 vs 等価ステア角
        axes[0].plot(t_k, np.degrees(s_i), label=f"{label} ステア実測", **kw)
        axes[0].plot(
            t_k,
            np.degrees(steer_equiv),
            color=d["color"],
            lw=1.2,
            ls=":",
            alpha=0.7,
            label=f"{label} 等価（実ヨーレートから逆算）",
        )

        # 段2: 差分
        axes[1].plot(t_k, diff_deg, label=label, **kw)

        # 段3: 実ヨーレート vs 予測ヨーレート
        axes[2].plot(t_k, np.degrees(yaw_rate), label=f"{label} 実測", **kw)
        axes[2].plot(
            t_k,
            np.degrees(yaw_rate_pred),
            color=d["color"],
            lw=1.2,
            ls=":",
            alpha=0.7,
            label=f"{label} 予測（自転車モデル）",
        )

        # 段4: yaw累積変化
        axes[3].plot(t_k, yaw_cum, label=label, **kw)

    # 共通装飾
    for ax in axes:
        ax.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)
        ax.axhline(0, color="gray", lw=0.5)
        ax.grid(True, lw=0.5)
        ax.legend(fontsize=8, ncol=2, loc="best")

    axes[0].set_title(
        "ステア角（実線）vs 等価ステア角（点線）── 一致するほど自転車モデルが成立", fontsize=10
    )
    axes[0].set_ylabel("deg")

    axes[1].set_title(
        f"ステア角 − 等価ステア角（進行方向とのズレ）  ─  L={WHEELBASE:.2f}m", fontsize=10
    )
    axes[1].set_ylabel("deg")

    axes[2].set_title("ヨーレート（実線=実測、点線=自転車モデル予測）", fontsize=10)
    axes[2].set_ylabel("deg/s")

    axes[3].set_title("yaw 累積変化量（t=0 基準）── カーブの総旋回量", fontsize=10)
    axes[3].set_ylabel("deg")
    axes[3].set_xlabel("発進からの時刻 [s]")

    fig.tight_layout()
    _save(fig, f"curve{curve_idx + 1}_yaw_steer")


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
    from scipy.signal import correlate

    T_ONSET_PRE = -0.5  # onset 前の余白 [s]
    T_ONSET_POST = 12.0  # onset 後の表示幅 [s]
    ONSET_THRESH_DEG = 1.0

    fig = plt.figure(figsize=(15, 14))
    fig.suptitle(
        f"{SCENARIO_NAME}\nステアリング応答性能比較　──　ステア入力開始 t=0 に揃えた比較",
        fontsize=12,
    )
    gs = fig.add_gridspec(3, 2, height_ratios=[1.6, 1.0, 1.0], hspace=0.50, wspace=0.30)
    ax_main = fig.add_subplot(gs[0, :])
    ax_err = fig.add_subplot(gs[1, 0])
    ax_rise = fig.add_subplot(gs[1, 1])
    ax_rmse = fig.add_subplot(gs[2, 0])
    ax_peak = fig.add_subplot(gs[2, 1])

    # 定量値を収集
    labels_list = []
    onset_delays = []  # 指令onset → 応答onset の遅延 [s]
    delays = []  # xcorr ラグ [s]
    rmse_list = []
    rise_list = []
    peak_ratio = []

    for label, d in data.items():
        t_l = _find_curve_launch(d["velocity"], launch_window)
        if t_l is None:
            continue

        vel_df = d["velocity"]
        steer_df = d["steering"]
        cmd_df = d["cmd"]

        def clip_rel(df, t_l=t_l):
            m = (df["t"] >= t_l - 3) & (df["t"] <= t_l + 20)
            sub = df[m].copy()
            sub["tr"] = sub["t"] - t_l  # 発進基準の相対時刻
            return sub

        steer_r = clip_rel(steer_df)
        cmd_r = clip_rel(cmd_df)
        if steer_r.empty or cmd_r.empty:
            continue

        cmd_deg = np.degrees(cmd_r["cmd_steer"].values)
        steer_deg = np.degrees(steer_r["steer"].values)

        # ステア指令の onset（発進後で閾値を超えた最初の時刻）
        onset_mask = (cmd_r["tr"] >= 0) & (np.abs(cmd_deg) > ONSET_THRESH_DEG)
        if not onset_mask.any():
            continue
        t_onset_cmd = float(cmd_r["tr"].values[onset_mask][0])

        onset_mask_act = (steer_r["tr"] >= 0) & (np.abs(steer_deg) > ONSET_THRESH_DEG)
        t_onset_act = (
            float(steer_r["tr"].values[onset_mask_act][0]) if onset_mask_act.any() else t_onset_cmd
        )

        onset_delay = t_onset_act - t_onset_cmd

        # onset t=0 で揃えた窓を切り出す
        t_c = cmd_r["tr"].values - t_onset_cmd
        t_a = steer_r["tr"].values - t_onset_cmd

        win_mask_c = (t_c >= T_ONSET_PRE) & (t_c <= T_ONSET_POST)
        win_mask_a = (t_a >= T_ONSET_PRE) & (t_a <= T_ONSET_POST)
        t_c_w, c_w = t_c[win_mask_c], cmd_deg[win_mask_c]
        t_a_w, a_w = t_a[win_mask_a], steer_deg[win_mask_a]

        if len(c_w) == 0 or len(a_w) == 0:
            continue

        # 正規化（ピーク指令値）
        peak_cmd = np.abs(c_w).max()
        peak_act = np.abs(a_w).max()
        c_norm = c_w / peak_cmd if peak_cmd > 0 else c_w
        a_norm = a_w / peak_cmd if peak_cmd > 0 else a_w

        # RMSE（共通グリッドで計算）
        dt = 0.02
        t_grid = np.arange(T_ONSET_PRE, T_ONSET_POST, dt)
        c_i = np.interp(t_grid, t_c_w, c_w, left=np.nan, right=np.nan)
        a_i = np.interp(t_grid, t_a_w, a_w, left=np.nan, right=np.nan)
        valid = ~(np.isnan(c_i) | np.isnan(a_i))
        rmse = float(np.sqrt(np.mean((a_i[valid] - c_i[valid]) ** 2))) if valid.any() else np.nan

        # xcorr による遅延推定
        if valid.sum() > 10:
            c_v, a_v = c_i[valid] - np.nanmean(c_i), a_i[valid] - np.nanmean(a_i)
            corr = correlate(a_v, c_v, mode="full")
            lag = (corr.argmax() - (len(c_v) - 1)) * dt
        else:
            lag = onset_delay

        # 立ち上がり時間（指令ピーク90% への到達）
        thresh90 = 0.90 * peak_cmd
        rise_mask = (t_a_w >= 0) & (np.abs(a_w) >= thresh90)
        rise_time = float(t_a_w[rise_mask][0]) if rise_mask.any() else np.nan

        # ---- メインプロット（正規化）----
        kw = dict(color=d["color"], lw=d["lw"], ls=d["ls"])
        ax_main.plot(t_c_w, c_norm, label=f"{label} 指令", **kw, alpha=0.55, marker=None)
        ax_main.plot(
            t_a_w,
            a_norm,
            label=f"{label} 応答",
            color=d["color"],
            lw=d["lw"] + 0.5,
            ls=d["ls"],
            marker=d["marker"],
            markersize=d["ms"],
            markevery=12,
            markerfacecolor=d["color"],
            markeredgecolor="white",
            markeredgewidth=0.5,
        )

        # 応答ピークに達した時刻をマーカー（点線）
        if rise_mask.any():
            tr_rise = t_a_w[rise_mask][0]
            ax_main.axvline(tr_rise, color=d["color"], lw=1.2, ls=":", alpha=0.6)

        # 応答 onset の縦線（指令 onset t=0 からの遅延を鎖線で表示）
        ax_main.axvline(onset_delay, color=d["color"], lw=2.0, ls="-.", alpha=0.9)

        # ---- 追従誤差 ----
        err_i = np.interp(t_grid, t_a_w, a_w, left=np.nan, right=np.nan) - np.interp(
            t_grid, t_c_w, c_w, left=np.nan, right=np.nan
        )
        ax_err.plot(t_grid, err_i, color=d["color"], lw=d["lw"], ls=d["ls"], label=label)
        # 実機のみ over/under-steer を塗り分け (旧 analyze_real_curve2 の understeer 検証ビューを統合)。
        # 追従誤差 = 応答 − 指令。正=オーバーステア (指令より切れている) / 負=アンダーステア。
        if label == "実機":
            ax_err.fill_between(t_grid, err_i, 0, where=(err_i >= 0),
                                color="tab:red", alpha=0.18, label="実機オーバーステア(+)")
            ax_err.fill_between(t_grid, err_i, 0, where=(err_i < 0),
                                color="tab:blue", alpha=0.18, label="実機アンダーステア(−)")

        # 収集
        labels_list.append(label)
        onset_delays.append(onset_delay)
        delays.append(lag)
        rmse_list.append(rmse)
        rise_list.append(rise_time)
        peak_ratio.append(peak_act / peak_cmd if peak_cmd > 0 else np.nan)

    # 遅延の ←→ アノテーション（ax_main の上端付近に配置）
    y_base = 1.10
    for i, (lbl, delay_s) in enumerate(zip(labels_list, onset_delays)):
        d_cfg = data[lbl]
        yp = y_base + i * 0.09
        x_end = max(delay_s, 0.015)  # 非常に小さい場合も矢印が見えるよう最低幅を確保
        ax_main.annotate(
            "",
            xy=(x_end, yp),
            xytext=(0.0, yp),
            xycoords="data",
            textcoords="data",
            arrowprops=dict(arrowstyle="<->", color=d_cfg["color"], lw=1.8, mutation_scale=10),
            annotation_clip=False,
        )
        ax_main.text(
            x_end / 2,
            yp + 0.035,
            f"{lbl}: {delay_s * 1000:.0f} ms",
            ha="center",
            va="bottom",
            fontsize=8,
            color=d_cfg["color"],
            fontweight="bold",
            clip_on=False,
        )

    # 装飾
    ax_main.axvline(0, color="gray", lw=1.2, ls="--", alpha=0.7, label="入力onset t=0")
    ax_main.axhline(0, color="gray", lw=0.5)
    ax_main.axhline(1.0, color="gray", lw=0.5, ls=":")
    ax_main.axhline(0.9, color="gray", lw=0.5, ls=":", alpha=0.5)
    ax_main.set_ylim(-0.15, 1.45)  # 上部に矢印アノテーション用の余白を確保
    ax_main.set_title(
        "正規化ステア角（ピーク指令=1.0）　実線=応答、薄線=指令\n"
        "縦点線=応答90%到達、縦鎖線=応答onset　↔矢印=応答遅延",
        fontsize=10,
    )
    ax_main.set_ylabel("正規化ステア角 [−]")
    ax_main.set_xlabel("入力開始からの時刻 [s]")
    ax_main.grid(True, lw=0.5)
    ax_main.legend(fontsize=8, ncol=3, loc="lower right")

    ax_err.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)
    ax_err.axhline(0, color="gray", lw=0.5)
    ax_err.set_title("追従誤差（応答 − 指令）", fontsize=10)
    ax_err.set_ylabel("deg")
    ax_err.set_xlabel("入力開始からの時刻 [s]")
    ax_err.grid(True, lw=0.5)
    ax_err.legend(fontsize=9)

    # 棒グラフ
    colors_bar = [data[l]["color"] for l in labels_list if l in data]
    x = np.arange(len(labels_list))
    bw = 0.5

    delay_ms = [d * 1000 if d is not None and not np.isnan(d) else 0 for d in onset_delays]
    ax_rise.bar(x, delay_ms, width=bw, color=colors_bar, alpha=0.85, edgecolor="white")
    ax_rise.set_xticks(x)
    ax_rise.set_xticklabels(labels_list, fontsize=9)
    ax_rise.set_title("応答遅延（指令onset → 応答onset）", fontsize=10)
    ax_rise.set_ylabel("ms")
    ax_rise.grid(True, axis="y", lw=0.5)
    for xi, v_ms in zip(x, delay_ms):
        ax_rise.text(xi, v_ms + 1, f"{v_ms:.0f} ms", ha="center", fontsize=9, fontweight="bold")

    ax_rmse.bar(
        x,
        [r if r is not None and not np.isnan(r) else 0 for r in rmse_list],
        width=bw,
        color=colors_bar,
        alpha=0.85,
        edgecolor="white",
    )
    ax_rmse.set_xticks(x)
    ax_rmse.set_xticklabels(labels_list, fontsize=9)
    ax_rmse.set_title("RMSE（指令追従誤差）", fontsize=10)
    ax_rmse.set_ylabel("deg")
    ax_rmse.grid(True, axis="y", lw=0.5)
    for xi, v in zip(x, rmse_list):
        if v is not None and not np.isnan(v):
            ax_rmse.text(xi, v + 0.02, f"{v:.3f}°", ha="center", fontsize=9)

    ax_peak.bar(
        x,
        [r if r is not None and not np.isnan(r) else 0 for r in peak_ratio],
        width=bw,
        color=colors_bar,
        alpha=0.85,
        edgecolor="white",
    )
    ax_peak.axhline(1.0, color="gray", lw=1.0, ls="--", alpha=0.6)
    ax_peak.set_xticks(x)
    ax_peak.set_xticklabels(labels_list, fontsize=9)
    ax_peak.set_title("ピーク追従率（応答ピーク / 指令ピーク）", fontsize=10)
    ax_peak.set_ylabel("−")
    ax_peak.set_ylim(0, 1.3)
    ax_peak.grid(True, axis="y", lw=0.5)
    for xi, v in zip(x, peak_ratio):
        if v is not None and not np.isnan(v):
            ax_peak.text(xi, v + 0.02, f"{v:.3f}", ha="center", fontsize=9)

    fig.tight_layout()
    _save(fig, f"curve{curve_idx + 1}_steer_response")


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
    plot_trajectory(loaded, map_ways)
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
