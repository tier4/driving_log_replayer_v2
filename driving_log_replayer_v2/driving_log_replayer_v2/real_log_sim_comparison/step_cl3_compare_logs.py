#!/usr/bin/env python3
"""実機ログ vs Godotシム vs 通常シム の三方比較プロット生成スクリプト.

Outputs: comparison/figures/*.svg (軌跡比較のみ *.html), comparison/report.md
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import warnings

import numpy as np
import pandas as pd

from .lib._events import (
    AUTONOMOUS_MODE as _AUTONOMOUS_MODE,
    find_autonomous_start as _find_autonomous_start,
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
from .lib._coverage import compute_coverage
from .lib._map import load_map_ways, resolve_map_osm
from .lib._model_viewer import plot_model_viewer
from .lib._params_utils import load_sim_params
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

WHEELBASE = 5.15  # m — kinematic_state × steering から実データで推定

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
# trajectory_playback.html の「距離軸」モードで対話的に確認できる。
_TIME_AXIS_NOTE = (
    "注: t=0 は各ログの初回発進 (initial launch)。実機の初期停止時間が長くても全ログが発進で揃う"
    "（pacing 差により同一 t は同一地点を意味しない。走行距離基準は軌跡再生ビューアの距離軸モードを参照）"
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


# ---------------------------------------------------------------------------
# 数値レポート
# ---------------------------------------------------------------------------


def nearest_point_distance(ref_xy: np.ndarray, query_xy: np.ndarray) -> np.ndarray:
    """各query点に対するref_xyの最近傍距離を返す。"""
    from scipy.spatial import cKDTree

    tree = cKDTree(ref_xy)
    dists, _ = tree.query(query_xy)
    return dists


# ゴール近傍除外 [m]。rosbag 終端のゴール付近は、実際に Autoware へ与えたゴールと一致せず
# (開始は初期状態を丁寧に合わせ込むが終端はそうでない)、終端の動きの差は構造的に不可避なため
# 精度算出から外す。env GOAL_EXCLUSION_M で上書き可 (0 で無効)。既定は closed-loop の
# goal_vicinity_tolerance 既定値 (30m) に合わせる。
try:
    GOAL_EXCLUSION_M = float(os.environ.get("GOAL_EXCLUSION_M", "30.0"))
except (TypeError, ValueError):
    GOAL_EXCLUSION_M = 30.0


def _goal_cut_time(kin: pd.DataFrame, goal_xy: tuple[float, float], excl_m: float) -> float:
    """ログがゴール近傍 (goal_xy から excl_m 以内) へ最終接近して入る時刻を返す。
    その時刻以降 (ゴール近傍区間) を精度算出から除外する。近傍に入らない (手前で停止等) や
    excl_m<=0 のときは inf (除外なし)。point-to-point 走行前提 (start は goal から遠い)。"""
    if kin.empty or excl_m <= 0.0 or "t" not in kin:
        return float("inf")
    x = kin["x"].to_numpy()
    y = kin["y"].to_numpy()
    t = kin["t"].to_numpy()
    inside = np.hypot(x - goal_xy[0], y - goal_xy[1]) < excl_m
    if not inside.any():
        return float("inf")
    idx = np.where(inside)[0]
    start = idx[-1]
    while start - 1 >= 0 and inside[start - 1]:
        start -= 1
    return float(t[start])


def _before(df: pd.DataFrame, t_cut: float) -> pd.DataFrame:
    """t < t_cut の行に限定 (ゴール近傍除外)。t_cut=inf なら全行。"""
    if df.empty or "t" not in df or not np.isfinite(t_cut):
        return df
    return df[df["t"].to_numpy() < t_cut]


def _log_summary(d: dict, t_cut: float = float("inf")) -> dict:
    """1 ログの走行サマリ (kinematic odometry 基準 + velocity_status 参考)。
    t_cut 未満 (ゴール近傍を除外した区間) で集計する。"""
    kin, vel = _before(d["kinematic"], t_cut), _before(d["velocity"], t_cut)
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


def _finite(x) -> float | None:
    """有限値なら float、それ以外 (NaN/inf/None) は None (JSON 化で NaN を出さない)。"""
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return None
    return xf if math.isfinite(xf) else None


def _fmt(x: float | None, prec: int = 1) -> str:
    """Markdown 表セル用の数値整形 (None は em-dash)。"""
    return f"{x:.{prec}f}" if x is not None else "—"


def _cmd_tracking_rmse(d: dict, t_cut: float = float("inf")) -> dict:
    """指令 vs 応答の RMSE (速度 [m/s]・ステア [deg])。データ欠落は None。
    t_cut 未満 (ゴール近傍を除外した区間) で算出する。"""
    out: dict = {"vel_rmse_mps": None, "steer_rmse_deg": None}
    cmd, vel, steer = d["cmd"], _before(d["velocity"], t_cut), _before(d["steering"], t_cut)
    if not cmd.empty and not vel.empty:
        cmd_i = np.interp(vel["t"], cmd["t"], cmd["cmd_vel"])
        out["vel_rmse_mps"] = _finite(np.sqrt(np.mean((vel["lon_vel"].values - cmd_i) ** 2)))
    if not cmd.empty and not steer.empty:
        cmd_i = np.interp(steer["t"], cmd["t"], cmd["cmd_steer"])
        out["steer_rmse_deg"] = _finite(
            np.degrees(np.sqrt(np.mean((steer["steer"].values - cmd_i) ** 2)))
        )
    return out


def compute_closed_loop_metrics(data: dict) -> dict:
    """closed-loop 比較メトリクスを機械可読 dict で計算する (report.md と JSON の共通ソース)。

    走行サマリ (_log_summary)・指令追従 RMSE・軌跡乖離 (双方向最近傍)・完走率に加え、
    実機ログの走行特性カバレッジ (lib._coverage) を real.coverage に格納する。
    step13_cross_dataset がデータセット横断行列の入力に使うため、値はすべて
    JSON 化可能 (NaN は None に正規化) にする。
    """
    real = data.get("実機")
    real_kin = real["kinematic"] if real is not None else pd.DataFrame()
    # ゴール = 実機 kinematic 終端位置。各ログのゴール近傍区間 (GOAL_EXCLUSION_M 以内) は
    # 実際の Autoware ゴールと一致せず動きの差が構造的に不可避なため、全精度算出から除外する。
    goal_xy = (
        (float(real_kin["x"].to_numpy()[-1]), float(real_kin["y"].to_numpy()[-1]))
        if not real_kin.empty
        else None
    )
    t_cut = {
        label: (
            _goal_cut_time(d["kinematic"], goal_xy, GOAL_EXCLUSION_M)
            if goal_xy is not None
            else float("inf")
        )
        for label, d in data.items()
    }
    real_cut = t_cut.get("実機", float("inf"))
    real_kin = _before(real_kin, real_cut)
    real_dist = (
        _finite(cumulative_arc_length(real_kin["x"].to_numpy(), real_kin["y"].to_numpy())[-1])
        if not real_kin.empty
        else None
    )
    summaries = {
        label: {k: _finite(v) for k, v in _log_summary(d, t_cut[label]).items()}
        for label, d in data.items()
    }

    real_rec = None
    if real is not None:
        real_rec = {
            "summary": summaries["実機"],
            **_cmd_tracking_rmse(real, real_cut),
            "coverage": compute_coverage(
                _before(real["kinematic"], real_cut), _before(real["velocity"], real_cut),
                _before(real["accel"], real_cut), _before(real["steering"], real_cut),
                wheelbase=float(WHEELBASE),
            ),
            "goal_exclusion_m": GOAL_EXCLUSION_M,
        }

    ref_xy = real_kin[["x", "y"]].to_numpy() if not real_kin.empty else None
    runs: dict = {}
    for label, d in data.items():
        if label == "実機":
            continue
        rec: dict = {
            "vehicle_model": d.get("vehicle_model", ""),
            "perfect": bool(d.get("perfect", False)),
            "summary": summaries[label],
            **_cmd_tracking_rmse(d, t_cut[label]),
            "completion_pct": None,
            "s2r_mean_m": None, "s2r_max_m": None,
            "r2s_mean_m": None, "r2s_max_m": None,
            "status": "データ不足",
        }
        dist = summaries[label]["dist"]
        if real_dist and dist is not None:
            rec["completion_pct"] = dist / real_dist * 100.0
        kin_c = _before(d["kinematic"], t_cut[label])
        q_xy = (
            kin_c[["x", "y"]].to_numpy() if not kin_c.empty
            else np.empty((0, 2))
        )
        if ref_xy is not None and len(q_xy) >= 10:
            try:
                s2r = nearest_point_distance(ref_xy, q_xy)
                r2s = nearest_point_distance(q_xy, ref_xy)
                comp = rec["completion_pct"]
                rec.update(
                    s2r_mean_m=float(s2r.mean()), s2r_max_m=float(s2r.max()),
                    r2s_mean_m=float(r2s.mean()), r2s_max_m=float(r2s.max()),
                    status="OK" if (comp is not None and comp >= 85.0) else "早期停止/未完走",
                )
            except Exception as e:  # noqa: BLE001
                rec["status"] = f"ERR: {e}"
        runs[label] = rec

    return {
        "schema_version": 1,
        "scenario_name": SCENARIO_NAME,
        "real": real_rec,
        "real_dist_m": real_dist,
        "runs": runs,
    }


def build_report(metrics: dict, data: dict) -> str:
    """compute_closed_loop_metrics の結果と data (provenance/診断用) から report.md を整形する。"""
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

    def _rec(label: str) -> dict | None:
        return metrics["real"] if label == "実機" else metrics["runs"].get(label)

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
        s = _rec(label)["summary"]
        lines.append(
            f"| {label} | {_fmt(s['elapsed'])} | {_fmt(s['dist'])} | "
            f"{_fmt(s['mean_speed'], 3)} | {_fmt(s['cruise'], 3)} | {_fmt(s['stopped'])} |"
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
        s = _rec(label)["summary"]
        lines.append(
            f"| {label} | {_fmt(s['vmean'], 3)} | {_fmt(s['vmax'], 3)} | {_fmt(s['vstd'], 3)} |"
        )
    lines.append("")

    # 速度指令 RMSE（指令 vs 応答）
    lines.append("## 速度 RMSE（指令 vs 応答）\n")
    lines.append("| ログ | RMSE [m/s] |")
    lines.append("|---|---|")
    for label in data:
        rmse = _rec(label)["vel_rmse_mps"]
        lines.append(f"| {label} | {_fmt(rmse, 4) if rmse is not None else 'N/A'} |")
    lines.append("")

    # ステア RMSE（指令 vs 応答） + perfect_tracker 注記 (B2)
    lines.append("## ステアリング RMSE（指令 vs 応答）\n")
    lines.append("| ログ | RMSE [deg] | 備考 |")
    lines.append("|---|---|---|")
    has_perfect = False
    for label, d in data.items():
        rmse = _rec(label)["steer_rmse_deg"]
        if rmse is None:
            lines.append(f"| {label} | N/A | |")
            continue
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
    real_dist = metrics["real_dist_m"]
    if real_dist is None:
        lines.append("| (実機 kinematic なし) | — | — | — | — | データ不足 |")
    else:
        lines.append(f"| 実機 | {real_dist:.1f} | 100.0 (基準) | — | — | 基準 |")
        for label, rec in metrics["runs"].items():
            cells = [
                _fmt(rec["summary"]["dist"]),
                _fmt(rec["completion_pct"]),
            ]
            if rec["s2r_mean_m"] is not None:
                cells.append(f"{rec['s2r_mean_m']:.3f}/{rec['s2r_max_m']:.3f}")
                cells.append(f"{rec['r2s_mean_m']:.3f}/{rec['r2s_max_m']:.3f}")
            else:
                cells += ["—", "—"]
            lines.append(f"| {label} | " + " | ".join(cells) + f" | {rec['status']} |")
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
    global SCENARIO_NAME, WHEELBASE, LOGS  # noqa: PLW0603

    BASE = cfg.base_dir
    LITE_DIR = cfg.lite_dir
    OUT_DIR = cfg.out_dir
    FIGS_DIR = cfg.figs_dir

    SCENARIO_NAME = cfg.scenario_name
    WHEELBASE = float(cfg.wheelbase_validation)

    # scenario.yaml 連動: cfg.scenario_config が指定されていれば Conditions.sim_runs の全 run を LOGS dict に追加
    sim_runs_cfg = None
    if cfg.scenario_config:
        try:
            from .lib._sim_runs_config import load_sim_runs_config  # noqa: PLC0415
            sim_runs_cfg = load_sim_runs_config(cfg.scenario_config)
        except Exception as exc:  # noqa: BLE001
            warnings.warn(f"scenario.yaml (sim_runs) 読み込み失敗: {exc} (sim 重ね描きスキップ)")

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

    print("\n=== メトリクス算出中 ===")
    metrics: dict | None = None
    try:
        metrics = compute_closed_loop_metrics(loaded)
        metrics_path = OUT_DIR / "metrics_closed_loop.json"
        metrics_path.write_text(
            json.dumps(metrics, ensure_ascii=False, allow_nan=False, indent=1),
            encoding="utf-8",
        )
        print(f"  保存: {metrics_path}")
        report = build_report(metrics, loaded)
        report_path = OUT_DIR / "report.md"
        report_path.write_text(report, encoding="utf-8")
        print(f"  保存: {report_path}")
    except Exception as e:  # noqa: BLE001
        warnings.warn(f"レポート生成失敗: {e}")

    print("\n=== プロット生成中 ===")
    # 軌跡再生ビューア (時刻同期/位置同期シークバー付き自己完結 HTML)
    # metrics を渡して凡例パネルにクローズループ指標を表示
    plot_trajectory_playback(loaded, map_ways, FIGS_DIR, title=SCENARIO_NAME, metrics=metrics)
    # 縦横独立モデル検証ビューア (実機のみ・指令→モデル積算 vs 観測、T/τ つまみ調整)。
    # モデルレジストリ (scenario.yaml の models) を渡し、対応モデル (best_normal 等) の params を
    # ドロップダウンから簡単に適用できるようにする。
    model_registry: dict = {}
    if cfg.scenario_config:
        try:
            from .lib._models_config import load_models_doc  # noqa: PLC0415
            model_registry = {
                name: dict(spec.params)
                for name, spec in load_models_doc(cfg.scenario_config).models.items()
            }
        except Exception as exc:  # noqa: BLE001
            warnings.warn(f"model registry 読み込み失敗 (ビューアは spec のみ): {exc}")
    plot_model_viewer(
        loaded, map_ways, FIGS_DIR, load_sim_params(), title=SCENARIO_NAME,
        model_registry=model_registry,
    )

    print("\n完了。出力先:", FIGS_DIR)


if __name__ == "__main__":
    main()
