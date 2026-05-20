"""イベント検出（AUTONOMOUS 開始、カーブ② 発進・脱出）の純関数.

設定は引数で受け取り、curve_config YAML や RuntimeConfig には直接依存しない。
すべて align_time 済み (列 `t` を持つ) DataFrame を前提とする。
"""

from __future__ import annotations

import warnings

import numpy as np
import pandas as pd


# autoware_adapi_v1_msgs.OperationModeState.AUTONOMOUS == 2
AUTONOMOUS_MODE = 2


def find_autonomous_start(df_mode: pd.DataFrame, df_vel: pd.DataFrame) -> int:
    """オートノマス開始時刻 (ns) を返す。

    df_mode に AUTONOMOUS 遷移があればそれ、無ければ速度ベースでフォールバック。
    df_vel は align_time 前 (`t_ns` 列を持つ) を想定。
    """
    if not df_mode.empty:
        auto_rows = df_mode[df_mode["mode"] == AUTONOMOUS_MODE]
        if not auto_rows.empty:
            return int(auto_rows["t_ns"].iloc[0])
        warnings.warn(
            "AUTONOMOUS モード遷移が見つからないため速度ベースにフォールバック",
            stacklevel=2,
        )
    if df_vel.empty:
        raise ValueError("df_mode も df_vel も空のため t0 を決定できない")
    moving = df_vel[df_vel["lon_vel"] > 0.1]
    if not moving.empty:
        return int(moving["t_ns"].iloc[0])
    return int(df_vel["t_ns"].iloc[0])


def find_curve2_launch(
    df_vel: pd.DataFrame,
    window: tuple[float, float] = (20.0, 120.0),
    stop_threshold: float = 0.05,
    min_stop_duration: float = 0.5,
    stop_gap: float = 2.0,
) -> float | None:
    """カーブ② 直前の一時停止から発進する時刻 [s] を返す。

    df_vel は align_time 済み (列 't' を持つ) を想定。
    停止区間を列挙し、`window` 範囲内で終わる停止のうち最も早い終端時刻を採用。
    """
    if df_vel.empty or "t" not in df_vel.columns:
        return None
    stopped = df_vel[df_vel["lon_vel"] < stop_threshold]["t"].values
    if len(stopped) == 0:
        return None
    gaps = np.where(np.diff(stopped) > stop_gap)[0]
    starts_idx = np.concatenate([[0], gaps + 1])
    ends_idx = np.concatenate([gaps, [len(stopped) - 1]])
    candidates = []
    for s, e in zip(starts_idx, ends_idx):
        dur = stopped[e] - stopped[s]
        t_end = float(stopped[e])
        if dur >= min_stop_duration and window[0] <= t_end <= window[1]:
            candidates.append(t_end)
    if not candidates:
        return None
    return float(min(candidates))


def find_curve2_exit(
    df_kin: pd.DataFrame,
    curve_center: tuple[float, float],
    t_launch: float,
    radius: float = 30.0,
) -> float | None:
    """カーブ② 領域 (中心から `radius` m 以内) を抜け出す時刻 [s] を返す。

    df_kin は align_time 済み (列 't', 'x', 'y' を持つ) を想定。
    `t_launch` 以降の軌跡で、領域に入った後に最初に外に出たタイミング。
    """
    if df_kin.empty:
        return None
    cx, cy = curve_center
    df_after = df_kin[df_kin["t"] >= t_launch]
    if df_after.empty:
        return None
    dist = np.sqrt((df_after["x"].values - cx) ** 2 + (df_after["y"].values - cy) ** 2)
    t_vals = df_after["t"].values
    entered = False
    for ins, t in zip(dist < radius, t_vals):
        if ins:
            entered = True
        elif entered:
            return float(t)
    return None


def find_sim_launch(
    df_vel: pd.DataFrame,
    threshold: float = 0.5,
    min_t: float = 5.0,
) -> float | None:
    """信号停止のないシム用の単純な発進検出。

    df_vel は align_time 済みを想定。
    `min_t` 秒以降で `threshold` m/s を超えた最初の時刻を返す。
    """
    if df_vel.empty:
        return None
    moving = df_vel[(df_vel["lon_vel"] > threshold) & (df_vel["t"] >= min_t)]
    return float(moving["t"].iloc[0]) if not moving.empty else None
