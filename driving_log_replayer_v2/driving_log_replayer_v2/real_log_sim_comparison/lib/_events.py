"""イベント検出（AUTONOMOUS 開始、発進）の純関数.

設定は引数で受け取り、RuntimeConfig には直接依存しない。
すべて align_time 済み (列 `t` を持つ) DataFrame を前提とする。
"""

from __future__ import annotations

import warnings

import numpy as np
import pandas as pd


# autoware_adapi_v1_msgs.OperationModeState.AUTONOMOUS == 2
AUTONOMOUS_MODE = 2


def _debounced_moving_start(
    df_vel: pd.DataFrame,
    vel_threshold: float,
    min_moving_duration: float,
) -> int:
    """速度が `min_moving_duration` 秒継続して `vel_threshold` を超える最初の時刻 (ns) を返す。

    単点ノイズに脆弱なため、窓内に複数サンプルがあり全て移動中である場合のみ
    「継続した発進」とみなす（デバウンス）。df_vel は align_time 前 (`t_ns` 列を持つ) を想定。
    """
    t_ns = df_vel["t_ns"].to_numpy()
    moving = (df_vel["lon_vel"].to_numpy() > vel_threshold)
    win_ns = min_moving_duration * 1e9
    for i in np.flatnonzero(moving):
        t_start = t_ns[i]
        in_win = (t_ns >= t_start) & (t_ns < t_start + win_ns)
        if in_win.sum() >= 2 and moving[in_win].all():
            return int(t_start)
    if moving.any():
        return int(t_ns[int(np.argmax(moving))])
    return int(t_ns[0])


def find_autonomous_start(
    df_mode: pd.DataFrame,
    df_vel: pd.DataFrame,
    *,
    vel_threshold: float = 0.1,
    min_moving_duration: float = 0.3,
) -> int:
    """解析開始時刻 (ns) を返す。

    先頭の「停止継続区間」と先頭の「MANUAL 区間」（AUTONOMOUS 以外）を、それぞれ
    独立に判定して一括カットし、両者の終端の遅い方 (和集合) を採用する。
    - 停止継続区間の終端: `min_moving_duration` 秒だけ移動が継続する最初の時刻
      （単点ノイズ除けのデバウンス、df_vel は align_time 前 `t_ns` 列を想定）
    - MANUAL 区間の終端: df_mode 中で最初に AUTONOMOUS になった時刻
      （df_mode が空、または AUTONOMOUS が一度も観測されない場合は制約なし）
    """
    if df_vel.empty:
        raise ValueError("df_vel が空のため停止区間カットの終端を決定できず t0 を決定できない")

    stop_cut_ns = _debounced_moving_start(df_vel, vel_threshold, min_moving_duration)

    if df_mode.empty:
        manual_cut_ns = int(df_vel["t_ns"].iloc[0])
    else:
        auto_rows = df_mode[df_mode["mode"] == AUTONOMOUS_MODE]
        if auto_rows.empty:
            warnings.warn(
                "df_mode に AUTONOMOUS が見つからないため速度カットのみを採用します",
                stacklevel=2,
            )
            manual_cut_ns = int(df_vel["t_ns"].iloc[0])
        else:
            manual_cut_ns = int(auto_rows["t_ns"].iloc[0])

    return max(stop_cut_ns, manual_cut_ns)


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


def find_initial_launch(
    df_vel: pd.DataFrame,
    threshold: float = 0.5,
    min_moving_duration: float = 0.5,
) -> float | None:
    """初期停止からの「最初の継続的な発進」時刻 [s] を返す (curve② 非設定時の汎用 anchor)。

    df_vel は align_time 済み (列 't') を想定。`threshold` m/s を超え、かつ
    `min_moving_duration` 秒だけ移動が継続する最初の時刻を採用する (単点ノイズ除けの
    デバウンス。`find_autonomous_start` の速度フォールバックと同方式)。

    `find_sim_launch` の `min_t` (既定 5s) のような固定窓を持たないため、5s より早く発進する
    走行や、長い初期停止のあと発進する走行 (例: stop→drive→stop の完全 start→goal) でも
    正しく初回発進を捉える。real / sim 双方に同一適用して整列の非対称性をなくすのが目的。
    """
    if df_vel.empty or "t" not in df_vel.columns:
        return None
    t = df_vel["t"].to_numpy()
    moving = df_vel["lon_vel"].to_numpy() > threshold
    win = min_moving_duration  # 秒 (t は align_time 済みで秒単位)
    for i in np.flatnonzero(moving):
        t_start = t[i]
        in_win = (t >= t_start) & (t < t_start + win)
        if in_win.sum() >= 2 and moving[in_win].all():
            return float(t_start)
    if moving.any():
        return float(t[int(np.argmax(moving))])
    return None
