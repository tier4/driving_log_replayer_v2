"""step5（N-step オープンループ解析）の図を組む build_fig_* 純関数群.

入力は `nstep_delta.csv` 由来の DataFrame（horizon, tr, err_ds_long/lat, err_steer,
yaw_err_deg, real_vx, ... の統一スキーマ）。横断軸統一（旧 LIMITS_DF グローバル）は
`limits` 引数で注入する（単一ケースは None で自己スケール、step6 が全ケース連結から
作って渡す）。ROS 非依存。
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import plotly.graph_objects as go

from ._common import (
    apply_base_layout,
    axis_range_from_limits,
    ma_window,
    make_grid,
)

# AUTONOMOUS 開始（tr=0）を示す緑点線の共通体裁。
_VLINE = dict(line=dict(color="green", width=1.0, dash="dot"))


def _ma(vals: np.ndarray, n: int) -> np.ndarray:
    """中央寄せ移動平均（端は短縮窓）。matplotlib 側の rolling(center, min_periods=1) と等価。"""
    return pd.Series(vals).rolling(ma_window(n), center=True, min_periods=1).mean().to_numpy()


def _set_yrange(fig: go.Figure, row: int, col: int, rng) -> None:
    if rng is not None:
        fig.update_yaxes(range=rng, row=row, col=col)


def _raw_ma_traces(fig, row, col, tr, vals, *, raw_color, ma_color, ma_name,
                   ma_dash="solid", legendgroup=None, showlegend=True):
    """raw（薄線）+ 移動平均（太線）の 2 trace を 1 パネルに足す共通処理。"""
    fig.add_trace(go.Scatter(
        x=tr, y=vals, mode="lines", showlegend=False, legendgroup=legendgroup,
        line=dict(color=raw_color, width=0.4), opacity=0.45, hoverinfo="skip",
    ), row=row, col=col)
    fig.add_trace(go.Scatter(
        x=tr, y=_ma(vals, len(vals)), mode="lines", name=ma_name,
        legendgroup=legendgroup, showlegend=showlegend,
        line=dict(color=ma_color, width=1.6, dash=ma_dash),
    ), row=row, col=col)


def build_fig_overview(
    df: pd.DataFrame, *, params: dict | None = None, limits_df=None
) -> go.Figure:
    """N=1 概観 2×2（速度・加速度・縦誤差・横誤差）。旧 plot_overview の plotly 版。"""
    tr = df["tr"].to_numpy()
    fig = make_grid(
        2, 2,
        # 行 1 にも x 軸タイトルを付けるため、行 2 の 2 行サブプロットタイトルと衝突しないよう
        # 行間を広げる（既定 0.10 では重なる）。
        vertical_spacing=0.16,
        subplot_titles=[
            "速度: 実機 vs モデル<br><sub>実機: kinematic_state/twist.linear.x  モデル: state_[3]</sub>",
            "加速度: 指令 vs 実機<br><sub>指令: control_cmd/longitudinal.acceleration  実機: acceleration/accel.linear.x</sub>",
            "1ステップ縦方向誤差<br><sub>実機: kinematic_state/pose.position  モデル: state_[0,1]</sub>",
            "1ステップ横方向誤差<br><sub>実機: kinematic_state/pose.position  モデル: state_[0,1]</sub>",
        ],
    )
    # 速度 (0,0)
    fig.add_trace(go.Scatter(x=tr, y=df["real_vx"], mode="lines", name="実機 vx",
                             line=dict(color="blue", width=1.2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=tr, y=df["sim_vx"], mode="lines", name="モデル vx",
                             line=dict(color="red", width=1.0, dash="dash")), row=1, col=1)
    fig.update_yaxes(title_text="速度 [m/s]", row=1, col=1)
    _set_yrange(fig, 1, 1, axis_range_from_limits(limits_df, ["real_vx", "sim_vx"], horizon=1))
    # 加速度 (0,1)
    fig.add_trace(go.Scatter(x=tr, y=df["accel_des"], mode="lines", name="指令 accel_des",
                             line=dict(color="gray", width=0.8)), row=1, col=2)
    fig.add_trace(go.Scatter(x=tr, y=df["real_ax"], mode="lines", name="実機 ax",
                             line=dict(color="blue", width=1.0)), row=1, col=2)
    fig.update_yaxes(title_text="加速度 [m/s²]", row=1, col=2)
    # 縦誤差 (1,0) / 横誤差 (1,1)
    for col, errcol, ylabel in [(1, "err_ds_long", "縦方向誤差 [cm]"), (2, "err_ds_lat", "横方向誤差 [cm]")]:
        vals = df[errcol].to_numpy(dtype=float) * 100
        _raw_ma_traces(fig, 2, col, tr, vals, raw_color="gray", ma_color="red",
                       ma_name="移動平均", showlegend=col == 1)
        fig.add_hline(y=0, line=dict(color="black", width=0.8), row=2, col=col)
        fig.update_yaxes(title_text=ylabel, row=2, col=col)
        _set_yrange(fig, 2, col, axis_range_from_limits(limits_df, errcol, 100, horizon=1))
    for r in (1, 2):
        for c in (1, 2):
            fig.add_vline(x=0, row=r, col=c, **_VLINE)
            fig.update_xaxes(title_text="AUTONOMOUS 開始からの時刻 [s]", row=r, col=c)
    return apply_base_layout(
        fig,
        title="全走行 N=1 (per-step) 分析<br><sub>(各ステップで実機状態にリセット — 計画挙動の差を除外)</sub>",
        height=820,
    )
