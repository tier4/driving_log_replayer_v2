"""step9（発進 brake_time_constant 同定）の図を組む build_fig_* 純関数群.

実機 post-gate cmd をオープンループ入力した発進シミュレーション速度と実機 actual 速度の
比較、cmd 加速度時系列、brake_tc スイープ RMSE を組む。ROS 非依存。
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ._common import (
    add_params_annotation_plotly,
    apply_base_layout,
    make_grid,
    viridis_colors,
)

_VLINE = dict(line=dict(color="gray", width=0.8, dash="dash"))


def build_fig_departure_brake_sensitivity(
    real_t: np.ndarray,
    real_v: np.ndarray,
    t_sim: np.ndarray,
    sim_results: dict[float, np.ndarray],
    variants: list[float],
    labels: list[str],
    *,
    scenario_name: str = "",
    params: dict | None = None,
) -> go.Figure:
    """発進速度プロファイルの brake_tc 感度（左:発進前後 / 右:発進直後ズーム）。

    両パネルに同じ系列（実機 actual 黒太線 + brake_tc バリアント viridis 破線）を描き、
    表示範囲だけ変える。凡例は左パネルのみ。
    """
    colors = viridis_colors(len(variants))
    fig = make_grid(
        1, 2,
        subplot_titles=["速度プロファイル比較（発進前後）", "発進直後ズーム (t=-0.5~3s)"],
    )
    for col in (1, 2):
        show = col == 1
        fig.add_trace(go.Scatter(
            x=real_t, y=real_v, mode="lines", name="実機 actual速度",
            legendgroup="real", showlegend=show,
            line=dict(color="#000000", width=3),
        ), row=1, col=col)
        for btc, color, lbl in zip(variants, colors, labels):
            fig.add_trace(go.Scatter(
                x=t_sim, y=sim_results[btc], mode="lines", name=f"FMU {lbl}",
                legendgroup=f"btc{btc}", showlegend=show,
                line=dict(color=color, width=1.8, dash="dash"),
            ), row=1, col=col)
        fig.add_vline(x=0, row=1, col=col, **_VLINE)
        fig.update_yaxes(title_text="速度 [m/s]", row=1, col=col)
        fig.update_xaxes(title_text="発進後 t [s]", row=1, col=col)
    fig.update_xaxes(range=[-1, 10], row=1, col=1)
    fig.update_yaxes(range=[-0.5, 8], row=1, col=1)
    fig.update_xaxes(range=[-0.5, 3.0], row=1, col=2)
    fig.update_yaxes(range=[-0.2, 2.5], row=1, col=2)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title=f"{scenario_name}<br>発進動作：brake_time_constant 感度分析"
        "<br><sub>(実機 post-gate cmd をオープンループ入力→シミュレーション速度 vs 実機 actual速度)</sub>",
        height=520,
    )


def build_fig_real_cmd_acc(
    t: np.ndarray, cmd_accel: np.ndarray, *, scenario_name: str = "", params: dict | None = None
) -> go.Figure:
    """実機 post-gate control_cmd 加速度の発進前後時系列。"""
    fig = make_grid(1, 1)
    fig.add_trace(go.Scatter(
        x=t, y=cmd_accel, mode="lines", name="実機 cmd_acc (post-gate)",
        line=dict(color="#1f77b4", width=1.5),
    ))
    fig.add_vline(x=0, **_VLINE)
    fig.update_xaxes(title_text="発進後 t [s]", range=[-2, 10])
    fig.update_yaxes(title_text="加速度指令 [m/s²]")
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title=f"{scenario_name}<br>実機 post-gate control_cmd (acc) — 発進前後",
        height=420,
    )


def build_fig_brake_sweep(
    variants: list[float], rmses: list[float], btc_id: float, *, params: dict | None = None
) -> go.Figure:
    """brake_time_constant スイープの発進フィット RMSE と同定値。"""
    fig = make_grid(1, 1)
    fig.add_trace(go.Scatter(
        x=variants, y=rmses, mode="lines+markers", name="発進 RMSE (t=0〜5s)",
        line=dict(color="#1f77b4"), marker=dict(size=7),
    ))
    fig.add_vline(x=btc_id, line=dict(color="red", width=1.2, dash="dash"),
                  annotation_text=f"identified btc≈{btc_id:.4f}s")
    fig.update_xaxes(title_text="brake_time_constant [s]")
    fig.update_yaxes(title_text="発進フィット RMSE [m/s]")
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig, title="brake_time_constant スイープ同定 (発進フィット)", height=460,
    )
