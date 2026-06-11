"""step4（実機 vs sim クローズループ比較）の図を組む build_fig_* 純関数群.

matplotlib 版（step4 の plot_*）と 1:1 対応。step4 は ROS から DataFrame を読んで
プレーンな配列/run dict に整形し、ここの純関数で `go.Figure` を組んで
`write_fig_json` で吐く。本モジュールは ROS 非依存。

run dict のスキーマ（時系列・軌跡で共通）::

    {
      "label": str, "color": str,
      "lw": float, "ls": str, "marker": str, "ms": float,  # matplotlib 由来スタイル
      "x": np.ndarray, "y": np.ndarray,                     # 軌跡用
      "t": np.ndarray | None,                               # hover 用時刻（任意）
      "prov": str | None,                                   # provenance 1 行（任意）
    }
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ._common import (
    add_bottom_note as _bottom_note,
    apply_base_layout,
    make_grid,
    plotly_dash,
)


def build_fig_timeseries_resp_cmd(
    runs: list[dict],
    *,
    title: str,
    resp_title: str,
    cmd_title: str,
    resp_ylabel: str,
    cmd_ylabel: str,
    xlabel: str = "発進からの経過時間 [s]",
    note: str | None = None,
    height: int = 720,
) -> go.Figure:
    """応答（上段）/ 指令（下段・点線薄色）の 2 段時系列重ね描き。

    旧 plot_velocity / plot_acceleration / plot_steering の共通 plotly 版。
    run dict: {label, color, lw, ls, t_resp, y_resp, t_cmd?, y_cmd?}。
    """
    fig = make_grid(2, 1, subplot_titles=[resp_title, cmd_title], shared_xaxes=False)
    for r in runs:
        label = r["label"]
        fig.add_trace(go.Scatter(
            x=r["t_resp"], y=r["y_resp"], name=label, legendgroup=label, mode="lines",
            line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-"))),
        ), row=1, col=1)
        if r.get("t_cmd") is not None and r.get("y_cmd") is not None:
            fig.add_trace(go.Scatter(
                x=r["t_cmd"], y=r["y_cmd"], name=label, legendgroup=label, mode="lines",
                showlegend=False, opacity=0.65,
                line=dict(color=r["color"], width=1.2, dash="dot"),
            ), row=2, col=1)
    fig.update_yaxes(title_text=resp_ylabel, row=1, col=1)
    fig.update_yaxes(title_text=cmd_ylabel, row=2, col=1)
    fig.update_xaxes(title_text=xlabel, row=2, col=1)
    _bottom_note(fig, note, height=height)
    return apply_base_layout(fig, title=title, height=height)


def build_fig_vs_distance(
    runs: list[dict],
    *,
    title: str,
    subplot_title: str,
    ylabel: str,
    xlabel: str = "走行距離 [m]",
    height: int = 460,
) -> go.Figure:
    """走行距離（arc-length）基準の 1 段重ね描き（旧 *_vs_distance）。

    run dict: {label, color, lw, ls, s, y}。
    """
    fig = make_grid(1, 1, subplot_titles=[subplot_title])
    for r in runs:
        fig.add_trace(go.Scatter(
            x=r["s"], y=r["y"], name=r["label"], legendgroup=r["label"], mode="lines",
            line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-"))),
        ))
    fig.update_xaxes(title_text=xlabel)
    fig.update_yaxes(title_text=ylabel)
    return apply_base_layout(fig, title=title, height=height)


