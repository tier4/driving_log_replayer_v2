"""step4（実機 vs sim クローズループ比較）の図を組む build_fig_* 純関数群.

step4 は ROS から DataFrame を読んでプレーンな配列/run dict に整形し、
ここの純関数で `go.Figure` を組んで `write_fig_json` で吐く。本モジュールは ROS 非依存。

run dict のスキーマ（走行距離基準図）::

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


