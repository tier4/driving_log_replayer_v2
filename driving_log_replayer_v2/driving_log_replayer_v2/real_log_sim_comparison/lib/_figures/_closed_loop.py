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
    add_params_annotation_plotly,
    apply_base_layout,
    lanes_to_trace,
    make_grid,
    map_ways_in_bbox,
    plotly_dash,
    plotly_marker,
)


def _bottom_note(fig: go.Figure, note: str | None) -> None:
    """図下部中央に注記（matplotlib の fig.text 相当）を足す。"""
    if not note:
        return
    fig.add_annotation(
        xref="paper", yref="paper", x=0.5, y=-0.08,
        xanchor="center", yanchor="top", showarrow=False,
        text=note, font=dict(size=10, color="#555555"),
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
    _bottom_note(fig, note)
    add_params_annotation_plotly(fig)
    return apply_base_layout(fig, title=title, height=height)


def _bbox_mask(x, y, cx, cy, mg):
    return (x >= cx - mg) & (x <= cx + mg) & (y >= cy - mg) & (y <= cy + mg)


def build_fig_curves_closeup(
    curves: list[dict],
    runs: list[dict],
    map_ways: list[np.ndarray] | None = None,
    *,
    scenario_name: str = "",
) -> go.Figure:
    """カーブ別の軌跡比較（横 N 列、各列が 1 カーブの bbox 拡大）。旧 plot_curves。

    curves: [{cx, cy, margin, label}]。runs: [{label, color, lw, ls, marker, ms, x, y}]（全軌跡）。
    各列でレーン背景（bbox 内）+ bbox 内に入る各 run の軌跡を等倍表示する。凡例は先頭列のみ。
    """
    n = len(curves)
    fig = make_grid(1, n, subplot_titles=[c.get("label", f"カーブ{i+1}") for i, c in enumerate(curves)],
                    horizontal_spacing=0.06)
    for col, c in enumerate(curves, start=1):
        cx, cy, mg = c["cx"], c["cy"], c["margin"]
        if map_ways:
            ways = map_ways_in_bbox(map_ways, (cx - mg, cx + mg), (cy - mg, cy + mg))
            if ways:
                fig.add_trace(lanes_to_trace(ways), row=1, col=col)
        for r in runs:
            x = np.asarray(r["x"]); y = np.asarray(r["y"])
            m = _bbox_mask(x, y, cx, cy, mg)
            if not m.any():
                continue
            xs, ys = x[m], y[m]
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="lines", name=r["label"], legendgroup=r["label"],
                showlegend=col == 1,
                line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-"))),
            ), row=1, col=col)
            me = max(1, len(xs) // 12)
            fig.add_trace(go.Scatter(
                x=xs[::me], y=ys[::me], mode="markers", legendgroup=r["label"], showlegend=False,
                marker=dict(symbol=plotly_marker(r.get("marker", "o")), size=r.get("ms", 4) + 2,
                            color=r["color"], line=dict(color="white", width=0.5)),
                hoverinfo="skip",
            ), row=1, col=col)
        fig.update_xaxes(title_text="x [m]", range=[cx - mg, cx + mg], row=1, col=col)
        ax_suffix = "" if col == 1 else str(col)
        fig.update_yaxes(title_text="y [m]" if col == 1 else None, range=[cy - mg, cy + mg],
                         scaleanchor=f"x{ax_suffix}", scaleratio=1, row=1, col=col)
    return apply_base_layout(
        fig, title=f"{scenario_name}<br>カーブ別軌跡比較" if scenario_name else "カーブ別軌跡比較",
        height=620,
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
    add_params_annotation_plotly(fig)
    return apply_base_layout(fig, title=title, height=height)


def build_fig_trajectory(
    runs: list[dict],
    *,
    map_ways: list[np.ndarray] | None = None,
    scenario_name: str = "",
    height: int = 800,
    margin_m: float = 30.0,
) -> go.Figure:
    """地図背景あり軌跡比較図（旧 plot_trajectory の plotly 純関数版）。

    表示範囲は全 run の bbox + マージンに限定。ズーム/パン/hover/凡例トグル可。
    `map_ways` があれば bbox 内のレーンを薄灰で背景描画する。
    """
    all_xy = [
        np.column_stack([r["x"], r["y"]])
        for r in runs
        if len(r.get("x", [])) > 0
    ]
    fig = go.Figure()
    if not all_xy:
        return apply_base_layout(fig, title=f"{scenario_name}<br>軌跡比較（データなし）", height=height)
    pts = np.concatenate(all_xy)
    x_min, y_min = pts.min(axis=0) - margin_m
    x_max, y_max = pts.max(axis=0) + margin_m

    if map_ways:
        ways = map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max))
        if ways:
            fig.add_trace(lanes_to_trace(ways))

    for r in runs:
        x = np.asarray(r["x"])
        y = np.asarray(r["y"])
        if len(x) == 0:
            continue
        label = r["label"]
        t = r.get("t")
        customdata = np.asarray(t) if t is not None else None
        hover_extra = "<br>t=%{customdata:.1f}s" if t is not None else ""
        fig.add_trace(go.Scatter(
            x=x, y=y, mode="lines", name=label, legendgroup=label,
            line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-"))),
            customdata=customdata,
            hovertemplate=f"{label}<br>x=%{{x:.1f}}m y=%{{y:.1f}}m{hover_extra}<extra></extra>",
        ))
        me = max(1, len(x) // 15)
        fig.add_trace(go.Scatter(
            x=x[::me], y=y[::me], mode="markers", legendgroup=label, showlegend=False,
            marker=dict(
                symbol=plotly_marker(r.get("marker", "o")),
                size=r.get("ms", 4) + 3, color=r["color"],
                line=dict(color="white", width=0.5),
            ),
            hoverinfo="skip",
        ))

    prov_lines = [f"{r['label']}: {r['prov']}" for r in runs if r.get("prov")]
    if prov_lines:
        fig.add_annotation(
            xref="paper", yref="paper", x=0.0, y=0.0,
            xanchor="left", yanchor="bottom", align="left", showarrow=False,
            text="provenance —<br>" + "<br>".join(prov_lines),
            font=dict(family="monospace", size=9, color="#555555"),
            bgcolor="rgba(255,255,255,0.7)",
        )
    add_params_annotation_plotly(fig)

    apply_base_layout(
        fig,
        title=f"{scenario_name}<br>軌跡比較" if scenario_name else "軌跡比較",
        height=height,
        xaxis=dict(title="x [m]", range=[x_min, x_max]),
        yaxis=dict(title="y [m]", range=[y_min, y_max], scaleanchor="x", scaleratio=1),
        legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.7)"),
    )
    return fig
