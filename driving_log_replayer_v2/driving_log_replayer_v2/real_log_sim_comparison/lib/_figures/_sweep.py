"""step7（パラメータ sweep 同定）の図を組む build_fig_* 純関数群.

2D ペア sweep のヒートマップと、全パラメータの感度オーバービュー。step7 が抽出した
プレーンなデータ（行列・グリッド・曲線）を受けて plotly 図を組む。ROS 非依存。
個別 sweep 図（plot_sweep）は実機根拠パネルの matplotlib コールバックを伴うため別途。
"""

from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go

from ._common import apply_base_layout, make_grid


def build_fig_sweep(
    panels: list[dict], *, title: str, params: dict | None = None, ncols: int = 2
) -> go.Figure:
    """個別パラメータ sweep 図（2 列グリッドに折り返し）。旧 plot_sweep。

    各パネルは panel-spec dict で記述する（描画ロジックと計算を分離）::

        {
          "title": str, "xlabel": str, "ylabel": str,
          "traces": [go.Scatter/go.Histogram, ...],   # row/col 未バインドの trace
          "vlines": [(x, color, dash, label)],          # label は annotation_text
          "hlines": [(y, color)],
          "vrects": [(x0, x1, color)],
          "log_x": bool,
        }

    パネル順は [同定メトリクス, 副メトリクス..., 実機根拠パネル...]。横 1 列に並べると各パネルが
    狭すぎるため、`ncols` 列で折り返して各パネル幅を確保する。
    """

    n = len(panels)
    ncols = max(1, min(ncols, n))
    rows = math.ceil(n / ncols)
    titles = [p.get("title", "") for p in panels] + [""] * (rows * ncols - n)
    fig = make_grid(rows, ncols, subplot_titles=titles,
                    vertical_spacing=min(0.12, 0.5 / rows), horizontal_spacing=0.09)
    for idx, p in enumerate(panels):
        r, c = idx // ncols + 1, idx % ncols + 1
        for tr in p.get("traces", []):
            fig.add_trace(tr, row=r, col=c)
        for x, color, dash, label in p.get("vlines", []):
            # 注釈はパネル下部に小さく置く（既定の上端だとサブプロットタイトルや凡例と重なる）。
            fig.add_vline(x=x, line=dict(color=color, width=1.2, dash=dash),
                          annotation_text=label, annotation_font_size=8,
                          annotation_position="bottom right", row=r, col=c)
        for y, color in p.get("hlines", []):
            fig.add_hline(y=y, line=dict(color=color, width=0.6), row=r, col=c)
        for x0, x1, color in p.get("vrects", []):
            fig.add_vrect(x0=x0, x1=x1, fillcolor=color, opacity=0.12, line_width=0, row=r, col=c)
        fig.update_xaxes(title_text=p.get("xlabel", ""),
                         type="log" if p.get("log_x") else "linear", row=r, col=c)
        fig.update_yaxes(title_text=p.get("ylabel", ""), row=r, col=c)
    # 余ったセル（n が ncols で割り切れない場合）は軸を隠す
    for idx in range(n, rows * ncols):
        r, c = idx // ncols + 1, idx % ncols + 1
        fig.update_xaxes(visible=False, row=r, col=c)
        fig.update_yaxes(visible=False, row=r, col=c)
    return apply_base_layout(fig, title=title, height=380 * rows + 80)


def build_fig_pair_sweep(
    mat: np.ndarray,
    grid_a: list[float],
    grid_b: list[float],
    name_a: str,
    name_b: str,
    min_ij: tuple[int, int],
    *,
    metric_label: str,
    metric_unit: str,
    h_max: int,
    base_str_a: str = "—",
    base_str_b: str = "—",
) -> go.Figure:
    """2D グリッドの RMSE ヒートマップ（最小点マーカー + セル注釈）。旧 plot_pair_sweep。

    mat: shape (len(grid_b), len(grid_a))。min_ij: (i=grid_b 行, j=grid_a 列) の最小位置。
    """
    xlabels = [f"{v:g}" for v in grid_a]
    ylabels = [f"{v:g}" for v in grid_b]
    fig = go.Figure(go.Heatmap(
        z=mat, x=xlabels, y=ylabels, colorscale="YlOrRd",
        colorbar=dict(title=f"{metric_label} [{metric_unit}] @ N={h_max}"),
        hovertemplate=f"{name_a}=%{{x}}<br>{name_b}=%{{y}}<br>%{{z:.4f}}<extra></extra>",
    ))
    # セル数値注釈（背景濃淡で文字色切替）。カテゴリ軸ラベルが数値風文字列（"0","0.85"等）だと
    # annotation の x="0.85" が plotly に数値 0.85 と解釈され index に誤マップして全セルが
    # 1 箇所に collapse する。そのため**カテゴリの整数 index (j, i)** で位置指定する（数値=index）。
    finite = mat[np.isfinite(mat)]
    thresh = (finite.min() + (finite.max() - finite.min()) * 0.6) if finite.size else 0.0
    for i in range(len(ylabels)):
        for j in range(len(xlabels)):
            if np.isfinite(mat[i, j]):
                fig.add_annotation(
                    x=j, y=i, text=f"{mat[i, j]:.3f}", showarrow=False,
                    xref="x", yref="y", xanchor="center", yanchor="middle",
                    font=dict(size=10, color="white" if mat[i, j] > thresh else "black"),
                )
    mi, mj = min_ij
    fig.add_trace(go.Scatter(
        x=[xlabels[mj]], y=[ylabels[mi]], mode="markers",
        name=f"最小 ({name_a}={grid_a[mj]:g}, {name_b}={grid_b[mi]:g})",
        marker=dict(symbol="circle-open", size=18, color="blue", line=dict(width=2)),
    ))
    fig.update_xaxes(title_text=name_a, type="category")
    fig.update_yaxes(title_text=name_b, type="category")
    return apply_base_layout(
        fig,
        title=f"2D スイープ: {name_a} × {name_b} ({metric_label} @ N={h_max})"
        f"<br><sub>仕様値: {name_a}={base_str_a}, {name_b}={base_str_b}</sub>",
        height=560,
        legend=dict(x=0.99, y=0.99, xanchor="right", yanchor="top"),
    )


def build_fig_sweep_overview(
    bars: list[dict],
    curves: list[dict],
    *,
    h_max: int,
) -> go.Figure:
    """全パラメータの感度オーバービュー（左: 改善率ランキング barh / 右: 正規化 RMSE カーブ）。

    旧 plot_sweep_overview。
    bars: [{name, pct, color, text}]（改善率昇順）。
    curves: [{name, xn, yn, bx, min_x, min_y, color}]。
    """
    fig = make_grid(1, 2, subplot_titles=[
        "感度ランキング: どのパラメータを直すと効くか",
        "正規化 RMSE カーブ (平坦=非感度 / 谷=同定可能 / 単調=端)",
    ], horizontal_spacing=0.12)
    # 左: barh（横棒）
    names = [b["name"] for b in bars]
    fig.add_trace(go.Bar(
        x=[b["pct"] for b in bars], y=names, orientation="h",
        marker=dict(color=[b["color"] for b in bars], opacity=0.85),
        text=[b["text"] for b in bars], textposition="outside",
        showlegend=False, hoverinfo="x+y",
    ), row=1, col=1)
    fig.update_xaxes(title_text=f"RMSE 改善率 [%] (仕様値 → 同定値, N={h_max})", row=1, col=1)
    # 右: 正規化カーブ + 仕様値○ / 最小▼
    for cv in curves:
        color = cv["color"]
        fig.add_trace(go.Scatter(x=cv["xn"], y=cv["yn"], mode="lines", name=cv["name"],
                                 legendgroup=cv["name"], line=dict(color=color, width=1.5)), row=1, col=2)
        fig.add_trace(go.Scatter(x=[cv["bx"]], y=[1.0], mode="markers", legendgroup=cv["name"],
                                 showlegend=False, marker=dict(symbol="circle", size=8, color=color,
                                 line=dict(color="white", width=0.8))), row=1, col=2)
        fig.add_trace(go.Scatter(x=[cv["min_x"]], y=[cv["min_y"]], mode="markers", legendgroup=cv["name"],
                                 showlegend=False, marker=dict(symbol="triangle-down", size=10, color=color,
                                 line=dict(color="white", width=0.8))), row=1, col=2)
    fig.add_hline(y=1.0, line=dict(color="black", width=0.8, dash="dot"), row=1, col=2)
    fig.update_xaxes(title_text="グリッド位置 (パラメータごとに [0,1] 正規化)", row=1, col=2)
    fig.update_yaxes(title_text="RMSE / RMSE@仕様値", row=1, col=2)
    return apply_base_layout(
        fig,
        title=f"パラメータ sweep 感度オーバービュー (N={h_max} 終端誤差, ○=仕様値, ▼=最小)",
        height=max(460, 55 * len(bars) + 180),
    )
