"""step6（ケース集約 overlay）の図を組む build_fig_* 純関数群.

全ケースの N-step 誤差を 1 枚に重ね描きする overlay 図。入力は step6 が用意した
ケース別 DataFrame（{tag: df}）や rmse_by_horizon 集約（{tag: {horizon: {...}}}）。
ROS 非依存。誤差時系列の対話的確認は縦横モデル検証ビューア（_model_viewer）へ移設済み。
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import plotly.graph_objects as go

from ._common import apply_base_layout, make_grid, qualitative_colors

_VLINE_GREEN = dict(line=dict(color="green", width=1.0, dash="dot"))

# (real_col, sim_col, err_col, scale, ylabel, title)。step6 cascade overlay の段定義。
_CASCADE_ROWS = [
    ("real_steer_kend", "sim_steer_kend", "err_steer", 180.0 / np.pi,
     "ステア角 [deg]", "① ステア応答 (cmd→actual): err_steer"),
    ("real_ds_lat", "sim_ds_lat", "err_ds_lat", 100.0,
     "横方向 Δpos [cm]", "② 横方向 1ステップ変位: err_ds_lat"),
]


def build_fig_cascade_error_overlay(case_dfs_n1: dict[str, pd.DataFrame]) -> go.Figure:
    """全 case の N=1 段階的誤差を重ね描き（左:時系列 / 右:誤差、3:1）。旧 plot_cascade_error_overlay。"""
    tags = list(case_dfs_n1.keys())
    colors = dict(zip(tags, qualitative_colors(len(tags))))
    titles = []
    for _rc, _sc, _ec, _s, _yl, title in _CASCADE_ROWS:
        titles += [title, "誤差 (real − sim) — case 別 RMSE"]
    fig = make_grid(len(_CASCADE_ROWS), 2, subplot_titles=titles, column_widths=[0.75, 0.25],
                    vertical_spacing=0.12, horizontal_spacing=0.08)
    for row, (real_col, sim_col, err_col, scale, ylabel, _t) in enumerate(_CASCADE_ROWS, start=1):
        for tag, df in case_dfs_n1.items():
            tr = df["tr"].to_numpy()
            fig.add_trace(go.Scatter(
                x=tr, y=df[sim_col].to_numpy(dtype=float) * scale, mode="lines",
                name=f"{tag} sim", legendgroup=tag, showlegend=row == 1,
                line=dict(color=colors[tag], width=1.2),
            ), row=row, col=1)
            err = df[err_col].to_numpy(dtype=float) * scale
            rmse = float(np.sqrt(np.nanmean(err**2)))
            fig.add_trace(go.Scatter(
                x=tr, y=err, mode="lines", name=f"{tag} RMSE={rmse:.3g}",
                legendgroup=tag, showlegend=False, opacity=0.8,
                line=dict(color=colors[tag], width=1.0),
            ), row=row, col=2)
        # real は最初の case のものを黒破線基準で（全 case 同じ実機ログ）
        first = next(iter(case_dfs_n1.values()))
        fig.add_trace(go.Scatter(
            x=first["tr"].to_numpy(), y=first[real_col].to_numpy(dtype=float) * scale,
            mode="lines", name="real", legendgroup="real", showlegend=row == 1,
            line=dict(color="black", width=1.5, dash="dash"),
        ), row=row, col=1)
        fig.add_hline(y=0, line=dict(color="black", width=0.5, dash="dot"), row=row, col=1)
        fig.add_hline(y=0, line=dict(color="black", width=0.6), row=row, col=2)
        fig.update_yaxes(title_text=ylabel, row=row, col=1)
        fig.update_yaxes(title_text=f"誤差 [{ylabel.split('[')[1]}", row=row, col=2)
    for col in (1, 2):
        fig.update_xaxes(title_text="AUTONOMOUS 開始からの時刻 [s]", row=len(_CASCADE_ROWS), col=col)
    return apply_base_layout(
        fig, title="cases overlay: N=1 段階的誤差 (ステア応答 → 横位置)", height=380 * len(_CASCADE_ROWS),
    )


def build_fig_error_growth_overlay(roll: dict[str, dict[int, dict[str, float]]]) -> go.Figure:
    """全 case の N-step 誤差成長（位置/yaw 2 パネル）。旧 plot_error_growth_overlay。"""
    tags = list(roll.keys())
    colors = dict(zip(tags, qualitative_colors(len(tags))))
    fig = make_grid(1, 2, subplot_titles=["位置誤差成長", "yaw 誤差成長"], horizontal_spacing=0.10)
    for c, (key, ylabel, dash, sym) in enumerate(
        [("pos", "位置 RMSE [cm]", "solid", "circle"), ("yaw", "yaw RMSE [deg]", "dash", "square")], start=1
    ):
        for tag, r in roll.items():
            horizons = sorted(r.keys())
            fig.add_trace(go.Scatter(
                x=horizons, y=[r[h][key] for h in horizons], mode="lines+markers",
                name=tag, legendgroup=tag, showlegend=c == 1,
                line=dict(color=colors[tag], width=1.4, dash=dash), marker=dict(symbol=sym, size=6),
            ), row=1, col=c)
        fig.update_xaxes(title_text="rollout 長 N [step]", row=1, col=c)
        fig.update_yaxes(title_text=ylabel, row=1, col=c)
    return apply_base_layout(fig, title="cases overlay: N-step 誤差成長 (free-running)", height=460)
