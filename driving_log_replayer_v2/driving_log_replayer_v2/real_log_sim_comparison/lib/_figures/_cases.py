"""step6（ケース集約 overlay）の図を組む build_fig_* 純関数群.

全ケースの N-step 誤差を 1 枚に重ね描きする overlay 図。入力は step6 が用意した
ケース別 DataFrame（{tag: df}）や rmse_by_horizon 集約（{tag: {horizon: {...}}}）。
ROS 非依存。rmse_heatmap（imshow + セル注釈）は Tier3 で別途。
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import plotly.graph_objects as go

from .._nstep_common import ERR_METRICS, YAW_SEED_NOTE
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


def build_fig_error_timeseries_overlay(
    case_dfs: dict[str, pd.DataFrame], horizon: int
) -> go.Figure:
    """指定 horizon の終端誤差時系列を全 case 重ね描き。旧 plot_error_timeseries_overlay。"""
    tags = list(case_dfs.keys())
    colors = dict(zip(tags, qualitative_colors(len(tags))))
    titles = []
    for col, _s, label, _u, _src in ERR_METRICS:
        t = f"{label} (N={horizon}, 実機 − モデル)"
        if col == "yaw_err_deg":
            t += f"<br><sub>{YAW_SEED_NOTE}</sub>"
        titles.append(t)
    fig = make_grid(len(ERR_METRICS), 1, subplot_titles=titles, shared_xaxes=True,
                    vertical_spacing=0.08)
    for row, (col, scale, label, unit, _src) in enumerate(ERR_METRICS, start=1):
        for tag, df in case_dfs.items():
            sub = df[df["horizon"] == horizon].sort_values("tr")
            err = sub[col].to_numpy(dtype=float) * scale
            rmse = float(np.sqrt(np.nanmean(err**2)))
            fig.add_trace(go.Scatter(
                x=sub["tr"].to_numpy(), y=err, mode="lines",
                name=f"{tag}  RMSE={rmse:.3g} {unit}", legendgroup=tag, showlegend=row == 1,
                opacity=0.85, line=dict(color=colors[tag], width=1.0),
            ), row=row, col=1)
        fig.add_hline(y=0, line=dict(color="black", width=0.6), row=row, col=1)
        fig.update_yaxes(title_text=f"誤差 [{unit}]", row=row, col=1)
    fig.update_xaxes(title_text="rollout 開始時刻 (AUTONOMOUS 開始からの経過) [s]",
                     row=len(ERR_METRICS), col=1)
    return apply_base_layout(
        fig, title=f"cases overlay: N-step 誤差時系列 (N={horizon})", height=300 * len(ERR_METRICS) + 60,
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


def build_fig_rmse_heatmap(
    roll: dict[str, dict[int, dict[str, float]]], horizons: list[int]
) -> go.Figure | None:
    """case × horizon の RMSE ヒートマップ（位置/yaw 2 パネル、セル数値注釈付き）。旧 plot_rmse_heatmap。"""
    tags = list(roll.keys())
    if not horizons or not tags:
        return None
    fig = make_grid(1, 2, subplot_titles=["位置 RMSE [cm]", "yaw RMSE [deg]"], horizontal_spacing=0.13)
    xlabels = [f"N={h}" for h in horizons]
    for c, (key, unit) in enumerate([("pos", "cm"), ("yaw", "deg")], start=1):
        mat = np.array([[roll[tag][h][key] for h in horizons] for tag in tags], dtype=float)
        fig.add_trace(go.Heatmap(
            z=mat, x=xlabels, y=tags, colorscale="YlOrRd", showscale=True,
            colorbar=dict(title=unit, x=(0.46 if c == 1 else 1.02), len=0.9),
            hovertemplate="%{y} / %{x}<br>%{z:.3f} " + unit + "<extra></extra>",
        ), row=1, col=c)
        # セル注釈（背景の濃淡で文字色を切替）。make_subplots の row/col 指定で軸参照が
        # 正しく解決される（plain go.Figure だと xref/yref が paper になり位置が崩れる）。
        thresh = mat.min() + (mat.max() - mat.min()) * 0.6
        for i, tag in enumerate(tags):
            for j, xl in enumerate(xlabels):
                fig.add_annotation(
                    x=xl, y=tag, text=f"{mat[i, j]:.2f}", showarrow=False,
                    font=dict(size=10, color="white" if mat[i, j] > thresh else "black"),
                    row=1, col=c,
                )
    return apply_base_layout(
        fig, title="cases × horizon: N-step RMSE ヒートマップ",
        height=200 + 60 * len(tags),
        # y 軸の case ラベル（best_normal 等）が既定の左マージン (60) で切れるため広げる。
        margin=dict(l=90, r=20, t=100, b=50),
    )


def build_fig_growth_relative(
    roll: dict[str, dict[int, dict[str, float]]], ref_tag: str, horizons: list[int]
) -> go.Figure | None:
    """reference 比の誤差成長（RMSE(case)/RMSE(ref)）。旧 plot_growth_relative。"""
    ref = roll.get(ref_tag)
    if not ref or not horizons:
        return None
    tags = list(roll.keys())
    colors = dict(zip(tags, qualitative_colors(len(tags))))
    fig = make_grid(1, 2, subplot_titles=["位置 RMSE 比", "yaw 誤差 比"], horizontal_spacing=0.10)
    for c, key in [(1, "pos"), (2, "yaw")]:
        for tag, r in roll.items():
            if tag == ref_tag:
                continue
            ratio = [r[h][key] / max(ref[h][key], 1e-12) for h in horizons]
            fig.add_trace(go.Scatter(
                x=horizons, y=ratio, mode="lines+markers", name=tag, legendgroup=tag,
                showlegend=c == 1, line=dict(color=colors[tag], width=1.4), marker=dict(size=6),
            ), row=1, col=c)
        fig.add_hline(y=1.0, line=dict(color="black", width=1.0, dash="dash"), row=1, col=c)
        fig.update_xaxes(title_text="rollout 長 N [step]", row=1, col=c)
        fig.update_yaxes(title_text=f"RMSE(case) / RMSE({ref_tag})", row=1, col=c)
    return apply_base_layout(
        fig,
        title=f"cases overlay: N-step 相対誤差成長 (対 {ref_tag} 比)"
        "<br><sub>1.0 より上=基準より悪化 / 下=改善</sub>",
        height=480,
    )
