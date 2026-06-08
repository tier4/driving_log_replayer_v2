"""step10（カーブ乖離詳細診断）の図を組む build_fig_* 純関数群.

step10 が `decompose_deviation` 等で算出した時系列配列（実機 yaw 基準の縦/横分解、
速度・ステア・yaw 差）を受けて 5 段スタック時系列を組む。ROS 非依存。
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ._common import apply_base_layout, make_grid

# t=0（発進）を示す灰色破線の共通体裁。
_VLINE = dict(line=dict(color="gray", width=0.8, dash="dash"))
_REAL = "#000000"
_SIM = "#e05c00"


def build_fig_curve_divergence(
    d: dict, *, scenario_name: str = "", sim_name: str = "シム", params: dict | None = None
) -> go.Figure:
    """カーブ乖離の 5 段診断（速度/速度差/ステア/yaw差/軌跡乖離の縦横分解）。

    旧 plot_detailed の plotly 版。共有 x 軸（発進相対時刻 t [s]）。
    d: {t, rv, sv, rc, sc, rs, ss, yaw_diff, dists, lons, lats}（いずれも numpy 配列）。
    """
    t = d["t"]
    fig = make_grid(
        5, 1, shared_xaxes=True, vertical_spacing=0.045,
        subplot_titles=[
            "速度（actual/cmd）", f"速度差（{sim_name} − 実機）", "ステアリング角応答",
            f"ヨー角差（{sim_name} − 実機）", "軌跡乖離の縦横分解（実機進行方向基準）",
        ],
    )

    def line(y, color, dash="solid", width=2.0, name=None, row=1, opacity=1.0, fill=None):
        fig.add_trace(go.Scatter(
            x=t, y=y, mode="lines", name=name, showlegend=name is not None,
            line=dict(color=color, width=width, dash=dash), opacity=opacity,
            fill=fill, fillcolor="rgba(255,0,0,0.15)" if fill else None,
        ), row=row, col=1)

    # 1: 速度（rv→sv の順で fill='tonexty' により速度差を薄赤塗り）
    line(d["rv"], _REAL, name="実機 actual", row=1)
    line(d["sv"], _SIM, dash="dash", name=f"{sim_name} actual", row=1, fill="tonexty")
    line(d["rc"], _REAL, dash="dot", width=1.2, name="実機 cmd", row=1, opacity=0.6)
    line(d["sc"], _SIM, dash="dot", width=1.2, name=f"{sim_name} cmd", row=1, opacity=0.6)
    # 2: 速度差
    line(d["sv"] - d["rv"], _SIM, dash="dash", name=f"actual差 ({sim_name}−実機)", row=2)
    line(d["sc"] - d["rc"], _SIM, dash="dot", width=1.5, name=f"cmd差 ({sim_name}−実機)", row=2, opacity=0.7)
    fig.add_hline(y=0, line=dict(color="gray", width=0.5), row=2, col=1)
    # 3: ステア応答
    line(d["rs"], _REAL, name="実機 actual", row=3)
    line(d["ss"], _SIM, dash="dash", name=f"{sim_name} actual", row=3)
    # 4: yaw 差
    line(d["yaw_diff"], "purple", name=f"yaw差 ({sim_name}−実機)", row=4)
    fig.add_hline(y=0, line=dict(color="gray", width=0.5), row=4, col=1)
    # 5: 軌跡乖離の縦横分解
    line(d["dists"], "red", name="総乖離距離 [m]", row=5)
    line(d["lons"], "blue", dash="dash", width=1.5, name="縦方向 (実機前方正) [m]", row=5)
    line(d["lats"], "green", dash="dashdot", width=1.5, name="横方向 (左正) [m]", row=5)
    fig.add_hline(y=0, line=dict(color="gray", width=0.5), row=5, col=1)

    for row, unit in [(1, "速度 [m/s]"), (2, "m/s"), (3, "deg"), (4, "deg"), (5, "m")]:
        fig.update_yaxes(title_text=unit, row=row, col=1)
        fig.add_vline(x=0, row=row, col=1, **_VLINE)
    fig.update_xaxes(title_text="発進からの時刻 [s]", row=5, col=1)

    return apply_base_layout(
        fig,
        title=f"{scenario_name}<br>カーブ② 乖離詳細診断" if scenario_name else "カーブ② 乖離詳細診断",
        height=1500,
    )
