"""step8（DiffusionPlanner 出力軌跡比較）の図を組む build_fig_* 純関数群.

step8 が frames_to_series 等で算出した時系列配列（DP 計画速度・実速度・速度差）を受けて
plotly 図を組む。ROS 非依存。dp_real_vs_sim（時刻で色付けした速度プロファイル + カラーバー）は
Tier3 で別途。
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ._common import add_params_annotation_plotly, apply_base_layout, make_grid, viridis_at

_VLINE = dict(line=dict(color="gray", width=0.8, dash="dash"))


def build_fig_dp_vs_actual(
    t_vec: np.ndarray,
    panels: list[dict],
    *,
    scenario_name: str = "",
    params: dict | None = None,
) -> go.Figure:
    """DP 計画速度(d=0) vs 実際速度の 1×len(panels) 比較。旧 dp_vs_actual。

    panels: [{label, dp_v, actual_t, actual_v}]（sim / 実機 の 2 枚を想定）。
    """
    fig = make_grid(1, len(panels), subplot_titles=[f"{p['label']}: DP計画 vs actual" for p in panels],
                    horizontal_spacing=0.08)
    for c, p in enumerate(panels, start=1):
        fig.add_trace(go.Scatter(x=t_vec, y=p["dp_v"], mode="lines", name="DP計画速度 (d=0m)",
                                 legendgroup="dp", showlegend=c == 1,
                                 line=dict(color="blue", width=2)), row=1, col=c)
        fig.add_trace(go.Scatter(x=p["actual_t"], y=p["actual_v"], mode="lines", name="actual速度",
                                 legendgroup="actual", showlegend=c == 1, opacity=0.8,
                                 line=dict(color="red", width=1.5)), row=1, col=c)
        fig.add_vline(x=0, row=1, col=c, **_VLINE)
        fig.update_xaxes(title_text="発進後 t [s]", row=1, col=c)
        fig.update_yaxes(title_text="速度 [m/s]", row=1, col=c)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig, title=f"{scenario_name}<br>DPが計画した速度(d=0) vs 実際の速度", height=460,
    )


_D_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
_TREL_MIN, _TREL_MAX = -1.0, 10.0


def build_fig_dp_real_vs_sim(
    t_vec: np.ndarray,
    plan_series: list[dict],
    profiles: dict,
    objects: dict,
    *,
    sim_name: str = "シム",
    scenario_name: str = "",
    params: dict | None = None,
) -> go.Figure:
    """DP計画軌跡 直接比較（実機 vs sim）。旧 dp_real_vs_sim。

    plan_series: [{d, color, v_sim, v_real}]（上段 3 列の計画速度・差用）。
    profiles: {"sim": [{t_rel, dists, vels}], "real": [...]}（下段の速度プロファイル、t_rel で viridis 着色）。
    objects: {real_t, real_n, sim_t, sim_n}（追跡物体数。sim_* は None 可）。
    """
    titles = [
        f"{sim_name} DP計画速度 [m/s]", "実機 DP計画速度 [m/s]", f"速度差 ({sim_name} − 実機) [m/s]",
        f"{sim_name} DP速度プロファイル（-1s〜+10s）", "実機 DP速度プロファイル（-1s〜+10s）",
        "追跡物体数（DiffusionPlanner への社会的コンテキスト）",
    ]
    fig = make_grid(2, 3, subplot_titles=titles, vertical_spacing=0.13, horizontal_spacing=0.07)
    # 上段: 計画速度 sim / real / 差
    for s in plan_series:
        fig.add_trace(go.Scatter(x=t_vec, y=s["v_sim"], mode="lines", name=f"d={s['d']}m",
                                 legendgroup=f"d{s['d']}", showlegend=True,
                                 line=dict(color=s["color"], width=1.5)), row=1, col=1)
        fig.add_trace(go.Scatter(x=t_vec, y=s["v_real"], mode="lines", name=f"d={s['d']}m",
                                 legendgroup=f"d{s['d']}", showlegend=False,
                                 line=dict(color=s["color"], width=1.5)), row=1, col=2)
        fig.add_trace(go.Scatter(x=t_vec, y=s["v_sim"] - s["v_real"], mode="lines", name=f"Δ d={s['d']}m",
                                 legendgroup=f"d{s['d']}", showlegend=False,
                                 line=dict(color=s["color"], width=1.5, dash="dash")), row=1, col=3)
    for c in (1, 2, 3):
        fig.add_vline(x=0, row=1, col=c, **_VLINE)
        fig.add_hline(y=0, line=dict(color="gray", width=0.5), row=1, col=c)
        fig.update_xaxes(title_text="発進後 t [s]", row=1, col=c)
    # 下段左/中: 速度プロファイル（フレームごとに t_rel で viridis 着色）
    for col, key in [(1, "sim"), (2, "real")]:
        frames = profiles.get(key, [])
        fracs = [(f["t_rel"] - _TREL_MIN) / (_TREL_MAX - _TREL_MIN) for f in frames]
        colors = viridis_at(fracs)
        for fr, color in zip(frames, colors):
            fig.add_trace(go.Scatter(x=fr["dists"], y=fr["vels"], mode="lines", showlegend=False,
                                     opacity=0.6, line=dict(color=color, width=1.2),
                                     hoverinfo="skip"), row=2, col=col)
        # launch (t≈0) を赤太線で強調
        launch = next((f for f in frames if abs(f["t_rel"]) < 0.15), None)
        if launch is not None:
            fig.add_trace(go.Scatter(x=launch["dists"], y=launch["vels"], mode="lines",
                                     name="t≈0 (launch)", legendgroup="launch", showlegend=col == 1,
                                     line=dict(color="red", width=3)), row=2, col=col)
        fig.update_xaxes(title_text="経路点距離 [m]", row=2, col=col)
        fig.update_yaxes(title_text="計画速度 [m/s]", row=2, col=col)
    # 共有カラーバー（t_rel）。不可視マーカー trace で colorbar だけ出す。
    fig.add_trace(go.Scatter(
        x=[None], y=[None], mode="markers", showlegend=False, hoverinfo="skip",
        marker=dict(colorscale="Viridis", cmin=_TREL_MIN, cmax=_TREL_MAX, color=[_TREL_MIN],
                    colorbar=dict(title="t [s]", x=1.02, len=0.4, y=0.2, thickness=12)),
    ), row=2, col=2)
    # 下段右: 追跡物体数
    if objects.get("real_t") is not None:
        fig.add_trace(go.Scatter(x=objects["real_t"], y=objects["real_n"], mode="lines",
                                 name="実機 追跡物体数", opacity=0.8,
                                 line=dict(color="black", width=1.5)), row=2, col=3)
    if objects.get("sim_t") is not None:
        fig.add_trace(go.Scatter(x=objects["sim_t"], y=objects["sim_n"], mode="lines",
                                 name=f"{sim_name} 追跡物体数", opacity=0.8,
                                 line=dict(color="#e05c00", width=1.5, dash="dash")), row=2, col=3)
    else:
        fig.add_hline(y=0, line=dict(color="#e05c00", width=1.5, dash="dash"), row=2, col=3)
    fig.add_vline(x=0, row=2, col=3, **_VLINE)
    fig.update_xaxes(title_text="発進後 t [s]", row=2, col=3)
    fig.update_yaxes(title_text="追跡物体数", row=2, col=3)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title=f"{scenario_name}<br>DiffusionPlanner計画軌跡 直接比較（実機 vs {sim_name}）",
        height=860,
    )


def build_fig_dp_vs_final_traj(
    t_vec: np.ndarray,
    top_panels: list[dict],
    diff_series: list[dict],
    summary: dict,
    *,
    sim_name: str = "シム",
    scenario_name: str = "",
    params: dict | None = None,
) -> go.Figure:
    """DP出力 vs 最終planning/trajectory（上段: 各距離の DP/最終/sim + 補正塗り、下段: 補正量/差/サマリ）。

    旧 dp_vs_final_traj。top_panels: [{d, v_dp, v_final, v_sim}]（先頭 3 距離）。
    diff_series: [{d, v_final, v_dp, v_sim}]（全距離）。summary: actual/DP/final 各系列。
    """
    titles = [f"d={p['d']}m 地点の計画速度" for p in top_panels[:3]]
    titles += [
        "optimizer補正量 (最終traj − DP出力) [実機]",
        f"{sim_name} − 実機 DP計画速度差",
        "DP出力 d=0 / 最終traj / actual速度 比較",
    ]
    fig = make_grid(2, 3, subplot_titles=titles, vertical_spacing=0.12, horizontal_spacing=0.07)
    # 上段: 各距離の DP / 最終 / sim + optimizer 補正の塗り
    for c, p in enumerate(top_panels[:3], start=1):
        fig.add_trace(go.Scatter(x=t_vec, y=p["v_dp"], mode="lines", name="実機 DP出力",
                                 legendgroup="dp", showlegend=c == 1,
                                 line=dict(color="black", width=2)), row=1, col=c)
        fig.add_trace(go.Scatter(x=t_vec, y=p["v_final"], mode="lines", name="実機 最終traj",
                                 legendgroup="final", showlegend=c == 1, fill="tonexty",
                                 fillcolor="rgba(0,0,255,0.2)",
                                 line=dict(color="blue", width=2, dash="dash")), row=1, col=c)
        fig.add_trace(go.Scatter(x=t_vec, y=p["v_sim"], mode="lines", name=f"{sim_name} DP出力",
                                 legendgroup="simdp", showlegend=c == 1,
                                 line=dict(color="red", width=1.5, dash="dot")), row=1, col=c)
        fig.add_vline(x=0, row=1, col=c, **_VLINE)
        fig.update_xaxes(title_text="発進後 t [s]", row=1, col=c)
        fig.update_yaxes(title_text="計画速度 [m/s]", row=1, col=c)
    # 下段左/中: 補正量・差（距離別）
    for i, s in enumerate(diff_series):
        color = _D_COLORS[i % len(_D_COLORS)]
        fig.add_trace(go.Scatter(x=t_vec, y=s["v_final"] - s["v_dp"], mode="lines",
                                 name=f"d={s['d']}m", legendgroup=f"d{s['d']}", showlegend=True,
                                 line=dict(color=color, width=1.5)), row=2, col=1)
        fig.add_trace(go.Scatter(x=t_vec, y=s["v_sim"] - s["v_dp"], mode="lines",
                                 name=f"d={s['d']}m", legendgroup=f"d{s['d']}", showlegend=False,
                                 line=dict(color=color, width=1.5)), row=2, col=2)
    for col in (1, 2):
        fig.add_hline(y=0, line=dict(color="gray", width=0.5), row=2, col=col)
        fig.add_vline(x=0, row=2, col=col, **_VLINE)
        fig.update_xaxes(title_text="発進後 t [s]", row=2, col=col)
        fig.update_yaxes(title_text="Δv [m/s]", row=2, col=col)
    # 下段右: サマリ
    fig.add_trace(go.Scatter(x=summary["real_t"], y=summary["real_v"], mode="lines",
                             name="実機 actual速度", line=dict(color="black", width=2)), row=2, col=3)
    if summary.get("sim_t") is not None:
        fig.add_trace(go.Scatter(x=summary["sim_t"], y=summary["sim_v"], mode="lines",
                                 name=f"{sim_name} actual速度",
                                 line=dict(color="red", width=2, dash="dash")), row=2, col=3)
        if summary.get("dp0_sim") is not None:
            fig.add_trace(go.Scatter(x=t_vec, y=summary["dp0_sim"], mode="lines",
                                     name=f"{sim_name} DP d=0", opacity=0.7,
                                     line=dict(color="red", width=1.5, dash="dot")), row=2, col=3)
    fig.add_trace(go.Scatter(x=t_vec, y=summary["dp0_real"], mode="lines", name="実機 DP d=0",
                             line=dict(color="black", width=1.5, dash="dot")), row=2, col=3)
    fig.add_trace(go.Scatter(x=t_vec, y=summary["fin0_real"], mode="lines", name="実機 最終traj d=0",
                             line=dict(color="blue", width=1.5, dash="dash")), row=2, col=3)
    fig.add_vline(x=0, row=2, col=3, **_VLINE)
    fig.update_xaxes(title_text="発進後 t [s]", row=2, col=3)
    fig.update_yaxes(title_text="速度 [m/s]", row=2, col=3)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title=f"{scenario_name}<br>DP出力 vs 最終planning/trajectory — 実機（上段）/ 速度差（下段）",
        height=820,
    )
