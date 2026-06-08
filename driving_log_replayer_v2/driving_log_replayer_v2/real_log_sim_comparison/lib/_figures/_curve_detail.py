"""step4 のカーブ別詳細図 (curveN_*) を組む build_fig_* 純関数群.

一時停止発進からの挙動を軌跡＋時系列で詳細比較する 4 種（analysis / steering_detail /
yaw_steer / steer_response）。step4 が発進検出・clip・派生量算出（ヨーレート・等価ステア角・
追従誤差・正規化・立ち上がり時間等）を済ませた配列を run dict で渡し、ここは plotly subplot
レイアウトに専念する。ROS 非依存。これらはサンプル scenario（plot_curves 未設定）では生成
されず、カーブ設定シナリオでのみ出力される。

run dict は図ごとに必要な配列を持つ（各図の docstring 参照）。共通スタイルキー:
label / color / lw / ls / marker / ms。
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ._common import (
    apply_base_layout,
    lanes_to_trace,
    map_ways_in_bbox,
    plotly_dash,
    plotly_marker,
)

_VLINE = dict(line=dict(color="gray", width=1.0, dash="dash"))


def _add_traj_panel(fig, row, col, curve, map_ways, traj_runs):
    """軌跡パネル（地図背景 + 各 run のカーブ周辺軌跡 + ★発進点）を描く。"""
    cx, cy, mg = curve["cx"], curve["cy"], curve.get("mg", 80)
    if map_ways:
        ways = map_ways_in_bbox(map_ways, (cx - mg, cx + mg), (cy - mg, cy + mg))
        if ways:
            fig.add_trace(lanes_to_trace(ways), row=row, col=col)
    for r in traj_runs:
        if len(r.get("seg_x", [])) == 0:
            continue
        fig.add_trace(go.Scatter(
            x=r["seg_x"], y=r["seg_y"], mode="lines", name=r["label"], legendgroup=r["label"],
            line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-"))),
        ), row=row, col=col)
        if r.get("launch_x") is not None:
            fig.add_trace(go.Scatter(
                x=[r["launch_x"]], y=[r["launch_y"]], mode="markers", legendgroup=r["label"],
                showlegend=False, marker=dict(symbol="star", size=14, color=r["color"],
                line=dict(color="white", width=0.5)), hoverinfo="skip",
            ), row=row, col=col)
    fig.update_xaxes(title_text="x [m]", range=[cx - mg, cx + mg], row=row, col=col)
    ax = "" if (row == 1 and col == 1) else str((row - 1) * 3 + col)  # 近似。scaleanchor は下で設定
    return ax


def _line(fig, row, col, t, y, r, *, cmd=False, name=None):
    """時系列 1 本（応答 or 指令点線薄）を足す。"""
    if cmd:
        fig.add_trace(go.Scatter(x=t, y=y, mode="lines", name=name, legendgroup=r["label"],
                      showlegend=False, opacity=0.65,
                      line=dict(color=r["color"], width=1.2, dash="dot")), row=row, col=col)
    else:
        fig.add_trace(go.Scatter(x=t, y=y, mode="lines", name=name or r["label"], legendgroup=r["label"],
                      showlegend=(name is None or name == r["label"]),
                      line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-")))),
                      row=row, col=col)


def build_fig_curve_analysis(
    curve: dict, map_ways, traj_runs: list[dict], ts_runs: list[dict], *,
    scenario_name: str = "", params: dict | None = None,
) -> go.Figure:
    """カーブ発進挙動: 上段全幅=軌跡、下段3列=速度/加速度/ステア。旧 plot_curve_analysis。

    ts_runs: {label,color,lw,ls, t_vel,vel,t_cmd,cmd_vel, t_acc,acc,cmd_acc,
              t_steer,steer_deg,cmd_steer_deg}（cmd_* は無ければ None）。
    """
    label = curve.get("label", "カーブ")
    fig = make_subplots(
        rows=2, cols=3, specs=[[{"colspan": 3}, None, None], [{}, {}, {}]],
        subplot_titles=[f"軌跡（★=発進点、{label} 周辺）", "速度", "加速度", "ステアリング角"],
        vertical_spacing=0.12, horizontal_spacing=0.07,
    )
    _add_traj_panel(fig, 1, 1, curve, map_ways, traj_runs)
    for r in ts_runs:
        _line(fig, 2, 1, r["t_vel"], r["vel"], r)
        if r.get("t_cmd") is not None:
            _line(fig, 2, 1, r["t_cmd"], r["cmd_vel"], r, cmd=True)
        _line(fig, 2, 2, r["t_acc"], r["acc"], r, name=r["label"])
        if r.get("t_cmd") is not None:
            _line(fig, 2, 2, r["t_cmd"], r["cmd_acc"], r, cmd=True)
        _line(fig, 2, 3, r["t_steer"], r["steer_deg"], r, name=r["label"])
        if r.get("t_cmd") is not None:
            _line(fig, 2, 3, r["t_cmd"], r["cmd_steer_deg"], r, cmd=True)
    for c, unit in [(1, "m/s"), (2, "m/s²"), (3, "deg")]:
        fig.add_vline(x=0, row=2, col=c, **_VLINE)
        fig.update_yaxes(title_text=unit, row=2, col=c)
        fig.update_xaxes(title_text="発進からの時刻 [s]", row=2, col=c)
    fig.update_yaxes(title_text="y [m]", scaleanchor="x", scaleratio=1, row=1, col=1)
    return apply_base_layout(
        fig, title=f"{scenario_name}<br>{label} 一時停止発進からの挙動比較", height=860,
    )


def build_fig_curve_steering_detail(
    curve: dict, map_ways, traj_runs: list[dict], steer_runs: list[dict], *,
    scenario_name: str = "", params: dict | None = None,
) -> go.Figure:
    """ステア詳細: 左上=軌跡 / 左下(縦2)=指令vs応答 / 右=角速度・追従誤差・累積操舵量。

    旧 plot_curve_steering_detail。steer_runs: {label,color,lw,ls, t_s, steer_deg,
    t_cmd, cmd_steer_deg(or None), rate_t, rate, err_t, err, integ}。
    """
    label = curve.get("label", "カーブ")
    fig = make_subplots(
        rows=3, cols=2, specs=[[{}, {}], [{"rowspan": 2}, {}], [None, {}]],
        subplot_titles=["軌跡（★=発進点）", "ステアリング角速度",
                        "ステア角 指令(点線) vs 応答(実線)", "指令追従誤差（応答−指令）",
                        "累積絶対操舵量（∫|dθ/dt|dt）"],
        vertical_spacing=0.10, horizontal_spacing=0.10,
    )
    _add_traj_panel(fig, 1, 1, curve, map_ways, traj_runs)
    fig.update_yaxes(title_text="y [m]", scaleanchor="x", scaleratio=1, row=1, col=1)
    for r in steer_runs:
        _line(fig, 2, 1, r["t_s"], r["steer_deg"], r, name=f"{r['label']} 応答")
        if r.get("t_cmd") is not None:
            _line(fig, 2, 1, r["t_cmd"], r["cmd_steer_deg"], r, cmd=True)
        if r.get("rate_t") is not None:
            _line(fig, 1, 2, r["rate_t"], r["rate"], r, name=r["label"])
        if r.get("err_t") is not None:
            _line(fig, 2, 2, r["err_t"], r["err"], r, name=r["label"])
        _line(fig, 3, 2, r["t_s"], r["integ"], r, name=r["label"])
    fig.update_yaxes(title_text="ステア角 [deg]", row=2, col=1)
    fig.update_xaxes(title_text="発進からの時刻 [s]", row=2, col=1)
    fig.update_yaxes(title_text="deg/s", row=1, col=2)
    fig.update_yaxes(title_text="誤差 [deg]", row=2, col=2)
    fig.update_yaxes(title_text="deg", row=3, col=2)
    fig.update_xaxes(title_text="発進からの時刻 [s]", row=3, col=2)
    for rc in [(2, 1), (1, 2), (2, 2), (3, 2)]:
        fig.add_vline(x=0, row=rc[0], col=rc[1], **_VLINE)
    for rc in [(1, 2), (2, 2)]:
        fig.add_hline(y=0, line=dict(color="gray", width=0.6), row=rc[0], col=rc[1])
    return apply_base_layout(
        fig, title=f"{scenario_name}<br>{label} ステアリング詳細分析（発進 t=0）", height=900,
    )


def build_fig_curve_yaw_steer(
    yaw_runs: list[dict], *, scenario_name: str = "", curve_label: str = "カーブ",
    wheelbase: float = 5.15, params: dict | None = None,
) -> go.Figure:
    """ステア角 vs 進行方向(ヨーレート換算) の 4 段。旧 plot_curve_yaw_steer。

    yaw_runs: {label,color,lw,ls, t, steer_deg, equiv_deg, diff_deg, yaw_rate_deg,
               yaw_rate_pred_deg, yaw_cum_deg}。
    """
    fig = make_subplots(
        rows=4, cols=1, shared_xaxes=True, vertical_spacing=0.06,
        subplot_titles=[
            "ステア角（実線）vs 等価ステア角（点線）── 一致するほど自転車モデル成立",
            f"ステア角 − 等価ステア角（進行方向とのズレ）  L={wheelbase:.2f}m",
            "ヨーレート（実線=実測、点線=自転車モデル予測）",
            "yaw 累積変化量（t=0 基準）── カーブの総旋回量",
        ],
    )
    for r in yaw_runs:
        _line(fig, 1, 1, r["t"], r["steer_deg"], r, name=f"{r['label']} ステア実測")
        _line(fig, 1, 1, r["t"], r["equiv_deg"], r, cmd=True)
        _line(fig, 2, 1, r["t"], r["diff_deg"], r, name=r["label"])
        _line(fig, 3, 1, r["t"], r["yaw_rate_deg"], r, name=f"{r['label']} 実測")
        _line(fig, 3, 1, r["t"], r["yaw_rate_pred_deg"], r, cmd=True)
        _line(fig, 4, 1, r["t"], r["yaw_cum_deg"], r, name=r["label"])
    for row, unit in [(1, "deg"), (2, "deg"), (3, "deg/s"), (4, "deg")]:
        fig.add_vline(x=0, row=row, col=1, **_VLINE)
        fig.add_hline(y=0, line=dict(color="gray", width=0.5), row=row, col=1)
        fig.update_yaxes(title_text=unit, row=row, col=1)
    fig.update_xaxes(title_text="発進からの時刻 [s]", row=4, col=1)
    return apply_base_layout(
        fig, title=f"{scenario_name}<br>{curve_label} ステア角 vs 実際の進行方向（自転車モデル L={wheelbase:.2f}m）",
        height=1280,
    )


def build_fig_steer_response(
    runs: list[dict], bars: dict, *, scenario_name: str = "", params: dict | None = None,
) -> go.Figure:
    """ステア応答性能: 上段全幅=正規化指令vs応答、中段=追従誤差/立ち上がり、下段=RMSE/ピーク率。

    旧 plot_steer_response。runs: {label,color,lw,ls,marker,ms, t_c,c_norm, t_a,a_norm,
    onset_delay, rise_t(or None), err_t,err, is_real}。bars: {labels, colors, rise, rmse, peak}。
    """
    fig = make_subplots(
        rows=3, cols=2, specs=[[{"colspan": 2}, None], [{}, {}], [{}, {}]],
        row_heights=[0.5, 0.25, 0.25], vertical_spacing=0.12, horizontal_spacing=0.10,
        subplot_titles=["正規化ステア角（指令 vs 応答、入力 onset t=0）", "追従誤差（応答 − 指令）",
                        "応答遅延（指令onset → 応答onset）", "RMSE（指令追従誤差）", "ピーク追従率"],
    )
    for r in runs:
        _line(fig, 1, 1, r["t_c"], r["c_norm"], r, cmd=True, name=f"{r['label']} 指令")
        fig.add_trace(go.Scatter(
            x=r["t_a"], y=r["a_norm"], mode="lines+markers", name=f"{r['label']} 応答",
            legendgroup=r["label"],
            line=dict(color=r["color"], width=r.get("lw", 1.5) + 0.5, dash=plotly_dash(r.get("ls", "-"))),
            marker=dict(symbol=plotly_marker(r.get("marker", "o")), size=r.get("ms", 4)),
        ), row=1, col=1)
        # 応答 onset の遅延を縦線 + 上端の双方向矢印 annotation
        fig.add_vline(x=r["onset_delay"], line=dict(color=r["color"], width=2.0, dash="dashdot"),
                      opacity=0.9, row=1, col=1)
        # 追従誤差
        fig.add_trace(go.Scatter(x=r["err_t"], y=r["err"], mode="lines", name=r["label"],
                      legendgroup=r["label"], showlegend=False,
                      line=dict(color=r["color"], width=r.get("lw", 1.5), dash=plotly_dash(r.get("ls", "-")))),
                      row=2, col=1)
        if r.get("is_real"):
            err = np.asarray(r["err"], dtype=float)
            fig.add_trace(go.Scatter(x=r["err_t"], y=np.where(err >= 0, err, 0), mode="lines",
                          name="実機オーバーステア(+)", line=dict(width=0), fill="tozeroy",
                          fillcolor="rgba(214,39,40,0.18)"), row=2, col=1)
            fig.add_trace(go.Scatter(x=r["err_t"], y=np.where(err < 0, err, 0), mode="lines",
                          name="実機アンダーステア(−)", line=dict(width=0), fill="tozeroy",
                          fillcolor="rgba(31,119,180,0.18)"), row=2, col=1)
    fig.add_vline(x=0, row=1, col=1, **_VLINE)
    fig.add_hline(y=1.0, line=dict(color="gray", width=0.5, dash="dot"), row=1, col=1)
    fig.add_hline(y=0.9, line=dict(color="gray", width=0.5, dash="dot"), row=1, col=1)
    fig.add_vline(x=0, row=2, col=1, **_VLINE)
    fig.add_hline(y=0, line=dict(color="gray", width=0.6), row=2, col=1)
    # 上端の遅延矢印（onset → 応答 onset）。各 run を少しずつ上にずらして注釈
    for i, r in enumerate(runs):
        yp = 1.12 + i * 0.09
        x_end = max(r["onset_delay"], 0.015)
        fig.add_annotation(x=x_end, y=yp, ax=0.0, ay=yp, xref="x", yref="y", axref="x", ayref="y",
                           showarrow=True, arrowhead=3, arrowside="end+start", arrowcolor=r["color"],
                           arrowwidth=1.8, text="", row=1, col=1)
        fig.add_annotation(x=x_end / 2, y=yp + 0.04, xref="x", yref="y", showarrow=False,
                           text=f"{r['label']}: {r['onset_delay'] * 1000:.0f} ms",
                           font=dict(size=9, color=r["color"]), row=1, col=1)
    # bar 3 種
    bl, bc = bars["labels"], bars["colors"]
    for (row, col, key) in [(2, 2, "rise"), (3, 1, "rmse"), (3, 2, "peak")]:
        fig.add_trace(go.Bar(x=bl, y=bars[key], marker=dict(color=bc), showlegend=False),
                      row=row, col=col)
    fig.update_yaxes(title_text="正規化ステア角", range=[-0.15, 1.45], row=1, col=1)
    fig.update_yaxes(title_text="誤差 [deg]", row=2, col=1)
    fig.update_yaxes(title_text="ms", row=2, col=2)
    fig.update_yaxes(title_text="deg", row=3, col=1)
    fig.update_yaxes(title_text="率", row=3, col=2)
    fig.update_xaxes(title_text="入力 onset からの時刻 [s]", row=1, col=1)
    return apply_base_layout(
        fig, title=f"{scenario_name}<br>ステアリング応答性能比較（ステア入力開始 t=0 に揃えた比較）",
        height=980,
    )
