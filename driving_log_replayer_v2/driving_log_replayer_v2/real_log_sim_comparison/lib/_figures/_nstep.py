"""step5（N-step オープンループ解析）の図を組む build_fig_* 純関数群.

入力は `nstep_delta.csv` 由来の DataFrame（horizon, tr, err_ds_long/lat, err_steer,
yaw_err_deg, real_vx, ... の統一スキーマ）。横断軸統一（旧 LIMITS_DF グローバル）は
`limits` 引数で注入する（単一ケースは None で自己スケール、step6 が全ケース連結から
作って渡す）。ROS 非依存。
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import plotly.graph_objects as go

from .._nstep_common import ERR_METRICS, YAW_SEED_NOTE
from ._common import (
    add_params_annotation_plotly,
    apply_base_layout,
    axis_range_from_limits,
    ma_window,
    make_grid,
    polyfit_line,
    speed_bin_masks,
    viridis_colors,
)

_RAD2DEG = 180.0 / np.pi


def _repr_horizons(df: pd.DataFrame) -> list[int]:
    """行表示に使う代表 horizon: [N=min] または [N=min, N=max]。"""
    h_min, h_max = int(df["horizon"].min()), int(df["horizon"].max())
    return [h_min] if h_max == h_min else [h_min, h_max]

# AUTONOMOUS 開始（tr=0）を示す緑点線の共通体裁。
_VLINE = dict(line=dict(color="green", width=1.0, dash="dot"))


def _ma(vals: np.ndarray, n: int) -> np.ndarray:
    """中央寄せ移動平均（端は短縮窓）。matplotlib 側の rolling(center, min_periods=1) と等価。"""
    return pd.Series(vals).rolling(ma_window(n), center=True, min_periods=1).mean().to_numpy()


def _set_yrange(fig: go.Figure, row: int, col: int, rng) -> None:
    if rng is not None:
        fig.update_yaxes(range=rng, row=row, col=col)


def build_fig_error_timeseries(
    df: pd.DataFrame, *, params: dict | None = None, limits_df=None
) -> go.Figure:
    """N-step 終端誤差の時系列（horizon 別 viridis 色分け、raw 薄 + 移動平均）。

    旧 plot_error_timeseries の plotly 版。ERR_METRICS の 4 メトリクスを縦に積む。
    凡例は最上段にのみ出し、legendgroup で全段同時トグルする。データは間引かず全点入れる。
    """
    horizons = sorted(df["horizon"].unique())
    colors = dict(zip(horizons, viridis_colors(len(horizons))))
    titles = []
    for col, _scale, label, _unit, source in ERR_METRICS:
        t = f"N-step 終端 {label}（実機 − モデル）<br><sub>{source}</sub>"
        if col == "yaw_err_deg":
            t += f"<br><sub>{YAW_SEED_NOTE}</sub>"
        titles.append(t)

    fig = make_grid(len(ERR_METRICS), 1, subplot_titles=titles, shared_xaxes=True,
                    vertical_spacing=0.07)

    for row, (col, scale, label, unit, _source) in enumerate(ERR_METRICS, start=1):
        for h in horizons:
            sub = df[df["horizon"] == h].sort_values("tr")
            tr = sub["tr"].to_numpy()
            vals = sub[col].to_numpy(dtype=float) * scale
            ma = _ma(vals, len(sub))
            grp = f"N{int(h)}"
            first = row == 1
            fig.add_trace(go.Scatter(
                x=tr, y=vals, mode="lines", legendgroup=grp, showlegend=False,
                line=dict(color=colors[h], width=0.4), opacity=0.25, hoverinfo="skip",
            ), row=row, col=1)
            fig.add_trace(go.Scatter(
                x=tr, y=ma, mode="lines", name=f"N={int(h)} MA",
                legendgroup=grp, showlegend=first,
                line=dict(color=colors[h], width=1.6),
            ), row=row, col=1)
        fig.add_hline(y=0, line=dict(color="black", width=0.8), row=row, col=1)
        fig.add_vline(x=0, row=row, col=1, **_VLINE)
        fig.update_yaxes(title_text=f"{label} [{unit}]", row=row, col=1)
        _set_yrange(fig, row, 1, axis_range_from_limits(limits_df, col, scale))

    fig.update_xaxes(
        title_text="rollout 開始時刻（AUTONOMOUS 開始からの経過）[s]",
        row=len(ERR_METRICS), col=1,
    )
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title="N-step オープンループ 誤差時系列（N=1 = 毎ステップリセット）",
        height=300 * len(ERR_METRICS) + 80,
    )


def _raw_ma_traces(fig, row, col, tr, vals, *, raw_color, ma_color, ma_name,
                   ma_dash="solid", legendgroup=None, showlegend=True):
    """raw（薄線）+ 移動平均（太線）の 2 trace を 1 パネルに足す共通処理。"""
    fig.add_trace(go.Scatter(
        x=tr, y=vals, mode="lines", showlegend=False, legendgroup=legendgroup,
        line=dict(color=raw_color, width=0.4), opacity=0.45, hoverinfo="skip",
    ), row=row, col=col)
    fig.add_trace(go.Scatter(
        x=tr, y=_ma(vals, len(vals)), mode="lines", name=ma_name,
        legendgroup=legendgroup, showlegend=showlegend,
        line=dict(color=ma_color, width=1.6, dash=ma_dash),
    ), row=row, col=col)


def build_fig_overview(
    df: pd.DataFrame, *, params: dict | None = None, limits_df=None
) -> go.Figure:
    """N=1 概観 2×2（速度・加速度・縦誤差・横誤差）。旧 plot_overview の plotly 版。"""
    tr = df["tr"].to_numpy()
    fig = make_grid(
        2, 2,
        subplot_titles=[
            "速度: 実機 vs モデル<br><sub>実機: kinematic_state/twist.linear.x  モデル: state_[3]</sub>",
            "加速度: 指令 vs 実機<br><sub>指令: control_cmd/longitudinal.acceleration  実機: acceleration/accel.linear.x</sub>",
            "1ステップ縦方向誤差<br><sub>実機: kinematic_state/pose.position  モデル: state_[0,1]</sub>",
            "1ステップ横方向誤差<br><sub>実機: kinematic_state/pose.position  モデル: state_[0,1]</sub>",
        ],
    )
    # 速度 (0,0)
    fig.add_trace(go.Scatter(x=tr, y=df["real_vx"], mode="lines", name="実機 vx",
                             line=dict(color="blue", width=1.2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=tr, y=df["sim_vx"], mode="lines", name="モデル vx",
                             line=dict(color="red", width=1.0, dash="dash")), row=1, col=1)
    fig.update_yaxes(title_text="速度 [m/s]", row=1, col=1)
    _set_yrange(fig, 1, 1, axis_range_from_limits(limits_df, ["real_vx", "sim_vx"], horizon=1))
    # 加速度 (0,1)
    fig.add_trace(go.Scatter(x=tr, y=df["accel_des"], mode="lines", name="指令 accel_des",
                             line=dict(color="gray", width=0.8)), row=1, col=2)
    fig.add_trace(go.Scatter(x=tr, y=df["real_ax"], mode="lines", name="実機 ax",
                             line=dict(color="blue", width=1.0)), row=1, col=2)
    fig.update_yaxes(title_text="加速度 [m/s²]", row=1, col=2)
    # 縦誤差 (1,0) / 横誤差 (1,1)
    for col, errcol, ylabel in [(1, "err_ds_long", "縦方向誤差 [cm]"), (2, "err_ds_lat", "横方向誤差 [cm]")]:
        vals = df[errcol].to_numpy(dtype=float) * 100
        _raw_ma_traces(fig, 2, col, tr, vals, raw_color="gray", ma_color="red",
                       ma_name="移動平均", showlegend=col == 1)
        fig.add_hline(y=0, line=dict(color="black", width=0.8), row=2, col=col)
        fig.update_yaxes(title_text=ylabel, row=2, col=col)
        _set_yrange(fig, 2, col, axis_range_from_limits(limits_df, errcol, 100, horizon=1))
    for r in (1, 2):
        for c in (1, 2):
            fig.add_vline(x=0, row=r, col=c, **_VLINE)
            fig.update_xaxes(title_text="AUTONOMOUS 開始からの時刻 [s]", row=r, col=c)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title="全走行 N=1 (per-step) 分析<br><sub>(各ステップで実機状態にリセット — 計画挙動の差を除外)</sub>",
        height=820,
    )


# (real_col, ylabel, title, source)。横方向諸量（sim_* はラッパー未 export のため実機のみ）。
_LATERAL_ROWS = [
    ("real_ay", "横加速度 ay [m/s²]", "横加速度 ay: 実機", "実機: 位置微分(2階)→移動平均スムージング"),
    ("real_vy", "横速度 vy [m/s]", "横速度 vy: 実機", "実機: 位置微分→body frame変換→移動平均スムージング"),
    ("real_wz", "角速度 wz [rad/s]", "角速度 wz: 実機", "実機: kinematic_state/twist.angular.z"),
    ("real_dwz", "角加速度 dwz [rad/s²]", "角加速度 dwz: 実機",
     "実機: d/dt(kinematic_state/twist.angular.z) np.gradient"),
]


def build_fig_lateral_dynamics_timeseries(
    df: pd.DataFrame, *, params: dict | None = None
) -> go.Figure:
    """横方向諸量 ay/vy/wz/dwz の N=1 時系列（実機 raw + MA）。旧 plot_lateral_dynamics_timeseries。"""
    tr = df["tr"].to_numpy()
    titles = [f"{title}<br><sub>{source}</sub>" for _c, _y, title, source in _LATERAL_ROWS]
    fig = make_grid(4, 1, shared_xaxes=True, subplot_titles=titles, vertical_spacing=0.06)
    for row, (col, ylabel, _t, _s) in enumerate(_LATERAL_ROWS, start=1):
        _raw_ma_traces(fig, row, 1, tr, df[col].to_numpy(dtype=float),
                       raw_color="#aaaaaa", ma_color="black", ma_name="実機 MA",
                       showlegend=row == 1)
        fig.add_hline(y=0, line=dict(color="black", width=0.6, dash="dot"), row=row, col=1)
        fig.add_vline(x=0, row=row, col=1, **_VLINE)
        fig.update_yaxes(title_text=ylabel, row=row, col=1)
    fig.update_xaxes(title_text="AUTONOMOUS 開始からの時刻 [s]", row=4, col=1)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title="全走行 N=1 (per-step): 横方向諸量 時系列<br><sub>(各ステップで実機状態にリセット)</sub>",
        height=1300,
    )


# (real_col, sim_col, err_col, scale, ylabel, title, source)
_CASCADE_ROWS = [
    ("real_steer_kend", "sim_steer_kend", "err_steer", 180.0 / np.pi, "ステア角 [deg]",
     "① ステア応答 (cmd→actual): 実機 vs モデル",
     "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias"),
    ("real_ds_lat", "sim_ds_lat", "err_ds_lat", 100.0, "横方向 Δpos [cm]",
     "② 横方向 1ステップ変位: 実機 vs モデル",
     "実機: kinematic_state/pose.position  モデル: state_[0,1]"),
]


def build_fig_cascade_error(
    df: pd.DataFrame, *, params: dict | None = None, limits_df=None
) -> go.Figure:
    """段階的誤差プロット（ステア応答→横位置）。左 3:1 右の時系列+誤差。旧 plot_cascade_error。"""
    tr = df["tr"].to_numpy()
    rows = len(_CASCADE_ROWS)
    titles = []
    for _rc, _sc, _ec, _sc2, _yl, title, source in _CASCADE_ROWS:
        titles += [f"{title}<br><sub>{source}</sub>", "誤差 (実機 − モデル)"]
    fig = make_grid(rows, 2, subplot_titles=titles, column_widths=[0.75, 0.25],
                    vertical_spacing=0.12, horizontal_spacing=0.08)
    for row, (real_col, sim_col, err_col, scale, ylabel, _t, _s) in enumerate(_CASCADE_ROWS, start=1):
        real_v = df[real_col].to_numpy(dtype=float) * scale
        sim_v = df[sim_col].to_numpy(dtype=float) * scale
        err_v = df[err_col].to_numpy(dtype=float) * scale
        # 左: 時系列（実機 + モデル）
        _raw_ma_traces(fig, row, 1, tr, real_v, raw_color="#aaaaaa", ma_color="black",
                       ma_name="実機 MA", showlegend=row == 1)
        _raw_ma_traces(fig, row, 1, tr, sim_v, raw_color="#ffaaaa", ma_color="red",
                       ma_name="モデル MA", ma_dash="dash", showlegend=row == 1)
        fig.add_hline(y=0, line=dict(color="black", width=0.5, dash="dot"), row=row, col=1)
        fig.update_yaxes(title_text=ylabel, row=row, col=1)
        _set_yrange(fig, row, 1, axis_range_from_limits(limits_df, [real_col, sim_col], scale, horizon=1))
        # 右: 誤差時系列
        rmse = float(np.sqrt(np.nanmean(err_v**2)))
        fig.add_trace(go.Scatter(x=tr, y=err_v, mode="lines", showlegend=False,
                                 line=dict(color="#aaaaaa", width=0.4), opacity=0.4,
                                 hoverinfo="skip"), row=row, col=2)
        fig.add_trace(go.Scatter(x=tr, y=_ma(err_v, len(err_v)), mode="lines",
                                 name=f"誤差 MA RMSE={rmse:.4g}", showlegend=row == 1,
                                 line=dict(color="purple", width=1.6)), row=row, col=2)
        fig.add_hline(y=0, line=dict(color="black", width=0.8), row=row, col=2)
        fig.update_yaxes(title_text=f"誤差 [{ylabel.split('[')[1]}", row=row, col=2)
        _set_yrange(fig, row, 2, axis_range_from_limits(limits_df, err_col, scale, horizon=1))
        for col in (1, 2):
            fig.add_vline(x=0, row=row, col=col, **_VLINE)
    for col in (1, 2):
        fig.update_xaxes(title_text="AUTONOMOUS 開始からの時刻 [s]", row=rows, col=col)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title="全走行 N=1 (per-step): 段階的誤差プロット<br><sub>ステア指示 → ステア応答 → 横位置</sub>",
        height=380 * rows,
    )


def _scatter_by_speed(fig, row, col, x, y, vx, *, showlegend):
    """速度ビン色分け散布を 1 パネルに足す（データは間引かず全点）。"""
    for mask, label, color in speed_bin_masks(vx):
        if mask.sum() == 0:
            continue
        fig.add_trace(go.Scatter(
            x=x[mask], y=y[mask], mode="markers", name=label,
            legendgroup=label, showlegend=showlegend,
            marker=dict(size=4, color=color, opacity=0.5),
        ), row=row, col=col)


def build_fig_error_vs_speed(
    df: pd.DataFrame, *, params: dict | None = None, limits_df=None
) -> go.Figure:
    """誤差の速度依存性散布（行 = N=1 / N=max、x=rollout 開始時 vx）。旧 plot_error_vs_speed。"""
    rows = _repr_horizons(df)
    h_min = rows[0]
    titles = []
    for h in rows:
        for col, _s, label, _u, source in ERR_METRICS:
            t = f"{label} vs 速度 (N={h})"
            if col == "yaw_err_deg" and h == h_min:
                t += f"<br><sub>{YAW_SEED_NOTE}</sub>"
            titles.append(t)
    fig = make_grid(len(rows), len(ERR_METRICS), subplot_titles=titles,
                    vertical_spacing=0.12, horizontal_spacing=0.06)
    for r, h in enumerate(rows, start=1):
        sub = df[df["horizon"] == h]
        vx = sub["real_vx"].to_numpy(dtype=float)
        for c, (col, scale, label, unit, _source) in enumerate(ERR_METRICS, start=1):
            vals = sub[col].to_numpy(dtype=float) * scale
            _scatter_by_speed(fig, r, c, vx, vals, vx, showlegend=(r == 1 and c == 1))
            fig.add_hline(y=0, line=dict(color="black", width=0.8), row=r, col=c)
            fig.update_xaxes(title_text="rollout 開始時点の速度 [m/s]", row=r, col=c)
            fig.update_yaxes(title_text=f"{label} [{unit}]", row=r, col=c)
            rng = axis_range_from_limits(limits_df, col, scale, horizon=h)
            if rng is not None:
                fig.update_yaxes(range=rng, row=r, col=c)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig, title="N-step オープンループ: 速度依存性 (上段 N=1, 下段 N=max)",
        height=460 * len(rows),
    )


def build_fig_steering_analysis(
    df: pd.DataFrame, *, params: dict | None = None, limits_df=None
) -> go.Figure:
    """ステア 1ステップ予測の 4 パネル分析。旧 plot_steering_analysis。"""
    tr = df["tr"].to_numpy()
    err_deg = df["err_steer"].to_numpy(dtype=float) * _RAD2DEG
    rmse_deg = float(np.sqrt(np.nanmean(err_deg**2)))
    fig = make_grid(
        2, 2, vertical_spacing=0.12, horizontal_spacing=0.08,
        subplot_titles=[
            "ステア角: 実機[k+1] vs モデル予測[k+1] vs 指令[k]<br><sub>実機: steering_status/tire_angle  モデル: state_[4]+bias  指令: control_cmd/lat.tire_angle</sub>",
            "実機ステア追従誤差 (actual[k+1] − cmd[k])<br><sub>実機: steering_status/tire_angle  指令: control_cmd/lat.tire_angle</sub>",
            f"1ステップ ステア予測誤差 (actual[k+1] − pred[k+1])  RMSE={rmse_deg:.4f}°<br><sub>実機: steering_status/tire_angle  モデル: state_[4]+steer_bias</sub>",
            "ステア予測誤差 vs 指令ステア角（色=速度域）<br><sub>誤差: tire_angle − state_[4]+bias  指令: control_cmd/lat.tire_angle</sub>",
        ],
    )
    # (0,0) ステア角 3 系列
    for col, color, dash, name in [
        ("real_steer_kend", "blue", "solid", "実機 steer[k+1]"),
        ("sim_steer_kend", "red", "dash", "予測 steer[k+1]"),
        ("steer_des", "gray", "dot", "指令 steer_des[k]"),
    ]:
        fig.add_trace(go.Scatter(x=tr, y=df[col].to_numpy(dtype=float) * _RAD2DEG, mode="lines",
                                 name=name, line=dict(color=color, width=1.1, dash=dash)), row=1, col=1)
    fig.update_yaxes(title_text="ステア角 [deg]", row=1, col=1)
    rng = axis_range_from_limits(limits_df, ["real_steer_kend", "sim_steer_kend", "steer_des"], _RAD2DEG, horizon=1)
    if rng is not None:
        fig.update_yaxes(range=rng, row=1, col=1)
    # (0,1) 追従誤差 raw+MA
    follow = (df["real_steer_kend"].to_numpy(dtype=float) - df["steer_des"].to_numpy(dtype=float)) * _RAD2DEG
    _raw_ma_traces(fig, 1, 2, tr, follow, raw_color="gray", ma_color="blue", ma_name="移動平均")
    fig.add_hline(y=0, line=dict(color="black", width=0.8), row=1, col=2)
    fig.update_yaxes(title_text="追従誤差 [deg]", row=1, col=2)
    # (1,0) 予測誤差 raw+MA
    _raw_ma_traces(fig, 2, 1, tr, err_deg, raw_color="gray", ma_color="red", ma_name="移動平均")
    fig.add_hline(y=0, line=dict(color="black", width=0.8), row=2, col=1)
    fig.update_yaxes(title_text="予測誤差 [deg]", row=2, col=1)
    rng = axis_range_from_limits(limits_df, "err_steer", _RAD2DEG, horizon=1)
    if rng is not None:
        fig.update_yaxes(range=rng, row=2, col=1)
    # (1,1) 予測誤差 vs 指令ステア角（速度色分け散布）
    cmd_deg = df["steer_des"].to_numpy(dtype=float) * _RAD2DEG
    _scatter_by_speed(fig, 2, 2, cmd_deg, err_deg, df["real_vx"].to_numpy(dtype=float), showlegend=True)
    fig.add_hline(y=0, line=dict(color="black", width=0.8), row=2, col=2)
    fig.update_xaxes(title_text="指令ステア角 [deg]", row=2, col=2)
    fig.update_yaxes(title_text="予測誤差 [deg]", row=2, col=2)
    rng = axis_range_from_limits(limits_df, "err_steer", _RAD2DEG, horizon=1)
    if rng is not None:
        fig.update_yaxes(range=rng, row=2, col=2)
    for r, c in [(1, 1), (1, 2), (2, 1)]:
        fig.add_vline(x=0, row=r, col=c, **_VLINE)
        fig.update_xaxes(title_text="AUTONOMOUS 開始からの時刻 [s]", row=r, col=c)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title="全走行 N=1 (per-step) ステアリング分析<br><sub>(1ステップ予測誤差: actual[kend] − model_pred[kend])</sub>",
        height=920,
    )


# (real_col, ylabel, source)。横方向諸量 vs ステア角（sim_* はラッパー未 export のため実機のみ）。
_STEER_LATERAL_COLS = [
    ("real_ay", "横加速度 ay [m/s²]", "横軸: steering_status/tire_angle(t_k)  縦軸: 位置微分(2階)"),
    ("real_vy", "横速度 vy [m/s]", "横軸: steering_status/tire_angle(t_k)  縦軸: 位置微分body frame"),
    ("real_wz", "角速度 wz [rad/s]", "横軸: steering_status/tire_angle(t_k)  縦軸: kinematic_state/twist.angular.z"),
]


def build_fig_steer_vs_lateral_scatter(
    df: pd.DataFrame, *, params: dict | None = None
) -> go.Figure:
    """ステア角 × 横方向諸量の散布 + 1 次フィット（速度域別）。旧 plot_steer_vs_lateral_scatter。"""
    steer_deg = df["real_steer_k0"].to_numpy(dtype=float) * _RAD2DEG
    vx = df["real_vx"].to_numpy(dtype=float)
    titles = [f"ステア角 vs {ylabel}<br><sub>{source}</sub>" for _c, ylabel, source in _STEER_LATERAL_COLS]
    fig = make_grid(1, 3, subplot_titles=titles, horizontal_spacing=0.07)
    for c, (col, ylabel, _s) in enumerate(_STEER_LATERAL_COLS, start=1):
        real_vals = df[col].to_numpy(dtype=float)
        _scatter_by_speed(fig, 1, c, steer_deg, real_vals, vx, showlegend=(c == 1))
        xs, ys, coef = polyfit_line(steer_deg, real_vals, n=100)
        if xs is not None:
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="lines", name=f"実機 fit: {coef[0]:.4f}·θ + {coef[1]:.4f}",
                line=dict(color="black", width=2),
            ), row=1, col=c)
        fig.add_hline(y=0, line=dict(color="black", width=0.6), row=1, col=c)
        fig.add_vline(x=0, line=dict(color="black", width=0.6), row=1, col=c)
        fig.update_xaxes(title_text="ステア角 [deg]", row=1, col=c)
        fig.update_yaxes(title_text=ylabel, row=1, col=c)
    add_params_annotation_plotly(fig, params)
    return apply_base_layout(
        fig,
        title="全走行 N=1 (per-step): ステア角 vs 横方向諸量 散布図<br><sub>(スケール誤差・バイアス確認)</sub>",
        height=520,
    )
