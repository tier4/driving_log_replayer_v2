"""車両制御モデルの**運動方程式そのもの**を可視化する概念図 (build_fig_*).

`delay_steer_acc_geared_wo_fall_guard` の数式が定数を変えるとどう振る舞うかを、実機データに
依存せず（純粋に式から）プロットする。議事録「運動方程式の可視化: 運動方程式をプロットして
記載する」に対応。各図は `params`（`load_sim_params()` の戻り = 仕様値）と best_normal の
モデル値（下記定数）を対比して描き、座標系・定数を図下部に注記する。詳細な式・定数表は
docs/vehicle_model.ja.md（report の「車両制御モデル」セクション）に集約。

**ROS 非依存**: numpy / plotly と `_common` のみに依存（実機配列を引数に取らない純関数）。
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ._common import (
    add_bottom_note,
    apply_base_layout,
    coord_note,
    make_grid,
    model_const_note,
)

# best_normal の同定値（sample/cases.yaml）。仕様値（params）との対比に使う。
BEST_NORMAL_K_US = 0.018
BEST_NORMAL_STEER_TC = 0.15
BEST_NORMAL_ACC_TC = 0.30

# 概念図の前進速度域・ステア角域（実機走行域を包含する範囲）。
_V_MAX = 14.0
_DELTA_SHOWN = (0.05, 0.10, 0.20)  # rad。yaw rate ファミリの代表ステア角


def _wheelbase(params: dict) -> float:
    return float(params.get("wheel_base", params.get("wheelbase", 4.76012)))


def _yaw_rate(v: np.ndarray, delta: float, L: float, k_us: float) -> np.ndarray:
    """ω = v·tan(δ)/(L + k_us·v²)（C++ calc_yaw_rate と同形・bias 無し）。"""
    return v * np.tan(delta) / (L + k_us * v * v)


def build_fig_yaw_rate_vs_v(params: dict) -> go.Figure:
    """ヨーレート ω(v,δ) の速度依存を k_us=0（理想）と k_us=0.018（best_normal）で対比する。

    分母 L+k_us·v² の v² 項により、同じステア角でも高速ほど ω が小さく（曲がりにくく）なる
    アンダーステアを可視化する。実線=理想キネマティック (k_us=0)、破線=best_normal。
    """
    L = _wheelbase(params)
    v = np.linspace(0.0, _V_MAX, 200)
    fig = go.Figure()
    palette = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for delta, color in zip(_DELTA_SHOWN, palette):
        fig.add_trace(go.Scatter(
            x=v, y=_yaw_rate(v, delta, L, 0.0), mode="lines",
            line=dict(color=color, width=2.0),
            name=f"δ={delta:.2f} rad・k_us=0 (理想)",
            legendgroup=f"d{delta}",
        ))
        fig.add_trace(go.Scatter(
            x=v, y=_yaw_rate(v, delta, L, BEST_NORMAL_K_US), mode="lines",
            line=dict(color=color, width=2.0, dash="dash"),
            name=f"δ={delta:.2f} rad・k_us={BEST_NORMAL_K_US} (best_normal)",
            legendgroup=f"d{delta}",
        ))
    height = 540
    apply_base_layout(fig, title="ヨーレート ω = v·tan(δ)/(L + k_us·v²) の速度依存", height=height)
    fig.update_xaxes(title_text="前進速度 v [m/s]")
    fig.update_yaxes(title_text="ヨーレート ω [rad/s]")
    add_bottom_note(
        fig, coord_note() + "<br>" + model_const_note(params, k_us=BEST_NORMAL_K_US),
        height=height,
    )
    return fig


def build_fig_first_order_step(params: dict) -> go.Figure:
    """アクチュエータ 1 次遅れ + 無駄時間のステップ応答（ステア / 加速度）。

    指令ステップに対し `y(t)=1-exp(-(t-T)/τ)`（t<T は 0、T=無駄時間）で立ち上がる様子を、
    仕様値 τ と best_normal の τ で対比する。τ は応答が定常値の約 63%(=1-1/e) に達する時間
    （議事録の「60% 程度の応答時間」の定義）。
    """
    t = np.linspace(0.0, 1.5, 400)

    def _resp(tau: float, delay: float) -> np.ndarray:
        return np.where(
            t >= delay,
            1.0 - np.exp(-np.clip(t - delay, 0.0, None) / max(tau, 1e-6)),
            0.0,
        )

    steer_delay = float(params.get("steer_time_delay", 0.0315))
    acc_delay = float(params.get("acc_time_delay", 0.101))
    spec_steer_tc = float(params.get("steer_time_constant", 0.4983))
    spec_acc_tc = float(params.get("acc_time_constant", 0.2589))

    fig = make_grid(1, 2, subplot_titles=("ステア δ の応答", "加速度 a の応答"))
    panels = [
        (1, steer_delay, spec_steer_tc, BEST_NORMAL_STEER_TC),
        (2, acc_delay, spec_acc_tc, BEST_NORMAL_ACC_TC),
    ]
    for col, delay, spec_tc, best_tc in panels:
        step = np.where(t >= delay, 1.0, 0.0)
        fig.add_trace(go.Scatter(x=t, y=step, mode="lines", line=dict(color="#888888", dash="dot"),
                                 name="指令 (無駄時間後)", showlegend=(col == 1)), row=1, col=col)
        fig.add_trace(go.Scatter(x=t, y=_resp(spec_tc, delay), mode="lines",
                                 line=dict(color="#1f77b4", width=2.0),
                                 name="応答 (仕様 τ)", showlegend=(col == 1)), row=1, col=col)
        fig.add_trace(go.Scatter(x=t, y=_resp(best_tc, delay), mode="lines",
                                 line=dict(color="#d62728", width=2.0, dash="dash"),
                                 name="応答 (best_normal τ)", showlegend=(col == 1)), row=1, col=col)
        fig.add_hline(y=1.0 - 1.0 / np.e, line=dict(color="#aaaaaa", width=1.0, dash="dash"),
                      row=1, col=col)
        fig.update_xaxes(title_text="時刻 t [s]", row=1, col=col)
    fig.update_yaxes(title_text="正規化応答", row=1, col=1)
    height = 460
    apply_base_layout(fig, title="1 次遅れ + 無駄時間 ステップ応答（仕様 vs best_normal）",
                      height=height)
    add_bottom_note(
        fig,
        f"破線水平=63%(=1-1/e) 到達ライン。仕様 τ_δ={spec_steer_tc:.3g}s / τ_a={spec_acc_tc:.3g}s、"
        f"best_normal τ_δ={BEST_NORMAL_STEER_TC}s / τ_a={BEST_NORMAL_ACC_TC}s。"
        f"無駄時間 T_δ={steer_delay:.3g}s / T_a={acc_delay:.3g}s。",
        height=height,
    )
    return fig


def build_fig_lateral_accel_map(params: dict) -> go.Figure:
    """横加速度 a_y = v·ω = v²·tan(δ)/(L + k_us·v²) の (v, δ) マップ（best_normal）。

    求心加速度がどの速度・ステア角域で大きくなるかを等高線で示す（旋回時の縦横連成や
    速度差調査の文脈づけに使う概念図）。
    """
    L = _wheelbase(params)
    v = np.linspace(0.0, _V_MAX, 120)
    delta = np.linspace(0.0, 0.30, 120)
    V, D = np.meshgrid(v, delta)
    omega = V * np.tan(D) / (L + BEST_NORMAL_K_US * V * V)
    a_y = V * omega
    fig = go.Figure(data=go.Contour(
        x=v, y=delta, z=a_y, colorscale="Viridis",
        colorbar=dict(title="a_y [m/s²]"),
        contours=dict(showlabels=True),
    ))
    height = 520
    apply_base_layout(fig, title="横加速度 a_y = v·ω のマップ（k_us=0.018, best_normal）",
                      height=height)
    fig.update_xaxes(title_text="前進速度 v [m/s]")
    fig.update_yaxes(title_text="ステア角 δ [rad]")
    add_bottom_note(
        fig, coord_note() + "<br>" + model_const_note(params, k_us=BEST_NORMAL_K_US),
        height=height,
    )
    return fig
