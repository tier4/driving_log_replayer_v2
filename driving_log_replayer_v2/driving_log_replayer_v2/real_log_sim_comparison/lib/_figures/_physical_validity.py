"""物理妥当性検証 (縦方向 / 操舵 / 横方向 k_us) の図スペック (ROS 非依存・純関数).

per-dataset (step6_analyze_cases) の単一データセット図と、collection 横断
(step13_cross_dataset) の best/worst データセット + 横断フィット重ね描き図の両方を提供する。
データ整形は `lib._physical_validity` (ROS 依存の MCAP 読み込みを含む) が担当し、
本モジュールは計算済み dict/list を受けて `go.Figure` を組むだけの純関数。
"""

from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go

from .._kus_profile import VX_EDGES, _kus_band_label, _kus_step_profile
from ._common import apply_base_layout, make_grid, qualitative_colors


def _placeholder_fig(msg: str, height: int = 200) -> go.Figure:
    """データ不足時のプレースホルダー図を返す。"""
    fig = go.Figure()
    fig.add_annotation(text=msg, x=0.5, y=0.5, xref="paper", yref="paper",
                       showarrow=False, font={"size": 14})
    fig.update_layout(height=height)
    return fig


# ---------------------------------------------------------------------------
# per-dataset: 単一データセット時系列
# ---------------------------------------------------------------------------
def build_fig_long_single(ts: dict | None, fit: dict | None) -> go.Figure:
    """縦方向モデルフィット時系列 (単一データセット、実測/指令/同定値/モデル別チューン値)。"""
    if ts is None:
        return _placeholder_fig("縦方向データ不足 (MCAP 読み込み失敗、または動的区間不足)")

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=ts["t"], y=ts["a_act"], name="実測加速度",
                              line=dict(color="black", width=1.5), connectgaps=False))
    fig.add_trace(go.Scatter(x=ts["t"], y=ts["a_cmd"], name="指令加速度",
                              line=dict(color="gray", width=1.2, dash="dash"), connectgaps=False))
    if ts.get("a_sim_fit") is not None:
        label = "同定値"
        if fit is not None and np.isfinite(fit.get("tau", float("nan"))):
            label = (
                f"同定値 τ={fit['tau']:.3f}s T={fit['delay']:.3f}s "
                f"RMSE={fit.get('rmse_mps2', float('nan')):.3f} m/s²"
            )
        fig.add_trace(go.Scatter(x=ts["t"], y=ts["a_sim_fit"], name=label,
                                  line=dict(color="royalblue", width=2.0), connectgaps=False))
    a_sim_models = ts.get("a_sim_models") or {}
    model_tune = ts.get("model_tune") or {}
    colors = qualitative_colors(len(a_sim_models))
    for (name, series), color in zip(a_sim_models.items(), colors):
        tune = model_tune.get(name, {})
        label = f"{name} τ={tune.get('tau', float('nan')):.3f}s T={tune.get('T', float('nan')):.3f}s"
        fig.add_trace(go.Scatter(x=ts["t"], y=series, name=label,
                                  line=dict(color=color, width=1.3, dash="dash"), connectgaps=False))
    fig.update_yaxes(title_text="a [m/s²]")
    fig.update_xaxes(title_text="時刻 [s]")
    return apply_base_layout(
        fig,
        title="縦方向モデルフィット（実測 vs 同定値 vs モデル別チューニング値、路面勾配補正込み・走行区間のみ表示）",
        height=380,
    )


def build_fig_steer_single(ts: dict | None, fit: dict | None) -> go.Figure:
    """操舵モデルフィット時系列 (単一データセット、実測/同定値/モデル別チューン値)。"""
    if ts is None:
        return _placeholder_fig("操舵データ不足 (MCAP 読み込み失敗、または動的区間不足)")

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=ts["t"], y=ts["d_act"], name="実測 δ_act",
                              line=dict(color="black", width=1.5), connectgaps=False))
    if ts.get("d_sim_fit") is not None:
        label = "同定値"
        if fit is not None and np.isfinite(fit.get("tau", float("nan"))):
            label = (
                f"同定値 τ={fit['tau']:.3f}s T={fit['delay']:.3f}s "
                f"RMSE={fit.get('rmse_mrad', float('nan')):.1f} mrad"
            )
        fig.add_trace(go.Scatter(x=ts["t"], y=ts["d_sim_fit"], name=label,
                                  line=dict(color="steelblue", width=1.5, dash="dot"), connectgaps=False))
    d_sim_models = ts.get("d_sim_models") or {}
    model_tune = ts.get("model_tune") or {}
    colors = qualitative_colors(len(d_sim_models))
    for (name, series), color in zip(d_sim_models.items(), colors):
        tune = model_tune.get(name, {})
        label = f"{name} τ={tune.get('tau', float('nan')):.3f}s T={tune.get('T', float('nan')):.3f}s"
        fig.add_trace(go.Scatter(x=ts["t"], y=series, name=label,
                                  line=dict(color=color, width=1.3, dash="dash"), connectgaps=False))
    fig.update_yaxes(title_text="δ [rad]")
    fig.update_xaxes(title_text="時刻 [s]")
    return apply_base_layout(
        fig, title="操舵モデルフィット（実測 vs 同定値 vs モデル別チューニング値、走行区間のみ表示）", height=380,
    )


def build_fig_kus_single(bins: dict | None, models: dict[str, dict]) -> go.Figure:
    """k_us(v) 速度ビン別最小二乗法推定 (単一データセット、モデル別チューニング値重ね描き)。"""
    if bins is None:
        return _placeholder_fig("横方向データ不足 (曲線走行サンプル不足)")
    return _build_fig_kus(
        bins, models or {},
        title="k_us 独立同定（速度ビン別 最小二乗法回帰、単一データセット）",
    )


# ---------------------------------------------------------------------------
# collection 横断: k_us(v) プール再構成 + モデル別チューン値重ね描き
# ---------------------------------------------------------------------------
def build_fig_cross_kus(bins: dict | None, models: dict | None) -> go.Figure:
    """dataset 横断 k_us(v) 独立同定 (十分統計量プール集計 + モデル別チューニング値比較)。"""
    if bins is None:
        return _placeholder_fig("横方向データ不足 (全データセットで曲線走行サンプル不足)")
    model_params = {name: spec.params for name, spec in (models or {}).items()}
    return _build_fig_kus(
        bins, model_params,
        title="dataset 横断 k_us(v) 独立同定（速度ビン別最小二乗法プール集計）",
    )


def _build_fig_kus(bins: dict, model_params: dict[str, dict], *, title: str) -> go.Figure:
    fig = make_grid(
        rows=1, cols=2,
        subplot_titles=["k_us 推定値（速度ビン別 最小二乗法）", "速度ビン別 曲線走行サンプル数"],
        horizontal_spacing=0.12,
    )

    vx_mid = np.asarray(bins["vx_mid"], dtype=float)
    kus_ols = np.asarray(bins["kus_ols"], dtype=float)
    n_pts = np.asarray(bins["n_pts"])
    valid = np.isfinite(kus_ols)

    kus_p25 = np.asarray(bins.get("kus_p25", np.full_like(vx_mid, np.nan)), dtype=float)
    kus_p75 = np.asarray(bins.get("kus_p75", np.full_like(vx_mid, np.nan)), dtype=float)
    valid_iqr = valid & np.isfinite(kus_p25) & np.isfinite(kus_p75)
    if np.any(valid_iqr):
        fig.add_trace(go.Scatter(
            x=list(vx_mid[valid_iqr]) + list(vx_mid[valid_iqr][::-1]),
            y=list(kus_p25[valid_iqr]) + list(kus_p75[valid_iqr][::-1]),
            fill="toself", fillcolor="rgba(70,130,180,0.15)",
            line=dict(color="rgba(255,255,255,0)"),
            showlegend=True, name="25–75%ile（個別サンプル）",
        ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=vx_mid[valid].tolist(), y=kus_ols[valid].tolist(),
        mode="markers+lines",
        marker=dict(color="steelblue", size=7),
        line=dict(color="steelblue", width=2),
        name="最小二乗法推定 k_us(v)",
    ), row=1, col=1)

    vx_dense = np.linspace(0.0, 12.0, 600)
    colors = qualitative_colors(len(model_params)) if model_params else []
    for (name, params), color in zip(model_params.items(), colors):
        kus_tune = _kus_step_profile(vx_dense, params)
        label = f"{name}: {_kus_band_label(params)}"
        fig.add_trace(go.Scatter(
            x=vx_dense.tolist(), y=kus_tune.tolist(), mode="lines",
            line=dict(color=color, width=2.5, dash="dash"), name=label,
        ), row=1, col=1)

    fig.add_hline(y=0.0, line=dict(color="gray", width=1, dash="dot"), row=1, col=1)

    bin_widths = np.diff(VX_EDGES)
    fig.add_trace(go.Bar(
        x=vx_mid.tolist(), y=n_pts.tolist(),
        width=(bin_widths * 0.8).tolist(),
        marker_color="steelblue", opacity=0.6,
        name="サンプル数", showlegend=False,
    ), row=1, col=2)

    fig.update_xaxes(title_text="車速 vx [m/s]")
    fig.update_yaxes(title_text="k_us [rad·s²/m]", range=[-0.05, 0.12], row=1, col=1)
    fig.update_yaxes(title_text="サンプル数", row=1, col=2)
    return apply_base_layout(
        fig, title=title, height=430,
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
    )


# ---------------------------------------------------------------------------
# collection 横断: best/worst 時系列 + 横断フィット + モデル別チューン値
# ---------------------------------------------------------------------------
def build_fig_cross_long(rows_data: list[dict], cross_fit: dict) -> go.Figure:
    """dataset 横断 縦方向モデルフィット (best/worst 時系列 + 横断最小二乗法同定値)。"""
    if not rows_data:
        return _placeholder_fig("MCAP 読み込み失敗（best/worst データセットが見つかりません）")

    n = len(rows_data)
    all_model_names: list[str] = []
    for r in rows_data:
        for name in (r.get("a_sim_models") or {}):
            if name not in all_model_names:
                all_model_names.append(name)
    colors = qualitative_colors(len(all_model_names))
    color_of = dict(zip(all_model_names, colors))

    fig = make_grid(rows=n, cols=1, subplot_titles=[r["label"] for r in rows_data], vertical_spacing=0.08)
    show_legend = True
    for i, r in enumerate(rows_data, 1):
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["a_act"], name="実測加速度",
            line=dict(color="black", width=1.5), showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["a_cmd"], name="指令加速度",
            line=dict(color="gray", width=1.2, dash="dash"), showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        if r.get("a_sim_cross") is not None:
            label = (
                f"横断同定値 τ={cross_fit['tau']:.3f}s T={cross_fit['delay']:.3f}s "
                f"RMSE={cross_fit.get('rmse_mps2', float('nan')):.3f} m/s²"
            )
            fig.add_trace(go.Scatter(
                x=r["t"], y=r["a_sim_cross"], name=label,
                line=dict(color="royalblue", width=2.0), showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        for name, series in (r.get("a_sim_models") or {}).items():
            fig.add_trace(go.Scatter(
                x=r["t"], y=series, name=f"{name} (チューニング値)",
                line=dict(color=color_of[name], width=1.3, dash="dot"),
                showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        show_legend = False
        fig.update_yaxes(title_text="a [m/s²]", row=i, col=1)
    fig.update_xaxes(title_text="時刻 [s]", row=n, col=1)

    p_min_deg = math.degrees(cross_fit.get("pitch_min", 0.0))
    p_max_deg = math.degrees(cross_fit.get("pitch_max", 0.0))
    slope_note = f"横断フィット使用データセット数={cross_fit.get('n_datasets', 0)}、pitch range {p_min_deg:+.2f}°〜{p_max_deg:+.2f}°"
    return apply_base_layout(
        fig,
        title=f"dataset 横断 縦方向モデルフィット（最良・最悪 計 {n} 件、路面勾配補正込み）<br><sup>{slope_note}</sup>",
        height=300 * n, margin=dict(t=80, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )


def build_fig_cross_steer(rows_data: list[dict]) -> go.Figure:
    """dataset 横断 操舵モデルフィット (best/worst 時系列 + per-dataset 同定値 + モデル別チューン値)。"""
    if not rows_data:
        return _placeholder_fig("MCAP 読み込み失敗（best/worst データセットが見つかりません）")

    n = len(rows_data)
    all_model_names: list[str] = []
    for r in rows_data:
        for name in (r.get("d_sim_models") or {}):
            if name not in all_model_names:
                all_model_names.append(name)
    colors = qualitative_colors(len(all_model_names))
    color_of = dict(zip(all_model_names, colors))

    fig = make_grid(rows=n, cols=1, subplot_titles=[r["label"] for r in rows_data], vertical_spacing=0.08)
    show_legend = True
    for i, r in enumerate(rows_data, 1):
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["d_act"], name="実測 δ_act",
            line=dict(color="black", width=1.5), showlegend=show_legend, connectgaps=False,
        ), row=i, col=1)
        if r.get("d_sim_fit") is not None:
            fig.add_trace(go.Scatter(
                x=r["t"], y=r["d_sim_fit"], name="非線形最小二乗法同定値（当該データセット）",
                line=dict(color="steelblue", width=1.5, dash="dot"), showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        for name, series in (r.get("d_sim_models") or {}).items():
            fig.add_trace(go.Scatter(
                x=r["t"], y=series, name=f"{name} (チューニング値)",
                line=dict(color=color_of[name], width=1.3, dash="dash"),
                showlegend=show_legend, connectgaps=False,
            ), row=i, col=1)
        show_legend = False
        fig.update_yaxes(title_text="δ [rad]", row=i, col=1)
    fig.update_xaxes(title_text="時刻 [s]", row=n, col=1)
    return apply_base_layout(
        fig, title=f"dataset 横断 操舵モデルフィット（最良・最悪 計 {n} 件）",
        height=300 * n, margin=dict(t=70, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )
