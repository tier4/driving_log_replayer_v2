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
from plotly.subplots import make_subplots

from .._kus_profile import VX_EDGES, _kus_band_label, _kus_step_profile
from .._map import map_ways_in_bbox
from .._plotly_utils import lanes_to_trace
from ._common import apply_base_layout, make_grid, qualitative_colors


def _placeholder_fig(msg: str, height: int = 200) -> go.Figure:
    """データ不足時のプレースホルダー図を返す。"""
    fig = go.Figure()
    fig.add_annotation(text=msg, x=0.5, y=0.5, xref="paper", yref="paper",
                       showarrow=False, font={"size": 14})
    fig.update_layout(height=height)
    return fig


def _has_visible_values(series: list | None) -> bool:
    if series is None:
        return False
    arr = np.asarray(series, dtype=float)
    return bool(np.isfinite(arr).any())


def _low_speed_trace(x: list, y: list, *, dash: str = "solid", showlegend: bool = False) -> go.Scatter:
    return go.Scatter(
        x=x, y=y,
        name="低速・停車区間（同定対象外）",
        line=dict(color="rgba(120,120,120,0.45)", width=1.0, dash=dash),
        showlegend=showlegend,
        connectgaps=False,
        hovertemplate="低速・停車区間<br>t=%{x:.2f}s<br>a=%{y:.3f} m/s²<extra></extra>",
    )


# ---------------------------------------------------------------------------
# per-dataset: 単一データセット時系列
# ---------------------------------------------------------------------------
def build_fig_long_single(ts: dict | None, fit: dict | None) -> go.Figure:
    """縦方向モデルフィット時系列 (単一データセット、実測/指令/同定値/モデル別チューン値)。"""
    if ts is None:
        return _placeholder_fig("縦方向データ不足 (MCAP 読み込み失敗、または動的区間不足)")

    fig = go.Figure()
    low_legend_shown = False
    for key, dash in (
        ("a_act_low", "solid"),
        ("a_cmd_low", "dash"),
        ("a_sim_fit_low", "solid"),
    ):
        if _has_visible_values(ts.get(key)):
            fig.add_trace(_low_speed_trace(
                ts["t"], ts[key], dash=dash, showlegend=not low_legend_shown,
            ))
            low_legend_shown = True
    for series in (ts.get("a_sim_models_low") or {}).values():
        if _has_visible_values(series):
            fig.add_trace(_low_speed_trace(
                ts["t"], series, dash="dot", showlegend=not low_legend_shown,
            ))
            low_legend_shown = True

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
        title="縦方向モデルフィット（実測 vs 同定値 vs モデル別チューニング値、路面勾配補正込み・低速/停車区間はグレー表示）",
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


def build_fig_kus_single(
    bins: dict | None,
    models: dict[str, dict],
    *,
    thresholds: list | None = None,
    title: str | None = None,
) -> go.Figure:
    """k_us(v) 速度ビン別最小二乗法推定 (単一データセット、モデル別チューニング値重ね描き)。

    thresholds: k_us 速度帯の切替閾値 [m/s]。指定時のみ縦線を描く。
    title: 図タイトルの上書き (None で既定の単一データセット向けタイトル)。
    """
    if bins is None:
        return _placeholder_fig("横方向データ不足 (曲線走行サンプル不足)")
    return _build_fig_kus(
        bins, models or {},
        title=title or "k_us 独立同定（速度ビン別 最小二乗法回帰、単一データセット）",
        thresholds=thresholds,
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


def _build_fig_kus(
    bins: dict,
    model_params: dict[str, dict],
    *,
    title: str,
    thresholds: list | None = None,
) -> go.Figure:
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

    # k_us 速度帯の切替閾値縦線 (指定時のみ)
    _thr_colors = ["green", "purple", "brown", "teal"]
    for i, thr in enumerate(thresholds or []):
        clr = _thr_colors[i % len(_thr_colors)]
        fig.add_vline(
            x=thr, line=dict(color=clr, width=1.2, dash="dash"),
            annotation_text=f"thr{i+1}={thr:.1f}", annotation_position="top right",
            row=1, col=1,
        )

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
    low_legend_shown = False
    for i, r in enumerate(rows_data, 1):
        for key, dash in (
            ("a_act_low", "solid"),
            ("a_cmd_low", "dash"),
            ("a_sim_cross_low", "solid"),
        ):
            if _has_visible_values(r.get(key)):
                fig.add_trace(_low_speed_trace(
                    r["t"], r[key], dash=dash, showlegend=not low_legend_shown,
                ), row=i, col=1)
                low_legend_shown = True
        for series in (r.get("a_sim_models_low") or {}).values():
            if _has_visible_values(series):
                fig.add_trace(_low_speed_trace(
                    r["t"], series, dash="dot", showlegend=not low_legend_shown,
                ), row=i, col=1)
                low_legend_shown = True

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
        title=f"dataset 横断 縦方向モデルフィット（最良・最悪 計 {n} 件、低速/停車区間はグレー表示）<br><sup>{slope_note}</sup>",
        height=300 * n, margin=dict(t=80, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )


def build_fig_cross_long_tau_pointwise(rows_data: list[dict], cross_fit: dict) -> go.Figure:
    """縦方向 τ_a の瞬時推定値（隣接 2 サンプル差分からの逆算, tau-t グラフ）。

    離散更新式 a[k]=(1-α)a[k-1]+αu[k] を τ_a について解いた
    τ_a[k] = Δt(u[k]-a[k-1])/(a[k]-a[k-1]) を各時刻でプロットし、
    横断最小二乗法フィット値 (横線) と比較する。
    """
    rows_with_tau = [r for r in rows_data if r.get("tau_pointwise") is not None]
    if not rows_with_tau:
        return _placeholder_fig("瞬時 τ_a 推定不可（横断フィット失敗、またはデータ不足）")

    n = len(rows_with_tau)
    fig = make_grid(
        rows=n, cols=1, subplot_titles=[r["label"] for r in rows_with_tau], vertical_spacing=0.08,
    )
    tau_fit = cross_fit.get("tau", float("nan"))
    show_legend = True
    for i, r in enumerate(rows_with_tau, 1):
        fig.add_trace(go.Scatter(
            x=r["t"], y=r["tau_pointwise"], mode="markers",
            marker=dict(color="steelblue", size=4, opacity=0.4),
            name="瞬時 τ_a 推定値（隣接サンプル差分から逆算）",
            showlegend=show_legend,
        ), row=i, col=1)
        if np.isfinite(tau_fit):
            fig.add_hline(
                y=tau_fit, line=dict(color="crimson", width=2, dash="dash"),
                row=i, col=1,
            )
        fig.update_yaxes(title_text="τ_a [s]", range=[0, 2.0], row=i, col=1)
        show_legend = False
    fig.update_xaxes(title_text="時刻 [s]", row=n, col=1)

    fit_note = f"横断最小二乗法フィット値 τ_a={tau_fit:.3f}s（赤破線）" if np.isfinite(tau_fit) else "横断フィット失敗"
    return apply_base_layout(
        fig,
        title=f"縦方向 τ_a の瞬時推定値（点ごとの逆算 vs 最小二乗法フィット、最良・最悪 計 {n} 件）<br><sup>{fit_note}</sup>",
        height=260 * n, margin=dict(t=80, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )


def build_fig_long_tau_pointwise_hist(cross_fit: dict) -> go.Figure:
    """縦方向 τ_a の瞬時推定値の分布（横断フィットに使用した全データセットをプールしたヒストグラム）。

    `fit_long_cross_dataset_bounded` が横断フィットに使った pooled データセット全件
    （best/worst の代表 2〜4 件だけでなく、n_dyn 上位 n_top 件全て）について、
    最適無駄時間固定で計算した瞬時 τ_a[k] 推定値 (`tau_pointwise_all`) の分布を示す。
    """
    values = cross_fit.get("tau_pointwise_all") or []
    if not values:
        return _placeholder_fig("瞬時 τ_a 推定不可（横断フィット失敗、またはデータ不足）")

    arr = np.asarray(values, dtype=float)
    tau_fit = cross_fit.get("tau", float("nan"))
    median = float(np.median(arr))

    fig = go.Figure()
    fig.add_trace(go.Histogram(
        x=arr.tolist(), nbinsx=60, marker_color="steelblue", opacity=0.75,
        name=f"瞬時 τ_a 推定値（n={len(arr)}）",
    ))
    if np.isfinite(tau_fit):
        fig.add_vline(
            x=tau_fit, line=dict(color="crimson", width=2, dash="dash"),
            annotation_text=f"横断LSフィット値 {tau_fit:.3f}s", annotation_position="top right",
        )
    fig.add_vline(
        x=median, line=dict(color="darkorange", width=1.5, dash="dot"),
        annotation_text=f"中央値 {median:.3f}s", annotation_position="top left",
    )
    fig.update_xaxes(title_text="瞬時 τ_a 推定値 [s]", range=[0, 2.0])
    fig.update_yaxes(title_text="サンプル数")
    n_ds = cross_fit.get("n_datasets", 0)
    return apply_base_layout(
        fig,
        title=f"縦方向 τ_a の瞬時推定値の分布（横断フィット使用 {n_ds} データセットをプール、計 {len(arr)} 点）",
        height=380,
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


# ---------------------------------------------------------------------------
# 理想追従評価プロット (physical_validity_report.py の描画ロジックを純関数化)
# ---------------------------------------------------------------------------
def build_fig_perfect_tracking_box(data: dict) -> go.Figure:
    """操舵理想追従のホライズン別誤差 Box Plot。"""
    fig = go.Figure()
    keys = ["10", "20", "50", "100"]
    h_labels = data.get("h_labels", ["0.10s", "0.20s", "0.50s", "1.00s"])
    per_h_errors = data.get("per_h_errors") or {}

    for k, hl in zip(keys, h_labels):
        errs = per_h_errors.get(k)
        if errs:
            fig.add_trace(go.Box(
                y=[e * 100 for e in errs],
                name=hl,
                boxpoints="outliers",
                marker_size=3,
                marker_color="steelblue",
            ))
    if not any(per_h_errors.get(k) for k in keys):
        return _placeholder_fig("操舵理想追従データなし")

    fig.update_layout(
        title=f"理想追従 N-step 横方向誤差（上位 {data.get('n_dataset', 0)} データセット プール）",
        xaxis_title="ホライズン [s]",
        yaxis_title="|横方向誤差| [cm]",
        height=400,
        showlegend=False,
        margin=dict(t=50, b=40),
    )
    return apply_base_layout(fig, title=fig.layout.title.text, height=400)


def build_fig_perfect_tracking_traj(
    data: dict,
    labels: dict | None = None,
    height: int = 380,
) -> go.Figure:
    """代表データセットの軌跡比較プロット。

    labels: 表示文言の上書き (省略時は従来どおり)。
      キー: "title", "gt_name", "model_name", "x_title", "y_title"
    """
    lb = labels or {}
    traj_data = data.get("traj_data") or []
    n_traj = len(traj_data)
    if n_traj == 0:
        return _placeholder_fig("軌跡データなし")

    fig = make_subplots(
        rows=1, cols=n_traj,
        subplot_titles=[f"データセット {d['uuid']}" for d in traj_data],
        horizontal_spacing=0.08,
    )
    for i, td in enumerate(traj_data, start=1):
        m = td["moving"]
        gt_xm = [x if is_m else None for x, is_m in zip(td["gt_x"], m)]
        gt_ym = [y if is_m else None for y, is_m in zip(td["gt_y"], m)]
        bxm   = [x if is_m else None for x, is_m in zip(td["bx"], m)]
        bym   = [y if is_m else None for y, is_m in zip(td["by"], m)]

        show_legend = (i == 1)
        fig.add_trace(go.Scatter(
            x=gt_xm, y=gt_ym,
            mode="lines", name=lb.get("gt_name", "GT 軌跡"),
            line=dict(color="black", width=2),
            legendgroup="gt", showlegend=show_legend,
        ), row=1, col=i)
        fig.add_trace(go.Scatter(
            x=bxm, y=bym,
            mode="lines", name=lb.get("model_name", "自転車モデル (理想追従)"),
            line=dict(color="royalblue", width=1.5, dash="dash"),
            legendgroup="model", showlegend=show_legend,
        ), row=1, col=i)
        fig.update_xaxes(title_text=lb.get("x_title", "x [m]"), row=1, col=i)
        fig.update_yaxes(title_text=lb.get("y_title", "y [m]"), scaleanchor=f"x{i}", scaleratio=1, row=1, col=i)

    fig.update_layout(
        title=lb.get("title", "代表データセットの軌跡比較（GT vs 自転車モデル）"),
        height=height,
        legend=dict(orientation="h", y=1.1, x=0),
    )
    return apply_base_layout(fig, title=fig.layout.title.text, height=height)


def build_fig_long_perf_box(data: dict, title: str | None = None) -> go.Figure:
    """縦方向理想追従誤差の Box Plot。title 指定でタイトルを上書きできる。"""
    fig = go.Figure()
    keys = ["10", "20", "50", "100"]
    h_labels = data.get("h_labels", ["0.10s", "0.20s", "0.50s", "1.00s"])
    per_h_errors = data.get("per_h_errors") or {}

    for k, hl in zip(keys, h_labels):
        errs = per_h_errors.get(k)
        if errs:
            fig.add_trace(go.Box(
                y=[e * 100 for e in errs], name=hl,
                boxpoints="outliers", marker_size=3, marker_color="darkorange",
            ))
    if not any(per_h_errors.get(k) for k in keys):
        return _placeholder_fig("縦方向理想追従データなし")

    fig.update_layout(
        title=title or f"縦方向 モデル構造限界評価（a_act 直接入力 vs GT 変位、上位 {data.get('n_dataset', 0)} データセット）",
        xaxis_title="ホライズン [s]",
        yaxis_title="|縦方向誤差| [cm]",
        height=400,
        showlegend=False,
        margin=dict(t=60, b=40),
    )
    return apply_base_layout(fig, title=fig.layout.title.text, height=400)


def build_fig_long_perf_growth(data: dict, labels: dict | None = None) -> go.Figure:
    """縦方向のドリフト成長カーブ。

    labels: 表示文言の上書き (省略時は従来どおり)。
      キー: "title", "subplot_titles" (2 要素), "y_titles" (2 要素)
    """
    lb = labels or {}
    verr_pool = data.get("verr_pool")
    serr_pool = data.get("serr_pool")
    phase_pool = data.get("phase_pool")

    if not verr_pool:
        return _placeholder_fig("縦方向ドリフトプロファイルなし")

    verr = np.array(verr_pool)
    serr = np.array(serr_pool)
    phase = np.array(phase_pool)

    _H_MAX = verr.shape[1] - 1
    _FIT_DT = 0.01
    t_axis = np.arange(_H_MAX + 1) * _FIT_DT

    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=lb.get("subplot_titles") or ["速度誤差  vx_sim − GT vx  [m/s]",
                                                    "変位誤差  s_sim − s_gt  [cm]"],
        horizontal_spacing=0.10,
    )

    _PHASE_COLORS = {0: "steelblue", 1: "gray", 2: "tomato"}
    _PHASE_NAMES  = {0: "減速 (a < -0.3)", 1: "巡航", 2: "加速 (a > +0.3)"}

    y_titles = lb.get("y_titles") or [
        "速度誤差 [m/s]<br><sup>正 = sim が GT より速い</sup>",
        "変位誤差 [cm]<br><sup>正 = sim が GT より進んでいる</sup>",
    ]
    for col, (pool, scale, ytitle) in enumerate([
        (verr, 1.0,   y_titles[0]),
        (serr, 100.0, y_titles[1]),
    ], start=1):
        data_scaled = pool * scale
        med  = np.median(data_scaled, axis=0)
        q25  = np.percentile(data_scaled, 25, axis=0)
        q75  = np.percentile(data_scaled, 75, axis=0)

        # IQR 帯
        fig.add_trace(go.Scatter(
            x=np.concatenate([t_axis, t_axis[::-1]]).tolist(),
            y=np.concatenate([q75, q25[::-1]]).tolist(),
            fill="toself", fillcolor="rgba(255,165,0,0.18)",
            line=dict(color="rgba(0,0,0,0)"),
            name="IQR 25–75%", showlegend=(col == 1),
            legendgroup="iqr",
        ), row=1, col=col)

        # 全体中央値
        fig.add_trace(go.Scatter(
            x=t_axis.tolist(), y=med.tolist(),
            mode="lines", line=dict(color="darkorange", width=2.5),
            name="中央値（全体）", showlegend=(col == 1),
            legendgroup="med_all",
        ), row=1, col=col)

        # 局面別中央値
        for ph_id, ph_name in _PHASE_NAMES.items():
            mask = phase == ph_id
            if mask.sum() < 5:
                continue
            ph_med = np.median(data_scaled[mask], axis=0)
            fig.add_trace(go.Scatter(
                x=t_axis.tolist(), y=ph_med.tolist(),
                mode="lines",
                line=dict(color=_PHASE_COLORS[ph_id], width=1.2, dash="dot"),
                name=ph_name, showlegend=(col == 1),
                legendgroup=f"ph{ph_id}",
            ), row=1, col=col)

        fig.add_hline(
            y=0, line=dict(color="black", width=1, dash="dash"),
            row=1, col=col,
        )
        fig.update_yaxes(title_text=ytitle, row=1, col=col)
        fig.update_xaxes(title_text="ロールアウト経過時間 [s]", row=1, col=col)

    fig.update_layout(
        title=lb.get("title") or f"縦方向ドリフト成長カーブ（符号付き、上位 {data.get('n_dataset', 0)} データセット）",
        height=430,
        margin=dict(t=70, b=50),
        legend=dict(orientation="v", x=1.02, y=1),
    )
    return apply_base_layout(fig, title=fig.layout.title.text, height=430)


def build_fig_long_perf_map(
    data: dict, map_ways: list | None = None, title: str | None = None,
) -> go.Figure:
    """縦方向変位誤差の地図上分布プロット。title 指定でタイトルを上書きできる。"""
    map_x = data.get("map_x")
    map_y = data.get("map_y")
    map_serr = data.get("map_serr")

    if not map_x:
        return _placeholder_fig("位置データなし")

    x_arr = np.array(map_x)
    y_arr = np.array(map_y)
    serr_arr = np.array(map_serr) * 100.0

    _MAP_MARGIN = 10.0
    x_min = float(x_arr.min()) - _MAP_MARGIN
    x_max = float(x_arr.max()) + _MAP_MARGIN
    y_min = float(y_arr.min()) - _MAP_MARGIN
    y_max = float(y_arr.max()) + _MAP_MARGIN
    vmax  = max(float(np.percentile(np.abs(serr_arr), 99)), 1e-6)

    fig = go.Figure()

    if map_ways is not None:
        lane_ways = map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max))
        if lane_ways:
            fig.add_trace(lanes_to_trace(lane_ways))

    fig.add_trace(go.Scatter(
        x=x_arr.tolist(), y=y_arr.tolist(),
        mode="markers",
        marker=dict(
            color=serr_arr.tolist(),
            colorscale="RdBu",
            reversescale=True,
            cmin=-vmax, cmax=vmax,
            size=4,
            colorbar=dict(title="cm", thickness=14),
        ),
        showlegend=False,
        hovertemplate=(
            "x=%{x:.1f}m y=%{y:.1f}m<br>"
            "変位誤差=%{marker.color:.2f}cm"
            "<extra></extra>"
        ),
    ))
    fig.update_xaxes(title_text="x [m]", range=[x_min, x_max])
    fig.update_yaxes(
        title_text="y [m]", range=[y_min, y_max],
        scaleanchor="x", scaleratio=1,
    )
    fig.update_layout(
        title=title or "縦方向変位誤差の地図分布（1.0s 終端・rollout 開始点）",
        height=600,
        margin=dict(t=70, b=50, r=80),
    )
    return apply_base_layout(fig, title=fig.layout.title.text, height=600)
