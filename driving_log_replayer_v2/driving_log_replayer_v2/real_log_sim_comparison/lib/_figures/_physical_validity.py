"""物理妥当性検証 (縦方向 / 操舵 / 横方向 k_us) の図スペック (ROS 非依存・純関数).

per-dataset (step6_analyze_cases) の単一データセット図と、collection 横断
(step13_cross_dataset) の最長連続データセット + 横断フィット重ね描き図の両方を提供する。
データ整形は `lib._physical_validity` (ROS 依存の MCAP 読み込みを含む) が担当し、
本モジュールは計算済み dict/list を受けて `go.Figure` を組むだけの純関数。
"""

from __future__ import annotations

import math

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

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


def _series_or(row: dict, *keys: str) -> list | None:
    for key in keys:
        value = row.get(key)
        if value is not None:
            return value
    return None


def _mask_or(row: dict, *keys: str) -> list | None:
    for key in keys:
        value = row.get(key)
        if value is not None:
            return value
    return None


def _add_inactive_bands(fig: go.Figure, t: list, mask: list | None, rows: list[int]) -> None:
    """mask=False の区間を薄い背景として描く。"""
    if mask is None or not t:
        return
    t_arr = np.asarray(t, dtype=float)
    mask_arr = np.asarray(mask, dtype=bool)
    if len(t_arr) != len(mask_arr):
        return
    inactive = ~mask_arr
    if not inactive.any():
        return
    changes = np.flatnonzero(np.diff(inactive.astype(np.int8)) != 0) + 1
    bounds = np.concatenate([[0], changes, [len(inactive)]])
    n_added = 0
    for start, end in zip(bounds[:-1], bounds[1:]):
        if not inactive[start]:
            continue
        if n_added >= 120:
            break
        x0 = float(t_arr[start])
        x1 = float(t_arr[end - 1])
        for row in rows:
            fig.add_vrect(
                x0=x0, x1=x1, row=row, col=1,
                fillcolor="lightgray", opacity=0.28,
                line_width=0, layer="below",
            )
        n_added += 1


def _mask_area_trace(t: list, mask: list | None, vx: list | None, *, showlegend: bool) -> go.Scatter | None:
    if mask is None or vx is None:
        return None
    mask_arr = np.asarray(mask, dtype=bool)
    vx_arr = np.asarray(vx, dtype=float)
    if len(mask_arr) != len(vx_arr):
        return None
    ymax = float(np.nanmax(vx_arr)) if np.isfinite(vx_arr).any() else 1.0
    y = np.where(mask_arr, 0.18 * max(ymax, 1.0), np.nan)
    return go.Scatter(
        x=t, y=y.tolist(), mode="lines", fill="tozeroy",
        line=dict(color="rgba(120,120,120,0.0)", width=0),
        fillcolor="rgba(120,120,120,0.22)",
        name="同定対象区間", legendgroup="fit_mask", legendrank=910,
        showlegend=showlegend,
        hovertemplate="同定対象区間<br>t=%{x:.2f}s<extra></extra>",
    )


def _model_color_map(names: list[str]) -> dict[str, str]:
    if not names:
        return {}
    base = ["darkorange", "seagreen", "mediumpurple", "indianred"]
    if len(names) <= len(base):
        colors = base[:len(names)]
    else:
        colors = base + qualitative_colors(len(names) - len(base))
    return dict(zip(names, colors))


def _legend_entry(name: str, group: str, showlegend: bool, rank: int) -> dict:
    return {
        "name": name,
        "legendgroup": group,
        "legendrank": rank,
        "showlegend": showlegend,
    }


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
    title: str | None = None,
) -> go.Figure:
    """スカラー k_us 最小二乗法推定 (単一データセット、モデル別チューニング値重ね描き)。

    title: 図タイトルの上書き (None で既定の単一データセット向けタイトル)。
    """
    if bins is None:
        return _placeholder_fig("横方向データ不足 (曲線走行サンプル不足)")
    return _build_fig_kus(
        bins, models or {},
        title=title or "k_us 独立同定（最小二乗法回帰・スカラー、単一データセット）",
    )


# ---------------------------------------------------------------------------
# collection 横断: k_us プール再構成 + モデル別チューン値重ね描き
# ---------------------------------------------------------------------------
def build_fig_cross_kus(bins: dict | None, models: dict | None) -> go.Figure:
    """dataset 横断 スカラー k_us 独立同定 (十分統計量プール集計 + モデル別チューニング値比較)。"""
    if bins is None:
        return _placeholder_fig("横方向データ不足 (全データセットで曲線走行サンプル不足)")
    model_params = {name: spec.params for name, spec in (models or {}).items()}
    return _build_fig_kus(
        bins, model_params,
        title="dataset 横断 スカラー k_us 独立同定（最小二乗法プール集計）",
    )


def _build_fig_kus(
    bins: dict,
    model_params: dict[str, dict],
    *,
    title: str,
) -> go.Figure:
    fig = go.Figure()

    k_us = float(bins.get("k_us", float("nan")))
    n_pts = int(bins.get("n_pts", 0))
    kus_p25 = float(bins.get("kus_p25", float("nan")))
    kus_p75 = float(bins.get("kus_p75", float("nan")))
    vx = [0.0, 12.0]

    # IQR 帯（個別サンプル分布）。k_us はスカラーなので速度域全体で水平な帯として描く。
    if np.isfinite(kus_p25) and np.isfinite(kus_p75):
        fig.add_trace(go.Scatter(
            x=vx + vx[::-1], y=[kus_p25, kus_p25, kus_p75, kus_p75],
            fill="toself", fillcolor="rgba(70,130,180,0.15)",
            line=dict(color="rgba(255,255,255,0)"),
            showlegend=True, name="25–75%ile（個別サンプル）",
        ))

    # 同定スカラー k_us（水平線）
    if np.isfinite(k_us):
        fig.add_trace(go.Scatter(
            x=vx, y=[k_us, k_us], mode="lines",
            line=dict(color="steelblue", width=2.5),
            name=f"最小二乗法推定 k_us={k_us:.4f}（n={n_pts}）",
        ))

    # モデル別チューニング値（水平破線）
    colors = qualitative_colors(len(model_params)) if model_params else []
    for (name, params), color in zip(model_params.items(), colors):
        kv = float(params.get("k_us", 0.0))
        fig.add_trace(go.Scatter(
            x=vx, y=[kv, kv], mode="lines",
            line=dict(color=color, width=2.0, dash="dash"),
            name=f"{name}: k_us={kv:.4f}",
        ))

    fig.add_hline(y=0.0, line=dict(color="gray", width=1, dash="dot"))
    fig.update_xaxes(title_text="車速 vx [m/s]", range=[0, 12])
    fig.update_yaxes(title_text="k_us [rad·s²/m]", range=[-0.05, 0.12])
    return apply_base_layout(
        fig, title=title, height=430,
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
    )


# ---------------------------------------------------------------------------
# collection 横断: 最長連続時系列 + 横断フィット + モデル別チューン値
# ---------------------------------------------------------------------------
def build_fig_cross_long(rows_data: list[dict], cross_fit: dict) -> go.Figure:
    """dataset 横断 縦方向モデルフィット (最長連続時系列 + 横断最小二乗法同定値)。"""
    if not rows_data:
        return _placeholder_fig("MCAP 読み込み失敗（最長連続データセットが見つかりません）")

    n = len(rows_data)
    n_datasets = sum(len(r.get("dataset_ids", [r.get("dataset_id")])) for r in rows_data)
    all_model_names: list[str] = []
    for r in rows_data:
        for name in (r.get("a_sim_models_raw") or r.get("a_sim_models") or {}):
            if name not in all_model_names:
                all_model_names.append(name)
    color_of = _model_color_map(all_model_names)

    total_rows = 3 * n
    titles: list[str] = []
    row_heights: list[float] = []
    for r in rows_data:
        titles.extend([r["label"], "", ""])
        row_heights.extend([0.48, 0.32, 0.20])
    fig = make_subplots(
        rows=total_rows, cols=1,
        subplot_titles=titles,
        shared_xaxes=False,
        vertical_spacing=0.025,
        row_heights=row_heights,
    )
    show_legend = True
    mask_legend_shown = False
    for i, r in enumerate(rows_data):
        row_main = 3 * i + 1
        row_diag = row_main + 1
        row_vx = row_main + 2
        t = r["t"]
        active_mask = _mask_or(r, "mask_dyn", "moving")
        _add_inactive_bands(fig, t, active_mask, [row_main, row_diag, row_vx])

        fig.add_trace(go.Scatter(
            x=t, y=_series_or(r, "a_act_raw", "a_act"),
            line=dict(color="black", width=1.5), connectgaps=False,
            **_legend_entry("実測: a_report / dot_a_real", "long_actual", show_legend, 10),
        ), row=row_main, col=1)
        fig.add_trace(go.Scatter(
            x=t, y=_series_or(r, "a_cmd_raw", "a_cmd"),
            line=dict(color="seagreen", width=1.1, dash="dash"), connectgaps=False,
            **_legend_entry("指令: a_cmd / dot_a_cmd", "long_cmd", show_legend, 20),
        ), row=row_main, col=1)
        if _series_or(r, "a_sim_cross_raw", "a_sim_cross") is not None:
            label = (
                f"横断同定: a_sim / dot_a_sim "
                f"(τ={cross_fit['tau']:.3f}s, T={cross_fit['delay']:.3f}s)"
            )
            fig.add_trace(go.Scatter(
                x=t, y=_series_or(r, "a_sim_cross_raw", "a_sim_cross"),
                line=dict(color="royalblue", width=2.0), connectgaps=False,
                **_legend_entry(label, "long_cross_fit", show_legend, 30),
            ), row=row_main, col=1)
        for name, series in (r.get("a_sim_models_raw") or r.get("a_sim_models") or {}).items():
            group = f"long_model_{name}"
            fig.add_trace(go.Scatter(
                x=t, y=series,
                line=dict(color=color_of[name], width=1.3, dash="dot"),
                connectgaps=False,
                **_legend_entry(f"{name}: a_sim / dot_a_sim (チューニング値)", group, show_legend, 40),
            ), row=row_main, col=1)

        if r.get("dot_a_act") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["dot_a_act"],
                line=dict(color="black", width=1.2), connectgaps=False,
                **_legend_entry("実測 dot_a_real", "long_actual", False, 10),
            ), row=row_diag, col=1)
        if r.get("dot_a_cmd") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["dot_a_cmd"],
                line=dict(color="seagreen", width=0.9, dash="dash"), connectgaps=False,
                **_legend_entry("指令 dot_a_cmd", "long_cmd", False, 20),
            ), row=row_diag, col=1)
        if r.get("dot_a_sim_cross") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["dot_a_sim_cross"],
                line=dict(color="royalblue", width=1.3), connectgaps=False,
                **_legend_entry("横断同定 dot_a_sim", "long_cross_fit", False, 30),
            ), row=row_diag, col=1)
        for name, series in (r.get("dot_a_sim_models") or {}).items():
            fig.add_trace(go.Scatter(
                x=t, y=series,
                line=dict(color=color_of.get(name, "darkorange"), width=1.0, dash="dot"),
                connectgaps=False,
                **_legend_entry(f"{name} dot_a_sim", f"long_model_{name}", False, 40),
            ), row=row_diag, col=1)
        fig.add_hline(y=0.0, line=dict(color="gray", width=0.7, dash="dot"), row=row_diag, col=1)

        if r.get("vx") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["vx"],
                line=dict(color="mediumpurple", width=1.2), connectgaps=False,
                **_legend_entry("車速 v_x", "speed", show_legend, 900),
            ), row=row_vx, col=1)
        mask_trace = _mask_area_trace(t, active_mask, r.get("vx"), showlegend=show_legend and not mask_legend_shown)
        if mask_trace is not None:
            fig.add_trace(mask_trace, row=row_vx, col=1)
            mask_legend_shown = True
        show_legend = False
        fig.update_yaxes(title_text="a [m/s²]", row=row_main, col=1)
        fig.update_yaxes(title_text="dot_a [m/s³]", row=row_diag, col=1)
        fig.update_yaxes(title_text="vx [m/s]", row=row_vx, col=1)
        fig.update_xaxes(title_text="時刻 [s]", row=row_vx, col=1)

    p_min_deg = math.degrees(cross_fit.get("pitch_min", 0.0))
    p_max_deg = math.degrees(cross_fit.get("pitch_max", 0.0))
    slope_note = f"横断フィット使用データセット数={cross_fit.get('n_datasets', 0)}、pitch range {p_min_deg:+.2f}°〜{p_max_deg:+.2f}°"
    return apply_base_layout(
        fig,
        title=(
            f"dataset 横断 縦方向モデルフィット（選択 {n_datasets} データセットを "
            f"{n} 最長連続区間で表示、同定対象外区間はグレー背景）<br><sup>{slope_note}</sup>"
        ),
        height=620 * n, margin=dict(t=90, b=40),
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
    n_datasets = sum(len(r.get("dataset_ids", [r.get("dataset_id")])) for r in rows_with_tau)
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
        title=(
            f"縦方向 τ_a の瞬時推定値（選択 {n_datasets} データセットを {n} 連続区間で表示）"
            f"<br><sup>{fit_note}</sup>"
        ),
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


def build_fig_equation_residual_hist(
    resid_samples,
    *,
    channel_label: str,
    unit_label: str,
    rmse_median: float = float("nan"),
) -> go.Figure:
    """同定済みパラメータでの ODE 方程式残差 E[k]=RHS−LHS の分布（診断・方針D）。

    パラメータは出力誤差型で推定した値を用い、その解が状態微分方程式
    `dot_x = RHS(θ)` をどれだけ満たすかを示す整合診断。0 中心・低分散なら良同定。
    目的関数ではない（縦・操舵・ヨーの 3 式をこの共通の残差形 E=RHS−LHS で統一提示する）。
    """
    arr = np.asarray(list(resid_samples), dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return _placeholder_fig(f"{channel_label}: 方程式残差の評価不可（同定失敗またはデータ不足）")

    mean = float(np.mean(arr))
    lo, hi = (float(x) for x in np.percentile(arr, [1, 99]))
    if not (hi > lo):  # 退化時のフォールバック
        lo, hi = float(arr.min()), float(arr.max() + 1e-9)

    # 全データセットをプールすると生サンプルは数十万点になるため、HTML には生値を埋め込まず
    # np.histogram で事前ビニングした棒 (go.Bar) にする (見た目は同じ・埋め込みサイズは一定)。
    counts, edges = np.histogram(arr, bins=80, range=(lo, hi))
    centers = (edges[:-1] + edges[1:]) / 2.0
    fig = go.Figure()
    fig.add_trace(go.Bar(
        x=centers.tolist(), y=counts.tolist(), width=float(edges[1] - edges[0]),
        marker_color="teal", opacity=0.75, name=f"残差 E[k]（n={arr.size}）",
    ))
    fig.add_vline(x=0.0, line=dict(color="black", width=1.5))
    fig.add_vline(
        x=mean, line=dict(color="crimson", width=1.5, dash="dot"),
        annotation_text=f"平均 {mean:.3g}", annotation_position="top right",
    )
    fig.update_xaxes(title_text=f"方程式残差 E[k] = RHS − LHS  [{unit_label}]", range=[lo, hi])
    fig.update_yaxes(title_text="サンプル数")
    extra = f"、残差 RMSE 中央値 {rmse_median:.3g} {unit_label}" if np.isfinite(rmse_median) else ""
    return apply_base_layout(
        fig,
        title=f"{channel_label}: 同定パラメータでの方程式残差 E[k] の分布（整合診断{extra}）",
        height=360,
    )


_NSTEP_ERROR_METRICS = [
    ("p14_yaw", "yaw [deg]", "steelblue"),
    ("p14_lat", "lat [cm]", "darkorange"),
    ("p14_long", "long [cm]", "seagreen"),
    ("p14_vx", "vx [m/s]", "indianred"),
]
_NSTEP_ERROR_QUANTILES = [
    (0.50, "中央値", "#555555"),
    (0.90, "90%ile", "#1f77b4"),
    (0.95, "95%ile", "#ff7f0e"),
    (0.99, "99%ile", "#d62728"),
]


def build_fig_nstep_error_hist(df: pd.DataFrame, label: str = "tuned") -> go.Figure:
    """N-step Open Loop評価: 終端誤差 (yaw/lat/long/vx) の horizon 別分布ヒストグラム（{label} モデルのみ）。

    サマリテーブルの 平均/99%ile だけでは裾の形（二峰性・突発外れ値の割合）が分からないため、
    データセット横断 RMSE の分布そのものを可視化し、50/90/95/99パーセンタイルを縦線で重ねる。
    """
    if df is None or len(df) == 0:
        return _placeholder_fig("N-step Open Loop評価データなし")

    horizons = sorted(df["h"].unique().tolist())
    if not horizons:
        return _placeholder_fig("N-step Open Loop評価データなし")

    fig = make_subplots(
        rows=len(horizons), cols=len(_NSTEP_ERROR_METRICS),
        subplot_titles=[f"N={h}  {mlabel}" for h in horizons for _, mlabel, _ in _NSTEP_ERROR_METRICS],
        vertical_spacing=0.05, horizontal_spacing=0.04,
    )
    for ri, h in enumerate(horizons, start=1):
        sub = df[df["h"] == h]
        for ci, (col, mlabel, color) in enumerate(_NSTEP_ERROR_METRICS, start=1):
            vals = sub[col].dropna().to_numpy(dtype=float) if col in sub else np.array([])
            if len(vals) == 0:
                continue
            fig.add_trace(go.Histogram(
                x=vals.tolist(), nbinsx=30, marker_color=color, opacity=0.75,
                showlegend=False, name=mlabel,
            ), row=ri, col=ci)
            for q, _qname, qcolor in _NSTEP_ERROR_QUANTILES:
                qv = float(np.quantile(vals, q))
                fig.add_vline(
                    x=qv, row=ri, col=ci,
                    line=dict(color=qcolor, width=1.3, dash="dot" if q < 0.99 else "dash"),
                )
        for ci, (_col, mlabel, _color) in enumerate(_NSTEP_ERROR_METRICS, start=1):
            if ri == len(horizons):
                fig.update_xaxes(title_text=mlabel, row=ri, col=ci)
    fig.update_yaxes(title_text="データセット数", col=1)

    # 凡例用ダミートレース (縦線の色 = パーセンタイルの対応を示すだけで描画には寄与しない)
    for q, qname, qcolor in _NSTEP_ERROR_QUANTILES:
        fig.add_trace(go.Scatter(
            x=[None], y=[None], mode="lines",
            line=dict(color=qcolor, width=1.3, dash="dot" if q < 0.99 else "dash"),
            name=qname, showlegend=True,
        ), row=1, col=1)

    return apply_base_layout(
        fig,
        title=f"N-step Open Loop評価: 終端誤差の分布（{label}、データセット横断 RMSE のヒストグラム）",
        height=280 * len(horizons) + 40,
        legend=dict(orientation="h", y=1.02, x=0),
    )


_NSTEP_GROWTH_METRICS = [
    ("yaw", "yaw [deg]"),
    ("lat", "lat [cm]"),
    ("long", "long [cm]"),
    ("vx", "vx [m/s]"),
]


def build_fig_nstep_error_growth(
    df: pd.DataFrame, label: str = "tuned", baseline_label: str = "baseline"
) -> go.Figure:
    """N-step Open Loop評価: 終端誤差 (yaw/lat/long/vx) の horizon 別ドリフト成長カーブ。

    ヒストグラムが各 horizon の分布（裾の形）を見るのに対し、こちらは横軸 = horizon N（ステップ数）で
    データセット横断 RMSE の中央値 + IQR 帯を描き、誤差がホライズンとともにどう成長するかを
    tuned と baseline で比較する。データは build_fig_nstep_error_hist と同じ df を消費する。
    """
    if df is None or len(df) == 0:
        return _placeholder_fig("N-step Open Loop評価データなし")
    horizons = sorted(df["h"].unique().tolist())
    if not horizons:
        return _placeholder_fig("N-step Open Loop評価データなし")

    fig = make_subplots(
        rows=1, cols=len(_NSTEP_GROWTH_METRICS),
        subplot_titles=[mlabel for _, mlabel in _NSTEP_GROWTH_METRICS],
        horizontal_spacing=0.06,
    )
    # (列プレフィックス, 凡例名, 線色, IQR 帯色)
    series = [
        ("p14", label, "darkorange", "rgba(255,165,0,0.18)"),
        ("bl", baseline_label, "steelblue", "rgba(70,130,180,0.15)"),
    ]
    hx = np.array(horizons, dtype=float)
    for ci, (metric, mlabel) in enumerate(_NSTEP_GROWTH_METRICS, start=1):
        for prefix, slabel, color, fillcolor in series:
            col = f"{prefix}_{metric}"
            if col not in df:
                continue
            med, q25, q75 = [], [], []
            for h in horizons:
                vals = df[df["h"] == h][col].dropna().to_numpy(dtype=float)
                if len(vals) == 0:
                    med.append(np.nan); q25.append(np.nan); q75.append(np.nan)
                    continue
                med.append(float(np.median(vals)))
                q25.append(float(np.percentile(vals, 25)))
                q75.append(float(np.percentile(vals, 75)))
            med_a, q25_a, q75_a = np.array(med), np.array(q25), np.array(q75)
            fig.add_trace(go.Scatter(
                x=np.concatenate([hx, hx[::-1]]).tolist(),
                y=np.concatenate([q75_a, q25_a[::-1]]).tolist(),
                fill="toself", fillcolor=fillcolor,
                line=dict(color="rgba(0,0,0,0)"),
                name=f"{slabel} IQR", legendgroup=prefix,
                showlegend=False, hoverinfo="skip",
            ), row=1, col=ci)
            fig.add_trace(go.Scatter(
                x=hx.tolist(), y=med_a.tolist(),
                mode="lines+markers", line=dict(color=color, width=2.2),
                name=slabel, legendgroup=prefix, showlegend=(ci == 1),
            ), row=1, col=ci)
        fig.update_xaxes(title_text="horizon N [step]", row=1, col=ci)
        fig.update_yaxes(title_text=mlabel, row=1, col=ci)

    return apply_base_layout(
        fig,
        title=f"N-step Open Loop評価: 終端誤差のドリフト成長カーブ（{label} vs {baseline_label}、中央値 + IQR）",
        height=380,
        legend=dict(orientation="h", y=1.10, x=0),
    )


def build_fig_cross_steer(rows_data: list[dict]) -> go.Figure:
    """dataset 横断 操舵モデルフィット (最長連続時系列 + per-dataset 同定値 + モデル別チューン値)。"""
    if not rows_data:
        return _placeholder_fig("MCAP 読み込み失敗（最長連続データセットが見つかりません）")

    n = len(rows_data)
    n_datasets = sum(len(r.get("dataset_ids", [r.get("dataset_id")])) for r in rows_data)
    all_model_names: list[str] = []
    for r in rows_data:
        for name in (r.get("d_sim_models_raw") or r.get("d_sim_models") or {}):
            if name not in all_model_names:
                all_model_names.append(name)
    color_of = _model_color_map(all_model_names)

    total_rows = 3 * n
    titles: list[str] = []
    row_heights: list[float] = []
    for r in rows_data:
        titles.extend([r["label"], "", ""])
        row_heights.extend([0.48, 0.32, 0.20])
    fig = make_subplots(
        rows=total_rows, cols=1,
        subplot_titles=titles,
        shared_xaxes=False,
        vertical_spacing=0.025,
        row_heights=row_heights,
    )
    show_legend = True
    mask_legend_shown = False
    for i, r in enumerate(rows_data):
        row_main = 3 * i + 1
        row_diag = row_main + 1
        row_vx = row_main + 2
        t = r["t"]
        active_mask = _mask_or(r, "mask_dyn", "moving")
        _add_inactive_bands(fig, t, active_mask, [row_main, row_diag, row_vx])

        if r.get("d_cmd") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["d_cmd"],
                line=dict(color="seagreen", width=1.1, dash="dash"),
                connectgaps=False,
                **_legend_entry("指令 δ_cmd", "steer_cmd", show_legend, 10),
            ), row=row_main, col=1)
        fig.add_trace(go.Scatter(
            x=t, y=_series_or(r, "d_act_raw", "d_act"),
            line=dict(color="black", width=1.5), connectgaps=False,
            **_legend_entry("実測: δ_act / dot_δ_act", "steer_actual", show_legend, 20),
        ), row=row_main, col=1)
        if _series_or(r, "d_sim_fit_raw", "d_sim_fit") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=_series_or(r, "d_sim_fit_raw", "d_sim_fit"),
                line=dict(color="royalblue", width=1.5), connectgaps=False,
                **_legend_entry("当該データセット同定: δ_sim / dot_δ_sim", "steer_fit", show_legend, 30),
            ), row=row_main, col=1)
        for name, series in (r.get("d_sim_models_raw") or r.get("d_sim_models") or {}).items():
            group = f"steer_model_{name}"
            fig.add_trace(go.Scatter(
                x=t, y=series,
                line=dict(color=color_of[name], width=1.3, dash="dot"),
                connectgaps=False,
                **_legend_entry(f"{name}: δ_sim / dot_δ_sim (チューニング値)", group, show_legend, 40),
            ), row=row_main, col=1)

        if r.get("dot_d_act") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["dot_d_act"],
                line=dict(color="black", width=1.2), connectgaps=False,
                **_legend_entry("実測 dot_δ_act", "steer_actual", False, 20),
            ), row=row_diag, col=1)
        if r.get("dot_d_sim_fit") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["dot_d_sim_fit"],
                line=dict(color="royalblue", width=1.2), connectgaps=False,
                **_legend_entry("当該データセット同定 dot_δ_sim", "steer_fit", False, 30),
            ), row=row_diag, col=1)
        if r.get("dot_d_open_fit") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["dot_d_open_fit"],
                line=dict(color="darkorange", width=1.2), connectgaps=False,
                **_legend_entry("当該データセット同定 RHS dot_δ", "steer_fit_rhs", show_legend, 35),
            ), row=row_diag, col=1)
        for name, series in (r.get("dot_d_sim_models") or {}).items():
            fig.add_trace(go.Scatter(
                x=t, y=series,
                line=dict(color=color_of.get(name, "darkorange"), width=1.0, dash="dot"),
                connectgaps=False,
                **_legend_entry(f"{name} dot_δ_sim", f"steer_model_{name}", False, 40),
            ), row=row_diag, col=1)
        fig.add_hline(y=0.0, line=dict(color="gray", width=0.7, dash="dot"), row=row_diag, col=1)

        if r.get("vx") is not None:
            fig.add_trace(go.Scatter(
                x=t, y=r["vx"],
                line=dict(color="mediumpurple", width=1.2), connectgaps=False,
                **_legend_entry("車速 v_x", "speed", show_legend, 900),
            ), row=row_vx, col=1)
        mask_trace = _mask_area_trace(t, active_mask, r.get("vx"), showlegend=show_legend and not mask_legend_shown)
        if mask_trace is not None:
            fig.add_trace(mask_trace, row=row_vx, col=1)
            mask_legend_shown = True
        show_legend = False
        fig.update_yaxes(title_text="δ [rad]", row=row_main, col=1)
        fig.update_yaxes(title_text="dot_delta [rad/s]", row=row_diag, col=1)
        fig.update_yaxes(title_text="vx [m/s]", row=row_vx, col=1)
        fig.update_xaxes(title_text="時刻 [s]", row=row_vx, col=1)
    return apply_base_layout(
        fig,
        title=f"dataset 横断 操舵モデルフィット（選択 {n_datasets} データセットを {n} 最長連続区間で表示、同定対象外区間はグレー背景）",
        height=620 * n, margin=dict(t=80, b=40),
        legend=dict(orientation="h", y=1.03, x=0),
    )


# ---------------------------------------------------------------------------
# 理想追従評価プロット (physical_validity_report.py の描画ロジックを純関数化)
# ---------------------------------------------------------------------------
def build_fig_perfect_tracking_box(data: dict) -> go.Figure:
    """操舵理想追従のホライズン別誤差 Box Plot。"""
    fig = go.Figure()
    keys = ["10", "20", "50", "100"]
    h_labels = data.get("h_labels", ["0.33s", "0.67s", "1.67s", "3.33s"])
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
        title=f"N-step Open Loop評価: 理想追従 横方向誤差（上位 {data.get('n_dataset', 0)} データセット プール）",
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
    h_labels = data.get("h_labels", ["0.33s", "0.67s", "1.67s", "3.33s"])
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
        title=title or f"縦方向 整合性評価（a_report 直接入力 vs GT 変位、上位 {data.get('n_dataset', 0)} データセット）",
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
    dt_s = float(data.get("dt_s", 1.0 / 30.0))
    t_axis = np.arange(_H_MAX + 1) * dt_s

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
