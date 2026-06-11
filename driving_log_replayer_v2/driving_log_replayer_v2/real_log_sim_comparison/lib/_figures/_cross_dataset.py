"""step13 (データセット横断分析) の図スペック (ROS 非依存・純関数).

モデル×データセット行列・正規化集約・走行特性カバレッジ・leave-one-out 安定性を
plotly Figure で組む。行列の欠損セル (DS によって run/case 集合が異なる) は NaN のまま
渡すこと — Heatmap は NaN セルを自動で空白にし、注釈側でスキップする。
"""

from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go

from ._common import apply_base_layout, make_grid, qualitative_colors


def _annotate_heatmap(fig, mat: np.ndarray, xlabels, ylabels, *, row: int, col: int,
                      fmt: str = "{:.2f}") -> None:
    """ヒートマップのセル数値注釈 (NaN セルはスキップ、背景濃淡で文字色切替)。

    座標はラベル文字列ではなく**カテゴリ index** (整数) で渡す。DS ラベルは "28443458" の
    ような純数字文字列になり得るため、文字列を渡すと category 軸でも数値として autorange に
    混入し軸 range が壊れる (tick 座標が NaN になる)。category 軸上の整数はカテゴリ index
    として解釈されるので安全。
    """
    finite = mat[np.isfinite(mat)]
    if len(finite) == 0:
        return
    thresh = finite.min() + (finite.max() - finite.min()) * 0.6
    for i in range(len(ylabels)):
        for j in range(len(xlabels)):
            v = mat[i, j]
            if not math.isfinite(v):
                continue
            fig.add_annotation(
                x=j, y=i, text=fmt.format(v), showarrow=False,
                font=dict(size=10, color="white" if v > thresh else "black"),
                row=row, col=col,
            )


def _matrix_height(n_rows: int) -> int:
    return 240 + 36 * max(n_rows, 1)


def _force_matrix_category_axes(fig: go.Figure) -> None:
    """行列図 (heatmap) の x/y 軸を category に明示する。

    DS ラベル (UUID 先頭 8 文字の hex) は "28443458" のように**純数字文字列**になり得る。
    plotly は軸 type を値から推論するため、数字文字列ラベルが数値軸と誤判定され
    tick/annotation の座標が NaN になる (SVG transform エラー)。行列図は常にカテゴリ軸。
    coverage / bar 図など数値軸が適切な図では呼ばない。
    """
    fig.update_xaxes(type="category")
    fig.update_yaxes(type="category")


def build_fig_cross_closed_loop_heatmap(
    ds_labels: list[str],
    run_tags: list[str],
    s2r_mean: np.ndarray,
    completion: np.ndarray,
) -> go.Figure | None:
    """closed-loop の DS × sim run 行列 (軌跡乖離 s2r 平均 [m] / 完走率 [%] の 2 パネル)。

    s2r_mean / completion: shape (len(ds_labels), len(run_tags))、欠損は NaN。
    """
    if not ds_labels or not run_tags:
        return None
    fig = make_grid(1, 2, subplot_titles=["sim→実機 平均乖離 [m]", "完走率 [%]"],
                    horizontal_spacing=0.13)
    panels = [
        (1, s2r_mean, "YlOrRd", "m", "{:.2f}", False),
        (2, completion, "RdYlGn", "%", "{:.0f}", True),  # 完走率は高いほど良い → 緑
    ]
    for c, mat, scale, unit, fmt, _ in panels:
        fig.add_trace(go.Heatmap(
            z=mat, x=run_tags, y=ds_labels, colorscale=scale, showscale=True,
            colorbar=dict(title=unit, x=(0.46 if c == 1 else 1.02), len=0.9),
            hovertemplate="%{y} / %{x}<br>%{z:.3f} " + unit + "<extra></extra>",
        ), row=1, col=c)
        _annotate_heatmap(fig, mat, run_tags, ds_labels, row=1, col=c, fmt=fmt)
        fig.update_xaxes(title_text="sim run", row=1, col=c)
    _force_matrix_category_axes(fig)
    fig.update_yaxes(title_text="dataset", row=1, col=1)
    return apply_base_layout(
        fig,
        title="dataset × sim run: closed-loop 軌跡乖離・完走率"
        "<br><sub>空白セル = その dataset で当該 run が欠損 (sim 失敗/未実行)</sub>",
        height=_matrix_height(len(ds_labels)),
        margin=dict(l=110, r=20, t=110, b=60),
    )


def build_fig_cross_nstep_heatmap(
    ds_labels: list[str],
    case_tags: list[str],
    mats: dict[str, np.ndarray],
    horizon: int,
) -> go.Figure | None:
    """open-loop N-step 終端誤差の DS × case 行列 (縦/横/yaw の 3 パネル)。

    mats: {"long": 2D [cm], "lat": 2D [cm], "yaw": 2D [deg]}、shape (DS, case)、欠損 NaN。
    """
    if not ds_labels or not case_tags:
        return None
    panels = [("long", "縦 [cm]", "{:.1f}"), ("lat", "横 [cm]", "{:.1f}"),
              ("yaw", "yaw [deg]", "{:.2f}")]
    fig = make_grid(1, 3, subplot_titles=[t for _, t, _ in panels], horizontal_spacing=0.10)
    for c, (key, unit, fmt) in enumerate(panels, start=1):
        mat = mats[key]
        fig.add_trace(go.Heatmap(
            z=mat, x=case_tags, y=ds_labels, colorscale="YlOrRd",
            showscale=(c == 3), colorbar=dict(len=0.9),
            hovertemplate="%{y} / %{x}<br>%{z:.3f}<extra>" + unit + "</extra>",
        ), row=1, col=c)
        _annotate_heatmap(fig, mat, case_tags, ds_labels, row=1, col=c, fmt=fmt)
    _force_matrix_category_axes(fig)
    fig.update_yaxes(title_text="dataset", row=1, col=1)
    return apply_base_layout(
        fig,
        title=f"dataset × case: open-loop N={horizon} 終端誤差 RMSE"
        "<br><sub>どのモデル (case) がどの dataset で悪いかの俯瞰。空白セル = 出力欠損</sub>",
        height=_matrix_height(len(ds_labels)),
        margin=dict(l=110, r=20, t=110, b=60),
    )


def build_fig_cross_normalized_bars(
    aggs: dict[str, dict],
    horizons: list[int],
    scores: dict[str, float],
) -> go.Figure | None:
    """case 別の正規化 mean/worst 集約 (横断ロバスト性ランキング)。

    aggs: {case_tag: aggregate_normalized の返り値}。case は score 昇順 (良い順) に並べる。
    各パネル (horizon × 成分): mean を bar、worst を菱形マーカーで重ねる。
    """
    if not aggs or not horizons:
        return None
    tags = sorted(aggs, key=lambda t: scores.get(t, float("inf")))
    comps = [("nyaw", "yaw"), ("nlong", "縦"), ("nlat", "横")]
    titles = [f"N={h} {label} (baseline 比)" for h in horizons for _, label in comps]
    fig = make_grid(len(horizons), 3, subplot_titles=titles,
                    vertical_spacing=0.16, horizontal_spacing=0.08)
    colors = dict(zip(tags, qualitative_colors(len(tags))))
    for r, h in enumerate(horizons, start=1):
        for c, (key, _label) in enumerate(comps, start=1):
            means = [aggs[t]["by_h"][h][f"{key}_mean"] for t in tags]
            worsts = [aggs[t]["by_h"][h][f"{key}_worst"] for t in tags]
            fig.add_trace(go.Bar(
                x=tags, y=means, marker_color=[colors[t] for t in tags],
                name="mean", showlegend=False,
                hovertemplate="%{x}<br>mean %{y:.3f}<extra></extra>",
            ), row=r, col=c)
            fig.add_trace(go.Scatter(
                x=tags, y=worsts, mode="markers", name="worst",
                marker=dict(symbol="diamond", size=9, color="#222",
                            line=dict(color="white", width=1)),
                showlegend=(r == 1 and c == 1),
                hovertemplate="%{x}<br>worst %{y:.3f}<extra></extra>",
            ), row=r, col=c)
            fig.add_hline(y=1.0, line=dict(color="black", width=1.0, dash="dash"),
                          row=r, col=c)
    score_note = "  ".join(f"{t}={scores[t]:.3f}" for t in tags if t in scores)
    return apply_base_layout(
        fig,
        title="dataset 横断 正規化集約 (mean=棒 / worst=◆、1.0=baseline 同等)"
        f"<br><sub>score (小さいほど良い): {score_note}</sub>",
        height=320 * len(horizons) + 120,
    )


def build_fig_coverage_overview(cov_rows: list[dict]) -> go.Figure | None:
    """DS 別走行特性カバレッジの 4 パネル俯瞰 (評価データの偏りの可視化)。

    cov_rows: [{"label": str, "coverage": lib._coverage.compute_coverage の dict}]。
    """
    rows = [r for r in cov_rows if r.get("coverage")]
    if not rows:
        return None
    labels = [r["label"] for r in rows]
    covs = [r["coverage"] for r in rows]
    fig = make_grid(
        2, 2,
        subplot_titles=[
            "速度域 滞在時間比率", "加減速分布 (ax percentile)",
            "曲率半径ビン別 走行距離", "走行距離・カーブ数",
        ],
        vertical_spacing=0.16, horizontal_spacing=0.10,
        specs=[[{}, {}], [{}, {"secondary_y": True}]],
    )

    # (1,1) 速度域 stacked bar
    speed_keys = list(covs[0].get("speed_bins", {}).keys())
    speed_colors = qualitative_colors(len(speed_keys))
    for k, color in zip(speed_keys, speed_colors):
        fig.add_trace(go.Bar(
            x=labels, y=[c["speed_bins"].get(k, 0.0) for c in covs],
            name=f"v={k} m/s", marker_color=color, legendgroup="speed",
            hovertemplate="%{x}<br>" + k + " m/s: %{y:.2f}<extra></extra>",
        ), row=1, col=1)

    # (1,2) ax percentile: p5-p95 帯 + p25-p75 帯 + p50 マーカー
    p = {q: [c["ax_percentiles"].get(f"p{q}", 0.0) for c in covs] for q in (5, 25, 50, 75, 95)}
    fig.add_trace(go.Bar(
        x=labels, y=[hi - lo for hi, lo in zip(p[95], p[5])], base=p[5],
        marker_color="rgba(31,119,180,0.25)", name="ax p5–p95", legendgroup="ax",
        hovertemplate="%{x}<br>p5–p95<extra></extra>",
    ), row=1, col=2)
    fig.add_trace(go.Bar(
        x=labels, y=[hi - lo for hi, lo in zip(p[75], p[25])], base=p[25],
        marker_color="rgba(31,119,180,0.55)", name="ax p25–p75", legendgroup="ax",
        hovertemplate="%{x}<br>p25–p75<extra></extra>",
    ), row=1, col=2)
    fig.add_trace(go.Scatter(
        x=labels, y=p[50], mode="markers", name="ax 中央値",
        marker=dict(symbol="line-ew", size=14, line=dict(color="#d62728", width=2)),
        legendgroup="ax",
        hovertemplate="%{x}<br>p50 %{y:.2f} m/s²<extra></extra>",
    ), row=1, col=2)
    fig.add_hline(y=0.0, line=dict(color="black", width=0.8), row=1, col=2)

    # (2,1) 曲率半径ビン stacked bar
    curv_keys = list((covs[0].get("curvature") or {}).get("bins_dist_m", {}).keys())
    curv_colors = qualitative_colors(len(curv_keys))
    for k, color in zip(curv_keys, curv_colors):
        fig.add_trace(go.Bar(
            x=labels, y=[(c.get("curvature") or {}).get("bins_dist_m", {}).get(k, 0.0)
                         for c in covs],
            name=k, marker_color=color, legendgroup="curv",
            hovertemplate="%{x}<br>" + k + ": %{y:.0f} m<extra></extra>",
        ), row=2, col=1)

    # (2,2) 走行距離 bar + カーブ数 (secondary y)
    fig.add_trace(go.Bar(
        x=labels, y=[c.get("dist_m", 0.0) for c in covs],
        name="走行距離 [m]", marker_color="#1f77b4", legendgroup="dist",
        hovertemplate="%{x}<br>%{y:.0f} m<extra></extra>",
    ), row=2, col=2)
    fig.add_trace(go.Scatter(
        x=labels, y=[(c.get("curvature") or {}).get("curve_count", 0) for c in covs],
        mode="markers", name="カーブ数",
        marker=dict(symbol="diamond", size=9, color="#d62728"), legendgroup="dist",
        hovertemplate="%{x}<br>%{y} curves<extra></extra>",
    ), row=2, col=2, secondary_y=True)
    fig.update_yaxes(title_text="カーブ数", row=2, col=2, secondary_y=True)

    fig.update_layout(barmode="stack")
    # DS ラベルは純数字文字列になり得るため x 軸はカテゴリを明示 (_force_matrix_category_axes 参照)
    fig.update_xaxes(type="category")
    fig.update_yaxes(title_text="時間比率", row=1, col=1)
    fig.update_yaxes(title_text="ax [m/s²]", row=1, col=2)
    fig.update_yaxes(title_text="走行距離 [m]", row=2, col=1)
    fig.update_yaxes(title_text="走行距離 [m]", row=2, col=2, secondary_y=False)
    return apply_base_layout(
        fig,
        title="dataset 走行特性カバレッジ"
        "<br><sub>評価データの偏り (低速ばかり・直線ばかり等) の俯瞰。速度域ビンは"
        " open-loop 速度域別 RMSE と同一</sub>",
        height=760,
        barmode="stack",
    )


def build_fig_loo_stability(
    ds_labels: list[str],
    case_tags: list[str],
    score_delta: np.ndarray,
    best_changed: list[bool],
) -> go.Figure | None:
    """leave-one-out 安定性: 除外 DS × case の score 変化ヒートマップ。

    score_delta: shape (DS, case)。除外時 score − 全 DS score (負 = その DS が当該 case の
    score を押し上げていた = その DS で case が悪い)。best_changed[i]=True の DS は
    「除外すると best case が入れ替わる」— ラベルに ★ を付ける。
    """
    if not ds_labels or not case_tags:
        return None
    ylabels = [f"★ {l}" if ch else l for l, ch in zip(ds_labels, best_changed)]
    amax = float(np.nanmax(np.abs(score_delta))) if np.isfinite(score_delta).any() else 1.0
    fig = make_grid(1, 1)
    fig.add_trace(go.Heatmap(
        z=score_delta, x=case_tags, y=ylabels,
        colorscale="RdBu", zmid=0.0, zmin=-amax, zmax=amax,
        colorbar=dict(title="Δscore"),
        hovertemplate="除外 %{y} / %{x}<br>Δscore %{z:+.3f}<extra></extra>",
    ))
    _annotate_heatmap(fig, score_delta, case_tags, ylabels, row=1, col=1, fmt="{:+.2f}")
    _force_matrix_category_axes(fig)
    fig.update_xaxes(title_text="case")
    fig.update_yaxes(title_text="除外 dataset")
    return apply_base_layout(
        fig,
        title="leave-one-out 安定性 (除外 DS × case の score 変化)"
        "<br><sub>青 (負) = その dataset が当該 case のスコアを悪化させていた。"
        "★ = 除外すると best case が入れ替わる dataset (結論を左右する)</sub>",
        height=_matrix_height(len(ds_labels)) + 40,
        margin=dict(l=130, r=20, t=110, b=60),
    )
