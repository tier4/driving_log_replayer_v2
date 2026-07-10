"""`lib/_figures` 配下の build_fig_* が共有する plotly 描画ヘルパー。"""

from __future__ import annotations

import numpy as np
import plotly.colors as pc
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from .._map import map_ways_in_bbox  # noqa: F401  (build_fig_* が地図 bbox 抽出に使う)
from .._plotly_utils import lanes_to_trace  # noqa: F401

_PLOTLY_DASH = {"-": "solid", "--": "dash", "-.": "dashdot", ":": "dot"}
_PLOTLY_DASH_TUPLES = {(4, 2): "longdash", (3, 1, 1, 1): "longdashdot"}


def plotly_dash(ls) -> str:
    """matplotlib linestyle（文字列 or (offset, on-off タプル)）を plotly dash enum へ。"""
    if isinstance(ls, tuple):
        return _PLOTLY_DASH_TUPLES.get(tuple(ls[1]), "dash")
    return _PLOTLY_DASH.get(ls, "solid")


# 全図共通の体裁。template は write_fig_json で剥がす（plotly.js 内蔵テーマで描く）ため
# ここでは指定しない。
def apply_base_layout(
    fig: go.Figure, *, title: str | None = None, height: int | None = None, **kwargs
) -> go.Figure:
    """全図共通のレイアウト既定を適用する（title/height は任意上書き）。"""
    # 凡例は既定で **水平・図の最上端**。右側縦置きは横幅を食ってプロットが潰れるため水平に
    # するが、`y=1.02`（paper 基準）だと最上段サブプロットタイトルと重なって隠れる
    # （paper 単位はプロット領域高=height-margin に正規化され、margin を増やしても凡例が
    # 相対的に下がるため衝突が解けない循環に陥る）。`yref="container"` で図の絶対最上端を
    # 基準に height 非依存の px オフセットで配置する。
    #
    # 【注意】title も legend も同じ y≈0.99〜1.0 帯域に置くと重なって表示が潰れる。さらに
    # title 単体でも、複数行 (`<br>` 付き、例: メインタイトル + `<sup>` 注記) を
    # `y=0.99, yanchor="top"` のような "ほぼ最上端" に置くと、1 行目が container の外側に
    # はみ出して欠ける（kaleido/plotly の複数行 title アンカー実装上、yanchor="top" でも
    # 見た目のブロック開始位置が y より上に来るため。1 行タイトルなら気づかない程度だが
    # 2 行だと明確に上端が欠ける。実測で確認済み）。よって title は行数に応じて y を
    # 下げて確保し、legend は title のさらに下、margin.t はその両方が収まる値まで
    # 自動的に引き上げる（呼び出し側が margin= で明示指定してもこれより小さくはしない）。
    h = height or 450
    line_px = 20  # フォントサイズ14相当の実測ベース行送り
    n_title_lines = title.count("<br>") + 1 if title else 0
    # title ブロックの「開始」に必要な最上端からのオフセット（複数行はみ出し対策込み）。
    title_top_px = 16 + line_px * n_title_lines if title else 6
    # legend は title ブロック（title_top_px 起点、n_title_lines 行分）の下に置く。
    legend_top_px = title_top_px + line_px * n_title_lines + 10 if title else 6
    # legend の下にサブプロットタイトル等と衝突しない余白を確保した上での必要マージン。
    required_margin_t = legend_top_px + 24

    margin_kw = kwargs.pop("margin", None)
    caller_margin_t = margin_kw.get("t") if isinstance(margin_kw, dict) else None
    margin_t = max(required_margin_t, caller_margin_t or 0, 100)
    margin = dict(l=60, r=20, t=margin_t, b=50)
    if isinstance(margin_kw, dict):
        margin.update({k: v for k, v in margin_kw.items() if k != "t"})

    legend_y = max(0.0, 1.0 - legend_top_px / h)
    layout = dict(
        autosize=True,
        margin=margin,
        legend=dict(orientation="h", yref="container", yanchor="top", y=legend_y,
                    xref="paper", xanchor="left", x=0, bgcolor="rgba(255,255,255,0.7)"),
        font=dict(size=12),
    )
    if title is not None:
        title_y = max(0.0, 1.0 - title_top_px / h)
        layout["title"] = dict(text=title, font=dict(size=14), x=0.5, xanchor="center",
                               y=title_y, yanchor="top")
    if height is not None:
        layout["height"] = height
    layout.update(kwargs)
    fig.update_layout(**layout)
    return fig


def add_bottom_note(
    fig: go.Figure, note: str | None, *, height: int,
    top_margin: int = 100, bottom_margin: int = 50, gap_px: int = 8,
) -> None:
    """図下部中央に注記を足す（下マージン内に収め、高さによらず切れないようにする）。

    annotation は `yref="container"` 非対応（paper か軸参照のみ）。素朴な `yref="paper"
    y=-0.08` は paper 単位がプロット領域高に正規化されるため、背の高い図ほど offset が
    px で増えて下マージン (bottom_margin) を超えて切れる。ここでは図高 `height` から
    「注記下端を図の最下端から gap_px だけ上」に置く paper y を逆算し、`yanchor="bottom"`
    で上方向にテキストを伸ばすことで height 非依存に下マージン内へ収める。
    """
    if not note:
        return
    plot_h = max(1, height - top_margin - bottom_margin)
    y = (gap_px - bottom_margin) / plot_h
    fig.add_annotation(
        xref="paper", yref="paper", x=0.5, y=y,
        xanchor="center", yanchor="bottom", showarrow=False,
        text=note, font=dict(size=10, color="#555555"),
    )


def make_grid(rows: int, cols: int, *, subplot_titles=None, **kwargs):
    """`make_subplots` の共通既定（余白・タイトル）付きラッパ。

    各図モジュールが 1:1 で matplotlib の subplot 構成を移植するための入口。
    `specs`/`shared_xaxes`/`column_widths`/`secondary_y` 等は kwargs でそのまま渡す。
    """
    defaults = dict(
        rows=rows,
        cols=cols,
        subplot_titles=subplot_titles,
        vertical_spacing=kwargs.pop("vertical_spacing", 0.10 if rows > 1 else 0.0),
        horizontal_spacing=kwargs.pop("horizontal_spacing", 0.08 if cols > 1 else 0.0),
    )
    defaults.update(kwargs)
    return make_subplots(**defaults)


# --- 数値ヘルパー（matplotlib 側の派生量を plotly でも再現するため） -----------------
def ma_window(n: int, divisor: int = 30, minimum: int = 3) -> int:
    """系列長 `n` から既定の移動平均窓を決める（matplotlib 側の `len/30` 慣習に合わせる）。"""
    return max(minimum, n // divisor)


def axis_range_from_limits(
    limits_df, cols, scale: float = 1.0, horizon: int | None = None,
    symmetric: bool = False, pad: float = 0.05,
):
    """ケース横断の `limits_df`（全ケース連結）から y 軸範囲 (lo, hi) を作る（旧 _unified_ylim）。

    `limits_df` が None なら None（自己スケール）。単一ケース実行では None、step6 が
    全ケース連結 DataFrame を渡して軸を統一する。ROS 非依存（pandas のみ）。
    """
    if limits_df is None:
        return None
    df = limits_df if horizon is None else limits_df[limits_df["horizon"] == horizon]
    cols = [cols] if isinstance(cols, str) else cols
    arrs = [df[c].to_numpy(dtype=float) * scale for c in cols if c in df.columns]
    if not arrs:
        return None
    vals = np.concatenate(arrs)
    vals = vals[np.isfinite(vals)]
    if len(vals) == 0:
        return None
    lo, hi = float(vals.min()), float(vals.max())
    if symmetric:
        m = max(abs(lo), abs(hi))
        lo, hi = -m, m
    span = (hi - lo) or 1.0
    return [lo - pad * span, hi + pad * span]


def qualitative_colors(n: int) -> list[str]:
    """case 別など離散系列の循環色（matplotlib 既定 prop_cycle 相当の D3 10 色）。"""
    base = pc.qualitative.D3
    return [base[i % len(base)] for i in range(n)]


def viridis_at(fracs) -> list[str]:
    """viridis を任意の位置 [0,1] でサンプルした色のリスト（連続値の色付け用）。"""
    fr = [min(1.0, max(0.0, float(f))) for f in fracs]
    return pc.sample_colorscale("Viridis", fr) if fr else []
