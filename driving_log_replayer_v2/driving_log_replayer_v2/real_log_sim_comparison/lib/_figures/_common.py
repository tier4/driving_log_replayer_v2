"""`lib/_figures` 配下の build_fig_* が共有する plotly 描画ヘルパー.

matplotlib SVG を廃し各図を `go.Figure` で組む新方式の土台。ここに描画の共通語彙
（サブプロット枠・移動平均・最小二乗フィット・速度ビン色分け・散布間引き・地図背景・
パラメータ注釈・matplotlib スタイル→plotly 変換）を集約し、各図モジュールは
データ整形済みの配列/DataFrame を受けて図を組み立てるだけにする。

**ROS 非依存**: plotly / numpy と ROS 非依存 lib（`_plotly_utils`/`_map`）のみに依存し、
notebook（rclpy 無し kernel）からも import できる。`_io`（rosbag2_py 依存）は絶対に
import しないこと。
"""

from __future__ import annotations

import numpy as np
import plotly.colors as pc
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ROS 非依存 lib のみ再利用（_io は import しない）
from .._map import map_ways_in_bbox  # noqa: F401  (build_fig_* が地図 bbox 抽出に使う)
from .._plotly_utils import add_params_annotation_plotly, lanes_to_trace  # noqa: F401

# --- matplotlib スタイル → plotly 変換（軌跡・時系列の線/マーカー） -------------------
_PLOTLY_DASH = {"-": "solid", "--": "dash", "-.": "dashdot", ":": "dot"}
_PLOTLY_DASH_TUPLES = {(4, 2): "longdash", (3, 1, 1, 1): "longdashdot"}
_PLOTLY_MARKER = {
    "o": "circle", "^": "triangle-up", "s": "square", "D": "diamond",
    "v": "triangle-down", "P": "cross", "*": "star", "X": "x",
}


def plotly_dash(ls) -> str:
    """matplotlib linestyle（文字列 or (offset, on-off タプル)）を plotly dash enum へ。"""
    if isinstance(ls, tuple):
        return _PLOTLY_DASH_TUPLES.get(tuple(ls[1]), "dash")
    return _PLOTLY_DASH.get(ls, "solid")


def plotly_marker(m) -> str:
    """matplotlib marker 記号を plotly symbol へ（未知は circle）。"""
    return _PLOTLY_MARKER.get(m, "circle")


# --- レイアウト ------------------------------------------------------------------
# 全図共通の体裁。template は write_fig_json で剥がす（plotly.js 内蔵テーマで描く）ため
# ここでは指定しない。
def apply_base_layout(
    fig: go.Figure, *, title: str | None = None, height: int | None = None, **kwargs
) -> go.Figure:
    """全図共通のレイアウト既定を適用する（title/height は任意上書き）。"""
    layout = dict(
        autosize=True,
        margin=dict(l=60, r=20, t=60 if title else 30, b=50),
        legend=dict(bgcolor="rgba(255,255,255,0.7)"),
        font=dict(size=12),
    )
    if title is not None:
        layout["title"] = dict(text=title, font=dict(size=14))
    if height is not None:
        layout["height"] = height
    layout.update(kwargs)
    fig.update_layout(**layout)
    return fig


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
def moving_average(y: np.ndarray, window: int) -> np.ndarray:
    """中央寄せ移動平均（端は短縮窓）。matplotlib 版の raw+MA 重ね描きを再現する。

    window<=1 や長さ不足では入力をそのまま返す。NaN は無視して平均する。
    """
    y = np.asarray(y, dtype=float)
    n = len(y)
    if window <= 1 or n == 0:
        return y
    half = window // 2
    out = np.empty(n, dtype=float)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        seg = y[lo:hi]
        valid = seg[~np.isnan(seg)]
        out[i] = valid.mean() if valid.size else np.nan
    return out


def ma_window(n: int, divisor: int = 30, minimum: int = 3) -> int:
    """系列長 `n` から既定の移動平均窓を決める（matplotlib 側の `len/30` 慣習に合わせる）。"""
    return max(minimum, n // divisor)


def polyfit_line(x: np.ndarray, y: np.ndarray, deg: int = 1, n: int = 50):
    """有限値のみで最小二乗フィットし、描画用の (xs, ys, coef) を返す。

    matplotlib 側の `np.polyfit` による回帰直線重ね描き（steer_vs_lateral 等）を再現する。
    フィット不能（点不足）なら (None, None, None)。
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() <= deg:
        return None, None, None
    coef = np.polyfit(x[m], y[m], deg)
    xs = np.linspace(x[m].min(), x[m].max(), n)
    ys = np.polyval(coef, xs)
    return xs, ys, coef


# --- 速度ビン色分け（散布図で共通） -------------------------------------------------
# (下限, 上限, 色)。step5 の速度域別散布と同じ 4 ビン・同じ配色。
SPEED_BINS: list[tuple[float, float, str]] = [
    (0.0, 2.0, "#4472C4"),
    (2.0, 5.0, "#ED7D31"),
    (5.0, 8.0, "#A9D18E"),
    (8.0, np.inf, "#FF0000"),
]


def speed_label(lo: float, hi: float) -> str:
    """速度ビンの凡例ラベル（上限が無限なら「v≥lo」）。"""
    return f"v={lo:.0f}–{hi:.0f} m/s" if np.isfinite(hi) else f"v≥{lo:.0f} m/s"


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


def viridis_colors(n: int) -> list[str]:
    """viridis を `n` 等分サンプルした色（matplotlib の horizon 別色分けを再現）。"""
    if n <= 0:
        return []
    if n == 1:
        return pc.sample_colorscale("Viridis", [0.0])
    return pc.sample_colorscale("Viridis", [i / (n - 1) for i in range(n)])


def qualitative_colors(n: int) -> list[str]:
    """case 別など離散系列の循環色（matplotlib 既定 prop_cycle 相当の D3 10 色）。"""
    base = pc.qualitative.D3
    return [base[i % len(base)] for i in range(n)]


def viridis_at(fracs) -> list[str]:
    """viridis を任意の位置 [0,1] でサンプルした色のリスト（連続値の色付け用）。"""
    fr = [min(1.0, max(0.0, float(f))) for f in fracs]
    return pc.sample_colorscale("Viridis", fr) if fr else []


def speed_bin_masks(vx: np.ndarray):
    """速度配列を `SPEED_BINS` のマスクへ分ける。`[(mask, label, color), ...]` を返す。"""
    vx = np.asarray(vx, dtype=float)
    return [
        ((vx >= lo) & (vx < hi), speed_label(lo, hi), color)
        for lo, hi, color in SPEED_BINS
    ]


def downsample(*arrays: np.ndarray, max_points: int = 1500):
    """同じ長さの配列群を等間隔間引きする（散布点の図スペック肥大を防ぐ）。

    matplotlib 側の `rasterized=True`（点数を保ったままビットマップ化）の代替。
    plotly では点数自体が JSON サイズに直結するため、上限を超えたら等間隔抽出する。
    """
    arrays = [np.asarray(a) for a in arrays]
    n = len(arrays[0]) if arrays else 0
    if n <= max_points:
        return arrays if len(arrays) > 1 else arrays[0]
    stride = int(np.ceil(n / max_points))
    sliced = [a[::stride] for a in arrays]
    return sliced if len(sliced) > 1 else sliced[0]
