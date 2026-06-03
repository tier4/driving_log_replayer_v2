"""plotly インタラクティブ図の共通ユーティリティ.

マップ上プロット（trajectory_with_map / map_distribution）を plotly のスタンドアロン
HTML として出力するためのヘルパー群。plotly.js はバンドルフォルダ
（`RuntimeConfig.base_dir` = index.html と同じ階層）に `plotly.min.js` を 1 つ同梱し、
各図 HTML から相対パスで参照する（CDN 不使用・オフライン動作維持）。
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import plotly.graph_objects as go

from ._params_utils import load_sim_params, make_annotation_text

PLOTLYJS_NAME = "plotly.min.js"

# インタラクティブ図 (*.html) の高さ [px] の単一ソース。生成側 (step4/step5 の
# update_layout) と埋め込み側 (step11 の iframe 高さ = 高さ + IFRAME_PAD) の両方が
# ここを参照する。新しい図を追加する際はここに stem → 高さを追記すること。
# plotly 以外の自己完結 HTML 図 (trajectory_playback 等) も iframe 高さはここで決まる。
FIG_HEIGHTS: dict[str, int] = {
    "trajectory_with_map": 800,
    "trajectory_xy": 800,
    "trajectory_playback": 760,
    "map_distribution": 520,
}
# iframe に追加する余白 [px]（plotly モードバー等のはみ出し分）
IFRAME_PAD = 30


def ensure_plotlyjs(archive_root: Path) -> Path:
    """`archive_root` 直下に plotly.min.js を書き出す（既存なら何もしない）。

    archive_root はバンドルフォルダ（index.html と同じ階層 = RuntimeConfig.base_dir）。
    """
    archive_root = Path(archive_root)
    archive_root.mkdir(parents=True, exist_ok=True)
    js_path = archive_root / PLOTLYJS_NAME
    if not js_path.exists():
        from plotly.offline import get_plotlyjs  # noqa: PLC0415

        js_path.write_text(get_plotlyjs(), encoding="utf-8")
        print(f"  保存: {js_path}")
    return js_path


def write_plotly_html(fig: go.Figure, out_path: Path, archive_root: Path) -> None:
    """plotly Figure をスタンドアロン HTML として書き出す。

    plotly.js は `archive_root/plotly.min.js` への相対パス <script> で参照する
    （include_plotlyjs にパス文字列を渡す挙動は plotly のバージョン依存のため、
    手動で <head> に注入する）。
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    js_path = ensure_plotlyjs(archive_root)
    rel_js = os.path.relpath(js_path, out_path.parent)

    html = fig.to_html(full_html=True, include_plotlyjs=False)
    script_tag = f'<script src="{rel_js.replace(os.sep, "/")}"></script>'
    assert "<head>" in html, "plotly to_html の出力に <head> がありません"
    html = html.replace("<head>", f"<head>{script_tag}", 1)
    out_path.write_text(html, encoding="utf-8")
    print(f"  保存: {out_path.name}")


def add_params_annotation_plotly(
    fig: go.Figure,
    params: dict | None = None,
    params_dir: Path | None = None,
) -> None:
    """図の右下にモデルパラメータの注釈ボックスを追加する（matplotlib 版と同等）。"""
    if params is None:
        params = load_sim_params(params_dir)
    text = make_annotation_text(params).replace("\n", "<br>")
    fig.add_annotation(
        xref="paper",
        yref="paper",
        x=1.0,
        y=0.0,
        xanchor="right",
        yanchor="bottom",
        align="right",
        showarrow=False,
        text=text,
        font=dict(family="monospace", size=9, color="#555555"),
        bgcolor="rgba(255,255,255,0.7)",
        bordercolor="#aaaaaa",
        borderwidth=1,
    )


def lanes_to_trace(ways: list[np.ndarray], **trace_kwargs) -> go.Scatter:
    """レーン polyline 群を NaN 区切りで連結した単一 trace にする。

    way ごとに trace を作ると数千 trace になり描画・凡例が破綻するため、
    1 本の Scatter にまとめる（WebGL 非対応環境でも表示できるよう Scattergl は
    使わない。bbox フィルタ後の頂点数は SVG レンダリングで十分軽い）。
    """
    xs: list[float] = []
    ys: list[float] = []
    for pts in ways:
        xs.extend(pts[:, 0].tolist())
        ys.extend(pts[:, 1].tolist())
        xs.append(float("nan"))
        ys.append(float("nan"))
    defaults = dict(
        mode="lines",
        line=dict(color="#cccccc", width=0.5),
        hoverinfo="skip",
        showlegend=False,
        name="lanelet",
    )
    defaults.update(trace_kwargs)
    return go.Scatter(x=xs, y=ys, **defaults)
