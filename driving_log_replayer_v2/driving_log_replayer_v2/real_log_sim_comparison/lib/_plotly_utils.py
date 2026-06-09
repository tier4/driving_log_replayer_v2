"""plotly インタラクティブ図の共通ユーティリティ.

図スペック (*.fig.json) を組む際に共有する高さ定義・地図レーン trace。
plotly.js のインラインは step11 が lib/_inline_assets.plotly_js_script で 1 回だけ行う
（CDN 不使用・オフライン動作維持）。
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

# 図 (*.fig.json) の高さ [px] の単一ソース。build_fig_* (生成側) と step11 の iframe 高さ
# (= 高さ + IFRAME_PAD; 自己完結 HTML ビューア用) の両方が参照する。
FIG_HEIGHTS: dict[str, int] = {
    "trajectory_with_map": 800,
    "trajectory_xy": 800,
    "trajectory_playback": 1020,  # 地図ビュー + 下部同期時系列プロット (6 パネル ~252px)
    "lon_lat_model": 1080,  # 地図ビュー + 運動方程式 + 検証パネル5枚(2列3行 ~540px) + つまみ6個
    "map_distribution": 1000,  # 2 行 (N=1 / N=max) × 3 列
}
# iframe に追加する余白 [px]（plotly モードバー等のはみ出し分）
IFRAME_PAD = 30


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
