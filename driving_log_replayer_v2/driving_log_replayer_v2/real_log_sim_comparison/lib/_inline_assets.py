"""report.html にアセットを埋め込むための純関数群.

step11_build_html_report が使う。(1) plotly.min.js を <head> に 1 回インラインする、
(2) 図スペック (*.fig.json) と自己完結ビューア HTML を gzip+base64 で
`<script type="application/gzip+...+base64">` に埋め込む、の 2 系統。

gzip+base64 埋め込みの理由: マルチデータセットでは図スペックの合計が数十〜数百 MB に
達する。plotly の図 JSON は圧縮率が高く (1/5〜1/10)、base64 は HTML 安全な文字
(英数 + / =) のみなので `</script>` エスケープも不要になる。展開はブラウザの
DecompressionStream('gzip') でレンダリング時 (= 図が可視化された時) のみ行う。
"""

from __future__ import annotations

import base64
import gzip


def gzip_b64(text: str) -> str:
    """UTF-8 テキストを gzip 圧縮して base64 文字列にする (script 埋め込み用)。

    mtime=0 固定で同一入力 → 同一出力 (再生成 diff を安定させる)。
    """
    return base64.b64encode(
        gzip.compress(text.encode("utf-8"), compresslevel=9, mtime=0)
    ).decode("ascii")


def plotly_js_script() -> str:
    """plotly.min.js の中身を <script> で包んだ文字列を返す（report に 1 回だけ埋める）。"""
    from plotly.offline import get_plotlyjs  # noqa: PLC0415

    return f"<script>{get_plotlyjs()}</script>"
