"""report.html に自己完結 HTML アセットを埋め込むための純関数群.

step11_build_html_report が使う。図スペック (*.fig.json) はクライアント側 plotly で描画する
ため埋め込み関数は不要だが、(1) plotly.min.js を <head> に 1 回インラインする、(2) 再生
ビューア (canvas+JS の自己完結 HTML) を `<iframe srcdoc>` で隔離埋め込みする、の 2 つは残る。
"""

from __future__ import annotations


def escape_srcdoc(text: str) -> str:
    """HTML を <iframe srcdoc='…'> 属性値に安全に埋め込めるようエスケープする.

    srcdoc は属性値なので最低限 & と引用符を実体参照化する。**順序が重要**:
    先に & を &amp; にしてから引用符を変換しないと、既存の &amp; 等が二重化・破損する。
    srcdoc は単引用符で囲む前提（playback HTML は単引用符を含まない）ため ' は変換不要だが、
    保険として両引用符を変換する。
    """
    return (
        text.replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("'", "&#39;")
    )


def plotly_js_script() -> str:
    """plotly.min.js の中身を <script> で包んだ文字列を返す（report に 1 回だけ埋める）。"""
    from plotly.offline import get_plotlyjs  # noqa: PLC0415

    return f"<script>{get_plotlyjs()}</script>"
