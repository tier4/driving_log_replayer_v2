"""自己完結 report.html へアセットを埋め込むための純関数群.

step11_build_html_report が comparison/ 配下の図 (SVG / plotly HTML / playback HTML)
を単一の report.html に**外部参照なし**で埋め込むために使う。種別ごとに埋め込み方が
異なる（一律化すると衝突・容量破綻する）:

- SVG       → data-URI base64。matplotlib SVG はインライン展開すると clip-path/glyph の
              id が図間で衝突して描画が壊れるため、img/背景画像として分離埋め込みする。
- plotly図  → 各 standalone HTML の <body> 内部フラグメント (div + Plotly.newPlot script)
              を抽出して本文に並置し、plotly.min.js は report 全体で 1 回だけ共有する
              （iframe 化すると plotly.min.js が図数分重複し数十 MB に膨らむ）。
- playback  → 独自 canvas JS の自己完結 HTML。グローバル衝突回避のため <iframe srcdoc>
              でそのまま隔離埋め込みする。

本モジュールは I/O とテキスト整形のみの純関数で構成し、HTML 組み立て (build_html) から
分離してテスト容易性を保つ。
"""

from __future__ import annotations

import base64
from pathlib import Path
import re

# matplotlib SVG の viewBox="0 0 W H"（全 SVG が保持）から縦横比を取るための正規表現。
_VIEWBOX_RE = re.compile(r'viewBox="0 0 ([\d.]+) ([\d.]+)"')
# plotly standalone HTML の <body>…</body> 内部を抜き出す（body は 1 つ・src なし script のみ）。
_BODY_RE = re.compile(r"<body[^>]*>(.*)</body>", re.DOTALL)


def svg_to_data_uri(path: Path) -> str:
    """SVG ファイルを `data:image/svg+xml;base64,…` data-URI 文字列にする。

    base64 化することで CSS background-image / img src にそのまま埋め込め、かつ
    matplotlib SVG 内部の id 衝突や引用符エスケープ問題を完全に回避できる。
    """
    raw = Path(path).read_bytes()
    b64 = base64.b64encode(raw).decode("ascii")
    return f"data:image/svg+xml;base64,{b64}"


def svg_aspect_ratio(path: Path) -> float | None:
    """SVG の viewBox から width/height（縦横比）を返す。取れなければ None。

    サムネ枠を画像読込前に確定させる CSS `aspect-ratio` 用。background-image 表示では
    要素自身に固有サイズが無いため、これが無いとレイアウトが潰れる。
    """
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    m = _VIEWBOX_RE.search(text)
    if not m:
        return None
    w, h = float(m.group(1)), float(m.group(2))
    if h <= 0:
        return None
    return w / h


def is_plotly_html(text: str) -> bool:
    """HTML 文字列が plotly 製図か（playback 等の自己完結 HTML と区別する）。

    plotly の出力は必ず `Plotly.newPlot(...)` を含む。playback ビューアは canvas 独自
    実装で 0 件のため、これで判別できる。
    """
    return "Plotly.newPlot" in text


def extract_plotly_body(text: str) -> str:
    """plotly standalone HTML の <body> 内部（div + 描画 script）を返す.

    plotly.min.js は <head> の <script src> にあり body には混入しないため、body 内部
    をそのまま本文に並置すればグローバルの Plotly を共有して描画される。div id は UUID で
    一意・`window.PLOTLYENV = window.PLOTLYENV || {}` は冪等なので、複数並置しても衝突しない。
    """
    m = _BODY_RE.search(text)
    if m is None:
        raise ValueError("plotly HTML に <body>…</body> が見つかりません")
    return m.group(1)


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
