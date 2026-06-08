"""plotly Figure を「データ + レイアウト」の図スペック JSON に書き出す I/O.

新方式（commit 59fef44 の base64 単一 HTML を置換）の中核。各解析ステップは
matplotlib SVG を吐く代わりに、`lib/_figures` の純関数で組んだ `go.Figure` を
`<stem>.fig.json` として `comparison/` に出力する。step11 がこれらを収集して
単一 report.html に `<script type="application/json">` で並置し、plotly.js が
クライアント側で `Plotly.newPlot` 描画する（SVG ベクタ全頂点 + base64 +33% の
肥大を回避し、ズーム/パン/hover も得る）。step12 は同じ JSON を notebook で
`plotly.io.from_json` 描画する。

容量対策（図スペックは生の数値配列を含むため）:
- `template` を剥がす（plotly.js 内蔵デフォルトテーマで描く。1 図あたり ~6KB の
  テンプレ重複を排除）。
- float を有効数字で丸める（float64 の repr 17 桁ノイズを削る）。大座標（地図 x≈9e4）と
  微小値（steer≈1e-4）が混在するため固定小数ではなく**有効数字**で丸める。
- 区切り詰めの最小化 JSON で出力。

本モジュールは plotly と標準ライブラリのみに依存し ROS 非依存（notebook からも import 可）。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import plotly.graph_objects as go

# 図スペックの拡張子。step11/step12 はこの suffix で収集する。
FIG_SUFFIX = ".fig.json"

# float 丸めの既定有効数字。地図座標（~9e4）で約 0.1mm、微小な steer（~1e-4）でも
# 相対精度を保つ。視覚描画には十分で、生 float64 repr より 30〜40% 小さい。
_DEFAULT_SIGFIGS = 8


def _round_sig(x: float, sig: int) -> float:
    """有効数字 `sig` 桁へ丸める（0・非有限はそのまま）。"""
    if x == 0 or not math.isfinite(x):
        return x
    return round(x, sig - 1 - math.floor(math.log10(abs(x))))


def _round_floats(obj, sig: int):
    """ネストした dict/list 内の全 float を有効数字 `sig` 桁へ再帰的に丸める。"""
    if isinstance(obj, float):
        return _round_sig(obj, sig)
    if isinstance(obj, list):
        return [_round_floats(v, sig) for v in obj]
    if isinstance(obj, dict):
        return {k: _round_floats(v, sig) for k, v in obj.items()}
    return obj


def fig_to_compact_json(fig: go.Figure, *, sigfigs: int = _DEFAULT_SIGFIGS) -> str:
    """`go.Figure` をコンパクトな図スペック JSON 文字列にする（template 剥離 + float 丸め）。"""
    spec = json.loads(fig.to_json())
    layout = spec.get("layout")
    if isinstance(layout, dict):
        layout.pop("template", None)
    spec = _round_floats(spec, sigfigs)
    return json.dumps(spec, separators=(",", ":"), ensure_ascii=False)


def write_fig_json(
    fig: go.Figure, out_path: Path, *, sigfigs: int = _DEFAULT_SIGFIGS
) -> Path:
    """`go.Figure` を `<stem>.fig.json` として書き出す。

    `out_path` の suffix は問わない（呼び出し側が `<stem>.fig.json` を渡す想定だが、
    `.svg`/`.html` を渡されても stem を取り `FIG_SUFFIX` に正規化する）。
    """
    out_path = Path(out_path)
    # `foo.fig.json` を渡された場合 stem は `foo.fig` になるため、まず .json/.fig を剥がす
    stem = out_path.name
    for suf in (".json", ".fig", ".svg", ".html"):
        if stem.endswith(suf):
            stem = stem[: -len(suf)]
    target = out_path.with_name(stem + FIG_SUFFIX)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(fig_to_compact_json(fig, sigfigs=sigfigs), encoding="utf-8")
    print(f"  保存: {target.name}")
    return target


def collect_fig_jsons(comparison_dir: Path) -> list[Path]:
    """`comparison_dir` 配下の `*.fig.json` を昇順で集める。"""
    comparison_dir = Path(comparison_dir)
    figs = [p for p in comparison_dir.rglob("*" + FIG_SUFFIX)]
    return sorted(figs, key=lambda p: str(p))
