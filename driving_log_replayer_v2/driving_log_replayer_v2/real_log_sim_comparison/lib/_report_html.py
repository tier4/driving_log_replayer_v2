"""report.html 系の生成で使う共通 HTML ヘルパー.

step11 の `step_report_html.py` から、アセット名の解釈・Markdown 組版・
設定 YAML の details 化・データセット表示名の生成を切り出す。
"""

from __future__ import annotations

import html
import re
from pathlib import Path

from ._fig_io import FIG_SUFFIX, collect_fig_jsons

_PLAYBACK_SUFFIX = ".html"


def slug(text: str) -> str:
    """HTML id/name 用に安全な文字へ変換する。"""
    return re.sub(r"[^A-Za-z0-9_-]+", "-", text).strip("-").lower()


def asset_stem(rel: Path) -> str:
    """アセット相対パスから拡張子（.fig.json / .html）を除いた stem を返す。"""
    name = rel.name
    for suf in (FIG_SUFFIX, _PLAYBACK_SUFFIX):
        if name.endswith(suf):
            return name[: -len(suf)]
    return rel.stem


def is_fig_json(rel: Path) -> bool:
    """plotly 図スペック (*.fig.json) かを返す。"""
    return rel.name.endswith(FIG_SUFFIX)


def scenario_date(scenario_yaml: Path | None) -> str:
    """auto_scenario.yaml の FileHeader.date (YYYY-MM-DD) を返す (取れなければ空)。"""
    if scenario_yaml is None or not scenario_yaml.is_file():
        return ""
    try:
        import yaml  # noqa: PLC0415

        doc = yaml.safe_load(scenario_yaml.read_text(encoding="utf-8")) or {}
        date = ((doc.get("OpenSCENARIO") or {}).get("FileHeader") or {}).get("date", "")
        return str(date)[:10]
    except Exception:  # noqa: BLE001
        return ""


def dataset_label(dataset_id: str, *, scenario_yaml: Path | None = None,
                  label: str | None = None) -> str:
    """セレクタ・見出しに使う人間可読ラベルを返す。"""
    if label:
        return label
    short = dataset_id[:8]
    date = scenario_date(scenario_yaml)
    return f"{short} ｜ {date}" if date else short


def render_markdown(text: str) -> str:
    """Markdown を HTML 化する。markdown パッケージが無ければ <pre> でフォールバック。"""
    try:
        import markdown as _md  # noqa: PLC0415
    except ImportError:
        return f"<pre class='md-fallback'>{html.escape(text)}</pre>"

    base = ["tables", "fenced_code"]
    try:
        return _md.markdown(
            text,
            extensions=[*base, "pymdownx.arithmatex"],
            extension_configs={"pymdownx.arithmatex": {"generic": True}},
        )
    except (ImportError, ValueError):
        return _md.markdown(text, extensions=base)


def render_config_details(config_files: list[tuple[str, Path]]) -> str:
    """設定 YAML 群を <details class='cfg-file'> の連結 HTML にする。"""
    blocks: list[str] = []
    for title, path in config_files:
        if path is None or not Path(path).exists():
            continue
        try:
            text = Path(path).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        blocks.append(
            f"<details class='cfg-file'><summary>{html.escape(title)}"
            f"<span class='fname'>{html.escape(Path(path).name)}</span></summary>"
            f"<pre class='cfg-pre'>{html.escape(text)}</pre></details>"
        )
    return "".join(blocks)


def collect_report_assets(comparison_dir: Path, selfcontained_html: set[str]) -> list[Path]:
    """比較ディレクトリ配下の report 対象アセットを返す。

    plotly 図スペック (*.fig.json) と、自己完結 HTML ビューア (*.html) を集める。
    dp_* はレポートに掲載しない。
    """
    comparison_dir = Path(comparison_dir)
    figs = [
        p for p in collect_fig_jsons(comparison_dir)
        if not asset_stem(p).startswith("dp_")
    ]
    playbacks = [
        p
        for p in comparison_dir.rglob("*" + _PLAYBACK_SUFFIX)
        if asset_stem(p) in selfcontained_html
    ]
    return sorted([*figs, *playbacks], key=lambda p: str(p))
