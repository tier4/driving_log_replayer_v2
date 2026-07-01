#!/usr/bin/env python3
"""Stage 12: comparison/ 配下の図スペック (*.fig.json) を束ねた report.ipynb を生成.

step11 が配布用の単一 report.html を作るのに対し、本ステージは**開発者が後から
インタラクティブに触れる** Jupyter notebook を生成する。各図は `plotly.io.read_json`
で読み込んで `.show()` 表示するため、ノートブックを開けばズーム・パン・hover でき、
セルを編集して軸や系列を試せる。さらに末尾に「再解析」セルを置き、`lib._figures` の
build_fig_* を import して生データ (nstep_delta.csv 等) から図を組み直す例を示す
（`lib._figures` は ROS 非依存なので rclpy 無しの素の kernel で動く）。

notebook は report.html と同じ階層 (result_archive/ = base_dir) に置き、図スペックは
`comparison/...` の相対パスで参照する。nbformat に依存せず、ipynb (JSON) を直接組み立てる
（追加依存を増やさない）。表示には plotly のみ必要。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import warnings

from .lib._fig_io import collect_fig_jsons
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config
from .step11_build_html_report import (
    _CATEGORY_DESCS,
    _CATEGORY_ORDER,
    _CATEGORY_TITLES,
    _asset_stem,
    _caption_for,
    _case_of,
    _classify,
)


def _md(text: str) -> dict:
    return {"cell_type": "markdown", "metadata": {}, "source": text}


def _code(src: str) -> dict:
    return {"cell_type": "code", "metadata": {}, "execution_count": None, "outputs": [], "source": src}


def _fig_cell(rel_from_nb: str, caption: str) -> tuple[dict, dict]:
    """1 図分の (見出し markdown, 描画 code) セルを返す。"""
    md = _md(f"#### {caption}\n\n`{rel_from_nb}`")
    code = _code(
        f'pio.read_json("{rel_from_nb}").show()'
    )
    return md, code


# 再解析セルの雛形（最初に見つかった nstep_delta.csv を生データから描き直す例）。
_REANALYSIS_TEMPLATE = """\
# === 再解析の例: 生データ (nstep_delta.csv) から図を組み直す ===
# lib._figures は ROS 非依存。ワークスペースを source 済み (driving_log_replayer_v2 が
# import 可能) ならそのまま動く。build_fig_* の引数を変えて軸や系列を試せる。
import pandas as pd
from driving_log_replayer_v2.real_log_sim_comparison.lib._figures import (
    build_fig_overview,
)

df = pd.read_csv("{csv_rel}")
fig = build_fig_overview(df[df["horizon"] == 1], params=None)  # limits_df= で横断軸統一も可
fig.show()
"""


def build_notebook(comparison_dir: Path, base_dir: Path, scenario_name: str,
                   images: list[Path] | None = None) -> dict:
    """report.ipynb の中身 (ipynb JSON dict) を組み立てる。

    図スペック (*.fig.json) を概念カテゴリ別に並べ、各図を read_json で描画するセルを作る。
    パスは base_dir (notebook の位置) からの相対。末尾に再解析セルを添える。
    """
    if images is None:
        images = collect_fig_jsons(comparison_dir)

    cells: list[dict] = []
    cells.append(_md(
        f"# real_log_sim_comparison 比較レポート (notebook)\n\n"
        f"シナリオ: **{scenario_name or '(不明)'}** ／ 図 {len(images)} 枚\n\n"
        "各図は `comparison/` 配下の図スペック (`*.fig.json` = データ + レイアウト) を "
        "plotly で描画したもの。セルを実行すると対話的に閲覧でき、編集して軸・系列を試せる。"
    ))
    cells.append(_code("import plotly.io as pio"))

    # カテゴリ別に分類
    rels = [p.relative_to(comparison_dir) for p in images]
    by_cat: dict[str, list[Path]] = {}
    for rel in rels:
        by_cat.setdefault(_classify(rel), []).append(rel)

    cmp_prefix = comparison_dir.relative_to(base_dir).as_posix()
    for cat in _CATEGORY_ORDER:
        if cat not in by_cat:
            continue
        cells.append(_md(f"## {_CATEGORY_TITLES[cat]}\n\n{_CATEGORY_DESCS[cat]}"))
        for rel in sorted(by_cat[cat], key=lambda p: str(p)):
            stem = _asset_stem(rel)
            caption = _caption_for(stem)
            case = _case_of(rel)
            if case is not None:
                caption = f"{caption} — ケース: {case}"
            rel_from_nb = f"{cmp_prefix}/{rel.as_posix()}"
            md, code = _fig_cell(rel_from_nb, caption)
            cells.extend([md, code])

    # 再解析セル（最初の nstep_delta.csv を使う）
    csvs = sorted(comparison_dir.rglob("nstep/*/nstep_delta.csv"))
    if csvs:
        csv_rel = f"{cmp_prefix}/{csvs[0].relative_to(comparison_dir).as_posix()}"
        cells.append(_md("## 再解析（生データから図を組み直す）"))
        cells.append(_code(_REANALYSIS_TEMPLATE.format(csv_rel=csv_rel)))

    return {
        "cells": cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="comparison/ 配下の図スペック (*.fig.json) を束ねた report.ipynb を生成"
    )
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    comparison_dir = cfg.out_dir
    base_dir = cfg.base_dir
    base_dir.mkdir(parents=True, exist_ok=True)

    images = collect_fig_jsons(comparison_dir)
    if not images:
        warnings.warn(f"{comparison_dir} 配下に図 (*.fig.json) が見つかりません。空の notebook を生成します")

    nb = build_notebook(comparison_dir, base_dir, cfg.scenario_name, images=images)
    out_path = base_dir / "report.ipynb"
    out_path.write_text(json.dumps(nb, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"  保存: {out_path} (図 {len(images)} 枚)")


if __name__ == "__main__":
    main()
