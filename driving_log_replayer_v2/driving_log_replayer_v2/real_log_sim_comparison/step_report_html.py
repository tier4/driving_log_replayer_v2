#!/usr/bin/env python3
"""Stage 11: comparison/ 配下の図スペック (*.fig.json) を 1 枚の単一 report.html に束ねる.

パイプライン (step4〜step10) は `comparison/` 配下の複数サブディレクトリ
(figures/, nstep/<tag>/, cases/overlay/, curve_diag/) に
多数の **plotly 図スペック (`*.fig.json` = データ + レイアウト)** と再生ビューア
(`viewer.html`)、Markdown レポートを散らして出力する。本ステージは
それらを走査し、全アセットを**外部参照なしで 1 枚に埋め込んだ**目次付き `report.html` を
生成する。

**マルチデータセット対応**: `--collection-dir` を渡すと collection (collect_datasets.py の
収集先) 内の全データセットを 1 枚に束ね、ページ右上のデータセットセレクタで切り替える。
デフォルト表示は「0. データセット横断サマリー」(step13_cross_dataset の出力) で、個別 DS の
詳細は選択時のみ表示される。単一データセット (従来 CLI) も**同じコードパス**
(`datasets=[entry]` の 1 要素リスト) で動き、セレクタ・横断セクションが無い従来通りの
見た目になる。

埋め込み形式 (容量・ロード対策):
- 各図は `<div class='plotly-fig'>` プレースホルダ + 直後の
  `<script type='application/gzip+json+base64'>` に **gzip+base64 圧縮した図スペック**。
  `IntersectionObserver` で可視化時に DecompressionStream('gzip') で展開 →
  `Plotly.newPlot` する (Chrome 80+ / Firefox 113+ 前提。非対応はエラーメッセージ表示)。
- 自己完結ビューア (viewer.html) も同様に gzip+base64 の
  `<script type='application/gzip+html+base64'>` で遅延埋め込みし、可視化時に
  `<iframe srcdoc>` を生成する (直書き iframe は非表示でも load 時にパース+実行され、
  マルチ DS では致命的なため)。
- plotly.min.js は <head> に 1 回だけインライン (オフライン動作維持)。
- 折りたたみ <details> / 非選択ケースタブ / 非選択データセットは display:none で IO が
  発火しないため、details の toggle・タブ/DS 切替でも未描画図を reveal-render する。

レポートは出力ディレクトリではなく **比較の概念** でセクション分けする。未知の図は捨てず
「その他」へ回す。末尾に実行構成 (設定 YAML) を折りたたみで埋め込む (per-DS 分は各 DS
セクション内、全 DS 共通分は末尾に 1 回)。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import html
import json
import re
from pathlib import Path
import warnings

from .lib._collection import CROSS_DIR_NAME, discover_collection
from .lib._fig_io import collect_fig_jsons
from .lib._inline_assets import gzip_b64, plotly_js_script
from .lib._params_utils import _INFO_YAML, _SIM_YAML
from .lib._report_html import (
    asset_stem,
    collect_report_assets,
    dataset_label,
    is_fig_json,
    render_config_details,
    render_markdown,
    scenario_date,
    slug,
)
from .lib._plotly_utils import FIG_HEIGHTS, IFRAME_PAD
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

# --- 画像キャプション (ファイル名 stem → 日本語説明) ----------------------------
# step4〜step13 が出力する既知の図（拡張子非依存の stem で引く）。
# 未登録ファイルはファイル名を人間可読化してフォールバック。
CAPTIONS: dict[str, str] = {
    # step4: figures/
    "viewer": "走行ログ統合ビューア（軌跡比較／縦横モデル検証）",
    # step5: nstep/<tag>/
    "overview": "誤差概観（N=1）",
    "map_distribution": "誤差の位置分布（地図上・N=1 / N=max・インタラクティブ）",
    # step6: cases/overlay/
    "cascade_error_overlay": "全ケース カスケード誤差 重ね描き（N=1）",
    "error_growth_overlay": "全ケース 誤差成長 重ね描き",
    # step6: cases/physical_validity/ (物理妥当性検証、Conditions.cases の N-way スイープとは独立な軸)
    "long_fit": "縦方向モデルフィット（実測 vs 同定値 vs チューニング値、路面勾配補正込み）",
    "steer_fit": "操舵モデルフィット（実測 vs 同定値 vs チューニング値）",
    # step13: cross_dataset/
    "cross_closed_loop_heatmap": "dataset × sim run: closed-loop 軌跡乖離・完走率行列",
    "cross_normalized_bars": "dataset 横断 正規化 mean/worst 集約（ロバスト性ランキング）",
    "coverage_overview": "dataset 走行特性カバレッジ（速度域・加減速・曲率域・走行距離の偏り）",
    "loo_stability": "leave-one-out 安定性（除外 DS × case の score 変化）",
    "steer_diff_overview": "steer 制御不感帯分析（per-DS: |steer−steer_des| 分布・速度層別・dead_band 対応）",
    # step13: cross_dataset/ (物理妥当性検証、dataset 横断)
    "cross_physical_validity_long": "dataset 横断 縦方向アクチュエータ遅れ同定（横断最小二乗法 + 路面勾配補正、最良/最悪データセット時系列）",
    "cross_physical_validity_steer": "dataset 横断 操舵追従同定（best/worst データセット時系列 + チューニング値比較）",
    # step13: perfect tracking
    "cross_perfect_tracking_box": "操舵理想追従 N-step 横方向誤差（モデル構造限界・ホライズン別）",
    "cross_perfect_tracking_traj": "操舵理想追従 代表データセット軌跡比較（GT vs 自転車モデル）",
    "cross_long_perf_box": "加速度理想追従 N-step 縦方向変位誤差（モデル構造限界・ホライズン別）",
    "cross_long_perf_growth": "加速度理想追従 ドリフト成長カーブ（速度誤差・変位誤差）",
    "cross_long_perf_map": "加速度理想追従 変位誤差の地図上分布",
}

# 再生ビューア等、plotly でない自己完結 HTML を埋め込む際の高さ [px]。
# 生成側 (step4) と共有する lib._plotly_utils.FIG_HEIGHTS + IFRAME_PAD から導出する。
_IFRAME_HEIGHTS: dict[str, int] = {stem: h + IFRAME_PAD for stem, h in FIG_HEIGHTS.items()}
_IFRAME_HEIGHT_DEFAULT = 650
# plotly 図 div の高さを fig.json の layout.height から確保する際の最終フォールバック [px]。
_FIG_HEIGHT_DEFAULT = 600

# iframe で埋め込む「自己完結 HTML ビューア」の stem 集合。canvas 独自 JS で
# 外部参照を持たないものに限定する（plotly standalone HTML は plotly.min.js を相対参照
# するため srcdoc 内で壊れる。それらは *.fig.json へ変換し本文に直接描画する）。
_SELFCONTAINED_HTML: set[str] = {"viewer"}

# report.html がこのサイズを超えたら警告する [MB]。
_SIZE_WARN_MB = 150


@dataclass(frozen=True)
class ReportDataset:
    """report.html に束ねる 1 データセット分の表示エントリ。"""

    dataset_id: str
    comparison_dir: Path
    scenario_yaml: Path | None = None   # auto_scenario.yaml (ラベル導出 + 実行構成埋め込み)
    label: str | None = None            # None なら dataset_label() で導出
    config_files: tuple[tuple[str, Path], ...] = field(default=())  # per-DS 追加設定 YAML


# --- 概念セクション定義 ---------------------------------------------------------
# (key, タイトル, 1 行説明)。表示順はこのリスト順。"other" は未分類フォールバック。
_CATEGORIES: list[tuple[str, str, str]] = [
    (
        "pre_estimation_deviation",
        "1. 推定前：実車とシミュレーションのズレ",
        "パラメータ推定を行う前の、理想追従（操舵完全一致と仮定）における自車位置やモデル構造の限界によるズレを提示します。",
    ),
    (
        "parameter_estimation",
        "2. パラメータ推定結果",
        "実機ログから遅延などを固定（または目視調整）した上で、最小二乗法で同定した車両パラメータとそのフィッティング精度を提示します。",
    ),
    (
        "post_estimation_residual",
        "3. 推定後：シミュレーション残差の提示",
        "パラメータ推定後における、直近10ステップ（ホライズン別）等の予測誤差や、位置（x, y）の変位誤差の成長度合い・分布を提示します。",
    ),
    (
        "closed_loop_comparison",
        "4. 最終的な Closed Loop シミュレーション残差",
        "最終同定パラメータを用いて closed-loop シミュレーションを実行した際の、実機との軌跡・速度・操舵の乖離（カーブや加速の誤差）を提示します。",
    ),
    (
        "other",
        "その他",
        "上記いずれにも分類されなかった図（新規追加図など）。",
    ),
]
_CATEGORY_TITLES: dict[str, str] = {key: title for key, title, _ in _CATEGORIES}
_CATEGORY_DESCS: dict[str, str] = {key: desc for key, _, desc in _CATEGORIES}
_CATEGORY_ORDER: list[str] = [key for key, _, _ in _CATEGORIES]

# step4 が figures/ に出力する closed-loop 比較図（実機 vs sim）の stem。
# 統合 viewer の軌跡比較タブが closed-loop 比較を担う。
_CLOSED_LOOP_STEMS: set[str] = {"viewer"}

# 取り込む Markdown レポート: (comparison/ からの相対パス, 見出し, 所属カテゴリ)。
# セクション内で「図の前」に描画する Markdown。プロットを追認 (§2–§4) の位置へ送り、
# 主軸の数式的導出 (§1, テキストのみ) を図より上に置くために使う。
_MARKDOWN_REPORTS_PRE: list[tuple[str, str, str]] = []
# セクション内で「図の後」に描画する Markdown (既定)。
_MARKDOWN_REPORTS: list[tuple[str, str, str]] = [
    ("cases/cases_summary.md", "ケース集約サマリ（step6: cases_summary.md）", "post_estimation_residual"),
]


def _caption_for(stem: str) -> str:
    """画像 stem（拡張子除去済み）に対応する日本語キャプションを返す。"""
    if stem in CAPTIONS:
        return CAPTIONS[stem]
    # step13 の horizon 別行列 (cross_nstep_heatmap_n<N>)
    m = re.match(r"cross_nstep_heatmap_n(\d+)$", stem)
    if m:
        return f"dataset × case: open-loop N={m.group(1)} 終端誤差行列"
    # フォールバック: アンダースコアを空白に
    return stem.replace("_", " ")


def _classify(rel: Path) -> str:
    """比較アセットの相対パスを概念セクションキーへ分類する。"""
    top = rel.parts[0] if len(rel.parts) > 1 else "."
    stem = asset_stem(rel)

    if stem.startswith("dp_"):
        return "other"

    if stem in {"cross_perfect_tracking_box", "cross_perfect_tracking_traj"}:
        return "pre_estimation_deviation"

    if stem in {
        "cross_physical_validity_long",
        "cross_physical_validity_steer",
        "viewer",
    }:
        return "parameter_estimation"

    if stem in {"cross_long_perf_box", "cross_long_perf_growth", "cross_long_perf_map"}:
        return "post_estimation_residual"
    if top in {"nstep", "cases"}:
        return "post_estimation_residual"

    if stem in _CLOSED_LOOP_STEMS:
        return "closed_loop_comparison"
    if stem in {
        "cross_closed_loop_heatmap",
        "cross_normalized_bars",
        "coverage_overview",
        "loo_stability",
        "steer_diff_overview",
    }:
        return "closed_loop_comparison"

    return "other"


_STYLE = """
:root { --fg:#1a1a1a; --muted:#666; --border:#ddd; --accent:#2563eb; --bg:#fff; }
* { box-sizing: border-box; }
body { font-family: -apple-system, "Segoe UI", "Hiragino Sans", "Noto Sans CJK JP", sans-serif;
       color: var(--fg); background: var(--bg); margin: 0; line-height: 1.6; }
.layout { display: flex; align-items: flex-start; }
/* 目次は既定で隠した固定オーバーレイ（横方向を占有しない＝図を全幅で使う）。左上の
   ☰ ボタン（純 CSS チェックボックス）でスライド表示する。表示中もフロー外なので main 幅は変わらない。 */
.toc-toggle { display: none; }
.toc-btn { position: fixed; top: 10px; left: 10px; z-index: 1001; background: var(--accent); color: #fff;
           padding: 4px 11px; border-radius: 4px; cursor: pointer; font-size: 0.85rem; user-select: none;
           box-shadow: 0 1px 3px rgba(0,0,0,0.3); }
nav.toc { position: fixed; left: 0; top: 0; width: 260px; min-width: 260px; height: 100vh; overflow-y: auto;
          padding: 3rem 1rem 1.5rem; border-right: 1px solid var(--border); background: #fafafa;
          font-size: 0.9rem; z-index: 1000; transform: translateX(-100%); transition: transform 0.2s;
          box-shadow: 2px 0 8px rgba(0,0,0,0.15); }
.toc-toggle:checked ~ .layout nav.toc { transform: translateX(0); }
nav.toc h2 { font-size: 1rem; margin: 0 0 0.5rem; }
nav.toc ul { list-style: none; padding-left: 0.5rem; margin: 0.25rem 0; }
nav.toc a { color: var(--accent); text-decoration: none; }
nav.toc a:hover { text-decoration: underline; }
main { flex: 1; padding: 2rem 2.5rem 2rem 3.5rem; min-width: 0; }
header.page { margin-bottom: 2rem; }
header.page h1 { margin: 0 0 0.25rem; }
header.page .meta { color: var(--muted); font-size: 0.9rem; }
nav.toc li.toc-sec { font-weight: 600; margin-top: 0.35rem; }
nav.toc li.toc-md { font-size: 0.82rem; padding-left: 1rem; font-weight: 400; }
nav.toc .toc-top { display: inline-block; margin-bottom: 0.6rem; font-size: 0.82rem; }
nav.toc li[hidden] { display: none; }

/* データセットセレクタ（マルチ DS 時のみ出力）。右上固定でスクロール中も切替可能。 */
.ds-bar { position: fixed; top: 10px; right: 10px; z-index: 1001; background: #fff;
          border: 1px solid var(--border); border-radius: 4px; padding: 4px 10px;
          font-size: 0.85rem; box-shadow: 0 1px 3px rgba(0,0,0,0.2); }
.ds-bar select { font-size: 0.85rem; max-width: 320px; }
section.dataset-report[hidden] { display: none; }
.ds-head { color: var(--muted); font-size: 0.9rem; border-left: 3px solid var(--accent);
           padding-left: 0.6rem; margin: 1.5rem 0 0.5rem; }

/* セクションは <details> で折りたたみ可能（既定 open）。 */
details.section { margin-bottom: 2.2rem; border-top: 2px solid var(--border); padding-top: 0.4rem; }
details.section > summary { cursor: pointer; font-size: 1.35rem; font-weight: 700; padding: 0.3rem 0; }
details.section > summary:hover { color: var(--accent); }
.sec-desc { color: var(--muted); font-size: 0.92rem; margin: 0.4rem 0 1.2rem; }
.sec-desc .toplink { margin-left: 0.6rem; font-size: 0.82rem; white-space: nowrap; }
figure { margin: 0 0 2rem; }
/* plotly 図 div / ビューア用 div。高さは図種別ごとに指定する。 */
figure .plotly-fig { width: 100%; min-height: 120px; border: 1px solid var(--border);
                     border-radius: 4px; background: #fff; }
figure .viewer-fig { width: 100%; min-height: 120px; }
figure .viewer-fig iframe { width: 100%; height: 100%; border: 1px solid var(--border);
                            border-radius: 4px; background: #fff; }
.plotly-fig.pending::before, .viewer-fig.pending::before {
  content: "図を描画中…"; display: block; padding: 2rem;
  color: var(--muted); font-size: 0.85rem; text-align: center; }
.fig-error { padding: 2rem; color: #b91c1c; font-size: 0.85rem; text-align: center; }
figcaption { margin-bottom: 0.4rem; font-weight: 600; }
figcaption .fname { font-weight: 400; color: var(--muted); font-size: 0.82rem; margin-left: 0.5rem; }
details.md-report { border-top: 1px dashed var(--border); margin-top: 2rem; padding-top: 0.5rem; }
details.md-report > summary { cursor: pointer; font-weight: 600; margin-bottom: 0.5rem; }
details.md-report > summary:hover { color: var(--accent); }
.md-report table { border-collapse: collapse; margin: 1rem 0; }
.md-report th, .md-report td { border: 1px solid var(--border); padding: 0.3rem 0.6rem; }
.md-report th { background: #f0f0f0; }
.md-fallback { background: #f6f6f6; padding: 1rem; overflow-x: auto; font-size: 0.85rem; }
.empty { color: var(--muted); font-style: italic; }

/* 純 CSS ケースタブ（プロット単位）。各プロットブロック先頭に独立したケースラジオを置き、
   そのブロック内のパネルだけを切り替える（他プロットには影響しない）。JS 不使用・オフライン可。 */
.casesync { margin: 0.4rem 0 2.2rem; }
.casesync > h3 { margin: 0 0 0.5rem; }
.casesync > .casesync-label { font-size: 0.85rem; color: var(--muted); margin-right: 0.4rem; }
.casesync > input { position: absolute; opacity: 0; pointer-events: none; }
.casesync > label { display: inline-block; padding: 0.3rem 0.9rem; margin: 0 0.3rem 0.9rem 0;
                    border: 1px solid var(--border); border-radius: 4px; cursor: pointer;
                    font-size: 0.88rem; color: var(--accent); background: #fafafa; user-select: none; }
.casesync > input:checked + label { background: var(--accent); color: #fff; border-color: var(--accent); }
.tabpanel { display: none; }

/* 実行構成（設定ファイル）の生テキスト表示 */
.cfg-pre { background: #f6f6f6; padding: 1rem; overflow-x: auto; font-size: 0.82rem;
           white-space: pre; max-height: 60vh; overflow-y: auto; }
details.cfg-file { border-top: 1px dashed var(--border); margin-top: 1rem; padding-top: 0.5rem; }
details.cfg-file > summary { cursor: pointer; font-weight: 600; }
details.cfg-file > summary:hover { color: var(--accent); }
"""


def _heading_html(caption: str | None, title_text: str | None, stem: str,
                  scenario_name: str) -> str:
    """figcaption 見出しの HTML を返す。

    plot 内タイトルは凡例・サブプロットタイトルと重なって隠れやすいので出さず、その内容を
    この HTML 見出しに統合する。case タブは渡された caption（ケース名）を優先。図タイトルが
    あればそれを見出しにする（冗長な「シナリオ名<br>」接頭辞は除去、<br>/<sub> はそのまま活かす）。
    無ければ CAPTIONS の定型キャプションにフォールバック。
    """
    if caption is not None:
        return html.escape(caption)
    if title_text:
        t = title_text
        prefix = f"{scenario_name}<br>"
        if scenario_name and t.startswith(prefix):
            t = t[len(prefix):]
        return t  # build_fig_* が生成した内部 HTML（<br>/<sub>）。外部入力ではないので素通し。
    return html.escape(_caption_for(stem))


def _figure(rel: Path, comparison_dir: Path, ns: str, caption: str | None = None,
            scenario_name: str = "", no_embed_viewers: bool = False,
            initial_tab: str | None = None) -> str:
    """1 図分の <figure> HTML を返す（gzip+base64 埋め込み・遅延描画）。

    - `*.fig.json` (plotly) → `<div class='plotly-fig pending'>` + 直後の
      `<script type='application/gzip+json+base64' class='figspec'>`。
      plot 内 `layout.title` は除去し、内容を figcaption 見出しへ統合する（重なって隠れるため）。
    - `*.html` (playback 等) → `<div class='viewer-fig pending'>` + 同様の
      `<script class='viewersrc'>`。描画時に glue JS が iframe srcdoc を生成する。

    `ns` はデータセット名前空間 (DS 間で同名 rel が重複するため id に織り込む)。
    base64 は HTML 安全な文字のみで `</script>` エスケープは不要。
    """
    fname = rel.as_posix()
    stem = asset_stem(rel)
    text = (comparison_dir / rel).read_text(encoding="utf-8", errors="replace")
    if asset_stem(rel) == "viewer" and initial_tab:
        text = re.sub(
            r'("initial_tab"\s*:\s*)"(?:playback|model)"',
            rf'\1"{initial_tab}"',
            text,
            count=1,
        )
    if is_fig_json(rel):
        fig_id = f"fig-{ns}-" + slug(fname)
        # spec を 1 度だけパースし、(1) 高さ確保用の layout.height を取り、(2) plot 内タイトルを
        # 抽出して layout から除去（HTML 見出しへ統合）する。
        title_text = None
        try:
            spec_obj = json.loads(text)
        except ValueError:
            spec_obj = None
        fig_h = None
        if isinstance(spec_obj, dict):
            layout = spec_obj.get("layout") or {}
            fig_h = layout.get("height")
            t = layout.get("title")
            if isinstance(t, dict):
                title_text = t.get("text")
            if "title" in layout:
                layout.pop("title", None)
                text = json.dumps(spec_obj, separators=(",", ":"), ensure_ascii=False)
        cap_html = (
            f"<figcaption>{_heading_html(caption, title_text, stem, scenario_name)}"
            f"<span class='fname'>{html.escape(fname)}</span></figcaption>"
        )
        # プレースホルダ div に図の高さを**事前に確保**する（autosize=true でコンテナ高に縮むため）。
        height = int(fig_h or FIG_HEIGHTS.get(stem) or _FIG_HEIGHT_DEFAULT)
        return (
            f"<figure>{cap_html}"
            f"<div class='plotly-fig pending' id='{fig_id}' style='height:{height}px'></div>"
            f"<script type='application/gzip+json+base64' class='figspec' "
            f"data-target='{fig_id}'>{gzip_b64(text)}</script>"
            f"</figure>"
        )
    # 自己完結 HTML（playback 等）。直書き iframe は非表示でも load 時に実行されるため、
    # gzip+base64 で遅延埋め込みし可視化時に iframe srcdoc を生成する。
    caption_text = caption if caption is not None else _caption_for(stem)
    cap_html = (
        f"<figcaption>{html.escape(caption_text)}"
        f"<span class='fname'>{html.escape(fname)}</span></figcaption>"
    )
    fig_id = f"vf-{ns}-" + slug(fname)
    height = _IFRAME_HEIGHTS.get(stem, _IFRAME_HEIGHT_DEFAULT)

    if no_embed_viewers:
        abs_uri = (comparison_dir / rel).resolve().absolute().as_uri()
        return (
            f"<figure>{cap_html}"
            f"<div style='padding: 16px; border: 2px dashed #ccc; border-radius: 6px; text-align: center; background: #fafafa; margin: 12px 0;'>"
            f"  <a href='{abs_uri}' target='_blank' style='font-weight: bold; color: #2b4a8b; text-decoration: underline; font-size: 1.05rem;'>"
            f"    🗁 インタラクティブビューアを別タブで開く"
            f"  </a>"
            f"  <div style='font-size: 0.85rem; color: #666; margin-top: 6px;'>ローカルファイルパス: {html.escape(fname)}</div>"
            f"</div>"
            f"</figure>"
        )

    return (
        f"<figure>{cap_html}"
        f"<div class='viewer-fig pending' id='{fig_id}' style='height:{height}px' "
        f"data-title='{html.escape(caption_text)}'></div>"
        f"<script type='application/gzip+html+base64' class='viewersrc' "
        f"data-target='{fig_id}' data-type='{stem}'>{gzip_b64(text)}</script>"
        f"</figure>"
    )


def _case_of(rel: Path) -> str | None:
    """nstep/<case>/<file> のケースタグを返す（nstep 図でなければ None）。"""
    if rel.parts[0] == "nstep" and len(rel.parts) > 2:
        return rel.parts[1]
    return None


def _sorted_cases(tags: list[str]) -> list[str]:
    """ケースタグ表示順: baseline（参照）を先頭、残りはアルファベット順。"""
    return sorted(tags, key=lambda t: (t != "baseline", t))


def _render_case_tabs(
    per_case: dict[str, dict[str, Path]], comparison_dir: Path, ns: str, cat: str,
    no_embed_viewers: bool = False
) -> list[str]:
    """プロット種別ごとにブロックを作り、ブロックごとに独立したケースタブを付ける。

    per_case: {case_tag: {stem: rel}}。各ブロック先頭に専用のケースラジオを置き、
    選んだケースの図がそのブロック内だけで切り替わる（他プロットには影響しない）。
    ラジオの name に ns を含め、データセット間でグループが衝突しないようにする。
    """
    cases = _sorted_cases(list(per_case.keys()))
    plot_types = sorted({stem for files in per_case.values() for stem in files})

    out: list[str] = []
    for pt in plot_types:
        caption = _caption_for(pt)
        group = slug(f"casesync-{ns}-{cat}-{pt}")
        block_cases = [c for c in cases if pt in per_case[c]]
        out.append("<div class='casesync'>")
        out.append(
            f"<h3>{html.escape(caption)} <span class='fname'>{html.escape(pt)}</span></h3>"
        )
        out.append("<span class='casesync-label'>ケース切替:</span>")
        for i, c in enumerate(block_cases):
            rid = f"{group}-{slug(c)}"
            checked = " checked" if i == 0 else ""
            out.append(
                f"<input type='radio' name='{group}' id='{rid}' class='cr-{slug(c)}'{checked}>"
            )
            out.append(f"<label for='{rid}'>{html.escape(c)}</label>")
        for c in block_cases:
            out.append(
                f"<div class='tabpanel case-{slug(c)}'>"
                f"{_figure(per_case[c][pt], comparison_dir, ns, caption=c, no_embed_viewers=no_embed_viewers)}</div>"
            )
        out.append("</div>")
    return out


def _render_category_images(
    rels: list[Path], comparison_dir: Path, ns: str, cat: str = "", scenario_name: str = "",
    no_embed_viewers: bool = False
) -> list[str]:
    """カテゴリ内の図群を描画する。

    nstep/<case>/ の図は「プロット種別ごとのブロック ＋ ブロック単位のケースタブ」で描画する。
    それ以外の図 (cases/overlay 等) は通常の figure として先に並べる。
    """
    flat = [r for r in rels if _case_of(r) is None]
    per_case: dict[str, dict[str, Path]] = {}
    for r in rels:
        case = _case_of(r)
        if case is not None:
            per_case.setdefault(case, {})[asset_stem(r)] = r

    initial_tab = "model" if cat == "parameter_estimation" else "playback"
    out: list[str] = [
        _figure(
            r, comparison_dir, ns, scenario_name=scenario_name,
            no_embed_viewers=no_embed_viewers, initial_tab=initial_tab,
        )
        for r in flat
    ]
    if per_case:
        out.extend(_render_case_tabs(per_case, comparison_dir, ns, cat, no_embed_viewers=no_embed_viewers))
    return out


def _casesync_css(case_tags: set[str]) -> str:
    """ケースごとのタブ表示規則を生成する（ケース slug 単位、ブロック間排他はラジオ name で閉じる）。

    case slug の class はデータセット間で共有できる（排他は ns 入りのラジオ name で閉じている）
    ため、全 DS の case タグ和集合で 1 回だけ生成する。
    """
    return "\n".join(
        f".casesync > input.cr-{slug(c)}:checked ~ .tabpanel.case-{slug(c)}"
        "{ display: block; }"
        for c in _sorted_cases(list(case_tags))
    )


_PIPELINE_INTRO = ""


# 車両制御モデルの数式・座標系・定数を解説する固定ドキュメント（リポジトリ同梱・YAML と同じ
# 固定パス読み）。MathJax で数式を組版するため Markdown 中の LaTeX をそのまま流す。
_MODEL_DOC = Path(__file__).parent / "docs" / "vehicle_model.ja.md"


# MathJax (tex-svg) を CDN から読み込み、Markdown 中の LaTeX 数式を組版する。delimiter は
# render_markdown の pymdownx.arithmatex(generic) 出力（inline \(…\) / display \[…\]）に合わせる。
# CDN 参照のため数式描画にはネット接続が要る（plotly.js はオフライン用にインライン済み）。
_MATHJAX_HEAD = (
    "<script>"
    "window.MathJax={tex:{inlineMath:[['\\\\(','\\\\)']],displayMath:[['\\\\[','\\\\]']]},"
    "svg:{fontCache:'global'}};"
    "</script>"
    "<script async src='https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js'></script>"
)


def _render_doc_section() -> str:
    """車両モデル解説ドキュメント (docs/vehicle_model.ja.md) を固定セクションとして埋め込む。

    ファイルが無ければ空文字（防御的）。_PIPELINE_INTRO と同じ「固定セクション」扱いで、
    本文は render_markdown（MathJax 対応）で HTML 化する。
    """
    if not _MODEL_DOC.exists():
        return ""
    try:
        md_html = render_markdown(_MODEL_DOC.read_text(encoding="utf-8"))
    except OSError:
        return ""
    return (
        "<details class='section' open id='sec-model-doc'>"
        "<summary>車両制御モデル（数式・座標系・定数）</summary>"
        "<p class='sec-desc'>シミュレータの車両モデル "
        "(delay_steer_acc_geared_wo_fall_guard) の運動方程式・座標系・定数定義と、"
        "現在値（仕様）vs モデル値（best_normal）の対比。"
        "<a class='toplink' href='#top'>↑ 先頭</a></p>"
        f"{md_html}</details>"
    )


# 図スペック/ビューアを遅延描画する glue JS。各プレースホルダ div は直後の
# <script type='application/gzip+...+base64'> に gzip+base64 圧縮データを持つ。
# IntersectionObserver で可視化時に DecompressionStream で展開して Plotly.newPlot /
# iframe srcdoc 生成するが、折りたたみ <details> / 非選択ケースタブ / 非選択データセットは
# display:none で IO が発火しないため、details の toggle・ケースタブ切替・DS 切替時にも
# 配下の未描画図を reveal-render する。
_RENDER_GLUE = """
<script>
(function () {
  function specScriptFor(div) {
    return div.parentNode.querySelector("script[data-target='" + div.id + "']");
  }
  async function inflate(b64) {
    var bin = atob(b64.trim());
    var bytes = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    if (!("DecompressionStream" in window)) {
      throw new Error("このブラウザは DecompressionStream 非対応です (Chrome 80+ / Firefox 113+ が必要)");
    }
    var stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream("gzip"));
    return await new Response(stream).text();
  }
  var templates = {};
  async function getTemplate(type) {
    if (templates[type]) return templates[type];
    var el = document.getElementById("tpl-" + type);
    if (!el) return "";
    var t = await inflate(el.textContent);
    templates[type] = t;
    return t;
  }
  async function render(div) {
    if (!div || div.dataset.rendered) return;
    var s = specScriptFor(div);
    if (!s) return;
    div.dataset.rendered = "1";
    var text;
    try { text = await inflate(s.textContent); }
    catch (e) {
      div.classList.remove("pending");
      div.innerHTML = "<div class='fig-error'>図の展開に失敗: " + String(e).replace(/</g, "&lt;") + "</div>";
      return;
    }
    div.classList.remove("pending");
    if (s.classList.contains("viewersrc")) {
      var iframe = document.createElement("iframe");
      iframe.title = div.dataset.title || "";
      iframe.srcdoc = text;
      div.appendChild(iframe);
      return;
    }
    if (!window.Plotly) return;
    var spec;
    try { spec = JSON.parse(text); } catch (e) { return; }
    window.Plotly.newPlot(div, spec.data || [], spec.layout || {}, {responsive: true});
  }
  function renderVisibleWithin(root) {
    (root || document).querySelectorAll(
      ".plotly-fig:not([data-rendered]), .viewer-fig:not([data-rendered])"
    ).forEach(function (div) {
      if (div.offsetParent !== null) render(div);  // display:none でない＝描画対象
    });
  }
  var io = ("IntersectionObserver" in window)
    ? new IntersectionObserver(function (entries) {
        entries.forEach(function (e) { if (e.isIntersecting) { render(e.target); io.unobserve(e.target); } });
      }, {rootMargin: "200px"})
    : null;
  function observeAll() {
    document.querySelectorAll(
      ".plotly-fig:not([data-rendered]), .viewer-fig:not([data-rendered])"
    ).forEach(function (div) {
      if (io) io.observe(div); else render(div);
    });
  }
  function resizeAll() {
    if (!window.Plotly) return;
    document.querySelectorAll(".plotly-fig[data-rendered]").forEach(function (d) {
      try { window.Plotly.Plots.resize(d); } catch (e) {}
    });
  }
  window.addEventListener("load", function () { observeAll(); renderVisibleWithin(document); });
  // 折りたたみセクションを開いた / ケースタブを切替えた時に、display:none で IO 未発火だった
  // 図を描画する（描画済みは resize）。
  document.addEventListener("toggle", function (e) {
    if (e.target.tagName === "DETAILS" && e.target.open) {
      setTimeout(function () { renderVisibleWithin(e.target); resizeAll(); }, 0);
    }
  }, true);
  document.querySelectorAll(".casesync > input").forEach(function (r) {
    r.addEventListener("change", function () {
      setTimeout(function () { renderVisibleWithin(r.closest(".casesync")); resizeAll(); }, 0);
    });
  });

  // --- データセットセレクタ（マルチ DS 時のみ DOM に存在） ---
  var sel = document.getElementById("ds-select");
  function applyDataset(v) {
    document.querySelectorAll("section.dataset-report").forEach(function (w) {
      w.hidden = (v === "" || w.dataset.ds !== v);
    });
    document.querySelectorAll("nav.toc .ds-only").forEach(function (li) {
      li.hidden = (li.dataset.ds !== v);
    });
    var cross = document.getElementById("sec-cross");
    if (cross) cross.open = (v === "");  // DS 選択時は横断サマリーを畳む（手動再展開は自由）
    // 表示復帰した未描画図は IO (observe 済み) が viewport 進入時に描画する。ここでは
    // 描画済み図の幅再計算のみ行う（renderVisibleWithin を呼ぶと選択 DS の全図を一括描画
    // してしまうため呼ばない）。
    setTimeout(function () { resizeAll(); }, 0);
  }
  if (sel) {
    sel.addEventListener("change", function () {
      applyDataset(sel.value);
      try { history.replaceState(null, "", sel.value ? "#ds=" + sel.value : "#"); } catch (e) {}
    });
    var m = location.hash.match(/^#ds=([A-Za-z0-9_-]+)/);
    if (m) {
      sel.value = m[1];
      if (sel.value !== m[1]) sel.value = "";  // 不明な DS ハッシュは集約ビューへ
    }
    applyDataset(sel.value);
  }
})();
</script>
"""


def _render_cross_section(cross_dir: Path | None) -> tuple[list[str], str]:
    """「0. データセット横断サマリー」セクション (toc 項目, body HTML) を返す。

    step13_cross_dataset の出力 (cross_dataset/*.fig.json + cross_summary.md) から構築する。
    図も md も無ければ空を返しセクションごと省略 (単一 DS は自然にこの経路)。
    """
    if cross_dir is None or not cross_dir.is_dir():
        return [], ""
    figs = collect_fig_jsons(cross_dir)
    mds = sorted(cross_dir.glob("*.md"))
    if not figs and not mds:
        return [], ""

    toc: list[str] = ["<li class='toc-sec'><a href='#sec-cross'>0. データセット横断サマリー</a></li>"]
    body: list[str] = [
        "<details class='section' open id='sec-cross'>",
        "<summary>0. データセット横断サマリー</summary>",
        "<p class='sec-desc'>全データセット横断のモデル×DS 誤差行列・正規化集約・走行特性カバレッジ・"
        "ランキング安定性。個別データセットの詳細はページ右上のセレクタで切り替える。"
        "<a class='toplink' href='#top'>↑ 先頭</a></p>",
    ]
    for md in mds:
        anchor = "md-cross-" + slug(md.stem)
        toc.append(f"<li class='toc-md'><a href='#{anchor}'>{html.escape(md.stem)}</a></li>")
        body.append(
            f"<details class='md-report' open id='{anchor}'>"
            f"<summary>{html.escape(md.stem)}（step13）</summary>"
            f"{render_markdown(md.read_text(encoding='utf-8'))}</details>"
        )
    # 物理妥当性検証 (cross_physical_validity_*) は他の step13 図と混ざると読みづらいため
    # 専用の見出しに分けて描画する (図の集合・順序は変えず、表示上の区切りのみ)。
    _PV_STEMS = {"cross_physical_validity_long", "cross_physical_validity_steer"}
    model_figs = [f for f in figs if asset_stem(f.relative_to(cross_dir)) not in _PV_STEMS]
    pv_figs = [f for f in figs if asset_stem(f.relative_to(cross_dir)) in _PV_STEMS]
    for fig in model_figs:
        body.append(_figure(fig.relative_to(cross_dir), cross_dir, "cross"))
    if pv_figs:
        body.append("<h3>物理的妥当性検証（縦・操舵、実測同定 vs チューニング値）</h3>")
        for fig in pv_figs:
            body.append(_figure(fig.relative_to(cross_dir), cross_dir, "cross"))
    body.append("</details>")
    return toc, "".join(body)


def _render_dataset_report(
    entry: ReportDataset, ns: str, scenario_name: str, *, multi: bool,
    no_embed_viewers: bool = False
) -> tuple[list[str], str, int, set[str]]:
    """1 データセット分のセクション群を描画する。

    返り値: (toc 項目, body HTML, 図枚数, case タグ集合)。
    multi=True では <section class='dataset-report' data-ds=… hidden> で包み、toc 項目に
    ds-only クラス + data-ds を付ける (セレクタ選択時のみ表示)。単一 DS では包みを出さず
    常時表示 (従来同等の見た目)。
    """
    images = collect_report_assets(entry.comparison_dir, _SELFCONTAINED_HTML)
    rels_all = [img.relative_to(entry.comparison_dir) for img in images]
    by_cat: dict[str, list[Path]] = {}
    for rel in rels_all:
        by_cat.setdefault(_classify(rel), []).append(rel)
        if asset_stem(rel) == "viewer":
            by_cat.setdefault("parameter_estimation", []).append(rel)
            by_cat.setdefault("closed_loop_comparison", []).append(rel)
    for rels in by_cat.values():
        rels[:] = list(dict.fromkeys(rels))
    case_tags = {c for rel in rels_all if (c := _case_of(rel)) is not None}

    if multi and no_embed_viewers:
        label = dataset_label(entry.dataset_id, scenario_yaml=entry.scenario_yaml, label=entry.label)
        rel_report_path = f"runs/{entry.dataset_id}/result_archive/real_log_sim_comparison/report.html"
        ds_attr = f" class='toc-sec ds-only' data-ds='{ns}' hidden"
        toc = [f"<li{ds_attr}><a href='#ds-{ns}'>個別詳細レポートリンク</a></li>"]
        body_html = f"""<section class='dataset-report' data-ds='{ns}' id='ds-{ns}' hidden>
  <p class='ds-head'>データセット {html.escape(label)}
    <span class='fname'>{html.escape(entry.dataset_id)}</span>
  </p>
  <div style='padding: 2rem; border: 2px dashed var(--border); border-radius: 8px; background: #fafafa; margin: 1.5rem 0; text-align: center;'>
    <h3 style='margin-top: 0; font-size: 1.2rem; color: var(--fg);'>詳細レポート (個別)</h3>
    <p style='color: var(--muted); font-size: 0.9rem; margin-bottom: 1.5rem;'>
      このデータセットに関するすべての可視化グラフ、シミュレーション比較、およびインタラクティブ再生ビューアは個別のレポートファイルに収録されています。
    </p>
    <a href='{rel_report_path}' target='_blank' 
       style='display: inline-block; padding: 0.75rem 1.5rem; background: var(--accent); color: white; text-decoration: none; font-weight: bold; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); transition: all 0.2s;'>
      🗁 個別レポート（グラフ・ビューア付き）を別タブで開く
    </a>
    <div style='font-size: 0.8rem; color: var(--muted); margin-top: 1rem;'>相対パス: <code style='background: #eee; padding: 2px 6px; border-radius: 4px;'>{rel_report_path}</code></div>
  </div>
</section>"""
        return toc, body_html, 0, case_tags


    def _collect_md(reports: list[tuple[str, str, str]]) -> dict[str, list[tuple[str, str, str]]]:
        out: dict[str, list[tuple[str, str, str]]] = {}
        for rel, title, cat in reports:
            path = entry.comparison_dir / rel
            if path.exists():
                anchor = f"md-{ns}-" + slug(rel)
                out.setdefault(cat, []).append(
                    (anchor, title, render_markdown(path.read_text(encoding="utf-8")))
                )
        return out

    md_pre_by_cat = _collect_md(_MARKDOWN_REPORTS_PRE)  # 図の前に描画 (§1 導出など)
    md_by_cat = _collect_md(_MARKDOWN_REPORTS)          # 図の後に描画 (既定)

    active_cats = [
        c for c in _CATEGORY_ORDER if c in by_cat or c in md_by_cat or c in md_pre_by_cat
    ]

    ds_attr = f" class='toc-sec ds-only' data-ds='{ns}' hidden" if multi else " class='toc-sec'"
    md_attr = f" class='toc-md ds-only' data-ds='{ns}' hidden" if multi else " class='toc-md'"
    toc: list[str] = []
    body: list[str] = []
    if multi:
        label = dataset_label(entry.dataset_id, scenario_yaml=entry.scenario_yaml, label=entry.label)
        body.append(f"<section class='dataset-report' data-ds='{ns}' id='ds-{ns}' hidden>")
        body.append(
            f"<p class='ds-head'>データセット {html.escape(label)}"
            f"<span class='fname'>{html.escape(entry.dataset_id)}</span>"
            f" ／ 図 {len(images)} 枚</p>"
        )
    def _emit_md(items: list[tuple[str, str, str]]) -> None:
        for anchor, mtitle, md_html in items:
            toc.append(f"<li{md_attr}><a href='#{anchor}'>{html.escape(mtitle)}</a></li>")
            body.append(
                f"<details class='md-report' open id='{anchor}'>"
                f"<summary>{html.escape(mtitle)}</summary>{md_html}</details>"
            )

    for cat in active_cats:
        sec_id = f"sec-{ns}-{cat}"
        toc.append(f"<li{ds_attr}><a href='#{sec_id}'>{html.escape(_CATEGORY_TITLES[cat])}</a></li>")
        body.append(f"<details class='section' open id='{sec_id}'>")
        body.append(f"<summary>{html.escape(_CATEGORY_TITLES[cat])}</summary>")
        body.append(
            f"<p class='sec-desc'>{html.escape(_CATEGORY_DESCS[cat])}"
            f"<a class='toplink' href='#top'>↑ 先頭</a></p>"
        )
        _emit_md(md_pre_by_cat.get(cat, []))  # 図の前: §1 主軸 (数式的導出・テキスト)
        if cat in by_cat:
            body.extend(_render_category_images(  # 追認プロット (実機ログ根拠・スイープ曲線)
                by_cat[cat], entry.comparison_dir, ns, cat=cat, scenario_name=scenario_name,
                no_embed_viewers=no_embed_viewers
            ))
        elif cat not in md_pre_by_cat:
            body.append("<p class='empty'>（このセクションに該当する図はありませんでした）</p>")
        _emit_md(md_by_cat.get(cat, []))      # 図の後: §2–§4 追認・整合
        body.append("</details>")

    # per-DS 実行構成 (auto_scenario + 追加設定)
    per_ds_cfg: list[tuple[str, Path]] = []
    if entry.scenario_yaml is not None:
        per_ds_cfg.append(("シナリオ (auto-scenario)", entry.scenario_yaml))
    per_ds_cfg.extend(entry.config_files)
    cfg_html = render_config_details(per_ds_cfg)
    if cfg_html:
        sec_id = f"sec-{ns}-config"
        toc.append(f"<li{ds_attr}><a href='#{sec_id}'>実行構成（このデータセット）</a></li>")
        body.append(
            f"<details class='section' open id='{sec_id}'>"
            "<summary>実行構成（このデータセット）</summary>"
            "<p class='sec-desc'>この報告を生成した際のシナリオ・sim 実行設定。"
            "<a class='toplink' href='#top'>↑ 先頭</a></p>"
            f"{cfg_html}</details>"
        )

    if multi:
        body.append("</section>")
    return toc, "".join(body), len(images), case_tags


def build_html(
    datasets: list[ReportDataset],
    *,
    cross_dir: Path | None = None,
    shared_config_files: list[tuple[str, Path]] | None = None,
    scenario_name: str = "",
    no_embed_viewers: bool = False,
) -> str:
    """datasets (1 つ以上) と横断分析出力から単一 report.html の文字列を組み立てる。

    単一 DS でもマルチ DS でも同じパス: `len(datasets) > 1` のときだけセレクタと
    dataset-report の包みを出力し、cross_dir に成果物が実在するときだけ横断セクションを出す。
    shared_config_files は全 DS 共通の設定 YAML (シミュレータモデル/車両情報) で末尾に 1 回。
    """
    multi = len(datasets) > 1
    # ns: dataset_id 先頭 8 文字の slug。衝突時のみフル ID。
    raw_ns = [slug(e.dataset_id[:8]) or f"ds{i}" for i, e in enumerate(datasets)]
    ns_list = [
        ns if raw_ns.count(ns) == 1 else slug(e.dataset_id)
        for ns, e in zip(raw_ns, datasets)
    ]

    cross_toc, cross_html = _render_cross_section(cross_dir)

    ds_tocs: list[str] = []
    ds_bodies: list[str] = []
    total_figs = 0
    all_case_tags: set[str] = set()
    for entry, ns in zip(datasets, ns_list):
        toc_items, body_html, n_figs, case_tags = _render_dataset_report(
            entry, ns, scenario_name, multi=multi, no_embed_viewers=no_embed_viewers
        )
        ds_tocs.extend(toc_items)
        ds_bodies.append(body_html)
        total_figs += n_figs
        all_case_tags |= case_tags

    # --- セレクタ (マルチ DS のみ) ---
    ds_bar = ""
    if multi:
        options = ["<option value=''>全体（横断サマリー）</option>"]
        options += [
            f"<option value='{ns}'>{html.escape(dataset_label(e.dataset_id, scenario_yaml=e.scenario_yaml, label=e.label))}</option>"
            for e, ns in zip(datasets, ns_list)
        ]
        ds_bar = (
            "<div class='ds-bar'>データセット: "
            f"<select id='ds-select'>{''.join(options)}</select></div>"
        )

    # --- 目次 ---
    toc: list[str] = ["<nav class='toc'><h2>目次</h2>"]
    toc.append("<a class='toc-top' href='#top'>↑ 先頭へ</a><ul>")
    doc_html = _render_doc_section()
    if doc_html:
        toc.append("<li class='toc-sec'><a href='#sec-model-doc'>車両制御モデル（数式・座標系・定数）</a></li>")
    toc.extend(cross_toc)
    toc.extend(ds_tocs)
    shared_cfg_html = render_config_details(shared_config_files or [])
    if shared_cfg_html:
        toc.append("<li class='toc-sec'><a href='#sec-config-shared'>実行構成（共通）</a></li>")
    toc.append("</ul></nav>")

    # --- メタ情報 ---
    meta_bits = []
    if multi:
        meta_bits.append(f"データセット {len(datasets)}")
    elif scenario_name:
        meta_bits.append(f"シナリオ: {html.escape(scenario_name)}")
    meta_bits.append(f"図 {'計 ' if multi else ''}{total_figs} 枚")
    meta = " ／ ".join(meta_bits)
    empty_note = (
        ""
        if total_figs or cross_html
        else "<p class='empty'>comparison/ 配下に図 (*.fig.json) が見つかりませんでした。"
        "先に step4〜step10 を実行してください。</p>"
    )

    shared_cfg_section = ""
    if shared_cfg_html:
        shared_cfg_section = (
            "<details class='section' open id='sec-config-shared'>"
            "<summary>実行構成（共通）</summary>"
            "<p class='sec-desc'>全データセット共通のシミュレータモデル・車両パラメータ。"
            "<a class='toplink' href='#top'>↑ 先頭</a></p>"
            f"{shared_cfg_html}</details>"
        )

    sync_css = _casesync_css(all_case_tags)

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>real_log_sim_comparison レポート</title>
<style>{_STYLE}
{sync_css}</style>
{plotly_js_script()}
{_MATHJAX_HEAD}
</head>
<body>
<input type="checkbox" id="toc-toggle" class="toc-toggle">
<label for="toc-toggle" class="toc-btn" title="目次の表示/非表示">☰ 目次</label>
{ds_bar}
<div class="layout">
{''.join(toc)}
<main>
<header class="page" id="top">
  <h1>real_log_sim_comparison 比較レポート</h1>
  <div class="meta">{meta}</div>
</header>
{doc_html}
{empty_note}
{cross_html}
{''.join(ds_bodies)}
{shared_cfg_section}
{_RENDER_GLUE}
</main>
</div>
</body>
</html>
"""


def _datasets_from_collection(collection_dir: Path) -> list[ReportDataset]:
    """collection (collect_datasets.py の収集先) から ReportDataset 群を構築する。

    comparison/ を持つ DS のみ対象 (metrics 欠損 DS は cross_summary.md 側で明示される)。
    並びは (走行日付, dataset_id)。
    """
    out: list[ReportDataset] = []
    for e in discover_collection(collection_dir):
        if e.comparison_dir is None:
            continue
        scenario_yaml = (
            e.scenarios_dir / "auto_scenario.yaml" if e.scenarios_dir is not None else None
        )
        out.append(ReportDataset(
            dataset_id=e.dataset_id,
            comparison_dir=e.comparison_dir,
            scenario_yaml=scenario_yaml if scenario_yaml and scenario_yaml.is_file() else None,
        ))
    return sorted(out, key=lambda d: (scenario_date(d.scenario_yaml), d.dataset_id))


VERBOSE = False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="comparison/ 配下の全図スペック (*.fig.json)・Markdown・設定 YAML を"
        "束ねた単一 report.html を生成 (マルチ DS は --collection-dir)"
    )
    add_common_cli_arguments(parser)
    parser.add_argument(
        "--collection-dir",
        default="",
        help="collect_datasets.py の収集先 (collection root)。指定するとマルチ DS レポートを"
        "<collection>/report.html に生成する",
    )
    parser.add_argument(
        "--report-name",
        default="report.html",
        help="--collection-dir 使用時のレポートファイル名 (デフォルト: report.html)。"
        "open-loop 集約レポートは aggregate_report.html を推奨 (クラウド per-dataset の"
        "report.html と衝突しない)",
    )
    parser.add_argument(
        "--no-embed-viewers",
        action="store_true",
        help="自己完結再生ビューア (*.html) をインライン埋め込みせず、別タブで開くリンクにする (ファイルサイズ削減用)",
    )
    args = parser.parse_args()

    global VERBOSE
    VERBOSE = args.verbose
    if not VERBOSE:
        import warnings
        warnings.simplefilter('ignore')

    def _print(*args, **kwargs):
        if VERBOSE:
            print(*args, **kwargs)

    shared_config_files = [
        ("シミュレータモデル (simulator_model.param)", _SIM_YAML),
        ("車両情報 (vehicle_info.param)", _INFO_YAML),
    ]

    if args.collection_dir:
        collection_dir = Path(args.collection_dir)
        datasets = _datasets_from_collection(collection_dir)
        if not datasets:
            warnings.warn(f"collection に comparison/ を持つデータセットがありません: {collection_dir}")
        cross_dir = collection_dir / CROSS_DIR_NAME
        scenario_name = ""
        out_path = collection_dir / args.report_name
    else:
        cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
        comparison_dir = cfg.out_dir
        comparison_dir.mkdir(parents=True, exist_ok=True)
        scenarios_dir = cfg.base_dir / "scenarios"
        per_ds_cfg: list[tuple[str, Path]] = []
        if cfg.scenario_config:
            per_ds_cfg.append(("scenario 設定 (models/cases/sim_runs)", cfg.scenario_config))
        scenario_yaml = scenarios_dir / "auto_scenario.yaml"
        datasets = [ReportDataset(
            dataset_id=cfg.scenario_name or "dataset",
            comparison_dir=comparison_dir,
            scenario_yaml=scenario_yaml if scenario_yaml.is_file() else None,
            config_files=tuple(per_ds_cfg),
        )]
        cross_dir = None
        scenario_name = cfg.scenario_name
        out_path = cfg.base_dir / "report.html"
        cfg.base_dir.mkdir(parents=True, exist_ok=True)

    html_text = build_html(
        datasets,
        cross_dir=cross_dir,
        shared_config_files=shared_config_files,
        scenario_name=scenario_name,
        no_embed_viewers=args.no_embed_viewers,
    )
    out_path.write_text(html_text, encoding="utf-8")
    size_mb = out_path.stat().st_size / 1024 / 1024
    _print(f"  保存: {out_path} (データセット {len(datasets)} / {size_mb:.1f} MB)")
    if size_mb > _SIZE_WARN_MB:
        warnings.warn(
            f"report.html が {size_mb:.0f} MB ({_SIZE_WARN_MB} MB 超)。ブラウザの初回ロードが"
            "重くなるため、データセット数か per-DS の図数の削減を検討してください"
        )


if __name__ == "__main__":
    main()
