#!/usr/bin/env python3
"""Stage 11: comparison/ 配下の図スペック (*.fig.json) を 1 枚の単一 report.html に束ねる.

パイプライン (step4〜step10) は `comparison/` 配下の複数サブディレクトリ
(figures/, nstep/<tag>/, cases/overlay/, param_sweep/, curve_diag/) に
多数の **plotly 図スペック (`*.fig.json` = データ + レイアウト)** と再生ビューア
(`trajectory_playback.html`)、Markdown レポートを散らして出力する。本ステージは
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
- 自己完結ビューア (trajectory_playback.html 等) も同様に gzip+base64 の
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
from pathlib import Path
import re
import warnings

from .lib._collection import CROSS_DIR_NAME, discover_collection
from .lib._fig_io import FIG_SUFFIX, collect_fig_jsons
from .lib._inline_assets import gzip_b64, plotly_js_script
from .lib._params_utils import _INFO_YAML, _SIM_YAML
from .lib._plotly_utils import FIG_HEIGHTS, IFRAME_PAD
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

# --- 画像キャプション (ファイル名 stem → 日本語説明) ----------------------------
# step4〜step13 が出力する既知の図（拡張子非依存の stem で引く）。
# 未登録ファイルはファイル名を人間可読化してフォールバック。
CAPTIONS: dict[str, str] = {
    # step4: figures/
    "trajectory_with_map": "軌跡比較（地図背景付き・インタラクティブ）",
    "trajectory_xy": "軌跡比較（地図なし・インタラクティブ）",
    "trajectory_playback": "軌跡再生ビューア（時刻同期/位置同期シークバー・速度矢印・追従ズーム・DP計画軌跡）",
    "lon_lat_model": "縦横モデル検証ビューア（実機：運動方程式〔1次遅れ+自転車モデル〕の前方積算と、加速度/速度/ステア/ヨーレート/横Gの観測・指令を重ね描き。地図にシミュレーション軌跡も重畳。T/τ/k_us/β つまみ調整・ステア源切替）",
    "velocity": "速度時系列",
    "acceleration": "加速度時系列",
    "steering": "ステア角時系列",
    "velocity_vs_distance": "走行距離基準 速度（pacing 差を除いた早期停止の露出）",
    "steering_vs_distance": "走行距離基準 ステア角",
    # step8: figures/
    "dp_real_vs_sim": "DiffusionPlanner 出力軌跡 実機 vs sim",
    "dp_vs_actual": "DP 計画速度 vs 実際速度",
    "dp_vs_final_traj": "実機 DP 出力 vs 最終 planning（optimizer 補正）",
    # step5: nstep/<tag>/
    "overview": "誤差概観（N=1）",
    "map_distribution": "誤差の位置分布（地図上・N=1 / N=max・インタラクティブ）",
    # step6: cases/overlay/
    "cascade_error_overlay": "全ケース カスケード誤差 重ね描き（N=1）",
    "error_growth_overlay": "全ケース 誤差成長 重ね描き",
    "rmse_heatmap": "RMSE ヒートマップ（case × horizon 俯瞰）",
    "growth_relative": "相対誤差成長（reference 比・dynamics 差の分離）",
    # step7: param_sweep/ (個別図のキャプションは _caption_for の正規表現で導出)
    "_overview_sensitivity": "スイープ感度オーバービュー（改善率ランキング + 正規化 RMSE カーブ）",
    # step13: cross_dataset/
    "cross_closed_loop_heatmap": "dataset × sim run: closed-loop 軌跡乖離・完走率行列",
    "cross_normalized_bars": "dataset 横断 正規化 mean/worst 集約（ロバスト性ランキング）",
    "coverage_overview": "dataset 走行特性カバレッジ（速度域・加減速・曲率域・走行距離の偏り）",
    "loo_stability": "leave-one-out 安定性（除外 DS × case の score 変化）",
}

# 再生ビューア等、plotly でない自己完結 HTML を埋め込む際の高さ [px]。
# 生成側 (step4) と共有する lib._plotly_utils.FIG_HEIGHTS + IFRAME_PAD から導出する。
_IFRAME_HEIGHTS: dict[str, int] = {stem: h + IFRAME_PAD for stem, h in FIG_HEIGHTS.items()}
_IFRAME_HEIGHT_DEFAULT = 650
# plotly 図 div の高さを fig.json の layout.height から確保する際の最終フォールバック [px]。
_FIG_HEIGHT_DEFAULT = 600

# 収集対象。plotly 図スペック (*.fig.json) と、自己完結 HTML ビューア (*.html)。
_PLAYBACK_SUFFIX = ".html"

# iframe で埋め込む「自己完結 HTML ビューア」の stem 集合。canvas 独自 JS で
# 外部参照を持たないものに限定する（plotly standalone HTML は plotly.min.js を相対参照
# するため srcdoc 内で壊れる。それらは *.fig.json へ変換し本文に直接描画する）。
_SELFCONTAINED_HTML: set[str] = {"trajectory_playback", "lon_lat_model"}

# report.html がこのサイズを超えたら警告する [MB]。
_SIZE_WARN_MB = 150


@dataclass(frozen=True)
class ReportDataset:
    """report.html に束ねる 1 データセット分の表示エントリ。"""

    dataset_id: str
    comparison_dir: Path
    scenario_yaml: Path | None = None   # auto_scenario.yaml (ラベル導出 + 実行構成埋め込み)
    label: str | None = None            # None なら _dataset_label() で導出
    config_files: tuple[tuple[str, Path], ...] = field(default=())  # per-DS 追加設定 YAML


def _scenario_date(scenario_yaml: Path | None) -> str:
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


def _dataset_label(entry: ReportDataset) -> str:
    """セレクタ・見出しに使う人間可読ラベル (UUID は不親切なので日付を併記)。"""
    if entry.label:
        return entry.label
    short = entry.dataset_id[:8]
    date = _scenario_date(entry.scenario_yaml)
    return f"{short} ｜ {date}" if date else short


def _asset_stem(rel: Path) -> str:
    """アセット相対パスから拡張子（.fig.json / .html）を除いた stem を返す。"""
    name = rel.name
    for suf in (FIG_SUFFIX, _PLAYBACK_SUFFIX):
        if name.endswith(suf):
            return name[: -len(suf)]
    return rel.stem


def _is_fig_json(rel: Path) -> bool:
    """plotly 図スペック (*.fig.json) か（self-contained HTML ビューアと区別）。"""
    return rel.name.endswith(FIG_SUFFIX)


# --- 概念セクション定義 ---------------------------------------------------------
# (key, タイトル, 1 行説明)。表示順はこのリスト順。"other" は未分類フォールバック。
_CATEGORIES: list[tuple[str, str, str]] = [
    (
        "real_analysis",
        "1. 実機 rosbag 解析",
        "実機走行ログ（SSOT）のみから抽出する特性・車両パラメータ同定。sim は介在しない。",
    ),
    (
        "dp",
        "2. プランナ出力比較（DiffusionPlanner）",
        "DiffusionPlanner の出力軌跡を、実機の実走・最終 planning・sim 出力と比較する。",
    ),
    (
        "ol_nstep",
        "3. N-step オープンループ比較",
        "各開始点で実機状態にリセットし、車両モデルを N ステップ連続予測（free-running rollout）して"
        "実機との差を評価する。N=1 が毎ステップリセットの 1 ステップ予測（累積誤差を排除）、"
        "N>1 で dynamics 差の累積が顕在化する。",
    ),
    (
        "closed_loop",
        "4. シナリオ クローズループ比較",
        "auto-scenario を Autoware+シミュレータで closed-loop 実行し、実機との軌跡・速度・操舵乖離を比較する。",
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

# step4 が figures/ に出力する closed-loop 比較図（実機 vs sim）の stem。dp_*・brake 同定図は
# 同じ figures/ に混在するため、ディレクトリではなくこの明示リスト + curveN_* パターンで判定する。
_CLOSED_LOOP_STEMS: set[str] = {
    "velocity",
    "velocity_vs_distance",
    "acceleration",
    "steering",
    "steering_vs_distance",
    "trajectory_with_map",
    "trajectory_xy",
    "trajectory_playback",
}

# 取り込む Markdown レポート: (comparison/ からの相対パス, 見出し, 所属カテゴリ)。
_MARKDOWN_REPORTS: list[tuple[str, str, str]] = [
    ("report.md", "比較レポート（step4: report.md）", "closed_loop"),
    ("param_sweep/param_sweep_summary.md", "パラメータ同定サマリ（step7: param_sweep_summary.md）", "real_analysis"),
    ("cases/cases_summary.md", "ケース集約サマリ（step6: cases_summary.md）", "ol_nstep"),
]


def _caption_for(stem: str) -> str:
    """画像 stem（拡張子除去済み）に対応する日本語キャプションを返す。"""
    if stem in CAPTIONS:
        return CAPTIONS[stem]
    # step13 の horizon 別行列 (cross_nstep_heatmap_n<N>)
    m = re.match(r"cross_nstep_heatmap_n(\d+)$", stem)
    if m:
        return f"dataset × case: open-loop N={m.group(1)} 終端誤差行列"
    # param_sweep の個別図 (step7): <param>_sweep / pair_<a>_<b>
    m = re.match(r"(.+)_sweep$", stem)
    if m:
        return f"{m.group(1)} グリッド sweep 同定"
    m = re.match(r"pair_(.+)$", stem)
    if m:
        return f"2D ペア sweep（{m.group(1)}）"
    # error_timeseries_overlay_n<N> パターン照合（step6、horizon 別の重ね描き）
    m = re.match(r"error_timeseries_overlay_n(\d+)$", stem)
    if m:
        return f"全ケース 誤差時系列 重ね描き（N={m.group(1)}）"
    # フォールバック: アンダースコアを空白に
    return stem.replace("_", " ")


def _classify(rel: Path) -> str:
    """図の相対パス (comparison/ 基準) を概念セクションキーへ分類する。

    figures/ 配下にはクローズループ図・DP 図が混在するため、
    ディレクトリだけでなく stem でも判定する。
    """
    top = rel.parts[0] if len(rel.parts) > 1 else "."
    stem = _asset_stem(rel)

    if stem.startswith("dp_"):
        return "dp"
    if stem == "lon_lat_model":
        return "real_analysis"
    if top == "param_sweep":
        return "real_analysis"
    if top in {"nstep", "cases"}:
        return "ol_nstep"
    if stem in _CLOSED_LOOP_STEMS:
        return "closed_loop"
    return "other"


def _render_markdown(text: str) -> str:
    """Markdown を HTML 化する。markdown パッケージが無ければ <pre> でフォールバック。

    `pymdownx.arithmatex`（generic）で LaTeX 数式を MathJax 用 span（`\\(…\\)` / `\\[…\\]`）に
    退避する。これにより `$$a_target$$` の `_` が `<em>` 化する等の Markdown 破壊を避ける
    （MathJax の delimiter 設定は build_html の <head> と一致させること）。拡張が無い環境では
    数式拡張のみ外して描画する（既存3レポートは `$` 非含有のため影響なし）。
    """
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
    except (ImportError, ValueError):  # pymdownx 不在等
        return _md.markdown(text, extensions=base)


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


def _slug(text: str) -> str:
    """HTML id/name 用に安全な文字へ変換する。"""
    return re.sub(r"[^A-Za-z0-9_-]+", "-", text).strip("-").lower()


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
            scenario_name: str = "") -> str:
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
    stem = _asset_stem(rel)
    text = (comparison_dir / rel).read_text(encoding="utf-8", errors="replace")
    if _is_fig_json(rel):
        fig_id = f"fig-{ns}-" + _slug(fname)
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
    fig_id = f"vf-{ns}-" + _slug(fname)
    height = _IFRAME_HEIGHTS.get(stem, _IFRAME_HEIGHT_DEFAULT)
    return (
        f"<figure>{cap_html}"
        f"<div class='viewer-fig pending' id='{fig_id}' style='height:{height}px' "
        f"data-title='{html.escape(caption_text)}'></div>"
        f"<script type='application/gzip+html+base64' class='viewersrc' "
        f"data-target='{fig_id}'>{gzip_b64(text)}</script>"
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
    per_case: dict[str, dict[str, Path]], comparison_dir: Path, ns: str, cat: str
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
        group = _slug(f"casesync-{ns}-{cat}-{pt}")
        block_cases = [c for c in cases if pt in per_case[c]]
        out.append("<div class='casesync'>")
        out.append(
            f"<h3>{html.escape(caption)} <span class='fname'>{html.escape(pt)}</span></h3>"
        )
        out.append("<span class='casesync-label'>ケース切替:</span>")
        for i, c in enumerate(block_cases):
            rid = f"{group}-{_slug(c)}"
            checked = " checked" if i == 0 else ""
            out.append(
                f"<input type='radio' name='{group}' id='{rid}' class='cr-{_slug(c)}'{checked}>"
            )
            out.append(f"<label for='{rid}'>{html.escape(c)}</label>")
        for c in block_cases:
            out.append(
                f"<div class='tabpanel case-{_slug(c)}'>"
                f"{_figure(per_case[c][pt], comparison_dir, ns, caption=c)}</div>"
            )
        out.append("</div>")
    return out


def _render_category_images(
    rels: list[Path], comparison_dir: Path, ns: str, cat: str = "", scenario_name: str = ""
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
            per_case.setdefault(case, {})[_asset_stem(r)] = r

    out: list[str] = [
        _figure(r, comparison_dir, ns, scenario_name=scenario_name) for r in flat
    ]
    if per_case:
        out.extend(_render_case_tabs(per_case, comparison_dir, ns, cat))
    return out


def _casesync_css(case_tags: set[str]) -> str:
    """ケースごとのタブ表示規則を生成する（ケース slug 単位、ブロック間排他はラジオ name で閉じる）。

    case slug の class はデータセット間で共有できる（排他は ns 入りのラジオ name で閉じている）
    ため、全 DS の case タグ和集合で 1 回だけ生成する。
    """
    return "\n".join(
        f".casesync > input.cr-{_slug(c)}:checked ~ .tabpanel.case-{_slug(c)}"
        "{ display: block; }"
        for c in _sorted_cases(list(case_tags))
    )


def _collect_figures(comparison_dir: Path) -> list[Path]:
    """comparison/ 配下の図（*.fig.json + 自己完結ビューア *.html）を集める。

    plotly 図スペックと、再生ビューア (trajectory_playback.html 等の自己完結 HTML) を
    対象とする。report.html / plotly.min.js は元々 comparison/ の外なので入らない。
    """
    figs = collect_fig_jsons(comparison_dir)
    playbacks = [
        p
        for p in comparison_dir.rglob("*" + _PLAYBACK_SUFFIX)
        if _asset_stem(p) in _SELFCONTAINED_HTML
    ]
    return sorted([*figs, *playbacks], key=lambda p: str(p))


# レポート冒頭に置く分析パイプライン解説（README.ja.md の表と同期）。
_PIPELINE_INTRO = """
<details class="section" open id="sec-pipeline">
<summary>分析パイプライン（12 段階）</summary>
<p class="sec-desc">評価ノードが実機ログ抽出から HTML/notebook 集約まで順に実行するパイプライン
（Stage 1〜12 はデータセット単位、Stage 13 は複数データセットの collection 単位）。
本レポートは各 stage の成果物（図・Markdown）をカテゴリ別に束ねたもの。
<a class="toplink" href="#top">↑ 先頭</a></p>
<table>
<thead><tr><th>Stage</th><th>名称</th><th>役割 / 主な成果物</th></tr></thead>
<tbody>
<tr><td>1</td><td>実機ログ抽出 (step1_make_lite)</td><td>input_bag から必要トピックを抽出し real.lite を生成</td></tr>
<tr><td>2</td><td>scenario 自動生成 (step2_bag_to_scenario)</td><td>実機 bag + 地図から OpenSCENARIO (auto_scenario.yaml) を生成</td></tr>
<tr><td>3</td><td>closed-loop シム実行 (step3_run_sims)</td><td>auto_scenario + sim_runs.yaml で sim を回し sim lite を生成</td></tr>
<tr><td>4</td><td>実機 + sim 比較解析 (step4_compare_logs)</td><td>速度・ステア・軌跡を N-way 重ね描き (report.md・metrics_closed_loop.json・図スペック)</td></tr>
<tr><td>5</td><td>VehicleModel N-step オープンループ解析 (step5_analyze_nstep)</td><td>real.lite + cases.yaml の各ケースで free-running rollout の終端誤差を評価 (nstep/&lt;tag&gt;/)</td></tr>
<tr><td>6</td><td>ケース集約解析 (step6_analyze_cases)</td><td>全ケースの N-step 誤差を横断集約 (cases_summary.md・cases_metrics.json・overlay)</td></tr>
<tr><td>7</td><td>パラメータ sweep 同定 (step7_sweep_params)</td><td>車両モデル各パラメータを sweep し終端誤差最小値を同定 (param_sweep_summary.md)</td></tr>
<tr><td>8</td><td>DP 軌跡比較 (step8_compare_dp_trajectory)</td><td>DiffusionPlanner 出力軌跡を実機 vs sim で比較 (dp_*)</td></tr>
<tr><td>11</td><td>HTML レポート生成 (step11_build_html_report)</td><td>図スペック・Markdown・設定 YAML を 1 枚に束ねた単一レポート (report.html)。マルチ DS では collection 全体 + 横断サマリーを束ねる</td></tr>
<tr><td>12</td><td>notebook 生成 (step12_build_notebook)</td><td>各図を plotly で再描画 + 生 CSV からの再解析セルを備えた開発者向け notebook (report.ipynb)</td></tr>
<tr><td>13</td><td>データセット横断分析 (step13_cross_dataset)</td><td>collection 内全 DS の metrics JSON を再集計 (モデル×DS 行列・正規化集約・カバレッジ・LOO 安定性)</td></tr>
</tbody>
</table>
</details>
"""


# 車両制御モデルの数式・座標系・定数を解説する固定ドキュメント（リポジトリ同梱・YAML と同じ
# 固定パス読み）。MathJax で数式を組版するため Markdown 中の LaTeX をそのまま流す。
_MODEL_DOC = Path(__file__).parent / "docs" / "vehicle_model.ja.md"


# MathJax (tex-svg) を CDN から読み込み、Markdown 中の LaTeX 数式を組版する。delimiter は
# _render_markdown の pymdownx.arithmatex(generic) 出力（inline \(…\) / display \[…\]）に合わせる。
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
    本文は _render_markdown（MathJax 対応）で HTML 化する。
    """
    if not _MODEL_DOC.exists():
        return ""
    try:
        md_html = _render_markdown(_MODEL_DOC.read_text(encoding="utf-8"))
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


def _render_config_details(config_files: list[tuple[str, Path]]) -> str:
    """設定 YAML 群を <details class='cfg-file'> の連結 HTML にする（存在するもののみ）。"""
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
        anchor = "md-cross-" + _slug(md.stem)
        toc.append(f"<li class='toc-md'><a href='#{anchor}'>{html.escape(md.stem)}</a></li>")
        body.append(
            f"<details class='md-report' open id='{anchor}'>"
            f"<summary>{html.escape(md.stem)}（step13）</summary>"
            f"{_render_markdown(md.read_text(encoding='utf-8'))}</details>"
        )
    for fig in figs:
        body.append(_figure(fig.relative_to(cross_dir), cross_dir, "cross"))
    body.append("</details>")
    return toc, "".join(body)


def _render_dataset_report(
    entry: ReportDataset, ns: str, scenario_name: str, *, multi: bool
) -> tuple[list[str], str, int, set[str]]:
    """1 データセット分のセクション群を描画する。

    返り値: (toc 項目, body HTML, 図枚数, case タグ集合)。
    multi=True では <section class='dataset-report' data-ds=… hidden> で包み、toc 項目に
    ds-only クラス + data-ds を付ける (セレクタ選択時のみ表示)。単一 DS では包みを出さず
    常時表示 (従来同等の見た目)。
    """
    images = _collect_figures(entry.comparison_dir)
    rels_all = [img.relative_to(entry.comparison_dir) for img in images]
    by_cat: dict[str, list[Path]] = {}
    for rel in rels_all:
        by_cat.setdefault(_classify(rel), []).append(rel)
    case_tags = {c for rel in rels_all if (c := _case_of(rel)) is not None}

    md_by_cat: dict[str, list[tuple[str, str, str]]] = {}
    for rel, title, cat in _MARKDOWN_REPORTS:
        path = entry.comparison_dir / rel
        if path.exists():
            anchor = f"md-{ns}-" + _slug(rel)
            md_by_cat.setdefault(cat, []).append(
                (anchor, title, _render_markdown(path.read_text(encoding="utf-8")))
            )

    active_cats = [c for c in _CATEGORY_ORDER if c in by_cat or c in md_by_cat]

    ds_attr = f" class='toc-sec ds-only' data-ds='{ns}' hidden" if multi else " class='toc-sec'"
    md_attr = f" class='toc-md ds-only' data-ds='{ns}' hidden" if multi else " class='toc-md'"
    toc: list[str] = []
    body: list[str] = []
    if multi:
        label = _dataset_label(entry)
        body.append(f"<section class='dataset-report' data-ds='{ns}' id='ds-{ns}' hidden>")
        body.append(
            f"<p class='ds-head'>データセット {html.escape(label)}"
            f"<span class='fname'>{html.escape(entry.dataset_id)}</span>"
            f" ／ 図 {len(images)} 枚</p>"
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
        if cat in by_cat:
            body.extend(_render_category_images(
                by_cat[cat], entry.comparison_dir, ns, cat=cat, scenario_name=scenario_name
            ))
        else:
            body.append("<p class='empty'>（このセクションに該当する図はありませんでした）</p>")
        for anchor, mtitle, md_html in md_by_cat.get(cat, []):
            toc.append(f"<li{md_attr}><a href='#{anchor}'>{html.escape(mtitle)}</a></li>")
            body.append(
                f"<details class='md-report' open id='{anchor}'>"
                f"<summary>{html.escape(mtitle)}</summary>{md_html}</details>"
            )
        body.append("</details>")

    # per-DS 実行構成 (auto_scenario + 追加設定)
    per_ds_cfg: list[tuple[str, Path]] = []
    if entry.scenario_yaml is not None:
        per_ds_cfg.append(("シナリオ (auto-scenario)", entry.scenario_yaml))
    per_ds_cfg.extend(entry.config_files)
    cfg_html = _render_config_details(per_ds_cfg)
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
) -> str:
    """datasets (1 つ以上) と横断分析出力から単一 report.html の文字列を組み立てる。

    単一 DS でもマルチ DS でも同じパス: `len(datasets) > 1` のときだけセレクタと
    dataset-report の包みを出力し、cross_dir に成果物が実在するときだけ横断セクションを出す。
    shared_config_files は全 DS 共通の設定 YAML (シミュレータモデル/車両情報) で末尾に 1 回。
    """
    multi = len(datasets) > 1
    # ns: dataset_id 先頭 8 文字の slug。衝突時のみフル ID。
    raw_ns = [_slug(e.dataset_id[:8]) or f"ds{i}" for i, e in enumerate(datasets)]
    ns_list = [
        ns if raw_ns.count(ns) == 1 else _slug(e.dataset_id)
        for ns, e in zip(raw_ns, datasets)
    ]

    cross_toc, cross_html = _render_cross_section(cross_dir)

    ds_tocs: list[str] = []
    ds_bodies: list[str] = []
    total_figs = 0
    all_case_tags: set[str] = set()
    for entry, ns in zip(datasets, ns_list):
        toc_items, body_html, n_figs, case_tags = _render_dataset_report(
            entry, ns, scenario_name, multi=multi
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
            f"<option value='{ns}'>{html.escape(_dataset_label(e))}</option>"
            for e, ns in zip(datasets, ns_list)
        ]
        ds_bar = (
            "<div class='ds-bar'>データセット: "
            f"<select id='ds-select'>{''.join(options)}</select></div>"
        )

    # --- 目次 ---
    toc: list[str] = ["<nav class='toc'><h2>目次</h2>"]
    toc.append("<a class='toc-top' href='#top'>↑ 先頭へ</a><ul>")
    toc.append("<li class='toc-sec'><a href='#sec-pipeline'>分析パイプライン（13 段階）</a></li>")
    doc_html = _render_doc_section()
    if doc_html:
        toc.append("<li class='toc-sec'><a href='#sec-model-doc'>車両制御モデル（数式・座標系・定数）</a></li>")
    toc.extend(cross_toc)
    toc.extend(ds_tocs)
    shared_cfg_html = _render_config_details(shared_config_files or [])
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
{_PIPELINE_INTRO}
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
    return sorted(out, key=lambda d: (_scenario_date(d.scenario_yaml), d.dataset_id))


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
    args = parser.parse_args()

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
        out_path = collection_dir / "report.html"
    else:
        cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
        comparison_dir = cfg.out_dir
        comparison_dir.mkdir(parents=True, exist_ok=True)
        scenarios_dir = cfg.base_dir / "scenarios"
        per_ds_cfg: list[tuple[str, Path]] = []
        if cfg.sim_runs_config:
            per_ds_cfg.append(("sim 実行設定 (sim_runs)", cfg.sim_runs_config))
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
    )
    out_path.write_text(html_text, encoding="utf-8")
    size_mb = out_path.stat().st_size / 1024 / 1024
    print(f"  保存: {out_path} (データセット {len(datasets)} / {size_mb:.1f} MB)")
    if size_mb > _SIZE_WARN_MB:
        warnings.warn(
            f"report.html が {size_mb:.0f} MB ({_SIZE_WARN_MB} MB 超)。ブラウザの初回ロードが"
            "重くなるため、データセット数か per-DS の図数の削減を検討してください"
        )


if __name__ == "__main__":
    main()
