#!/usr/bin/env python3
"""Stage 11: comparison/ 配下の図スペック (*.fig.json) を 1 枚の単一 report.html に束ねる.

10 段階パイプライン (step4〜step10) は `comparison/` 配下の複数サブディレクトリ
(figures/, nstep/<tag>/, cases/overlay/, param_sweep/, brake_sweep/, curve_diag/) に
多数の **plotly 図スペック (`*.fig.json` = データ + レイアウト)** と再生ビューア
(`trajectory_playback.html`)、3 種の Markdown レポートを散らして出力する。本ステージは
それらを走査し、全アセットを**外部参照なしで 1 枚に埋め込んだ**目次付き `report.html` を
`result_archive/` 直下 (comparison/ の親) に生成する。

旧方式（commit 59fef44）は matplotlib SVG を base64 data-URI で詰め込み ~38MB に肥大した。
本方式は SVG を廃し、各図を **数値データ + plotly.js クライアント描画** に変える:
- `plotly.min.js` を <head> に **1 回だけ**インライン（オフライン動作維持）。
- 各図は `<div class='plotly-fig'>` プレースホルダ + 直後の
  `<script type='application/json'>` に図スペックを並置。
- `IntersectionObserver` で**ビューポート進入時に遅延 `Plotly.newPlot`**（数十図を一度に
  描画しない）。折りたたみ `<details>` / 非選択ケースタブは display:none で IO が発火しない
  ため、details の toggle・ケースタブ切替でも未描画図を reveal-render する。

レポートは出力ディレクトリではなく **比較の概念** でセクション分けする（読み手が
「何を何と比べた図か」で辿れるようにするため）。各図は (ディレクトリ, ファイル名 stem) の
組で分類する。未知の図は捨てず「その他」へ回す。末尾に実行構成 (シナリオ / sim_runs /
車両パラメータ YAML) を折りたたみで埋め込み、どの設定で生成した報告か追跡できるようにする。

再生ビューア (`trajectory_playback.html`) は独自 canvas JS の自己完結 HTML のため
`<iframe srcdoc>` で隔離埋め込みする（グローバル衝突回避）。`markdown` パッケージが
あれば Markdown を整形 HTML 化し、無ければ生テキストを <pre> で埋め込む。本ステージは
読み取り専用 (バンドル直下に report.html を 1 枚足すだけ)。
"""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import re
import warnings

from .lib._fig_io import FIG_SUFFIX, collect_fig_jsons
from .lib._inline_assets import escape_srcdoc, plotly_js_script
from .lib._params_utils import _INFO_YAML, _SIM_YAML
from .lib._plotly_utils import FIG_HEIGHTS, IFRAME_PAD
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

# --- 画像キャプション (ファイル名 stem → 日本語説明) ----------------------------
# step4〜step10 が出力する既知の図（拡張子非依存の stem で引く）。
# 未登録ファイルはファイル名を人間可読化してフォールバック。
CAPTIONS: dict[str, str] = {
    # step4: figures/
    "trajectory_with_map": "軌跡比較（地図背景付き・インタラクティブ）",
    "trajectory_xy": "軌跡比較（地図なし・インタラクティブ）",
    "trajectory_playback": "軌跡再生ビューア（時刻同期/位置同期シークバー・速度矢印・追従ズーム・DP計画軌跡）",
    "velocity": "速度時系列",
    "acceleration": "加速度時系列",
    "steering": "ステア角時系列",
    "velocity_vs_distance": "走行距離基準 速度（pacing 差を除いた早期停止の露出）",
    "steering_vs_distance": "走行距離基準 ステア角",
    "curves_closeup": "カーブ一覧図（全カーブ概観）",
    # step8: figures/
    "dp_real_vs_sim": "DiffusionPlanner 出力軌跡 実機 vs sim",
    "dp_vs_actual": "DP 計画速度 vs 実際速度",
    "dp_vs_final_traj": "実機 DP 出力 vs 最終 planning（optimizer 補正）",
    # step5: nstep/<tag>/
    "overview": "誤差概観（N=1）",
    "error_timeseries": "誤差時系列（horizon 別）",
    "error_vs_speed": "速度別 誤差散布（N=1 / N=max）",
    "error_growth": "誤差成長（horizon 別 RMSE）",
    "steering_analysis": "ステア分析（N=1）",
    "map_distribution": "誤差の位置分布（地図上・N=1 / N=max・インタラクティブ）",
    "lateral_dynamics_timeseries": "横方向 dynamics 時系列",
    "steer_vs_lateral_scatter": "ステア vs 横変位 散布",
    "cascade_error": "カスケード誤差（N=1 連鎖）",
    # step6: cases/overlay/
    "cascade_error_overlay": "全ケース カスケード誤差 重ね描き（N=1）",
    "error_growth_overlay": "全ケース 誤差成長 重ね描き",
    "rmse_heatmap": "RMSE ヒートマップ（case × horizon 俯瞰）",
    "growth_relative": "相対誤差成長（reference 比・dynamics 差の分離）",
    # step7: param_sweep/ (個別図のキャプションは _caption_for の正規表現で導出)
    "_overview_sensitivity": "スイープ感度オーバービュー（改善率ランキング + 正規化 RMSE カーブ）",
    # step9: brake_sweep/ + figures/
    "departure_brake_tc_sensitivity": "発進時 brake_tc 感度分析",
    "real_cmd_acc_departure": "発進時 制御指令（実機）",
    "brake_sweep": "brake_time_constant sweep",
    # step10: curve_diag/
    "curve_divergence": "カーブ乖離 詳細診断（縦横分解 + yaw 差）",
}

# step4 のカーブ別連番図 (curve1_analysis 等)。接頭辞除去後のサフィックスで照合。
_CURVE_SUFFIX_CAPTIONS: dict[str, str] = {
    "analysis": "カーブ個別分析（3 段）",
    "steering_detail": "カーブ ステア詳細",
    "yaw_steer": "カーブ yaw-steer 関係",
    "steer_response": "カーブ ステア応答",
}

# 再生ビューア等、plotly でない自己完結 HTML を <iframe> 埋め込みする際の高さ [px]。
# 生成側 (step4) と共有する lib._plotly_utils.FIG_HEIGHTS + IFRAME_PAD から導出する。
_IFRAME_HEIGHTS: dict[str, int] = {stem: h + IFRAME_PAD for stem, h in FIG_HEIGHTS.items()}
_IFRAME_HEIGHT_DEFAULT = 650
# plotly 図 div の高さを fig.json の layout.height から確保する際の最終フォールバック [px]。
_FIG_HEIGHT_DEFAULT = 600

# 収集対象。plotly 図スペック (*.fig.json) と、自己完結 HTML ビューア (*.html)。
_PLAYBACK_SUFFIX = ".html"

# iframe srcdoc で埋め込む「自己完結 HTML ビューア」の stem 集合。canvas 独自 JS で
# 外部参照を持たないものに限定する（plotly standalone HTML は plotly.min.js を相対参照
# するため srcdoc 内で壊れる。それらは *.fig.json へ変換し本文に直接描画する）。
_SELFCONTAINED_HTML: set[str] = {"trajectory_playback"}


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
    "curves_closeup",
}

# 取り込む Markdown レポート: (comparison/ からの相対パス, 見出し, 所属カテゴリ)。
_MARKDOWN_REPORTS: list[tuple[str, str, str]] = [
    ("report.md", "比較レポート（step4: report.md）", "closed_loop"),
    ("param_sweep/param_sweep_summary.md", "パラメータ同定サマリ（step7: param_sweep_summary.md）", "real_analysis"),
    ("cases/cases_summary.md", "ケース集約サマリ（step6: cases_summary.md）", "ol_nstep"),
    ("curve_diag/curve_divergence.md", "カーブ乖離サマリ（step10: curve_divergence.md）", "closed_loop"),
]


def _caption_for(stem: str) -> str:
    """画像 stem（拡張子除去済み）に対応する日本語キャプションを返す。"""
    if stem in CAPTIONS:
        return CAPTIONS[stem]
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
    # curveN_<suffix> パターン照合
    m = re.match(r"curve(\d+)_(.+)$", stem)
    if m:
        idx, suffix = m.group(1), m.group(2)
        cap = _CURVE_SUFFIX_CAPTIONS.get(suffix)
        if cap:
            return f"カーブ{idx}: {cap}"
    # フォールバック: アンダースコアを空白に
    return stem.replace("_", " ")


def _classify(rel: Path) -> str:
    """図の相対パス (comparison/ 基準) を概念セクションキーへ分類する。

    figures/ 配下にはクローズループ図・DP 図・brake 同定図が混在するため、
    ディレクトリだけでなく stem でも判定する。
    """
    top = rel.parts[0] if len(rel.parts) > 1 else "."
    stem = _asset_stem(rel)

    if stem.startswith("dp_"):
        return "dp"
    if stem in {"departure_brake_tc_sensitivity", "real_cmd_acc_departure"}:
        return "real_analysis"
    if top in {"param_sweep", "brake_sweep"}:
        return "real_analysis"
    if top in {"nstep", "cases"}:
        return "ol_nstep"
    if top == "curve_diag":
        return "closed_loop"
    if stem in _CLOSED_LOOP_STEMS or re.match(r"curve\d+_.+$", stem):
        return "closed_loop"
    return "other"


def _render_markdown(text: str) -> str:
    """Markdown を HTML 化する。markdown パッケージが無ければ <pre> でフォールバック。"""
    try:
        import markdown as _md  # noqa: PLC0415

        return _md.markdown(text, extensions=["tables", "fenced_code"])
    except ImportError:
        return f"<pre class='md-fallback'>{html.escape(text)}</pre>"


_STYLE = """
:root { --fg:#1a1a1a; --muted:#666; --border:#ddd; --accent:#2563eb; --bg:#fff; }
* { box-sizing: border-box; }
body { font-family: -apple-system, "Segoe UI", "Hiragino Sans", "Noto Sans CJK JP", sans-serif;
       color: var(--fg); background: var(--bg); margin: 0; line-height: 1.6; }
.layout { display: flex; align-items: flex-start; }
nav.toc { position: sticky; top: 0; align-self: flex-start; width: 260px; min-width: 260px;
          height: 100vh; overflow-y: auto; padding: 1.5rem 1rem; border-right: 1px solid var(--border);
          background: #fafafa; font-size: 0.9rem; }
nav.toc h2 { font-size: 1rem; margin: 0 0 0.5rem; }
nav.toc ul { list-style: none; padding-left: 0.5rem; margin: 0.25rem 0; }
nav.toc a { color: var(--accent); text-decoration: none; }
nav.toc a:hover { text-decoration: underline; }
main { flex: 1; padding: 2rem 2.5rem; max-width: 1100px; min-width: 0; }
header.page { margin-bottom: 2rem; }
header.page h1 { margin: 0 0 0.25rem; }
header.page .meta { color: var(--muted); font-size: 0.9rem; }
nav.toc li.toc-sec { font-weight: 600; margin-top: 0.35rem; }
nav.toc li.toc-md { font-size: 0.82rem; padding-left: 1rem; font-weight: 400; }
nav.toc .toc-top { display: inline-block; margin-bottom: 0.6rem; font-size: 0.82rem; }

/* セクションは <details> で折りたたみ可能（既定 open）。 */
details.section { margin-bottom: 2.2rem; border-top: 2px solid var(--border); padding-top: 0.4rem; }
details.section > summary { cursor: pointer; font-size: 1.35rem; font-weight: 700; padding: 0.3rem 0; }
details.section > summary:hover { color: var(--accent); }
.sec-desc { color: var(--muted); font-size: 0.92rem; margin: 0.4rem 0 1.2rem; }
.sec-desc .toplink { margin-left: 0.6rem; font-size: 0.82rem; white-space: nowrap; }
figure { margin: 0 0 2rem; }
/* plotly 図 div / playback iframe。高さは図種別ごとに指定する。 */
figure .plotly-fig { width: 100%; min-height: 120px; border: 1px solid var(--border);
                     border-radius: 4px; background: #fff; }
figure iframe.playback-fig { width: 100%; border: 1px solid var(--border); border-radius: 4px; background: #fff; }
.plotly-fig.pending::before { content: "図を描画中…"; display: block; padding: 2rem;
                              color: var(--muted); font-size: 0.85rem; text-align: center; }
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


def _figure(rel: Path, comparison_dir: Path, caption: str | None = None) -> str:
    """1 図分の <figure> HTML を返す（形式で分岐、外部参照なしで埋め込む）。

    - `*.fig.json` (plotly) → `<div class='plotly-fig pending'>` プレースホルダ + 直後の
      `<script type='application/json'>` に図スペック。描画は glue JS が遅延実行する。
    - `*.html` (playback 等) → 自己完結 canvas HTML。`<iframe srcdoc>` で隔離埋め込み。
    """
    fname = rel.as_posix()
    stem = _asset_stem(rel)
    if caption is None:
        caption = _caption_for(stem)
    cap_html = (
        f"<figcaption>{html.escape(caption)}"
        f"<span class='fname'>{html.escape(fname)}</span></figcaption>"
    )
    if _is_fig_json(rel):
        spec = (comparison_dir / rel).read_text(encoding="utf-8", errors="replace")
        fig_id = "fig-" + _slug(fname)
        # プレースホルダ div に図の高さを**事前に確保**する。各 fig.json は build_fig_* が
        # layout.height を持つ（plotly は autosize=true でコンテナ高に縮むため、div に高さが
        # 無いと描画前後で潰れる/領域不足になる）。layout.height → FIG_HEIGHTS → 既定の順。
        try:
            fig_h = json.loads(spec).get("layout", {}).get("height")
        except (ValueError, AttributeError):
            fig_h = None
        height = int(fig_h or FIG_HEIGHTS.get(stem) or _FIG_HEIGHT_DEFAULT)
        style = f" style='height:{height}px'"
        # 図スペック JSON は <script type='application/json'> にそのまま入れる（実行されない）。
        # </script> 直書きの早期終了だけ無害化する。
        spec_safe = spec.replace("</", "<\\/")
        return (
            f"<figure>{cap_html}"
            f"<div class='plotly-fig pending' id='{fig_id}'{style}></div>"
            f"<script type='application/json' class='figspec' data-target='{fig_id}'>{spec_safe}</script>"
            f"</figure>"
        )
    # 自己完結 HTML（playback 等）は独自 JS 衝突回避のため iframe srcdoc で隔離。
    text = (comparison_dir / rel).read_text(encoding="utf-8", errors="replace")
    height = _IFRAME_HEIGHTS.get(stem, _IFRAME_HEIGHT_DEFAULT)
    return (
        f"<figure>{cap_html}"
        f"<iframe class='playback-fig' srcdoc='{escape_srcdoc(text)}' "
        f"style='height:{height}px' title='{html.escape(caption)}'></iframe></figure>"
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
    per_case: dict[str, dict[str, Path]], comparison_dir: Path, cat: str
) -> list[str]:
    """プロット種別ごとにブロックを作り、ブロックごとに独立したケースタブを付ける。

    per_case: {case_tag: {stem: rel}}。各ブロック先頭に専用のケースラジオを置き、
    選んだケースの図がそのブロック内だけで切り替わる（他プロットには影響しない）。
    """
    cases = _sorted_cases(list(per_case.keys()))
    plot_types = sorted({stem for files in per_case.values() for stem in files})

    out: list[str] = []
    for pt in plot_types:
        caption = _caption_for(pt)
        group = _slug(f"casesync-{cat}-{pt}")
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
                f"{_figure(per_case[c][pt], comparison_dir, caption=c)}</div>"
            )
        out.append("</div>")
    return out


def _render_category_images(
    rels: list[Path], comparison_dir: Path, cat: str = ""
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

    out: list[str] = [_figure(r, comparison_dir) for r in flat]
    if per_case:
        out.extend(_render_case_tabs(per_case, comparison_dir, cat))
    return out


def _casesync_css(by_cat: dict[str, list[Path]]) -> str:
    """ケースごとのタブ表示規則を生成する（ケース slug 単位、ブロック間排他はラジオ name で閉じる）。"""
    cases: set[str] = set()
    for rels in by_cat.values():
        cases.update(c for r in rels if (c := _case_of(r)) is not None)
    return "\n".join(
        f".casesync > input.cr-{_slug(c)}:checked ~ .tabpanel.case-{_slug(c)}"
        "{ display: block; }"
        for c in _sorted_cases(list(cases))
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
<summary>分析パイプライン（10 段階）</summary>
<p class="sec-desc">評価ノードが実機ログ抽出から HTML 集約まで順に実行する 10 段階パイプライン。
本レポートは各 stage の成果物（図・Markdown）をカテゴリ別に束ねたもの。
<a class="toplink" href="#top">↑ 先頭</a></p>
<table>
<thead><tr><th>Stage</th><th>名称</th><th>役割 / 主な成果物</th></tr></thead>
<tbody>
<tr><td>1</td><td>実機ログ抽出 (step1_make_lite)</td><td>input_bag から必要トピックを抽出し real.lite を生成</td></tr>
<tr><td>2</td><td>scenario 自動生成 (step2_bag_to_scenario)</td><td>実機 bag + 地図から OpenSCENARIO (auto_scenario.yaml) を生成</td></tr>
<tr><td>3</td><td>closed-loop シム実行 (step3_run_sims)</td><td>auto_scenario + sim_runs.yaml で sim を回し sim lite を生成</td></tr>
<tr><td>4</td><td>実機 + sim 比較解析 (step4_compare_logs)</td><td>速度・ステア・軌跡を N-way 重ね描き (report.md・図スペック)</td></tr>
<tr><td>5</td><td>VehicleModel N-step オープンループ解析 (step5_analyze_nstep)</td><td>real.lite + cases.yaml の各ケースで free-running rollout の終端誤差を評価 (nstep/&lt;tag&gt;/)</td></tr>
<tr><td>6</td><td>ケース集約解析 (step6_analyze_cases)</td><td>全ケースの N-step 誤差を横断集約 (cases_summary.md・overlay)</td></tr>
<tr><td>7</td><td>パラメータ sweep 同定 (step7_sweep_params)</td><td>車両モデル各パラメータを sweep し終端誤差最小値を同定 (param_sweep_summary.md)</td></tr>
<tr><td>8</td><td>DP 軌跡比較 (step8_compare_dp_trajectory)</td><td>DiffusionPlanner 出力軌跡を実機 vs sim で比較 (dp_*)</td></tr>
<tr><td>9</td><td>縦パラ同定 (step9_identify_brake)</td><td>発進フィットで brake_time_constant を同定 (brake_sweep)</td></tr>
<tr><td>10</td><td>カーブ乖離診断 (step10_diagnose_curve)</td><td>カーブ/発進区間の乖離を縦横・速度・yaw で診断 (curve_divergence.md)</td></tr>
<tr><td>11</td><td>HTML レポート生成 (step11_build_html_report)</td><td>comparison/ 配下の全図スペック・Markdown・設定 YAML を 1 枚に束ねた単一レポート (report.html)</td></tr>
</tbody>
</table>
</details>
"""


# 図スペック (*.fig.json) を遅延描画する glue JS。各 <div.plotly-fig> は直後の
# <script type='application/json'> に図スペックを持つ。IntersectionObserver で可視化時に
# Plotly.newPlot するが、折りたたみ <details> / 非選択ケースタブは display:none で IO が
# 発火しないため、details の toggle・ケースタブ切替時にも配下の未描画図を reveal-render する。
_RENDER_GLUE = """
<script>
(function () {
  function specFor(div) {
    var s = div.parentNode.querySelector("script.figspec[data-target='" + div.id + "']");
    return s ? s.textContent : null;
  }
  function render(div) {
    if (!div || div.dataset.rendered || !window.Plotly) return;
    var raw = specFor(div);
    if (!raw) return;
    var spec;
    try { spec = JSON.parse(raw); } catch (e) { return; }
    div.dataset.rendered = "1";
    div.classList.remove("pending");
    window.Plotly.newPlot(div, spec.data || [], spec.layout || {}, {responsive: true});
  }
  function renderVisibleWithin(root) {
    (root || document).querySelectorAll(".plotly-fig:not([data-rendered])").forEach(function (div) {
      if (div.offsetParent !== null) render(div);  // display:none でない＝描画対象
    });
  }
  var io = ("IntersectionObserver" in window)
    ? new IntersectionObserver(function (entries) {
        entries.forEach(function (e) { if (e.isIntersecting) { render(e.target); io.unobserve(e.target); } });
      }, {rootMargin: "200px"})
    : null;
  function observeAll() {
    document.querySelectorAll(".plotly-fig:not([data-rendered])").forEach(function (div) {
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
})();
</script>
"""


def _render_config_section(config_files: list[tuple[str, Path]]) -> str:
    """実行構成 (設定 YAML) を折りたたみセクションとして埋め込む（存在するファイルのみ）。"""
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
    if not blocks:
        return ""
    return (
        "<details class='section' open id='sec-config'>"
        "<summary>実行構成（設定ファイル）</summary>"
        "<p class='sec-desc'>この報告を生成した際のシナリオ・sim 実行設定・車両モデルパラメータ。"
        "<a class='toplink' href='#top'>↑ 先頭</a></p>"
        + "".join(blocks)
        + "</details>"
    )


def build_html(
    comparison_dir: Path,
    scenario_name: str,
    images: list[Path] | None = None,
    config_files: list[tuple[str, Path]] | None = None,
) -> str:
    """comparison_dir を走査して単一 report.html の文字列を組み立てる。

    全図スペック (*.fig.json) を `<script type='application/json'>` で並置し、plotly.min.js を
    1 回だけインライン、glue JS が遅延描画する。playback (*.html) は iframe srcdoc で隔離。
    config_files は (見出し, パス) のリストで末尾の「実行構成」に生テキストで埋め込む。
    """
    if images is None:
        images = _collect_figures(comparison_dir)
    if config_files is None:
        config_files = []
    rels_all = [img.relative_to(comparison_dir) for img in images]
    by_cat: dict[str, list[Path]] = {}
    for rel in rels_all:
        by_cat.setdefault(_classify(rel), []).append(rel)

    # Markdown レポートをカテゴリ別に取り込み（存在するもののみ）
    md_by_cat: dict[str, list[tuple[str, str, str]]] = {}
    for rel, title, cat in _MARKDOWN_REPORTS:
        path = comparison_dir / rel
        if path.exists():
            anchor = "md-" + re.sub(r"[^a-z0-9]+", "-", rel.lower())
            md_by_cat.setdefault(cat, []).append(
                (anchor, title, _render_markdown(path.read_text(encoding="utf-8")))
            )

    active_cats = [c for c in _CATEGORY_ORDER if c in by_cat or c in md_by_cat]

    # --- 目次 ---（sticky）
    toc: list[str] = ["<nav class='toc'><h2>目次</h2>"]
    toc.append("<a class='toc-top' href='#top'>↑ 先頭へ</a><ul>")
    toc.append("<li class='toc-sec'><a href='#sec-pipeline'>分析パイプライン（10 段階）</a></li>")
    for cat in active_cats:
        toc.append(f"<li class='toc-sec'><a href='#sec-{cat}'>{html.escape(_CATEGORY_TITLES[cat])}</a></li>")
        for anchor, mtitle, _ in md_by_cat.get(cat, []):
            toc.append(f"<li class='toc-md'><a href='#{anchor}'>{html.escape(mtitle)}</a></li>")
    config_html = _render_config_section(config_files)
    if config_html:
        toc.append("<li class='toc-sec'><a href='#sec-config'>実行構成（設定ファイル）</a></li>")
    toc.append("</ul></nav>")

    # --- 本文 ---（各セクションは折りたたみ可能な <details open>）
    body: list[str] = []
    for cat in active_cats:
        body.append(f"<details class='section' open id='sec-{cat}'>")
        body.append(f"<summary>{html.escape(_CATEGORY_TITLES[cat])}</summary>")
        body.append(
            f"<p class='sec-desc'>{html.escape(_CATEGORY_DESCS[cat])}"
            f"<a class='toplink' href='#top'>↑ 先頭</a></p>"
        )
        if cat in by_cat:
            body.extend(_render_category_images(by_cat[cat], comparison_dir, cat=cat))
        else:
            body.append("<p class='empty'>（このセクションに該当する図はありませんでした）</p>")
        for anchor, mtitle, md_html in md_by_cat.get(cat, []):
            body.append(
                f"<details class='md-report' open id='{anchor}'>"
                f"<summary>{html.escape(mtitle)}</summary>{md_html}</details>"
            )
        body.append("</details>")

    meta_bits = [f"図 {len(images)} 枚"]
    if scenario_name:
        meta_bits.insert(0, f"シナリオ: {html.escape(scenario_name)}")
    meta = " ／ ".join(meta_bits)
    empty_note = (
        ""
        if images
        else "<p class='empty'>comparison/ 配下に図 (*.fig.json) が見つかりませんでした。"
        "先に step4〜step10 を実行してください。</p>"
    )

    sync_css = _casesync_css(by_cat)

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>real_log_sim_comparison レポート</title>
<style>{_STYLE}
{sync_css}</style>
{plotly_js_script()}
</head>
<body>
<div class="layout">
{''.join(toc)}
<main>
<header class="page" id="top">
  <h1>real_log_sim_comparison 比較レポート</h1>
  <div class="meta">{meta}</div>
</header>
{_PIPELINE_INTRO}
{empty_note}
{''.join(body)}
{config_html}
{_RENDER_GLUE}
</main>
</div>
</body>
</html>
"""


def _resolve_config_files(cfg, comparison_dir: Path) -> list[tuple[str, Path]]:
    """report.html 末尾「実行構成」に埋め込む設定ファイル群を (見出し, パス) で返す。"""
    scenarios_dir = comparison_dir.parent / "scenarios"
    return [
        ("シナリオ (auto-scenario)", scenarios_dir / "auto_scenario.yaml"),
        ("sim 実行設定 (sim_runs)", cfg.sim_runs_config) if cfg.sim_runs_config else None,
        ("シミュレータモデル (simulator_model.param)", _SIM_YAML),
        ("車両情報 (vehicle_info.param)", _INFO_YAML),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="comparison/ 配下の全図スペック (*.fig.json)・Markdown・設定 YAML を"
        "束ねた単一 report.html を生成"
    )
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    comparison_dir = cfg.out_dir
    comparison_dir.mkdir(parents=True, exist_ok=True)

    # report.html は comparison/ の親 (result_archive/ = base_dir) 直下に置く。
    html_dir = cfg.base_dir
    html_dir.mkdir(parents=True, exist_ok=True)

    images = _collect_figures(comparison_dir)
    if not images:
        warnings.warn(
            f"{comparison_dir} 配下に図 (*.fig.json) が見つかりません。"
            "目次のみの report.html を生成します"
        )

    config_files = [c for c in _resolve_config_files(cfg, comparison_dir) if c is not None]

    out_path = html_dir / "report.html"
    out_path.write_text(
        build_html(comparison_dir, cfg.scenario_name, images=images, config_files=config_files),
        encoding="utf-8",
    )
    size_mb = out_path.stat().st_size / 1024 / 1024
    print(f"  保存: {out_path} (図 {len(images)} 枚 / {size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
