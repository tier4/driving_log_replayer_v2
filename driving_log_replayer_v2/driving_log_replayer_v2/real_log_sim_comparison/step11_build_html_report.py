#!/usr/bin/env python3
"""Stage 11: comparison/ 配下の全プロットを 1 枚の HTML に集約する閲覧用レポート生成.

10 段階パイプライン (step4〜step10) は `comparison/` 配下の複数サブディレクトリ
(figures/, per_step/<tag>/, cases/overlay/, kus_sweep/, brake_sweep/, curve_diag/) に
多数の SVG・plotly HTML と 3 種の Markdown レポート (report.md, cases/cases_summary.md,
curve_diag/curve_divergence.md) を散らして出力する。本ステージはそれらを走査し、
全図を**相対パス**でリンクした目次付き `index.html` を `result_archive/` 直下
(comparison/ の親) に生成する。図は comparison/ 配下にあるため src には `comparison/`
プレフィックスを付ける。

レポートは出力ディレクトリではなく **比較の概念** でセクション分けする（読み手が
「何を何と比べた図か」で辿れるようにするため）:

1. 実機 rosbag 解析   — 実機ログ(SSOT)のみから抽出する特性・車両パラメータ同定 (sim 非介在)
2. プランナ出力比較   — DiffusionPlanner 出力軌跡を実走・最終 planning・sim 出力と比較
3. 1-step オープンループ比較 — 各ステップで実機状態にリセットした車両モデル 1 ステップ予測の差
4. multi-step オープンループ比較 — 実機初期状態からの N ステップ連続 rollout 誤差成長
5. シナリオ クローズループ比較 — auto-scenario を sim で closed-loop 実行した実機との乖離

各図は出力先ディレクトリではなく (ディレクトリ, ファイル名) の組で分類する。figures/ には
クローズループ図・DP 図・brake 同定図が混在するため、ディレクトリ単位では分けられない。
未知の図は捨てずに「その他」セクションへ回す。

図は 2 形式を扱う:
- `*.svg`  — matplotlib 製の静止画。<img> 表示＋純 CSS ライトボックスで拡大可能。
- `*.html` — plotly 製のインタラクティブ図（マップ上プロット）。<iframe> で埋め込み、
  図自体がズーム・パン・ホバー可能なためライトボックス対象外。plotly.js はバンドル直下の
  `plotly.min.js` を各図が相対参照する（CDN 不使用・オフライン可）。

index.html を開けば全プロットと Markdown レポートを 1 画面で閲覧でき、相対リンクなので
バンドルフォルダ (`result_archive/real_log_sim_comparison/`) ごとアーカイブ・共有してもそのまま動く。

本ステージは読み取り専用 (バンドル直下に index.html / plotly.min.js を足すだけ)。
他ステージの出力は変更しない。`markdown` パッケージがあれば Markdown を整形 HTML 化し、
無ければ生テキストを <pre> で埋め込む (ハード依存にしない)。
"""

from __future__ import annotations

import argparse
import html
from pathlib import Path
import re
import warnings

from .lib._plotly_utils import FIG_HEIGHTS, IFRAME_PAD, ensure_plotlyjs
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

# --- 画像キャプション (ファイル名 stem → 日本語説明) ----------------------------
# step4〜step10 が出力する既知の図（拡張子非依存の stem で引く）。
# 未登録ファイルはファイル名を人間可読化してフォールバック。
CAPTIONS: dict[str, str] = {
    # step4: figures/
    "trajectory_with_map": "軌跡比較（地図背景付き・インタラクティブ）",
    "trajectory_xy": "軌跡比較（地図なし・インタラクティブ）",
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
    # step5: per_step/<tag>/
    "overview": "誤差概観",
    "error_timeseries": "誤差時系列",
    "error_vs_speed": "速度別 誤差散布",
    "steering_analysis": "ステア分析",
    "map_distribution": "位置分布（地図上・インタラクティブ）",
    "lateral_dynamics_timeseries": "横方向 dynamics 時系列",
    "steer_vs_lateral_scatter": "ステア vs 横変位 散布",
    "cascade_error": "カスケード誤差（1 ステップ連鎖）",
    "rollout_error_growth": "rollout 誤差成長（多段 free-running）",
    # step6: cases/overlay/
    "cascade_error_overlay": "全ケース カスケード誤差 重ね描き",
    "error_timeseries_overlay": "全ケース 誤差時系列 重ね描き",
    # step7: kus_sweep/
    "kus_sweep": "k_us（アンダーステア係数）グリッド sweep",
    # step9: brake_sweep/ + figures/
    "departure_brake_tc_sensitivity": "発進時 brake_tc 感度分析",
    "real_cmd_acc_departure": "発進時 制御指令（実機）",
    "brake_sweep": "brake_time_constant sweep",
    # step10: curve_diag/
    "curve_divergence": "カーブ乖離 詳細診断（縦横分解 + yaw 差）",
}

# step4 のカーブ別連番図 (curve1_analysis.svg 等)。接頭辞除去後のサフィックスで照合。
_CURVE_SUFFIX_CAPTIONS: dict[str, str] = {
    "analysis": "カーブ個別分析（3 段）",
    "steering_detail": "カーブ ステア詳細",
    "yaw_steer": "カーブ yaw-steer 関係",
    "steer_response": "カーブ ステア応答",
}

# plotly 図 (*.html) を <iframe> 埋め込みする際の高さ [px]（stem → px）。
# 生成側 (step4/step5) と共有する lib._plotly_utils.FIG_HEIGHTS（layout 高さの単一
# ソース）+ IFRAME_PAD（モードバー等の余白）から導出する。
_IFRAME_HEIGHTS: dict[str, int] = {stem: h + IFRAME_PAD for stem, h in FIG_HEIGHTS.items()}
_IFRAME_HEIGHT_DEFAULT = 650

# 図として収集する拡張子と、インタラクティブ図（plotly HTML、iframe 埋め込み・
# ライトボックス対象外）の判定。形式の追加・変更時はここだけ直せばよい。
_FIGURE_SUFFIXES = {".svg", ".html"}


def _is_interactive(rel: Path) -> bool:
    """plotly 製インタラクティブ図（iframe 埋め込み対象）かどうか。"""
    return rel.suffix == ".html"

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
        "ol_1step",
        "3. 1-step オープンループ比較",
        "各ステップで実機状態にリセットし、車両モデルの 1 ステップ予測と実機の差を評価（累積誤差を排除）。",
    ),
    (
        "ol_multistep",
        "4. multi-step オープンループ比較",
        "実機初期状態から N ステップ連続予測（free-running rollout）し、dynamics 差の累積を顕在化する。"
        "横断的な rollout RMSE 表は §3 末尾の cases_summary.md を参照。",
    ),
    (
        "closed_loop",
        "5. シナリオ クローズループ比較",
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
    "curves_closeup",
}

# 取り込む Markdown レポート: (comparison/ からの相対パス, 見出し, 所属カテゴリ)。
_MARKDOWN_REPORTS: list[tuple[str, str, str]] = [
    ("report.md", "比較レポート（step4: report.md）", "closed_loop"),
    ("cases/cases_summary.md", "ケース集約サマリ（step6: cases_summary.md）", "ol_1step"),
    ("curve_diag/curve_divergence.md", "カーブ乖離サマリ（step10: curve_divergence.md）", "closed_loop"),
]


def _caption_for(filename: str) -> str:
    """画像ファイル名に対応する日本語キャプションを返す（拡張子非依存の stem で照合）。"""
    stem = Path(filename).stem
    if stem in CAPTIONS:
        return CAPTIONS[stem]
    # curveN_<suffix> パターン照合
    m = re.match(r"curve(\d+)_(.+)$", stem)
    if m:
        idx, suffix = m.group(1), m.group(2)
        cap = _CURVE_SUFFIX_CAPTIONS.get(suffix)
        if cap:
            return f"カーブ{idx}: {cap}"
    # フォールバック: 拡張子除去 + アンダースコアを空白に
    return stem.replace("_", " ")


def _classify(rel: Path) -> str:
    """画像の相対パス (comparison/ 基準) を概念セクションキーへ分類する。

    figures/ 配下にはクローズループ図・DP 図・brake 同定図が混在するため、
    ディレクトリだけでなくファイル名でも判定する。
    """
    top = rel.parts[0] if len(rel.parts) > 1 else "."
    stem = rel.stem

    # DiffusionPlanner 出力比較（step8、figures/dp_*.svg）
    if stem.startswith("dp_"):
        return "dp"
    # brake_tc 同定の補助図（step9、figures/ に出力される）
    if stem in {"departure_brake_tc_sensitivity", "real_cmd_acc_departure"}:
        return "real_analysis"
    # 車両パラメータ同定 sweep（step7 / step9、実機ログのみ）
    if top in {"kus_sweep", "brake_sweep"}:
        return "real_analysis"
    # per-step delta 解析（step5）: rollout のみ multi-step、他は 1-step
    if top == "per_step":
        return "ol_multistep" if stem == "rollout_error_growth" else "ol_1step"
    # ケース集約の重ね描き（step6、1-step 誤差の横断比較）
    if top == "cases":
        return "ol_1step"
    # カーブ乖離詳細診断（step10、実機 vs sim closed-loop）
    if top == "curve_diag":
        return "closed_loop"
    # step4 の closed-loop 比較図（既知 stem + カーブ別連番図 curveN_*）
    if stem in _CLOSED_LOOP_STEMS or re.match(r"curve\d+_.+$", stem):
        return "closed_loop"
    # 既知のいずれにも当たらない図は捨てず「その他」へ（黙って誤分類しない）
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

/* セクションは <details> で折りたたみ可能（既定 open）。ネイティブ HTML・JS 不使用。 */
details.section { margin-bottom: 2.2rem; border-top: 2px solid var(--border); padding-top: 0.4rem; }
details.section > summary { cursor: pointer; font-size: 1.35rem; font-weight: 700; padding: 0.3rem 0; }
details.section > summary:hover { color: var(--accent); }
.sec-desc { color: var(--muted); font-size: 0.92rem; margin: 0.4rem 0 1.2rem; }
.sec-desc .toplink { margin-left: 0.6rem; font-size: 0.82rem; white-space: nowrap; }
figure { margin: 0 0 2rem; }
figure img { width: 100%; height: auto; border: 1px solid var(--border); border-radius: 4px; cursor: zoom-in; }
/* plotly インタラクティブ図の iframe。高さは図種別ごとにインラインで指定する。 */
figure iframe.plotly-fig { width: 100%; border: 1px solid var(--border); border-radius: 4px; background: #fff; }
figcaption { margin-top: 0.4rem; font-weight: 600; }
figcaption .fname { font-weight: 400; color: var(--muted); font-size: 0.82rem; margin-left: 0.5rem; }
details.md-report { border-top: 1px dashed var(--border); margin-top: 2rem; padding-top: 0.5rem; }
details.md-report > summary { cursor: pointer; font-weight: 600; margin-bottom: 0.5rem; }
details.md-report > summary:hover { color: var(--accent); }
.md-report table { border-collapse: collapse; margin: 1rem 0; }
.md-report th, .md-report td { border: 1px solid var(--border); padding: 0.3rem 0.6rem; }
.md-report th { background: #f0f0f0; }
.md-fallback { background: #f6f6f6; padding: 1rem; overflow-x: auto; font-size: 0.85rem; }
.empty { color: var(--muted); font-style: italic; }

/* 純 CSS ケースタブ（一括連動）。セクション先頭に 1 組のラジオを置き、選んだケースの図が
   セクション内全ブロックで一斉に切り替わる。JS 不使用・オフライン可。CSS 無効環境では全パネルが
   縦に並んで degrade。連動規則 (#group-case:checked ~ .tabblock .tabpanel.case-X) は build_html が
   セクション×ケースごとに動的生成する。 */
.casesync { margin-top: 0.3rem; }
.casesync > .casesync-label { font-size: 0.85rem; color: var(--muted); margin-right: 0.4rem; }
.casesync > input { position: absolute; opacity: 0; pointer-events: none; }
.casesync > label { display: inline-block; padding: 0.3rem 0.9rem; margin: 0 0.3rem 0.9rem 0;
                    border: 1px solid var(--border); border-radius: 4px; cursor: pointer;
                    font-size: 0.88rem; color: var(--accent); background: #fafafa; user-select: none; }
.casesync > input:checked + label { background: var(--accent); color: #fff; border-color: var(--accent); }
.casesync .tabblock { margin: 0.4rem 0 2.2rem; }
.casesync .tabblock > h3 { margin: 0 0 0.5rem; }
.tabpanel { display: none; }

/* 純 CSS ライトボックス（画像拡大）。サムネクリックで #lb-* を :target にしオーバーレイ表示。
   オーバーレイは display:none/details の祖先内だと :target が効かないため <main> 末尾に一括配置する。 */
.lightboxes:empty { display: none; }
.lightbox { display: none; }
.lightbox:target { display: flex; position: fixed; inset: 0; z-index: 1000;
                   background: rgba(0,0,0,0.85); align-items: center; justify-content: center; padding: 2rem; }
.lightbox .lb-close { position: absolute; inset: 0; cursor: zoom-out; }
.lightbox .lb-fig { position: relative; z-index: 1; margin: 0; max-width: 96vw; max-height: 94vh;
                    display: flex; flex-direction: column; align-items: center; }
.lightbox .lb-fig img { max-width: 96vw; max-height: 88vh; width: auto; height: auto;
                        border: 1px solid #444; border-radius: 4px; background: #fff; }
.lightbox .lb-cap { color: #eee; font-size: 0.85rem; margin-top: 0.5rem; text-align: center; }
.lightbox .lb-cap a { color: #9bf; }
"""


def _slug(text: str) -> str:
    """HTML id/name 用に安全な文字へ変換する。"""
    return re.sub(r"[^A-Za-z0-9_-]+", "-", text).strip("-").lower()


def _lb_id(rel: Path) -> str:
    """画像のライトボックス用 id（rel パスから一意に生成）。"""
    return "lb-" + _slug(rel.as_posix())


def _figure(rel: Path, link_prefix: str, caption: str | None = None) -> str:
    """1 図分の <figure> HTML を返す（形式で分岐）。

    rel は comparison/ 基準の相対パス。link_prefix を前置して HTML 出力ディレクトリ
    (result_archive/) からの相対 src にする（例 link_prefix="comparison/"）。
    caption 省略時はファイル名から日本語キャプションを導出する。

    - `*.svg`  → <img> サムネ。クリックでライトボックス #lb-* を開く（拡大用オーバーレイ
      本体は build_html が <main> 末尾に一括出力する _lightbox_overlays）。
    - `*.html` → plotly インタラクティブ図。<iframe> 埋め込み（図自体がズーム可能なため
      ライトボックス無し）。高さは _IFRAME_HEIGHTS（plotly layout 高さと整合）。
    """
    src = link_prefix + rel.as_posix()
    if caption is None:
        caption = _caption_for(rel.name)
    if _is_interactive(rel):
        height = _IFRAME_HEIGHTS.get(rel.stem, _IFRAME_HEIGHT_DEFAULT)
        # loading='lazy' は付けない: display:none のタブパネル内 iframe の遅延ロードは
        # ブラウザ挙動が非一貫（表示切替後に読み込まれない場合がある）。plotly iframe は
        # ケース数+1 程度と少ないため eager ロードで問題ない。
        return (
            f"<figure><iframe class='plotly-fig' "
            f"src='{html.escape(src)}' style='height:{height}px' "
            f"title='{html.escape(caption)}'></iframe>"
            f"<figcaption>{html.escape(caption)}"
            f"<span class='fname'>{html.escape(src)}</span></figcaption></figure>"
        )
    return (
        f"<figure><a class='thumb' href='#{_lb_id(rel)}'>"
        f"<img loading='lazy' src='{html.escape(src)}' alt='{html.escape(caption)}'></a>"
        f"<figcaption>{html.escape(caption)}"
        f"<span class='fname'>{html.escape(src)}</span></figcaption></figure>"
    )


def _lightbox_overlays(images: list[Path], link_prefix: str) -> str:
    """全画像の拡大用オーバーレイを 1 まとめで返す（<main> 末尾に配置する想定）。

    オーバーレイは display:none やタブパネル・<details> の中に置くと :target が効かないため、
    祖先に隠し要素を持たない位置（main 直下末尾）へ一括出力する。
    plotly 図 (*.html) は図自体がズーム可能なためライトボックス対象外。
    """
    parts = ["<div class='lightboxes'>"]
    for rel in images:
        if _is_interactive(rel):
            continue
        src = link_prefix + rel.as_posix()
        caption = _caption_for(rel.name)
        # 閉じると元セクションに戻す（href='#' だとページ先頭へ飛んでしまうため）
        close_href = f"#sec-{_classify(rel)}"
        parts.append(
            f"<div class='lightbox' id='{_lb_id(rel)}'>"
            f"<a class='lb-close' href='{close_href}' title='閉じる'></a>"
            f"<figure class='lb-fig'>"
            f"<img loading='lazy' src='{html.escape(src)}' alt='{html.escape(caption)}'>"
            f"<figcaption class='lb-cap'>{html.escape(caption)} "
            f"<a href='{html.escape(src)}'>原寸を開く ↗</a></figcaption>"
            f"</figure></div>"
        )
    parts.append("</div>")
    return "".join(parts)


def _case_of(rel: Path) -> str | None:
    """per_step/<case>/<file> のケースタグを返す（per_step 図でなければ None）。"""
    if rel.parts[0] == "per_step" and len(rel.parts) > 2:
        return rel.parts[1]
    return None


def _sorted_cases(tags: list[str]) -> list[str]:
    """ケースタグ表示順: baseline（参照）を先頭、残りはアルファベット順。"""
    return sorted(tags, key=lambda t: (t != "baseline", t))


def _casesync_group(cat: str) -> str:
    """セクション (カテゴリ) のケース連動ラジオグループ名。"""
    return _slug(f"casesync-{cat}")


def _render_synced_case_tabs(
    per_case: dict[str, dict[str, Path]], link_prefix: str, cat: str
) -> list[str]:
    """プロット種別ごとにブロックを作り、ケースをセクション全体で一括連動切り替えする。

    per_case: {case_tag: {filename: rel}}。セクション先頭に 1 組のケースラジオを置き、選んだ
    ケースの図がセクション内の全ブロックで一斉に切り替わる（連動）。各パネルは `case-<slug>`
    クラスを持ち、build_html が生成する `#group-case:checked ~ .tabblock .tabpanel.case-<slug>`
    規則で表示制御する（パネルの順序・個数に依存しない）。
    """
    cases = _sorted_cases(list(per_case.keys()))
    plot_types = sorted({fn for files in per_case.values() for fn in files})
    group = _casesync_group(cat)

    out: list[str] = ["<div class='casesync'>"]
    out.append("<span class='casesync-label'>ケース切替:</span>")
    # セクション共通のケースラジオ + ラベル（最初のケースを既定選択）
    for i, c in enumerate(cases):
        rid = f"{group}-{_slug(c)}"
        checked = " checked" if i == 0 else ""
        out.append(f"<input type='radio' name='{group}' id='{rid}'{checked}>")
        out.append(f"<label for='{rid}'>{html.escape(c)}</label>")
    # プロット種別ごとのブロック（パネルは存在するケースのみ。class でケース対応）
    for pt in plot_types:
        caption = _caption_for(pt)
        out.append("<div class='tabblock'>")
        out.append(
            f"<h3>{html.escape(caption)} <span class='fname'>{html.escape(pt)}</span></h3>"
        )
        for c in cases:
            if pt in per_case[c]:
                out.append(
                    f"<div class='tabpanel case-{_slug(c)}'>"
                    f"{_figure(per_case[c][pt], link_prefix, caption=c)}</div>"
                )
        out.append("</div>")
    out.append("</div>")
    return out


def _render_category_images(
    rels: list[Path], link_prefix: str, cat: str = ""
) -> list[str]:
    """カテゴリ内の画像群を描画する。

    per_step/<case>/ の図は「プロット種別ごとのブロック ＋ ケース一括連動タブ」で描画する。
    それ以外の図 (cases/overlay 等) は通常の figure として先に並べる。
    """
    flat = [r for r in rels if _case_of(r) is None]
    per_case: dict[str, dict[str, Path]] = {}
    for r in rels:
        case = _case_of(r)
        if case is not None:
            per_case.setdefault(case, {})[r.name] = r

    out: list[str] = [_figure(r, link_prefix) for r in flat]
    if per_case:
        out.extend(_render_synced_case_tabs(per_case, link_prefix, cat))
    return out


def _casesync_css(by_cat: dict[str, list[Path]]) -> str:
    """セクション×ケースごとのケース連動表示規則を生成する。

    あるセクションのケースラジオが checked のとき、そのセクション内の全 .tabblock の対応
    `.tabpanel.case-<slug>` を表示する。パネルの順序・個数に依存しないクラス対応方式。
    """
    rules: list[str] = []
    for cat, rels in by_cat.items():
        cases = {_case_of(r) for r in rels}
        cases.discard(None)
        if not cases:
            continue
        group = _casesync_group(cat)
        for c in _sorted_cases(list(cases)):
            cslug = _slug(c)
            rules.append(
                f"#{group}-{cslug}:checked ~ .tabblock .tabpanel.case-{cslug}"
                "{ display: block; }"
            )
    return "\n".join(rules)


def _collect_figures(comparison_dir: Path) -> list[Path]:
    """comparison/ 配下の図（*.svg + plotly *.html）を集める。

    自身の生成物（index.html / plotly.min.js はバンドル直下なので元々入らない）を
    念のため除外する。
    """
    figures = [
        p
        for p in comparison_dir.rglob("*")
        if p.suffix in _FIGURE_SUFFIXES and p.name not in {"index.html", "plotly.min.js"}
    ]
    return sorted(figures, key=lambda p: str(p))


def build_html(
    comparison_dir: Path,
    scenario_name: str,
    link_prefix: str = "",
    images: list[Path] | None = None,
) -> str:
    """comparison_dir を走査して index.html の文字列を組み立てる。

    link_prefix は HTML 出力位置から comparison_dir への相対プレフィックス。index.html を
    comparison/ の親 (result_archive/) に置く場合は "comparison/" を渡す。comparison/ 直下に
    置く従来挙動なら ""。
    """
    # SVG / plotly HTML を概念カテゴリへ分類（相対パスは comparison_dir 基準）。
    # images は呼び出し側が _collect_figures 済みなら再走査を省くために渡せる。
    if images is None:
        images = _collect_figures(comparison_dir)
    rels_all = [img.relative_to(comparison_dir) for img in images]
    by_cat: dict[str, list[Path]] = {}
    for rel in rels_all:
        by_cat.setdefault(_classify(rel), []).append(rel)

    # Markdown レポートをカテゴリ別に取り込み（存在するもののみ）
    md_by_cat: dict[str, list[tuple[str, str, str]]] = {}  # cat -> [(anchor, title, html)]
    for rel, title, cat in _MARKDOWN_REPORTS:
        path = comparison_dir / rel
        if path.exists():
            anchor = "md-" + re.sub(r"[^a-z0-9]+", "-", rel.lower())
            md_by_cat.setdefault(cat, []).append(
                (anchor, title, _render_markdown(path.read_text(encoding="utf-8")))
            )

    # 表示するカテゴリ: 画像か Markdown のいずれかを持つもののみ、_CATEGORY_ORDER 順
    active_cats = [c for c in _CATEGORY_ORDER if c in by_cat or c in md_by_cat]

    # --- 目次 ---（sticky。スクロール追従ハイライトは JS が必要なため未実装。各セクション見出しに
    #     「↑ 先頭」リンクを置いて目次/先頭へ戻れるようにする。）
    toc: list[str] = ["<nav class='toc'><h2>目次</h2>"]
    toc.append("<a class='toc-top' href='#top'>↑ 先頭へ</a><ul>")
    for cat in active_cats:
        toc.append(f"<li class='toc-sec'><a href='#sec-{cat}'>{html.escape(_CATEGORY_TITLES[cat])}</a></li>")
        for anchor, mtitle, _ in md_by_cat.get(cat, []):
            toc.append(f"<li class='toc-md'><a href='#{anchor}'>{html.escape(mtitle)}</a></li>")
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
            body.extend(_render_category_images(by_cat[cat], link_prefix, cat=cat))
        else:
            body.append("<p class='empty'>（このセクションに該当する図はありませんでした）</p>")
        # カテゴリ末尾に所属 Markdown レポートを折りたたみで埋め込む
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
        else "<p class='empty'>comparison/ 配下に図 (SVG / plotly HTML) が見つかりませんでした。"
        "先に step4〜step10 を実行してください。</p>"
    )

    # ケース一括連動の表示規則（セクション×ケース）と、拡大用ライトボックス（main 末尾に一括配置）
    sync_css = _casesync_css(by_cat)
    lightboxes = _lightbox_overlays(rels_all, link_prefix) if rels_all else ""

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>real_log_sim_comparison レポート</title>
<style>{_STYLE}
{sync_css}</style>
</head>
<body>
<div class="layout">
{''.join(toc)}
<main>
<header class="page" id="top">
  <h1>real_log_sim_comparison 比較レポート</h1>
  <div class="meta">{meta}</div>
</header>
{empty_note}
{''.join(body)}
{lightboxes}
</main>
</div>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(
        description="comparison/ 配下の全プロットを集約した閲覧用 index.html を生成"
    )
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    comparison_dir = cfg.out_dir
    comparison_dir.mkdir(parents=True, exist_ok=True)

    # index.html は comparison/ の親 (result_archive/ = base_dir) 直下に置く。
    # 図は comparison/ 配下にあるため、src には base_dir からの相対プレフィックスを付ける。
    html_dir = cfg.base_dir
    html_dir.mkdir(parents=True, exist_ok=True)
    link_prefix = comparison_dir.relative_to(html_dir).as_posix() + "/"

    # plotly 図 (*.html) が参照する plotly.min.js をバンドル直下に同梱する
    # （step4/5 実行時にも書かれるが、欠けていた場合の保険として冪等に呼ぶ）。
    ensure_plotlyjs(html_dir)

    images = _collect_figures(comparison_dir)
    if not images:
        warnings.warn(
            f"{comparison_dir} 配下に図 (SVG / plotly HTML) が見つかりません。"
            "目次のみの index.html を生成します"
        )

    out_path = html_dir / "index.html"
    out_path.write_text(
        build_html(comparison_dir, cfg.scenario_name, link_prefix, images=images),
        encoding="utf-8",
    )
    print(f"  保存: {out_path} (図 {len(images)} 枚)")


if __name__ == "__main__":
    main()
