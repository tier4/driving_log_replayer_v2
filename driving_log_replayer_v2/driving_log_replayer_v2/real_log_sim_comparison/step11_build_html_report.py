#!/usr/bin/env python3
"""Stage 11: comparison/ 配下の全プロットを 1 枚の HTML に集約する閲覧用レポート生成.

10 段階パイプライン (step4〜step10) は `comparison/` 配下の複数サブディレクトリ
(figures/, per_step/<tag>/, cases/overlay/, kus_sweep/, brake_sweep/, curve_diag/) に
多数の PNG と 3 種の Markdown レポート (report.md, cases/cases_summary.md,
curve_diag/curve_divergence.md) を散らして出力する。本ステージはそれらを走査し、
全画像を**相対パス**でリンクした目次付き `comparison/index.html` を生成する。

index.html を開けば全プロットと Markdown レポートを 1 画面で閲覧でき、相対リンクなので
`result_archive/comparison/` ごとアーカイブ・共有してもそのまま動く。

本ステージは読み取り専用 (comparison/ に index.html を 1 枚足すだけ)。他ステージの出力は変更しない。
`markdown` パッケージがあれば Markdown を整形 HTML 化し、無ければ生テキストを <pre> で埋め込む
(ハード依存にしない)。
"""

from __future__ import annotations

import argparse
import html
from pathlib import Path
import re
import warnings

from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

# --- 画像キャプション (ファイル名 → 日本語説明) ---------------------------------
# step4〜step10 が出力する既知の PNG。未登録ファイルはファイル名を人間可読化してフォールバック。
CAPTIONS: dict[str, str] = {
    # step4: figures/
    "trajectory_with_map.png": "軌跡比較（地図背景付き）",
    "velocity.png": "速度時系列",
    "acceleration.png": "加速度時系列",
    "steering.png": "ステア角時系列",
    "velocity_vs_distance.png": "走行距離基準 速度（pacing 差を除いた早期停止の露出）",
    "steering_vs_distance.png": "走行距離基準 ステア角",
    "curves_closeup.png": "カーブ一覧図（全カーブ概観）",
    # step8: figures/
    "dp_real_vs_sim.png": "DiffusionPlanner 出力軌跡 実機 vs sim",
    "dp_vs_actual.png": "DP 計画速度 vs 実際速度",
    "dp_vs_final_traj.png": "実機 DP 出力 vs 最終 planning（optimizer 補正）",
    # step5: per_step/<tag>/
    "overview.png": "誤差概観",
    "error_timeseries.png": "誤差時系列",
    "error_vs_speed.png": "速度別 誤差散布",
    "steering_analysis.png": "ステア分析",
    "map_distribution.png": "位置分布",
    "lateral_dynamics_timeseries.png": "横方向 dynamics 時系列",
    "steer_vs_lateral_scatter.png": "ステア vs 横変位 散布",
    "cascade_error.png": "カスケード誤差",
    "rollout_error_growth.png": "rollout 誤差成長",
    # step6: cases/overlay/
    "cascade_error_overlay.png": "全ケース カスケード誤差 重ね描き",
    "error_timeseries_overlay.png": "全ケース 誤差時系列 重ね描き",
    # step7: kus_sweep/
    "kus_sweep.png": "k_us（アンダーステア係数）グリッド sweep",
    # step9: brake_sweep/
    "departure_brake_tc_sensitivity.png": "発進時 brake_tc 感度分析",
    "real_cmd_acc_departure.png": "発進時 制御指令（実機）",
    "brake_sweep.png": "brake_time_constant sweep",
    # step10: curve_diag/
    "curve_divergence.png": "カーブ乖離 詳細診断（縦横分解 + yaw 差）",
}

# step4 のカーブ別連番図 (curve1_analysis.png 等)。接頭辞除去後のサフィックスで照合。
_CURVE_SUFFIX_CAPTIONS: dict[str, str] = {
    "analysis": "カーブ個別分析（3 段）",
    "steering_detail": "カーブ ステア詳細",
    "yaw_steer": "カーブ yaw-steer 関係",
    "steer_response": "カーブ ステア応答",
}

# セクションの論理表示順 (comparison/ 直下のサブディレクトリ名)。ここに無いものは末尾に回す。
_SECTION_ORDER = ["figures", "per_step", "cases", "kus_sweep", "brake_sweep", "curve_diag"]

_SECTION_TITLES: dict[str, str] = {
    "figures": "比較プロット（step4 / step8）",
    "per_step": "per-step delta 解析（step5・ケース別）",
    "cases": "ケース集約（step6）",
    "kus_sweep": "k_us 同定（step7）",
    "brake_sweep": "brake_time_constant 同定（step9）",
    "curve_diag": "カーブ乖離診断（step10）",
    ".": "その他（comparison/ 直下）",
}

# 取り込む Markdown レポート (comparison/ からの相対パス → 見出し)。
_MARKDOWN_REPORTS: list[tuple[str, str]] = [
    ("report.md", "比較レポート（step4: report.md）"),
    ("cases/cases_summary.md", "ケース集約サマリ（step6: cases_summary.md）"),
    ("curve_diag/curve_divergence.md", "カーブ乖離サマリ（step10: curve_divergence.md）"),
]


def _caption_for(filename: str) -> str:
    """画像ファイル名に対応する日本語キャプションを返す。"""
    if filename in CAPTIONS:
        return CAPTIONS[filename]
    # curveN_<suffix>.png パターン照合
    m = re.match(r"curve(\d+)_(.+)\.png$", filename)
    if m:
        idx, suffix = m.group(1), m.group(2)
        cap = _CURVE_SUFFIX_CAPTIONS.get(suffix)
        if cap:
            return f"カーブ{idx}: {cap}"
    # フォールバック: 拡張子除去 + アンダースコアを空白に
    return Path(filename).stem.replace("_", " ")


def _section_key(rel_path: Path) -> str:
    """画像の相対パス (comparison/ 基準) から所属セクションキーを返す。"""
    parts = rel_path.parts
    return parts[0] if len(parts) > 1 else "."


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
section { margin-bottom: 3rem; border-top: 2px solid var(--border); padding-top: 1rem; }
section h2 { margin-top: 0; }
.subgroup h3 { margin: 1.5rem 0 0.5rem; color: var(--muted); }
figure { margin: 0 0 2rem; }
figure img { width: 100%; height: auto; border: 1px solid var(--border); border-radius: 4px; }
figcaption { margin-top: 0.4rem; font-weight: 600; }
figcaption .fname { font-weight: 400; color: var(--muted); font-size: 0.82rem; margin-left: 0.5rem; }
.md-report table { border-collapse: collapse; margin: 1rem 0; }
.md-report th, .md-report td { border: 1px solid var(--border); padding: 0.3rem 0.6rem; }
.md-report th { background: #f0f0f0; }
.md-fallback { background: #f6f6f6; padding: 1rem; overflow-x: auto; font-size: 0.85rem; }
.empty { color: var(--muted); font-style: italic; }
"""


def build_html(comparison_dir: Path, scenario_name: str) -> str:
    """comparison_dir を走査して index.html の文字列を組み立てる。"""
    # PNG を収集（index.html 自身が参照する相対パスは comparison_dir 基準）
    images = sorted(comparison_dir.rglob("*.png"), key=lambda p: str(p))
    by_section: dict[str, list[Path]] = {}
    for img in images:
        rel = img.relative_to(comparison_dir)
        by_section.setdefault(_section_key(rel), []).append(rel)

    # セクション表示順: _SECTION_ORDER 優先、残りはアルファベット順、"." は最後
    known = [s for s in _SECTION_ORDER if s in by_section]
    extra = sorted(k for k in by_section if k not in _SECTION_ORDER and k != ".")
    ordered_sections = known + extra + (["."] if "." in by_section else [])

    # Markdown レポート（存在するもののみ）
    md_sections: list[tuple[str, str, str]] = []  # (anchor, title, html)
    for rel, title in _MARKDOWN_REPORTS:
        path = comparison_dir / rel
        if path.exists():
            anchor = "md-" + re.sub(r"[^a-z0-9]+", "-", rel.lower())
            md_sections.append((anchor, title, _render_markdown(path.read_text(encoding="utf-8"))))

    # --- 目次 ---
    toc: list[str] = ["<nav class='toc'><h2>目次</h2><ul>"]
    for sec in ordered_sections:
        title = _SECTION_TITLES.get(sec, sec)
        toc.append(f"<li><a href='#sec-{sec}'>{html.escape(title)}</a></li>")
    if md_sections:
        toc.append("<li style='margin-top:0.6rem;font-weight:600'>レポート</li>")
        for anchor, title, _ in md_sections:
            toc.append(f"<li><a href='#{anchor}'>{html.escape(title)}</a></li>")
    toc.append("</ul></nav>")

    # --- 本文: 画像セクション ---
    body: list[str] = []
    for sec in ordered_sections:
        title = _SECTION_TITLES.get(sec, sec)
        body.append(f"<section id='sec-{sec}'><h2>{html.escape(title)}</h2>")
        rels = by_section[sec]
        if sec == "per_step":
            # per_step/<tag>/... を tag ごとにサブグループ化
            groups: dict[str, list[Path]] = {}
            for rel in rels:
                tag = rel.parts[1] if len(rel.parts) > 2 else "(直下)"
                groups.setdefault(tag, []).append(rel)
            for tag in sorted(groups):
                body.append(f"<div class='subgroup'><h3>case: {html.escape(tag)}</h3>")
                body.extend(_figure(rel) for rel in groups[tag])
                body.append("</div>")
        else:
            body.extend(_figure(rel) for rel in rels)
        body.append("</section>")

    # --- 本文: Markdown レポート ---
    for anchor, title, md_html in md_sections:
        body.append(
            f"<section id='{anchor}' class='md-report'><h2>{html.escape(title)}</h2>{md_html}</section>"
        )

    meta_bits = [f"画像 {len(images)} 枚"]
    if scenario_name:
        meta_bits.insert(0, f"シナリオ: {html.escape(scenario_name)}")
    meta = " ／ ".join(meta_bits)
    empty_note = (
        ""
        if images
        else "<p class='empty'>comparison/ 配下に PNG が見つかりませんでした。"
        "先に step4〜step10 を実行してください。</p>"
    )

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>real_log_sim_comparison レポート</title>
<style>{_STYLE}</style>
</head>
<body>
<div class="layout">
{''.join(toc)}
<main>
<header class="page">
  <h1>real_log_sim_comparison プロット一覧</h1>
  <div class="meta">{meta}</div>
</header>
{empty_note}
{''.join(body)}
</main>
</div>
</body>
</html>
"""


def _figure(rel: Path) -> str:
    """1 画像分の <figure> HTML を返す（src は comparison/ 基準の相対パス）。"""
    src = rel.as_posix()
    caption = _caption_for(rel.name)
    return (
        f"<figure><a href='{html.escape(src)}'>"
        f"<img loading='lazy' src='{html.escape(src)}' alt='{html.escape(caption)}'></a>"
        f"<figcaption>{html.escape(caption)}"
        f"<span class='fname'>{html.escape(src)}</span></figcaption></figure>"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="comparison/ 配下の全プロットを集約した閲覧用 index.html を生成"
    )
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    comparison_dir = cfg.out_dir
    comparison_dir.mkdir(parents=True, exist_ok=True)

    n_png = len(list(comparison_dir.rglob("*.png")))
    if n_png == 0:
        warnings.warn(
            f"{comparison_dir} 配下に PNG が見つかりません。目次のみの index.html を生成します"
        )

    out_path = comparison_dir / "index.html"
    out_path.write_text(build_html(comparison_dir, cfg.scenario_name), encoding="utf-8")
    print(f"  保存: {out_path} (画像 {n_png} 枚)")


if __name__ == "__main__":
    main()
