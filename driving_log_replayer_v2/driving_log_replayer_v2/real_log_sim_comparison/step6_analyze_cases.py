"""Stage 4: VehicleModel N-step 解析の全ケース集約 (overlay 図 + RMSE 表).

入力:
  - cases.yaml (Stage 3 が走らせた各ケースの定義)
  - comparison/nstep/<tag>/nstep_delta.csv (Stage 3 出力, 全 tag・全 horizon 統一スキーマ)

出力:
  comparison/cases/
    ├── overlay/
    │   ├── cascade_error_overlay.svg            # N=1 段階的誤差 (ステア応答→横位置)
    │   ├── error_timeseries_overlay_n1.svg      # N=1 誤差時系列 重ね描き
    │   ├── error_timeseries_overlay_n<max>.svg  # N=max 誤差時系列 重ね描き
    │   ├── error_growth_overlay.svg             # horizon 別 RMSE 成長 重ね描き
    │   ├── rmse_heatmap.svg                     # case × horizon 俯瞰
    │   └── growth_relative.svg                  # reference 比の相対成長
    └── cases_summary.md   # N=1 詳細 RMSE 表 + horizon 別 RMSE 表

欠損ケース (CSV が無い) は警告ログを出してスキップ。集約処理は continue できる。

また、全ケースの CSV を持つ本ステージが nstep/<tag>/ のケース別図を
「ケース横断で軸範囲を統一」して上書き再描画する (rerender_case_figures)。
シミュレータ (ケース) ごとに図が分かれていてもレポートのタブ切替で軸が動かない。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager as _fm
import numpy as np
import pandas as pd

from .lib._nstep_common import ERR_METRICS, YAW_SEED_NOTE, common_horizons, n1, rmse_by_horizon

# Japanese font (best-effort; absent ならフォールバック)。
# rcParams への代入は例外を投げないので、実在フォントを font_manager で確認する。
_installed_fonts = {f.name for f in _fm.fontManager.ttflist}
for _f in ("Noto Sans CJK JP", "TakaoGothic", "IPAGothic", "DejaVu Sans"):
    if _f in _installed_fonts:
        plt.rcParams["font.family"] = _f
        break


_CASCADE_ROWS = [
    # (real_col, sim_col, err_col, scale, ylabel, title)
    ("real_steer_kend", "sim_steer_kend", "err_steer", 180.0 / math.pi,
     "ステア角 [deg]", "① ステア応答 (cmd→actual): err_steer"),
    ("real_ds_lat", "sim_ds_lat", "err_ds_lat", 100.0,
     "横方向 Δpos [cm]", "② 横方向 1ステップ変位: err_ds_lat"),
]


def _color_cycle(n: int) -> list[str]:
    base = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    return [base[i % len(base)] for i in range(n)]


def _load_case_csvs(nstep_root: Path, tags: list[str]) -> dict[str, pd.DataFrame]:
    """nstep/<tag>/nstep_delta.csv (Stage 3 出力) を全 tag 分読み込む。欠損 tag はスキップ。"""
    out: dict[str, pd.DataFrame] = {}
    for tag in tags:
        csv = nstep_root / tag / "nstep_delta.csv"
        if not csv.exists():
            print(f"[WARN] {csv} が無いため case={tag} をスキップ", file=sys.stderr)
            continue
        df = pd.read_csv(csv)
        if df.empty:
            continue
        out[tag] = df
    return out


def rerender_case_figures(
    case_dfs: dict[str, pd.DataFrame],
    cases_cfg,
    nstep_root: Path,
    base_dir: Path,
) -> None:
    """ケース横断で軸範囲を統一して nstep/<tag>/ の図を再描画する。

    step5 はケース単体実行のため各図の軸が自ケースの値域で自動スケールされ、
    レポートのケース切替タブで軸が動いて比較しにくい。全ケースの CSV を持つ
    step6 が、step5 のプロット関数 (LIMITS_DF 設定で統一軸モード) を呼び直して
    同名ファイルを上書きする。実機データのみの図 (lateral_dynamics 等) は
    ケース間で同一のため再描画しない。
    """
    from . import step5_analyze_nstep as s5  # noqa: PLC0415 (plotly 含む重 import のため遅延)

    s5.LIMITS_DF = pd.concat(
        [df.assign(case=tag) for tag, df in case_dfs.items()], ignore_index=True
    )
    try:
        for case in cases_cfg.cases:
            df = case_dfs.get(case.tag)
            if df is None:
                continue
            params = s5._build_params()
            params.update(case.params)
            s5.BASE = base_dir
            s5.OUT_DIR = nstep_root / case.tag
            df1 = n1(df)
            print(f"  [{case.tag}] ケース横断の統一軸で再描画")
            s5.plot_error_growth(df, params)
            s5.plot_error_timeseries(df, params)
            s5.plot_error_vs_speed(df, params)
            s5.plot_map_distribution(df, params)
            s5.plot_overview(df1, params)
            s5.plot_steering_analysis(df1, params)
            s5.plot_cascade_error(df1, params)
    finally:
        s5.LIMITS_DF = None


def plot_cascade_error_overlay(
    case_dfs: dict[str, pd.DataFrame], out_path: Path
) -> None:
    """全 case の N=1 err_* を 1 枚に重ね描き (各段は real / sim / err の 3 系列)."""
    dfs1 = {tag: n1(df) for tag, df in case_dfs.items()}
    # _CASCADE_ROWS は 2 要素固定なので axes は常に 2D 配列
    fig, axes = plt.subplots(
        len(_CASCADE_ROWS), 2, figsize=(16, 4 * len(_CASCADE_ROWS)),
        gridspec_kw={"width_ratios": [3, 1]},
    )

    colors = dict(zip(dfs1.keys(), _color_cycle(len(dfs1))))

    for row_idx, (real_col, sim_col, err_col, scale, ylabel, title) in enumerate(_CASCADE_ROWS):
        ax_ts = axes[row_idx, 0]
        ax_err = axes[row_idx, 1]

        for tag, df in dfs1.items():
            tr = df["tr"].values
            color = colors[tag]
            # 時系列: sim を tag 別の色 (real は灰色で全 case 共通描画)
            ax_ts.plot(tr, df[sim_col].values * scale, color=color, lw=1.2, label=f"{tag} sim")
            # 誤差時系列
            err = df[err_col].values * scale
            rmse = float(np.sqrt(np.nanmean(err ** 2)))
            ax_err.plot(tr, err, color=color, lw=1.0, alpha=0.8,
                        label=f"{tag} RMSE={rmse:.3g}")

        # real は最初の case のものを灰色基準として描画 (全 case で同じ実機ログ)
        first_df = next(iter(dfs1.values()))
        ax_ts.plot(first_df["tr"].values, first_df[real_col].values * scale,
                   color="black", lw=1.5, ls="--", label="real", zorder=10)

        ax_ts.axhline(0, color="black", lw=0.5, ls=":")
        ax_ts.set_ylabel(ylabel)
        ax_ts.set_title(title, fontsize=10)
        ax_ts.legend(fontsize=8, ncol=2)
        ax_ts.grid(True, alpha=0.3)

        ax_err.axhline(0, color="black", lw=0.6)
        ax_err.set_ylabel(f"誤差 [{ylabel.split('[')[1]}")
        ax_err.set_title("誤差 (real − sim) — case 別 RMSE", fontsize=10)
        ax_err.legend(fontsize=8)
        ax_err.grid(True, alpha=0.3)

    for col in range(2):
        axes[-1, col].set_xlabel("AUTONOMOUS 開始からの時刻 [s]")

    fig.suptitle("cases overlay: N=1 段階的誤差 (ステア応答 → 横位置)", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_error_timeseries_overlay(
    case_dfs: dict[str, pd.DataFrame], horizon: int, out_path: Path
) -> None:
    """指定 horizon の終端誤差時系列を全 case 重ね描き。

    N=1 では従来の per-step 誤差比較、N=max では RMSE 集約で消える
    「コースのどこ (どのカーブ) でどのモデルが乖離するか」の比較になる。
    """
    fig, axes = plt.subplots(len(ERR_METRICS), 1,
                             figsize=(14, 3.5 * len(ERR_METRICS)), sharex=True)
    colors = dict(zip(case_dfs.keys(), _color_cycle(len(case_dfs))))

    for ax, (col, scale, label, unit, _source) in zip(axes, ERR_METRICS):
        for tag, df in case_dfs.items():
            sub = df[df["horizon"] == horizon].sort_values("tr")
            err = sub[col].values * scale
            rmse = float(np.sqrt(np.nanmean(err ** 2)))
            ax.plot(sub["tr"].values, err, color=colors[tag], lw=1.0, alpha=0.85,
                    label=f"{tag}  RMSE={rmse:.3g} {unit}")
        ax.axhline(0, color="black", lw=0.6)
        ax.set_ylabel(f"誤差 [{unit}]")
        full_title = f"{label} (N={horizon}, 実機 − モデル)"
        if col == "yaw_err_deg":
            full_title += f"\n{YAW_SEED_NOTE}"
        ax.set_title(full_title, fontsize=11)
        ax.legend(fontsize=9, ncol=2)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("rollout 開始時刻 (AUTONOMOUS 開始からの経過) [s]")
    fig.suptitle(f"cases overlay: N-step 誤差時系列 (N={horizon})", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_error_growth_overlay(
    roll: dict[str, dict[int, dict[str, float]]], out_path: Path
) -> None:
    """全 case の N-step 誤差成長 (horizon 別 RMSE) を 1 枚に重ね描き。

    roll: tag → {horizon → {"pos","long","lat","yaw"}} (rmse_by_horizon の集約)。
    """
    fig, (ax_pos, ax_yaw) = plt.subplots(1, 2, figsize=(14, 5))
    colors = dict(zip(roll.keys(), _color_cycle(len(roll))))

    for tag, r in roll.items():
        horizons = sorted(r.keys())
        ax_pos.plot(horizons, [r[h]["pos"] for h in horizons], "o-",
                    color=colors[tag], lw=1.4, label=tag)
        ax_yaw.plot(horizons, [r[h]["yaw"] for h in horizons], "s--",
                    color=colors[tag], lw=1.4, label=tag)

    for ax, ylabel, title in [
        (ax_pos, "位置 RMSE [cm]", "位置誤差成長"),
        (ax_yaw, "yaw RMSE [deg]", "yaw 誤差成長"),
    ]:
        ax.set_xlabel("rollout 長 N [step]")
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=9)
        ax.grid(True, lw=0.5, alpha=0.6)

    fig.suptitle("cases overlay: N-step 誤差成長 (free-running)", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_rmse_heatmap(
    roll: dict[str, dict[int, dict[str, float]]], out_path: Path
) -> None:
    """case × horizon の RMSE ヒートマップ (位置/yaw の 2 パネル、数値注釈付き)。

    cases_summary.md の表の図版化。2 変数 (シミュレータ構成 × rollout 長) の
    全組み合わせを 1 枚で俯瞰する。ケース数が増えても破綻しない。
    """
    tags = list(roll.keys())
    horizons = common_horizons(r.keys() for r in roll.values())
    if not horizons:
        return

    fig, axes = plt.subplots(1, 2, figsize=(7 + 1.2 * len(horizons), 1.5 + 0.6 * len(tags)))
    for ax, key, unit, title in [
        (axes[0], "pos", "cm", "位置 RMSE [cm]"),
        (axes[1], "yaw", "deg", "yaw RMSE [deg]"),
    ]:
        mat = np.array([[roll[tag][h][key] for h in horizons] for tag in tags])
        im = ax.imshow(mat, cmap="YlOrRd", aspect="auto")
        ax.set_xticks(range(len(horizons)), [f"N={h}" for h in horizons])
        ax.set_yticks(range(len(tags)), tags)
        ax.set_title(title, fontsize=11)
        # セル注釈: 背景の濃淡で文字色を切り替える
        thresh = mat.min() + (mat.max() - mat.min()) * 0.6
        for i in range(len(tags)):
            for j in range(len(horizons)):
                ax.text(
                    j, i, f"{mat[i, j]:.2f}",
                    ha="center", va="center", fontsize=9,
                    color="white" if mat[i, j] > thresh else "black",
                )
        fig.colorbar(im, ax=ax, label=unit, shrink=0.85)

    fig.suptitle("cases × horizon: N-step RMSE ヒートマップ", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_growth_relative(
    roll: dict[str, dict[int, dict[str, float]]], ref_tag: str, out_path: Path
) -> None:
    """reference 比の誤差成長 (RMSE(case)/RMSE(ref) vs N)。

    絶対 RMSE は全ケース共通のノイズ床 (小 N では実機計測ノイズが支配) に埋もれるため、
    reference との比を取って dynamics 差だけを浮き上がらせる。
    「N をどこまで伸ばすとモデル差が判別可能になるか」が読める。
    """
    ref = roll.get(ref_tag)
    if not ref:
        print(f"[WARN] reference_tag={ref_tag} の nstep 出力が無いため相対成長プロットをスキップ",
              file=sys.stderr)
        return
    horizons = common_horizons(r.keys() for r in roll.values())
    if not horizons:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    colors = dict(zip(roll.keys(), _color_cycle(len(roll))))

    for ax, key, title in [
        (axes[0], "pos", "位置 RMSE 比"),
        (axes[1], "yaw", "yaw RMSE 比"),
    ]:
        for tag, r in roll.items():
            if tag == ref_tag:
                continue
            ratio = [r[h][key] / max(ref[h][key], 1e-12) for h in horizons]
            ax.plot(horizons, ratio, "o-", color=colors[tag], lw=1.4, label=tag)
        ax.axhline(1.0, color="black", lw=1.0, ls="--", label=f"{ref_tag} (基準)")
        ax.set_xlabel("rollout 長 N [step]")
        ax.set_ylabel(f"RMSE(case) / RMSE({ref_tag})")
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=9)
        ax.grid(True, lw=0.5, alpha=0.6)

    fig.suptitle(
        f"cases overlay: N-step 相対誤差成長 (対 {ref_tag} 比)\n"
        "1.0 より上=基準より悪化 / 下=改善。位置比は小 N で共通ノイズ床に埋もれて 1.0 近傍。"
        "yaw 比は seed (k_us=0 の bicycle 逆算) 由来の差で小 N でも分離し、大 N ほど真の dynamics 差が支配",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def write_cases_summary(
    case_dfs: dict[str, pd.DataFrame],
    roll: dict[str, dict[int, dict[str, float]]],
    cases_cfg,
    out_path: Path,
) -> None:
    """N-step オープンループのケース横断 Markdown 表を出力する。

    N=1 詳細 RMSE 表 (steer/縦/横, reference との Δ 付き) と
    horizon 別 終端誤差 RMSE 表 (pos/yaw) を 1 ファイルに集約する。
    """
    rad2deg = 180.0 / math.pi
    ref_tag = cases_cfg.overlay.reference_tag

    # N=1 RMSE をケース別に集計
    ps: dict[str, dict[str, float]] = {}
    for tag, df in case_dfs.items():
        df1 = n1(df)
        ps[tag] = {
            "steer": float(np.sqrt(np.nanmean(df1["err_steer"].values ** 2))) * rad2deg,
            "long": float(np.sqrt(np.nanmean(df1["err_ds_long"].values ** 2))) * 100,
            "lat": float(np.sqrt(np.nanmean(df1["err_ds_lat"].values ** 2))) * 100,
            "n": float(len(df1)),
        }
    ref_steer = ps.get(ref_tag, {}).get("steer") if ref_tag else None

    lines: list[str] = ["# cases summary (N-step オープンループ)\n"]
    if ref_tag:
        lines.append(f"reference tag: `{ref_tag}`\n")
    lines.append("")

    # --- N=1 詳細 RMSE 表 (reference との Δ 付き) ---
    lines.append("## N=1 (毎ステップリセット) RMSE\n")
    lines.append(
        "| tag | vehicle_model | n_steps | RMSE err_steer [deg] | Δsteer vs ref [deg] | "
        "RMSE err_ds_long [cm] | RMSE err_ds_lat [cm] | note |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---:|---|")
    for case in cases_cfg.cases:
        if case.tag not in ps:
            lines.append(f"| {case.tag} | {case.vehicle_model} | — | — | — | — | — | 出力欠損 |")
            continue
        d = ps[case.tag]
        if case.tag == ref_tag:
            delta = "基準"
        elif ref_steer is not None:
            delta = f"{d['steer'] - ref_steer:+.4f}"
        else:
            delta = "—"
        lines.append(
            f"| {case.tag} | {case.vehicle_model} | {int(d['n'])} | "
            f"{d['steer']:.4f} | {delta} | {d['long']:.3f} | {d['lat']:.3f} |  |"
        )
    lines.append("")
    lines.append(
        "> N=1 は毎ステップ実機状態にリセットするため、k_us/wheelbase の累積差は "
        "位置 RMSE にほぼ現れない。dynamics 差は下記 horizon 別 RMSE 表を参照。"
    )
    lines.append("")

    # --- horizon 別 終端誤差 RMSE 表 (free-running, ケース横断) ---
    roll = {t: r for t, r in roll.items() if r}
    if roll:
        # 表示する horizon は全 case で共通に存在するものを採用
        horizons = common_horizons(r.keys() for r in roll.values())
        # 全ケース共通の horizon が無い (horizon 集合が不一致) 場合は
        # Δ 列の horizons[-1] 参照を避けるため reference 比較を無効化する。
        ref_roll = roll.get(ref_tag) if horizons else None
        lines.append("## horizon 別 終端誤差 RMSE (free-running, ケース横断)\n")
        head = "| tag |" + "".join(f" pos@N{h}[cm] |" for h in horizons) \
            + "".join(f" yaw@N{h}[deg] |" for h in horizons)
        if ref_roll:
            head += f" Δyaw@N{horizons[-1]} vs ref [deg] |"
        lines.append(head)
        lines.append("|---|" + "---:|" * (len(horizons) * 2 + (1 if ref_roll else 0)))
        for case in cases_cfg.cases:
            r = roll.get(case.tag)
            if not r:
                continue
            row = f"| {case.tag} |"
            row += "".join(f" {r[h]['pos']:.2f} |" for h in horizons)
            row += "".join(f" {r[h]['yaw']:.3f} |" for h in horizons)
            if ref_roll:
                hl = horizons[-1]
                if case.tag == ref_tag:
                    row += " 基準 |"
                else:
                    row += f" {r[hl]['yaw'] - ref_roll[hl]['yaw']:+.3f} |"
            lines.append(row)
        lines.append("")
        lines.append(
            "> N ステップ連続予測 (途中リセット無し) の終端誤差。N を増やすほど dynamics 差が "
            "累積し、k_us/wheelbase の効果が分離する (パラメータ同定の指標)。"
            "小 N の yaw RMSE は seed (k_us=0 bicycle 逆算) 由来のバイアスを含む点に注意。"
        )
        lines.append("")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  Saved: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="cases 集約解析 (Stage 4)")
    parser.add_argument(
        "--cases-config",
        default=os.environ.get("CASES_CONFIG_YAML", ""),
        help="cases.yaml のパス (必須; env: CASES_CONFIG_YAML)",
    )
    parser.add_argument(
        "--base-dir",
        default=os.environ.get("BEST_MODEL_BASE_DIR", ""),
        help="comparison/nstep/ の親ディレクトリ (env: BEST_MODEL_BASE_DIR)",
    )
    args = parser.parse_args()

    if not args.cases_config:
        print("ERROR: --cases-config (or CASES_CONFIG_YAML env) が未指定です", file=sys.stderr)
        sys.exit(2)
    if not args.base_dir:
        print("ERROR: --base-dir (or BEST_MODEL_BASE_DIR env) が未指定です", file=sys.stderr)
        sys.exit(2)

    from driving_log_replayer_v2.real_log_sim_comparison.lib._cases_config import (  # noqa: PLC0415
        load_cases_config,
    )

    cases_cfg = load_cases_config(args.cases_config)
    nstep_root = Path(args.base_dir) / "comparison" / "nstep"
    out_root = Path(args.base_dir) / "comparison" / "cases"
    overlay_dir = out_root / "overlay"
    overlay_dir.mkdir(parents=True, exist_ok=True)

    case_dfs = _load_case_csvs(nstep_root, cases_cfg.tags)
    if not case_dfs:
        print("ERROR: 全 case の CSV が見つかりません。Stage 3 が成功したか確認してください",
              file=sys.stderr)
        sys.exit(1)

    # ケース横断で軸範囲を統一して nstep/<tag>/ の図を上書き再描画
    rerender_case_figures(case_dfs, cases_cfg, nstep_root, Path(args.base_dir))

    horizons = common_horizons(df["horizon"].unique() for df in case_dfs.values())
    plots_wanted = set(cases_cfg.overlay.plots)
    if "cascade_error" in plots_wanted:
        plot_cascade_error_overlay(case_dfs, overlay_dir / "cascade_error_overlay.svg")
    if "error_timeseries" in plots_wanted and horizons:
        # N=min (通常 1) と N=max の 2 枚。単一 horizon なら 1 枚のみ。
        for h in sorted({horizons[0], horizons[-1]}):
            plot_error_timeseries_overlay(
                case_dfs, h, overlay_dir / f"error_timeseries_overlay_n{h}.svg"
            )

    # horizon 横断集約 (誤差成長・ヒートマップ・相対成長・RMSE 表)
    roll = {tag: rmse_by_horizon(df) for tag, df in case_dfs.items()}
    plot_error_growth_overlay(roll, overlay_dir / "error_growth_overlay.svg")
    plot_rmse_heatmap(roll, overlay_dir / "rmse_heatmap.svg")
    plot_growth_relative(
        roll, cases_cfg.overlay.reference_tag, overlay_dir / "growth_relative.svg"
    )
    write_cases_summary(case_dfs, roll, cases_cfg, out_root / "cases_summary.md")

    print(f"\n完了。出力先: {out_root}")


if __name__ == "__main__":
    main()
