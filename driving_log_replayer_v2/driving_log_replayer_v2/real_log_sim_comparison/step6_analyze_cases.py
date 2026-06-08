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

from .lib._fig_io import write_fig_json
from .lib._figures import (
    build_fig_cascade_error_overlay,
    build_fig_error_growth_overlay,
    build_fig_error_timeseries_overlay,
    build_fig_growth_relative,
    build_fig_rmse_heatmap,
)
from .lib._nstep_common import (
    ERR_METRICS,
    YAW_SEED_NOTE,
    common_horizons,
    metrics_description_md,
    n1,
    rmse_by_horizon,
)

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
    fig = build_fig_cascade_error_overlay({tag: n1(df) for tag, df in case_dfs.items()})
    write_fig_json(fig, out_path)


def plot_error_timeseries_overlay(
    case_dfs: dict[str, pd.DataFrame], horizon: int, out_path: Path
) -> None:
    """指定 horizon の終端誤差時系列を全 case 重ね描き。

    N=1 では従来の per-step 誤差比較、N=max では RMSE 集約で消える
    「コースのどこ (どのカーブ) でどのモデルが乖離するか」の比較になる。
    """
    fig = build_fig_error_timeseries_overlay(case_dfs, horizon)
    write_fig_json(fig, out_path)


def plot_error_growth_overlay(
    roll: dict[str, dict[int, dict[str, float]]], out_path: Path
) -> None:
    """全 case の N-step 誤差成長 (horizon 別 RMSE) を 1 枚に重ね描き。

    roll: tag → {horizon → {"pos","long","lat","yaw"}} (rmse_by_horizon の集約)。
    """
    fig = build_fig_error_growth_overlay(roll)
    write_fig_json(fig, out_path)


def plot_rmse_heatmap(
    roll: dict[str, dict[int, dict[str, float]]], out_path: Path
) -> None:
    """case × horizon の RMSE ヒートマップ (位置/yaw の 2 パネル、数値注釈付き)。

    cases_summary.md の表の図版化。2 変数 (シミュレータ構成 × rollout 長) の
    全組み合わせを 1 枚で俯瞰する。ケース数が増えても破綻しない。
    """
    horizons = common_horizons(r.keys() for r in roll.values())
    fig = build_fig_rmse_heatmap(roll, horizons)
    if fig is None:
        return
    write_fig_json(fig, out_path)


def plot_growth_relative(
    roll: dict[str, dict[int, dict[str, float]]], ref_tag: str, out_path: Path
) -> None:
    """reference 比の誤差成長 (RMSE(case)/RMSE(ref) vs N)。

    絶対 RMSE は全ケース共通のノイズ床 (小 N では実機計測ノイズが支配) に埋もれるため、
    reference との比を取って dynamics 差だけを浮き上がらせる。
    「N をどこまで伸ばすとモデル差が判別可能になるか」が読める。
    """
    if ref_tag not in roll:
        print(f"[WARN] reference_tag={ref_tag} の nstep 出力が無いため相対成長プロットをスキップ",
              file=sys.stderr)
        return
    horizons = common_horizons(r.keys() for r in roll.values())
    fig = build_fig_growth_relative(roll, ref_tag, horizons)
    if fig is None:
        return
    write_fig_json(fig, out_path)


def write_cases_summary(
    case_dfs: dict[str, pd.DataFrame],
    roll: dict[str, dict[int, dict[str, float]]],
    cases_cfg,
    out_path: Path,
) -> None:
    """N-step オープンループのケース横断 Markdown 表を出力する。

    N=1 詳細 RMSE 表 (steer/縦/横, reference との Δ 付き) と
    horizon 別 終端誤差 RMSE 表 (縦/横/yaw) を 1 ファイルに集約する。
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
    lines.append(metrics_description_md())
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
        head = "| tag |" + "".join(f" 縦@N{h}[cm] |" for h in horizons) \
            + "".join(f" 横@N{h}[cm] |" for h in horizons) \
            + "".join(f" yaw@N{h}[deg] |" for h in horizons)
        if ref_roll:
            head += f" Δyaw@N{horizons[-1]} vs ref [deg] |"
        lines.append(head)
        lines.append("|---|" + "---:|" * (len(horizons) * 3 + (1 if ref_roll else 0)))
        for case in cases_cfg.cases:
            r = roll.get(case.tag)
            if not r:
                continue
            row = f"| {case.tag} |"
            row += "".join(f" {r[h]['long']:.2f} |" for h in horizons)
            row += "".join(f" {r[h]['lat']:.2f} |" for h in horizons)
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
