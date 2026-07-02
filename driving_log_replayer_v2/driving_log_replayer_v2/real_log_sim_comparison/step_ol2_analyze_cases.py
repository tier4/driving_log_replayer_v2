"""Stage 4: VehicleModel N-step 解析の全ケース集約 (overlay 図 + RMSE 表).

入力:
  - cases.yaml (Stage 3 が走らせた各ケースの定義)
  - comparison/nstep/<tag>/nstep_delta.csv (Stage 3 出力, 全 tag・全 horizon 統一スキーマ)
  - lite/real.lite (物理妥当性検証用、Stage 1 出力)

出力:
  comparison/cases/
    ├── overlay/
    │   ├── cascade_error_overlay.svg            # N=1 段階的誤差 (ステア応答→横位置)
    │   └── error_growth_overlay.svg             # horizon 別 RMSE 成長 重ね描き
    ├── physical_validity/
    │   ├── long_fit.fig.json                    # 縦方向モデルフィット (実測/指令/同定値/チューン値)
    │   ├── steer_fit.fig.json                   # 操舵モデルフィット
    │   └── kus_bins.fig.json                    # k_us(v) 速度ビン別最小二乗法推定
    ├── cases_summary.md   # N=1 詳細 RMSE 表 + horizon 別 RMSE 表 + 物理妥当性サマリ
    └── cases_metrics.json # 上記の機械可読版 ("physical_validity" キーに同定値を含む)

誤差時系列の対話的確認は縦横モデル検証ビューア (step4: lon_lat_model.html) の
誤差パネルへ移設済み。case × horizon の俯瞰は cases_summary.md の表が担う。

欠損ケース (CSV が無い) は警告ログを出してスキップ。集約処理は continue できる。

また、全ケースの CSV を持つ本ステージが nstep/<tag>/ のケース別図を
「ケース横断で軸範囲を統一」して上書き再描画する (rerender_case_figures)。
シミュレータ (ケース) ごとに図が分かれていてもレポートのタブ切替で軸が動かない。

**物理妥当性検証** (`analyze_physical_validity`) は Conditions.cases の N-way スイープとは
独立な軸で、real.lite から縦(τ_a/T_a + 路面勾配補正)・操舵(τ_δ/T_δ)・横 k_us(v) を
直接同定し、Conditions.cases の全モデルパラメータ (base マージ済みチューニング値) と比較する
2-way (同定値 vs チューニング値) の妥当性検証。overlay.reference_tag は baseline 用の
指定であり「チューニング値」ではないため、単体ではなく cases 全モデルを重ね描き対象にする。
既存の N-way 集約 (cases_summary/cases_metrics の "cases" キー) は変更しない —
"physical_validity" キーとして追加するのみ。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from .lib._fig_io import write_fig_json
from .lib._figures import (
    build_fig_cascade_error_overlay,
    build_fig_error_growth_overlay,
    build_fig_kus_single,
    build_fig_long_single,
    build_fig_steer_single,
)
from .lib._io import resolve_lite_bag
from .lib._nstep_common import (
    common_horizons,
    build_cases_metrics_payload,
    horizon_summary_lines,
    load_case_csvs,
    metrics_description_md,
    n1,
    n1_summary_lines,
    rmse_by_horizon,
)
from .lib._physical_validity import (
    compute_kus_bins_single,
    compute_long_timeseries,
    compute_steer_timeseries,
    fit_long_single,
    fit_steer_single,
)

VERBOSE = False


def rerender_case_figures(
    case_dfs: dict[str, pd.DataFrame],
    cases_cfg,
    nstep_root: Path,
    base_dir: Path,
) -> None:
    """ケース横断で軸範囲を統一して nstep/<tag>/ の図を再描画する。

    step5 はケース単体実行のため各図の軸が自ケースの値域で自動スケールされ、
    レポートのケース切替タブで軸が動いて比較しにくい。全ケースの CSV を持つ
    step_ol2 が、step_ol1 のプロット関数 (LIMITS_DF 設定で統一軸モード) を呼び直して
    同名ファイルを上書きする。
    """
    from . import step_ol1_analyze_nstep as s5  # noqa: PLC0415 (plotly 含む重 import のため遅延)

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
            if VERBOSE:
                print(f"  [{case.tag}] ケース横断の統一軸で再描画")
            s5.plot_map_distribution(df, params)
            s5.plot_overview(df1, params)
    finally:
        s5.LIMITS_DF = None


def analyze_physical_validity(real_lite: Path | None, models: dict[str, dict]) -> dict | None:
    """real.lite から縦・操舵・横方向の物理妥当性同定を行う。

    Conditions.cases の N-way スイープとは独立な 2-way (実測同定 vs チューニング値) 検証。
    models: {ケース tag: raw params (case.params、base 未マージ)}。base マージは描画側で行う。
    real.lite が無い、またはいずれの軸も同定不能なら None (呼び出し側は WARN のみで継続)。
    """
    if real_lite is None:
        if VERBOSE:
            print("[WARN] real.lite が見つからないため物理妥当性検証をスキップ", file=sys.stderr)
        return None

    long_fit = fit_long_single(real_lite)
    steer_fit = fit_steer_single(real_lite)
    kus_bins = compute_kus_bins_single(real_lite)

    if long_fit is None and steer_fit is None and kus_bins is None:
        if VERBOSE:
            print("[WARN] 物理妥当性検証: 縦・操舵・横 k_us のいずれも同定不能", file=sys.stderr)
        return None

    return {"long": long_fit, "steer": steer_fit, "kus_bins": kus_bins, "models": models}


def write_physical_validity_figures(pv: dict | None, real_lite: Path | None, out_dir: Path) -> None:
    """物理妥当性検証の図 (long_fit/steer_fit/kus_bins.fig.json) を書き出す。"""
    if pv is None or real_lite is None:
        return
    models = pv.get("models") or {}

    long_ts = compute_long_timeseries(real_lite, pv.get("long"), models)
    write_fig_json(build_fig_long_single(long_ts, pv.get("long")), out_dir / "long_fit")

    steer_ts = compute_steer_timeseries(real_lite, pv.get("steer"), models)
    write_fig_json(build_fig_steer_single(steer_ts, pv.get("steer")), out_dir / "steer_fit")

    write_fig_json(build_fig_kus_single(pv.get("kus_bins"), models), out_dir / "kus_bins")


def _physical_validity_summary_lines(pv: dict | None) -> list[str]:
    """cases_summary.md に追記する物理妥当性検証セクションの Markdown 行。"""
    lines = ["## 物理妥当性検証 (実測同定 vs チューニング値)\n"]
    if pv is None:
        lines.append("real.lite が無い、または同定に必要なデータが不足しているためスキップ。\n")
        return lines

    lines.append(
        "Conditions.cases の N-way スイープとは独立に、real.lite から車両モデルの物理パラメータを"
        "直接同定し、Conditions.cases の各モデルのチューニング値 (base マージ済み) と比較する"
        " (2-way 検証)。\n"
    )

    long_fit = pv.get("long")
    if long_fit is not None:
        lines.append(
            f"- **縦方向** (路面勾配補正込み): τ_a={long_fit['tau']:.3f}s, "
            f"T_a={long_fit['delay']:.3f}s, RMSE={long_fit['rmse_mps2']:.3f} m/s², "
            f"n_dyn={long_fit['n_dyn']}, "
            f"pitch range {math.degrees(long_fit['pitch_min']):+.2f}°〜{math.degrees(long_fit['pitch_max']):+.2f}°"
        )
    else:
        lines.append("- **縦方向**: 動的区間不足のため同定不能")

    steer_fit = pv.get("steer")
    if steer_fit is not None:
        lines.append(
            f"- **操舵**: τ_δ={steer_fit['tau']:.3f}s, T_δ={steer_fit['delay']:.3f}s, "
            f"RMSE={steer_fit['rmse_mrad']:.1f} mrad, n_dyn={steer_fit['n_dyn']}"
        )
    else:
        lines.append("- **操舵**: 動的区間不足のため同定不能")

    kus_bins = pv.get("kus_bins")
    if kus_bins is not None:
        n_curve = int(np.sum(kus_bins["n_pts"]))
        lines.append(f"- **横方向 k_us(v)**: 曲線走行サンプル数={n_curve} (詳細は kus_bins.fig.json)")
    else:
        lines.append("- **横方向 k_us(v)**: 曲線走行サンプル不足のため同定不能")

    lines.append("")
    return lines


def plot_cascade_error_overlay(
    case_dfs: dict[str, pd.DataFrame], out_path: Path
) -> None:
    """全 case の N=1 err_* を 1 枚に重ね描き (各段は real / sim / err の 3 系列)."""
    fig = build_fig_cascade_error_overlay({tag: n1(df) for tag, df in case_dfs.items()})
    write_fig_json(fig, out_path)


def plot_error_growth_overlay(
    roll: dict[str, dict[int, dict[str, float]]], out_path: Path
) -> None:
    """全 case の N-step 誤差成長 (horizon 別 RMSE) を 1 枚に重ね描き。

    roll: tag → {horizon → {"pos","long","lat","yaw"}} (rmse_by_horizon の集約)。
    """
    fig = build_fig_error_growth_overlay(roll)
    write_fig_json(fig, out_path)


def write_cases_summary(
    case_dfs: dict[str, pd.DataFrame],
    roll: dict[str, dict[int, dict[str, float]]],
    cases_cfg,
    out_path: Path,
    physical_validity: dict | None = None,
) -> None:
    """N-step オープンループのケース横断 Markdown 表を出力する。

    N=1 詳細 RMSE 表 (steer/縦/横, reference との Δ 付き) と
    horizon 別 終端誤差 RMSE 表 (縦/横/yaw) を 1 ファイルに集約する。
    """
    lines: list[str] = ["# cases summary (N-step オープンループ)\n"]
    ref_tag = cases_cfg.overlay.reference_tag
    if ref_tag:
        lines.append(f"reference tag: `{ref_tag}`\n")
    lines.append("")
    lines.append(metrics_description_md())
    lines.append("")
    lines += n1_summary_lines(case_dfs, cases_cfg)
    lines += horizon_summary_lines(roll, cases_cfg)
    lines += _physical_validity_summary_lines(physical_validity)

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    if VERBOSE:
        print(f"  Saved: {out_path}")


def write_cases_metrics(
    case_dfs: dict[str, pd.DataFrame],
    roll: dict[str, dict[int, dict[str, float]]],
    cases_cfg,
    horizons: list[int],
    out_path: Path,
    physical_validity: dict | None = None,
) -> None:
    """ケース横断の horizon 別終端誤差 RMSE を機械可読 JSON で出力する。

    step13_cross_dataset がデータセット横断行列・正規化集約の入力に使う
    (per-dataset の open-loop 側メトリクスはこの 1 ファイルで完結する)。
    reference_tag / horizons を含めることで step13 が DS 間の評価条件整合を検査できる。

    "physical_validity" キーは Conditions.cases の N-way スイープ ("cases" キー) とは独立に
    追加する物理妥当性検証の機械可読 SSOT (schema は lib._physical_validity の
    physical_validity_jsonable 参照)。
    """
    payload = build_cases_metrics_payload(
        case_dfs,
        roll,
        cases_cfg,
        horizons,
        physical_validity,
    )
    out_path.write_text(
        json.dumps(payload, ensure_ascii=False, allow_nan=False, indent=1), encoding="utf-8"
    )
    if VERBOSE:
        print(f"  Saved: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="cases 集約解析 (Stage 4)")
    parser.add_argument(
        "--scenario",
        default=os.environ.get("SCENARIO_CONFIG_YAML", ""),
        help="scenario.yaml のパス (Conditions.models / cases / overlay を含む; "
             "env: SCENARIO_CONFIG_YAML)",
    )
    parser.add_argument(
        "--base-dir",
        default=os.environ.get("BEST_MODEL_BASE_DIR", ""),
        help="comparison/nstep/ の親ディレクトリ (env: BEST_MODEL_BASE_DIR)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="詳細情報を出力する",
    )
    args = parser.parse_args()
    base_dir = Path(args.base_dir)

    global VERBOSE
    VERBOSE = args.verbose
    if not VERBOSE:
        import warnings
        warnings.simplefilter('ignore')

    if not args.scenario:
        print("ERROR: --scenario (or SCENARIO_CONFIG_YAML env) が未指定です", file=sys.stderr)
        sys.exit(2)
    if not args.base_dir:
        print("ERROR: --base-dir (or BEST_MODEL_BASE_DIR env) が未指定です", file=sys.stderr)
        sys.exit(2)

    from driving_log_replayer_v2.real_log_sim_comparison.lib._cases_config import (  # noqa: PLC0415
        load_cases_config,
    )

    cases_cfg = load_cases_config(args.scenario)
    nstep_root = base_dir / "comparison" / "nstep"
    out_root = base_dir / "comparison" / "cases"
    overlay_dir = out_root / "overlay"
    overlay_dir.mkdir(parents=True, exist_ok=True)

    case_dfs = load_case_csvs(nstep_root, cases_cfg.tags, verbose=VERBOSE)
    if not case_dfs:
        print("ERROR: 全 case の CSV が見つかりません。Stage 3 が成功したか確認してください",
              file=sys.stderr)
        sys.exit(1)

    # ケース横断で軸範囲を統一して nstep/<tag>/ の図を上書き再描画
    rerender_case_figures(case_dfs, cases_cfg, nstep_root, base_dir)

    horizons = common_horizons(df["horizon"].unique() for df in case_dfs.values())
    plots_wanted = set(cases_cfg.overlay.plots)
    if "cascade_error" in plots_wanted:
        plot_cascade_error_overlay(case_dfs, overlay_dir / "cascade_error_overlay.svg")

    # horizon 横断集約 (誤差成長 + RMSE 表)。誤差時系列は縦横モデル検証ビューアの誤差パネルへ移設。
    roll = {tag: rmse_by_horizon(df) for tag, df in case_dfs.items()}
    plot_error_growth_overlay(roll, overlay_dir / "error_growth_overlay.svg")

    # 物理妥当性検証 (Conditions.cases の N-way スイープとは独立、real.lite から直接同定)。
    # real.lite 欠損や同定不能は WARN のみで継続し、既存 of N-way 集約には影響しない。
    lite_dir = base_dir / "lite"
    real_lite = resolve_lite_bag(lite_dir, "real")
    # overlay.reference_tag は baseline 用の指定であり「チューニング値」ではないため、
    # 単体ではなく Conditions.cases の全モデルを重ね描き対象にする (base マージは描画側で行う)。
    models = {c.tag: c.params for c in cases_cfg.cases}
    pv = analyze_physical_validity(real_lite, models)
    pv_dir = out_root / "physical_validity"
    pv_dir.mkdir(parents=True, exist_ok=True)
    write_physical_validity_figures(pv, real_lite, pv_dir)

    write_cases_summary(
        case_dfs, roll, cases_cfg, out_root / "cases_summary.md", physical_validity=pv,
    )
    write_cases_metrics(
        case_dfs, roll, cases_cfg, horizons, out_root / "cases_metrics.json", physical_validity=pv,
    )

    if VERBOSE:
        print(f"\n完了。出力先: {out_root}")


if __name__ == "__main__":
    main()
