"""Stage 4: VehicleModel 解析の全ケース集約 (overlay 図 + RMSE 表).

入力:
  - cases.yaml (Stage 3 が走らせた各ケースの定義)
  - comparison/per_step/<tag>/per_step_delta.csv (Stage 3 出力, 全 tag 分)

出力:
  comparison/cases/
    ├── overlay/
    │   ├── cascade_error_overlay.png
    │   └── error_timeseries_overlay.png
    └── cases_summary.md         # 各 tag の RMSE を 1 表に集約

欠損ケース (CSV が無い) は警告ログを出してスキップ。集約処理は continue できる。
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

# Japanese font (best-effort; absent ならフォールバック)。
# rcParams への代入は例外を投げないので、実在フォントを font_manager で確認する。
_installed_fonts = {f.name for f in _fm.fontManager.ttflist}
for _f in ("Noto Sans CJK JP", "TakaoGothic", "IPAGothic", "DejaVu Sans"):
    if _f in _installed_fonts:
        plt.rcParams["font.family"] = _f
        break


_CASCADE_ROWS = [
    # (real_col, sim_col, err_col, scale, ylabel, title)
    ("real_steer_kp1", "sim_steer_kp1", "err_steer", 180.0 / math.pi,
     "ステア角 [deg]", "① ステア応答 (cmd→actual): err_steer"),
    ("real_ds_lat", "sim_ds_lat", "err_ds_lat", 100.0,
     "横方向 Δpos [cm]", "② 横方向 1ステップ変位: err_ds_lat"),
]


def _color_cycle(n: int) -> list[str]:
    base = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    return [base[i % len(base)] for i in range(n)]


def _load_case_csvs(per_step_root: Path, tags: list[str]) -> dict[str, pd.DataFrame]:
    out: dict[str, pd.DataFrame] = {}
    for tag in tags:
        csv = per_step_root / tag / "per_step_delta.csv"
        if not csv.exists():
            print(f"[WARN] {csv} が無いため case={tag} をスキップ", file=sys.stderr)
            continue
        out[tag] = pd.read_csv(csv)
    return out


def plot_cascade_error_overlay(
    case_dfs: dict[str, pd.DataFrame], out_path: Path
) -> None:
    """全 case の err_* を 1 枚に重ね描き (各段は real / sim / err の 3 系列)."""
    # _CASCADE_ROWS は 2 要素固定なので axes は常に 2D 配列
    fig, axes = plt.subplots(
        len(_CASCADE_ROWS), 2, figsize=(16, 4 * len(_CASCADE_ROWS)),
        gridspec_kw={"width_ratios": [3, 1]},
    )

    colors = dict(zip(case_dfs.keys(), _color_cycle(len(case_dfs))))

    for row_idx, (real_col, sim_col, err_col, scale, ylabel, title) in enumerate(_CASCADE_ROWS):
        ax_ts = axes[row_idx, 0]
        ax_err = axes[row_idx, 1]

        for tag, df in case_dfs.items():
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
        first_df = next(iter(case_dfs.values()))
        ax_ts.plot(first_df["tr"].values, first_df[real_col].values * scale,
                   color="black", lw=1.5, ls="--", label="real", zorder=10)

        ax_ts.axhline(0, color="black", lw=0.5, ls=":")
        ax_ts.set_ylabel(ylabel)
        ax_ts.set_title(title, fontsize=10)
        ax_ts.legend(fontsize=8, ncol=2)
        ax_ts.grid(True, alpha=0.3)

        ax_err.axhline(0, color="black", lw=0.6)
        ax_err.set_ylabel(f"誤差 [{ylabel.split('[')[1]}")
        ax_err.set_title(f"誤差 (real − sim) — case 別 RMSE", fontsize=10)
        ax_err.legend(fontsize=8)
        ax_err.grid(True, alpha=0.3)

    for col in range(2):
        axes[-1, col].set_xlabel("AUTONOMOUS 開始からの時刻 [s]")

    fig.suptitle("cases overlay: 段階的誤差 (ステア応答 → 横位置)", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_error_timeseries_overlay(
    case_dfs: dict[str, pd.DataFrame], out_path: Path
) -> None:
    """全 case の err_steer / err_ds_long / err_ds_lat を 1 枚に重ね描き."""
    cols = [
        ("err_steer", 180.0 / math.pi, "deg", "ステア予測誤差"),
        ("err_ds_long", 100.0, "cm", "縦方向 1ステップ誤差"),
        ("err_ds_lat", 100.0, "cm", "横方向 1ステップ誤差"),
    ]
    # cols は 3 要素固定なので axes は常に 1D 配列
    fig, axes = plt.subplots(len(cols), 1, figsize=(14, 3.5 * len(cols)), sharex=True)

    colors = dict(zip(case_dfs.keys(), _color_cycle(len(case_dfs))))

    for ax, (col, scale, unit, title) in zip(axes, cols):
        for tag, df in case_dfs.items():
            err = df[col].values * scale
            rmse = float(np.sqrt(np.nanmean(err ** 2)))
            ax.plot(df["tr"].values, err, color=colors[tag], lw=1.0, alpha=0.85,
                    label=f"{tag}  RMSE={rmse:.3g} {unit}")
        ax.axhline(0, color="black", lw=0.6)
        ax.set_ylabel(f"誤差 [{unit}]")
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=9, ncol=2)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
    fig.suptitle("cases overlay: 誤差時系列", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def write_cases_summary(
    case_dfs: dict[str, pd.DataFrame],
    cases_cfg,
    out_path: Path,
) -> None:
    """case × RMSE の Markdown 表を出力."""
    rad2deg = 180.0 / math.pi
    lines: list[str] = []
    lines.append("# cases summary\n")
    if cases_cfg.overlay.reference_tag:
        lines.append(f"reference tag: `{cases_cfg.overlay.reference_tag}`\n")
    lines.append("")
    lines.append(
        "| tag | vehicle_model | n_steps | RMSE err_steer [deg] | "
        "RMSE err_ds_long [cm] | RMSE err_ds_lat [cm] | note |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---|")

    for case in cases_cfg.cases:
        if case.tag not in case_dfs:
            lines.append(
                f"| {case.tag} | {case.vehicle_model} | — | — | — | — | 出力欠損 |"
            )
            continue
        df = case_dfs[case.tag]
        rmse_steer = float(np.sqrt(np.nanmean(df["err_steer"].values ** 2))) * rad2deg
        rmse_long = float(np.sqrt(np.nanmean(df["err_ds_long"].values ** 2))) * 100
        rmse_lat = float(np.sqrt(np.nanmean(df["err_ds_lat"].values ** 2))) * 100
        lines.append(
            f"| {case.tag} | {case.vehicle_model} | {len(df)} | "
            f"{rmse_steer:.4f} | {rmse_long:.3f} | {rmse_lat:.3f} |  |"
        )

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
        help="comparison/per_step/ の親ディレクトリ (env: BEST_MODEL_BASE_DIR)",
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
    per_step_root = Path(args.base_dir) / "comparison" / "per_step"
    out_root = Path(args.base_dir) / "comparison" / "cases"
    overlay_dir = out_root / "overlay"
    overlay_dir.mkdir(parents=True, exist_ok=True)

    case_dfs = _load_case_csvs(per_step_root, cases_cfg.tags)
    if not case_dfs:
        print("ERROR: 全 case の CSV が見つかりません。Stage 3 が成功したか確認してください",
              file=sys.stderr)
        sys.exit(1)

    plots_wanted = set(cases_cfg.overlay.plots)
    if "cascade_error" in plots_wanted:
        plot_cascade_error_overlay(case_dfs, overlay_dir / "cascade_error_overlay.png")
    if "error_timeseries" in plots_wanted:
        plot_error_timeseries_overlay(case_dfs, overlay_dir / "error_timeseries_overlay.png")

    write_cases_summary(case_dfs, cases_cfg, out_root / "cases_summary.md")

    print(f"\n完了。出力先: {out_root}")


if __name__ == "__main__":
    main()
