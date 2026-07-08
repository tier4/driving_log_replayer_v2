#!/usr/bin/env python3
"""[Step4b] matplotlib 図とテキストの簡易妥当性レポート。"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager
import matplotlib.pyplot as plt
import numpy as np
import yaml

# 日本語ラベル (縦方向/操舵 等) が文字化けしないよう、環境にあれば Noto Sans CJK を使う。
# 無ければ matplotlib の既定フォントにフォールバックする (文字化けするが実行は継続する)。
for _font in ("Noto Sans CJK JP", "IPAexGothic", "IPAGothic"):
    if _font in {f.name for f in matplotlib.font_manager.fontManager.ttflist}:
        matplotlib.rcParams["font.family"] = _font
        break

from . import fit_core
from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .physical_constants import DWZ_MAX, VX_MIN_CURVE, WZ_MIN
from .scenario_params import resolve_wheelbase
from .settings import (
    K_US_CLIP,
    REPORT_MAX_PLOT_DATASETS,
    REPORT_PLOT_WINDOW_S,
    REPORT_SAMPLE_DATASETS,
    REPORT_VALID_ACC_TAU_BOUNDS,
    REPORT_VALID_STEER_TAU_BOUNDS,
    RESAMPLE_DT,
    STEER_CLIP_RAD,
)

REQUIRED_REPORT_PARAMS = (
    "acc_time_constant",
    "acc_time_delay",
    "debug_acc_scaling_factor",
    "steer_time_constant",
    "steer_time_delay",
    "debug_steer_scaling_factor",
    "steer_bias",
    "k_us",
)


def load_tuned_params(path: Path) -> dict:
    with Path(path).open("r") as f:
        data = yaml.safe_load(f)
    params = dict(data.get("params", data))
    missing = sorted(k for k in REQUIRED_REPORT_PARAMS if k not in params)
    if missing:
        raise ValueError(f"tuned params に必要キーがありません: {missing}")
    return params


def _sample_datasets(collection_dir: Path, n: int) -> list[dict]:
    tasks = discover_cached_datasets(collection_dir)[:n]
    out = []
    for ds_id, csv_path in tasks:
        dfs = read_dataset_csv(csv_path)
        ds = build_resampled(dfs, RESAMPLE_DT, context=f"report:{ds_id}")
        if ds is not None:
            ds["dataset_id"] = ds_id
            out.append(ds)
    return out


def _channel_rmse(cmd: np.ndarray, act: np.ndarray, sim: np.ndarray, mask: np.ndarray) -> float:
    if mask.sum() == 0:
        return float("nan")
    return float(np.sqrt(np.mean((sim[mask] - act[mask]) ** 2)))


def build_report_figure(datasets: list[dict], params: dict, wheelbase: float) -> tuple[plt.Figure, dict]:
    """縦・操舵の当てはめ品質重畳プロット + k_us 散布図を1枚の図にまとめる。"""
    n_ds = min(len(datasets), REPORT_MAX_PLOT_DATASETS)
    fig, axes = plt.subplots(n_ds + 1, 2, figsize=(12, 3.2 * (n_ds + 1)), squeeze=False)
    metrics: dict = {"long_rmse": [], "steer_rmse": []}

    acc_delay_n = int(round(params["acc_time_delay"] / RESAMPLE_DT))
    steer_delay_n = int(round(params["steer_time_delay"] / RESAMPLE_DT))

    for i in range(n_ds):
        ds = datasets[i]
        n_win = min(len(ds["a_cmd"]), int(REPORT_PLOT_WINDOW_S / RESAMPLE_DT))
        t = np.arange(n_win) * RESAMPLE_DT

        sim_a = fit_core.sim_first_order(
            ds["a_cmd"][:n_win].astype(float), params["acc_time_constant"],
            acc_delay_n, RESAMPLE_DT, y0=float(ds["a_act"][0]),
        ) * params["debug_acc_scaling_factor"]
        ax = axes[i, 0]
        ax.plot(t, ds["a_cmd"][:n_win], label="a_cmd", alpha=0.5)
        ax.plot(t, ds["a_act"][:n_win], label="a_act (実測)", linewidth=1.5)
        ax.plot(t, sim_a, label="sim (tuned)", linewidth=1.2, linestyle="--")
        ax.set_title(f"[{ds['dataset_id']}] 縦方向")
        ax.set_ylabel("accel [m/s^2]")
        ax.legend(fontsize=7)

        sim_d = fit_core.sim_first_order(
            ds["d_cmd"][:n_win].astype(float), params["steer_time_constant"],
            steer_delay_n, RESAMPLE_DT, y0=float(ds["d_act"][0]),
        ) * params["debug_steer_scaling_factor"] + params["steer_bias"]
        ax2 = axes[i, 1]
        ax2.plot(t, ds["d_cmd"][:n_win], label="d_cmd", alpha=0.5)
        ax2.plot(t, ds["d_act"][:n_win], label="d_act (実測)", linewidth=1.5)
        ax2.plot(t, sim_d, label="sim (tuned)", linewidth=1.2, linestyle="--")
        ax2.set_title(f"[{ds['dataset_id']}] 操舵")
        ax2.set_ylabel("steer [rad]")
        ax2.legend(fontsize=7)

        mask_a = ds["gear_drive"][:n_win]
        mask_d = ds["gear_drive"][:n_win]
        metrics["long_rmse"].append(_channel_rmse(ds["a_cmd"][:n_win], ds["a_act"][:n_win], sim_a, mask_a))
        metrics["steer_rmse"].append(_channel_rmse(ds["d_cmd"][:n_win], ds["d_act"][:n_win], sim_d, mask_d))

    # k_us 散布図 (全サンプルデータセットをプール)
    all_vx, all_wz, all_steer, all_dwz, all_gear = [], [], [], [], []
    for ds in datasets:
        wz = ds["vx"] * 0 + ds["wz"]  # copy
        dwz_mid = np.diff(wz) / RESAMPLE_DT
        dwz = np.empty_like(wz)
        dwz[0] = dwz_mid[0] if len(dwz_mid) else 0.0
        dwz[-1] = dwz_mid[-1] if len(dwz_mid) else 0.0
        dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])
        all_vx.append(ds["vx"])
        all_wz.append(wz)
        all_steer.append(ds["d_act"])
        all_dwz.append(dwz)
        all_gear.append(ds["gear_drive"])

    ax_k = axes[n_ds, 0]
    if all_vx:
        vx_all = np.concatenate(all_vx)
        wz_all = np.concatenate(all_wz)
        steer_all = np.concatenate(all_steer)
        dwz_all = np.concatenate(all_dwz)
        gear_all = np.concatenate(all_gear)
        mask_ok = gear_all & (np.abs(wz_all) > WZ_MIN) & (np.abs(dwz_all) < DWZ_MAX) & (vx_all > VX_MIN_CURVE)
        x = vx_all[mask_ok] * wz_all[mask_ok]
        y = (
            np.tan(np.clip(steer_all[mask_ok] - params["steer_bias"], -STEER_CLIP_RAD, STEER_CLIP_RAD))
            - wheelbase * wz_all[mask_ok] / vx_all[mask_ok]
        )
        ax_k.scatter(x, y, s=4, alpha=0.3)
        if len(x):
            xs = np.linspace(x.min(), x.max(), 50)
            ax_k.plot(xs, params["k_us"] * xs, color="red", label=f"k_us={params['k_us']:.5f}")
        ax_k.set_xlabel("v * wz")
        ax_k.set_ylabel("tan(steer) - L*wz/v")
        ax_k.set_title("k_us 定常旋回回帰")
        ax_k.legend(fontsize=8)
    axes[n_ds, 1].axis("off")

    fig.tight_layout()
    return fig, metrics


def build_text_summary(params: dict, wheelbase: float, metrics: dict, *, n_datasets: int) -> str:
    lines = [
        "=" * 72,
        "簡易物理妥当性レポート (reidentify v2, 機能ダウングレード版)",
        "=" * 72,
        f"サンプルデータセット数: {n_datasets}",
        f"wheelbase = {wheelbase:.5f} m",
        "",
        "-- 同定パラメータ --",
    ]
    for k in sorted(params.keys()):
        lines.append(f"  {k:32s} = {params[k]}")
    lines.append("")
    lines.append("-- 当てはめ品質 (サンプル窓 RMSE) --")
    if metrics["long_rmse"]:
        lines.append(f"  縦方向 accel RMSE (サンプル平均) = {np.nanmean(metrics['long_rmse']):.4f} m/s^2")
    if metrics["steer_rmse"]:
        lines.append(f"  操舵 RMSE (サンプル平均)         = {np.nanmean(metrics['steer_rmse']):.5f} rad")
    lines.append("")
    lines.append("-- 妥当性チェック (閾値は lib._physical_validity 由来の定数) --")
    k_us = params["k_us"]
    ok = 0.0 <= k_us <= K_US_CLIP
    lines.append(f"  k_us={k_us:.5f} in [0, {K_US_CLIP:.2f}]: {'OK' if ok else 'NG'}")
    tau_a = params["acc_time_constant"]
    ok = REPORT_VALID_ACC_TAU_BOUNDS[0] <= tau_a <= REPORT_VALID_ACC_TAU_BOUNDS[1]
    lines.append(
        f"  acc_time_constant={tau_a:.4f} in "
        f"[{REPORT_VALID_ACC_TAU_BOUNDS[0]}, {REPORT_VALID_ACC_TAU_BOUNDS[1]}]: "
        f"{'OK' if ok else 'NG (要確認)'}"
    )
    tau_d = params["steer_time_constant"]
    ok = REPORT_VALID_STEER_TAU_BOUNDS[0] <= tau_d <= REPORT_VALID_STEER_TAU_BOUNDS[1]
    lines.append(
        f"  steer_time_constant={tau_d:.4f} in "
        f"[{REPORT_VALID_STEER_TAU_BOUNDS[0]}, {REPORT_VALID_STEER_TAU_BOUNDS[1]}]: "
        f"{'OK' if ok else 'NG (要確認)'}"
    )
    lines.append("=" * 72)
    return "\n".join(lines)


def build_report(
    collection_dir: Path, tuned_params_path: Path, out_png: Path, out_txt: Path,
    *, scenario: Path | None = None, case: str = "current", n_sample_datasets: int = REPORT_SAMPLE_DATASETS,
) -> None:
    params = load_tuned_params(tuned_params_path)
    wheelbase = resolve_wheelbase(scenario, case)
    datasets = _sample_datasets(collection_dir, n_sample_datasets)
    if not datasets:
        raise RuntimeError("レポート対象の有効な CSV キャッシュがありません。")

    fig, metrics = build_report_figure(datasets, params, wheelbase)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120)
    plt.close(fig)

    text = build_text_summary(params, wheelbase, metrics, n_datasets=len(datasets))
    out_txt.parent.mkdir(parents=True, exist_ok=True)
    out_txt.write_text(text, encoding="utf-8")
    print(text)
    print(f"\n[INFO] 図を保存しました: {out_png}")
    print(f"[INFO] テキストサマリーを保存しました: {out_txt}")


def main() -> None:
    ap = argparse.ArgumentParser(description="簡易物理妥当性レポート (Step4b、機能ダウングレード版)")
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--tuned-params", type=Path, required=True)
    ap.add_argument("--scenario", type=Path, default=None)
    ap.add_argument("--case", default="current")
    ap.add_argument("--out-png", type=Path, required=True)
    ap.add_argument("--out-txt", type=Path, required=True)
    args = ap.parse_args()
    build_report(
        args.collection_dir, args.tuned_params, args.out_png, args.out_txt,
        scenario=args.scenario, case=args.case,
    )


if __name__ == "__main__":
    main()
