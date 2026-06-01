#!/usr/bin/env python3
"""Stage 7: k_us (アンダーステア係数) を複数値でスイープし、multi-step rollout 誤差で同定する.

6 段階パイプラインの後段に位置する独立ステージ。Stage 1 が生成した実機 lite
(`lite/real.lite/`) を SSOT とし、k_us グリッドについて Stage 5 の free-running rollout
(`step5.run_free_rollout`) を回して、実車の実効アンダーステア係数を同定する。

なぜ rollout で同定するか:
  - per-step delta は各ステップで実機状態にリセットするため k_us に非感度。
  - per-step の err_wz は steer を k_us=0 の運動学逆算で seed するため、k_us が異なる
    ケース間で比較できない (seeding バイアス)。
  - free-running rollout は seed が step0 のみで、N ステップ後は真の dynamics が支配する
    ため、最大 horizon の yaw RMSE を最小化する k_us が実効 understeer の推定になる。

real.lite を 1 回だけ読み込み、k_us グリッドの rollout を回して horizon 別 yaw / 位置 RMSE を
集計し、最大 horizon の yaw RMSE を最小化する k_us を identified value として報告する
(近傍 3 点の放物線フィットでサブグリッド推定も行う)。

evaluator_node が Stage 6 の後に env (BEST_MODEL_BASE_DIR) のみで実行する (追加設定不要)。
手動実行・グリッド変更も可能:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.step7_identify_kus \
        --base-dir <out_dir> \
        [--kus-values 0,0.005,0.01,0.015,0.02,0.025,0.03,0.04,0.05] \
        [--horizons 2,5,10,20] [--stride 5]

出力: <base-dir>/comparison/kus_sweep/{kus_sweep.csv, kus_sweep.png}
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from . import step5_analyze_per_step as s5
from .lib._params_utils import add_params_annotation, setup_jp_font
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

setup_jp_font()

_MODEL_TYPE = "delay_steer_acc_geared_wo_fall_guard"  # k_us を持つ唯一の対応モデル


def _rms(a: np.ndarray) -> float:
    a = np.asarray(a, dtype=float)
    return float(np.sqrt(np.nanmean(a**2))) if len(a) else float("nan")


def _parabolic_min(xs: list[float], ys: list[float]) -> float | None:
    """argmin 近傍 3 点に二次フィットして頂点 (サブグリッド最小) を返す。端なら None。"""
    i = int(np.argmin(ys))
    if i == 0 or i == len(xs) - 1:
        return None
    x0, x1, x2 = xs[i - 1], xs[i], xs[i + 1]
    y0, y1, y2 = ys[i - 1], ys[i], ys[i + 1]
    denom = (x0 - x1) * (x0 - x2) * (x1 - x2)
    if abs(denom) < 1e-18:
        return None
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom
    b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denom
    if a <= 0:  # 下に凸でなければ頂点は最小でない
        return None
    return -b / (2 * a)


def run_sweep(
    data: dict,
    t0_ns: int,
    base_params: dict,
    kus_values: list[float],
    horizons: tuple[int, ...],
    stride: int,
) -> pd.DataFrame:
    """各 k_us について rollout を回し horizon 別 RMSE を集計した DataFrame を返す。"""
    rows: list[dict] = []
    for kus in kus_values:
        params = dict(base_params)
        params["k_us"] = kus
        # run_free_rollout / _prepare_gt はモジュール global SUB_DT を参照するため同期する
        s5.SUB_DT = params["sub_dt"]
        df_roll = s5.run_free_rollout(
            data, t0_ns, params, _MODEL_TYPE, horizons=horizons, stride=stride
        )
        for horizon in horizons:
            sub = df_roll[df_roll["horizon"] == horizon]
            rows.append({
                "k_us": kus,
                "horizon": int(horizon),
                "yaw_rmse_deg": _rms(sub["yaw_err_deg"].values),
                "pos_rmse_cm": _rms(sub["pos_err"].values) * 100.0,
                "lat_rmse_cm": _rms(sub["err_lat"].values) * 100.0,
                "n": len(sub),
            })
        print(
            f"  k_us={kus:.4f}: "
            + ", ".join(
                f"N{h} yaw={rows[-(len(horizons) - i)]['yaw_rmse_deg']:.3f}"
                for i, h in enumerate(horizons)
            )
        )
    return pd.DataFrame(rows)


def plot_sweep(res: pd.DataFrame, identified: dict, params: dict, out_path: Path) -> None:
    horizons = sorted(res["horizon"].unique())
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    cmap = plt.get_cmap("viridis")
    for idx, horizon in enumerate(horizons):
        sub = res[res["horizon"] == horizon].sort_values("k_us")
        color = cmap(idx / max(1, len(horizons) - 1))
        kus = sub["k_us"].to_numpy()
        ax1.plot(kus, sub["yaw_rmse_deg"].to_numpy(), "o-", color=color, label=f"N={horizon}")
        ax2.plot(kus, sub["pos_rmse_cm"].to_numpy(), "o-", color=color, label=f"N={horizon}")
    kus_id = identified.get("k_us_parabolic") or identified.get("k_us_grid")
    for ax, ylab, title in (
        (ax1, "yaw RMSE [deg]", "yaw rate 累積誤差 vs k_us"),
        (ax2, "位置 RMSE [cm]", "位置 累積誤差 vs k_us"),
    ):
        if kus_id is not None:
            ax.axvline(kus_id, color="red", ls="--", lw=1.2,
                       label=f"identified k_us≈{kus_id:.4f}")
        ax.set_xlabel("k_us [rad/(m/s²)]")
        ax.set_ylabel(ylab)
        ax.set_title(title, fontsize=10)
        ax.grid(True, lw=0.5, alpha=0.6)
        ax.legend(fontsize=8)
    fig.suptitle("k_us スイープ同定 (free-running rollout)", fontsize=12)
    add_params_annotation(fig, params)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="k_us スイープ同定 (rollout)")
    add_common_cli_arguments(parser)
    parser.add_argument(
        "--kus-values",
        default="0,0.005,0.01,0.015,0.02,0.025,0.03,0.04,0.05",
        help="カンマ区切りの k_us グリッド [rad/(m/s^2)]",
    )
    parser.add_argument("--horizons", default="2,5,10,20", help="カンマ区切りの rollout horizon")
    parser.add_argument("--stride", type=int, default=5, help="開始点サンプリング間隔 [step]")
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path.cwd())
    s5.LITE_DIR = cfg.lite_dir

    try:
        real_mcap = s5._resolve_real_mcap(cfg.lite_dir / "real.lite.mcap")
    except FileNotFoundError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Loading: {real_mcap}")
    data = s5.load_real_bag(real_mcap)
    t0_ns = s5.find_autonomous_start(data)

    base_params = s5._build_params(cfg)  # wheelbase=cfg.wheelbase_sim, steer_bias/sub_dt は仕様値
    kus_values = sorted(float(x) for x in args.kus_values.split(",") if x.strip())
    horizons = tuple(int(x) for x in args.horizons.split(",") if x.strip())
    print(
        f"k_us grid={kus_values}\n"
        f"horizons={horizons}, stride={args.stride}, "
        f"wheelbase={base_params['wheelbase']:.5f}, steer_bias={base_params['steer_bias']:.5f}"
    )

    print("\n=== k_us スイープ実行 ===")
    res = run_sweep(data, t0_ns, base_params, kus_values, horizons, args.stride)

    # 同定: 最大 horizon の yaw RMSE を最小化する k_us (グリッド + 放物線サブグリッド)
    h_max = max(horizons)
    at = res[res["horizon"] == h_max].sort_values("k_us").reset_index(drop=True)
    xs = at["k_us"].tolist()
    ys = at["yaw_rmse_deg"].tolist()
    grid_best_i = int(np.argmin(ys))
    identified = {
        "horizon": h_max,
        "k_us_grid": float(xs[grid_best_i]),
        "yaw_rmse_grid": float(ys[grid_best_i]),
        "k_us_parabolic": _parabolic_min(xs, ys),
    }

    out_dir = cfg.out_dir / "kus_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    res.to_csv(out_dir / "kus_sweep.csv", index=False)
    print(f"  Saved: {out_dir / 'kus_sweep.csv'}")
    plot_sweep(res, identified, base_params, out_dir / "kus_sweep.png")

    print("\n=== 同定結果 (yaw RMSE 最小化 @ horizon N={}) ===".format(h_max))
    print(f"  グリッド最小: k_us = {identified['k_us_grid']:.4f}  "
          f"(yaw RMSE = {identified['yaw_rmse_grid']:.4f} deg)")
    if identified["k_us_parabolic"] is not None:
        print(f"  放物線サブグリッド推定: k_us ≈ {identified['k_us_parabolic']:.4f}")
    else:
        print("  放物線サブグリッド推定: 最小がグリッド端のため範囲外 "
              "(--kus-values を広げて再試行)")
    # 端での単調性を警告
    if grid_best_i in (0, len(xs) - 1):
        print(f"  [WARN] 最小がグリッド端 (k_us={xs[grid_best_i]:.4f})。"
              "真の最小はこの外側の可能性。範囲を広げて再実行を推奨。")


if __name__ == "__main__":
    main()
