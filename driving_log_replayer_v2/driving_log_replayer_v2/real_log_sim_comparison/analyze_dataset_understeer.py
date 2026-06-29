#!/usr/bin/env python3
"""
データセット分析レポート生成スクリプト

アンダーステア係数 (k_us) の速度依存性と、チューニング結果の説明材料となるグラフを生成。
出力: /home/kotaroyoshimoto/data/openloop_j6_15/dataset_analysis_report/report.html
"""

import sys
import base64
import io
import math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

# リポジトリパスを sys.path に追加
_REPO = Path("/home/kotaroyoshimoto/workspace/x2_e2e_curve/src/simulator")
_REPO /= "driving_log_replayer_v2/driving_log_replayer_v2/driving_log_replayer_v2"
sys.path.insert(0, str(_REPO / "real_log_sim_comparison"))
sys.path.insert(0, str(_REPO))

from lib._io import load_kinematic, load_steering  # noqa: E402

# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------
DATASETS_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june/datasets")
OUTPUT_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15/dataset_analysis_report")
STEER_DEADBAND_CSV = Path("/home/kotaroyoshimoto/data/openloop_j6_15/steer_deadband_report.csv")

WHEELBASE = 4.76012   # [m]
STEER_BIAS = 0.0005   # [rad] デフォルト (lib/_params_utils.py)
DEAD_BAND_OPT = 0.002438735319718077  # [rad] 最適化済み steer_dead_band

# チューニング済みパラメータ
VX_LO_TUNED = 3.7303
VX_HI_TUNED = 8.9344
K_US_TUNED = 0.00434

# フィルタ閾値
VX_MIN_CURVE = 1.5    # [m/s] 曲線部判定最低速度
WZ_MIN = 0.02         # [rad/s] 最低ヨーレート
DWZ_MAX = 0.30        # [rad/s^2] 定常状態判定 (角加速度上限)
K_US_CLIP = 0.5       # 外れ値除去

# Phase 42 最適化パラメータ (k_us ランプなし / k_end GT yaw)
K_US_PHASE42 = 0.003499  # [rad·s²/m]


# ---------------------------------------------------------------------------
# ワーカー: 1データセット読み込み
# ---------------------------------------------------------------------------
def _load_one(uuid_path: Path) -> dict | None:
    mcap = uuid_path / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None
    try:
        df_kin = load_kinematic(mcap)
        df_steer = load_steering(mcap)
    except Exception:
        return None
    if df_kin.empty or df_steer.empty:
        return None

    # t [s] へ変換
    t_kin = df_kin["t_ns"].values * 1e-9
    t_steer = df_steer["t_ns"].values * 1e-9

    if len(t_kin) < 10 or len(t_steer) < 5:
        return None

    vx = df_kin["vx"].values
    wz = df_kin["wz"].values

    # steer を kinematic タイムスタンプへ補間
    steer_raw = df_steer["steer"].values
    steer = np.interp(t_kin, t_steer, steer_raw)

    # バイアス除去
    steer_eff = steer - STEER_BIAS

    # 横加速度 (centripetal approx)
    ay = vx * wz

    # 角加速度 (定常状態フィルタ用)
    dt = np.diff(t_kin)
    dt_safe = np.where(dt > 0, dt, 1e-3)
    dwz_mid = np.diff(wz) / dt_safe
    # edges で 0 pad
    dwz = np.empty_like(wz)
    dwz[0] = dwz_mid[0] if len(dwz_mid) > 0 else 0.0
    dwz[-1] = dwz_mid[-1] if len(dwz_mid) > 0 else 0.0
    dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])

    # 移動フィルタ
    mask_moving = vx > 0.5
    n_moving = mask_moving.sum()
    if n_moving == 0:
        return None

    # データセットレベル統計
    vx_m = vx[mask_moving]
    steer_m = steer[mask_moving]
    dur = float(t_kin[-1] - t_kin[0])

    return {
        "uuid": uuid_path.name,
        "vx": vx,
        "wz": wz,
        "steer_eff": steer_eff,
        "ay": ay,
        "dwz": dwz,
        "t": t_kin,
        "vx_mean": float(np.mean(vx_m)),
        "vx_max": float(np.max(vx_m)),
        "vx_p90": float(np.percentile(vx_m, 90)),
        "steer_abs_max": float(np.max(np.abs(steer_m))),
        "steer_abs_mean": float(np.mean(np.abs(steer_m))),
        "duration": dur,
        "n_samples": len(vx),
    }


# ---------------------------------------------------------------------------
# k_us 速度ビン別回帰
# ---------------------------------------------------------------------------
def compute_kus_bins(
    all_vx: np.ndarray,
    all_wz: np.ndarray,
    all_steer_eff: np.ndarray,
    all_dwz: np.ndarray,
    vx_edges: np.ndarray,
) -> dict:
    """速度ビン別 OLS 回帰で k_us(v) を推定。

    モデル: tan(steer) = (L/vx + k_us * vx) * wz
    各ビン内で OLS: tan(steer) = C * wz (原点回帰)
    => k_us = (C - L/vx_mid) / vx_mid
    """
    mask_ok = (
        (np.abs(all_wz) > WZ_MIN) &
        (np.abs(all_dwz) < DWZ_MAX) &
        (all_vx > VX_MIN_CURVE)
    )
    vx_f = all_vx[mask_ok]
    wz_f = all_wz[mask_ok]
    steer_f = all_steer_eff[mask_ok]

    tan_steer = np.tan(np.clip(steer_f, -0.8, 0.8))

    n_bins = len(vx_edges) - 1
    vx_mid = np.empty(n_bins)
    kus_ols = np.full(n_bins, np.nan)
    kus_p10 = np.full(n_bins, np.nan)
    kus_p90 = np.full(n_bins, np.nan)
    kus_p25 = np.full(n_bins, np.nan)
    kus_p75 = np.full(n_bins, np.nan)
    n_pts = np.zeros(n_bins, dtype=int)

    for i in range(n_bins):
        lo, hi = vx_edges[i], vx_edges[i + 1]
        mask_bin = (vx_f >= lo) & (vx_f < hi)
        n = mask_bin.sum()
        n_pts[i] = n
        if n < 10:
            vx_mid[i] = (lo + hi) / 2
            continue
        vx_b = vx_f[mask_bin]
        wz_b = wz_f[mask_bin]
        ts_b = tan_steer[mask_bin]

        vm = float(np.median(vx_b))
        vx_mid[i] = vm

        # OLS 原点回帰: tan(steer) = C * wz
        C_ols = np.sum(wz_b * ts_b) / np.sum(wz_b ** 2)
        kus_ols[i] = (C_ols - WHEELBASE / vm) / vm

        # 個別サンプル k_us (外れ値確認用: percentile)
        with np.errstate(divide="ignore", invalid="ignore"):
            kus_each = (ts_b / np.where(np.abs(wz_b) > 1e-6, wz_b, np.nan) - WHEELBASE / vx_b) / vx_b
        kus_each = kus_each[np.isfinite(kus_each)]
        kus_each = np.clip(kus_each, -K_US_CLIP, K_US_CLIP)
        if len(kus_each) > 10:
            kus_p10[i] = float(np.percentile(kus_each, 10))
            kus_p25[i] = float(np.percentile(kus_each, 25))
            kus_p75[i] = float(np.percentile(kus_each, 75))
            kus_p90[i] = float(np.percentile(kus_each, 90))

    return {
        "vx_mid": vx_mid,
        "kus_ols": kus_ols,
        "kus_p10": kus_p10,
        "kus_p25": kus_p25,
        "kus_p75": kus_p75,
        "kus_p90": kus_p90,
        "n_pts": n_pts,
    }


# ---------------------------------------------------------------------------
# 図: k_us 物理モデル直接可視化 (X=ω·vx, Y=tan(δ)−L·ω/vx, 傾き=k_us)
# ---------------------------------------------------------------------------
_SPEED_BINS_SCATTER = [
    (0.0,  3.0,  "0–3 m/s",   "#4e79a7"),
    (3.0,  5.0,  "3–5 m/s",   "#f28e2b"),
    (5.0,  7.0,  "5–7 m/s",   "#e15759"),
    (7.0,  10.0, "7–10 m/s",  "#76b7b2"),
    (10.0, 30.0, "10+ m/s",   "#59a14f"),
]


def plot_kus_scatter_by_speed(
    all_vx: np.ndarray,
    all_wz: np.ndarray,
    all_steer_eff: np.ndarray,
    all_dwz: np.ndarray,
) -> tuple[plt.Figure, plt.Figure]:
    """両辺に vx をかけた形で k_us を速度帯別にプロット。

    元の式: tan(δ) − L·ω/vx = k_us·(ω·vx) は低速で L·ω/vx の分母発散でノイズが増幅される。
    → 両辺に vx をかけて分母を消去:
         Y' = tan(δ)·vx − L·ω  =  k_us · (ω·vx²)  = k_us · X'

    低速域では X' = ω·vx² が小さく原点付近に集中 → 傾き推定への影響が自然に抑制される。

    Returns:
        fig_main : 全データ重ね + 速度帯別回帰直線 + k_us 棒グラフ
        fig_facet: 速度帯別ファセット (2×3 パネル)
    """
    MAX_PER_BIN = 5000

    mask_ok = (
        (np.abs(all_wz) > WZ_MIN) &
        (np.abs(all_dwz) < DWZ_MAX) &
        (all_vx > VX_MIN_CURVE)
    )
    vx_f    = all_vx[mask_ok]
    wz_f    = all_wz[mask_ok]
    steer_f = all_steer_eff[mask_ok]

    tan_steer_f = np.tan(np.clip(steer_f, -0.8, 0.8))
    # 両辺に vx をかけた形: 分母 vx が消えてノイズ増幅なし
    X_all = wz_f * vx_f ** 2                               # ω·vx² [rad·m/s]
    Y_all = tan_steer_f * vx_f - WHEELBASE * wz_f          # tan(δ)·vx − L·ω [m/s]

    valid_xy = np.isfinite(X_all) & np.isfinite(Y_all) & (np.abs(Y_all) < 2.0)

    rng = np.random.default_rng(42)
    bin_data = []
    for vlo, vhi, label, color in _SPEED_BINS_SCATTER:
        m = valid_xy & (vx_f >= vlo) & (vx_f < vhi)
        n = int(m.sum())
        if n < 10:
            bin_data.append({"label": label, "color": color, "n": n,
                             "kus": np.nan, "X": np.array([]), "Y": np.array([])})
            continue
        X_b = X_all[m]
        Y_b = Y_all[m]
        kus_est = float(np.sum(X_b * Y_b) / np.sum(X_b ** 2))
        n_sub = min(n, MAX_PER_BIN)
        idx = rng.choice(n, n_sub, replace=False)
        bin_data.append({
            "label": label, "color": color, "n": n, "kus": kus_est,
            "X": X_b[idx], "Y": Y_b[idx],
        })

    # X, Y の適切な表示範囲を分位数から自動決定
    all_X_valid = X_all[valid_xy]
    all_Y_valid = Y_all[valid_xy]
    x_lim = float(np.percentile(np.abs(all_X_valid), 98)) if len(all_X_valid) > 0 else 10.0
    y_lim = float(np.percentile(np.abs(all_Y_valid), 98)) if len(all_Y_valid) > 0 else 0.5
    x_lim = max(x_lim, 1.0)
    y_lim = max(y_lim, 0.1)

    # ---- Figure 1: 全データ重ね + k_us 棒グラフ ----
    fig1 = plt.figure(figsize=(17, 7))
    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1], figure=fig1, wspace=0.30)
    ax_sc = fig1.add_subplot(gs[0])
    ax_bar = fig1.add_subplot(gs[1])

    x_line = np.linspace(-x_lim * 1.05, x_lim * 1.05, 300)
    for bd in bin_data:
        if bd["n"] < 10:
            continue
        ax_sc.scatter(bd["X"], bd["Y"],
                      s=3, alpha=0.12, color=bd["color"], rasterized=True)
        ax_sc.plot(x_line, bd["kus"] * x_line,
                   "-", color=bd["color"], lw=2.5,
                   label=f"{bd['label']}  k_us={bd['kus']*1e3:.2f}×10⁻³  (n={bd['n']:,})")

    ax_sc.plot(x_line, K_US_PHASE42 * x_line, "k--", lw=2.0,
               label=f"Phase 42 全体最適  k_us={K_US_PHASE42*1e3:.2f}×10⁻³")
    ax_sc.axhline(0, ls=":", color="gray", lw=0.8)
    ax_sc.axvline(0, ls=":", color="gray", lw=0.8)
    ax_sc.set_xlabel("ω·vx²  [rad·m/s]  ← 低速ほど原点付近に集中", fontsize=11)
    ax_sc.set_ylabel("tan(δ)·vx − L·ω  [m/s]", fontsize=11)
    ax_sc.set_title(
        "k_us の速度依存性: 各直線の傾き = k_us\n"
        "tan(δ)·vx − L·ω = k_us · (ω·vx²)  ← 分母発散なし",
        fontsize=12,
    )
    ax_sc.set_xlim(-x_lim, x_lim)
    ax_sc.set_ylim(-y_lim, y_lim)
    ax_sc.legend(fontsize=9, loc="upper left")
    ax_sc.grid(True, alpha=0.3)

    # 棒グラフ: 速度帯別 k_us
    valid_bd = [bd for bd in bin_data if bd["n"] >= 10]
    bar_x = np.arange(len(valid_bd))
    bars = ax_bar.bar(
        bar_x, [bd["kus"] * 1e3 for bd in valid_bd],
        color=[bd["color"] for bd in valid_bd],
        alpha=0.85, edgecolor="black", linewidth=0.8,
    )
    ax_bar.axhline(K_US_PHASE42 * 1e3, ls="--", color="black", lw=1.5,
                   label=f"Phase 42\n({K_US_PHASE42*1e3:.2f}×10⁻³)")
    for bar, bd in zip(bars, valid_bd):
        h = bar.get_height()
        ax_bar.text(bar.get_x() + bar.get_width() / 2, h + 0.02,
                    f"{bd['n']//1000:.0f}k", ha="center", va="bottom", fontsize=7)
    ax_bar.set_xticks(bar_x)
    ax_bar.set_xticklabels([bd["label"] for bd in valid_bd], rotation=40, ha="right", fontsize=8)
    ax_bar.set_ylabel("k_us 推定値  [×10⁻³ rad·s²/m]")
    ax_bar.set_title("速度帯別 k_us")
    ax_bar.legend(fontsize=8)
    ax_bar.grid(True, alpha=0.3, axis="y")
    kus_vals = [bd["kus"] * 1e3 for bd in valid_bd]
    ax_bar.set_ylim(min(0, min(kus_vals) * 1.2) if kus_vals else 0,
                    max(kus_vals) * 1.5 if kus_vals else 10)

    fig1.suptitle(
        "アンダーステア係数 k_us の速度依存性\n"
        "変換形式: tan(δ)·vx − L·ω = k_us·(ω·vx²)  — 分母発散を解消",
        fontsize=13, y=1.01,
    )
    fig1.tight_layout()

    # ---- Figure 2: ファセットプロット ----
    valid_bins = [bd for bd in bin_data if bd["n"] >= 10]
    n_valid = len(valid_bins)
    n_cols = 3
    n_rows = math.ceil(n_valid / n_cols)

    fig2, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 5 * n_rows), squeeze=False)
    axes_flat = axes.flatten()

    for i, bd in enumerate(valid_bins):
        ax = axes_flat[i]
        # 各速度帯ごとの表示範囲
        if len(bd["X"]) > 10:
            bx_lim = float(np.percentile(np.abs(bd["X"]), 98))
            by_lim = float(np.percentile(np.abs(bd["Y"]), 98))
        else:
            bx_lim, by_lim = x_lim, y_lim
        bx_lim = max(bx_lim, 0.1)
        by_lim = max(by_lim, 0.05)
        x_line_b = np.linspace(-bx_lim * 1.1, bx_lim * 1.1, 300)

        ax.scatter(bd["X"], bd["Y"], s=3, alpha=0.15, color=bd["color"], rasterized=True)
        ax.plot(x_line_b, bd["kus"] * x_line_b, "-", color=bd["color"], lw=2.5)
        ax.plot(x_line_b, K_US_PHASE42 * x_line_b, "k--", lw=1.5,
                label=f"Phase 42 ({K_US_PHASE42*1e3:.2f}×10⁻³)")
        ax.axhline(0, ls=":", color="gray", lw=0.8)
        ax.axvline(0, ls=":", color="gray", lw=0.8)
        ax.set_xlabel("ω·vx² [rad·m/s]", fontsize=9)
        ax.set_ylabel("tan(δ)·vx − L·ω [m/s]", fontsize=9)
        ax.set_title(f"{bd['label']}  (n={bd['n']:,})", fontsize=10)
        ax.set_xlim(-bx_lim, bx_lim)
        ax.set_ylim(-by_lim, by_lim)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.text(0.97, 0.97,
                f"k_us = {bd['kus']*1e3:.2f}×10⁻³",
                transform=ax.transAxes, ha="right", va="top", fontsize=11,
                color=bd["color"],
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85, edgecolor=bd["color"]))

    for i in range(n_valid, len(axes_flat)):
        axes_flat[i].set_visible(False)

    fig2.suptitle(
        "速度帯別 k_us 線形回帰（ファセット）\n"
        "高速域ほど傾き（k_us）が大きい → アンダーステアが強まる",
        fontsize=13,
    )
    fig2.tight_layout()

    return fig1, fig2


# ---------------------------------------------------------------------------
# 図: k_us vs 速度
# ---------------------------------------------------------------------------
def plot_kus_vs_speed(bins: dict, vx_edges: np.ndarray) -> plt.Figure:
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("アンダーステア係数 vs 車速", fontsize=14)

    # ---- 左: k_us 推定値 ----
    ax = axes[0]
    vx_mid = bins["vx_mid"]
    valid = np.isfinite(bins["kus_ols"])

    ax.fill_between(
        vx_mid[valid], bins["kus_p10"][valid], bins["kus_p90"][valid],
        alpha=0.15, color="steelblue", label="10–90%ile (per sample)"
    )
    ax.fill_between(
        vx_mid[valid], bins["kus_p25"][valid], bins["kus_p75"][valid],
        alpha=0.25, color="steelblue", label="25–75%ile"
    )
    ax.plot(vx_mid[valid], bins["kus_ols"][valid],
            "o-", color="steelblue", lw=2, ms=5, label="OLS 推定 k_us(v)")

    # チューニング済み k_us (一定値)
    ax.axhline(K_US_TUNED, ls="--", color="darkorange", lw=1.5,
               label=f"チューニング k_us={K_US_TUNED:.4f}")
    ax.axhline(0.0, ls=":", color="gray", lw=1.0, label="k_us = 0 (純運動学)")

    # vx_lo / vx_hi の縦線
    ax.axvline(VX_LO_TUNED, ls="--", color="green", lw=1.2, alpha=0.8,
               label=f"vx_lo={VX_LO_TUNED:.1f} m/s (tuned)")
    ax.axvline(VX_HI_TUNED, ls="--", color="purple", lw=1.2, alpha=0.8,
               label=f"vx_hi={VX_HI_TUNED:.1f} m/s (tuned)")

    ax.set_xlabel("車速 vx [m/s]")
    ax.set_ylabel("k_us 推定値 [rad·s²/m]")
    ax.set_ylim(-0.05, 0.10)
    ax.set_xlim(0, max(vx_edges))
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("速度別アンダーステア係数推定 (OLS 回帰)")

    # ---- 右: サンプル数 ----
    ax2 = axes[1]
    ax2.bar(vx_mid, bins["n_pts"], width=np.diff(vx_edges), align="center",
            color="steelblue", alpha=0.6, edgecolor="white")
    ax2.axvline(VX_LO_TUNED, ls="--", color="green", lw=1.2, alpha=0.8,
                label=f"vx_lo={VX_LO_TUNED:.1f}")
    ax2.axvline(VX_HI_TUNED, ls="--", color="purple", lw=1.2, alpha=0.8,
                label=f"vx_hi={VX_HI_TUNED:.1f}")
    ax2.set_xlabel("車速 vx [m/s]")
    ax2.set_ylabel("曲線走行サンプル数")
    ax2.set_title("速度帯別サンプル分布")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 図: データセット集団分布
# ---------------------------------------------------------------------------
def plot_dataset_population(df_meta: pd.DataFrame) -> plt.Figure:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("データセット母集団の分布 (N={})".format(len(df_meta)),
                 fontsize=13)

    # vx_mean ヒストグラム
    ax = axes[0, 0]
    ax.hist(df_meta["vx_mean"], bins=30, color="steelblue", edgecolor="white", alpha=0.8)
    ax.axvline(df_meta["vx_mean"].mean(), ls="--", color="red", lw=1.5,
               label=f"平均 {df_meta['vx_mean'].mean():.1f} m/s")
    ax.set_xlabel("平均車速 [m/s]")
    ax.set_ylabel("データセット数")
    ax.set_title("データセット平均速度分布")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")

    # steer_abs_max ヒストグラム
    ax = axes[0, 1]
    steer_deg = np.degrees(df_meta["steer_abs_max"])
    ax.hist(steer_deg, bins=30, color="darkorange", edgecolor="white", alpha=0.8)
    ax.set_xlabel("最大ステア角 [deg]")
    ax.set_ylabel("データセット数")
    ax.set_title("データセット最大ステア角分布")
    ax.grid(True, alpha=0.3, axis="y")

    # vx_mean vs steer_abs_max スキャタープロット
    ax = axes[1, 0]
    sc = ax.scatter(
        df_meta["vx_mean"],
        np.degrees(df_meta["steer_abs_max"]),
        c=df_meta["duration"], cmap="viridis", s=20, alpha=0.6
    )
    plt.colorbar(sc, ax=ax, label="走行時間 [s]")
    ax.set_xlabel("平均車速 [m/s]")
    ax.set_ylabel("最大ステア角 [deg]")
    ax.set_title("平均速度 vs 最大ステア (色: 走行時間)")
    ax.grid(True, alpha=0.3)

    # duration ヒストグラム
    ax = axes[1, 1]
    ax.hist(df_meta["duration"], bins=30, color="green", edgecolor="white", alpha=0.8)
    ax.set_xlabel("走行時間 [s]")
    ax.set_ylabel("データセット数")
    ax.set_title("データセット走行時間分布")
    ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 図: 横加速度 vs 速度
# ---------------------------------------------------------------------------
def plot_lateral_accel(
    all_vx: np.ndarray,
    all_ay: np.ndarray,
    all_wz: np.ndarray,
) -> plt.Figure:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("横加速度・ヨーレート vs 車速", fontsize=13)

    # サブサンプル (全データだと散布図が重すぎる)
    rng = np.random.default_rng(42)
    n_sub = min(80_000, len(all_vx))
    idx = rng.choice(len(all_vx), n_sub, replace=False)
    vx_s = all_vx[idx]
    ay_s = all_ay[idx]
    wz_s = all_wz[idx]

    # 横加速度 vs vx
    ax = axes[0]
    ax.scatter(vx_s, np.abs(ay_s), s=2, alpha=0.15, color="steelblue", rasterized=True)

    # ビン別 95%ile
    edges = np.arange(0, 20.5, 1.0)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (all_vx >= lo) & (all_vx < hi)
        if m.sum() > 50:
            p95 = np.percentile(np.abs(all_ay[m]), 95)
            ax.plot((lo + hi) / 2, p95, "ro", ms=4)

    ax.set_xlabel("車速 vx [m/s]")
    ax.set_ylabel("|横加速度| [m/s²]")
    ax.set_title("横加速度 vs 車速 (点: 全サンプル、赤: 95%ile)")
    ax.set_ylim(0, 5)
    ax.grid(True, alpha=0.3)

    # |ヨーレート| vs vx
    ax = axes[1]
    ax.scatter(vx_s, np.abs(wz_s), s=2, alpha=0.15, color="darkorange", rasterized=True)

    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (all_vx >= lo) & (all_vx < hi)
        if m.sum() > 50:
            p95 = np.percentile(np.abs(all_wz[m]), 95)
            ax.plot((lo + hi) / 2, p95, "bo", ms=4)

    ax.set_xlabel("車速 vx [m/s]")
    ax.set_ylabel("|ヨーレート| [rad/s]")
    ax.set_title("|ヨーレート| vs 車速 (点: 全サンプル、青: 95%ile)")
    ax.set_ylim(0, 0.5)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 図: ステア角 vs 車速
# ---------------------------------------------------------------------------
def plot_steer_vs_speed(all_vx: np.ndarray, all_steer_eff: np.ndarray) -> plt.Figure:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("ステア角 vs 車速", fontsize=13)

    rng = np.random.default_rng(0)
    n_sub = min(80_000, len(all_vx))
    idx = rng.choice(len(all_vx), n_sub, replace=False)

    ax = axes[0]
    ax.scatter(all_vx[idx], np.degrees(np.abs(all_steer_eff[idx])),
               s=2, alpha=0.15, color="purple", rasterized=True)

    edges = np.arange(0, 20.5, 1.0)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (all_vx >= lo) & (all_vx < hi)
        if m.sum() > 50:
            p50 = np.degrees(np.percentile(np.abs(all_steer_eff[m]), 50))
            p95 = np.degrees(np.percentile(np.abs(all_steer_eff[m]), 95))
            ax.plot((lo + hi) / 2, p50, "gs", ms=5, zorder=5)
            ax.plot((lo + hi) / 2, p95, "r^", ms=5, zorder=5)

    ax.set_xlabel("車速 vx [m/s]")
    ax.set_ylabel("|ステア角| [deg]")
    ax.set_title("|ステア角| vs 車速 (緑□: 50%ile, 赤△: 95%ile)")
    ax.grid(True, alpha=0.3)

    # 2D ヒートマップ
    ax2 = axes[1]
    vx_edges = np.arange(0, 20.5, 0.5)
    st_edges = np.arange(0, 20.5, 0.5)  # [deg]
    H, xe, ye = np.histogram2d(
        all_vx,
        np.degrees(np.abs(all_steer_eff)),
        bins=[vx_edges, st_edges],
    )
    pcm = ax2.pcolormesh(xe, ye, H.T, cmap="hot_r")
    plt.colorbar(pcm, ax=ax2, label="サンプル数")
    ax2.set_xlabel("車速 vx [m/s]")
    ax2.set_ylabel("|ステア角| [deg]")
    ax2.set_title("速度 × ステア角 2D ヒストグラム")

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 図: k_us_eff (速度ランプ適用後の有効 k_us)
# ---------------------------------------------------------------------------
def plot_kus_eff(bins: dict, vx_edges: np.ndarray) -> plt.Figure:
    """実際にモデルで適用される k_us_eff = k_us * clamp((v-vx_lo)/(vx_hi-vx_lo), 0, 1) を可視化"""
    vx_plot = np.linspace(0, 20, 200)
    ramp = np.clip((vx_plot - VX_LO_TUNED) / (VX_HI_TUNED - VX_LO_TUNED), 0, 1)
    kus_eff = K_US_TUNED * ramp

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(vx_plot, kus_eff * 1e3, "b-", lw=2.5,
            label=f"モデル k_us_eff\n(k_us={K_US_TUNED:.4f}, vx_lo={VX_LO_TUNED:.1f}, vx_hi={VX_HI_TUNED:.1f})")

    # データから推定した k_us ビン平均
    valid = np.isfinite(bins["kus_ols"]) & (bins["n_pts"] >= 30)
    if valid.sum() > 0:
        ax.plot(bins["vx_mid"][valid], bins["kus_ols"][valid] * 1e3,
                "rs-", ms=6, lw=1.5, label="データ推定 k_us (OLS)")

    ax.axhline(0, ls=":", color="gray", lw=1)
    ax.axvline(VX_LO_TUNED, ls="--", color="green", lw=1.2, alpha=0.7,
               label=f"vx_lo = {VX_LO_TUNED:.1f} m/s")
    ax.axvline(VX_HI_TUNED, ls="--", color="purple", lw=1.2, alpha=0.7,
               label=f"vx_hi = {VX_HI_TUNED:.1f} m/s")

    ax.set_xlabel("車速 vx [m/s]")
    ax.set_ylabel("k_us_eff [x1e-3 rad*s^2/m]")
    ax.set_title("有効アンダーステア係数 k_us_eff(v): モデル vs データ推定")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 20)
    ax.set_ylim(-1, 6)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 図: steer 制御不感帯分析
# ---------------------------------------------------------------------------
def plot_steer_deadband(df_db: pd.DataFrame) -> plt.Figure:
    """steer 制御不感帯分析図 (per-DS |steer − steer_des| 統計)。"""
    db = DEAD_BAND_OPT
    db_deg = np.degrees(db)
    low  = df_db[df_db["vx_mean"] < 2.5]
    high = df_db[df_db["vx_mean"] >= 2.5]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(
        f"steer 制御不感帯分析: |steer − steer_des| per-DS 統計\n"
        f"dead_band_opt={db:.5f} rad ({db_deg:.3f}°)  N={len(df_db)} DS",
        fontsize=13,
    )

    # 左: steer_diff_p50 vs vx_mean 散布図 (色 = frac_below_db)
    ax = axes[0]
    sc = ax.scatter(
        df_db["vx_mean"], df_db["steer_diff_p50"],
        c=df_db["frac_below_db"], cmap="RdYlGn", vmin=0, vmax=1,
        s=12, alpha=0.6,
    )
    plt.colorbar(sc, ax=ax, label="frac_below_db (dead_band 以下の割合)")
    ax.axhline(db, ls="--", color="red", lw=1.5, label=f"dead_band={db:.5f} rad")
    ax.axhline(df_db["steer_diff_p50"].median(), ls=":", color="black", lw=1.2,
               label=f"全体中央値={df_db['steer_diff_p50'].median():.5f} rad")
    ax.set_xlabel("平均速度 vx_mean [m/s]")
    ax.set_ylabel("steer_diff_p50 [rad]")
    ax.set_title("steer 追従誤差の中央値 vs 平均速度\n（色: dead_band 以下の割合）")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 右: 速度層別ヒストグラム
    ax2 = axes[1]
    bins_hist = np.arange(0, 0.051, 0.001)
    ax2.hist(low["steer_diff_p50"],  bins=bins_hist, alpha=0.6, color="#1f77b4",
             label=f"低速 (<2.5 m/s, n={len(low)})")
    ax2.hist(high["steer_diff_p50"], bins=bins_hist, alpha=0.6, color="#ff7f0e",
             label=f"高速 (≥2.5 m/s, n={len(high)})")
    ax2.axvline(db, ls="--", color="red", lw=1.5, label=f"dead_band={db:.5f}")
    ax2.axvline(df_db["steer_diff_p50"].median(), ls=":", color="black", lw=1.2,
                label=f"全体中央値={df_db['steer_diff_p50'].median():.5f}")
    ax2.set_xlabel("steer_diff_p50 [rad]")
    ax2.set_ylabel("DS 数")
    ax2.set_title("速度層別 steer_diff_p50 分布")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 図を base64 PNG に変換
# ---------------------------------------------------------------------------
def fig_to_b64(fig: plt.Figure, dpi: int = 120) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode()


# ---------------------------------------------------------------------------
# HTML レポート生成
# ---------------------------------------------------------------------------
def make_html(sections: list[dict]) -> str:
    """sections: [{title, body_html, img_b64 (optional)}, ...]"""
    imgs_html = ""
    for sec in sections:
        imgs_html += f"<h2>{sec['title']}</h2>\n"
        if sec.get("body"):
            imgs_html += f"<p>{sec['body']}</p>\n"
        if sec.get("img_b64"):
            imgs_html += f'<img src="data:image/png;base64,{sec["img_b64"]}" style="max-width:100%;margin:12px 0;">\n'

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<title>データセット分析レポート - アンダーステア係数解析</title>
<style>
  body {{ font-family: "Noto Sans CJK JP", sans-serif; margin: 2em auto; max-width: 1100px; color: #222; }}
  h1 {{ color: #1a237e; border-bottom: 3px solid #1a237e; padding-bottom: 0.3em; }}
  h2 {{ color: #283593; margin-top: 2em; }}
  p {{ line-height: 1.7; }}
  table {{ border-collapse: collapse; margin: 1em 0; }}
  th, td {{ border: 1px solid #ccc; padding: 6px 12px; text-align: right; }}
  th {{ background: #e8eaf6; }}
</style>
</head>
<body>
<h1>データセット分析レポート: アンダーステア係数解析</h1>
<p>生成日時: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M')}</p>
{imgs_html}
</body>
</html>
"""


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # データセット一覧
    dataset_dirs = sorted(
        d for d in DATASETS_DIR.iterdir()
        if d.is_dir() and (d / "real.lite" / "real.lite_0.mcap").exists()
    )
    print(f"対象データセット数: {len(dataset_dirs)}")

    # 並列読み込み
    results = []
    print("データ読み込み中 (並列 8 プロセス)...")
    with ProcessPoolExecutor(max_workers=8) as ex:
        futs = {ex.submit(_load_one, d): d for d in dataset_dirs}
        done = 0
        for fut in as_completed(futs):
            done += 1
            if done % 50 == 0:
                print(f"  {done}/{len(futs)} 完了")
            r = fut.result()
            if r is not None:
                results.append(r)

    print(f"読み込み成功: {len(results)} / {len(dataset_dirs)}")

    # 全サンプル結合
    all_vx = np.concatenate([r["vx"] for r in results])
    all_wz = np.concatenate([r["wz"] for r in results])
    all_steer = np.concatenate([r["steer_eff"] for r in results])
    all_ay = np.concatenate([r["ay"] for r in results])
    all_dwz = np.concatenate([r["dwz"] for r in results])

    print(f"総サンプル数: {len(all_vx):,}")

    # データセットレベル統計
    df_meta = pd.DataFrame([{
        "uuid": r["uuid"],
        "vx_mean": r["vx_mean"],
        "vx_max": r["vx_max"],
        "vx_p90": r["vx_p90"],
        "steer_abs_max": r["steer_abs_max"],
        "steer_abs_mean": r["steer_abs_mean"],
        "duration": r["duration"],
        "n_samples": r["n_samples"],
    } for r in results])

    # k_us 速度ビン別推定
    vx_edges = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
                          9.0, 10.0, 11.0, 12.0, 14.0, 16.0, 20.0])
    bins = compute_kus_bins(all_vx, all_wz, all_steer, all_dwz, vx_edges)

    # OLS 結果表示
    print("\n速度別 k_us 推定:")
    for i, (vm, ko) in enumerate(zip(bins["vx_mid"], bins["kus_ols"])):
        n = bins["n_pts"][i]
        if n > 0 and np.isfinite(ko):
            print(f"  vx={vm:4.1f} m/s : k_us={ko:.5f}  (n={n})")

    # グラフ生成
    print("\nグラフ生成中...")
    sections = []

    # 1. データセット母集団
    fig = plot_dataset_population(df_meta)
    sections.append({
        "title": "1. データセット母集団の分布",
        "body": (
            f"合計 {len(df_meta)} データセットを分析。"
            f"平均走行速度は {df_meta['vx_mean'].mean():.1f} ± {df_meta['vx_mean'].std():.1f} m/s、"
            f"平均走行時間は {df_meta['duration'].mean():.0f}s。"
            f"低速・大舵角のデータセットが右下象限に集中する場合、ヨー誤差の最悪ケースとなりやすい。"
        ),
        "img_b64": fig_to_b64(fig),
    })

    # 2a. k_us 物理モデル直接可視化 (新メイングラフ)
    print("  k_us 散布図生成中 (2a, 2b)...")
    fig_main, fig_facet = plot_kus_scatter_by_speed(all_vx, all_wz, all_steer, all_dwz)
    sections.append({
        "title": "2a. アンダーステア係数 k_us の速度依存性: 変換形式による直接可視化",
        "body": (
            f"物理モデル <em>wz = vx·tan(δ) / (L + k_us·vx²)</em> を変形した後、両辺に vx をかけて分母発散を除去: "
            f"<em>tan(δ)·vx − L·ω = k_us·(ω·vx²)</em>。"
            f"X軸: ω·vx² [rad·m/s]、Y軸: tan(δ)·vx−L·ω [m/s]、<strong>各直線の傾き = k_us</strong>。"
            f"低速域では X = ω·vx² が小さく原点付近に集中するため、傾き推定への影響が自然に抑制される。"
            f"フィルタ条件: |ω| > {WZ_MIN} rad/s、|dω/dt| < {DWZ_MAX} rad/s²、vx > {VX_MIN_CURVE} m/s。"
            f"黒破線は Phase 42 全体最適値 k_us={K_US_PHASE42:.5f}。"
        ),
        "img_b64": fig_to_b64(fig_main),
    })
    sections.append({
        "title": "2b. 速度帯別ファセット: 各パネルで傾き (k_us) を比較",
        "body": (
            "各パネルは独立した速度帯のデータのみを表示。"
            "パネル右上に推定 k_us 値 (×10⁻³)、黒破線は Phase 42 全体最適値。"
            "低速域 (0–3 m/s) は k_us ≈ 0 に近く、高速域ほど傾きが大きくなることで "
            "タイヤの横力飽和によるアンダーステア増大が視覚的に確認できる。"
        ),
        "img_b64": fig_to_b64(fig_facet),
    })

    # 3. k_us vs 速度 (OLS 折れ線グラフ)
    fig = plot_kus_vs_speed(bins, vx_edges)
    lo_str = f"{VX_LO_TUNED:.1f}"
    hi_str = f"{VX_HI_TUNED:.1f}"
    sections.append({
        "title": "3. 速度別 k_us 推定値 (OLS 折れ線グラフ)",
        "body": (
            f"各速度ビンで <em>tan(δ) = C·ω</em> の OLS 回帰を実施し、"
            f"C から k_us を逆算 (<em>k_us = (C − L/v) / v</em>、L={WHEELBASE} m)。"
            f"チューニング済みランプ区間 vx_lo={lo_str}, vx_hi={hi_str} m/s (緑/紫破線)。"
            f"Section 2a のファセット傾きと対応しており、高速域ほど k_us が増大する傾向を折れ線で確認。"
        ),
        "img_b64": fig_to_b64(fig),
    })

    # 4. モデル適用後 k_us_eff
    fig = plot_kus_eff(bins, vx_edges)
    sections.append({
        "title": "4. モデル適用後の有効アンダーステア係数 k_us_eff(v)",
        "body": (
            f"k_us_eff = k_us × clamp((v − vx_lo)/(vx_hi − vx_lo), 0, 1)。"
            f"低速では k_us_eff = 0 (純運動学モード) へ滑らかに遷移。"
            f"赤点: データ推定値、青線: 現チューニングパラメータ適用後のモデル曲線。"
        ),
        "img_b64": fig_to_b64(fig),
    })

    # 5. 横加速度 vs 速度
    fig = plot_lateral_accel(all_vx, all_ay, all_wz)
    sections.append({
        "title": "5. 横加速度・ヨーレート vs 車速",
        "body": (
            "横加速度 ay ≈ vx·ω (求心加速度近似)。"
            "低速域では |ay| が小さくても大きなヨーレートとなりうる (直角コーナー等)。"
            "高速域では小さなステア角でも大きな横加速度が生じ、k_us の影響が支配的になる。"
        ),
        "img_b64": fig_to_b64(fig),
    })

    # 6. ステア角 vs 速度
    fig = plot_steer_vs_speed(all_vx, all_steer)
    sections.append({
        "title": "6. ステア角 vs 車速",
        "body": (
            "低速域 (< 4 m/s) では大きなステア角が多用される (駐車場・低速コーナー)。"
            "一方高速域では小ステア角で大きなヨーレートを発生させるため、"
            "k_us の影響 (タイヤ横力による大回り傾向) を補正する必要がある。"
        ),
        "img_b64": fig_to_b64(fig),
    })

    # 7. steer 制御不感帯分析 (steer_deadband_report.csv が存在する場合)
    if STEER_DEADBAND_CSV.exists():
        df_db = pd.read_csv(STEER_DEADBAND_CSV)
        fig = plot_steer_deadband(df_db)
        db = DEAD_BAND_OPT
        low_db  = df_db[df_db["vx_mean"] < 2.5]
        high_db = df_db[df_db["vx_mean"] >= 2.5]
        sections.append({
            "title": "7. steer 制御不感帯分析 (|steer − steer_des| 分布)",
            "body": (
                f"最適化パラメータ <em>steer_dead_band={db:.5f} rad ({np.degrees(db):.3f}°)</em> の物理的根拠を実機ログから確認。"
                f"per-DS の |steer − steer_des| 中央値 (steer_diff_p50) の全体中央値は "
                f"{df_db['steer_diff_p50'].median():.5f} rad で dead_band_opt にほぼ一致する。"
                f"高速 DS (≥2.5 m/s, n={len(high_db)}) では steer_diff_p50 < dead_band が多く"
                f"（中央値 {high_db['steer_diff_p50'].median():.5f} rad）、"
                f"低速 DS (&lt;2.5 m/s, n={len(low_db)}) では逆転する"
                f"（中央値 {low_db['steer_diff_p50'].median():.5f} rad）。"
                " EPS 追従コントローラが steer_diff &lt; ~0.002 rad で追従を打ち切る特性を"
                " dead_band が近似しており、系統的な未モデル化現象の補正として機能している。"
            ),
            "img_b64": fig_to_b64(fig),
        })
    else:
        print(f"[スキップ] {STEER_DEADBAND_CSV} が存在しません。先に analyze_steer_deadband.py を実行してください。")

    # 統計表
    stat_html = "<h3>データセット統計サマリー</h3>\n<table>\n"
    stat_html += "<tr><th>指標</th><th>平均</th><th>中央値</th><th>std</th><th>min</th><th>max</th></tr>\n"
    for col, label in [
        ("vx_mean", "平均速度 [m/s]"),
        ("vx_max", "最大速度 [m/s]"),
        ("steer_abs_max", "最大|ステア| [rad]"),
        ("duration", "走行時間 [s]"),
        ("n_samples", "サンプル数"),
    ]:
        vals = df_meta[col]
        stat_html += (
            f"<tr><td>{label}</td>"
            f"<td>{vals.mean():.3f}</td><td>{vals.median():.3f}</td>"
            f"<td>{vals.std():.3f}</td><td>{vals.min():.3f}</td><td>{vals.max():.3f}</td></tr>\n"
        )
    stat_html += "</table>\n"
    sections.append({"title": "8. 統計サマリー", "body": "", "img_b64": None})
    sections[-1]["body"] = stat_html

    # HTML 出力
    html = make_html(sections)
    out_path = OUTPUT_DIR / "report.html"
    out_path.write_text(html, encoding="utf-8")
    print(f"\nレポート生成完了: {out_path}")

    # 個別 PNG も保存
    # (re-generate from bin data since figs were closed)


if __name__ == "__main__":
    main()
