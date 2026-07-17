"""縦方向レジーム分割評価 (brake / coast / throttle 別の残差指標)。

減速領域はそもそもダイナミクスが違う可能性がある (定常ゲイン: スロットル ≈1.0 /
ブレーキ ≈0.7、per-dataset 最適 scaling が brake_frac と負相関) という仮説に対し、
第一歩として評価だけをレジーム別に分ける。residual traces の窓平均指令 a_cmd_mean で
各 (k0, horizon) サンプルを 3 レジームへ分類し、レジーム別の RMSE / 署名付き平均を出す。

集計は dataset をクラスタ単位とする 2 段集計 (per-dataset 値 → 横断 mean + bootstrap CI)。
レジーム間で ax の RMSE / バイアスが系統的に違えば「モデル分割 (符号別 τ・ゲイン)」の
直接の根拠になり、v3 採用ゲートにもレジーム別セルを足す判断材料になる。
"""
from __future__ import annotations

import numpy as np
import pandas as pd

# レジーム境界 (窓平均指令 [m/s^2])。oracle.BRAKE_A_CMD / ACCEL_A_CMD と同値。
BRAKE_A_CMD = -0.3
ACCEL_A_CMD = 0.3

REGIMES: tuple[str, ...] = ("brake", "coast", "throttle")
# レジーム別に見る残差列とその horizon (プラトー N=30 + 過渡 N=10)。
TARGET_COLS: tuple[str, ...] = ("err_ax", "err_vx", "err_long_cm")
REGIME_HORIZONS: tuple[int, ...] = (10, 30)

BOOTSTRAP_B = 1000
BOOTSTRAP_SEED = 42
MIN_ROWS_PER_DATASET = 10


def classify_regime(a_cmd_mean: np.ndarray) -> np.ndarray:
    """窓平均指令からレジームラベル配列を返す。"""
    a = np.asarray(a_cmd_mean, dtype=float)
    out = np.full(a.shape, "coast", dtype=object)
    out[a < BRAKE_A_CMD] = "brake"
    out[a > ACCEL_A_CMD] = "throttle"
    return out


def analyze_regimes(traces: pd.DataFrame) -> pd.DataFrame:
    """レジーム × horizon × 残差列の 2 段集計表を返す。

    列: regime, horizon, target, n_datasets, n_rows, rms, mean, mean_ci_lo, mean_ci_hi。
    rms は per-dataset RMS の横断平均 (dataset 等重み)、mean は署名付き per-dataset
    平均の横断平均 + bootstrap 95%CI (系統バイアスの検出用)。
    """
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    df = traces[traces["horizon"].isin(REGIME_HORIZONS)].copy()
    df["regime"] = classify_regime(df["a_cmd_mean"].to_numpy())

    rows: list[dict] = []
    for (regime, horizon), sub in df.groupby(["regime", "horizon"]):
        for target in TARGET_COLS:
            per_ds = sub.groupby("dataset_id")[target].agg(
                mean="mean", n="size", sq=lambda s: float(np.mean(np.asarray(s, dtype=float) ** 2)),
            )
            per_ds = per_ds[per_ds["n"] >= MIN_ROWS_PER_DATASET]
            if per_ds.empty:
                continue
            means = per_ds["mean"].to_numpy(dtype=float)
            boot = rng.choice(means, size=(BOOTSTRAP_B, len(means)), replace=True).mean(axis=1)
            rows.append({
                "regime": regime,
                "horizon": int(horizon),
                "target": target,
                "n_datasets": int(len(per_ds)),
                "n_rows": int(per_ds["n"].sum()),
                "rms": float(np.sqrt(per_ds["sq"].mean())),
                "mean": float(means.mean()),
                "mean_ci_lo": float(np.quantile(boot, 0.025)),
                "mean_ci_hi": float(np.quantile(boot, 0.975)),
            })
    out = pd.DataFrame(rows)
    if not out.empty:
        out["regime"] = pd.Categorical(out["regime"], categories=list(REGIMES), ordered=True)
        out = out.sort_values(["target", "horizon", "regime"]).reset_index(drop=True)
    return out
