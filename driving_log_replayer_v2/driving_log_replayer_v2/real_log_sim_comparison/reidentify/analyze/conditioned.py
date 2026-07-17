"""A1: 特徴量条件付き残差分析。

residual_traces の per-start 署名付き終端誤差を、構造仮説に対応する特徴量で条件付けし、
系統依存の有無を統計的に判定する。判定は dataset をクラスタ単位とする 2 段集計:
bin 内 per-dataset 平均 → dataset 横断 mean + bootstrap 95%CI。per-dataset 回帰係数の
符号一致率 (二項検定、BH 補正) と効果量 (説明 std / 残差 RMS) を併記する。

観測 → v3 構造候補の対応 (実装計画 §A-M1):
  err_ax vs g·sin(pitch_lf) の傾き ≈ 1  → SLOPE_ACCX フィード
  err_ax vs vx の単調依存               → 走行抵抗 c0 + c2·v²
  err_ax の a_cmd>0/<0 での系統差       → ブレーキ/スロットル非対称ゲイン・τ
  err_ax vs jerk 依存                   → acc 2 次系 / レートリミット
  err_steer vs steer レート符号          → ヒステリシス/デッドバンド
  err_steer vs vx 依存                  → 速度依存 steer ゲイン
  err_yaw/err_lat vs ay                 → 速度依存 k_us
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

GRAVITY = 9.80665

# 条件付け対象: (残差列, 評価 horizon)。プラトー (ax/steer N=30) と
# 蓄積誤差 (yaw N=70, lat N=150) を代表点で見る。
TARGETS: tuple[tuple[str, int], ...] = (
    ("err_ax", 30),
    ("err_steer_deg", 30),
    ("err_yaw_deg", 70),
    ("err_lat_cm", 150),
)

# 条件付け特徴量 (traces 列から導出する派生列を含む)。
FEATURES: tuple[str, ...] = (
    "g_sin_pitch_lf",
    "vx_mean",
    "a_cmd_mean",
    "jerk_cmd_abs_mean",
    "steer_rate_mean",
    "ay_mean",
)

N_BINS = 10
BOOTSTRAP_B = 1000
BOOTSTRAP_SEED = 42
# 候補化しきい値: その特徴で説明される std が残差 RMS の 10% 以上。
EFFECT_RATIO_THRESHOLD = 0.10
# 符号一致率の有意基準。
SIGN_AGREEMENT_THRESHOLD = 0.70
BH_ALPHA = 0.05
# per-dataset 回帰の成立最小サンプル数。
MIN_ROWS_PER_DATASET = 30


def add_derived_features(traces: pd.DataFrame) -> pd.DataFrame:
    """traces に派生特徴量列 (g_sin_pitch_lf) を追加した copy を返す。"""
    out = traces.copy()
    out["g_sin_pitch_lf"] = GRAVITY * np.sin(out["pitch_lf_mean"].astype(float))
    return out


@dataclass
class ConditionResult:
    """1 つの (target, feature) 組の判定。"""

    target: str
    horizon: int
    feature: str
    n_datasets: int
    pooled_slope: float
    median_slope: float
    sign_agreement: float
    sign_p: float
    sign_p_bh: float
    effect_std: float
    residual_rms: float
    effect_ratio: float
    significant: bool


def _per_dataset_slopes(df: pd.DataFrame, target: str, feature: str) -> pd.DataFrame:
    """dataset ごとの 1 次回帰 (target ~ feature) の傾きを返す。"""
    rows = []
    for ds_id, sub in df.groupby("dataset_id"):
        x = sub[feature].to_numpy(dtype=float)
        y = sub[target].to_numpy(dtype=float)
        if len(x) < MIN_ROWS_PER_DATASET or float(np.std(x)) == 0.0:
            continue
        slope, intercept = np.polyfit(x, y, 1)
        rows.append({"dataset_id": ds_id, "slope": float(slope), "intercept": float(intercept),
                     "x_std": float(np.std(x)), "n": len(x)})
    return pd.DataFrame(rows)


def _binomial_sign_p(k: int, n: int) -> float:
    """両側二項検定 (p=0.5) の p 値。scipy.stats.binomtest 依存。"""
    from scipy.stats import binomtest

    if n == 0:
        return float("nan")
    return float(binomtest(k, n, 0.5).pvalue)


def _bh_adjust(pvalues: list[float]) -> list[float]:
    """Benjamini–Hochberg 補正済み p 値 (NaN は素通し)。"""
    p = np.asarray(pvalues, dtype=float)
    adjusted = np.full_like(p, np.nan)
    mask = np.isfinite(p)
    pm = p[mask]
    m = len(pm)
    if m == 0:
        return adjusted.tolist()
    order = np.argsort(pm)
    ranked = pm[order] * m / (np.arange(m) + 1)
    # 単調化 (後ろから累積最小)。
    ranked = np.minimum.accumulate(ranked[::-1])[::-1]
    out = np.empty(m)
    out[order] = np.clip(ranked, 0.0, 1.0)
    adjusted[mask] = out
    return adjusted.tolist()


def _bin_table(
    df: pd.DataFrame, target: str, feature: str, *, rng: np.random.Generator,
) -> pd.DataFrame:
    """特徴量 quantile bin ごとの 2 段集計 (per-dataset 平均 → 横断 mean + bootstrap CI)。"""
    x = df[feature].to_numpy(dtype=float)
    edges = np.unique(np.quantile(x, np.linspace(0.0, 1.0, N_BINS + 1)))
    if len(edges) < 3:
        return pd.DataFrame()
    bin_idx = np.clip(np.searchsorted(edges, x, side="right") - 1, 0, len(edges) - 2)

    work = pd.DataFrame({
        "dataset_id": df["dataset_id"].to_numpy(),
        "bin": bin_idx,
        "y": df[target].to_numpy(dtype=float),
        "x": x,
    })
    per_ds = work.groupby(["bin", "dataset_id"], sort=True).agg(
        y_mean=("y", "mean"), x_mean=("x", "mean"), n=("y", "size"),
    ).reset_index()

    rows = []
    for b, sub in per_ds.groupby("bin"):
        values = sub["y_mean"].to_numpy(dtype=float)
        n_ds = len(values)
        if n_ds == 0:
            continue
        boot = rng.choice(values, size=(BOOTSTRAP_B, n_ds), replace=True).mean(axis=1)
        rows.append({
            "bin": int(b),
            "x_center": float(sub["x_mean"].mean()),
            "n_datasets": n_ds,
            "n_rows": int(sub["n"].sum()),
            "mean": float(values.mean()),
            "ci_lo": float(np.quantile(boot, 0.025)),
            "ci_hi": float(np.quantile(boot, 0.975)),
        })
    return pd.DataFrame(rows)


def analyze_conditioned(
    traces: pd.DataFrame,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """全 (target, feature) 組を判定する。

    返り値: (summary, bins, slopes)
      summary — ConditionResult の表 (BH 補正込み)
      bins    — bin ごとの 2 段集計 (プロット用)
      slopes  — per-dataset 回帰傾き (診断用)
    """
    df_all = add_derived_features(traces)
    rng = np.random.default_rng(BOOTSTRAP_SEED)

    results: list[dict] = []
    bin_frames: list[pd.DataFrame] = []
    slope_frames: list[pd.DataFrame] = []
    for target, horizon in TARGETS:
        df_h = df_all[df_all["horizon"] == horizon]
        if df_h.empty:
            continue
        residual_rms = float(np.sqrt(np.mean(df_h[target].to_numpy(dtype=float) ** 2)))
        for feature in FEATURES:
            slopes = _per_dataset_slopes(df_h, target, feature)
            if slopes.empty:
                continue
            n_ds = len(slopes)
            pooled_x = df_h[feature].to_numpy(dtype=float)
            pooled_slope, _ = np.polyfit(pooled_x, df_h[target].to_numpy(dtype=float), 1)
            n_pos = int((slopes["slope"] > 0).sum())
            sign_agreement = max(n_pos, n_ds - n_pos) / n_ds
            sign_p = _binomial_sign_p(n_pos, n_ds)
            effect_std = abs(float(pooled_slope)) * float(np.std(pooled_x))
            results.append({
                "target": target,
                "horizon": horizon,
                "feature": feature,
                "n_datasets": n_ds,
                "pooled_slope": float(pooled_slope),
                "median_slope": float(slopes["slope"].median()),
                "sign_agreement": float(sign_agreement),
                "sign_p": sign_p,
                "effect_std": effect_std,
                "residual_rms": residual_rms,
                "effect_ratio": effect_std / residual_rms if residual_rms > 0 else float("nan"),
            })
            bins = _bin_table(df_h, target, feature, rng=rng)
            if not bins.empty:
                bins.insert(0, "feature", feature)
                bins.insert(0, "horizon", horizon)
                bins.insert(0, "target", target)
                bin_frames.append(bins)
            slopes.insert(0, "feature", feature)
            slopes.insert(0, "horizon", horizon)
            slopes.insert(0, "target", target)
            slope_frames.append(slopes)

    summary = pd.DataFrame(results)
    if not summary.empty:
        summary["sign_p_bh"] = _bh_adjust(summary["sign_p"].tolist())
        summary["significant"] = (
            (summary["sign_agreement"] >= SIGN_AGREEMENT_THRESHOLD)
            & (summary["sign_p_bh"] < BH_ALPHA)
            & (summary["effect_ratio"] >= EFFECT_RATIO_THRESHOLD)
        )
        summary = summary.sort_values(
            ["significant", "effect_ratio"], ascending=[False, False],
        ).reset_index(drop=True)
    bins_df = pd.concat(bin_frames, ignore_index=True) if bin_frames else pd.DataFrame()
    slopes_df = pd.concat(slope_frames, ignore_index=True) if slope_frames else pd.DataFrame()
    return summary, bins_df, slopes_df


def counterfactual_corrections(traces: pd.DataFrame) -> pd.DataFrame:
    """反実仮想チェック: 候補補正を N-step 残差へ注入したときの RMS 低下を予測する。

    C++ 実装前の go/no-go 判定材料。N-step 署名付き残差への注入なので、
    1-step 直接同定と N-step rollout のミスマッチ (v2_t の失敗モード) を構造的に回避する。
    返り値: correction, target, horizon, rms_before, rms_after, reduction_pct。
    """
    df_all = add_derived_features(traces)
    rows: list[dict] = []

    def _record(name: str, target: str, horizon: int, y: np.ndarray, y_corr: np.ndarray) -> None:
        before = float(np.sqrt(np.mean(y**2)))
        after = float(np.sqrt(np.mean(y_corr**2)))
        rows.append({
            "correction": name,
            "target": target,
            "horizon": horizon,
            "rms_before": before,
            "rms_after": after,
            "reduction_pct": (1.0 - after / before) * 100.0 if before > 0 else float("nan"),
        })

    df30 = df_all[df_all["horizon"] == 30]
    if not df30.empty:
        y = df30["err_ax"].to_numpy(dtype=float)
        g_sin = df30["g_sin_pitch_lf"].to_numpy(dtype=float)
        vx = df30["vx_mean"].to_numpy(dtype=float)
        a_cmd = df30["a_cmd_mean"].to_numpy(dtype=float)

        # slope フィード: β=1 固定 (物理値そのまま給電) と β 自由 (減衰込み最良)。
        _record("slope_feed_beta1", "err_ax", 30, y, y - g_sin)
        beta = float(np.polyfit(g_sin, y, 1)[0])
        _record(f"slope_feed_beta_fit({beta:+.3f})", "err_ax", 30, y, y - beta * g_sin)

        # 走行抵抗: err_ax ~ c0 + c2·vx² (slope 補正後の残差に対して)。
        y_s = y - beta * g_sin
        design = np.column_stack([np.ones_like(vx), vx**2])
        coef, _r, _rank, _sv = np.linalg.lstsq(design, y_s, rcond=None)
        _record(
            f"drag_c0_c2(c0={coef[0]:+.4f},c2={coef[1]:+.2e})", "err_ax", 30,
            y_s, y_s - design @ coef,
        )

        # ブレーキ/スロットル非対称: a_cmd 符号別の 1 次補正 (slope 補正後)。
        y_b = y_s.copy()
        for mask in (a_cmd < -0.05, a_cmd > 0.05):
            if int(mask.sum()) > 100:
                s, c = np.polyfit(a_cmd[mask], y_s[mask], 1)
                y_b[mask] = y_s[mask] - (s * a_cmd[mask] + c)
        _record("brake_throttle_asym", "err_ax", 30, y_s, y_b)

        # steer: 速度依存ゲイン相当の 1 次補正。
        ys = df30["err_steer_deg"].to_numpy(dtype=float)
        s, c = np.polyfit(vx, ys, 1)
        _record(f"steer_vx_linear(s={s:+.4f})", "err_steer_deg", 30, ys, ys - (s * vx + c))

    df70 = df_all[df_all["horizon"] == 70]
    if not df70.empty:
        # 速度依存 k_us 相当: err_yaw ~ ay の 1 次補正。
        yy = df70["err_yaw_deg"].to_numpy(dtype=float)
        ay = df70["ay_mean"].to_numpy(dtype=float)
        s, c = np.polyfit(ay, yy, 1)
        _record(f"yaw_ay_linear(s={s:+.4f})", "err_yaw_deg", 70, yy, yy - (s * ay + c))

    return pd.DataFrame(rows)
