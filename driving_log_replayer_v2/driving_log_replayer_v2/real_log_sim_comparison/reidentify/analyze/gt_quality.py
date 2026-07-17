"""加速度 GT の品質診断 — GT・評価系再設計 (G-1) の証拠収集。

背景 (2026-07-17): 現行 GT (kinematic_savgol, 窓 0.4 s) はブレーキ過渡をなまして
実効ゲインを過小評価する疑いがあり (savgol GT の brake ゲイン 0.72 vs 位置整合 0.84)、
oracle 下限 (per-dataset 最適でも ax RMSE ≈0.17) の相当部分が GT ノイズフロアの可能性がある。

診断は 2 軸のパレートで行う (微分系 GT は「自入力の積分再現」が自明に近いため、
単一指標では平滑度の低い候補が必ず勝ってしまう):

1. 積分整合性 (ラグ・減衰を罰する):
     err_T(t) = ∫_t^{t+T} a_gt dτ − (vx(t+T) − vx(t))
   を生の localization vx に対して T = 0.5/1/3 s で計算し、窓平均指令による
   レジーム (brake/coast/throttle) 別に RMS 集計する。ブレーキ過渡のなまり損失が
   brake レジームの整合性誤差として直接現れる。
2. ノイズフロア (雑音を罰する): 定常指令区間での高周波残差
   std(a − rolling_mean(a, 0.3 s))。N-step ax RMSE の下限に直接加算される成分。

候補: 現行 savgol 0.4 s / 窓幅違い (0.2, 0.1 s) / raw accel topic (遅延 80 ms 補正) /
RTS スムーザ (状態 [v, a] のカルマン往復平滑、q 数点)。RTS は SG 微分と異なり
ゼロ遅れで平滑でき、窓端の減衰もない — 新 GT の本命候補。
"""
from __future__ import annotations

import datetime  # noqa: F401 (metadata timestamp)
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from ..load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from ..settings import RESAMPLE_DT
from ...lib._accel_source import ACCEL_DELAY_MAP, _savgol_window, rts_smooth_accel  # noqa: F401 (re-export)
from .regime import ACCEL_A_CMD, BRAKE_A_CMD
from .split import split_of

_CONTEXT = "reidentify.analyze.gt_quality"

# 積分整合性の評価窓 [s]。
CONSISTENCY_HORIZONS_S = (0.5, 1.0, 3.0)
# ノイズフロア推定: 定常指令判定と高周波分離の窓。
STEADY_A_CMD_STD_MAX = 0.05
STEADY_WINDOW_S = 1.0
NOISE_SMOOTH_S = 0.3
# RTS の process noise (ジャーク PSD) 候補 [(m/s^3)^2/Hz 相当]。大 = 観測追従 (雑音増)。
# 合成検証 (2026-07-17): q=0.2 が現行 savgol 0.4 s を過渡・雑音の両軸で支配、
# q=0.5〜1 は雑音を少し許して過渡をさらに改善するパレート点。
RTS_Q_CANDIDATES = (0.2, 0.5, 1.0)


def _savgol_deriv_window(values: np.ndarray, dt: float, window_s: float) -> np.ndarray:
    from scipy.signal import savgol_filter

    win = _savgol_window(len(values), dt, window_s, 2)
    if win is None:
        return np.gradient(values, dt)
    return savgol_filter(values, window_length=win, polyorder=2, deriv=1, delta=dt)


def _candidates(dfs: dict, t_s: np.ndarray, t0: float, vx_kin: np.ndarray, dt: float) -> dict[str, np.ndarray]:
    """候補 GT (名前 -> グリッド上の加速度) を構築する。"""
    out: dict[str, np.ndarray] = {
        "savgol_0.4 (現行)": _savgol_deriv_window(vx_kin, dt, 0.4),
        "savgol_0.2": _savgol_deriv_window(vx_kin, dt, 0.2),
        "savgol_0.1": _savgol_deriv_window(vx_kin, dt, 0.1),
    }
    df_accel = dfs["accel"]
    if not df_accel.empty:
        # accel topic は ACCEL_DELAY_MAP の遅延を持つ → t+delay の値が時刻 t の加速度。
        delay = ACCEL_DELAY_MAP["accel"]
        t_topic = (df_accel["t_ns"].to_numpy(dtype=float) - t0) * 1e-9
        out["accel_topic (遅延補正)"] = np.interp(
            t_s + delay, t_topic, df_accel["accel"].to_numpy(dtype=float),
        )
    for q in RTS_Q_CANDIDATES:
        out[f"rts_q{q:g}"] = rts_smooth_accel(vx_kin, dt, q)
    return out


def _integral_consistency(
    a: np.ndarray, vx_ref: np.ndarray, dt: float, horizon_s: float,
    regime_mask: dict[str, np.ndarray],
) -> dict[str, float]:
    """レジーム別の積分整合性 RMS [m/s]。窓 [t, t+T] の ∫a − Δvx。"""
    k = max(1, int(round(horizon_s / dt)))
    # 台形積分 (サンプリング規約の O(dt) 差を両端で折半する)。
    cum_a = np.concatenate([[0.0], np.cumsum((a[1:] + a[:-1]) * 0.5)]) * dt
    int_a = (cum_a[k:] - cum_a[:-k])[: len(a) - k]
    dv = (vx_ref[k:] - vx_ref[:-k])[: len(int_a)]
    err = int_a - dv
    out = {}
    for name, mask in regime_mask.items():
        m = mask[: len(err)]
        out[name] = float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else float("nan")
    return out


def _regime_masks(a_cmd: np.ndarray, gear: np.ndarray, dt: float, horizon_s: float) -> dict[str, np.ndarray]:
    """窓 [t, t+T] の平均指令でレジーム分類した mask 群 (all 含む)。"""
    k = max(1, int(round(horizon_s / dt)))
    cum = np.concatenate([[0.0], np.cumsum(a_cmd)])
    mean_cmd = (cum[k:] - cum[:-k]) / k
    mean_cmd = mean_cmd[: len(a_cmd) - k]
    gear_ok = np.asarray(gear, dtype=bool)[: len(mean_cmd)]
    return {
        "all": gear_ok,
        "brake": gear_ok & (mean_cmd < BRAKE_A_CMD),
        "coast": gear_ok & (mean_cmd >= BRAKE_A_CMD) & (mean_cmd <= ACCEL_A_CMD),
        "throttle": gear_ok & (mean_cmd > ACCEL_A_CMD),
    }


def _noise_floor(a: np.ndarray, a_cmd: np.ndarray, gear: np.ndarray, dt: float) -> float:
    """定常指令区間での高周波残差 std [m/s²]。"""
    win_steady = max(2, int(round(STEADY_WINDOW_S / dt)))
    steady = (
        pd.Series(a_cmd).rolling(win_steady, min_periods=win_steady).std().to_numpy()
        < STEADY_A_CMD_STD_MAX
    )
    mask = steady & np.asarray(gear, dtype=bool)
    if not mask.any():
        return float("nan")
    win_smooth = max(3, int(round(NOISE_SMOOTH_S / dt)))
    smooth = pd.Series(a).rolling(win_smooth, min_periods=1, center=True).mean().to_numpy()
    return float(np.std((a - smooth)[mask]))


def analyze_gt_quality(
    collection_dir: Path,
    scenario: Path,  # noqa: ARG001 (署名の一貫性のため受けるが GT 診断はソース非依存)
    *,
    splits: tuple[str, ...] = ("dev",),
    max_datasets: int | None = None,
) -> tuple[pd.DataFrame, dict]:
    """候補 GT × (積分整合性 レジーム×T + ノイズフロア) の診断表を返す。"""
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    rows: list[dict] = []
    n_used = 0
    for ds_id, csv_path in tasks:
        if split_of(ds_id) not in splits:
            continue
        if max_datasets is not None and n_used >= max_datasets:
            break
        try:
            dfs = read_dataset_csv(csv_path)
            res = build_resampled(dfs, RESAMPLE_DT, context=_CONTEXT, acceleration_source="accel")
        except Exception as e:  # noqa: BLE001
            print(f"[WARN] gt_quality 読み込み失敗 ({ds_id}): {e}", file=sys.stderr)
            continue
        if res is None:
            continue
        n_used += 1
        n = len(res["a_cmd"])
        t_s = np.arange(n) * RESAMPLE_DT
        df_kin = dfs["kinematic"]
        t0 = max(df["t_ns"].iloc[0] for df in (dfs["cmd"], dfs["accel"], dfs["steering"], dfs["velocity"], df_kin))
        vx_kin = np.interp(
            t_s, (df_kin["t_ns"].to_numpy(dtype=float) - t0) * 1e-9,
            df_kin["vx"].to_numpy(dtype=float),
        )
        a_cmd = np.asarray(res["a_cmd"], dtype=float)
        gear = res["gear_drive"]
        candidates = _candidates(dfs, t_s, t0, vx_kin, RESAMPLE_DT)
        for name, a in candidates.items():
            record: dict = {"dataset_id": ds_id, "candidate": name}
            for horizon_s in CONSISTENCY_HORIZONS_S:
                masks = _regime_masks(a_cmd, gear, RESAMPLE_DT, horizon_s)
                cons = _integral_consistency(a, vx_kin, RESAMPLE_DT, horizon_s, masks)
                for regime, value in cons.items():
                    record[f"int_err_{regime}_T{horizon_s:g}"] = value
            record["noise_floor"] = _noise_floor(a, a_cmd, gear, RESAMPLE_DT)
            rows.append(record)

    if not rows:
        raise RuntimeError("gt_quality を計算できる dataset が 0 件です")
    df = pd.DataFrame(rows)
    pooled = df.groupby("candidate").mean(numeric_only=True).reset_index()
    metadata = {
        "n_datasets": int(df["dataset_id"].nunique()),
        "horizons_s": list(CONSISTENCY_HORIZONS_S),
        "rts_q_candidates": list(RTS_Q_CANDIDATES),
        "rts_r_v_std": 0.03,
        "splits": sorted(splits),
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    return pooled, {"metadata": metadata, "per_dataset": df}
