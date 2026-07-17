"""localization EKF の観測モデル同定 (レポート用の正式ステージ)。

車両動特性の同定 (3/4 章) と同じ「むだ時間 + 時定数」の枠組みで、localization の
出力特性を同定する。対象は 2 つ:

1. /localization/acceleration の観測特性:
     a_topic ≈ delay(LPF1(a_true; τ_loc); d_loc)
   a_true のプロキシは RTS 平滑加速度 (kinematic vx 由来、低ラグ・低ノイズ)。
   dataset ごとに (d, τ) をグリッド最小二乗で同定し、分布 (中央値/IQR) を報告する。

2. pose–twist の位相差・スケール差:
     Δ(pose 変位 − ∫twist vx dt) = L_pose·Δv + c_scale·path + c_pitch·Δpitch + c0
   の窓回帰 (1 s 窓)。L_pose = pose の実効ラグ、c_scale = 速度スケール差。
   c_pitch ≈ 0 ならピッチ投影説は棄却。レジーム別の変位差も併record する。

これらは車両ではなく localization の性質であり、モデルが再現すべきではない。
オフライン評価では視点変換 (lib/_localization_observation) で吸収し、本番
シミュレータでは下流へ渡す localization 出力に反映する対象となる。
"""
from __future__ import annotations

import datetime
from pathlib import Path
import sys

import numpy as np

from ..load_data import discover_cached_datasets, read_dataset_csv
from ...lib._accel_source import rts_smooth_accel
from ...lib._localization_observation import first_order_lpf
from .split import split_of

DT = 0.01
# accel topic 同定のグリッド。
ACCEL_DELAY_GRID = np.arange(0.0, 0.21, 0.02)
ACCEL_TAU_GRID = np.arange(0.0, 0.61, 0.05)
ACCEL_FIT_MAX_DATASETS = 60
# pose–twist 窓回帰。
POSE_WINDOW_S = 1.0
POSE_WINDOW_STRIDE = 50
BRAKE_A_CMD = -0.3
ACCEL_A_CMD = 0.3


def _fit_accel_topic(kin, acc) -> tuple[float, float, float, float] | None:
    """1 dataset の (delay, tau, fit_rms, raw_rms) を返す。"""
    if len(kin) < 100 or len(acc) < 100:
        return None
    t0 = max(kin["t_ns"].iloc[0], acc["t_ns"].iloc[0])
    t1 = min(kin["t_ns"].iloc[-1], acc["t_ns"].iloc[-1])
    t = np.arange(0, (t1 - t0) * 1e-9, DT)
    if len(t) < 2000:
        return None
    vx = np.interp(t, (kin["t_ns"].to_numpy(float) - t0) * 1e-9, kin["vx"].to_numpy(float))
    a_topic = np.interp(t, (acc["t_ns"].to_numpy(float) - t0) * 1e-9, acc["accel"].to_numpy(float))
    a_true = rts_smooth_accel(vx, DT, q_jerk=0.5)
    mask = vx > 2.0
    if mask.sum() < 2000:
        return None
    best = (np.inf, 0.0, 0.0)
    for tau in ACCEL_TAU_GRID:
        filtered = first_order_lpf(a_true, DT, tau)
        for delay in ACCEL_DELAY_GRID:
            k = int(round(delay / DT))
            pred = np.concatenate([np.full(k, filtered[0]), filtered[: len(filtered) - k]]) if k else filtered
            rms = float(np.sqrt(np.mean((a_topic[mask] - pred[mask]) ** 2)))
            if rms < best[0]:
                best = (rms, delay, tau)
    raw_rms = float(np.sqrt(np.mean((a_topic[mask] - a_true[mask]) ** 2)))
    return best[1], best[2], best[0], raw_rms


def analyze_observation_model(
    collection_dir: Path,
    *,
    splits: tuple[str, ...] = ("dev",),
) -> dict:
    """観測モデル同定の結果 dict (JSON 化可能) を返す。"""
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    accel_fits: list[tuple[float, float, float, float]] = []
    reg_rows: list[list[float]] = []
    regime_delta: dict[str, list[float]] = {"brake": [], "coast": [], "throttle": []}
    n_ds = 0
    k_win = int(round(POSE_WINDOW_S / DT))
    for ds_id, csv_path in tasks:
        if split_of(ds_id) not in splits:
            continue
        try:
            dfs = read_dataset_csv(csv_path)
        except Exception as e:  # noqa: BLE001
            print(f"[WARN] observation_model 読み込み失敗 ({ds_id}): {e}", file=sys.stderr)
            continue
        kin = dfs["kinematic"]
        if len(kin) < 100:
            continue
        n_ds += 1
        if len(accel_fits) < ACCEL_FIT_MAX_DATASETS:
            fit = _fit_accel_topic(kin, dfs["accel"])
            if fit is not None:
                accel_fits.append(fit)

        # pose–twist 窓回帰の材料。
        t_raw = (kin["t_ns"].to_numpy(float) - kin["t_ns"].iloc[0]) * 1e-9
        t = np.arange(t_raw[0], t_raw[-1], DT)
        x = np.interp(t, t_raw, kin["x"].to_numpy(float))
        y = np.interp(t, t_raw, kin["y"].to_numpy(float))
        vx = np.interp(t, t_raw, kin["vx"].to_numpy(float))
        pitch = np.interp(t, t_raw, kin["pitch"].to_numpy(float))
        cmd = dfs["cmd"]
        a_cmd = np.interp(
            t, (cmd["t_ns"].to_numpy(float) - kin["t_ns"].iloc[0]) * 1e-9,
            cmd["cmd_accel"].to_numpy(float),
        )
        s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
        cumv = np.concatenate([[0.0], np.cumsum((vx[1:] + vx[:-1]) * 0.5 * DT)])
        cum_cmd = np.concatenate([[0.0], np.cumsum(a_cmd)])
        for k0 in range(0, len(t) - k_win, POSE_WINDOW_STRIDE):
            ke = k0 + k_win
            if vx[k0] < 2.0 or vx[ke] < 0.5:
                continue
            delta = (s[ke] - s[k0]) - (cumv[ke] - cumv[k0])
            reg_rows.append([vx[ke] - vx[k0], cumv[ke] - cumv[k0], pitch[ke] - pitch[k0], delta])
            mean_cmd = (cum_cmd[ke] - cum_cmd[k0]) / k_win
            regime = "brake" if mean_cmd < BRAKE_A_CMD else (
                "throttle" if mean_cmd > ACCEL_A_CMD else "coast")
            regime_delta[regime].append(delta * 100.0)  # cm

    if not accel_fits or not reg_rows:
        raise RuntimeError("observation_model を同定できる dataset が不足しています")

    fits = np.array(accel_fits)
    rows = np.array(reg_rows)
    design = np.column_stack([rows[:, 0], rows[:, 1], rows[:, 2], np.ones(len(rows))])
    coef, *_ = np.linalg.lstsq(design, rows[:, 3], rcond=None)
    pred = design @ coef
    ss_res = float(np.sum((rows[:, 3] - pred) ** 2))
    ss_tot = float(np.sum((rows[:, 3] - np.mean(rows[:, 3])) ** 2))

    def _pct(a: np.ndarray, q: float) -> float:
        return float(np.percentile(a, q))

    return {
        "accel_topic": {
            "model": "a_topic = delay(LPF1(a_true; tau); d)",
            "delay_s": {"median": float(np.median(fits[:, 0])),
                        "p25": _pct(fits[:, 0], 25), "p75": _pct(fits[:, 0], 75)},
            "tau_s": {"median": float(np.median(fits[:, 1])),
                      "p25": _pct(fits[:, 1], 25), "p75": _pct(fits[:, 1], 75)},
            "fit_rms": float(np.median(fits[:, 2])),
            "raw_rms": float(np.median(fits[:, 3])),
            "n_datasets": int(len(fits)),
        },
        "pose_twist": {
            "model": "delta(pose_disp - int(vx dt)) = lag*dv + scale*path + pitch_coef*dpitch + c0",
            "lag_s": float(coef[0]),
            "scale": float(coef[1]),
            "pitch_coef_m": float(coef[2]),
            "const_m": float(coef[3]),
            "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
            "n_windows": int(len(rows)),
            "window_s": POSE_WINDOW_S,
            "regime_delta_cm": {
                name: {"mean": float(np.mean(v)), "median": float(np.median(v)), "n": len(v)}
                for name, v in regime_delta.items() if v
            },
        },
        "n_datasets": n_ds,
        "splits": sorted(splits),
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
