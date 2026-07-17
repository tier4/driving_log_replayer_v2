"""A3: ETFE 周波数応答 (cmd → achieved) — 1 次 + むだ時間モデルの適合性チェック。

per-dataset の Welch クロススペクトルを入力パワー S_uu で重み付きプールし、
G(f) = S_uy / S_uu とコヒーレンスを推定する。v2 の 1 次 + むだ時間
G_model(f) = K·e^{-j2πfL} / (1 + j2πfτ) と振幅・位相を比較する。

注意 (事前登録): 運転は閉ループ (cmd が外乱とフィードバック相関) なので、特に低域
(<0.1 Hz) の ETFE はバイアスを持つ。ここでの結果は「1 次 + むだ時間で足りるか」の
適合性チェックに限定し、パラメータ同定には使わない。
判定の観点: 高周波位相の超過遅れ → 2 次系候補 / 入力振幅依存 → レートリミット・非線形。
"""
from __future__ import annotations

import datetime
from pathlib import Path
import sys

import numpy as np
import pandas as pd
from scipy.signal import csd, welch

from ..load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from ..model_config import load_model_config
from ..settings import RESAMPLE_DT
from .split import split_of

# Welch セグメント長 (秒)。dataset は約 60 s なので 20 s セグメント (50% overlap で
# 実質 5 平均、0.05 Hz 分解能) とし、drive 区間 30 s 以上を成立条件にする。
SEGMENT_S = 20.0
FREQ_MIN_HZ = 0.05
FREQ_MAX_HZ = 5.0
MIN_SAMPLES = 3000  # 30 s 以上
# 入力振幅 tercile 分割用: dataset ごとの入力 RMS で 3 群に分ける。
_CONTEXT = "reidentify.analyze.etfe"

CHANNELS: tuple[tuple[str, str, str], ...] = (
    ("acc", "a_cmd", "a_act"),
    ("steer", "d_cmd", "d_act"),
)


def _dataset_spectra(res: dict, fs: float) -> dict[str, dict] | None:
    """1 dataset の S_uu / S_uy / S_yy (drive 区間のみ、チャネル別)。"""
    gear = np.asarray(res["gear_drive"], dtype=bool)
    if int(gear.sum()) < MIN_SAMPLES:
        return None
    nperseg = int(SEGMENT_S * fs)
    out: dict[str, dict] = {}
    for name, u_key, y_key in CHANNELS:
        u = np.asarray(res[u_key], dtype=float)[gear]
        y = np.asarray(res[y_key], dtype=float)[gear]
        if len(u) < nperseg:
            return None
        u = u - float(np.mean(u))
        y = y - float(np.mean(y))
        f, s_uu = welch(u, fs=fs, nperseg=nperseg)
        _, s_yy = welch(y, fs=fs, nperseg=nperseg)
        _, s_uy = csd(u, y, fs=fs, nperseg=nperseg)
        band = (f >= FREQ_MIN_HZ) & (f <= FREQ_MAX_HZ)
        out[name] = {
            "f": f[band], "s_uu": s_uu[band], "s_yy": s_yy[band], "s_uy": s_uy[band],
            "u_rms": float(np.sqrt(np.mean(u**2))),
        }
    return out


def _pool(spectra: list[dict]) -> pd.DataFrame:
    """入力パワー重み付きプール: G = Σ S_uy / Σ S_uu、coh² = |Σ S_uy|² / (Σ S_uu · Σ S_yy)。"""
    f = spectra[0]["f"]
    s_uu = np.sum([s["s_uu"] for s in spectra], axis=0)
    s_yy = np.sum([s["s_yy"] for s in spectra], axis=0)
    s_uy = np.sum([s["s_uy"] for s in spectra], axis=0)
    g = s_uy / s_uu
    coh2 = np.abs(s_uy) ** 2 / (s_uu * s_yy)
    return pd.DataFrame({
        "freq_hz": f,
        "gain": np.abs(g),
        "phase_deg": np.degrees(np.unwrap(np.angle(g))),
        "coherence2": coh2,
        "n_datasets": len(spectra),
    })


def model_response(freq_hz: np.ndarray, k: float, tau: float, delay: float) -> tuple[np.ndarray, np.ndarray]:
    """1 次 + むだ時間 G(f) = K·e^{-j2πfL}/(1+j2πfτ) の (gain, phase_deg)。"""
    w = 2.0 * np.pi * np.asarray(freq_hz, dtype=float)
    g = k * np.exp(-1j * w * delay) / (1.0 + 1j * w * tau)
    return np.abs(g), np.degrees(np.unwrap(np.angle(g)))


def analyze_etfe(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str,
    splits: tuple[str, ...] = ("dev",),
) -> tuple[pd.DataFrame, dict]:
    """プール ETFE 表 (チャネル × 振幅 tercile) と v2 モデル応答パラメータを返す。"""
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    case = cfg.find_case(case_name)
    accel_source = case.acceleration_source
    fs = 1.0 / RESAMPLE_DT

    per_ds: dict[str, list[dict]] = {name: [] for name, _u, _y in CHANNELS}
    for ds_id, csv_path in tasks:
        if split_of(ds_id) not in splits:
            continue
        try:
            dfs = read_dataset_csv(csv_path)
            res = build_resampled(
                dfs, RESAMPLE_DT, context=_CONTEXT, acceleration_source=accel_source,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[WARN] etfe 読み込み失敗 ({ds_id}): {e}", file=sys.stderr)
            continue
        if res is None:
            continue
        spectra = _dataset_spectra(res, fs)
        if spectra is None:
            continue
        for name, _u, _y in CHANNELS:
            per_ds[name].append(spectra[name])

    frames: list[pd.DataFrame] = []
    for name, _u, _y in CHANNELS:
        group = per_ds[name]
        if not group:
            continue
        pooled = _pool(group)
        pooled.insert(0, "amplitude_group", "all")
        pooled.insert(0, "channel", name)
        frames.append(pooled)
        # 入力振幅 tercile 別 (レートリミット/非線形の振幅依存チェック)。
        rms = np.array([s["u_rms"] for s in group])
        q1, q2 = np.quantile(rms, [1 / 3, 2 / 3])
        for label, mask in (
            ("low", rms <= q1),
            ("mid", (rms > q1) & (rms <= q2)),
            ("high", rms > q2),
        ):
            sel = [s for s, m in zip(group, mask) if m]
            if len(sel) < 3:
                continue
            pooled_t = _pool(sel)
            pooled_t.insert(0, "amplitude_group", label)
            pooled_t.insert(0, "channel", name)
            frames.append(pooled_t)
    if not frames:
        raise RuntimeError("ETFE を計算できる dataset が 0 件です")

    params = case.params
    metadata = {
        "case": case_name,
        "model_response": {
            "acc": {
                "k": float(params.get("debug_acc_scaling_factor", 1.0)),
                "tau": float(params["acc_time_constant"]),
                "delay": float(params["acc_time_delay"]),
            },
            "steer": {
                "k": float(params.get("debug_steer_scaling_factor", 1.0)),
                "tau": float(params["steer_time_constant"]),
                "delay": float(params["steer_time_delay"]),
            },
        },
        "segment_s": SEGMENT_S,
        "freq_range_hz": [FREQ_MIN_HZ, FREQ_MAX_HZ],
        "splits": sorted(splits),
        "n_datasets": {name: len(group) for name, group in per_ds.items()},
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    return pd.concat(frames, ignore_index=True), metadata
