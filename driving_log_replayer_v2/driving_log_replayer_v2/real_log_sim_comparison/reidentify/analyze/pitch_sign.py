"""pitch 符号規約の経験的検証 (slope フィード実装前の必須ステップ)。

extract.py の kinematic pitch は ZYX オイラーの asin(2(w·y − z·x))。理論上は
「正 pitch = ノーズ下げ (下り)」で、重力の body-x 投影は +g·sin(pitch)
(dvx/dt を加速させる向き) のはず。ここではコースト区間 (指令加速度がほぼ 0 で
持続) の実加速度 a_act を g·sin(pitch) に回帰し、係数 β ≈ +1 を経験的に確認する。
β ≈ −1 なら符号規約が逆 (正 pitch = ノーズ上げ) なので slope フィードは
-g·sin(pitch) を用いる。走行抵抗は切片 (+ pooled 回帰では vx² 項) に吸収させる。
"""
from __future__ import annotations

import datetime
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from ..load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from ..model_config import load_model_config
from ..settings import RESAMPLE_DT
from .split import split_of

GRAVITY = 9.80665
# コースト判定: 直近 COAST_PERSIST_S の間 |a_cmd| < COAST_A_CMD_MAX が持続し、
# vx > COAST_VX_MIN (低速の停止揺らぎ・クリープを除外)。
COAST_A_CMD_MAX = 0.05
COAST_VX_MIN = 3.0
COAST_PERSIST_S = 1.0
# 回帰の成立条件: 有効サンプル数と g·sin(pitch) の変動幅 (これ未満は勾配情報なし)。
MIN_COAST_SAMPLES = 200
MIN_GSIN_STD = 0.02

_CONTEXT = "reidentify.analyze.pitch_sign"


def _dataset_regression(res: dict) -> dict | None:
    """1 dataset のコースト区間で a_act = β·g·sin(pitch) + c を回帰する。"""
    a_cmd = np.asarray(res["a_cmd"], dtype=float)
    a_act = np.asarray(res["a_act"], dtype=float)
    vx = np.asarray(res["vx"], dtype=float)
    pitch = np.asarray(res["pitch"], dtype=float)
    gear = np.asarray(res["gear_drive"], dtype=bool)

    persist = max(1, int(round(COAST_PERSIST_S / RESAMPLE_DT)))
    calm = pd.Series(np.abs(a_cmd)).rolling(persist, min_periods=persist).max().values
    mask = gear & (vx > COAST_VX_MIN) & (calm < COAST_A_CMD_MAX)
    n = int(np.count_nonzero(mask))
    if n < MIN_COAST_SAMPLES:
        return None

    g_sin = GRAVITY * np.sin(pitch[mask])
    y = a_act[mask]
    g_std = float(np.std(g_sin))
    if g_std < MIN_GSIN_STD:
        return {"n": n, "g_sin_std": g_std, "slope": np.nan, "intercept": np.nan, "r2": np.nan}

    design = np.column_stack([g_sin, np.ones_like(g_sin)])
    coef, _res, _rank, _sv = np.linalg.lstsq(design, y, rcond=None)
    pred = design @ coef
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return {
        "n": n,
        "g_sin_std": g_std,
        "slope": float(coef[0]),
        "intercept": float(coef[1]),
        "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan,
        "_g_sin": g_sin,
        "_y": y,
        "_vx": vx[mask],
    }


def verify_pitch_sign(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str,
    splits: tuple[str, ...] = ("dev",),
) -> tuple[pd.DataFrame, dict]:
    """全 dev dataset のコースト回帰を集計し、pitch 符号の判定サマリを返す。"""
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    accel_source = cfg.find_case(case_name).acceleration_source

    rows: list[dict] = []
    pooled_g_sin: list[np.ndarray] = []
    pooled_y: list[np.ndarray] = []
    pooled_vx: list[np.ndarray] = []
    for ds_id, csv_path in tasks:
        if split_of(ds_id) not in splits:
            continue
        try:
            dfs = read_dataset_csv(csv_path)
            res = build_resampled(
                dfs, RESAMPLE_DT, context=_CONTEXT, acceleration_source=accel_source,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[WARN] pitch_sign 読み込み失敗 ({ds_id}): {e}", file=sys.stderr)
            continue
        if res is None:
            continue
        reg = _dataset_regression(res)
        if reg is None:
            continue
        if "_g_sin" in reg:
            # プール回帰はメモリ節約のため 1/5 に間引く。
            pooled_g_sin.append(reg.pop("_g_sin")[::5])
            pooled_y.append(reg.pop("_y")[::5])
            pooled_vx.append(reg.pop("_vx")[::5])
        rows.append({"dataset_id": ds_id, **reg})

    if not rows:
        raise RuntimeError("コースト区間を持つ dataset が 0 件です")
    df = pd.DataFrame(rows)

    valid = df[np.isfinite(df["slope"])]
    summary: dict = {
        "n_datasets_with_coast": int(len(df)),
        "n_datasets_regressed": int(len(valid)),
        "median_slope": float(valid["slope"].median()) if len(valid) else float("nan"),
        "positive_sign_rate": float((valid["slope"] > 0).mean()) if len(valid) else float("nan"),
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }

    if pooled_g_sin:
        g_sin = np.concatenate(pooled_g_sin)
        y = np.concatenate(pooled_y)
        vx = np.concatenate(pooled_vx)
        # 走行抵抗 (c0 + c2 v²) を吸収する pooled 回帰: a_act = β·g·sin(pitch) + c0 + c2·vx²。
        design = np.column_stack([g_sin, np.ones_like(g_sin), vx**2])
        coef, _res, _rank, _sv = np.linalg.lstsq(design, y, rcond=None)
        pred = design @ coef
        ss_res = float(np.sum((y - pred) ** 2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2))
        summary.update({
            "pooled_n": int(len(y)),
            "pooled_slope": float(coef[0]),
            "pooled_intercept": float(coef[1]),
            "pooled_v2_coeff": float(coef[2]),
            "pooled_r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
        })

    slope = summary.get("pooled_slope", summary["median_slope"])
    if np.isfinite(slope) and abs(slope) > 0.3:
        sign = "+" if slope > 0 else "-"
        summary["verdict"] = (
            f"slope_accx = {sign}g*sin(pitch) (回帰係数 {slope:+.3f}; "
            f"正 pitch = {'ノーズ下げ (下り)' if slope > 0 else 'ノーズ上げ (上り)'})"
        )
    else:
        summary["verdict"] = f"判定不能 (回帰係数 {slope:+.3f} が小さすぎる — pitch 品質を確認)"
    return df, summary
