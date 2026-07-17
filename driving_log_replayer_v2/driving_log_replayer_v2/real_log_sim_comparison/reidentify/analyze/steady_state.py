"""A4: 定常 Hammerstein マップ (cmd → achieved の静的非線形の直接推定)。

resampled グリッド (dt=0.01) 上で指令が定常な区間を検出し、a_cmd vs a_act /
d_cmd vs d_act の bin 中央値カーブを作る。グローバル scaling (v2 の
debug_*_scaling_factor) で表現できない静的非線形 (原点オフセット・折れ線・
領域依存ゲイン) を検出する。|pitch| 小の条件付き版も出し、acc 定常ゲインの
勾配交絡 (scaling 0.9 クランプが勾配由来か) を独立判定する。

定常判定は指令側 (外生) のみで行う: 出力側で選ぶと出力ノイズによる選択バイアスが入る。
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

# 定常判定: 直近 STEADY_WINDOW_S の指令 rolling std がしきい値未満。
STEADY_WINDOW_S = 1.0
STEADY_A_CMD_STD_MAX = 0.05   # m/s^2
STEADY_D_CMD_STD_MAX = 0.002  # rad
STEADY_VX_MIN = 1.0
# 勾配交絡除去版の pitch 制限 (|pitch| < 0.5 deg)。
FLAT_PITCH_MAX_RAD = np.deg2rad(0.5)
# bin 設定 (指令値で固定幅 bin、解釈性優先)。
ACC_BIN_EDGES = np.arange(-3.0, 3.01, 0.2)
STEER_BIN_EDGES = np.arange(-0.5, 0.501, 0.02)
MIN_BIN_ROWS = 200
# ゲイン推定の指令領域 (原点近傍の不感帯・飽和域を避ける)。
ACC_GAIN_RANGE = (0.2, 1.5)
STEER_GAIN_RANGE = (0.02, 0.4)

_CONTEXT = "reidentify.analyze.steady_state"


def _steady_points(res: dict) -> pd.DataFrame:
    """1 dataset の定常サンプル (a_cmd, a_act, d_cmd, d_act, vx, pitch) を返す。"""
    a_cmd = pd.Series(np.asarray(res["a_cmd"], dtype=float))
    d_cmd = pd.Series(np.asarray(res["d_cmd"], dtype=float))
    win = max(2, int(round(STEADY_WINDOW_S / RESAMPLE_DT)))
    a_steady = a_cmd.rolling(win, min_periods=win).std().values < STEADY_A_CMD_STD_MAX
    d_steady = d_cmd.rolling(win, min_periods=win).std().values < STEADY_D_CMD_STD_MAX

    vx = np.asarray(res["vx"], dtype=float)
    base = np.asarray(res["gear_drive"], dtype=bool) & (vx > STEADY_VX_MIN)
    mask = base & a_steady & d_steady
    if not np.any(mask):
        return pd.DataFrame()
    return pd.DataFrame({
        "a_cmd": np.asarray(res["a_cmd"], dtype=float)[mask],
        "a_act": np.asarray(res["a_act"], dtype=float)[mask],
        "d_cmd": np.asarray(res["d_cmd"], dtype=float)[mask],
        "d_act": np.asarray(res["d_act"], dtype=float)[mask],
        "vx": vx[mask],
        "pitch": np.asarray(res["pitch"], dtype=float)[mask],
    })


def _bin_curve(x: np.ndarray, y: np.ndarray, edges: np.ndarray, label: str) -> pd.DataFrame:
    idx = np.searchsorted(edges, x, side="right") - 1
    valid = (idx >= 0) & (idx < len(edges) - 1)
    df = pd.DataFrame({"bin": idx[valid], "x": x[valid], "y": y[valid]})
    rows = []
    for b, sub in df.groupby("bin"):
        if len(sub) < MIN_BIN_ROWS:
            continue
        rows.append({
            "channel": label,
            "x_center": float((edges[b] + edges[b + 1]) / 2.0),
            "n": int(len(sub)),
            "y_median": float(sub["y"].median()),
            "y_p25": float(sub["y"].quantile(0.25)),
            "y_p75": float(sub["y"].quantile(0.75)),
        })
    return pd.DataFrame(rows)


def _gain_through_origin(x: np.ndarray, y: np.ndarray, lo: float, hi: float) -> dict:
    """指令領域 [lo, hi] (符号別) の原点通過ゲイン y/x の中央値。"""
    out = {}
    for name, mask in (
        ("pos", (x >= lo) & (x <= hi)),
        ("neg", (x <= -lo) & (x >= -hi)),
    ):
        n = int(mask.sum())
        out[f"gain_{name}"] = float(np.median(y[mask] / x[mask])) if n >= MIN_BIN_ROWS else float("nan")
        out[f"n_{name}"] = n
    return out


def analyze_steady_state(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str,
    splits: tuple[str, ...] = ("dev",),
) -> tuple[pd.DataFrame, dict]:
    """定常マップ (bin カーブ表) とゲイン推定サマリを返す。"""
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    accel_source = cfg.find_case(case_name).acceleration_source

    frames: list[pd.DataFrame] = []
    for ds_id, csv_path in tasks:
        if split_of(ds_id) not in splits:
            continue
        try:
            dfs = read_dataset_csv(csv_path)
            res = build_resampled(
                dfs, RESAMPLE_DT, context=_CONTEXT, acceleration_source=accel_source,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[WARN] steady_state 読み込み失敗 ({ds_id}): {e}", file=sys.stderr)
            continue
        if res is None:
            continue
        pts = _steady_points(res)
        if not pts.empty:
            # bin 集計とゲイン推定のみに使うため 1/2 に間引く (メモリ節約)。
            frames.append(pts.iloc[::2])
    if not frames:
        raise RuntimeError("定常サンプルを持つ dataset が 0 件です")
    pts = pd.concat(frames, ignore_index=True)

    flat = pts[np.abs(pts["pitch"]) < FLAT_PITCH_MAX_RAD]
    curves = pd.concat([
        _bin_curve(pts["a_cmd"].values, pts["a_act"].values, ACC_BIN_EDGES, "acc_all"),
        _bin_curve(flat["a_cmd"].values, flat["a_act"].values, ACC_BIN_EDGES, "acc_flat_pitch"),
        _bin_curve(pts["d_cmd"].values, pts["d_act"].values, STEER_BIN_EDGES, "steer_all"),
    ], ignore_index=True)

    summary: dict = {
        "n_points": int(len(pts)),
        "n_points_flat_pitch": int(len(flat)),
        "acc": _gain_through_origin(
            pts["a_cmd"].values, pts["a_act"].values, *ACC_GAIN_RANGE,
        ),
        "acc_flat_pitch": _gain_through_origin(
            flat["a_cmd"].values, flat["a_act"].values, *ACC_GAIN_RANGE,
        ),
        "steer": _gain_through_origin(
            pts["d_cmd"].values, pts["d_act"].values, *STEER_GAIN_RANGE,
        ),
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    return curves, summary
