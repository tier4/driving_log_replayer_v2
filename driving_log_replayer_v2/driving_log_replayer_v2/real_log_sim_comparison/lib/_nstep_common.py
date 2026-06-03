"""N-step オープンループ解析 (step5/6/7) の共有定数・集計ヘルパー.

nstep_delta.csv の統一スキーマ (horizon, tr, err_ds_long/lat, err_steer,
pos_err, yaw_err_deg, ...) を消費するモジュール間で、メトリクス定義・
注意書き・horizon 集計を一元管理する。
"""

from __future__ import annotations

from collections.abc import Iterable
import math

import numpy as np
import pandas as pd

# 誤差メトリクス定義: (列名, スケール, ラベル, 単位, データソース注)。
# step5 の時系列/散布図と step6 の overlay が同じ並びで使う。
ERR_METRICS: list[tuple[str, float, str, str, str]] = [
    ("err_ds_long", 100.0, "縦方向誤差", "cm",
     "実機: kinematic_state/pose.position  モデル: rollout 終端 state_[0,1]"),
    ("err_ds_lat", 100.0, "横方向誤差", "cm",
     "実機: kinematic_state/pose.position  モデル: rollout 終端 state_[0,1]"),
    ("err_steer", 180.0 / math.pi, "ステア予測誤差", "deg",
     "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias"),
    ("yaw_err_deg", 1.0, "yaw 誤差", "deg",
     "実機: kinematic_state/pose.orientation  モデル: rollout 終端 state_[2]"),
]

# yaw/wz 誤差パネルに必ず添える seed バイアス注意 (step5 run_rollout docstring 参照)
YAW_SEED_NOTE = (
    "注: 小 N の yaw 誤差は seed (k_us=0 bicycle 逆算) 由来のバイアスを含む。"
    "dynamics 差の判定は大 N を参照"
)


def n1(df: pd.DataFrame) -> pd.DataFrame:
    """最小 horizon (通常 N=1, 毎ステップリセット相当) のサブセットを返す。"""
    return df[df["horizon"] == df["horizon"].min()]


def common_horizons(horizon_sets: Iterable[Iterable[int]]) -> list[int]:
    """全集合に共通する horizon の昇順リストを返す。"""
    return sorted(int(h) for h in set.intersection(*map(set, horizon_sets)))


def rmse_by_horizon(df: pd.DataFrame) -> dict[int, dict[str, float]]:
    """horizon 別の終端誤差 RMSE を返す: {N: {"pos","long","lat" [cm], "yaw" [deg]}}。"""

    def _rms(v: np.ndarray) -> float:
        return float(np.sqrt(np.nanmean(np.asarray(v, dtype=float) ** 2)))

    out: dict[int, dict[str, float]] = {}
    for horizon in sorted(df["horizon"].unique()):
        sub = df[df["horizon"] == horizon]
        out[int(horizon)] = {
            "pos": _rms(sub["pos_err"].values) * 100.0,
            "long": _rms(sub["err_ds_long"].values) * 100.0,
            "lat": _rms(sub["err_ds_lat"].values) * 100.0,
            "yaw": _rms(sub["yaw_err_deg"].values),
        }
    return out
