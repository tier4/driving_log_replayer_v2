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
    ("err_vx", 1.0, "速度予測誤差", "m/s",
     "実機: velocity_status vx[k_end]  モデル: rollout 終端 state_[3]"),
    ("err_ax", 1.0, "加速度予測誤差", "m/s²",
     "実機: localization/acceleration ax[k_end]  モデル: rollout 終端 state_[5]"),
]

# yaw/wz 誤差パネルに必ず添える seed バイアス注意 (step5 run_rollout docstring 参照)
YAW_SEED_NOTE = (
    "注: 小 N の yaw 誤差は seed (k_us=0 bicycle 逆算) 由来のバイアスを含む。"
    "dynamics 差の判定は大 N を参照"
)


def metrics_description_md() -> str:
    """各レポート冒頭に埋め込むメトリクス説明 (Markdown)。

    ERR_METRICS / YAW_SEED_NOTE を一次情報として、N-step オープンループ評価で
    使う縦/横/yaw/steer 誤差の定義・座標系・データソースと N-step の意味を一元的に記す。
    step5(summary.txt) は `# ` プレフィックスを付けて行コメントとして埋め込む。
    """
    return (
        "## メトリクス説明\n"
        "\n"
        "各開始点 k0 で実機状態 (過去コマンド履歴を含む) にリセットし、実コマンド系列を\n"
        "N ステップ連続適用 (途中リセット無し = free-running) した終端 k_end=k0+N の予測状態と\n"
        "実機状態を比較した終端誤差の RMSE を horizon (N) 別に集計する。\n"
        "N を増やすほど dynamics 差 (k_us / wheelbase / 各時定数) の累積が顕在化する。\n"
        "\n"
        "- **縦方向誤差 (long)** [cm]: k0 時点の実機ヨーを基準とした車両ローカル座標系での\n"
        "  前後方向の変位誤差。実機 kinematic_state/pose.position と rollout 終端 state_[0,1] の差。\n"
        "- **横方向誤差 (lat)** [cm]: 同ローカル座標系での左右方向の変位誤差 (データソースは縦と同じ)。\n"
        "- **yaw 誤差 (yaw)** [deg]: ヨー角の差 (−π〜π に正規化)。\n"
        "  実機 kinematic_state/pose.orientation と rollout 終端 state_[2] の差。\n"
        "- **ステア予測誤差 (steer)** [deg]: 実機 steering_status/tire_angle と"
        " モデル state_[4]+steer_bias の差。\n"
        "- **速度予測誤差 (vx)** [m/s]: 実機 velocity_status と rollout 終端 state_[3] の差。\n"
        "  縦方向ダイナミクス (acc 時定数/むだ時間/スケーリング) の累積効果を保持し、終端評価でも弁別力が高い。\n"
        "- **加速度予測誤差 (ax)** [m/s²]: 実機 localization/acceleration と rollout 終端 state_[5] の差。\n"
        "  瞬時応答を直接反映するが信号が雑なため、縦方向同定では vx を主・ax を副に用いる。\n"
        "\n"
        f"> {YAW_SEED_NOTE}。\n"
    )


def n1(df: pd.DataFrame) -> pd.DataFrame:
    """最小 horizon (通常 N=1, 毎ステップリセット相当) のサブセットを返す。"""
    return df[df["horizon"] == df["horizon"].min()]


def common_horizons(horizon_sets: Iterable[Iterable[int]]) -> list[int]:
    """全集合に共通する horizon の昇順リストを返す。"""
    return sorted(int(h) for h in set.intersection(*map(set, horizon_sets)))


def parabolic_min(xs: list[float], ys: list[float]) -> float | None:
    """argmin 近傍 3 点に二次フィットして頂点 (サブグリッド最小) を返す。端なら None。

    パラメータ sweep 同定 (step7) の共有ヘルパー。
    """
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


def rmse_by_horizon(df: pd.DataFrame) -> dict[int, dict[str, float]]:
    """horizon 別の終端誤差 RMSE を返す。

    {N: {"pos","long","lat" [cm], "yaw" [deg], "steer" [deg], "vx" [m/s], "ax" [m/s²]}}

    vx/ax 列が無い古い nstep_delta.csv との後方互換のため、列欠落時はそのキーを省く。
    """

    def _rms(v: np.ndarray) -> float:
        return float(np.sqrt(np.nanmean(np.asarray(v, dtype=float) ** 2)))

    out: dict[int, dict[str, float]] = {}
    for horizon in sorted(df["horizon"].unique()):
        sub = df[df["horizon"] == horizon]
        rec = {
            "pos": _rms(sub["pos_err"].values) * 100.0,
            "long": _rms(sub["err_ds_long"].values) * 100.0,
            "lat": _rms(sub["err_ds_lat"].values) * 100.0,
            "yaw": _rms(sub["yaw_err_deg"].values),
            "steer": _rms(sub["err_steer"].values) * 180.0 / math.pi,
        }
        if "err_vx" in sub.columns:
            rec["vx"] = _rms(sub["err_vx"].values)  # [m/s]
        if "err_ax" in sub.columns:
            rec["ax"] = _rms(sub["err_ax"].values)  # [m/s²]
        out[int(horizon)] = rec
    return out
