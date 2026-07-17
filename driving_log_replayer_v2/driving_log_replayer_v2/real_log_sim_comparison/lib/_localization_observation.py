"""localization EKF の観測モデル — 真値状態から localization 視点の出力への変換。

実データ同定 (2026-07-17, openloop_j6_16_onwards dev split):

1. pose–twist 位相差/スケール差 (窓回帰 n=16,127, R²=0.74):
     Δ(pose変位 − ∫twist vx dt) = +93 ms·Δv − 0.58%·path (+定数)
   Δv 係数が「正」であることは、twist vx が pose (地図アンカー) に対して実効 ~93 ms
   遅延していることを意味する (pose が遅れているなら係数は負になる: pose 変位 ≈
   真値変位 − L·Δv)。当初は「pose 遅れ」と解釈していたが、整合フレームの実データ検証
   (2026-07-17) で pose 前進補正がレジーム別変位差を悪化させたことから符号解釈を訂正した。
   pose のタイミングは steer→yaw (指令→応答) チェーンの同定と整合しており、動かすと
   横系の同定を壊すため、補正は twist 側 (前進 + スケール) に適用する。
   twist vx は pose 距離より ~0.4–0.6% 過大。ピッチ投影説は棄却 (Δpitch 係数 ≈ 0.09 m ≈ 0)。
   この不整合が N-step 評価で「ax は減速を弱めろ (twist 系) / long は強めろ (pose 系)」
   という矛盾要求を生み、縦系の構造改善 (brake 分離・bite) がゲートで棄却され続けた
   根本原因。

2. /localization/acceleration の観測特性 (60 datasets, 中央値が同一最適点):
     a_topic ≈ delay(LPF1(a_twist; τ=0.15 s); d=0.02 s)
   残差 RMS 0.019 m/s² (無補正 0.075)。真値プロキシが twist vx 微分のため、この d は
   twist 時間基準の相対遅延 — 絶対時刻 (pose 基準) では twist 自体の ~93 ms 遅延が加わる。
   旧 ACCEL_DELAY_MAP の 0.080 s はこの合成群遅れの粗い近似だった。

用途:
- オフライン評価 (observation_frame="localization_consistent"): kinematic_state の twist vx
  を +TWIST_VX_LAG_S 前進 + ×TWIST_VX_SCALE して pose と運動学整合させる
  (consistent_kinematic_frame)。縦系チェーン cmd→ax→vx→long が単一系統で閉じる。
- 本番シミュレータ: 下流 (planner 等) へ渡す localization 相当出力に実機の位相特性を
  反映する (twist_vx_localization_view / accel_localization_view。universe への実装は
  リリースプロセスに従い別途)。
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd

# pose–twist 窓回帰 (2026-07-17, R²=0.74) の同定値。
TWIST_VX_LAG_S = 0.093      # twist vx の pose (地図アンカー) に対する実効遅延 [s]
TWIST_VX_SCALE = 0.996      # v_pose / v_twist (twist が ~0.4% 過大)

# /localization/acceleration の観測モデル (2026-07-17, 60 datasets 中央値)。
# むだ時間は twist 時間基準の相対値 (上記 docstring 参照)。
ACCEL_LPF_TAU_S = 0.15      # 一次 LPF 時定数 [s]
ACCEL_DELAY_S = 0.02        # むだ時間 [s]

# N-step 評価の GT フレーム。
#   raw                     — 従来どおり localization 出力をそのまま GT に使う
#   localization_consistent — pose をアンカー (地図固定・変位の真値、指令→応答チェーンと
#                             整合したタイミング) とし、観測モデルの同定値で twist vx を
#                             補正する: +TWIST_VX_LAG_S 前進 + ×TWIST_VX_SCALE。
#                             raw フレームでは「ax/vx (twist 系) は減速を弱めろ / long
#                             (pose 系) は強めろ」という矛盾要求が生じ、縦系の構造改善が
#                             全て採用ゲートで棄却された (2026-07-17 根本原因分析、
#                             report 9 章)。
OBSERVATION_FRAMES = ("raw", "localization_consistent")


def normalize_observation_frame(value: Any, *, default: str = "raw") -> str:
    frame = default if value is None else str(value)
    if frame not in OBSERVATION_FRAMES:
        raise ValueError(
            f"未対応の observation_frame: {frame!r} (対応: {OBSERVATION_FRAMES})"
        )
    return frame


def consistent_kinematic_frame(
    df_kin: pd.DataFrame,
    *,
    lag_s: float = TWIST_VX_LAG_S,
    twist_scale: float = TWIST_VX_SCALE,
) -> pd.DataFrame:
    """kinematic_state を pose アンカーの整合フレームへ補正した copy を返す。

    - twist vx を +lag_s 前進させ (t+lag_s の値を取る)、pose に対する実効遅延を除去する
    - twist vx を ×twist_scale して pose 変位 (地図アンカー) と積分整合させる
    pose (x, y, yaw, pitch) は真値アンカーとして無補正。wz は twist 側だが遅延の証拠が
    vx (車輪速系) に限られるため補正しない (ジャイロ由来なら遅延源が異なる)。
    """
    df_kin = df_kin.sort_values("t_ns").reset_index(drop=True)
    out = df_kin.copy()
    if "vx" in out.columns:
        t = df_kin["t_ns"].to_numpy(dtype=np.float64) * 1e-9
        vx = df_kin["vx"].to_numpy(dtype=np.float64)
        out["vx"] = np.interp(t + lag_s, t, vx) * twist_scale
    return out


def first_order_lpf(values: np.ndarray, dt: float, tau: float) -> np.ndarray:
    """一次 IIR LPF (前進オイラー等価、EKF/実装系と同じ因果フィルタ)。"""
    values = np.asarray(values, dtype=float)
    if tau <= 0.0 or len(values) == 0:
        return values.copy()
    out = np.empty_like(values)
    out[0] = values[0]
    alpha = dt / (tau + dt)
    for i in range(1, len(values)):
        out[i] = out[i - 1] + alpha * (values[i] - out[i - 1])
    return out


def delay_hold(values: np.ndarray, dt: float, delay_s: float) -> np.ndarray:
    """先頭ホールドの純遅延 (グリッド丸め)。"""
    values = np.asarray(values, dtype=float)
    k = int(round(max(delay_s, 0.0) / dt))
    if k <= 0 or len(values) == 0:
        return values.copy()
    return np.concatenate([np.full(k, values[0]), values[: len(values) - k]])


def accel_localization_view(
    a_true: np.ndarray, dt: float,
    *, tau: float = ACCEL_LPF_TAU_S, delay_s: float = ACCEL_DELAY_S,
) -> np.ndarray:
    """真の加速度系列を /localization/acceleration 視点へ変換する (LPF + むだ時間)。

    delay_s の既定値は twist 時間基準の相対遅延。pose (絶対時刻) 基準へ変換する場合は
    TWIST_VX_LAG_S を加える。
    """
    return delay_hold(first_order_lpf(a_true, dt, tau), dt, delay_s)


def twist_vx_localization_view(
    v_true: np.ndarray, dt: float,
    *, lag_s: float = TWIST_VX_LAG_S, scale: float = TWIST_VX_SCALE,
) -> np.ndarray:
    """真の速度系列を localization twist vx 視点へ変換する (遅延 + 1/scale)。

    consistent_kinematic_frame の逆変換 (本番シミュレータの publisher 層向け):
    twist は真値より lag_s 遅れ、1/scale 倍過大に出る。
    """
    return delay_hold(np.asarray(v_true, dtype=float), dt, lag_s) / scale
