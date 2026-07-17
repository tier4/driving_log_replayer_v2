"""localization EKF の観測モデル — 真値状態から localization 視点の出力への変換。

実データ同定 (2026-07-17, openloop_j6_16_onwards dev split):

1. pose–twist 位相差/スケール差 (窓回帰 n=14,313, R²=0.73):
     Δ(pose変位 − ∫twist vx dt) = +93 ms·Δv − 0.60%·path (+1.8 cm)
   pose は twist に対し実効 ~93 ms 遅延し、twist vx は pose 距離より ~0.4–0.6% 過大。
   ピッチ投影説は棄却 (Δpitch 係数 ≈ 0.07 m ≈ 0)。この不整合が N-step 評価で
   「ax は減速を弱めろ (twist 系) / long は強めろ (pose 系)」という矛盾要求を生み、
   縦系の構造改善 (brake 分離・bite) がゲートで棄却され続けた根本原因。

2. /localization/acceleration の観測特性 (43 datasets, 全データセットが同一最適点):
     a_topic ≈ delay(LPF1(a_true; τ=0.15 s); d=0.02 s)
   残差 RMS 0.019 m/s² (無補正 0.070)。旧 ACCEL_DELAY_MAP の 0.080 s は
   この LPF の実効群遅れの粗い近似だった。

用途:
- オフライン評価: sim 出力を localization 視点へ変換してから GT トピックと比較する
  (縦系チェーン cmd→ax→vx→long を単一系統で閉じる)。ax の GT には twist 微分でなく
  /localization/acceleration 生トピックを使い、モデル側を accel_localization_view で
  位相整合させる、という構成が可能になる。
- 本番シミュレータ: 下流 (planner 等) へ渡す localization 相当出力に実機の位相特性を
  反映する (universe への実装はリリースプロセスに従い別途)。
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pandas as pd

# pose–twist 窓回帰 (2026-07-17, R²=0.73) の同定値。
POSE_LAG_S = 0.093          # pose の twist に対する実効遅延 [s]
TWIST_VX_SCALE = 0.996      # v_pose / v_twist (twist が ~0.4% 過大)

# N-step 評価の GT フレーム。
#   raw                     — 従来どおり localization 出力をそのまま GT に使う
#   localization_consistent — pose をアンカー (地図固定・変位の真値) とし、観測モデルの
#                             同定値で pose–twist を運動学的に整合させる:
#                             pose 系列を +POSE_LAG_S 前進、twist vx を ×TWIST_VX_SCALE。
#                             raw フレームでは「ax/vx (twist 系) は減速を弱めろ / long (pose 系)
#                             は強めろ」という矛盾要求が生じ、縦系の構造改善が全て採用ゲートで
#                             棄却された (2026-07-17 根本原因分析、report 9 章)。
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
    lag_s: float = POSE_LAG_S,
    twist_scale: float = TWIST_VX_SCALE,
) -> pd.DataFrame:
    """kinematic_state を pose アンカーの整合フレームへ補正した copy を返す。

    - pose 系列 (x, y, yaw, pitch) を +lag_s 前進させ、EKF pose の実効遅延を除去する
    - twist vx を ×twist_scale して pose 変位 (地図アンカー) と積分整合させる
    wz はジャイロ由来でどちらの系統誤差も持たないため補正しない。yaw は unwrap して
    補間する (±π 跨ぎ対策) ため、返り値の yaw は unwrap 済み連続系列になる。
    """
    df_kin = df_kin.sort_values("t_ns").reset_index(drop=True)
    out = df_kin.copy()
    t = df_kin["t_ns"].to_numpy(dtype=np.float64) * 1e-9
    t_q = t + lag_s
    for col in ("x", "y", "pitch"):
        if col in out.columns:
            out[col] = np.interp(t_q, t, df_kin[col].to_numpy(dtype=np.float64))
    if "yaw" in out.columns:
        yaw = np.unwrap(df_kin["yaw"].to_numpy(dtype=np.float64))
        out["yaw"] = np.interp(t_q, t, yaw)
    if "vx" in out.columns:
        out["vx"] = df_kin["vx"].to_numpy(dtype=np.float64) * twist_scale
    return out

# /localization/acceleration の観測モデル (2026-07-17, 43 datasets 一致)。
ACCEL_LPF_TAU_S = 0.15      # 一次 LPF 時定数 [s]
ACCEL_DELAY_S = 0.02        # むだ時間 [s]


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
    """真の加速度系列を /localization/acceleration 視点へ変換する (LPF + むだ時間)。"""
    return delay_hold(first_order_lpf(a_true, dt, tau), dt, delay_s)


def pose_displacement_localization_view(
    ds_true: np.ndarray, t: np.ndarray,
    *, lag_s: float = POSE_LAG_S, scale: float = TWIST_VX_SCALE,
) -> np.ndarray:
    """真値 (twist 系) の累積変位系列を pose 視点へ変換する (遅延 + スケール)。

    ds_true: 累積縦変位 [m] (例: ∫vx dt)。t: 対応する時刻 [s] (単調増加、非一様可)。
    """
    ds_true = np.asarray(ds_true, dtype=float)
    t = np.asarray(t, dtype=float)
    return scale * np.interp(t - lag_s, t, ds_true, left=ds_true[0])
