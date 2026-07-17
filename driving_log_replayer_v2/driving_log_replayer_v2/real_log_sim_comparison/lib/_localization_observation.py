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

import numpy as np

# pose–twist 窓回帰 (2026-07-17, R²=0.73) の同定値。
POSE_LAG_S = 0.093          # pose の twist に対する実効遅延 [s]
TWIST_VX_SCALE = 0.996      # v_pose / v_twist (twist が ~0.4% 過大)

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
