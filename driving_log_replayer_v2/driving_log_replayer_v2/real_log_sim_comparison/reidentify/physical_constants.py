"""Physical thresholds used by the dedicated steering fit."""
from __future__ import annotations

GRAVITY = 9.80665  # [m/s^2] 標準重力 (slope_accx = +g·sin(pitch)、pitch 符号検証済み 2026-07-17)

VX_MIN_CURVE = 1.5  # [m/s] k_us 定常旋回フィルタの速度下限
WZ_MIN = 0.02  # [rad/s] k_us 定常旋回フィルタのヨーレート下限
DWZ_MAX = 0.30  # [rad/s^2] k_us 定常旋回フィルタのヨー角加速度上限
