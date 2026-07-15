"""Shared settings for the reidentify pipeline."""
from __future__ import annotations

DEFAULT_OUTPUT_DIR_NAME = "reidentify"
BASELINE_MODEL_NAME = "baseline"
TARGET_MODEL_NAME = "current"
# フィット結果を公開するときの表示名 (metrics.csv の model 列・レポート)。
TUNED_MODEL_DISPLAY_NAME = "tuned"
TARGET_MODEL_TYPE = "delay_steer_acc_geared_for_diffusion_planner"

RESAMPLE_DT = 0.01
ROLLOUT_SUB_DT = 1.0 / 30.0
ROLLOUT_STRIDE = 5

# Sparse N-step horizons used by the cross-dataset optimization score.
HORIZONS = (10, 30, 70, 150, 300)

# robust_score のアクチュエータ (steer/ax) 項に使う horizon。HORIZONS のサブセットであること
# (rollout の追加評価なしで済む。tests/test_multi_agg.py でアサート)。
# steer/ax の open-loop 誤差は N≈20 (steer) / N≈9 (ax) で定常値に飽和するプラトー特性を持つため、
# 過渡 (N=10: 時定数/むだ時間の情報) とプラトー (N=30: 定常ゲイン/バイアスの情報) の 2 点で足りる。
ACT_SCORE_HORIZONS = (10, 30)

MIN_FIT_SAMPLES = 50
MIN_K_US_SAMPLES = 10

LONG_VX_MIN = 0.5
LONG_DA_THRESH = 0.15

STEER_VX_MIN = 0.5
STEER_DSTEER_MIN = 0.001
STEER_STRAIGHT_VX_MIN = 3.0
STEER_STRAIGHT_WZ_MAX = 0.005
STEER_BIAS_MIN_SAMPLES = 20
STEER_CMD_ENERGY_MIN = 1e-5
STEER_CLIP_RAD = 0.8

ROLLING_SMOOTH_WINDOW_S = 0.3
KINEMATIC_STEER_VX_MIN = 0.5
BAD_INTERVAL_MIN_S = 0.001
BAD_INTERVAL_MAX_S = 1.0
ROLL_OUT_CONTEXT = "reidentify.rollout"

RELEASE_MODEL_KEY = TARGET_MODEL_TYPE

# Differentiated velocity source for longitudinal acceleration fitting.
ACCEL_SOURCE = "kinematic_savgol"
