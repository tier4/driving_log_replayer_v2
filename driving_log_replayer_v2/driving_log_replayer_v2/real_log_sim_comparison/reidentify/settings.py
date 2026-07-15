"""Shared settings for the reidentify pipeline."""
from __future__ import annotations

DEFAULT_OUTPUT_DIR_NAME = "reidentify"
BASELINE_MODEL_NAME = "baseline"
TARGET_MODEL_NAME = "current"
TARGET_MODEL_TYPE = "delay_steer_acc_geared_for_diffusion_planner"

RESAMPLE_DT = 0.01
ROLLOUT_SUB_DT = 1.0 / 30.0
ROLLOUT_STRIDE = 5

# Sparse N-step horizons used by the cross-dataset optimization score.
HORIZONS = (10, 30, 70, 150, 300)

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
