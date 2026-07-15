"""Shared settings for the reidentify pipeline."""
from __future__ import annotations

DEFAULT_OUTPUT_DIR_NAME = "reidentify"
BASELINE_MODEL_NAME = "baseline"
TARGET_MODEL_NAME = "current"
TARGET_MODEL_TYPE = "delay_steer_acc_geared_for_diffusion_planner"

RESAMPLE_DT = 0.01
ROLLOUT_SUB_DT = 1.0 / 30.0
ROLLOUT_STRIDE = 5

DEFAULT_WHEELBASE = 4.76012

MIN_FIT_SAMPLES = 50
MIN_K_US_SAMPLES = 10

LONG_VX_MIN = 0.5
LONG_DA_THRESH = 0.15
LONG_TAU_BOUNDS = (0.01, 5.0)
LONG_DELAY_GRID = tuple(i / 100.0 for i in range(0, 31))
LONG_RESULT_TAU_BOUNDS = (0.1, 3.0)
LONG_RESULT_DELAY_BOUNDS = (0.0, 0.3)
ACC_SCALE_BOUNDS = (0.8, 1.2)

STEER_VX_MIN = 0.5
STEER_DSTEER_MIN = 0.001
STEER_TAU_BOUNDS = (0.01, 2.0)
STEER_DELAY_GRID = tuple(i / 100.0 for i in range(0, 16))
STEER_RESULT_TAU_BOUNDS = (0.05, 0.8)
STEER_RESULT_DELAY_BOUNDS = (0.0, 0.15)
STEER_SCALE_BOUNDS = (0.8, 1.2)
STEER_STRAIGHT_VX_MIN = 3.0
STEER_STRAIGHT_WZ_MAX = 0.005
STEER_BIAS_MIN_SAMPLES = 20
STEER_CMD_ENERGY_MIN = 1e-5
STEER_DEFAULT_DEAD_BAND = 0.001
STEER_DEFAULT_RATE_LIM = 5.0
STEER_CLIP_RAD = 0.8

K_US_CLIP = 0.05

SEARCH_DELAY_CANDIDATES = (0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50)
SEARCH_SPACE_ACC = {
    "acc_time_constant": LONG_RESULT_TAU_BOUNDS,
    "debug_acc_scaling_factor": (0.80, 1.20),
}

SEARCH_SPACE_STEER = {
    "steer_time_constant": (0.05, 0.80),
    "debug_steer_scaling_factor": (0.75, 1.20),
    "k_us": (0.0, 0.05),
    "steer_dead_band": (0.0, 0.02),
    "steer_bias": (-0.01, 0.01),
}

ROLLING_SMOOTH_WINDOW_S = 0.3
KINEMATIC_STEER_VX_MIN = 0.5
BAD_INTERVAL_MIN_S = 0.001
BAD_INTERVAL_MAX_S = 1.0
ROLL_OUT_CONTEXT = "reidentify.rollout"

RELEASE_MODEL_KEY = TARGET_MODEL_TYPE

# Differentiated velocity source for longitudinal acceleration fitting.
ACCEL_SOURCE = "kinematic_savgol"
