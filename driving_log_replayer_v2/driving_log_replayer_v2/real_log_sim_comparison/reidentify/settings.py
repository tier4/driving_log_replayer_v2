"""Shared settings for the simplified reidentify pipeline."""
from __future__ import annotations

from pathlib import Path

DEFAULT_OUTPUT_DIR_NAME = "reidentify_v2"
DEFAULT_INPUT_PARAM = Path(
    "/home/kotaroyoshimoto/workspace/x2_e2e_44/src/description/vehicle/"
    "j6_gen2_description/j6_gen2_description/config/simulator_model.param.yaml"
)

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

SEARCH_DELAY_CANDIDATES = (0.10, 0.15, 0.20, 0.30, 0.40, 0.50)
SEARCH_SPACE_ACC = {
    "acc_time_constant": (0.20, 0.30),
    "debug_acc_scaling_factor": (0.80, 1.20),
}
SEARCH_SPACE_STEER = {
    "steer_time_constant": (0.05, 0.80),
    "debug_steer_scaling_factor": (0.75, 1.20),
    "k_us": (0.0, 0.05),
    "steer_dead_band": (0.0, 0.02),
    "steer_bias": (-0.01, 0.01),
}

REPORT_SAMPLE_DATASETS = 5
REPORT_MAX_PLOT_DATASETS = 3
REPORT_PLOT_WINDOW_S = 60.0
REPORT_VALID_ACC_TAU_BOUNDS = (0.05, 1.0)
REPORT_VALID_STEER_TAU_BOUNDS = (0.02, 1.0)

ROLLING_SMOOTH_WINDOW_S = 0.3
KINEMATIC_STEER_VX_MIN = 0.5
BAD_INTERVAL_MIN_S = 0.001
BAD_INTERVAL_MAX_S = 1.0
ROLL_OUT_CONTEXT = "reidentify.rollout"

TAIGA_DYN_DEFAULTS = {
    "mass": 6560.0,
    "inertia_z": 25868.2318,
    "cg_offset_x": -0.94323,
    "cornering_stiffness_front": 115830.0,
    "cornering_stiffness_rear": 535860.0,
    "vx_min_dyn": 1.0,
}
TAIGA_X_DEFAULTS = {
    "track_width": 1.754,
    "mass": 6560.0,
    "inertia_z": 25868.2318,
    "cg_offset_x": -0.94323,
    "max_accel": 2.3,
    "max_brake": 5.9,
    "wheel_radius": 0.3725,
    "taiga_x_fixed_dt": 1.0 / 1200.0,
}

RELEASE_MODEL_KEY = "delay_steer_acc_geared_for_diffusion_planner"
