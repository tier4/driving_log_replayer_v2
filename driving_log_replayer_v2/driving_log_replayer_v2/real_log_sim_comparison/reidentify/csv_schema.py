"""Raw rosbag と ROS 非依存の同定処理をつなぐ CSV 契約。"""
from __future__ import annotations

CACHE_NAME = "reidentify_cache.csv"

# 同定に必要な raw bag 上の 7 topic。キャッシュでは短い
# signal タグに変換し、以降のステップを topic 名の変更から切り離す。
SIGNAL_TOPICS: dict[str, str] = {
    "mode": "/system/operation_mode/state",
    "velocity": "/vehicle/status/velocity_status",
    "steering": "/vehicle/status/steering_status",
    "gear": "/vehicle/status/gear_status",
    "kinematic": "/localization/kinematic_state",
    "accel": "/localization/acceleration",
    "cmd": "/control/command/control_cmd",
}

# topic タグ -> タグ固有列。t_ns は全タグ共通。
SIGNAL_COLUMNS: dict[str, list[str]] = {
    "mode": ["mode"],
    "velocity": ["lon_vel"],
    "steering": ["steer"],
    "kinematic": ["x", "y", "yaw", "pitch", "vx", "vy", "wz"],
    "accel": ["accel"],
    "cmd": ["cmd_vel", "cmd_accel", "cmd_steer"],
    "gear": ["gear"],
}
ALL_VALUE_COLUMNS = sorted({c for cols in SIGNAL_COLUMNS.values() for c in cols})
CSV_COLUMNS = ["t_ns", "topic", *ALL_VALUE_COLUMNS]
