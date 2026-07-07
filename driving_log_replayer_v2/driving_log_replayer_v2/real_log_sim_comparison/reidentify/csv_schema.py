"""reidentify_cache.csv の列スキーマ (extract.py / load_data.py 共有、ROS 非依存)。

`extract.py` は rclpy/rosbag2_py に依存するため、load_data.py がそれを import
してしまうと ROS フリーの前提が壊れる。両者が参照する定数だけをこの薄いモジュール
に集約し、依存の混入を防ぐ。
"""
from __future__ import annotations

CACHE_NAME = "reidentify_cache.csv"

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
