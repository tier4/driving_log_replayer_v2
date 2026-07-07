"""物理同定の閾値定数 (ROS フリー)。

SSOT は `lib/_physical_validity.py` だが、そのモジュールは先頭で `lib._io`
(rosbag2_py/rclpy 依存) を import するため、定数だけを import しても
reidentify の ROS フリー方針が壊れてしまう。ここでは単純な閾値定数
(同定パラメータそのものではない、乖離リスクの小さい値) のみを複製する。
`lib/_physical_validity.py` 側でこれらの値を変更した場合は追随させること。

wheelbase は `lib/_params_utils.load_sim_params()`(ROS フリー、
j6_gen2_description/config を実読込する真の SSOT) から取得するため
ここには含めない。
"""
from __future__ import annotations

VX_MIN_CURVE = 1.5  # [m/s] k_us 定常旋回フィルタの速度下限
WZ_MIN = 0.02  # [rad/s] k_us 定常旋回フィルタのヨーレート下限
DWZ_MAX = 0.30  # [rad/s^2] k_us 定常旋回フィルタのヨー角加速度上限
