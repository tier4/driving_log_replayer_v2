# Copyright (c) 2024 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

RECORD_TOPIC = """^/tf$\
|^/diagnostics$\
|^/localization/pose_estimator/exe_time_ms$\
|^/localization/pose_estimator/iteration_num$\
|^/localization/pose_estimator/transform_probability$\
|^/localization/pose_estimator/nearest_voxel_transformation_likelihood$\
|^/localization/pose_estimator/initial_to_result_relative_pose$\
|^/localization/pose_estimator/ndt_marker$\
|^/localization/pose_estimator/pose$\
|^/localization/pose_estimator/pose_with_covariance$\
|^/localization/kinematic_state$\
|^/localization/acceleration$\
|^/localization/reference_kinematic_state$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
|^/driving_log_replayer/.*\
"""

AUTOWARE_DISABLE = {
    "perception": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {
    "pose_source": "ndt",
    "twist_source": "gyro_odom",
}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "enable_analysis",
        default_value="true",
        description="Enable analysis.",
    ),
]

OPTIONAL_NODE_ARGS: list[DeclareLaunchArgument] = []
