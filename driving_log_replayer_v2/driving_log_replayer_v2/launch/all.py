# Copyright (c) 2025 TIER IV.inc
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

RECORD_TOPIC = """^/localization/util/downsample/pointcloud$\
|^/localization/pose_twist_fusion_filter/biased_pose_with_covariance$\
"""

AUTOWARE_DISABLE = {}

AUTOWARE_ARGS = {
    "pose_source": "ndt",
    "twist_source": "gyro_odom",
}

NODE_PARAMS = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []
