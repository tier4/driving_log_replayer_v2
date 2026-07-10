# Copyright (c) 2026 TIER IV.inc
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

AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "evaluation_pointcloud_topic",
        default_value="/perception/obstacle_segmentation/pointcloud",
        description="ROS pointcloud topic name to evaluate. If you do not want to evaluate pointcloud topic, set '' or 'None'.",
    ),
    DeclareLaunchArgument(
        "evaluation_object_topic",
        default_value="/perception/object_recognition/objects",
        description="ROS object topic name to evaluate. If you do not want to evaluate object topic, set '' or 'None'.",
    ),
    DeclareLaunchArgument(
        "enable_analysis",
        default_value="false",
        description="Enable analysis.",
    ),
]
