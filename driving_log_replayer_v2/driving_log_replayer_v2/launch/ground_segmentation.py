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
|^/diagnostics$"\
"""

AUTOWARE_DISABLE = {
    "sensing": "false",
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {"perception_mode": "lidar"}

NODE_PARAMS: dict[str, LaunchConfiguration] = {
    "evaluation_target_topic": LaunchConfiguration("evaluation_target_topic"),
}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "evaluation_target_topic",
        default_value="/perception/obstacle_segmentation/pointcloud",
    )
]

OPTIONAL_NODE_ARGS: list[DeclareLaunchArgument] = []
