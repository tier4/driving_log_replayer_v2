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

AUTOWARE_DISABLE = {
    "localization": "false",
}

AUTOWARE_ARGS = {"use_aeb_autoware_state_check": "false"}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "gt_source_mode",
        default_value="kinematic_state",
        description="Open-loop GT source mode for planner evaluation. kinematic_state (default) or gt_trajectory.",
    ),
    DeclareLaunchArgument(
        "gt_trajectory_topic",
        default_value="/planning/ground_truth_trajectory",
        description="GT trajectory topic used when gt_source_mode is gt_trajectory.",
    ),
    DeclareLaunchArgument(
        "gt_sync_tolerance_ms",
        default_value="200.0",
        description="GT trajectory alignment tolerance in milliseconds for gt_trajectory mode.",
    ),
]
