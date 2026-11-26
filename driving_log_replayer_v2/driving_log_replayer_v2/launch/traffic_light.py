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
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "enable_analysis",
        default_value="false",
    ),
    DeclareLaunchArgument(
        "traffic_light_runner",
        default_value="logging_simulator",
        description="logging_simulator (default) runs Autoware and ros2 bag play / record. bag2bag runs traffic_light_pipeline_bag2bag_runner in place of them.",
    ),
    DeclareLaunchArgument(
        "bag2bag_ml_model_path",
        default_value="",
        description="--ml-model-path of the bag2bag runner. Required when traffic_light_runner is bag2bag.",
    ),
    DeclareLaunchArgument(
        "bag2bag_camera_launch",
        default_value="",
        description="Launch file whose camera_namespaces default gives --camera of the bag2bag runner. Required when traffic_light_runner is bag2bag.",
    ),
    DeclareLaunchArgument(
        "bag2bag_recognition_config",
        default_value="",
        description="--recognition-config of the bag2bag runner. Required when traffic_light_runner is bag2bag.",
    ),
    DeclareLaunchArgument(
        "bag2bag_fusion_config",
        default_value="",
        description="--fusion-config of the bag2bag runner. Required when traffic_light_runner is bag2bag.",
    ),
]
