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
|^/perception/.*/traffic_signals$\
|^/perception/traffic_light_recognition/traffic_signals/markers$\
|^/perception/traffic_light_recognition/.*/debug/rois/compressed$\
|^/driving_log_replayer_v2/.*\
"""

AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {}

NODE_PARAMS = {"map_path": LaunchConfiguration("map_path")}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []
