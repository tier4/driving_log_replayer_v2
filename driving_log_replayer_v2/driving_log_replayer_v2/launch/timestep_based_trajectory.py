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
from launch.substitutions import LaunchConfiguration

RECORD_TOPIC = """^/tf$\
|^/tf_static$\
|^/parameter_events$\
|^/diagnostics$\
|^/map/vector_map$\
|^/map/vector_map_marker$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/traffic_light_recognition/traffic_signals$\
|^/localization/kinematic_state$\
|^/localization/initialization_state$\
|^/localization/pose_with_covariance$\
|^/localization/acceleration$\
|^/system/processing_time_checker/metrics$\
|^/driving_log_replayer/.*$\
|^/api/.*/get/.*$\
|^/awapi/.*/get/.*$\
|^/planning/trajectory_generator/.*$\
|^/planning/trajectory$\
|^/planning/mission_planning/route\
"""

AUTOWARE_DISABLE = {
    "localization": "false",
    "perception": "false",
    "control": "false",
}

AUTOWARE_ARGS = {}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []
