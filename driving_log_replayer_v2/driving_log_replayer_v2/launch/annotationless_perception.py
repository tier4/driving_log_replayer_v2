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
|^/system/processing_time_checker/metrics$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/perception/perception_online_evaluator/.*\
|^/driving_log_replayer/.*\
"""


AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {
    "use_perception_online_evaluator": "true",
}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []
