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

RECORD_TOPIC = """^/tf$\
|^/localization/kinematic_state$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/objects$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/perception/object_recognition/detection/rois(11|10|[0-9])$\
|^/perception/traffic_light_recognition/camera(11|10|[0-9])/detection/rois$\
|^/perception/traffic_light_recognition/camera(11|10|[0-9])/detection/rough/rois$\
|^/perception/traffic_light_recognition/camera(11|10|[0-9])/classification/traffic_signals$\
|^/perception/traffic_light_recognition/traffic_signals$\
|^/tf_static$\
|^/diagnostics$\
|^/planning/scenario_planning/lane_driving/behavior_planning/path$\
|^/planning/scenario_planning/trajectory$\
|^/.*/virtual_wall/.*$\
|^/.*/path_candidate/.*\
"""

AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {
    "use_perception_online_evaluator": "true",
}

NODE_PARAMS = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []
