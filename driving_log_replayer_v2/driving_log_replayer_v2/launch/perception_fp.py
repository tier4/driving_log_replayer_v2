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

RECORD_TOPIC = """^/tf$\
|^/tf_static$\
|^/diagnostics$\
|^/system/processing_time_checker/metrics$\
|^/awapi/autoware/get/status$\
|^/sensing/camera/.*/compressed$\
|^/sensing/camera/.*/camera_info$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/prediction/map_based_prediction/debug/pipeline_latency_ms$\
|^/perception/object_recognition/.*/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/detection/rois[0-9]+$\
|^/perception/object_recognition/detection/objects_before_filter$\
|^/sensing/.*detected_objects$\
|^/sensing/.*tracked_objects$\
|^/map/vector_map_marker$\
|^/localization/kinematic_state$\
"""


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
