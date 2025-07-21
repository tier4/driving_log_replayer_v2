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
|^/tf_static$\
|^/diagnostics$\
|^/sensing/camera/.*\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/prediction/map_based_prediction/debug/pipeline_latency_ms$\
|^/perception/object_recognition/.*/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/detection/objects_before_filter$\
|^/sensing/.*detected_objects$\
|^/sensing/.*tracked_objects$\
"""

AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {}

NODE_PARAMS = {}

EVALUATION_DETECTION_TOPIC_REGEX = """\
^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/detection/centerpoint/objects$\
|^/perception/object_recognition/detection/centerpoint_short_range/objects$\
|^/perception/object_recognition/detection/clustering/objects$\
|^/perception/object_recognition/detection/detection_by_tracker/objects$\
|^/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects$\
|^/sensing/radar/detected_objects$\
"""

EVALUATION_TRACKING_TOPIC_REGEX = """\
^/perception/object_recognition/tracking/objects$\
"""

EVALUATION_PREDICTION_TOPIC_REGEX = """\
^/perception/object_recognition/objects$\
"""

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "evaluation_detection_topic_regex",
        default_value=EVALUATION_DETECTION_TOPIC_REGEX,
        description="Regex pattern for evaluation detection topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    ),
    DeclareLaunchArgument(
        "evaluation_tracking_topic_regex",
        default_value=EVALUATION_TRACKING_TOPIC_REGEX,
        description="Regex pattern for evaluation tracking topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    ),
    DeclareLaunchArgument(
        "evaluation_prediction_topic_regex",
        default_value=EVALUATION_PREDICTION_TOPIC_REGEX,
        description="Regex pattern for evaluation prediction topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    ),
    DeclareLaunchArgument(
        "analysis_max_distance",
        default_value="150",
        description="Maximum distance for analysis.",
    ),
    DeclareLaunchArgument(
        "analysis_distance_interval",
        default_value="150",
        description="Distance interval for analysis.",
    ),
]
