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
"""

AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

AUTOWARE_ARGS = {}

NODE_PARAMS = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "evaluation_detection_topic_regex",
        default_value="""\
            |^/perception/object_recognition/detection/objects$\
            |^/perception/object_recognition/detection/centerpoint/objects$\
            |^/perception/object_recognition/detection/centerpoint/validation/objects$\
            |^/perception/object_recognition/detection/clustering/objects$\
            |^/perception/object_recognition/detection/detection_by_tracker/objects$\
            """,
    ),
    DeclareLaunchArgument(
        "evaluation_tracking_topic_regex",
        default_value="""\
            |^/perception/object_recognition/tracking/objects$\
            """,
    ),
    DeclareLaunchArgument(
        "evaluation_prediction_topic_regex",
        default_value="""\
            """,
    ),
]
