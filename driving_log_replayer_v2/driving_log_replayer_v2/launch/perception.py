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

from pathlib import Path

from launch.actions import DeclareLaunchArgument
from pydantic import ValidationError
import yaml

from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.scenario import load_scenario

RECORD_TOPIC = """^/tf$\
|^/tf_static$\
|^/diagnostics$\
|^/awapi/autoware/get/status$\
|^/sensing/camera/.*/compressed$\
|^/sensing/camera/.*/camera_info$\
|^/sensing/lidar/concatenated/pointcloud$\
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
|^/diagnostics/.$\
"""


def autoware_disable(conf: dict) -> dict[str, str]:
    default = {
        "localization": "false",
        "planning": "false",
        "control": "false",
    }

    try:
        scenario = load_scenario(Path(conf["scenario_path"]), PerceptionScenario)
    except (FileNotFoundError, PermissionError, yaml.YAMLError, ValidationError, KeyError):
        return default
    if scenario.Evaluation.Conditions.stop_reason_criterion is not None:
        return {
            "localization": "false",
            "planning": "true",
            "control": "true",
        }
    return default


AUTOWARE_DISABLE = autoware_disable

AUTOWARE_ARGS = {}

NODE_PARAMS = {}

EVALUATION_DETECTION_TOPIC_REGEX = """\
^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/detection/centerpoint/objects$\
|^/perception/object_recognition/detection/centerpoint_short_range/objects$\
|^/perception/object_recognition/detection/bevfusion/objects$\
|^/perception/object_recognition/camera_only/objects$\
"""

# removed following topics to reduce calculation cost
# |^/perception/object_recognition/detection/clustering/objects$\
# |^/perception/object_recognition/detection/detection_by_tracker/objects$\
# |^/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects$\

EVALUATION_TRACKING_TOPIC_REGEX = """\
^/perception/object_recognition/tracking/objects$\
|^/sensing/radar/front_center/tracked_objects$\
"""

# EVALUATION_PREDICTION_TOPIC_REGEX = """\
# ^/perception/object_recognition/objects$\
# """
# skip prediction evaluation for now
EVALUATION_PREDICTION_TOPIC_REGEX = ""

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
    DeclareLaunchArgument(
        "publish_topic_from_rosbag",
        default_value="",
        description="The topic to publish in rosbag before play rosbag. Using comma separated string.",
    ),
]
