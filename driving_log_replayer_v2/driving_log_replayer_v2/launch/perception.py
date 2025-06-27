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
|^/map/vector_map$\
|^/map/pointcloud_map$\
|^/sensing/camera/.*\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/prediction/map_based_prediction/debug/pipeline_latency_ms$\
|^/perception/object_recognition/.*/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/detection/objects_before_filter$\
|^/perception/occupancy_grid_map/map$\
|^/perception/traffic_light_recognition/traffic_signals$\
|^/awapi/autoware/get/status$\
|^/awapi/tmp/virtual_traffic_light_states$\
|^/planning/planning_factors/obstacle_stop$\
|^/planning/mission_planning/route$\
|^/planning/scenario_planning/scenario$\
|^/planning/scenario_planning/parking/costmap_generator/occupancy_grid$\
|^/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection/ego_ttc$\
|^/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection/object_ttc$\
|^/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection$\
|^/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection/decision_state$\
|^/localization/initialization_state$\
|^/api/localization/initialization_state$\
|^/api/operation_mode/state$\
|^/api/planning/velocity_factors$\
|^/api/planning/steering_factors$\
|^/api/external/get/rtc_auto_mode$\
|^/api/external/get/rtc_status$\
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
|^/perception/object_recognition/detection/centerpoint/validation/objects$\
|^/perception/object_recognition/detection/clustering/objects$\
|^/perception/object_recognition/detection/detection_by_tracker/objects$\
|^/perception/object_recognition/detection/objects_before_filter$\
"""

EVALUATION_TRACKING_TOPIC_REGEX = """\
^/perception/object_recognition/tracking/objects$\
"""

EVALUATION_PREDICTION_TOPIC_REGEX = """\
^/perception/object_recognition/objects$\
"""

EVALUATION_FP_VALIDATION_TOPIC_REGEX = """\
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
        "evaluation_fp_validation_topic_regex",
        default_value=EVALUATION_FP_VALIDATION_TOPIC_REGEX,
        description="Regex pattern for evaluation fp_validation topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    ),
    DeclareLaunchArgument(
        "max_distance",
        default_value="100",
        description="Maximum distance for analysis. Default is 100m.",
    ),
    DeclareLaunchArgument(
        "distance_interval",
        default_value="10",
        description="Distance interval for analysis. Default is 10m.",
    ),
]
