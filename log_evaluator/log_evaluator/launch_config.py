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

from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue

ANNOTATIONLESS_PERCEPTION_RECORD_TOPIC = """^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/diagnostic/perception_online_evaluator/.*\
"""

ANNOTATIONLESS_PERCEPTION_AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

ANNOTATIONLESS_PERCEPTION_AUTOWARE_ARGS = {
    "use_perception_online_evaluator": "true",
}

ANNOTATIONLESS_PERCEPTION_NODE_PARAMS = {
    "annotationless_threshold_file": LaunchConfiguration("annotationless_threshold_file"),
    # annotationless_pass_range is json format string. Avoid interpreting json format strings as dict
    # [ERROR] [launch]: Caught exception in launch (see debug for traceback): Allowed value types are bytes, bool, int, float, str, Sequence[bool], Sequence[int], Sequence[float], Sequence[str].
    # Got <class 'dict'>.If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
    "annotationless_pass_range": ParameterValue(
        LaunchConfiguration("annotationless_pass_range"),
        value_type=str,
    ),
}

AR_TAG_BASED_LOCALIZER_RECORD_TOPIC = """^/tf$\
|^/diagnostics$"\
|^/localization/kinematic_state$\
"""

AR_TAG_BASED_LOCALIZER_AUTOWARE_DISABLE = {
    "perception": "false",
    "planning": "false",
    "control": "false",
}

AR_TAG_BASED_LOCALIZER_AUTOWARE_ARGS = {
    "pose_source": "artag",
    "twist_source": "gyro_odom",
}

AR_TAG_BASED_LOCALIZER_NODE_PARAMS = {}

EAGLEYE_RECORD_TOPIC = """^/tf$\
|^/diagnostics$"\
|^/localization/kinematic_state$\
|^/localization/pose_estimator/pose$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
"""

EAGLEYE_AUTOWARE_DISABLE = {"perception": "false", "planning": "false", "control": "false"}

EAGLEYE_AUTOWARE_ARGS = {
    "pose_source": "eagleye",
    "twist_source": "eagleye",
}

EAGLEYE_NODE_PARAMS = {}

LOCALIZATION_RECORD_TOPIC = """^/tf$\
|^/diagnostics$\
|^/localization/pose_estimator/transform_probability$\
|^/localization/pose_estimator/nearest_voxel_transformation_likelihood$\
|^/localization/pose_estimator/pose$\
|^/localization/kinematic_state$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
|^/log_evaluator/.*\
"""

LOCALIZATION_AUTOWARE_DISABLE = {
    "perception": "false",
    "planning": "false",
    "control": "false",
}

LOCALIZATION_AUTOWARE_ARGS = {
    "pose_source": "ndt",
    "twist_source": "gyro_odom",
}

LOCALIZATION_NODE_PARAMS = {}

OBSTACLE_SEGMENTATION_RECORD_TOPIC = """^/tf$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/planning/scenario_planning/trajectory$\
|^/planning/scenario_planning/status/stop_reasons$\
|^/log_evaluator/.*\
"""

OBSTACLE_SEGMENTATION_AUTOWARE_DISABLE = {
    "localization": "false",
    "control": "false",
}

OBSTACLE_SEGMENTATION_AUTOWARE_ARGS = {
    "scenario_simulation": "true",
}

OBSTACLE_SEGMENTATION_NODE_PARAMS = {
    "vehicle_model": LaunchConfiguration("vehicle_model"),
    "map_path": LaunchConfiguration("map_path"),
}

PERCEPTION_2D_RECORD_TOPIC = """^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/sensing/camera/.*\
|^/log_evaluator/.*\
"""

PERCEPTION_2D_AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

PERCEPTION_2D_AUTOWARE_ARGS = {
    "perception_mode": "camera_lidar_fusion",
}

PERCEPTION_2D_NODE_PARAMS = {}

PERCEPTION_RECORD_TOPIC = """^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*\
|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$\
|^/log_evaluator/.*\
|^/sensing/camera/.*\
"""

PERCEPTION_AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

PERCEPTION_AUTOWARE_ARGS = {}

PERCEPTION_NODE_PARAMS = {}

PERFORMANCE_DIAG_RECORD_TOPIC = """^/tf$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/diagnostics$\
|^/sensing/lidar/.*/blockage_diag/debug/blockage_mask_image$\
|^/sensing/lidar/.*/pointcloud_raw_ex$\
|^/log_evaluator/.*\
"""

PERFORMANCE_DIAG_AUTOWARE_DISABLE = {
    "perception": "false",
    "planning": "false",
    "control": "false",
}


PERFORMANCE_DIAG_AUTOWARE_ARGS = {}

PERFORMANCE_DIAG_NODE_PARAMS = {}

TRAFFIC_LIGHT_RECORD_TOPIC = """^/tf$\
|^/sensing/camera/camera[67]/image_raw/compressed$\
|^/perception/.*/traffic_signals$\
|^/log_evaluator/.*\
"""

TRAFFIC_LIGHT_AUTOWARE_DISABLE = {
    "localization": "false",
    "planning": "false",
    "control": "false",
}

TRAFFIC_LIGHT_AUTOWARE_ARGS = {}

TRAFFIC_LIGHT_NODE_PARAMS = {"map_path": LaunchConfiguration("map_path")}

YABLOC_RECORD_TOPIC = """^/tf$\
|^/diagnostics$\
|^/localization/pose_estimator/pose$\
|^/localization/kinematic_state$\
|^/localization/util/downsample/pointcloud$\
|^/localization/pose_estimator/points_aligned$\
"""

YABLOC_AUTOWARE_DISABLE = {
    "perception": "false",
    "planning": "false",
    "control": "false",
}

YABLOC_AUTOWARE_ARGS = {
    "pose_source": "yabloc",
    "twist_source": "gyro_odom",
}

YABLOC_NODE_PARAMS = {}

log_evaluator_config = {
    "annotationless_perception": {
        "record": ANNOTATIONLESS_PERCEPTION_RECORD_TOPIC,
        "disable": ANNOTATIONLESS_PERCEPTION_AUTOWARE_DISABLE,
        "autoware": ANNOTATIONLESS_PERCEPTION_AUTOWARE_ARGS,
        "node": ANNOTATIONLESS_PERCEPTION_NODE_PARAMS,
    },
    "ar_tag_based_localizer": {
        "record": AR_TAG_BASED_LOCALIZER_RECORD_TOPIC,
        "disable": AR_TAG_BASED_LOCALIZER_AUTOWARE_DISABLE,
        "autoware": AR_TAG_BASED_LOCALIZER_AUTOWARE_ARGS,
        "node": AR_TAG_BASED_LOCALIZER_NODE_PARAMS,
    },
    "eagleye": {
        "record": EAGLEYE_RECORD_TOPIC,
        "disable": EAGLEYE_AUTOWARE_DISABLE,
        "autoware": EAGLEYE_AUTOWARE_ARGS,
        "node": EAGLEYE_NODE_PARAMS,
    },
    "localization": {
        "record": LOCALIZATION_RECORD_TOPIC,
        "disable": LOCALIZATION_AUTOWARE_DISABLE,
        "autoware": LOCALIZATION_AUTOWARE_ARGS,
        "node": LOCALIZATION_NODE_PARAMS,
    },
    "obstacle_segmentation": {
        "record": OBSTACLE_SEGMENTATION_RECORD_TOPIC,
        "disable": OBSTACLE_SEGMENTATION_AUTOWARE_DISABLE,
        "autoware": OBSTACLE_SEGMENTATION_AUTOWARE_ARGS,
        "node": OBSTACLE_SEGMENTATION_NODE_PARAMS,
    },
    "perception_2d": {
        "record": PERCEPTION_2D_RECORD_TOPIC,
        "disable": PERCEPTION_2D_AUTOWARE_DISABLE,
        "autoware": PERCEPTION_AUTOWARE_ARGS,
        "node": PERCEPTION_2D_NODE_PARAMS,
    },
    "perception": {
        "record": PERCEPTION_RECORD_TOPIC,
        "disable": PERCEPTION_AUTOWARE_DISABLE,
        "autoware": PERCEPTION_AUTOWARE_ARGS,
        "node": PERCEPTION_NODE_PARAMS,
    },
    "performance_diag": {
        "record": PERFORMANCE_DIAG_RECORD_TOPIC,
        "disable": PERFORMANCE_DIAG_AUTOWARE_DISABLE,
        "autoware": PERFORMANCE_DIAG_AUTOWARE_ARGS,
        "node": PERFORMANCE_DIAG_NODE_PARAMS,
    },
    "traffic_light": {
        "record": TRAFFIC_LIGHT_RECORD_TOPIC,
        "disable": TRAFFIC_LIGHT_AUTOWARE_DISABLE,
        "autoware": TRAFFIC_LIGHT_AUTOWARE_ARGS,
        "node": TRAFFIC_LIGHT_NODE_PARAMS,
    },
    "yabloc": {
        "record": YABLOC_RECORD_TOPIC,
        "disable": YABLOC_AUTOWARE_DISABLE,
        "autoware": YABLOC_AUTOWARE_ARGS,
        "node": YABLOC_NODE_PARAMS,
    },
}
