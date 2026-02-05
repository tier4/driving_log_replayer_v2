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

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from driving_log_replayer_v2.launch.argument import add_use_case_arguments
from driving_log_replayer_v2.launch.rosbag import launch_bag_recorder
from driving_log_replayer_v2.launch.rosbag import launch_perception_reproducer
from driving_log_replayer_v2.launch.use_case import launch_autoware
from driving_log_replayer_v2.launch.use_case import launch_evaluator_node
from driving_log_replayer_v2.launch.use_case import launch_goal_pose_node
from driving_log_replayer_v2.launch.use_case import launch_initial_pose_node

RECORD_TOPIC = """^/tf$\
||^/tf_static$\
||^/parameter_events$\
||^/diagnostics$\
||^/map/vector_map_marker$\
||^/perception/object_recognition/objects$\
||^/perception/object_recognition/tracking/objects$\
||^/perception/traffic_light_recognition/traffic_signals$\
||^/system/v2x/virtual_traffic_light_states$\
||^/localization/kinematic_state$\
||^/localization/initialization_state$\
||^/localization/pose_with_covariance$\
||^/localization/acceleration$\
||^/planning/.*$\
||^/control/.*$\
||^/system/processing_time_checker/metrics$\
||^/driving_log_replayer/.*$\
||^/api/.*/get/.*$\
||^/api/operation_mode/state$\
||^/awapi/.*/get/.*\
"""

AUTOWARE_DISABLE = {}

AUTOWARE_ARGS = {
    "use_aeb_autoware_state_check": "false",
    "use_sim_time": "false",
    "enable_all_modules_auto_mode": "true",
}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []


def launch_engage_node(context: LaunchContext) -> list:
    conf = context.launch_configurations

    params = {
        "use_sim_time": False,
        "timeout_s": float(conf["timeout_s"]),
    }

    return [
        Node(
            package="driving_log_replayer_v2",
            namespace="/driving_log_replayer_v2",
            executable="engage_node.py",
            output="screen",
            name="engage_node",
            parameters=[params],
        ),
    ]


def launch_perception_reproducer_use_case() -> list:
    return [
        OpaqueFunction(function=add_use_case_arguments),
        OpaqueFunction(function=launch_autoware),
        OpaqueFunction(function=launch_perception_reproducer),
        OpaqueFunction(function=launch_bag_recorder),
        OpaqueFunction(function=launch_evaluator_node),
        OpaqueFunction(function=launch_initial_pose_node),
        OpaqueFunction(function=launch_goal_pose_node),
        OpaqueFunction(function=launch_engage_node),
    ]
