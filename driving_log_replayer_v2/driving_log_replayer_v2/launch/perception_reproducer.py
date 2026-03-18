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

import json

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from driving_log_replayer_v2.launch.argument import add_use_case_arguments
from driving_log_replayer_v2.launch.rosbag import launch_bag_recorder
from driving_log_replayer_v2.launch.rosbag import launch_perception_reproducer
from driving_log_replayer_v2.launch.use_case import launch_autoware
from driving_log_replayer_v2.launch.use_case import launch_evaluator_node
from driving_log_replayer_v2.launch.use_case import launch_goal_pose_node
from driving_log_replayer_v2.launch.use_case import launch_initial_pose_node
from driving_log_replayer_v2.pose import offset_pose_dict_forward
from driving_log_replayer_v2.pose import pose_str_to_dict

RECORD_TOPIC = """^/tf$\
|^/tf_static$\
|^/parameter_events$\
|^/diagnostics$\
|^/map/vector_map_marker$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/traffic_light_recognition/traffic_signals$\
|^/system/v2x/virtual_traffic_light_states$\
|^/localization/kinematic_state$\
|^/localization/initialization_state$\
|^/localization/pose_with_covariance$\
|^/localization/acceleration$\
|^/planning/.*$\
|^/control/.*$\
|^/system/processing_time_checker/metrics$\
|^/driving_log_replayer/.*$\
|^/api/.*/get/.*$\
|^/api/operation_mode/state$\
|^/awapi/.*/get/.*$\
|^/perception_reproducer/rosbag_ego_odom$\
|^/sensing/gnss/septentrio/nav_sat_fix\
"""

AUTOWARE_DISABLE = {}

AUTOWARE_ARGS = {
    "use_aeb_autoware_state_check": "false",
    "use_sim_time": "false",
    "enable_all_modules_auto_mode": "true",
}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []


def _get_post_engage_direct_initial_pose(conf: dict) -> str | None:
    move_ego_forward_after_engage_m = float(conf["move_ego_forward_after_engage_m"])
    if move_ego_forward_after_engage_m == 0.0:
        return None
    base_pose = conf["direct_initial_pose"]
    if base_pose == "{}":
        base_pose = conf["initial_pose"]
    if base_pose == "{}":
        return None
    # Perturb ego after engage so the planner stops publishing a stopping trajectory.
    shifted_pose_dict = offset_pose_dict_forward(
        pose_str_to_dict(base_pose),
        move_ego_forward_after_engage_m,
    )
    return json.dumps(shifted_pose_dict)


def launch_engage_sequence(context: LaunchContext) -> list:
    conf = context.launch_configurations

    engage_node = Node(
        package="driving_log_replayer_v2",
        namespace="/driving_log_replayer_v2",
        executable="engage_node.py",
        output="screen",
        name="engage_node",
        parameters=[
            {
                "use_sim_time": False,
                "timeout_s": float(conf["timeout_s"]),
            }
        ],
    )

    def launch_post_engage_initial_pose_node(_: LaunchContext) -> list:
        post_engage_direct_initial_pose = _get_post_engage_direct_initial_pose(conf)
        if post_engage_direct_initial_pose is None:
            return [LogInfo(msg="post_engage_initial_pose_node is not activated")]
        return [
            LogInfo(msg="engage_node exited. launching post_engage_initial_pose_node"),
            Node(
                package="driving_log_replayer_v2",
                namespace="/driving_log_replayer_v2",
                executable="initial_pose_node.py",
                output="screen",
                name="post_engage_initial_pose_node",
                parameters=[
                    {
                        "use_sim_time": False,
                        "initial_pose": "{}",
                        "direct_initial_pose": post_engage_direct_initial_pose,
                    }
                ],
            ),
        ]

    return [
        engage_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=engage_node,
                on_exit=[OpaqueFunction(function=launch_post_engage_initial_pose_node)],
            )
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
        OpaqueFunction(function=launch_engage_sequence),
    ]
