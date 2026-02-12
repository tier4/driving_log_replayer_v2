# Copyright (c) 2022 TIER IV.inc
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


from importlib import import_module
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from driving_log_replayer_v2.scenario import load_scenario
from driving_log_replayer_v2.shutdown_once import ShutdownOnce
from launch import LaunchContext
from launch.actions import (GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from driving_log_replayer_v2.launch.argument import add_use_case_arguments
from driving_log_replayer_v2.launch.camera_2d_detector import \
    launch_camera_2d_detector
from driving_log_replayer_v2.launch.rosbag import (launch_bag_player,
                                                   launch_bag_recorder)
from driving_log_replayer_v2.launch.util import output_dummy_result_jsonl


def launch_autoware(context: LaunchContext) -> list:
    conf = context.launch_configurations
    if conf["with_autoware"] != "true":
        return [LogInfo(msg="Autoware is not launched. Only the evaluation node is launched.")]
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    launch_args = {
        "map_path": conf["map_path"],
        "vehicle_model": conf["vehicle_model"],
        "sensor_model": conf["sensor_model"],
        "vehicle_id": conf["vehicle_id"],
        "launch_vehicle_interface": "true",
        "launch_system_monitor": "true",
    }
    # Add pointcloud_map_file if specified
    if conf.get("pointcloud_map_file", "") != "":
        launch_args["pointcloud_map_file"] = conf["pointcloud_map_file"]
    launch_config = import_module(f"driving_log_replayer_v2.launch.{conf['use_case']}")
    launch_args |= launch_config.AUTOWARE_ARGS
    return [
        GroupAction(
            [
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        autoware_launch_file.as_posix(),
                    ),
                    launch_arguments=launch_args.items(),
                ),
            ],
            scoped=False,
            forwarding=True,
        ),
    ]


def launch_optional_nodes(context: LaunchContext) -> list:
    nodes_list = context.launch_configurations["with_optional_nodes"].split(",")
    optional_nodes = []
    if "2d_detector" in nodes_list:
        optional_nodes.append(LogInfo(msg="launching 2D detector......"))
        optional_nodes.extend(launch_camera_2d_detector(context))
    if len(optional_nodes) == 0:
        optional_nodes.append(LogInfo(msg="no optional nodes to launch."))
    return optional_nodes


def launch_map_height_fitter(context: LaunchContext) -> list:
    # Autoware specifications
    if context.launch_configurations.get("localization", "true") != "true":
        return [LogInfo(msg="map_height_fitter is not launched because localization is false")]

    fitter_launch_file = Path(
        get_package_share_directory("autoware_map_height_fitter"),
        "launch",
        "map_height_fitter.launch.xml",
    )
    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(fitter_launch_file.as_posix()))]


def launch_evaluator_node(context: LaunchContext) -> list:
    conf = context.launch_configurations
    if conf["record_only"] != "false":
        # output dummy result for Evaluator
        output_dummy_result_jsonl(conf["result_json_path"])
        return [LogInfo(msg="evaluator_node is not launched due to record only mode")]
    params = {
        "use_sim_time": True,
        "scenario_path": conf["scenario_path"],
        "t4_dataset_path": conf["t4_dataset_path"],
        "result_json_path": conf["result_json_path"],
        "result_archive_path": conf["result_archive_path"],
        "dataset_index": conf["dataset_index"],
    }
    launch_config = import_module(f"driving_log_replayer_v2.launch.{conf['use_case']}")
    params |= launch_config.NODE_PARAMS

    evaluator_name = conf["use_case"] + "_evaluator"

    return [
        Node(
            package="driving_log_replayer_v2",
            namespace="/driving_log_replayer_v2",
            executable=evaluator_name + "_node.py",
            output="screen",
            name=evaluator_name,
            parameters=[params],
            on_exit=ShutdownOnce(),
        ),
    ]


def launch_topic_state_monitor(context: LaunchContext) -> list:
    conf = context.launch_configurations
    if conf["use_case"] != "localization":
        return [
            LogInfo(msg="topic_state_monitor is not launched because use_case is not localization.")
        ]
    # autoware_component_state_monitor launch
    component_state_monitor_launch_file = Path(
        get_package_share_directory("autoware_component_state_monitor"),
        "launch",
        "component_state_monitor.launch.py",
    )
    topic_monitor_config_path = Path(
        get_package_share_directory("driving_log_replayer_v2"),
        "config",
        conf["use_case"],
        "topic_state_monitor.yaml",
    )
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(component_state_monitor_launch_file.as_posix()),
            launch_arguments={
                "file": topic_monitor_config_path.as_posix(),
                "mode": "logging_simulation",
            }.items(),
        ),
    ]


def launch_initial_pose_node(context: LaunchContext) -> list:
    conf = context.launch_configurations
    initial_pose = conf["initial_pose"]
    direct_initial_pose = conf["direct_initial_pose"]

    if initial_pose == "{}" and direct_initial_pose == "{}":
        return [LogInfo(msg="initial_pose_node is not activated")]

    params = {
        "use_sim_time": True,
        "initial_pose": initial_pose,
        "direct_initial_pose": direct_initial_pose,
    }

    return [
        Node(
            package="driving_log_replayer_v2",
            namespace="/driving_log_replayer_v2",
            executable="initial_pose_node.py",
            output="screen",
            name="initial_pose_node",
            parameters=[params],
        ),
    ]


def launch_goal_pose_node(context: LaunchContext) -> list:
    conf = context.launch_configurations
    goal_pose = conf["goal_pose"]

    if goal_pose == "{}":
        return [LogInfo(msg="goal_pose_node is not activated")]

    params = {
        "use_sim_time": True,
        "goal_pose": goal_pose,
    }

    return [
        Node(
            package="driving_log_replayer_v2",
            namespace="/driving_log_replayer_v2",
            executable="goal_pose_node.py",
            output="screen",
            name="goal_pose_node",
            parameters=[params],
        ),
    ]


def launch_ground_truth_publisher_node(context: LaunchContext) -> list:
    """Launch ground truth publisher node for ground_segmentation use case."""
    conf = context.launch_configurations
    if conf["use_case"] != "ground_segmentation":
        return []

    try:
        from driving_log_replayer_v2.ground_segmentation.models import \
            GroundSegmentationScenario

        scenario_path = Path(conf["scenario_path"])
        scenario = load_scenario(scenario_path, GroundSegmentationScenario)
        conditions = scenario.Evaluation.Conditions

        t4_dataset_path = conf.get("t4_dataset_path", "")
        if not t4_dataset_path:
            return [
                LogInfo(
                    msg="WARNING: t4_dataset_path is not set. Ground truth publisher node will not be launched."
                ),
            ]

        params = {
            "use_sim_time": True,
            "t4_dataset_path": t4_dataset_path,
            "ground_label": conditions.ground_label,
            "obstacle_label": conditions.obstacle_label,
            "publish_ground_only": False,
        }

        return [
            LogInfo(
                msg=f"Launching ground truth publisher node for ground_segmentation "
                f"(t4_dataset_path={t4_dataset_path})"
            ),
            Node(
                package="driving_log_replayer_v2",
                namespace="/driving_log_replayer_v2",
                executable="ground_truth_publisher_node.py",
                output="screen",
                name="ground_truth_publisher_node",
                parameters=[params],
            ),
        ]
    except Exception as e:
        import traceback
        error_msg = (
            f"Failed to launch ground truth publisher node: {e}\n"
            f"{traceback.format_exc()}"
        )
        return [LogInfo(msg=error_msg)]


def launch_use_case() -> list:
    return [
        OpaqueFunction(function=add_use_case_arguments),  # after ensure_arg_compatibility
        OpaqueFunction(function=launch_autoware),
        OpaqueFunction(function=launch_optional_nodes),
        OpaqueFunction(function=launch_map_height_fitter),
        OpaqueFunction(function=launch_evaluator_node),
        OpaqueFunction(function=launch_ground_truth_publisher_node),
        OpaqueFunction(function=launch_bag_player),
        OpaqueFunction(function=launch_bag_recorder),
        OpaqueFunction(function=launch_topic_state_monitor),
        OpaqueFunction(function=launch_initial_pose_node),
        OpaqueFunction(function=launch_goal_pose_node),
    ]
