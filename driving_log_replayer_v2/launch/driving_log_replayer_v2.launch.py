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

import datetime
import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

from driving_log_replayer_v2.launch_config import driving_log_replayer_v2_config
from driving_log_replayer_v2.shutdown_once import ShutdownOnce


def get_launch_arguments() -> list:
    """
    Set and return launch argument.

    scenario_path
    output_dir
    dataset_dir
    dataset_index
    play_rate
    play_delay
    with_autoware
    """
    launch_arguments = []

    def add_launch_arg(
        name: str,
        default_value: str | None = None,
        description: str = "",
    ) -> None:
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description),
        )

    add_launch_arg("scenario_path", description="scenario file path")
    add_launch_arg(
        "output_dir",
        default_value="",
        description="Directory to output evaluation results. If omitted, the out/${datetime} directory is created in the same directory as scenario. Mount in read-write mode when using docker",
    )
    add_launch_arg(
        "dataset_dir",
        default_value="",
        description="Directory where the dataset is located. If not specified, the directory where the scenario is located.",
    )
    add_launch_arg("dataset_index", default_value="", description="index number of dataset")
    add_launch_arg("play_rate", default_value="1.0", description="ros2 bag play rate")
    add_launch_arg("play_delay", default_value="10.0", description="ros2 bag play delay")
    add_launch_arg(
        "with_autoware",
        default_value="true",
        description="Whether to launch Autoware or not. set false if Autoware is started on a different PC.",
    )

    return launch_arguments


def create_output_dir(output_dir_str: str, scenario_path: Path) -> Path:
    if output_dir_str != "":
        output_dir = Path(output_dir_str)
        output_dir.mkdir(exist_ok=True, parents=True)
        return output_dir
    # create output_dir in scenario path
    time_now = datetime.datetime.now().strftime("%Y-%m%d-%H%M%S")  # noqa
    output_dir = scenario_path.parent.joinpath("out", time_now)
    output_dir.mkdir(exist_ok=True, parents=True)
    symlink_dst = output_dir.parent.joinpath("latest").as_posix()
    update_symlink = ["ln", "-snf", output_dir.as_posix(), symlink_dst]
    subprocess.run(update_symlink, check=False)
    return output_dir


def check_launch_component(conf: dict) -> dict:
    use_case_launch_arg = driving_log_replayer_v2_config[conf["use_case"]]["disable"]
    # update autoware component launch or not
    autoware_components = ["sensing", "localization", "perception", "planning", "control"]
    launch_component = {}
    for component in autoware_components:
        # argument has higher priority than the launch_config.py setting.
        if conf.get(component) is None and use_case_launch_arg.get(component) is not None:
            conf[component] = use_case_launch_arg[component]
        launch_component[component] = conf.get(component, "true")
    return launch_component


def ensure_arg_compatibility(context: LaunchContext) -> list:
    conf = context.launch_configurations
    scenario_path = Path(conf["scenario_path"])
    dataset_dir = scenario_path.parent if conf["dataset_dir"] == "" else Path(conf["dataset_dir"])
    output_dir = create_output_dir(conf["output_dir"], scenario_path)
    conf["output_dir"] = output_dir.as_posix()

    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    # check datasets length and index
    datasets = yaml_obj["Evaluation"]["Datasets"]
    idx_str = conf["dataset_index"]
    if idx_str == "":  # default value
        if len(datasets) == 1:
            dataset_index = 0
        else:
            return [
                LogInfo(msg="launch argument 'dataset_index:=i' is required"),
            ]
    else:
        dataset_index = int(idx_str)
    for k, v in datasets[dataset_index].items():
        dataset_path = dataset_dir.joinpath(k)
        conf["vehicle_id"] = v["VehicleId"]
        init_pose: dict | None = v.get(
            "InitialPose",
        )  # nullに設定されている。または書かれてない場合はNone
        if init_pose is not None:
            conf["initial_pose"] = json.dumps(init_pose)
        direct_pose: dict | None = v.get("DirectInitialPose")
        if direct_pose is not None:
            conf["direct_initial_pose"] = json.dumps(direct_pose)
        goal_pose: dict | None = v.get("GoalPose")
        if goal_pose is not None:
            conf["goal_pose"] = json.dumps(goal_pose)
    conf["map_path"] = dataset_path.joinpath("map").as_posix()
    conf["vehicle_model"] = yaml_obj["VehicleModel"]
    conf["sensor_model"] = yaml_obj["SensorModel"]
    conf["t4_dataset_path"] = dataset_path.as_posix()
    conf["input_bag"] = dataset_path.joinpath("input_bag").as_posix()
    conf["result_json_path"] = output_dir.joinpath("result.json").as_posix()
    conf["result_bag_path"] = output_dir.joinpath("result_bag").as_posix()
    conf["result_archive_path"] = output_dir.joinpath("result_archive").as_posix()
    conf["use_case"] = yaml_obj["Evaluation"]["UseCaseName"]

    return [
        LogInfo(
            msg=f"{dataset_path=}, {dataset_index=}, {output_dir=}, use_case={conf['use_case']}",
        ),
        LogInfo(
            msg=f"{check_launch_component(conf)=}",
        ),
        LogInfo(
            msg=f"{conf.get('initial_pose')=}, {conf.get('direct_initial_pose')=}",
        ),
        LogInfo(
            msg=f"{conf.get('goal_pose')=}",
        ),
    ]


def launch_autoware(context: LaunchContext) -> list:
    conf = context.launch_configurations
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
    }
    launch_args |= driving_log_replayer_v2_config[conf["use_case"]]["autoware"]
    return [
        GroupAction(
            [
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        autoware_launch_file.as_posix(),
                    ),
                    launch_arguments=launch_args.items(),
                    condition=IfCondition(conf["with_autoware"]),
                ),
            ],
            scoped=False,
            forwarding=True,
        ),
    ]


def launch_map_height_fitter(context: LaunchContext) -> list:
    fitter_launch_file = Path(
        get_package_share_directory("autoware_map_height_fitter"),
        "launch",
        "map_height_fitter.launch.xml",
    )
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                fitter_launch_file.as_posix(),
            ),
            condition=IfCondition(context.launch_configurations["localization"]),
        ),
    ]


def launch_evaluator_node(context: LaunchContext) -> list:
    conf = context.launch_configurations
    params = {
        "use_sim_time": True,
        "scenario_path": conf["scenario_path"],
        "t4_dataset_path": conf["t4_dataset_path"],
        "result_json_path": conf["result_json_path"],
        "result_archive_path": conf["result_archive_path"],
        "dataset_index": conf["dataset_index"],
    }
    params |= driving_log_replayer_v2_config[conf["use_case"]]["node"]

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


def launch_bag_player(
    context: LaunchContext,
) -> IncludeLaunchDescription:
    conf = context.launch_configurations
    play_cmd = [
        "ros2",
        "bag",
        "play",
        conf["input_bag"],
        "--delay",
        conf["play_delay"],
        "--rate",
        conf["play_rate"],
        "--clock",
        "200",
        "--qos-profile-overrides-path",
        Path(
            get_package_share_directory("driving_log_replayer_v2"),
            "config",
            conf["use_case"],
            "qos.yaml",
        ).as_posix(),
    ]
    remap_list = ["--remap"]
    if conf.get("sensing", "true") == "true":
        remap_list.append(
            "/sensing/lidar/concatenated/pointcloud:=/unused/concatenated/pointcloud",
        )
    if conf.get("localization", "true") == "true":
        remap_list.append(
            "/tf:=/unused/tf",
        )
        remap_list.append(
            "/localization/kinematic_state:=/unused/localization/kinematic_state",
        )
        remap_list.append(
            "/localization/acceleration:=/unused/localization/acceleration",
        )
    if conf.get("perception", "true") == "true":
        # remap perception msgs in bag
        remap_list.append(
            "/perception/obstacle_segmentation/pointcloud:=/unused/perception/obstacle_segmentation/pointcloud",
        )
        remap_list.append(
            "/perception/object_recognition/objects:=/unused/perception/object_recognition/objects",
        )
    if conf.get("goal_pose") is not None:
        remap_list.append(
            "/planning/mission_planning/route:=/unused/planning/mission_planning/route",
        )
    if len(remap_list) != 1:
        play_cmd.extend(remap_list)
    bag_player = ExecuteProcess(cmd=play_cmd, output="screen")
    return [bag_player]


def launch_bag_recorder(context: LaunchContext) -> list:
    conf = context.launch_configurations
    record_cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        conf["result_bag_path"],
        "--qos-profile-overrides-path",
        Path(
            get_package_share_directory("driving_log_replayer_v2"),
            "config",
            conf["use_case"],
            "qos.yaml",
        ).as_posix(),
        "-e",
        driving_log_replayer_v2_config[conf["use_case"]]["record"],
        "--use-sim-time",
    ]
    return [ExecuteProcess(cmd=record_cmd)]


def launch_topic_state_monitor(context: LaunchContext) -> list:
    conf = context.launch_configurations
    # component_state_monitor launch
    component_state_monitor_launch_file = Path(
        get_package_share_directory("component_state_monitor"),
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
            AnyLaunchDescriptionSource(
                component_state_monitor_launch_file.as_posix(),
            ),
            launch_arguments={
                "file": topic_monitor_config_path.as_posix(),
                "mode": "logging_simulation",
            }.items(),
            condition=IfCondition(str(conf["use_case"] == "localization")),
        ),
    ]


def launch_initial_pose_node(context: LaunchContext) -> list:
    conf = context.launch_configurations
    initial_pose = conf.get("initial_pose", "")
    direct_initial_pose = conf.get("direct_initial_pose", "")
    params = {
        "use_sim_time": True,
        "initial_pose": initial_pose,
        "direct_initial_pose": direct_initial_pose,
    }

    if initial_pose == "" and direct_initial_pose == "":
        return [LogInfo(msg="initial_pose_node is not activated")]

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
    goal_pose = conf.get("goal_pose", "")
    params = {
        "use_sim_time": True,
        "goal_pose": goal_pose,
    }

    if goal_pose == "":
        return [LogInfo(msg="goal_pose_node is not activated")]

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


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=ensure_arg_compatibility),
            OpaqueFunction(function=launch_autoware),
            OpaqueFunction(function=launch_map_height_fitter),
            OpaqueFunction(function=launch_evaluator_node),
            OpaqueFunction(function=launch_bag_player),
            OpaqueFunction(function=launch_bag_recorder),
            OpaqueFunction(function=launch_topic_state_monitor),
            OpaqueFunction(function=launch_initial_pose_node),
            OpaqueFunction(function=launch_goal_pose_node),
        ],
    )
