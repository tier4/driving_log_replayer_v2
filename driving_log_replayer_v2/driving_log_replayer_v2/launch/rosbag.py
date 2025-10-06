# Copyright (c) 2025 TIER IV.inc
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
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import yaml

from driving_log_replayer_v2.shutdown_once import ShutdownOnce

PACKAGE_SHARE = get_package_share_directory("driving_log_replayer_v2")
QOS_PROFILE_PATH_STR = Path(PACKAGE_SHARE, "config", "qos.yaml").as_posix()


def extract_topics_from_profile(profile_name: str, profile_type: str) -> list[str]:
    if not profile_name:
        return []

    assert profile_type in ["publish", "remap"]
    profile_file = Path(
        get_package_share_directory("driving_log_replayer_v2"),
        "config",
        profile_type,
        f"{profile_name}.yaml",
    )
    # Make it work with symlink install as well.
    if profile_file.is_symlink():
        profile_file = profile_file.resolve()
    if not profile_file.exists():
        return []
    with profile_file.open("r") as f:
        remap_dict = yaml.safe_load(f)
        return remap_dict.get("profile_type", [])


def remap_str(topic: str) -> str:
    if topic == "/localization/kinematic_state":
        return "/localization/kinematic_state:=/localization/reference_kinematic_state"
    return f"{topic}:=/unused{topic}"


def add_remap(topic: str, remap_list: list) -> None:
    remap_list.append(remap_str(topic))


def system_defined_remap(conf: dict) -> list[str]:
    remap_list = []
    if conf["use_case"] == "localization":
        add_remap("/tf", remap_list)
        add_remap("/localization/kinematic_state", remap_list)
        add_remap("/localization/acceleration", remap_list)
    if conf.get("goal_pose", "{}") != "{}":
        add_remap("/planning/mission_planning/route", remap_list)
    return remap_list


def user_defined_publish(conf: dict) -> list[str]:
    publish_list = []
    if conf["publish_profile"] != "":
        publish_list.extend(extract_topics_from_profile(conf["publish_profile"], "publish"))
    return publish_list


def user_defined_remap(conf: dict) -> list[str]:
    remap_list = []
    # user defined remap
    user_remap_topics: list[str] = (
        conf["remap_arg"].split(",")
        if conf["remap_arg"] != ""
        else extract_topics_from_profile(conf["remap_profile"], "remap")
    )
    for topic in user_remap_topics:
        if topic.startswith("/"):
            user_remap_str = remap_str(topic)
            if user_remap_str not in remap_list:
                remap_list.append(user_remap_str)
    return remap_list


def get_pre_task_before_play_rosbag(
    context: LaunchContext, on_exit: ExecuteProcess
) -> Node | ExecuteProcess:
    conf = context.launch_configurations
    if conf.get("publish_topic_from_rosbag", "None") not in [
        "",
        "None",
    ]:  # FIXME: should change explicit declaration
        return Node(
            package="driving_log_replayer_v2",
            namespace="/driving_log_replayer_v2",
            executable="publish_topic_from_rosbag_node.py",
            output="screen",
            name="publish_topic_from_rosbag_node",
            parameters=[
                {
                    "use_sim_time": False,  # In order to trigger the timer without play rosbag
                    "input_bag": conf["input_bag"],
                    "storage_type": "sqlite3",
                    "publish_topic_from_rosbag": conf["publish_topic_from_rosbag"],
                }
            ],
            on_exit=[on_exit],
        )
    return ExecuteProcess(
        cmd=["echo", "pre-task before play rosbag is not activated"],
        on_exit=[on_exit],
    )


def launch_bag_player(
    context: LaunchContext,
) -> IncludeLaunchDescription:
    conf = context.launch_configurations
    play_cmd = [
        "ros2",
        "bag",
        "play",
        conf["input_bag"],
        "--rate",
        conf["play_rate"],
        "--clock",
        "200",
        "--qos-profile-overrides-path",
        QOS_PROFILE_PATH_STR,
    ]
    # topics
    publish_list = ["--topics"]
    publish_list.extend(user_defined_publish(conf))
    if len(publish_list) != 1:
        play_cmd.extend(publish_list)
    # remap
    remap_list = ["--remap"]
    remap_list.extend(system_defined_remap(conf))
    remap_list.extend(user_defined_remap(conf))
    if len(remap_list) != 1:
        play_cmd.extend(remap_list)
    bag_player = (
        ExecuteProcess(
            cmd=play_cmd,
            output="screen",
            on_exit=[ExecuteProcess(cmd=["sleep", "3"], on_exit=[ShutdownOnce()])],
        )  # If compression is enabled, it takes a long time to write the record, and if the play finishes immediately, the record will not be finished in time.
        if conf["record_only"] == "true"
        else ExecuteProcess(cmd=play_cmd, output="screen")
    )
    delay_player_for_pre_task = ExecuteProcess(
        cmd=["sleep", conf["play_delay"]], on_exit=[bag_player]
    )
    pre_task_player = get_pre_task_before_play_rosbag(context, delay_player_for_pre_task)
    delay_player_for_autoware = ExecuteProcess(
        cmd=["sleep", conf["play_delay"]], on_exit=[pre_task_player]
    )
    return [
        delay_player_for_autoware,
        LogInfo(msg=f"remap_command is {remap_list}, topics_command is {publish_list}"),
    ]


def launch_bag_recorder(context: LaunchContext) -> list:
    conf = context.launch_configurations
    record_cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        conf["storage"],
        "-o",
        conf["result_bag_path"],
        "--qos-profile-overrides-path",
        QOS_PROFILE_PATH_STR,
        "--use-sim-time",
    ]
    if conf["storage"] == "mcap":
        record_cmd += ["--storage-preset-profile", "zstd_fast"]
    if conf["override_topics_regex"] == "":
        launch_config = import_module(f"driving_log_replayer_v2.launch.{conf['use_case']}")
        record_cmd += ["-e", launch_config.RECORD_TOPIC]
    else:
        record_cmd += ["-e", conf["override_topics_regex"]]
    return [ExecuteProcess(cmd=record_cmd)]
