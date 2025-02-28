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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource

from driving_log_replayer_v2.launch.util import output_dummy_result_bag
from driving_log_replayer_v2.launch.util import output_dummy_result_jsonl

RECORD_TOPIC = ""

AUTOWARE_DISABLE = {}

AUTOWARE_ARGS = {}

NODE_PARAMS = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []


def launch_ndt_convergence(context: LaunchContext) -> list:
    conf = context.launch_configurations
    ndt_convergence_launch_file = Path(
        get_package_share_directory("ndt_convergence_evaluation"),
        "launch",
        "ndt_convergence_evaluation.launch.py",
    )
    if not ndt_convergence_launch_file.exists():
        return [LogInfo(msg="ndt_convergence_evaluation is not launched. The file does not exist.")]

    # Output dummies to comply with Evaluator specifications
    output_dummy_result_jsonl(conf["result_json_path"], summary="NDT Convergence always success")
    output_dummy_result_bag(conf["result_bag_path"])

    launch_args = {
        "map_path": conf["map_path"] + "/pointcloud_map.pcd",
        "rosbag_file_name": conf["input_bag"],
        "save_dir": conf["result_archive_path"],
    }
    return [
        GroupAction(
            [
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        ndt_convergence_launch_file.as_posix(),
                    ),
                    launch_arguments=launch_args.items(),
                ),
            ]
        )
    ]
