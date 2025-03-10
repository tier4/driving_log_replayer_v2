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


from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction

from driving_log_replayer_v2.launch.argument import ensure_arg_compatibility
from driving_log_replayer_v2.launch.argument import get_launch_arguments


def post_process(context: LaunchContext) -> list:
    conf = context.launch_configurations
    if conf["use_case"] == "localization":
        localization_analysis_cmd = [
            "ros2",
            "run",
            "autoware_localization_evaluation_scripts",
            "analyze_rosbags_parallel.py",
            f"{conf['output_dir']}",
            "--save_dir_relative",
            "result_archive",
            "--topic_reference",
            "/localization/reference_kinematic_state"
        ]
        localization_analysis = ExecuteProcess(
            cmd=localization_analysis_cmd, output="screen", name="localization_analyze"
        )
        return [LogInfo(msg="run localization analysis."), localization_analysis]

    return [LogInfo(msg="No post-processing is performed.")]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=ensure_arg_compatibility),
            OpaqueFunction(function=post_process),
        ],
    )
