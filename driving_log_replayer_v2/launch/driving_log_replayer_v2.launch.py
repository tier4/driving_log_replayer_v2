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


def parse_launch_arguments(context: LaunchContext) -> list:
    launch_args = []
    conf = context.launch_configurations
    for k, v in conf.items():
        if v != "" and v is not None:
            launch_args.append(f"{k}:={v}")
    return launch_args


def launch_setup(context: LaunchContext) -> list:
    arguments = parse_launch_arguments(context)
    launch_base_cmd = ["ros2", "launch", "driving_log_replayer_v2"]
    # Specify the output dir from the parent because it will be created in the child process if the output dir is not specified.
    output_dir = context.launch_configurations.get("output_dir")
    launch_post_fix = [f"output_dir:={output_dir}", *arguments]
    enable_pre_process = context.launch_configurations.get("pre_process")
    enable_simulation = context.launch_configurations.get("simulation")
    enable_post_process = context.launch_configurations.get("post_process")

    actions = [LogInfo(msg=f"{arguments=}")]

    # prepare the process chains
    post_process = (
        ExecuteProcess(
            cmd=[*launch_base_cmd, "post_process.launch.py", *launch_post_fix],
            output="screen",
            name="post_process",
        )
        if enable_post_process == "true"
        else None
    )

    simulation = (
        ExecuteProcess(
            cmd=[*launch_base_cmd, "simulation.launch.py", *launch_post_fix],
            output="screen",
            name="simulation",
            on_exit=(
                [LogInfo(msg="Simulation done. start Post-process"), post_process]
                if post_process
                else [LogInfo(msg="Simulation done.")]
            ),
        )
        if enable_simulation == "true"
        else None
    )

    pre_process = (
        ExecuteProcess(
            cmd=[*launch_base_cmd, "pre_process.launch.py", *launch_post_fix],
            output="screen",
            name="pre_process",
            on_exit=(
                [LogInfo(msg="Pre-process done. start Simulation"), simulation]
                if simulation
                else (
                    [LogInfo(msg="Pre-process done. start Post-process"), post_process]
                    if post_process
                    else [LogInfo(msg="Pre-process done.")]
                )
            ),
        )
        if enable_pre_process == "true"
        else None
    )

    # define the process chains
    if enable_pre_process == "true":
        actions += [LogInfo(msg="Pre-process: enabled"), pre_process]
    else:
        actions.append(LogInfo(msg="Pre-process: disabled"))

    if enable_simulation == "true":
        # launch from simulation if pre_process is not defined else do nothing
        actions += [LogInfo(msg="Simulation: enabled"), simulation] if not pre_process else []
    else:
        actions.append(LogInfo(msg="Simulation: disabled"))

    if enable_post_process == "true":
        # launch from post_process if pre_process and simulation are not defined else do nothing
        actions += (
            [LogInfo(msg="Post-process: enabled"), post_process]
            if not pre_process and not simulation
            else []
        )
    else:
        actions.append(LogInfo(msg="Post-process: disabled"))

    return actions


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=ensure_arg_compatibility),
            OpaqueFunction(function=launch_setup),
        ],
    )
