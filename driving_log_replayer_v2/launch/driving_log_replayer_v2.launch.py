from pathlib import Path

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

from driving_log_replayer_v2.launch.argument import get_launch_arguments


def parse_launch_arguments(context: LaunchContext) -> list:
    return []


def launch_pre_process(context: LaunchContext) -> list:
    arguments = parse_launch_arguments(context)
    pre_process_cmd = [
        "ros2",
        "launch",
        "driving_log_replayer_v2",
        "pre_process.launch.py",
        *arguments,
    ]
    pre_process = ExecuteProcess(cmd=pre_process_cmd, output="screen", name="pre_process")
    return [pre_process]


def launch_simulation(context: LaunchContext) -> list:
    arguments = parse_launch_arguments(context)
    simulation_cmd = [
        "ros2",
        "launch",
        "driving_log_replayer_v2",
        "simulation.launch.py",
        *arguments,
    ]
    simulation = ExecuteProcess(cmd=simulation_cmd, output="screen", name="simulation")
    return [simulation]


def launch_post_process(context: LaunchContext) -> list:
    arguments = parse_launch_arguments(context)
    post_process_cmd = [
        "ros2",
        "launch",
        "driving_log_replayer_v2",
        "post_process.launch.py",
        *arguments,
    ]
    post_process = ExecuteProcess(cmd=post_process_cmd, output="screen", name="post_process")
    return [post_process]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_pre_process),
            OpaqueFunction(function=launch_post_process),
        ],
    )
