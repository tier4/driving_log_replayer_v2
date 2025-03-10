from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

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
    pre_process_cmd = [*launch_base_cmd, "pre_process.launch.py", *arguments]
    pre_process = ExecuteProcess(cmd=pre_process_cmd, output="screen", name="pre_process")
    simulation_cmd = [*launch_base_cmd, "simulation.launch.py", *arguments]
    simulation = ExecuteProcess(cmd=simulation_cmd, output="screen", name="simulation")
    start_simulation = RegisterEventHandler(
        OnProcessExit(
            target_action=pre_process,
            on_exit=[LogInfo(msg="Pre-process done. start Simulation"), simulation],
        )
    )
    post_process_cmd = [*launch_base_cmd, "post_process.launch.py", *arguments]
    post_process = ExecuteProcess(cmd=post_process_cmd, output="screen", name="post_process")
    start_post_process = RegisterEventHandler(
        OnProcessExit(
            target_action=simulation,
            on_exit=[LogInfo(msg="Simulation done. start Post-process"), post_process],
        )
    )
    log_args = LogInfo(msg=f"{arguments=}")

    return [log_args, pre_process, start_simulation, start_post_process]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ],
    )
