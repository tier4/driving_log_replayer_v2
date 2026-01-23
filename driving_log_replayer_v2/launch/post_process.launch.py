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

from dataclasses import dataclass
from os.path import expandvars
from pathlib import Path
import shutil

from launch import LaunchContext
from launch import LaunchDescription
from launch.action import Action
from launch.actions import ExecuteLocal
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from driving_log_replayer_v2.ground_segmentation.runner import (
    evaluate as evaluate_ground_segmentation,
)
from driving_log_replayer_v2.launch.argument import add_use_case_arguments
from driving_log_replayer_v2.launch.argument import ensure_arg_compatibility
from driving_log_replayer_v2.launch.argument import get_launch_arguments
from driving_log_replayer_v2.perception.runner import evaluate as evaluate_perception
from driving_log_replayer_v2.result import MultiResultEditor
from driving_log_replayer_v2.result import ResultAnalyzer
from driving_log_replayer_v2.rosbag import create_metadata_yaml


@dataclass(frozen=True, slots=True)
class ProcessInfo:
    process_list: list[Action]
    last_action: Action | None


def localization(conf: dict[str, str]) -> ProcessInfo:
    if conf["enable_analysis"] != "true":
        return ProcessInfo(
            process_list=[LogInfo(msg="skip localization analysis.")], last_action=None
        )
    localization_analysis_cmd = [
        "ros2",
        "run",
        "autoware_localization_evaluation_scripts",
        "analyze_rosbags_parallel.py",
        f"{conf['output_dir']}",
        "--save_dir_relative",
        "result_archive",
        "--topic_reference",
        "/localization/reference_kinematic_state",
        "--scenario_file",
        f"{conf['scenario_path']}",
    ]
    localization_analysis = ExecuteProcess(
        cmd=localization_analysis_cmd, output="screen", name="localization_analyze"
    )
    localization_update_jsonl_cmd = [
        "ros2",
        "run",
        "driving_log_replayer_v2",
        "localization_update_result_jsonl.py",
        f"{conf['output_dir']}/result.jsonl",
        f"{conf['output_dir']}/result_archive",
    ]
    localization_update_jsonl = ExecuteProcess(
        cmd=localization_update_jsonl_cmd, output="screen", name="localization_update"
    )
    localization_update_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_analysis,
            on_exit=[localization_update_jsonl],
        )
    )
    return ProcessInfo(
        process_list=[
            LogInfo(msg="run localization analysis."),
            localization_analysis,
            localization_update_event_handler,
        ],
        last_action=localization_update_jsonl,
    )


def perception() -> ProcessInfo:
    def _run_perception_and_replace_rosbag(context: LaunchContext) -> list:
        conf = context.launch_configurations

        absolute_result_jsonl_path = Path(expandvars(conf["result_jsonl_path"]))
        absolute_result_jsonl_path.unlink()
        evaluate_perception(
            conf["scenario_path"],
            conf["result_bag_path"],
            conf["t4_dataset_path"],
            conf["result_jsonl_path"],
            conf["result_archive_path"],
            conf["storage"],
            conf["evaluation_detection_topic_regex"],
            conf["evaluation_tracking_topic_regex"],
            conf["evaluation_prediction_topic_regex"],
            conf["degradation_topic"],
            conf["enable_analysis"],
            conf["analysis_max_distance"],
            conf["analysis_distance_interval"],
        )
        shutil.rmtree(
            Path(conf["result_bag_path"]).as_posix(),
        )
        shutil.move(
            Path(conf["result_archive_path"]).joinpath("result_bag").as_posix(),
            Path(conf["result_bag_path"]).as_posix(),
        )
        return [LogInfo(msg="perception post process finished.")]

    perception_action = OpaqueFunction(function=_run_perception_and_replace_rosbag)
    return ProcessInfo(
        process_list=[LogInfo(msg="run perception analysis."), perception_action],
        last_action=perception_action,
    )


def ground_segmentation() -> ProcessInfo:
    def _run_ground_segmentation(context: LaunchContext) -> list:
        conf = context.launch_configurations

        absolute_result_jsonl_path = Path(expandvars(conf["result_jsonl_path"]))
        absolute_result_jsonl_path.unlink()
        evaluate_ground_segmentation(
            conf["scenario_path"],
            conf["result_bag_path"],
            conf["t4_dataset_path"],
            conf["result_jsonl_path"],
            conf["result_archive_path"],
            conf["storage"],
            conf["evaluation_topic"],
            conf["enable_analysis"],
        )
        shutil.rmtree(
            Path(conf["result_bag_path"]).as_posix(),
        )
        shutil.move(
            Path(conf["result_archive_path"]).joinpath("result_bag").as_posix(),
            Path(conf["result_bag_path"]).as_posix(),
        )
        return [LogInfo(msg="ground_segmentation post process finished.")]

    ground_segmentation_action = OpaqueFunction(function=_run_ground_segmentation)
    return ProcessInfo(
        process_list=[LogInfo(msg="run ground_segmentation analysis."), ground_segmentation_action],
        last_action=ground_segmentation_action,
    )


def planning_control(conf: dict[str, str]) -> ProcessInfo:
    # merge diagnostic result.jsonl
    diag_result_path = Path(conf["result_archive_path"]).joinpath("diag_result.jsonl")
    planning_factor_result_path = Path(conf["result_archive_path"]).joinpath(
        "planning_factor_result.jsonl"
    )
    metric_result_path = Path(conf["result_archive_path"]).joinpath("metric_result.jsonl")
    result_paths = [Path(conf["result_jsonl_path"]).as_posix()]

    if diag_result_path.exists():
        result_paths.append(diag_result_path.as_posix())
    if planning_factor_result_path.exists():
        result_paths.append(planning_factor_result_path.as_posix())
    if metric_result_path.exists():
        result_paths.append(metric_result_path.as_posix())

    if len(result_paths) == 1:
        process_list = [LogInfo(msg="No additional result.jsonl found. Abort merging result.jsonl")]
    else:
        multi_result_editor = MultiResultEditor(result_paths)
        multi_result_editor.write_back_result()
        process_list = [LogInfo(msg="Merge results")]

    return ProcessInfo(process_list=process_list, last_action=None)


def time_step_based_trajectory(conf: dict[str, str]) -> ProcessInfo:
    # This use_case is record_only so not create result_archive directory.
    Path(conf["result_archive_path"]).mkdir(parents=True, exist_ok=True)
    time_step_analysis_cmd = [
        "ros2",
        "launch",
        "autoware_planning_data_analyzer",
        "planning_data_analyzer.launch.xml",
        f"bag_path:={conf['result_bag_path']}/result_bag_0.mcap",
        f"output_dir:={conf['result_archive_path']}",
    ]

    time_step_analysis = ExecuteProcess(
        cmd=time_step_analysis_cmd, output="screen", name="time_step_analysis"
    )

    return ProcessInfo(
        process_list=[LogInfo(msg="run time_step_based_trajectory analysis."), time_step_analysis],
        last_action=time_step_analysis,
    )


def perception_reproducer(conf: dict[str, str]) -> ProcessInfo:
    # Merge pass_result.jsonl if it exists
    pass_result_path = Path(conf["result_archive_path"]).joinpath("pass_result.jsonl")
    result_paths = [Path(conf["result_jsonl_path"]).as_posix()]

    if pass_result_path.exists():
        result_paths.append(pass_result_path.as_posix())

    if len(result_paths) == 1:
        process_list = [LogInfo(msg="No additional result.jsonl found. Abort merging result.jsonl")]
    else:
        multi_result_editor = MultiResultEditor(result_paths)
        multi_result_editor.write_back_result()
        process_list = [LogInfo(msg="Merge perception_reproducer results")]

    return ProcessInfo(process_list=process_list, last_action=None)


def post_process(context: LaunchContext) -> list:
    conf = context.launch_configurations
    create_metadata_yaml(conf["result_bag_path"])

    if conf["use_case"] == "localization":
        process_info = localization(conf)
    elif conf["use_case"] == "perception":
        process_info = perception()
    elif conf["use_case"] == "ground_segmentation":
        process_info = ground_segmentation()
    elif conf["use_case"] == "planning_control":
        process_info = planning_control(conf)
    elif conf["use_case"] == "time_step_based_trajectory":
        process_info = time_step_based_trajectory(conf)
    elif conf["use_case"] == "perception_reproducer":
        process_info = perception_reproducer(conf)
    else:
        err_msg = f"Unsupported use_case for post_process: {conf['use_case']}"
        raise ValueError(err_msg)

    def _run_analyze_results(context: LaunchContext) -> list:
        conf = context.launch_configurations

        result_analyzer = ResultAnalyzer(
            Path(conf["result_jsonl_path"]),
            Path(conf["result_archive_path"]),
        )
        result_analyzer.analyze_results()
        return [LogInfo(msg="analyze results finished.")]

    if isinstance(process_info.last_action, ExecuteLocal):
        return [
            *process_info.process_list,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=process_info.last_action,
                    on_exit=[
                        LogInfo(msg="start analyzing results."),
                        OpaqueFunction(function=_run_analyze_results),
                    ],
                )
            ),
        ]
    if isinstance(process_info.last_action, OpaqueFunction):
        # NOTE: OpaqueFunction blocks the launch thread so seems to guarantee the execution order.
        return [
            *process_info.process_list,
            LogInfo(msg="start analyzing results."),
            OpaqueFunction(function=_run_analyze_results),
        ]

    return [
        *process_info.process_list,
        LogInfo(msg="start analyzing results."),
        OpaqueFunction(function=_run_analyze_results),
    ]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=ensure_arg_compatibility),
            OpaqueFunction(function=add_use_case_arguments),
            OpaqueFunction(function=post_process),
        ],
    )
