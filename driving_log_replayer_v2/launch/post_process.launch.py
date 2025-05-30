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

from os.path import expandvars
from pathlib import Path
import shutil

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from rosbag2_py import Reindexer
from rosbag2_py import StorageOptions

from driving_log_replayer_v2.launch.argument import add_use_case_arguments
from driving_log_replayer_v2.launch.argument import ensure_arg_compatibility
from driving_log_replayer_v2.launch.argument import get_launch_arguments
from driving_log_replayer_v2.perception.runner import evaluate
from driving_log_replayer_v2.result import MultiResultEditor


def check_and_create_metadata_yaml(conf: dict) -> None:
    # for debug
    import time

    print("please delete the metadata.yaml file")  # noqa
    time.sleep(10)
    metadata_path = Path(conf["result_bag_path"]).joinpath("metadata.yaml")
    if metadata_path.exists():
        return
    storage_type = "sqlite3"
    mcap_bag_path = Path(conf["result_bag_path"]).joinpath("result_bag_0.mcap")
    if not mcap_bag_path.exists():
        storage_type = "mcap"
    storage_options = StorageOptions(
        storage_id=storage_type, uri=Path(conf["result_bag_path"]).as_posix()
    )
    Reindexer().reindex(storage_options)


def post_process(context: LaunchContext) -> list:
    conf = context.launch_configurations
    check_and_create_metadata_yaml(conf)

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
            "/localization/reference_kinematic_state",
        ]
        localization_analysis = ExecuteProcess(
            cmd=localization_analysis_cmd, output="screen", name="localization_analyze"
        )
        return [LogInfo(msg="run localization analysis."), localization_analysis]

    if conf["use_case"] == "perception":
        absolute_result_json_path = Path(
            expandvars(context.launch_configurations["result_json_path"])
        )

        def _run_perception_and_replace_rosbag(context: LaunchContext) -> list:
            absolute_result_json_path.parent.joinpath(
                absolute_result_json_path.stem + ".jsonl"
            ).unlink()
            evaluate(
                context.launch_configurations["scenario_path"],
                context.launch_configurations["result_bag_path"],
                context.launch_configurations["t4_dataset_path"],
                context.launch_configurations["result_json_path"],
                context.launch_configurations["result_archive_path"],
                context.launch_configurations["storage"],
                context.launch_configurations["evaluation_detection_topic_regex"],
                context.launch_configurations["evaluation_tracking_topic_regex"],
                context.launch_configurations["evaluation_prediction_topic_regex"],
                context.launch_configurations["evaluation_fp_validation_topic_regex"],
                context.launch_configurations["max_distance"],
                context.launch_configurations["distance_interval"],
            )
            shutil.rmtree(
                Path(context.launch_configurations["result_bag_path"]).as_posix(),
            )
            shutil.move(
                Path(context.launch_configurations["result_archive_path"])
                .joinpath("result_bag")
                .as_posix(),
                Path(context.launch_configurations["result_bag_path"]).as_posix(),
            )
            return [LogInfo(msg="perception post process finished.")]

        return [
            LogInfo(msg="run perception analysis."),
            OpaqueFunction(function=_run_perception_and_replace_rosbag),
        ]

    if conf["use_case"] == "planning_control":
        # merge diagnostic result.jsonl
        diag_result_path = Path(conf["result_archive_path"]).joinpath("diag_result.jsonl")
        if not diag_result_path.exists():
            return [LogInfo(msg="No diagnostics result.jsonl found. Abort merging result.jsonl")]
        multi_result_editor = MultiResultEditor(
            [
                Path(conf["result_json_path"]).as_posix() + "l",  # "json + l"
                diag_result_path.as_posix(),
            ]
        )
        multi_result_editor.write_back_result()
        return [LogInfo(msg="Merge results")]
    return [LogInfo(msg="No post-processing is performed.")]


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
