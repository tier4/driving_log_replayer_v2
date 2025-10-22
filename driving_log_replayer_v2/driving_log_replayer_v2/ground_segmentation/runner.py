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

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd  # Ensure this is imported
from rclpy.clock import Clock
import ros2_numpy
from rosbag2_py import TopicMetadata
from std_msgs.msg import String

from driving_log_replayer_v2.ground_segmentation.manager import GroundSegmentationEvaluationManager
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationResult
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.ros2_utils import RosBagManager
from driving_log_replayer_v2_msgs.msg import GroundSegmentationEvalResult

if TYPE_CHECKING:
    from sensor_msgs.msg import PointCloud2


def convert_to_numpy_pointcloud(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 message to numpy array."""
    numpy_pcd = ros2_numpy.numpify(msg)
    return np.stack((numpy_pcd["x"], numpy_pcd["y"], numpy_pcd["z"]), axis=-1)


def write_result(
    additional_record_topic_name: dict[str],
    msg: PointCloud2,
    result: GroundSegmentationResult,
    result_writer: ResultWriter,
    frame_result: GroundSegmentationEvalResult,
    rosbag_manager: RosBagManager,
    subscribed_timestamp_nanosec: int,
) -> None:
    if isinstance(frame_result, GroundSegmentationEvalResult):
        result.set_frame(frame_result)
    elif isinstance(frame_result, str):
        if frame_result == "No Ground Truth":
            result.set_info_frame(frame_result)
        else:
            err_msg = f"Unknown evaluate failure: {frame_result}"
            raise ValueError(err_msg)
    else:
        err_msg = f"Unknown frame result: {frame_result}"
        raise TypeError(err_msg)
    res_str = result_writer.write_result_with_time(result, subscribed_timestamp_nanosec)
    rosbag_manager.write_results(
        additional_record_topic_name["string/results"],
        String(data=res_str),
        msg.header.stamp,
    )


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4_dataset_path: str,
    result_json_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_topic_regex: str,
) -> None:
    evaluation_topics = [evaluation_topic_regex]

    # initialize GroundSegmentationEvaluationManager
    evaluator = GroundSegmentationEvaluationManager(
        scenario_path,
        t4_dataset_path,
        result_archive_path,
        evaluation_topics,
    )
    if evaluator.check_scenario_error() is not None:
        error = evaluator.check_scenario_error()
        result_writer = ResultWriter(
            result_json_path,
            Clock(),
            {},
        )
        error_dict = {
            "Result": {"Success": False, "Summary": "ScenarioFormatError"},
            "Stamp": {"System": 0.0},
            "Frame": {"ErrorMsg": error.__str__()},
        }
        result_writer.write_line(error_dict)
        result_writer.close()
        raise error
    degradation_topic = evaluator.get_degradation_topic()

    # initialize ResultWriter
    result_writer = ResultWriter(
        result_json_path,
        Clock(),
        evaluator.get_evaluation_condition(),
    )
    result = GroundSegmentationResult(evaluator.get_evaluation_condition())

    # initialize RosBagManager
    additional_record_topics_name = {
        "string/results": "/driving_log_replayer_v2/ground_segmentation/results",
    }
    additional_record_topics = [
        TopicMetadata(
            name=topic_name,
            type="std_msgs/String",
            serialization_format="cdr",
            offered_qos_profiles="",
        )
        for topic_name in additional_record_topics_name.values()
    ]
    rosbag_manager = RosBagManager(
        rosbag_dir_path,
        Path(result_archive_path).joinpath("result_bag").as_posix(),
        storage,
        evaluator.get_evaluation_topics(),
        additional_record_topics,
    )

    # save scenario.yaml to check later
    shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    # Prepare to collect frame results for CSV
    frame_results_list = []

    # Extract dataset info (customize as needed)
    t4dataset_id = getattr(evaluator, "t4dataset_id", Path(t4_dataset_path).stem)
    # main evaluation process
    for topic_name, msg, subscribed_timestamp_nanosec in rosbag_manager.read_messages():
        header_timestamp_microsec = eval_conversions.unix_time_microsec_from_ros_msg(msg.header)
        pointcloud = convert_to_numpy_pointcloud(msg)

        frame_result = evaluator.evaluate(
            topic_name,
            header_timestamp_microsec,
            pointcloud,
        )

        # Only collect valid GroundSegmentationEvalResult frames
        if isinstance(frame_result, GroundSegmentationEvalResult):
            frame_results_list.append({
                "topic_name": topic_name,
                "t4dataset_id": t4dataset_id,
                "tp": frame_result.tp,
                "fp": frame_result.fp,
                "tn": frame_result.tn,
                "fn": frame_result.fn,
                "accuracy": frame_result.accuracy,
                "precision": frame_result.precision,
                "recall": frame_result.recall,
                "specificity": frame_result.specificity,
                "f1_score": frame_result.f1_score,
            })

        if topic_name == degradation_topic:
            write_result(
                additional_record_topics_name,
                msg,
                result,
                result_writer,
                frame_result,
                rosbag_manager,
                subscribed_timestamp_nanosec,
            )

    # Save all frame results to CSV
    if frame_results_list:
        df = pd.DataFrame(frame_results_list)
        csv_path = Path(result_archive_path).joinpath("analysis_result.csv")
        df.fillna("nan").to_csv(csv_path, index=False)

    result_writer.close()
    rosbag_manager.close_writer()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate ground segmentation rosbag w/ t4dataset")
    parser.add_argument("--scenario-path", required=True, help="File path to scenario files")
    parser.add_argument(
        "--rosbag-dir-path",
        required=True,
        help="Directory path to rosbag which is outputted by Autoware",
    )
    parser.add_argument("--t4-dataset-path", required=True, help="Directory path to t4dataset")
    parser.add_argument(
        "--result-json-path", required=True, help="Output file path for the result in JSONL format"
    )
    parser.add_argument(
        "--result-archive-path", required=True, help="Output directory path for the result"
    )
    parser.add_argument(
        "--storage",
        required=True,
        help="Storage type for rosbag2 to read and write",
    )
    parser.add_argument(
        "--evaluation-topic",
        default="",
        help="ROS topic name to evaluate",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    evaluate(
        args.scenario_path,
        args.rosbag_dir_path,
        args.t4_dataset_path,
        args.result_json_path,
        args.result_archive_path,
        args.storage,
        args.evaluation_topic,
    )


if __name__ == "__main__":
    main()
