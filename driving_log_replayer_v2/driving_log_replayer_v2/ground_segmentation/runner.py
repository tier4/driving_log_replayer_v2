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
from typing import Any
from typing import TYPE_CHECKING

import numpy as np
import ros2_numpy
from rosbag2_py import TopicMetadata
from std_msgs.msg import String

from driving_log_replayer_v2.ground_segmentation.evaluation_manager import (
    GroundSegmentationEvaluationManager,
)
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationResult
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationScenario
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.post_process.runner import ConvertedData
from driving_log_replayer_v2.post_process.runner import Runner
from driving_log_replayer_v2.post_process.runner import UseCaseInfo

if TYPE_CHECKING:
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header

    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.result import ResultWriter


def convert_to_ground_seg(msg: PointCloud2, subscribed_timestamp_nanosec: int) -> ConvertedData:
    """Convert PointCloud2 message to ground segmentation data."""
    header_timestamp_microsec = eval_conversions.unix_time_microsec_from_ros_msg(msg.header)
    subscribed_timestamp_microsec = eval_conversions.unix_time_microsec_from_ros_time_nanosec(
        subscribed_timestamp_nanosec
    )
    numpy_pcd = ros2_numpy.numpify(msg)
    data = np.stack((numpy_pcd["x"], numpy_pcd["y"], numpy_pcd["z"]), axis=-1)
    return ConvertedData(
        header_timestamp=header_timestamp_microsec,
        subscribed_timestamp=subscribed_timestamp_microsec,
        data=data,
    )


class GroundSegmentationRunner(Runner):
    def __init__(
        self,
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_jsonl_path: str,
        result_archive_path: str,
        storage: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        enable_analysis: str,
    ) -> None:
        super().__init__(
            GroundSegmentationScenario,
            scenario_path,
            rosbag_dir_path,
            t4_dataset_path,
            result_jsonl_path,
            result_archive_path,
            storage,
            evaluation_topics_with_task,
            degradation_topic,
            enable_analysis,
        )

    def _get_use_case_info_list(
        self,
        scenario: GroundSegmentationScenario,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        result_jsonl_path: str,
        result_archive_path: str,
    ) -> list[UseCaseInfo]:
        _ = result_archive_path  # unused
        return [
            UseCaseInfo(
                evaluation_manager_class=GroundSegmentationEvaluationManager,
                result_class=GroundSegmentationResult,
                conditions=scenario.Evaluation.Conditions,
                name="ground_segmentation",
                evaluation_topics_with_task=evaluation_topics_with_task,
                degradation_topic=degradation_topic,
                result_jsonl_path=result_jsonl_path,
            ),
        ]

    def _get_external_record_topics(self) -> list[TopicMetadata]:
        return [
            TopicMetadata(
                name="/driving_log_replayer_v2/ground_segmentation/results",
                type="std_msgs/msg/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
        ]

    @property
    def ground_seg_eval_manager(self) -> GroundSegmentationEvaluationManager:
        return self._use_cases["ground_segmentation"].evaluation_manager

    @property
    def ground_seg_result(self) -> GroundSegmentationResult:
        return self._use_cases["ground_segmentation"].result

    @property
    def ground_seg_result_writer(self) -> ResultWriter:
        return self._use_cases["ground_segmentation"].result_writer

    def _convert_ros_msg_to_data(
        self,
        topic_name: str,
        msg: Any,
        subscribed_timestamp_nanosec: int,
    ) -> ConvertedData:
        _ = topic_name  # unused
        # convert ros message to numpy array
        return convert_to_ground_seg(msg, subscribed_timestamp_nanosec)

    def _write_result(
        self, frame_result: FrameResult, header: Header, subscribed_timestamp_nanosec: int
    ) -> None:
        if frame_result.is_valid:
            self.ground_seg_result.set_frame(frame_result.data, frame_result.skip_counter)
        else:
            self.ground_seg_result.set_info_frame(
                frame_result.invalid_reason, frame_result.skip_counter
            )
        res_str = self.ground_seg_result_writer.write_result_with_time(
            self.ground_seg_result, subscribed_timestamp_nanosec
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/ground_segmentation/results",
            String(data=res_str),
            header.stamp,
        )

    def _evaluate_on_post_process(self) -> None:
        pass

    def _analysis(self) -> None:
        pass


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4_dataset_path: str,
    result_jsonl_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_topic: str,
    enable_analysis: str,
) -> None:
    evaluation_topics_with_task = {"dummy_task": [evaluation_topic]}

    runner = GroundSegmentationRunner(
        scenario_path,
        rosbag_dir_path,
        t4_dataset_path,
        result_jsonl_path,
        result_archive_path,
        storage,
        evaluation_topics_with_task,
        "",
        enable_analysis,
    )

    runner.evaluate()


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
        "--result-jsonl-path", required=True, help="Output file path for the result in JSONL format"
    )
    parser.add_argument(
        "--result-archive-path", required=True, help="Output directory path for the result"
    )
    parser.add_argument(
        "--storage",
        required=True,
        help="Output rosbag storage type mcap or sqlite3",
    )

    parser.add_argument(
        "--evaluation-topic",
        default="",
        help="ROS topic name to evaluate",
    )
    parser.add_argument(
        "--enable-analysis",
        default="false",
        help="If set, enable detailed analysis and output the analysis result",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    evaluate(
        args.scenario_path,
        args.rosbag_dir_path,
        args.t4_dataset_path,
        args.result_jsonl_path,
        args.result_archive_path,
        args.storage,
        args.evaluation_topic,
        args.enable_analysis,
    )


if __name__ == "__main__":
    main()
