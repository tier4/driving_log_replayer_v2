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
from std_msgs.msg import String

from driving_log_replayer_v2.ground_segmentation.evaluation_manager import (
    GroundSegmentationEvaluationManager,
)
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationResult
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationScenario
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.post_process.runner import Runner
from driving_log_replayer_v2.post_process.runner import SimulationInfo
from driving_log_replayer_v2.post_process.runner import TopicInfo

if TYPE_CHECKING:
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header

    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.result import ResultWriter


def convert_to_numpy_pointcloud(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 message to numpy array."""
    numpy_pcd = ros2_numpy.numpify(msg)
    return np.stack((numpy_pcd["x"], numpy_pcd["y"], numpy_pcd["z"]), axis=-1)


class GroundSegmentationRunner(Runner):
    def __init__(
        self,
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_json_path: str,
        result_archive_path: str,
        storage: str,
        evaluation_topics: list[str],
        external_record_topics: list[TopicInfo],
        enable_analysis: str,
    ) -> None:
        simulation_info_list = [
            SimulationInfo(
                evaluation_manager_class=GroundSegmentationEvaluationManager,
                result_class=GroundSegmentationResult,
                name="ground_segmentation",
                evaluation_topics=evaluation_topics,
                result_json_path=result_json_path,
            ),
        ]
        super().__init__(
            GroundSegmentationScenario,
            simulation_info_list,
            scenario_path,
            rosbag_dir_path,
            t4_dataset_path,
            result_json_path,
            result_archive_path,
            storage,
            external_record_topics,
            enable_analysis,
        )

    @property
    def ground_seg_eval_manager(self) -> GroundSegmentationEvaluationManager:
        return self._simulation["ground_segmentation"].evaluation_manager

    @property
    def ground_seg_result(self) -> GroundSegmentationResult:
        return self._simulation["ground_segmentation"].result

    @property
    def ground_seg_result_writer(self) -> ResultWriter:
        return self._simulation["ground_segmentation"].result_writer

    def _evaluate_frame(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> FrameResult:
        # convert ros message to numpy array
        header_timestamp_microsec = eval_conversions.unix_time_microsec_from_ros_msg(msg.header)
        pointcloud = convert_to_numpy_pointcloud(msg)
        subscribed_timestamp_microsec = eval_conversions.unix_time_microsec_from_ros_time_nanosec(
            subscribed_timestamp_nanosec
        )
        return self.ground_seg_eval_manager.evaluate_frame(
            topic_name,
            header_timestamp_microsec,
            subscribed_timestamp_microsec,
            pointcloud,
        )

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
    result_json_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_topic: str,
    enable_analysis: str,
) -> None:
    evaluation_topics = [evaluation_topic]

    external_record_topics = [
        TopicInfo(
            name="/driving_log_replayer_v2/ground_segmentation/results",
            msg_type="std_msgs/msg/String",
        ),
    ]

    runner = GroundSegmentationRunner(
        scenario_path,
        rosbag_dir_path,
        t4_dataset_path,
        result_json_path,
        result_archive_path,
        storage,
        evaluation_topics,
        external_record_topics,
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
        args.result_json_path,
        args.result_archive_path,
        args.storage,
        args.evaluation_topic,
        args.enable_analysis,
    )


if __name__ == "__main__":
    main()
