# Copyright (c) 2026 TIER IV.inc
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
from dataclasses import dataclass
from typing import TYPE_CHECKING

from std_msgs.msg import String

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.perception_eval_conversions import (
    unix_time_microsec_from_ros_timestamp,
)
from driving_log_replayer_v2.post_process.ros2_utils import get_topic_metadata
from driving_log_replayer_v2.post_process.ros2_utils import lookup_transform
from driving_log_replayer_v2.post_process.runner import ConvertedData
from driving_log_replayer_v2.post_process.runner import Runner
from driving_log_replayer_v2.post_process.runner import UseCaseInfo
from driving_log_replayer_v2.traffic_light.evaluation_manager import TrafficLightEvaluationManager
from driving_log_replayer_v2.traffic_light.models import TrafficLightResult
from driving_log_replayer_v2.traffic_light.models import TrafficLightScenario

if TYPE_CHECKING:
    from autoware_perception_msgs.msg import TrafficLightGroup
    from autoware_perception_msgs.msg import TrafficLightGroupArray
    from geometry_msgs.msg import TransformStamped
    from rosbag2_py import TopicMetadata
    from std_msgs.msg import Header

    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.result import ResultWriter

DEFAULT_DEGRADATION_TOPIC = "/perception/traffic_light_recognition/traffic_signals"
RESULT_TOPIC = "/driving_log_replayer_v2/traffic_light/results"


@dataclass(frozen=True, slots=True)
class TrafficLightData:
    traffic_light_groups: list[TrafficLightGroup]
    map_to_baselink: TransformStamped


class TrafficLightRunner(Runner):
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
        ignore_frames: str,
        enable_analysis: str,
    ) -> None:
        super().__init__(
            TrafficLightScenario,
            scenario_path,
            rosbag_dir_path,
            t4_dataset_path,
            result_jsonl_path,
            result_archive_path,
            storage,
            evaluation_topics_with_task,
            degradation_topic,
            ignore_frames,
            enable_analysis,
        )

    def _get_use_case_info_list(
        self,
        scenario: TrafficLightScenario,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        result_jsonl_path: str,
        result_archive_path: str,
        ignore_frames: str,
    ) -> list[UseCaseInfo]:
        _ = result_archive_path  # unused
        # argument has higher priority than scenario setting
        if degradation_topic in ("", "None"):
            degradation_topic = scenario.Evaluation.degradation_topic or DEFAULT_DEGRADATION_TOPIC
        if not evaluation_topics_with_task:
            evaluation_topics_with_task = {"classification2d": [degradation_topic]}
        return [
            UseCaseInfo(
                evaluation_manager_class=TrafficLightEvaluationManager,
                result_class=TrafficLightResult,
                conditions=scenario.Evaluation.Conditions,
                name="traffic_light",
                evaluation_topics_with_task=evaluation_topics_with_task,
                degradation_topic=degradation_topic,
                result_jsonl_path=result_jsonl_path,
                ignore_frames=ignore_frames,
            )
        ]

    def _get_external_record_topics(self) -> list[TopicMetadata]:
        return [get_topic_metadata(name=RESULT_TOPIC, topic_type="std_msgs/msg/String")]

    @property
    def traffic_light_eval_manager(self) -> TrafficLightEvaluationManager:
        return self._use_cases["traffic_light"].evaluation_manager

    @property
    def traffic_light_result(self) -> TrafficLightResult:
        return self._use_cases["traffic_light"].result

    @property
    def traffic_light_result_writer(self) -> ResultWriter[TrafficLightResult]:
        return self._use_cases["traffic_light"].result_writer

    def _convert_ros_msg_to_data(
        self,
        topic_name: str,
        msg: TrafficLightGroupArray,
        subscribed_timestamp_nanosec: int,
    ) -> ConvertedData:
        _ = topic_name  # unused
        return ConvertedData(
            header_timestamp=unix_time_microsec_from_ros_timestamp(msg.stamp),
            subscribed_timestamp=subscribed_timestamp_nanosec // 1000,
            data=TrafficLightData(
                traffic_light_groups=msg.traffic_light_groups,
                map_to_baselink=lookup_transform(self._rosbag_manager.get_tf_buffer(), msg.stamp),
            ),
        )

    def _write_result(
        self,
        frame_result: FrameResult,
        header: Header,
        subscribed_timestamp_nanosec: int,
    ) -> None:
        # NOTE: same as the previous evaluator node, the frame without ground truth is not written
        if not frame_result.is_valid:
            return
        map_to_baselink = lookup_transform(self._rosbag_manager.get_tf_buffer(), header.stamp)
        self.traffic_light_result.set_frame(
            frame_result.data,
            frame_result.skip_counter,
            DLREvaluatorV2.transform_stamped_with_euler_angle(map_to_baselink),
        )
        res_str = self.traffic_light_result_writer.write_result_with_time(
            self.traffic_light_result, subscribed_timestamp_nanosec
        )
        self._rosbag_manager.write_results(RESULT_TOPIC, String(data=res_str), header.stamp)

    def _evaluate_on_post_process(self) -> None:
        self.traffic_light_result.set_final_metrics(
            self.traffic_light_eval_manager.get_final_metrics()
        )
        res_str = self.traffic_light_result_writer.write_result_with_time(
            self.traffic_light_result,
            self._rosbag_manager.get_last_subscribed_timestamp(),
        )
        self._rosbag_manager.write_results(
            RESULT_TOPIC,
            String(data=res_str),
            self._rosbag_manager.get_last_subscribed_timestamp(),
        )

    def _analysis(self) -> None:
        pass


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4_dataset_path: str,
    result_jsonl_path: str,
    result_archive_path: str,
    storage: str,
    degradation_topic: str,
    enable_analysis: str,
) -> None:
    runner = TrafficLightRunner(
        scenario_path,
        rosbag_dir_path,
        t4_dataset_path,
        result_jsonl_path,
        result_archive_path,
        storage,
        {},
        degradation_topic,
        "",
        enable_analysis,
    )

    runner.evaluate()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate traffic light recognition rosbag")
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
        "--degradation-topic",
        default="",
        help="ROS traffic signals topic name to evaluate. If not set, degradation_topic of the scenario is used.",
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
        args.degradation_topic,
        args.enable_analysis,
    )


if __name__ == "__main__":
    main()
