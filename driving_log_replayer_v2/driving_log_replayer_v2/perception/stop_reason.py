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

from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path

import pandas as pd
from tier4_api_msgs.msg import AwapiAutowareStatus

from driving_log_replayer_v2.post_process.evaluation_manager import EvaluationManager
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.runner import ConvertedData
from driving_log_replayer_v2.scenario import ScenarioType


@dataclass(frozen=True, slots=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class Orientation:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True, slots=True)
class Reason:
    index: int
    reason: str
    dist_to_stop_pose: float
    position: Position
    orientation: Orientation


@dataclass(frozen=True, slots=True)
class StopReasonData:
    seconds: int
    nanoseconds: int
    reasons: list[Reason]


def convert_to_stop_reason(
    msg: AwapiAutowareStatus, subscribed_timestamp_nanosec: int
) -> ConvertedData:
    stop_reason = msg.stop_reason
    header_timestamp_nanosec = (
        stop_reason.header.stamp.sec * 10**9 + stop_reason.header.stamp.nanosec
    )
    return ConvertedData(
        header_timestamp=header_timestamp_nanosec,
        subscribed_timestamp=subscribed_timestamp_nanosec,
        data=StopReasonData(
            seconds=stop_reason.header.stamp.sec,
            nanoseconds=stop_reason.header.stamp.nanosec,
            reasons=[
                Reason(
                    index=i,
                    reason=reason.reason,
                    dist_to_stop_pose=reason.stop_factors[0].dist_to_stop_pose,
                    position=Position(
                        x=reason.stop_factors[0].stop_pose.position.x,
                        y=reason.stop_factors[0].stop_pose.position.y,
                        z=reason.stop_factors[0].stop_pose.position.z,
                    ),
                    orientation=Orientation(
                        x=reason.stop_factors[0].stop_pose.orientation.x,
                        y=reason.stop_factors[0].stop_pose.orientation.y,
                        z=reason.stop_factors[0].stop_pose.orientation.z,
                        w=reason.stop_factors[0].stop_pose.orientation.w,
                    ),
                )
                for i, reason in enumerate(stop_reason.stop_reasons)
            ],
        ),
    )


class StopReasonAnalyzer:
    def __init__(self, result_archive_path: str, evaluation_topic: str) -> None:
        self._data: list[StopReasonData] = []
        self._result_archive_path_w_topic_path = Path(result_archive_path).joinpath(
            evaluation_topic
        )
        self._result_archive_path_w_topic_path.mkdir(parents=True, exist_ok=True)

    def append(self, stop_reason: StopReasonData) -> None:
        self._data.append(stop_reason)

    def save_as_csv(self) -> None:
        data_dict = [asdict(data) for data in self._data]
        pd.DataFrame(data_dict).to_csv(
            self._result_archive_path_w_topic_path.joinpath("stop_reason.csv"), index=False
        )


class StopReasonEvaluationManager(EvaluationManager):
    def __init__(
        self,
        scenario: ScenarioType,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topic: dict[str, list[str]],
    ) -> None:
        super().__init__(scenario, t4_dataset_path, result_archive_path, evaluation_topic)

    def _set_evaluators(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
    ) -> None:
        _ = t4_dataset_path  # unused
        evaluation_topics = [
            topic for topics in evaluation_topics_with_task.values() for topic in topics
        ]
        self._evaluators = {
            topic: StopReasonEvaluator(result_archive_path, topic) for topic in evaluation_topics
        }

    def _set_degradation_topic(self) -> None:
        self._degradation_topic = next(
            iter(self._evaluators.keys())
        )  # set first topic as degradation topic

    def analyze(self, topic_name: str | None = None) -> None:
        """
        Analyze and save results as CSV.

        Args:
            topic_name (str | None): Name of the topic to analyze. If None, analyze all topics.

        """
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            evaluator.save_as_csv()
            return
        for evaluator in self._evaluators.values():
            evaluator.save_as_csv()


class StopReasonEvaluator(Evaluator):
    def __init__(self, result_archive_path: str, evaluation_topic: str) -> None:
        super().__init__(result_archive_path, evaluation_topic)
        dir_name = evaluation_topic.lstrip("/").replace("/", ".")
        self._analyzer = StopReasonAnalyzer(result_archive_path, dir_name)

    def evaluate_frame(
        self,
        converted_data: ConvertedData,
    ) -> FrameResult:
        self._analyzer.append(converted_data.data)
        return FrameResult(is_valid=True, data=converted_data.data, skip_counter=0)

    def save_as_csv(self) -> None:
        self._analyzer.save_as_csv()
