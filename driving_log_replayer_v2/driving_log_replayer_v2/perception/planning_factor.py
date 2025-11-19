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
from typing import Any

from driving_log_replayer_v2.post_process.evaluation_manager import EvaluationManager
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.runner import ConvertedData
from driving_log_replayer_v2.scenario import ScenarioType


@dataclass(frozen=True, slots=True)
class PlanningFactorEvalData:
    topic_name: str
    msg: Any  # use ROS message directly in result


def convert_to_planning_factor(
    msg: Any,
    subscribed_timestamp_nanosec: int,
    topic_name: str,
) -> ConvertedData:
    header_timestamp_nanosec = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    return ConvertedData(
        header_timestamp=header_timestamp_nanosec,
        subscribed_timestamp=subscribed_timestamp_nanosec,
        data=PlanningFactorEvalData(topic_name=topic_name, msg=msg),
    )


class PlanningFactorEvaluationManager(EvaluationManager):
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
            topic: PlanningFactorEvaluator(result_archive_path, topic)
            for topic in evaluation_topics
        }

    def _set_degradation_topics(self) -> None:
        # For planning factor evaluation, all topics are considered degradation topics.
        self._degradation_topics = list(self._evaluators.keys())


class PlanningFactorEvaluator(Evaluator):
    def __init__(self, result_archive_path: str, evaluation_topic: str) -> None:
        super().__init__(result_archive_path, evaluation_topic)

    def evaluate_frame(
        self,
        converted_data: ConvertedData,
    ) -> FrameResult:
        return FrameResult(is_valid=True, data=converted_data.data, skip_counter=0)
