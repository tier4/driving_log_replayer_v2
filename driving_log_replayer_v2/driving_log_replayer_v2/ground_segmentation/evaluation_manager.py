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

from driving_log_replayer_v2.ground_segmentation.evaluator import GroundSegmentationEvaluator
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationScenario
from driving_log_replayer_v2.post_process.evaluation_manager import EvaluationManager


class GroundSegmentationEvaluationManager(EvaluationManager):
    def __init__(
        self,
        scenario: GroundSegmentationScenario,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
    ) -> None:
        super().__init__(
            scenario, t4_dataset_path, result_archive_path, evaluation_topics_with_task
        )

    def _set_evaluators(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
    ) -> None:
        evaluation_condition = self._scenario.Evaluation.Conditions
        evaluation_topics = [
            topic for topics in evaluation_topics_with_task.values() for topic in topics
        ]
        self._evaluators = {
            topic: GroundSegmentationEvaluator(
                t4_dataset_path,
                result_archive_path,
                topic,
                evaluation_condition,
            )
            for topic in evaluation_topics
        }

    def _set_degradation_topics(self) -> None:
        self._degradation_topics = [
            next(iter(self._evaluators.keys()))
        ]  # set first topic as degradation topic
