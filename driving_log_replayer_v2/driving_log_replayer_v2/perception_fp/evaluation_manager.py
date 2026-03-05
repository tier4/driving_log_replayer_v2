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

from driving_log_replayer_v2.perception_fp.evaluator import PerceptionFPEvaluator
from driving_log_replayer_v2.perception_fp.models import PerceptionFPScenario
from driving_log_replayer_v2.post_process.evaluation_manager import EvaluationManager


class PerceptionFPEvaluationManager(EvaluationManager):
    def __init__(
        self,
        scenario: PerceptionFPScenario,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
    ) -> None:
        super().__init__(
            scenario,
            t4_dataset_path,
            result_archive_path,
            evaluation_topics_with_task,
            degradation_topic,
        )

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
            topic: PerceptionFPEvaluator(
                result_archive_path,
                topic,
            )
            for topic in evaluation_topics
        }

    def _set_degradation_topics(self, degradation_topic: str) -> None:
        # For perception_fp, all topics are considered degradation topics.
        _ = degradation_topic  # unused
        self._degradation_topics = list(self._evaluators.keys())
