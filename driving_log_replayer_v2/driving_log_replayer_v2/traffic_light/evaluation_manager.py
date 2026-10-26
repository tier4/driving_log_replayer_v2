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

from typing import TYPE_CHECKING

from driving_log_replayer_v2.post_process.evaluation_manager import EvaluationManager
from driving_log_replayer_v2.traffic_light.evaluator import TrafficLightEvaluator

if TYPE_CHECKING:
    from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
    from driving_log_replayer_v2.traffic_light.models import TrafficLightScenario


class TrafficLightEvaluationManager(EvaluationManager):
    def __init__(
        self,
        scenario: TrafficLightScenario,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        ignore_frames: str,
    ) -> None:
        super().__init__(
            scenario,
            t4_dataset_path,
            result_archive_path,
            evaluation_topics_with_task,
            degradation_topic,
            ignore_frames,
        )

    def _set_evaluators(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
        ignore_frames: IgnoreFrames,
    ) -> None:
        evaluation_topics = [
            topic for topics in evaluation_topics_with_task.values() for topic in topics
        ]
        self._evaluators = {
            topic: TrafficLightEvaluator(
                self._scenario.Evaluation.PerceptionEvaluationConfig,
                self._scenario.Evaluation.CriticalObjectFilterConfig,
                self._scenario.Evaluation.PerceptionPassFailConfig,
                t4_dataset_path,
                result_archive_path,
                topic,
                ignore_frames,
            )
            for topic in evaluation_topics
        }

    def _set_degradation_topics(self, degradation_topic: str) -> None:
        self._degradation_topics = [degradation_topic]

    def get_final_metrics(self) -> dict:
        return self._evaluators[self._degradation_topics[0]].get_final_metrics()
