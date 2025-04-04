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

from pathlib import Path
from typing import TYPE_CHECKING

from pydantic import ValidationError
import yaml

from driving_log_replayer_v2.perception.evaluator import PerceptionEvaluator
from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.scenario import load_scenario

if TYPE_CHECKING:
    from perception_eval.common.object import DynamicObject
    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.evaluation import PerceptionFrameResult
    from perception_eval.tool import PerceptionAnalyzer3D


class EvaluationManager:
    def __init__(
        self,
        scenario_path: str,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics: dict[str, list[str]],
    ) -> None:
        self._scenario: PerceptionScenario
        self._evaluation_condition: dict[str, str]
        self._evaluators: dict[str, PerceptionEvaluator]

        try:
            self._scenario: PerceptionScenario = load_scenario(
                Path(scenario_path), PerceptionScenario
            )
            evaluation_condition = {}
            if (
                hasattr(self._scenario.Evaluation, "Conditions")
                and self._scenario.Evaluation.Conditions is not None
            ):
                evaluation_condition = self._scenario.Evaluation.Conditions
            self._evaluation_condition = evaluation_condition
        except (FileNotFoundError, PermissionError, yaml.YAMLError, ValidationError) as e:
            self._error = e
            return

        self._evaluators: dict[str, PerceptionEvaluator] = {
            topic: PerceptionEvaluator(
                self._scenario.Evaluation.PerceptionEvaluationConfig,
                self._scenario.Evaluation.CriticalObjectFilterConfig,
                self._scenario.Evaluation.PerceptionPassFailConfig,
                t4_dataset_path,
                result_archive_path,
                topic,
                task,
            )
            for task, topics in evaluation_topics.items()
            for topic in topics
        }

    def check_error(self) -> Exception | None:
        return self._error if hasattr(self, "_error") else None

    def get_evaluation_condition(self) -> dict[str, str]:
        return self._evaluation_condition

    def get_evaluation_topics(self) -> list[str]:
        return self._evaluators.keys()

    def get_degradation_topic(self) -> str:
        return self._scenario.Evaluation.Conditions.DegradationTopic

    def get_evaluation_config(self, topic_name: str) -> PerceptionEvaluationConfig:
        evaluator = self._evaluators[topic_name]
        return evaluator.get_evaluation_config()

    def get_archive_path(self, topic_name: str) -> Path:
        return self._evaluators[topic_name].get_archive_path()

    def get_evaluation_results(self) -> dict[str, dict]:
        return {
            topic: evaluator.get_evaluation_results(save_frame_results=True)
            for topic, evaluator in self._evaluators.items()
        }

    def get_analyzers(self) -> dict[str, PerceptionAnalyzer3D]:
        return {topic: evaluator.get_analyzer() for topic, evaluator in self._evaluators.items()}

    def add_frame(
        self,
        topic_name: str,
        estimated_objects: list[DynamicObject] | str,
        header_unix_time: int,
        subscribed_unix_time: int,
        *,
        interpolation: bool,
    ) -> tuple[PerceptionFrameResult | str, int]:
        evaluator = self._evaluators[topic_name]

        frame_result, skip_counter = evaluator.add_frame(
            estimated_objects,
            header_unix_time,
            subscribed_unix_time,
            interpolation=interpolation,
        )

        return frame_result, skip_counter
