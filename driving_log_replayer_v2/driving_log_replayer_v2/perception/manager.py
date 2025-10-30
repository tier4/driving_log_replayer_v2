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

import copy
from typing import TYPE_CHECKING

from driving_log_replayer_v2.evaluation_manager import EvaluationManager
from driving_log_replayer_v2.perception.evaluator import PerceptionEvaluator

if TYPE_CHECKING:
    from pathlib import Path

    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.tool import PerceptionAnalyzer3D

    from driving_log_replayer_v2.perception.runner import PerceptionEvalData
    from driving_log_replayer_v2.post_process_evaluator import FrameResult
    from driving_log_replayer_v2.scenario import ScenarioType


class PerceptionEvaluationManager(EvaluationManager):
    def __init__(
        self,
        scenario: ScenarioType,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics: dict[str, list[str]],
    ) -> None:
        # additional instance variables
        self._degradation_evaluation_task: str = scenario.Evaluation.PerceptionEvaluationConfig[
            "evaluation_config_dict"
        ]["evaluation_task"]
        super().__init__(scenario, t4_dataset_path, result_archive_path, evaluation_topics)

    def _set_evaluators(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics: dict[str, list[str]],
    ) -> None:
        self._evaluators = {
            topic: PerceptionEvaluator(
                copy.deepcopy(self._scenario.Evaluation.PerceptionEvaluationConfig),
                copy.deepcopy(self._scenario.Evaluation.CriticalObjectFilterConfig),
                copy.deepcopy(self._scenario.Evaluation.PerceptionPassFailConfig),
                t4_dataset_path,
                result_archive_path,
                topic,
                task if self._degradation_evaluation_task != "fp_validation" else "fp_validation",
                "base_link" if task == "detection" else "map",
            )
            for task, topics in evaluation_topics.items()
            for topic in topics
        }

    def _set_degradation_topic(self) -> None:
        if self._scenario.Evaluation.degradation_topic is not None:
            self._degradation_topic = self._scenario.Evaluation.degradation_topic
        # If degradation topic is not set, set it based on the evaluation task.
        elif self._degradation_evaluation_task in ["detection", "fp_validation"]:
            self._degradation_topic = "/perception/object_recognition/detection/objects"
        elif self._degradation_evaluation_task == "tracking":
            self._degradation_topic = "/perception/object_recognition/tracking/objects"
        elif self._degradation_evaluation_task == "prediction":
            self._degradation_topic = "/perception/object_recognition/objects"
        else:
            err_msg = f"Invalid evaluation task: {self._degradation_evaluation_task}"
            raise ValueError(err_msg)

    def evaluate_frame(
        self,
        topic_name: str,
        header_timestamp: int,
        subscribed_timestamp: int,
        data: PerceptionEvalData,
    ) -> FrameResult:
        evaluator = self._evaluators[topic_name]

        return evaluator.evaluate_frame(
            header_timestamp,
            subscribed_timestamp,
            data,
        )

    def get_degradation_evaluation_task(self) -> str:
        return self._degradation_evaluation_task

    def get_evaluation_config(
        self, topic_name: str | None = None
    ) -> PerceptionEvaluationConfig | dict[str, PerceptionEvaluationConfig]:
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_evaluation_config()
        return {
            topic: evaluator.get_evaluation_config()
            for topic, evaluator in self._evaluators.items()
        }

    def get_archive_path(self, topic_name: str | None = None) -> Path | dict[str, Path]:
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_archive_path()
        return {
            topic: evaluator.get_archive_path() for topic, evaluator in self._evaluators.items()
        }

    def get_evaluation_results(self, topic_name: str | None = None) -> dict | dict[str, dict]:
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_evaluation_results(save_frame_results=True)
        return {
            topic: evaluator.get_evaluation_results(save_frame_results=True)
            for topic, evaluator in self._evaluators.items()
        }

    def get_analyzer(
        self, topic_name: str | None = None
    ) -> PerceptionAnalyzer3D | dict[str, PerceptionAnalyzer3D]:
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_analyzer()
        return {topic: evaluator.get_analyzer() for topic, evaluator in self._evaluators.items()}
