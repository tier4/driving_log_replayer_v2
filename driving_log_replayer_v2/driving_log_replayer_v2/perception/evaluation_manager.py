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

from driving_log_replayer_v2.perception.evaluator import PerceptionEvaluator
from driving_log_replayer_v2.post_process.evaluation_manager import EvaluationManager

if TYPE_CHECKING:
    from pathlib import Path

    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.tool import PerceptionAnalyzer3D

    from driving_log_replayer_v2.scenario import ScenarioType


class PerceptionEvaluationManager(EvaluationManager):
    ML_MODELS = ("centerpoint", "pointpainting", "transfusion", "apollo")

    def __init__(
        self,
        scenario: ScenarioType,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
    ) -> None:
        # additional instance variables
        self._degradation_evaluation_task: str = scenario.Evaluation.PerceptionEvaluationConfig[
            "evaluation_config_dict"
        ]["evaluation_task"]
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
        self._evaluators = {
            topic: PerceptionEvaluator(
                copy.deepcopy(self._scenario.Evaluation.PerceptionEvaluationConfig),
                copy.deepcopy(self._scenario.Evaluation.CriticalObjectFilterConfig),
                copy.deepcopy(self._scenario.Evaluation.PerceptionPassFailConfig),
                t4_dataset_path,
                result_archive_path,
                topic,
                task
                if self._degradation_evaluation_task != "fp_validation"
                else "fp_validation",  # NOTE: The t4dataset used in fp_validation is specialized, so it cannot be performed concurrently with other evaluation tasks.
                "base_link" if task == "detection" else "map",
            )
            for task, topics in evaluation_topics_with_task.items()
            for topic in topics
        }

    def _set_degradation_topics(self, degradation_topic: str) -> None:
        # NOTE: argument has higher priority than scenario setting
        if degradation_topic not in ("", "None"):
            self._degradation_topics = [degradation_topic]
            return
        if self._scenario.Evaluation.degradation_topic is not None:
            self._degradation_topics = [self._scenario.Evaluation.degradation_topic]
        # If degradation topic is not set, set it based on the evaluation task.
        elif self._degradation_evaluation_task in ["detection", "fp_validation"]:
            # Set degradation topic by ML model
            self._degradation_topics = [
                topic
                for topic in self.get_evaluation_topics()
                if any(model in topic.split("/") for model in self.ML_MODELS)
            ]
            if len(self._degradation_topics) > 1:
                self._degradation_topics = [self._degradation_topics[0]]
            if not self._degradation_topics:
                err_msg = f"Could not found the topic by ML model for degradation topic. Evaluation topics: {self.get_evaluation_topics()}"
                raise ValueError(err_msg)
        elif self._degradation_evaluation_task == "tracking":
            self._degradation_topics = ["/perception/object_recognition/tracking/objects"]
        elif self._degradation_evaluation_task == "prediction":
            self._degradation_topics = ["/perception/object_recognition/objects"]
        else:
            err_msg = f"Invalid evaluation task: {self._degradation_evaluation_task}"
            raise ValueError(err_msg)

    def get_degradation_evaluation_task(self) -> str:
        """Get the degradation evaluation task."""
        return self._degradation_evaluation_task

    def get_evaluation_config(
        self, topic_name: str | None = None
    ) -> PerceptionEvaluationConfig | dict[str, PerceptionEvaluationConfig]:
        """
        Get the PerceptionEvaluationConfig for each/specific topic.

        Args:
            topic_name (str | None): Name of the topic to get the evaluation config. If None, get all evaluation configs.

        Returns:
            PerceptionEvaluationConfig | dict[str, PerceptionEvaluationConfig]: The evaluation config(s).

        """
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_evaluation_config()
        return {
            topic: evaluator.get_evaluation_config()
            for topic, evaluator in self._evaluators.items()
        }

    def get_archive_path(self, topic_name: str | None = None) -> Path | dict[str, Path]:
        """
        Get the archive path for each/specific evaluation topic.

        Args:
            topic_name (str | None): Name of the topic to get the archive path. If None, get all archive paths.

        Returns:
            Path | dict[str, Path]: The archive path(s).

        """
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_archive_path()
        return {
            topic: evaluator.get_archive_path() for topic, evaluator in self._evaluators.items()
        }

    def get_evaluation_results(self, topic_name: str | None = None) -> dict | dict[str, dict]:
        """
        Get the evaluation results for each/specific evaluation topic. If called, frame results are also saved.

        Args:
            topic_name (str | None): Name of the topic to get the evaluation results. If None, get all evaluation results.

        Returns:
            dict | dict[str, dict]: The evaluation results.

        """
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
        """
        Get the PerceptionAnalyzer3D for each/specific evaluation topic.

        Args:
            topic_name (str | None): Name of the topic to get the analyzer. If None, get all analyzers.

        Returns:
            PerceptionAnalyzer3D | dict[str, PerceptionAnalyzer3D]: The analyzer(s).

        """
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_analyzer()
        return {topic: evaluator.get_analyzer() for topic, evaluator in self._evaluators.items()}
