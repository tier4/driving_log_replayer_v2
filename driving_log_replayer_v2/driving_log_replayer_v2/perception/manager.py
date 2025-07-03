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
    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
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
        self._degradation_evaluation_task: str
        self._evaluators: dict[str, PerceptionEvaluator]

        try:
            self._scenario = load_scenario(Path(scenario_path), PerceptionScenario)
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

        self._degradation_evaluation_task = self._scenario.Evaluation.PerceptionEvaluationConfig[
            "evaluation_config_dict"
        ]["evaluation_task"]

        if self._degradation_evaluation_task == "fp_validation":
            # If fp_validation is specified, use only detection topic as `fp_validation` evaluation task.
            self._evaluators = {
                topic: PerceptionEvaluator(
                    copy.deepcopy(self._scenario.Evaluation.PerceptionEvaluationConfig),
                    copy.deepcopy(self._scenario.Evaluation.CriticalObjectFilterConfig),
                    copy.deepcopy(self._scenario.Evaluation.PerceptionPassFailConfig),
                    t4_dataset_path,
                    result_archive_path,
                    topic,
                    "fp_validation",
                )
                for task, topics in evaluation_topics.items()
                for topic in topics
                if task == "detection"
            }
        else:
            self._evaluators = {
                topic: PerceptionEvaluator(
                    copy.deepcopy(self._scenario.Evaluation.PerceptionEvaluationConfig),
                    copy.deepcopy(self._scenario.Evaluation.CriticalObjectFilterConfig),
                    copy.deepcopy(self._scenario.Evaluation.PerceptionPassFailConfig),
                    t4_dataset_path,
                    result_archive_path,
                    topic,
                    task,
                )
                for task, topics in evaluation_topics.items()
                for topic in topics
            }

    def check_scenario_error(self) -> Exception | None:
        return self._error if hasattr(self, "_error") else None

    def get_evaluation_condition(self) -> dict[str, str]:
        return self._evaluation_condition

    def get_evaluation_topics(self) -> list[str]:
        return self._evaluators.keys()

    def get_degradation_evaluation_task(self) -> str:
        return self._degradation_evaluation_task

    def get_degradation_topic(self) -> str:
        # TODO: Define topic itself in the same line as Criterion in Conditions
        if self._degradation_evaluation_task in ["detection", "fp_validation"]:
            topic = "/perception/object_recognition/detection/objects"
        elif self._degradation_evaluation_task == "tracking":
            topic = "/perception/object_recognition/tracking/objects"
        elif self._degradation_evaluation_task == "prediction":
            topic = "/perception/object_recognition/objects"
        else:
            err_msg = f"Invalid evaluation task: {self._degradation_evaluation_task}"
            raise ValueError(err_msg)
        return topic

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

    def get_analyzers(
        self, topic_name: str | None = None
    ) -> PerceptionAnalyzer3D | dict[str, PerceptionAnalyzer3D]:
        if topic_name is not None:
            evaluator = self._evaluators[topic_name]
            return evaluator.get_analyzer()
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
