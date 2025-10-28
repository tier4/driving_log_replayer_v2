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

from abc import ABC
from abc import abstractmethod
from pathlib import Path
from typing import Any
from typing import TypeVar

from pydantic import BaseModel
from pydantic import ValidationError
import yaml

from driving_log_replayer_v2.post_process_evaluator import Evaluator
from driving_log_replayer_v2.post_process_evaluator import FrameResult
from driving_log_replayer_v2.scenario import load_scenario
from driving_log_replayer_v2.scenario import Scenario

ScenarioType = TypeVar("ScenarioType", bound=Scenario)
EvaluatorType = TypeVar("EvaluatorType", bound=Evaluator)
BaseModelType = TypeVar("BaseModelType", bound=BaseModel)


class EvaluationManager(ABC):
    """
    Manager for evaluation process.

    - load scenario
    - manage multiple evaluators
    """

    def __init__(
        self,
        scenario_path: str,
        scenario_class: ScenarioType,
    ) -> None:
        self._scenario: ScenarioType
        self._evaluation_condition: BaseModelType
        self._evaluators: dict[str, EvaluatorType]
        self._degradation_topic: str

        try:
            self._scenario = load_scenario(Path(scenario_path), scenario_class)
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

    @abstractmethod
    def set_degradation_topic(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def evaluate_frame(
        self,
        topic_name: str,
        header_timestamp_microsec: int,
        subscribed_timestamp_microsec: int,
        data: Any,
        *args: Any,
        **kwargs: Any,
    ) -> FrameResult:
        raise NotImplementedError

    def get_degradation_topic(self) -> str:
        return self._degradation_topic

    def check_scenario_error(self) -> Exception | None:
        return self._error if hasattr(self, "_error") else None

    def get_evaluation_condition(self) -> dict[str, str]:
        return self._evaluation_condition

    def get_evaluation_topics(self) -> list[str]:
        return self._evaluators.keys()
