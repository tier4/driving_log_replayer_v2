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
from typing import Any
from typing import TYPE_CHECKING
from typing import TypeVar

if TYPE_CHECKING:
    from driving_log_replayer_v2.post_process_evaluator import EvaluatorType
    from driving_log_replayer_v2.post_process_evaluator import FrameResult
    from driving_log_replayer_v2.scenario import ScenarioType


class EvaluationManager(ABC):
    """
    Manager for evaluation process.

    - manage multiple evaluators
    """

    def __init__(
        self,
        scenario: ScenarioType,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics: list[str],
    ) -> None:
        self._scenario: ScenarioType = scenario
        self._evaluators: dict[str, EvaluatorType]
        self._degradation_topic: str

        self._set_evaluators(t4_dataset_path, result_archive_path, evaluation_topics)
        self._set_degradation_topic()

    @abstractmethod
    def _set_evaluators(
        self, t4_dataset_path: str, result_archive_path: str, evaluation_topics: list[str]
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def _set_degradation_topic(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def evaluate_frame(
        self,
        topic_name: str,
        header_timestamp: int,  # do not care time unit
        subscribed_timestamp: int,  # do not care time unit
        data: Any,
        *args: Any,
        **kwargs: Any,
    ) -> FrameResult:
        raise NotImplementedError

    def get_degradation_topic(self) -> str:
        return self._degradation_topic

    def get_evaluation_topics(self) -> list[str]:
        return self._evaluators.keys()


ManagerType = TypeVar("ManagerType", bound=EvaluationManager)
