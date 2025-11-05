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
from typing import TYPE_CHECKING
from typing import TypeVar

if TYPE_CHECKING:
    from driving_log_replayer_v2.post_process.evaluator import EvaluatorType
    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.post_process.runner import ConvertedData
    from driving_log_replayer_v2.scenario import ScenarioType


class EvaluationManager(ABC):
    """
    Base class for evaluation manager.

    Responsible for following items:
        initializing subclass of Evaluator for each evaluation topic
        managing evaluation process for each frame
    """

    def __init__(
        self,
        scenario: ScenarioType,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
    ) -> None:
        # instance variables
        self._scenario: ScenarioType = scenario
        self._evaluators: dict[str, EvaluatorType]
        self._degradation_topic: str

        self._set_evaluators(t4_dataset_path, result_archive_path, evaluation_topics_with_task)
        self._set_degradation_topic()

    @abstractmethod
    def _set_evaluators(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
    ) -> None:
        """
        Set evaluators for each evaluation topic.

        Args:
            t4_dataset_path (str): Path to T4 dataset.
            result_archive_path (str): Path to result archive.
            evaluation_topics_with_task (dict[str, list[str]]): Dictionary mapping evaluation topics to their tasks.

        """
        raise NotImplementedError

    @abstractmethod
    def _set_degradation_topic(self) -> None:
        """Set the degradation topic for the evaluation manager."""
        raise NotImplementedError

    def evaluate_frame(
        self,
        topic_name: str,
        converted_data: ConvertedData,
    ) -> FrameResult:
        """
        Evaluate a frame for a given topic.

        Args:
            topic_name (str): Name of the topic to evaluate.
            data (ConvertedData): Data to be evaluated.

        Returns:
            FrameResult: The result of the frame evaluation.

        """
        evaluator = self._evaluators[topic_name]
        return evaluator.evaluate_frame(converted_data)

    def get_degradation_topic(self) -> str:
        """Get the degradation topic for the evaluation manager."""
        return self._degradation_topic

    def get_evaluation_topics(self) -> list[str]:
        """Get the evaluation topics for the evaluation manager."""
        return list(self._evaluators.keys())


ManagerType = TypeVar("ManagerType", bound=EvaluationManager)
