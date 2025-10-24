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

import numpy as np

from driving_log_replayer_v2.evaluation_manager import EvaluationManager
from driving_log_replayer_v2.ground_segmentation.evaluator import GroundSegmentationEvaluator
from driving_log_replayer_v2.ground_segmentation.models import GroundSegmentationScenario
from driving_log_replayer_v2.post_process_evaluator import FrameResult


class GroundSegmentationEvaluationManager(EvaluationManager):
    def __init__(
        self,
        scenario_path: str,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics: list[str],
    ) -> None:
        super().__init__(scenario_path, GroundSegmentationScenario)
        self._evaluators = {
            topic: GroundSegmentationEvaluator(
                t4_dataset_path,
                result_archive_path,
                topic,
                self._evaluation_condition,
            )
            for topic in evaluation_topics
        }

        self.set_degradation_topic()

    def set_degradation_topic(self) -> None:
        self._degradation_topic = next(iter(self._evaluators.keys()))

    def evaluate_frame(
        self, topic_name: str, header_timestamp_microsec: int, 
        subscribed_timestamp_microsec: int, data: np.ndarray
    ) -> FrameResult:
        evaluator = self._evaluators[topic_name]
        return evaluator.evaluate_frame(header_timestamp_microsec, subscribed_timestamp_microsec, data)
