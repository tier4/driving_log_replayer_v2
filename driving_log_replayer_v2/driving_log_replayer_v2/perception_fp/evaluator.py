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

import numpy as np
from perception_eval.common.object import DynamicObject

from driving_log_replayer_v2.perception.evaluator import PerceptionInvalidReason
from driving_log_replayer_v2.perception.runner import PerceptionEvalData
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult

if TYPE_CHECKING:
    from driving_log_replayer_v2.post_process.runner import ConvertedData


class PerceptionFPEvaluator(Evaluator):
    def __init__(
        self,
        result_archive_path: str,
        evaluation_topic: str,
    ) -> None:
        # additional instance variables
        self._skip_counter = 0

        super().__init__(result_archive_path, evaluation_topic)

    def evaluate_frame(
        self,
        converted_data: ConvertedData,
    ) -> FrameResult:
        if isinstance(converted_data.data, PerceptionEvalData):
            if not (
                isinstance(converted_data.data.estimated_objects, list)
                and all(
                    isinstance(obj, DynamicObject) for obj in converted_data.data.estimated_objects
                )
            ):
                self._skip_counter += 1
                self._logger.warning(
                    "Estimated objects is invalid for timestamp: %s",
                    converted_data.header_timestamp,
                )
                return FrameResult(
                    is_valid=False,
                    invalid_reason=PerceptionInvalidReason.INVALID_ESTIMATED_OBJECTS,
                    skip_counter=self._skip_counter,
                )
            data = converted_data.data.estimated_objects
        elif isinstance(converted_data.data, np.ndarray):
            data = converted_data.data
        return FrameResult(is_valid=True, data=data, skip_counter=self._skip_counter)
