#!/usr/bin/env python3

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

from sensor_msgs.msg import PointCloud2
from tier4_localization_msgs.msg import LocalizedPositionValidatorPrediction

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.localization_position_validator import (
    LocalizationPositionValidatorResult,
)
from driving_log_replayer_v2.localization_position_validator import (
    LocalizationPositionValidatorScenario,
)


class LocalizationPositionValidatorEvaluator(DLREvaluatorV2):
    def __init__(self, name: str) -> None:
        super().__init__(
            name, LocalizationPositionValidatorScenario, LocalizationPositionValidatorResult
        )

        self._scenario: LocalizationPositionValidatorScenario
        self._result: LocalizationPositionValidatorResult

        # for counting the number that validator should infer
        self.__sub_points_aligned = self.create_subscription(
            PointCloud2,
            "/localization/pose_estimator/points_aligned",
            self.points_aligned_cb,
            1,
        )
        self.__sub_prediction = self.create_subscription(
            LocalizedPositionValidatorPrediction,
            "/localization/localized_position_validator/validation_result",
            self.validator_cb,
            1,
        )

    def points_aligned_cb(self, msg: PointCloud2) -> None:
        self._result.set_frame(msg)

    def validator_cb(self, msg: LocalizedPositionValidatorPrediction) -> None:
        self._result.set_frame(msg)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluatorV2:
    return LocalizationPositionValidatorEvaluator("localization_position_validator_evaluator")


if __name__ == "__main__":
    main()
