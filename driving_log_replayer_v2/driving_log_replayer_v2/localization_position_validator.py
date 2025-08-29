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

from dataclasses import dataclass
from functools import singledispatchmethod
from typing import Literal

from pydantic import BaseModel
from sensor_msgs.msg import PointCloud2
from tier4_localization_msgs.msg import LocalizedPositionValidatorPrediction

from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import number
from driving_log_replayer_v2.scenario import Scenario


class CheckInputAndOutputCountCondition(BaseModel):
    PassRate: number


class Conditions(BaseModel):
    CheckInputAndOutputCount: CheckInputAndOutputCountCondition


class Evaluation(BaseModel):
    UseCaseName: Literal["localization_position_validator"]
    UseCaseFormatVersion: Literal["2.0.0"]
    Conditions: Conditions
    Datasets: list[dict]


class LocalizationPositionValidatorScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class CheckInputAndOutputCount(EvaluationItem):
    """Check input and output topic has about same count."""

    name: str = "CheckInputAndOutputCount"

    def set_frame(
        self, *, prediction: bool, prediction_count: int, points_aligned_count: int
    ) -> dict:
        self.total += 1
        frame_success = "Fail"

        if points_aligned_count == prediction_count:
            self.passed += 1
            frame_success = "Success"

        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = (
            f"{self.name} ({self.success_str()}): "
            f"{self.passed} / {self.total} -> {self.rate():.2f}%"
        )

        return {
            "CheckInputAndOutputCount": {
                "Result": {
                    "Total": self.success_str(),
                    "Frame": frame_success,
                },
                "Prediction": {"is_position_valid": prediction},
                "Info": {
                    "InputCount": f"{points_aligned_count}",
                    "OutputCount": f"{prediction_count}",
                },
            }
        }


class LocalizationPositionValidatorResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__check_pred_count = CheckInputAndOutputCount(
            condition=condition.CheckInputAndOutputCount
        )

        # in case points aligned dropped
        self.__latest_points_aligned = None

        self.__prediction_started = False
        self.__input_topic_count = 0
        self.__output_topic_count = 0

    def update(self) -> None:
        summary_str = f"{self.__check_pred_count.summary}"

        if self.__check_pred_count.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    @singledispatchmethod
    def set_frame(self) -> None:
        raise NotImplementedError

    @set_frame.register
    def set_input_frame(self, msg: PointCloud2) -> None:
        if self.__prediction_started:
            self.__latest_points_aligned = msg
            self.__input_topic_count += 1

    @set_frame.register
    def set_prediction_frame(self, msg: LocalizedPositionValidatorPrediction) -> None:
        if self.__prediction_started:
            self.__latest_input_timestamp = msg.header.stamp
            self.__latest_prediction = msg
            self.__output_topic_count += 1

            self._frame = self.__check_pred_count.set_frame(
                prediction=msg.is_position_valid,
                prediction_count=self.__output_topic_count,
                points_aligned_count=self.__input_topic_count,
            )

            self.update()
        else:
            # input topic will be publish before model can start to predict
            # so we need wait until model can predict and discard the input topic
            self.__prediction_started = True
