# Copyright (c) 2022 TIER IV.inc
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
from sys import float_info
from typing import ClassVar
from typing import Literal

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator

from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import Scenario


class StartEnd(BaseModel):
    start: float = Field(0.0, ge=0.0)
    end: float = Field(float_info.max, ge=0.0)

    @model_validator(mode="after")
    def validate_start_end(self) -> "StartEnd":
        err_msg = "end must be a greater number than start"

        if self.end < self.start:
            raise ValueError(err_msg)
        return self


class DiagCondition(BaseModel):
    hardware_id: str
    name: str
    level: list[Literal["OK", "WARN", "ERROR", "STALE"]]
    time: StartEnd


class Conditions(BaseModel):
    DiagConditions: list[DiagCondition]
    target_hardware_ids: list[str] | None = None


class Evaluation(BaseModel):
    UseCaseName: Literal["diagnostics"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions
    Datasets: list[dict]


class DiagnosticsScenario(Scenario):
    Evaluation: Evaluation


class DiagnosticsResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()

    def update(self) -> None:
        tmp_success = self.__visibility.success
        tmp_summary = self.__visibility.summary + " Blockage:"
        for v in self.__blockages.values():
            tmp_summary += " " + v.summary
            if not v.success:
                tmp_success = False
        prefix_str = "Passed: " if tmp_success else "Failed: "
        self._success = tmp_success
        self._summary = prefix_str + tmp_summary

    def set_frame(self) -> None:
        # abstract method
        pass
