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
from typing import Literal

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator

from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import Scenario


def stamp_to_float(stamp: Time) -> float:
    return stamp.sec + (stamp.nanosec / 1e9)


def get_diagnostic_level_string(status: DiagnosticStatus) -> str:
    level = status.level

    level_map = {
        DiagnosticStatus.OK: "OK",
        DiagnosticStatus.WARN: "WARN",
        DiagnosticStatus.ERROR: "ERROR",
        DiagnosticStatus.STALE: "STALE",
    }

    return level_map.get(level, f"UNKNOWN({level})")


class StartEnd(BaseModel):
    start: float = Field(0.0, ge=0.0)
    end: float = Field(float_info.max, ge=0.0)

    @model_validator(mode="after")
    def validate_start_end(self) -> "StartEnd":
        err_msg = "end must be a greater number than start"

        if self.end < self.start:
            raise ValueError(err_msg)
        return self

    def match_condition(self, float_time: float) -> bool:
        return self.start <= float_time <= self.end


class DiagCondition(BaseModel):
    condition_name: str | None = None
    hardware_id: str
    name: str
    level: list[Literal["OK", "WARN", "ERROR", "STALE"]]
    time: StartEnd
    condition_type: Literal["any_of", "all_of"]


class Conditions(BaseModel):
    DiagConditions: list[DiagCondition]
    target_hardware_ids: list[str] = []

    @model_validator(mode="after")
    def validate_target_hardware_ids(self) -> "Conditions":
        err_msg = "No condition is set"

        for diag_condition in self.DiagConditions:
            self.target_hardware_ids.append(diag_condition.hardware_id)

        if self.target_hardware_ids == []:
            raise ValueError(err_msg)
        return self


class Evaluation(BaseModel):
    UseCaseName: Literal["diagnostics"]
    UseCaseFormatVersion: Literal["0.1.0", "0.2.0"]
    Conditions: Conditions
    Datasets: list[dict]


class DiagnosticsScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Diag(EvaluationItem):
    def set_frame(self, msg: DiagnosticArray) -> dict | None:
        self.condition: DiagCondition
        # check time condition
        if not self.condition.time.match_condition(stamp_to_float(msg.header.stamp)):
            return None
        for status in msg.status:
            status: DiagnosticStatus
            if status.name == self.condition.name:
                self.total += 1
                frame_success = "Fail"
                level_str = get_diagnostic_level_string(status)
                if level_str in self.condition.level:
                    frame_success = "Success"
                    self.passed += 1
                self.success = (
                    self.passed > 0
                    if self.condition.condition_type == "any_of"
                    else self.passed == self.total
                )
                return {
                    "Result": {"Total": self.success_str(), "Frame": frame_success},
                    "Info": {
                        "TotalPassed": self.passed,
                        "Level": level_str,
                    },
                }
        # not match status.name
        return None


class DiagClassContainer:
    def __init__(self, conditions: list[DiagCondition]) -> None:
        self.__container: list[Diag] = []
        for i, cond in enumerate(conditions):
            condition_name = (
                cond.condition_name if cond.condition_name is not None else f"Condition_{i}"
            )
            self.__container.append(Diag(condition_name, cond))

    def set_frame(self, msg: DiagnosticArray) -> dict:
        frame_result: dict[int, dict] = {}
        for evaluation_item in self.__container:
            result_i = evaluation_item.set_frame(msg)
            if result_i is not None:
                frame_result[f"{evaluation_item.name}"] = result_i
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTestTarget"]
        for evaluation_item in self.__container:
            if not evaluation_item.success:
                rtn_success = False
                rtn_summary.append(f"{evaluation_item.name} (Fail)")
            else:
                rtn_summary.append(f"{evaluation_item.name} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":" + ", ".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class DiagnosticsResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__diag_container = DiagClassContainer(condition.DiagConditions)

    def update(self) -> None:
        self._success, self._summary = self.__diag_container.update()

    def set_frame(self, msg: DiagnosticArray) -> None:
        self._frame = self.__diag_container.set_frame(msg)
        self.update()
