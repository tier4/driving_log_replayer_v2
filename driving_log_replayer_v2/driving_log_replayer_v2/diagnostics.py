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
    condition_type: Literal[
        "any_of",
        "all_of",
        "duration_larger_than",
        "duration_less_than",
        "percentage_larger_than",
        "percentage_less_than",
    ] = "any_of"
    duration_threshold: float | None = Field(None, ge=0.0)
    percentage_threshold: float | None = Field(None, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def validate_condition_type(self) -> "DiagCondition":
        err_msg = "Threshold value duration_*_than and percentage_*_than is required for the selected condition_type"
        if self.condition_type == "duration_larger_than" and self.duration_threshold is None:
            raise ValueError(err_msg)
        if self.condition_type == "duration_less_than" and self.duration_threshold is None:
            raise ValueError(err_msg)
        if self.condition_type == "percentage_larger_than" and self.percentage_threshold is None:
            raise ValueError(err_msg)
        if self.condition_type == "percentage_less_than" and self.percentage_threshold is None:
            raise ValueError(err_msg)
        return self


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
    UseCaseFormatVersion: Literal["0.1.0", "0.2.0", "0.3.0"]
    Conditions: Conditions
    Datasets: list[dict]


class DiagnosticsScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Diag(EvaluationItem):
    def __post_init__(self) -> None:
        self.condition: DiagCondition
        self.last_start_time: float | None = None
        self.max_consecutive_duration: float = 0.0
        self.current_consecutive_duration: float = 0.0

    def set_frame(self, msg: DiagnosticArray) -> dict | None:
        current_time = stamp_to_float(msg.header.stamp)

        if not self.condition.time.match_condition(current_time):
            return None

        for status in msg.status:
            status: DiagnosticStatus
            if status.name != self.condition.name:
                continue

            self.total += 1
            level_str = get_diagnostic_level_string(status)

            status_match = level_str in self.condition.level

            if status_match:
                frame_success = "Success"
                self.passed += 1

                # update max duration
                if self.last_start_time is None:
                    self.last_start_time = current_time
                self.current_consecutive_duration = current_time - self.last_start_time
                self.max_consecutive_duration = max(
                    self.max_consecutive_duration, self.current_consecutive_duration
                )
            else:
                frame_success = "Fail"
                self.last_start_time = None
                self.current_consecutive_duration = 0.0

            self.success = self._check_success()
            self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

            return {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "TotalPassed": self.passed,
                    "Level": level_str,
                    "ConsecutiveDuration": self.current_consecutive_duration,
                },
            }
        # not match status.name
        return None

    def _check_success(self) -> bool:
        result = False
        if self.condition.condition_type == "any_of":
            result = self.passed > 0
        elif self.condition.condition_type == "all_of":
            result = self.passed == self.total
        elif self.condition.condition_type == "duration_larger_than":
            result = self.max_consecutive_duration > self.condition.duration_threshold
        elif self.condition.condition_type == "duration_less_than":
            result = self.max_consecutive_duration < self.condition.duration_threshold

        elif self.condition.condition_type == "percentage_larger_than":
            result = (
                (self.passed / self.total) > self.condition.percentage_threshold
                if self.total > 0
                else False
            )
        elif self.condition.condition_type == "percentage_less_than":
            result = (
                (self.passed / self.total) < self.condition.percentage_threshold
                if self.total > 0
                else True
            )
        return result


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
                prefix_str = "Failed: "
            else:
                prefix_str = "Passed: "
            rtn_summary.append(prefix_str + evaluation_item.summary)
        rtn_summary_str = ",".join(rtn_summary)
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
