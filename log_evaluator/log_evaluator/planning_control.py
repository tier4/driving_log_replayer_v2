# Copyright (c) 2024 TIER IV.inc
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
from dataclasses import field
from sys import float_info
from typing import Literal

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator

from log_evaluator.result import EvaluationItem
from log_evaluator.result import ResultBase
from log_evaluator.scenario import Scenario


class MinMax(BaseModel):
    min: float | None = float_info.min
    max: float | None = float_info.max

    @model_validator(mode="after")
    def validate_min_max(self) -> "MinMax":
        err_msg = "max must be a greater number than min"

        if self.max < self.min:
            raise ValueError(err_msg)
        return self


class LeftRight(BaseModel):
    left: float = Field(float_info.max, gt=0.0) # +
    right: float = Field(float_info.max, gt=0.0) # -

    def match_condition(self, t: float):
        if (-1)*self.right <= t <= self.left:
            return True
        return False


class LaneInfo(BaseModel):
    id: int
    s: float | None = None
    t: LeftRight | None = None

    def is_started(self, lane_info: tuple) -> bool:
        lane_id, s, t = lane_info
        if self.id != lane_id:
            return False
        if self.s is not None and self.s > s:
            return False
        if self.t is not None and not self.t.match_condition(t):
            return False
        return True

    def is_finished(self, lane_info: tuple) -> bool:
        lane_id, s, _ = lane_info
        if self.id != lane_id:
            return False
        if self.s is not None and self.s < s:
            return False
        return True


class LaneCondition(BaseModel):
    start: LaneInfo | None = None
    end: LaneInfo | None = None

    @classmethod
    def diag_lane_info(cls, lane_info: DiagnosticStatus) -> tuple[float, float, float]:
        for kv in lane_info.values:
            kv: KeyValue
            if kv.key == "lane_id":
                lane_id = kv.value
            if kv.key == "s":
                s = kv.value
            if kv.key == "t":
                t = kv.value
        return (lane_id, s, t)

    def match_condition(self, lane_info: DiagnosticStatus) -> bool:
        diag_lane_info = LaneCondition.diag_lane_info(lane_info)
        if not self.started:
            self.started = self.start.is_started(diag_lane_info)
        if not self.finished:
            self.finished = self.end.is_finished(diag_lane_info)
        if not self.finished:
            self.

    def is_started(self, lane_info: DiagnosticStatus) -> bool:
        self.start.is_started(lane_info)

    def is_finished(self, lane_info: DiagnosticStatus) -> bool:
        if self.end is None:
            return False
        return self.end.is_finished(lane_info)


class KinematicCondition(BaseModel):
    vel: MinMax
    acc: MinMax
    jerk: MinMax


class PlanningControlCondition(BaseModel):
    module: str
    decision: str
    condition_type: Literal["any_of", "all_of"]
    lane_condition: LaneCondition | None = None
    kinematic_condition: KinematicCondition | None = None


class Conditions(BaseModel):
    ControlConditions: list[PlanningControlCondition] = []
    PlanningConditions: list[PlanningControlCondition] = []


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Metrics(EvaluationItem):
    is_under_evaluation: bool = False

    def set_frame(self, msg: DiagnosticArray) -> dict | None:
        self.condition: PlanningControlCondition

        for status in msg.status:
            status: DiagnosticStatus
            if status.name != self.condition.module:
                continue
            if status.values[0].key != "decision":
                continue
            if status.values[0].value != self.condition.decision:
                continue

            self.total += 1

            frame_success = "Fail"
            if (self.condition.Value0Value == status.values[0].value) and (
                self.condition.DetailedConditions is None
                or self.check_detailed_condition(status.values[1:])
            ):
                frame_success = "Success"
                self.passed += 1
            self.success = self.passed >= required_successes
            return {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {"TotalPassed": self.passed, "RequiredSuccess": required_successes},
            }
        return None


class MetricsClassContainer:
    def __init__(self, conditions: list[PlanningControlCondition], module: str) -> None:
        self.__container: list[Metrics] = []
        for i, module_cond in enumerate(conditions):
            self.__container.append(Metrics(f"{module}_{i}", module_cond))

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


class PlanningControlResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__control_container = MetricsClassContainer(
            condition.ControlConditions,
            "control",
        )
        self.__planning_container = MetricsClassContainer(
            condition.PlanningConditions,
            "planning",
        )

    def update(self) -> None:
        control_success, control_summary = self.__control_container.update()
        planning_success, planning_summary = self.__planning_container.update()
        self._success = control_success and planning_success
        self._summary = "Control: " + control_summary + " Planning: " + planning_summary

    def set_frame(self, msg: DiagnosticArray, module: str) -> None:
        if module == "control":
            self._frame = self.__control_container.set_frame(msg)
        if module == "planning":
            self._frame = self.__planning_container.set_frame(msg)
        self.update()
