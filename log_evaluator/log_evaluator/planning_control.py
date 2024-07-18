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
    min: float = float_info.min
    max: float = float_info.max

    @model_validator(mode="after")
    def validate_min_max(self) -> "MinMax":
        err_msg = "max must be a greater number than min"

        if self.max < self.min:
            raise ValueError(err_msg)
        return self


class LeftRight(BaseModel):
    left: float = Field(float_info.max, gt=0.0)  # +
    right: float = Field(float_info.max, gt=0.0)  # -

    def match_condition(self, t: float) -> bool:
        if (-1.0) * self.right <= t <= self.left:
            return True
        return False


class LaneInfo(BaseModel):
    id: int
    s: float | None = None
    t: LeftRight | None = None

    def match_condition(self, lane_info: tuple, *, start_condition: bool = False) -> bool:
        lane_id, s, t = lane_info
        if self.id != lane_id:
            return False
        if self.s is not None and self.s >= s:  # 超えたら開始、または終了
            return False
        if start_condition and self.t is not None and not self.t.match_condition(t):
            return False
        return True


class LaneCondition(BaseModel):
    start: LaneInfo
    end: LaneInfo
    started: bool = False
    ended: bool = False

    @classmethod
    def diag_lane_info(cls, lane_info: DiagnosticStatus) -> tuple[float, float, float]:
        lane_id, s, t = None, None, None
        for kv in lane_info.values:
            kv: KeyValue
            if kv.key == "lane_id":
                lane_id = int(kv.value)
            if kv.key == "s":
                s = float(kv.value)
            if kv.key == "t":
                t = float(kv.value)
        return (lane_id, s, t)

    def is_started(self, lane_info_tuple: tuple[float, float, float]) -> bool:
        # 一度Trueになったら変更しない
        if not self.started:
            self.started = self.start.match_condition(lane_info_tuple, start_condition=True)
        return self.started

    def is_ended(self, lane_info_tuple: tuple[float, float, float]) -> bool:
        # 一度Trueになったら変更しない
        if not self.ended:
            self.ended = self.end.match_condition(lane_info_tuple)
        return self.ended


class KinematicCondition(BaseModel):
    vel: MinMax
    acc: MinMax
    jerk: MinMax

    @classmethod
    def diag_kinematic_state(cls, kinematic_state: DiagnosticStatus) -> tuple[float, float, float]:
        vel, acc, jerk = None, None, None
        for kv in kinematic_state.values:
            kv: KeyValue
            if kv.key == "vel":
                vel = float(kv.value)
            if kv.key == "acc":
                acc = float(kv.value)
            if kv.key == "jerk":
                jerk = float(kv.value)
        return (vel, acc, jerk)

    def match_condition(self, kinematic_state_tuple: tuple[float, float, float]) -> bool:
        vel, acc, jerk = kinematic_state_tuple
        if not self.vel.min <= vel <= self.vel.max:
            return False
        if not self.acc.min <= acc <= self.acc.max:
            return False
        if not self.jerk.min <= jerk <= self.jerk.max:
            return False
        return True


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
    Datasets: list[dict]


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Metrics(EvaluationItem):
    def __post_init__(self) -> None:
        self.condition: PlanningControlCondition
        self.use_lane_condition = self.condition.lane_condition is not None
        self.use_kinetic_condition = self.condition.kinematic_condition is not None

    def set_frame(self, msg: DiagnosticArray) -> dict | None:  # noqa
        if len(msg.status) == 0:
            return None

        # temporary
        if len(msg.status) == 1:
            # to avoid local variable 'lane_info_tuple' referenced before assignment
            return None

        # key check
        status0: DiagnosticStatus = msg.status[0]
        if status0.name != self.condition.module:
            """
            return {
                "Error": f"{status0.name=}, {self.condition.module=} module name is not matched",
            }
            """
            return None
        if status0.values[0].key != "decision":
            return None

        # get additional condition
        for _, status in enumerate(msg.status, 1):
            status: DiagnosticStatus
            if status.name == "ego_lane_info":
                lane_info_tuple = LaneCondition.diag_lane_info(status)
            if status.name == "kinematic_state":
                kinetic_state_tuple = KinematicCondition.diag_kinematic_state(status)

        if self.use_lane_condition:
            started = self.condition.lane_condition.is_started(lane_info_tuple)
            ended = self.condition.lane_condition.is_ended(lane_info_tuple)
            if not (started and not ended):
                """
                return {
                    "Error": {
                        "LaneInfo": lane_info_tuple,
                        "KinematicState": kinetic_state_tuple,
                        "started": started,
                        "ended": ended,
                    },
                }
                """
                return None

        self.total += 1
        frame_success = "Fail"
        # decisionが一致している、且つkinetic_stateが条件を満たしていればOK
        if self.condition.decision == status0.values[0].value:
            if self.use_kinetic_condition:
                if self.condition.kinematic_condition.match_condition(kinetic_state_tuple):
                    frame_success = "Success"
                    self.passed += 1
            else:
                frame_success = "Success"
                self.passed += 1
        # any_ofなら1個あればいい。all_ofは全部
        self.success = (
            self.passed > 0
            if self.condition.condition_type == "any_of"
            else self.passed == self.total
        )
        return {
            "Result": {"Total": self.success_str(), "Frame": frame_success},
            "Info": {
                "TotalPassed": self.passed,
                "LaneInfo": lane_info_tuple,
                "KinematicState": kinetic_state_tuple,
            },
        }


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
