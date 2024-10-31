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
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator
from tier4_metric_msgs.msg import Metric
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import Scenario


class StartEnd(BaseModel):
    start: float = Field(float_info.max, ge=0.0)
    end: float = float_info.max

    @model_validator(mode="after")
    def validate_min_max(self) -> "StartEnd":
        err_msg = "end must be a greater number than start"

        if self.end < self.start:
            raise ValueError(err_msg)
        return self


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
        return (-1.0) * self.right <= t <= self.left


class LaneInfo(BaseModel):
    id: int
    s: float | None = None
    t: LeftRight | None = None

    def match_condition(self, lane_info: tuple, *, start_condition: bool = False) -> bool:
        lane_id, s, t = lane_info
        if self.id != lane_id:
            return False
        if self.s is not None and self.s >= s:  # Start or end when exceeded
            return False
        if start_condition and self.t is not None and not self.t.match_condition(t):  # noqa
            return False
        return True


class LaneCondition(BaseModel):
    start: LaneInfo
    end: LaneInfo
    started: bool = False
    ended: bool = False

    @classmethod
    def metric_lane_info(cls, msg: MetricArray) -> tuple[float, float, float]:
        lane_id, s, t = None, None, None
        for metric in msg.metric_array:
            metric: Metric
            if metric.name == "ego_lane_info/lane_id":
                lane_id = int(metric.value)
            if metric.name == "ego_lane_info/s":
                s = float(metric.value)
            if metric.name == "ego_lane_info/t":
                t = float(metric.value)
        return (lane_id, s, t)

    def is_started(self, lane_info_tuple: tuple[float, float, float]) -> bool:
        # Once True, do not change.
        if not self.started:
            self.started = self.start.match_condition(lane_info_tuple, start_condition=True)
        return self.started

    def is_ended(self, lane_info_tuple: tuple[float, float, float]) -> bool:
        # Once True, do not change.
        if not self.ended:
            self.ended = self.end.match_condition(lane_info_tuple)
        return self.ended


class KinematicCondition(BaseModel):
    vel: MinMax | None = None
    acc: MinMax | None = None
    jerk: MinMax | None = None

    @classmethod
    def metric_kinematic_state(cls, msg: MetricArray) -> tuple[float, float, float]:
        vel, acc, jerk = None, None, None
        for metric in msg.metric_array:
            metric: Metric
            if metric.name == "kinematic_state/vel":
                vel = float(metric.value)
            if metric.name == "kinematic_state/acc":
                acc = float(metric.value)
            if metric.name == "kinematic_state/jerk":
                jerk = float(metric.value)
        return (vel, acc, jerk)

    def match_condition(self, kinematic_state_tuple: tuple[float, float, float]) -> bool:
        vel, acc, jerk = kinematic_state_tuple
        if self.vel is not None and not self.vel.min <= vel <= self.vel.max:
            return False
        if self.acc is not None and not self.acc.min <= acc <= self.acc.max:
            return False
        if self.jerk is not None and not self.jerk.min <= jerk <= self.jerk.max:  # noqa
            return False
        return True


class MetricCondition(BaseModel):
    module: str
    name: str
    value: str
    condition_type: Literal["any_of", "all_of"]
    lane_condition: LaneCondition | None = None
    kinematic_condition: KinematicCondition | None = None


class DiagCondition(BaseModel):
    module: str
    level: list[Literal["OK", "WARN", "ERROR", "STALE"]]
    time: StartEnd | None = None


class Conditions(BaseModel):
    MetricConditions: list[MetricCondition] = []
    DiagConditions: list[DiagCondition] = []


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["1.0.0"]
    Conditions: Conditions
    Datasets: list[dict]


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Metrics(EvaluationItem):
    def __post_init__(self) -> None:
        self.condition: MetricCondition
        self.use_lane_condition = self.condition.lane_condition is not None
        self.use_kinematic_condition = self.condition.kinematic_condition is not None

    def set_frame(
        self, msg: MetricArray, planning_metrics: MetricArray, control_metrics: MetricArray
    ) -> dict | None:  # noqa
        if len(msg.metric_array) == 0:
            """
            return {
                "Error": "len(msg.metric_array) == 0",
            }
            """
            return None

        metric_array0: Metric = msg.metric_array[0]
        if metric_array0.name != self.condition.name:
            """
            return {
                "Error": f"{metric_array0.name=}, {self.condition.module=} module name is not matched",
            }
            """
            return None

        lane_info_tuple = None
        kinematic_state_tuple = None

        lane_info_tuple = LaneCondition.metric_lane_info(control_metrics)
        kinematic_state_tuple = KinematicCondition.diag_kinematic_state(control_metrics)

        if lane_info_tuple is None or kinematic_state_tuple is None:
            """
            return {"Error": "lane_info_tuple or kinematic_state_tuple is None"}
            """
            return None

        if self.use_lane_condition:
            started = self.condition.lane_condition.is_started(lane_info_tuple)
            ended = self.condition.lane_condition.is_ended(lane_info_tuple)
            if not (started and not ended):
                """
                return {
                    "Error": {
                        "LaneInfo": lane_info_tuple,
                        "KinematicState": kinematic_state_tuple,
                        "started": started,
                        "ended": ended,
                    },
                }
                """
                return None

        self.total += 1
        frame_success = "Fail"
        # OK if decision matches and kinematic_state satisfies the condition
        if self.condition.value == metric_array0.value:
            if self.use_kinematic_condition:
                if self.condition.kinematic_condition.match_condition(kinematic_state_tuple):
                    frame_success = "Success"
                    self.passed += 1
            else:
                frame_success = "Success"
                self.passed += 1
        # any_of would only need one, all_of would need all of them.
        self.success = (
            self.passed > 0
            if self.condition.condition_type == "any_of"
            else self.passed == self.total
        )
        return {
            "Result": {"Total": self.success_str(), "Frame": frame_success},
            "Info": {
                "TotalPassed": self.passed,
                "Decision": metric_array0.value,
                "LaneInfo": lane_info_tuple,
                "KinematicState": kinematic_state_tuple,
            },
        }


class MetricsClassContainer:
    def __init__(self, conditions: list[MetricCondition]) -> None:
        self.__container: list[Metrics] = []
        for i, module_cond in enumerate(conditions):
            self.__container.append(Metrics(f"Metrics_{i}", module_cond))

    def set_frame(
        self, msg: MetricArray, planning_metrics: MetricArray, control_metrics: MetricArray
    ) -> dict:
        frame_result: dict[int, dict] = {}
        for evaluation_item in self.__container:
            result_i = evaluation_item.set_frame(msg, planning_metrics, control_metrics)
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
        self.__metrics_container = MetricsClassContainer(condition.MetricConditions)
        # self.__diag_container = DiagClassContainer(condition.DiagConditions)

    def update(self) -> None:
        pass

    def set_frame(self) -> None:
        pass

    def set_metrics_frame(
        self, msg: MetricArray, planning_metrics: MetricArray, control_metrics: MetricArray
    ) -> None:
        self._frame = self.__metrics_container.set_frame(msg, planning_metrics, control_metrics)
        self.update()

    def set_diag_frame(self, msg: DiagnosticArray) -> None:
        pass
