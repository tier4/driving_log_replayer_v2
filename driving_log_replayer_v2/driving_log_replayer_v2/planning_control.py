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
import math
from sys import float_info
from typing import Literal

from autoware_internal_planning_msgs.msg import PlanningFactorArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator
from tier4_metric_msgs.msg import Metric
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.diagnostics import Conditions as DiagnosticsConditions
from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import Scenario


def stamp_to_float(stamp: Time) -> float:
    return stamp.sec + (stamp.nanosec / 1e9)


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

    def match_condition(
        self,
        lane_info: tuple[int, float, float],
        *,
        start_condition: bool = False,
    ) -> bool:
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
    def metric_lane_info(cls, msg: MetricArray) -> tuple[int, float, float]:
        # If conditions cannot be taken, return conditions that can never be met.
        lane_id, s, t = -1, float_info.max, 0.0
        for metric in msg.metric_array:
            metric: Metric
            if metric.name == "ego_lane_info/lane_id":
                lane_id = int(metric.value)
            if metric.name == "ego_lane_info/s":
                s = float(metric.value)
            if metric.name == "ego_lane_info/t":
                t = float(metric.value)
        return (lane_id, s, t)

    def is_started(self, lane_info_tuple: tuple[int, float, float]) -> bool:
        # Once True, do not change.
        if not self.started:
            self.started = self.start.match_condition(lane_info_tuple, start_condition=True)
        return self.started

    def is_ended(self, lane_info_tuple: tuple[int, float, float]) -> bool:
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
        vel, acc, jerk = 0.0, 0.0, 0.0
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
    condition_name: str | None = None
    topic: str
    name: str
    value: str
    condition_type: Literal["any_of", "all_of"]
    lane_condition: LaneCondition | None = None
    kinematic_condition: KinematicCondition | None = None


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


class Area(BaseModel):
    x: float
    y: float
    range: float = Field(gt=0.0)


class PlanningFactorCondition(BaseModel):
    condition_name: str | None = None
    topic: str
    time: StartEnd
    condition_type: Literal["any_of", "all_of"]
    area: Area
    judgement: Literal["positive", "negative"]  # positive or negative


class Conditions(BaseModel):
    MetricConditions: list[MetricCondition] = []
    PlanningFactorConditions: list[PlanningFactorCondition] = []


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["2.0.0"]
    Conditions: Conditions
    Datasets: list[dict]


class IncludeUseCase(BaseModel):
    UseCaseName: Literal["diagnostics"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: DiagnosticsConditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation
    include_use_case: IncludeUseCase | None = None


@dataclass
class Metrics(EvaluationItem):
    def __post_init__(self) -> None:
        self.condition: MetricCondition
        self.use_lane_condition = self.condition.lane_condition is not None
        self.use_kinematic_condition = self.condition.kinematic_condition is not None

    def set_frame(self, msg: MetricArray, control_metrics: MetricArray) -> dict | None:
        lane_info_tuple = LaneCondition.metric_lane_info(control_metrics)
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

        aeb_value: str = "none" if len(msg.metric_array) == 0 else msg.metric_array[0].value
        kinematic_state_tuple = KinematicCondition.metric_kinematic_state(control_metrics)

        # OK if decision matches and kinematic_state satisfies the condition
        if self.condition.value == aeb_value:
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
                "Decision": aeb_value,
                "LaneInfo": lane_info_tuple,
                "KinematicState": kinematic_state_tuple,
            },
        }


class MetricsClassContainer:
    def __init__(self, conditions: list[MetricCondition]) -> None:
        self.__container: list[Metrics] = []
        for i, cond in enumerate(conditions):
            condition_name = (
                cond.condition_name if cond.condition_name is not None else f"Condition_{i}"
            )
            self.__container.append(Metrics(condition_name, cond))

    def set_frame(self, msg: MetricArray, control_metrics: MetricArray) -> dict:
        frame_result: dict[int, dict] = {}
        for evaluation_item in self.__container:
            result_i = evaluation_item.set_frame(msg, control_metrics)
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


class MetricResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__metrics_container = MetricsClassContainer(condition.MetricConditions)

    def update(self) -> None:
        self._success, self._summary = self.__metrics_container.update()

    def set_frame(self, msg: MetricArray, control_metrics: MetricArray) -> None:
        self._frame = self.__metrics_container.set_frame(msg, control_metrics)
        self.update()


@dataclass
class PlanningFactor(EvaluationItem):
    def set_frame(self, msg: PlanningFactorArray) -> dict | None:
        self.condition: PlanningFactorCondition
        # check time condition
        if not self.condition.time.match_condition(stamp_to_float(msg.header.stamp)):
            return None
        if len(msg.factors) == 0:
            return None

        self.total += 1
        frame_success = "Fail"

        # get factors[0] # factorsはarrayになっているが、実際には1個しか入ってない。
        in_range, info_dict = self.judge_in_range(msg.factors[0].control_points[0].pose)

        # Check if the condition is met based on judgement type
        condition_met = in_range if self.condition.judgement == "positive" else not in_range
        if condition_met:
            frame_success = "Success"
            self.passed += 1

        self.success = (
            self.passed > 0
            if self.condition.condition_type == "any_of"
            else self.passed == self.total
        )
        return {
            "Result": {"Total": self.success_str(), "Frame": frame_success},
            "Info": info_dict,
        }

    def judge_in_range(self, msg: Pose) -> tuple[bool, dict]:
        control_point_pose_pos_x = msg.position.x
        control_point_pose_pos_y = msg.position.y
        distance = math.sqrt(
            (control_point_pose_pos_x - self.condition.area.x) ** 2
            + (control_point_pose_pos_y - self.condition.area.y) ** 2
        )
        info_dict = {
            "Distance": distance,
            "ControlPointPoseX": control_point_pose_pos_x,
            "ControlPointPoseY": control_point_pose_pos_y,
        }
        if distance <= self.condition.area.range:
            return True, info_dict
        return False, info_dict


class FactorsClassContainer:
    def __init__(self, conditions: list[PlanningFactorCondition]) -> None:
        self.__container: dict[PlanningFactor] = {}
        for i, cond in enumerate(conditions):
            self.__container[cond.topic] = PlanningFactor(f"Condition_{i}", cond)

    def set_frame(self, msg: PlanningFactorArray, topic: str) -> dict:
        frame_result: dict[str, dict] = {}
        topic_result = self.__container[topic].set_frame(msg)
        if topic_result is not None:
            frame_result[topic] = topic_result
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTestTarget"]
        for _, evaluation_item in self.__container.items():
            if not evaluation_item.success:
                rtn_success = False
                rtn_summary.append(f"{evaluation_item.name} (Fail)")
            else:
                rtn_summary.append(f"{evaluation_item.name} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":" + ", ".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class PlanningFactorResult(ResultBase):
    def __init__(self, conditions: list[PlanningFactorCondition]) -> None:
        super().__init__()
        self.__factors_container = FactorsClassContainer(conditions)

    def update(self) -> None:
        self._success, self._summary = self.__factors_container.update()

    def set_frame(self, msg: PlanningFactorArray, topic: str) -> None:
        self._frame = self.__factors_container.set_frame(msg, topic)
        self.update()
