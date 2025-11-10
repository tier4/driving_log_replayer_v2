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
import re
from sys import float_info
from typing import Literal

from autoware_internal_planning_msgs.msg import PlanningFactor as PlanningFactorMsg
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from pydantic import BaseModel
from pydantic import Field
from pydantic import model_validator
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.diagnostics import Conditions as DiagnosticsConditions
from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import Scenario


def stamp_to_float(stamp: Time) -> float:
    return stamp.sec + (stamp.nanosec / 1e9)


def get_planning_factor_behavior_string(planning_factor: PlanningFactorMsg) -> str:
    behavior = planning_factor.behavior

    behavior_map = {
        PlanningFactorMsg.UNKNOWN: "UNKNOWN",
        PlanningFactorMsg.NONE: "NONE",
        PlanningFactorMsg.SLOW_DOWN: "SLOW_DOWN",
        PlanningFactorMsg.STOP: "STOP",
        PlanningFactorMsg.SHIFT_LEFT: "SHIFT_LEFT",
        PlanningFactorMsg.SHIFT_RIGHT: "SHIFT_RIGHT",
        PlanningFactorMsg.TURN_LEFT: "TURN_LEFT",
        PlanningFactorMsg.TURN_RIGHT: "TURN_RIGHT",
    }

    return behavior_map.get(behavior, f"UNKNOWN({behavior})")


class MinMax(BaseModel):
    min: float = -float_info.max
    max: float = float_info.max

    @model_validator(mode="after")
    def validate_min_max(self) -> "MinMax":
        err_msg = "max must be greater or equal to min"

        if self.max < self.min:
            raise ValueError(err_msg)
        return self

    def match_condition(self, value: float) -> bool:
        return self.min <= value <= self.max

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
    area_condition: Literal["inside", "outside"] = "inside"


class MetricCondition(BaseModel):
    condition_name: str | None = None
    topic: str
    metric_name: str
    value_type: Literal["string", "number"]
    value_range: MinMax | None = None
    value_target: str | None = None
    condition_type: Literal["any_of", "all_of"]


class PlanningFactorCondition(BaseModel):
    condition_name: str | None = None
    topic: str
    time: StartEnd
    condition_type: Literal["any_of", "all_of"]
    area: Area | None = None
    behavior: (
        list[
            Literal[
                "UNKNOWN",
                "NONE",
                "SLOW_DOWN",
                "STOP",
                "SHIFT_LEFT",
                "SHIFT_RIGHT",
                "TURN_LEFT",
                "TURN_RIGHT",
            ]
        ]
        | None
    ) = None
    distance: MinMax | None = None  # s of frenet coordinate
    judgement: Literal["positive", "negative"]  # positive or negative


class Conditions(BaseModel):
    MetricConditions: list[MetricCondition] = []
    PlanningFactorConditions: list[PlanningFactorCondition] = []


class Evaluation(BaseModel):
    UseCaseName: Literal["planning_control"]
    UseCaseFormatVersion: Literal["2.0.0", "2.1.0", "2.2.0", "2.3.0"]
    Conditions: Conditions
    Datasets: list[dict]


class IncludeUseCase(BaseModel):
    UseCaseName: Literal["diagnostics"]
    UseCaseFormatVersion: Literal["0.1.0", "0.2.0", "0.3.0"]
    Conditions: DiagnosticsConditions


class PlanningControlScenario(Scenario):
    Evaluation: Evaluation
    include_use_case: IncludeUseCase | None = None


@dataclass
class Metrics(EvaluationItem):
    def __post_init__(self) -> None:
        self.condition: MetricCondition

    def set_frame(self, value: str) -> dict | None:
        self.total += 1
        info_dict = {}
        condition_met = True

        if  self.condition.value_type == "number":
            float_pattern = r"^-?\d+(\.\d+)?$"
            int_pattern = r"^-?\d+$"
            if re.compile(int_pattern).match(str(value)):
                value = int(value)
            elif re.compile(float_pattern).match(str(value)):
                value = float(value)
            else:
                raise ValueError("Unexpected metric value type")
            info_dict = {
                "Value": str(value)
            }   
            info_dict.update(info_dict)
            condition_met &= self.condition.value_range.match_condition(value)
        elif self.condition.value_type == "string":
            target = str(self.condition.value_target)
            info_dict = {
                "Value": value
            }
            info_dict.update(info_dict)
            condition_met &= (value == target)

        if condition_met:
            self.passed += 1
        frame_success = "Success" if condition_met else "Fail"

        self.success = (
            self.passed > 0
            if self.condition.condition_type == "any_of"
            else self.passed == self.total
        )
        return {
            "Result": {"Total": self.success_str(), "Frame": frame_success},
            "Info": info_dict,
        }


class MetricsClassContainer:
    def __init__(self, conditions: list[MetricCondition]) -> None:
        self.__container: dict[str, list[Metrics]] = {}
        for i, cond in enumerate(conditions):
            condition_name = (
                cond.condition_name if cond.condition_name is not None else f"Condition_{i}"
            )
            self.__container.setdefault(cond.topic, []).append(Metrics(condition_name, cond))

    def set_frame(self, msg: MetricArray, topic: str) -> dict:
        frame_result: dict[int, dict] = {}
        for metric in self.__container.get(topic, []):
            for value in msg.metric_array:
                if value.name == metric.name:
                    topic_result = metric.set_frame(value.value)
                    if topic_result is not None:
                        frame_result[f"{metric.name}"] = topic_result
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTestTarget"]
        for _, metric_topic_items in self.__container.items():
            for metric_item in metric_topic_items:
                if not metric_item.success:
                    rtn_success = False
                    rtn_summary.append(f"{metric_item.name} (Fail)")
                else:
                    rtn_summary.append(f"{metric_item.name} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":" + ", ".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class MetricResult(ResultBase):
    def __init__(self, conditions: list[MetricCondition]) -> None:
        super().__init__()
        self.__metrics_container = MetricsClassContainer(conditions)

    def update(self) -> None:
        self._success, self._summary = self.__metrics_container.update()

    def set_frame(self, msg: MetricArray, topic: str) -> None:
        self._frame = self.__metrics_container.set_frame(msg, topic)
        self.update()


@dataclass
class PlanningFactor(EvaluationItem):
    def __post_init__(self) -> None:
        self.condition: PlanningFactorCondition
        self.success = self.condition.judgement == "negative"

    def set_frame(self, msg: PlanningFactorArray) -> dict | None:
        # check time condition
        if not self.condition.time.match_condition(stamp_to_float(msg.header.stamp)):
            return None

        self.total += 1

        if len(msg.factors) == 0:
            condition_met = self.condition.judgement == "negative"
            info_dict = {
                "Factor_0": "NO_FACTOR",
            }
        else:
            condition_met = False
            info_dict = {}

            for i, factor in enumerate(msg.factors):
                info_dict_per_factor = {}
                condition_met_per_factor = True

                if self.condition.area is not None:
                    in_range, info_dict_area = self.judge_in_range(factor.control_points[0].pose)
                    info_dict_per_factor.update(info_dict_area)
                    condition_met_per_factor &= (
                        in_range if self.condition.area.area_condition == "inside" else not in_range
                    )

                if self.condition.behavior is not None:
                    behavior_met, info_dict_behavior = self.judge_behavior(factor)
                    info_dict_per_factor.update(info_dict_behavior)
                    condition_met_per_factor &= behavior_met

                if self.condition.distance is not None:
                    distance_met, info_dict_distance = self.judge_distance(
                        factor.control_points[0].distance
                    )
                    info_dict_per_factor.update(info_dict_distance)
                    condition_met_per_factor &= distance_met

                condition_met_per_factor ^= self.condition.judgement == "negative"

                info_dict[f"Factor_{i}"] = info_dict_per_factor

                # Check if any factor in the current message meets the condition.
                # Note: 'any_of'/'all_of' applies across timestamps, not within a single message.
                condition_met |= condition_met_per_factor

        if condition_met:
            self.passed += 1
        frame_success = "Success" if condition_met else "Fail"

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

    def judge_behavior(self, msg: PlanningFactorMsg) -> tuple[bool, dict]:
        behavior = get_planning_factor_behavior_string(msg)
        info_dict = {
            "Behavior": behavior,
        }
        return behavior in self.condition.behavior, info_dict

    def judge_distance(self, distance: float) -> tuple[bool, dict]:
        info_dict = {
            "Distance": distance,
        }
        return self.condition.distance.match_condition(distance), info_dict


class FactorsClassContainer:
    def __init__(self, conditions: list[PlanningFactorCondition]) -> None:
        self.__container: dict[str, list[PlanningFactor]] = {}
        for i, cond in enumerate(conditions):
            condition_name = (
                cond.condition_name if cond.condition_name is not None else f"Condition_{i}"
            )
            self.__container.setdefault(cond.topic, []).append(PlanningFactor(condition_name, cond))

    def set_frame(self, msg: PlanningFactorArray, topic: str) -> dict:
        frame_result: dict[str, dict] = {}
        for factor in self.__container.get(topic, []):
            topic_result = factor.set_frame(msg)
            if topic_result is not None:
                frame_result[f"{factor.name}"] = topic_result
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTestTarget"]
        for _, pf_topic_items in self.__container.items():
            for evaluation_item in pf_topic_items:
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
