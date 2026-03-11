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

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import pprint
from sys import float_info
from typing import Annotated
from typing import Literal
from typing import TYPE_CHECKING
from typing import Union

from pydantic import BaseModel
from pydantic import Discriminator
from pydantic import Field
from pydantic import model_validator

if TYPE_CHECKING:
    from geometry_msgs.msg import AccelWithCovarianceStamped
    from nav_msgs.msg import Odometry

from driving_log_replayer_v2.diagnostics import DiagCondition as DiagConditionBase
from driving_log_replayer_v2.planning_control import Area
from driving_log_replayer_v2.planning_control import MetricCondition as MetricConditionBase
from driving_log_replayer_v2.planning_control import MinMax
from driving_log_replayer_v2.planning_control import (
    PlanningFactorCondition as PlanningFactorConditionBase,
)
from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import Scenario


def _validate_unique_group_names(groups: list[ConditionGroup], context: str) -> None:
    """Validate that group_name is unique within a list of groups."""
    group_names: set[str] = set()
    for group in groups:
        if group.group_name in group_names:
            err_msg = f"Duplicate group_name '{group.group_name}' found in {context}. Each group_name must be unique."
            raise ValueError(err_msg)
        group_names.add(group.group_name)


class PerceptionReproducerConfig(BaseModel):
    noise: bool = False
    reproduce_cool_down: float = Field(999.0, ge=0.0)
    tracked_object: bool = False
    search_radius: float = Field(1.5, ge=0.0)
    pub_route: bool = False


class _DeprecatedTimeField:
    @model_validator(mode="before")
    @classmethod
    def set_default_time(cls, data: dict) -> dict:
        if isinstance(data, dict):
            if "time" in data:
                warn_msg = (
                    f"Removed available time field for {cls.__name__}: {pprint.pformat(data)}"
                )
                logging.warning(warn_msg)
            data["time"] = {"start": 0.0, "end": float_info.max}
        return data


class MetricCondition(_DeprecatedTimeField, MetricConditionBase):
    condition_class: Literal["metric"] = "metric"


class DiagCondition(_DeprecatedTimeField, DiagConditionBase):
    condition_class: Literal["diagnostic"] = "diagnostic"


class PlanningFactorCondition(_DeprecatedTimeField, PlanningFactorConditionBase):
    condition_class: Literal["planning_factor"] = "planning_factor"


class EgoKinematicConditionBase(BaseModel):
    condition_class: Literal["ego_kinematic", "ego_kinematic_trigger"]
    condition_name: str | None = None
    area: Area | None = None
    velocity: MinMax | None = None
    acceleration: MinMax | None = None
    area_check_type: Literal["baselink", "footprint_overlap", "footprint_inside"] = "baselink"
    judgement: Literal["positive", "negative"] = "positive"


class EgoKinematicCondition(EgoKinematicConditionBase):
    condition_class: Literal["ego_kinematic"] = "ego_kinematic"


class EgoKinematicTriggerCondition(EgoKinematicConditionBase):
    condition_class: Literal["ego_kinematic_trigger"] = "ego_kinematic_trigger"
    condition_type: Literal["any_of"] = "any_of"


class TimeWaitTriggerCondition(BaseModel):
    condition_class: Literal["time_wait_trigger"] = "time_wait_trigger"
    condition_name: str | None = None
    wait_seconds: float = Field(ge=0.0)  # Number of seconds to wait


class ConditionGroup(BaseModel):
    """
    A condition group that can contain conditions or nested condition groups.

    start_at: Reference to another group_name. When that group passes (meets condition),
              this group starts evaluating. If null, starts at ENGAGED.
    end_at: Reference to another group_name. When that group passes, this group stops
            evaluating. If null, ends at timeout_s.
    Note: For nested condition groups, the child group's time window is automatically
          constrained by the parent group's time window.
    """

    condition_class: Literal["condition_group"] = "condition_group"
    group_name: str
    start_at: str | None = None  # Reference to another group_name or null (starts at ENGAGED)
    end_at: str | None = None  # Reference to another group_name or null (ends at timeout_s)
    group_type: Literal["any_of", "all_of"]
    condition_list: list[
        Annotated[
            Union[  # noqa: UP007
                ConditionGroup,
                EgoKinematicCondition,
                EgoKinematicTriggerCondition,
                TimeWaitTriggerCondition,
                MetricCondition,
                DiagCondition,
                PlanningFactorCondition,
            ],
            Discriminator("condition_class"),
        ]
    ] = []

    @model_validator(mode="after")
    def validate_nested_group_names(self) -> ConditionGroup:
        """Validate that nested ConditionGroups have unique group_name within this group."""
        nested_groups = [item for item in self.condition_list if isinstance(item, ConditionGroup)]
        _validate_unique_group_names(nested_groups, f"nested groups in '{self.group_name}'")
        return self


class Conditions(BaseModel):
    timeout_s: float = Field(120.0, ge=0.0)  # Timeout from ENGAGED
    terminated_after_fail_s: float = Field(10.0, ge=0.0)  # Terminate after fail condition triggers
    pass_conditions: list[ConditionGroup] = []
    fail_conditions: list[ConditionGroup] = []

    @model_validator(mode="after")
    def validate_condition_types(self) -> Conditions:
        # Available condition classes for validation
        pass_condition_classes: tuple[type, ...] = (
            EgoKinematicTriggerCondition,
            TimeWaitTriggerCondition,
            ConditionGroup,
        )
        fail_condition_classes: tuple[type, ...] = (
            EgoKinematicCondition,
            MetricCondition,
            DiagCondition,
            PlanningFactorCondition,
            ConditionGroup,
        )
        for group in self.pass_conditions:
            self._validate_condition_group(group, group.group_name, pass_condition_classes)
        for group in self.fail_conditions:
            self._validate_condition_group(group, group.group_name, fail_condition_classes)
        return self

    @model_validator(mode="after")
    def validate_group_names(self) -> Conditions:
        """Validate that group_name is unique."""
        _validate_unique_group_names(
            self.pass_conditions + self.fail_conditions, "pass_conditions and fail_conditions"
        )
        return self

    @classmethod
    def _validate_condition_group(
        cls,
        group: ConditionGroup,
        group_path: str,
        allowed_condition_classes: tuple[type, ...],
    ) -> None:
        for item in group.condition_list:
            if isinstance(item, allowed_condition_classes):
                if isinstance(item, ConditionGroup):
                    # Recursively validate nested groups
                    cls._validate_condition_group(
                        item, f"{group_path}.{item.group_name}", allowed_condition_classes
                    )
                continue

            condition_type = type(item).__name__
            allowed_classes_str = ", ".join(c.__name__ for c in allowed_condition_classes)
            err_msg = (
                f"Group '{group_path}' contains unsupported condition type: {condition_type}. "
                f"Only {allowed_classes_str} are allowed."
            )
            raise ValueError(err_msg)


class Evaluation(BaseModel):
    UseCaseName: Literal["perception_reproducer"]
    UseCaseFormatVersion: Literal["0.1.0"]
    perception_reproducer_config: PerceptionReproducerConfig
    Conditions: Conditions
    Datasets: list[dict]


class PerceptionReproducerScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class EgoKinematic(EvaluationItem):
    """Evaluate ego kinematic conditions (position, velocity, acceleration)."""

    def __post_init__(self) -> None:
        self.condition: EgoKinematicConditionBase

    def set_frame(
        self, msg: Odometry, acceleration_msg: AccelWithCovarianceStamped | None = None
    ) -> dict | None:
        if self.success and self.condition.condition_class == "ego_kinematic_trigger":
            return None

        self.total += 1

        # Get the kinematic state
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        longitudinal_velocity = msg.twist.twist.linear.x
        longitudinal_acceleration = (
            acceleration_msg.accel.accel.linear.x
            if acceleration_msg is not None
            else float_info.nan
        )
        result_info = {
            "PositionX": position_x,
            "PositionY": position_y,
            "Velocity": longitudinal_velocity,
            "Acceleration": longitudinal_acceleration,
        }

        # Check each sub condition
        condition_met = True

        if self.condition.area is not None:
            if self.condition.area_check_type == "baselink":
                distance_to_center = math.sqrt(
                    (position_x - self.condition.area.x) ** 2
                    + (position_y - self.condition.area.y) ** 2
                )
                if self.condition.area.area_condition == "inside":
                    distance_to_area = max(0.0, distance_to_center - self.condition.area.range)
                else:
                    distance_to_area = max(0.0, self.condition.area.range - distance_to_center)
            else:
                err_msg = f"area_check_type = {self.condition.area_check_type} is to be implemented"
                raise ValueError(err_msg)

            condition_met &= distance_to_area == 0.0
            result_info["DistanceToArea"] = distance_to_area

        if self.condition.velocity is not None:
            condition_met &= self.condition.velocity.match_condition(longitudinal_velocity)

        # Check acceleration condition
        if self.condition.acceleration is not None and not math.isnan(longitudinal_acceleration):
            condition_met &= self.condition.acceleration.match_condition(longitudinal_acceleration)

        condition_met ^= self.condition.judgement == "negative"

        # Update pass count and frame success
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
            "Info": result_info,
        }


class EgoKinematicClassContainer:
    def __init__(self, conditions: list[EgoKinematicConditionBase]) -> None:
        self.__container: list[EgoKinematic] = []
        for i, cond in enumerate(conditions):
            condition_name = (
                cond.condition_name if cond.condition_name is not None else f"Condition_{i}"
            )
            self.__container.append(EgoKinematic(condition_name, cond))

    def set_frame(
        self, msg: Odometry, acceleration_msg: AccelWithCovarianceStamped | None = None
    ) -> dict:
        frame_result: dict[str, dict] = {}
        for evaluation_item in self.__container:
            topic_result = evaluation_item.set_frame(msg, acceleration_msg)
            if topic_result is not None:
                frame_result[f"{evaluation_item.name}"] = topic_result
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTestTarget"]
        for kinematic_item in self.__container:
            if not kinematic_item.success:
                rtn_success = False
                rtn_summary.append(f"{kinematic_item.name} (Fail)")
            else:
                rtn_summary.append(f"{kinematic_item.name} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":" + ", ".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class EgoKinematicResult(ResultBase):
    def __init__(self, conditions: list[EgoKinematicConditionBase]) -> None:
        super().__init__()
        self.__kinematic_container = EgoKinematicClassContainer(conditions)

    def update(self) -> None:
        self._success, self._summary = self.__kinematic_container.update()

    def set_frame(
        self, msg: Odometry, acceleration_msg: AccelWithCovarianceStamped | None = None
    ) -> None:
        self._frame = self.__kinematic_container.set_frame(msg, acceleration_msg)
        self.update()


@dataclass
class TimeWait(EvaluationItem):
    """Evaluate time wait trigger conditions."""

    def __post_init__(self) -> None:
        self.condition: TimeWaitTriggerCondition
        self.activation_time: float | None = None

    def set_frame(self, current_time: float) -> dict | None:
        """Set frame with current time and check if wait condition is met."""
        # For trigger conditions, stop updating once successful
        if self.success and self.condition.condition_class == "time_wait_trigger":
            return None

        # Record activation time on first call
        if self.activation_time is None:
            self.activation_time = current_time
        self.total += 1

        elapsed_time = (
            current_time - self.activation_time if self.activation_time is not None else 0.0
        )
        condition_met = elapsed_time >= self.condition.wait_seconds

        if condition_met:
            self.passed += 1
        frame_success = "Success" if condition_met else "Fail"

        self.success = condition_met

        return {
            "Result": {"Total": self.success_str(), "Frame": frame_success},
            "Info": {
                "ElapsedTime": elapsed_time,
                "WaitSeconds": self.condition.wait_seconds,
                "ActivationTime": self.activation_time,
            },
        }


class TimeWaitClassContainer:
    def __init__(self, conditions: list[TimeWaitTriggerCondition]) -> None:
        self.__container: list[TimeWait] = []
        for i, cond in enumerate(conditions):
            condition_name = (
                cond.condition_name if cond.condition_name is not None else f"Condition_{i}"
            )
            self.__container.append(TimeWait(condition_name, cond))

    def set_frame(self, current_time: float) -> dict:
        frame_result: dict[str, dict] = {}
        for evaluation_item in self.__container:
            topic_result = evaluation_item.set_frame(current_time)
            if topic_result is not None:
                frame_result[f"{evaluation_item.name}"] = topic_result
        return frame_result

    def update(self) -> tuple[bool, str]:
        rtn_success = True
        rtn_summary = [] if len(self.__container) != 0 else ["NotTestTarget"]
        for time_wait_item in self.__container:
            if not time_wait_item.success:
                rtn_success = False
                rtn_summary.append(f"{time_wait_item.name} (Fail)")
            else:
                rtn_summary.append(f"{time_wait_item.name} (Success)")
        prefix_str = "Passed" if rtn_success else "Failed"
        rtn_summary_str = prefix_str + ":" + ", ".join(rtn_summary)
        return (rtn_success, rtn_summary_str)


class TimeWaitResult(ResultBase):
    def __init__(self, conditions: list[TimeWaitTriggerCondition]) -> None:
        super().__init__()
        self.__time_wait_container = TimeWaitClassContainer(conditions)

    def update(self) -> None:
        self._success, self._summary = self.__time_wait_container.update()

    def set_frame(self, current_time: float) -> None:
        self._frame = self.__time_wait_container.set_frame(current_time)
        self.update()
