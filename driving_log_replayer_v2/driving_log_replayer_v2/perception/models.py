# Copyright (c) 2023 TIER IV.inc
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
import sys
from typing import Literal
from typing import TYPE_CHECKING

from pydantic import BaseModel
from pydantic import field_validator
from pydantic import model_validator

from driving_log_replayer_v2.criteria import PerceptionCriteria
from driving_log_replayer_v2.perception_eval_conversions import FrameDescriptionWriter
from driving_log_replayer_v2.perception_eval_conversions import summarize_pass_fail_result
from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import number
from driving_log_replayer_v2.scenario import Scenario

if TYPE_CHECKING:
    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult


class Region(BaseModel):
    x_position: tuple[float, float] | None = None
    y_position: tuple[float, float] | None = None

    @field_validator("x_position", "y_position", mode="before")
    @classmethod
    def validate_region_range(cls, v: str | None) -> tuple[number, number] | None:
        if v is None:
            return None

        err_non_specify_msg = "both min and max values must be specified."
        err_range_msg = (
            f"{v} is not valid distance range, expected ordering min-max with min < max."
        )

        s_lower, s_upper = v.split(",")

        if s_upper == "" or s_lower == "":
            raise ValueError(err_non_specify_msg)

        lower = float(s_lower)
        upper = float(s_upper)

        if lower >= upper:
            raise ValueError(err_range_msg)

        return (lower, upper)


class Filter(BaseModel):
    Distance: tuple[float, float] | None = None
    Region: Region | None
    # add filter condition here

    @field_validator("Distance", mode="before")
    @classmethod
    def validate_distance_range(cls, v: str | None) -> tuple[number, number] | None:
        if v is None:
            return None

        err_msg = f"{v} is not valid distance range, expected ordering min-max with min < max."

        s_lower, s_upper = v.split("-")
        if s_upper == "":
            s_upper = sys.float_info.max

        lower = float(s_lower)
        upper = float(s_upper)

        if lower >= upper:
            raise ValueError(err_msg)
        return (lower, upper)

    @field_validator("Region", mode="after")
    @classmethod
    def validate_region_value(cls, v: Region | None) -> Region | None:
        if v is None:
            return None
        if v.x_position is None and v.y_position is None:
            return None
        return v

    @model_validator(mode="before")
    @classmethod
    def set_region_default_value(cls, v: dict) -> dict:
        if "Region" not in v:
            v["Region"] = None
        return v

    @model_validator(mode="after")
    def validate_duplicate_filter(self) -> Filter:
        if self.Distance is not None and self.Region is not None:
            error_msg = "Distance and Region filter cannot be used at the same time."
            raise ValueError(error_msg)
        return self


class Criteria(BaseModel):
    PassRate: number
    CriteriaMethod: (
        Literal[
            "num_tp",
            "num_gt_tp",
            "label",
            "metrics_score",
            "metrics_score_maph",
            "velocity_x_error",
            "velocity_y_error",
            "speed_error",
            "yaw_error",
        ]
        | list[str]
        | None
    ) = None
    CriteriaLevel: (
        Literal["perfect", "hard", "normal", "easy"] | list[str] | number | list[number] | None
    ) = None
    Filter: Filter


class StopReasonEvaluation(BaseModel):
    """Configuration for stop reason evaluation."""
    start_time: number
    end_time: number
    base_stop_line_dist: dict[str, number]  # min and max values
    tolerance_interval: number
    pass_rate: number

    @field_validator("base_stop_line_dist")
    @classmethod
    def validate_base_stop_line_dist(cls, v: dict) -> dict:
        if "min" not in v or "max" not in v:
            raise ValueError("base_stop_line_dist must contain 'min' and 'max' values")
        if v["min"] >= v["max"]:
            raise ValueError("base_stop_line_dist min must be less than max")
        return v


class Conditions(BaseModel):
    Criterion: list[Criteria]
    StopReasonCriterion: dict[str, StopReasonEvaluation] | None = None


class Evaluation(BaseModel):
    UseCaseName: Literal["perception"]
    UseCaseFormatVersion: Literal["1.0.0", "1.1.0"]
    Datasets: list[dict]
    Conditions: Conditions
    PerceptionEvaluationConfig: dict
    CriticalObjectFilterConfig: dict
    PerceptionPassFailConfig: dict


class PerceptionScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class Perception(EvaluationItem):
    success: bool = True

    def __post_init__(self) -> None:
        self.criteria = PerceptionCriteria(
            methods=self.condition.CriteriaMethod,
            levels=self.condition.CriteriaLevel,
            filters=self.condition.Filter,
        )

    def set_frame(self, frame: PerceptionFrameResult) -> dict:
        frame_success = "Fail"
        # ret_frame might be filtered frame result or original frame result.
        result, ret_frame = self.criteria.get_result(frame)

        if result is None:
            self.no_gt_no_obj += 1
            return {"NoGTNoObj": self.no_gt_no_obj}
        if result.is_success():
            self.passed += 1
            frame_success = "Success"

        self.total += 1
        self.success: bool = self.rate() >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

        return {
            "PassFail": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": summarize_pass_fail_result(ret_frame.pass_fail_result),
            },
            "Objects": FrameDescriptionWriter.extract_pass_fail_objects_description(
                ret_frame.pass_fail_result,
            ),
        }


@dataclass
class StopReasonEvaluationItem(EvaluationItem):
    """Evaluation item for stop reason evaluation."""
    success: bool = True
    target_reason: str = ""
    start_time: float = 0.0
    end_time: float = 0.0
    min_distance: float = 0.0
    max_distance: float = 0.0
    tolerance_interval: float = 0.0
    pass_rate: float = 0.0
    # For tracking
    last_accepted_time: float = -float('inf')
    last_check_time: float = -float('inf')  # Last time we checked for timeout
    passed: int = 0
    total: int = 0
    per_frame_results: list[dict] = None

    def __post_init__(self) -> None:
        self.condition: StopReasonEvaluation
        self.target_reason = self.name
        self.start_time = self.condition.start_time
        self.end_time = self.condition.end_time
        self.min_distance = self.condition.base_stop_line_dist["min"]
        self.max_distance = self.condition.base_stop_line_dist["max"]
        self.tolerance_interval = self.condition.tolerance_interval
        self.pass_rate = self.condition.pass_rate
        self.last_accepted_time = -float('inf')
        self.last_check_time = -float('inf')
        self.passed = 0
        self.total = 0
        self.per_frame_results = []

    def set_frame(self, stop_reason_data: dict) -> dict:
        """Set frame result for stop reason evaluation."""
        frame_success = "Fail"
        reason = stop_reason_data.get("reason")
        timestamp = stop_reason_data.get("timestamp", 0.0)
        dist_to_stop_pos = stop_reason_data.get("dist_to_stop_pos", 0.0)
        # Only evaluate if reason matches and within time window
        if reason != self.target_reason or not (self.start_time <= timestamp <= self.end_time):
            self.per_frame_results.append({
                "StopReason": {
                    "Result": "Skip",
                    "Info": f"Not target or outside time window ({reason})"
                }
            })
            return {"StopReason": {"Result": "Skip", "Info": "Not target or outside time window"}}
        # Enforce tolerance interval
        if timestamp - self.last_accepted_time < self.tolerance_interval:
            self.per_frame_results.append({
                "StopReason": {
                    "Result": "Skip",
                    "Info": f"Within tolerance interval ({timestamp - self.last_accepted_time:.2f}s)"
                }
            })
            return {"StopReason": {"Result": "Skip", "Info": "Within tolerance interval"}}
        self.total += 1
        if self.min_distance <= dist_to_stop_pos <= self.max_distance:
            self.passed += 1
            frame_success = "Success"
        self.last_accepted_time = timestamp
        self.success = self.rate() >= self.pass_rate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"
        frame_result = {
            "StopReason": {
                "Result": {"Total": self.success_str(), "Frame": frame_success},
                "Info": {
                    "Reason": reason,
                    "Distance": dist_to_stop_pos,
                    "Timestamp": timestamp,
                },
            },
        }
        self.per_frame_results.append(frame_result)
        return frame_result

    def get_summary(self) -> str:
        return self.summary

    def get_pass_rate(self) -> float:
        return self.rate()

    def get_total(self) -> int:
        return self.total

    def get_passed(self) -> int:
        return self.passed

    def get_per_frame_results(self) -> list[dict]:
        return self.per_frame_results

    def check_timeout(self, current_time: float) -> dict | None:
        """Check if we've exceeded the tolerance interval without receiving the target stop reason."""
        # Only check if we're within the evaluation time window
        if not (self.start_time <= current_time <= self.end_time):
            return None
            
        # Only check if we haven't checked recently (avoid spam)
        if current_time - self.last_check_time < 0.1:  # Check at most every 0.1 seconds
            return None
            
        self.last_check_time = current_time
        
        # If we haven't received the target reason within tolerance_interval, it's a timeout
        if (self.last_accepted_time < self.start_time and 
            current_time - self.start_time >= self.tolerance_interval):
            self.total += 1
            self.success = self.rate() >= self.pass_rate
            self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"
            
            timeout_result = {
                "StopReason": {
                    "Result": {"Total": self.success_str(), "Frame": "Fail"},
                    "Info": {
                        "Reason": "TIMEOUT",
                        "Distance": 0.0,
                        "Timestamp": current_time,
                        "Message": f"No {self.target_reason} received within {self.tolerance_interval}s"
                    },
                },
            }
            self.per_frame_results.append(timeout_result)
            return timeout_result
            
        return None


class PerceptionResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__perception_criterion: list[Perception] = []
        for i, criteria in enumerate(condition.Criterion):
            self.__perception_criterion.append(
                Perception(name=f"criteria{i}", condition=criteria),
            )
        # Initialize stop reason evaluation items
        self.__stop_reason_criterion: list[StopReasonEvaluationItem] = []
        if condition.StopReasonCriterion:
            for reason_name, reason_config in condition.StopReasonCriterion.items():
                self.__stop_reason_criterion.append(
                    StopReasonEvaluationItem(name=reason_name, condition=reason_config),
                )

    def update(self) -> None:
        """Update success and summary."""
        success_list = []
        summary_list = []
        # Update perception criteria
        for criterion in self.__perception_criterion:
            criterion.update()
            success_list.append(criterion.success)
            summary_list.append(criterion.summary)
        # Update stop reason criteria
        for criterion in self.__stop_reason_criterion:
            criterion.update()
            success_list.append(criterion.success)
            summary_list.append(criterion.summary)
        # Add stop reason summary in required format
        for criterion in self.__stop_reason_criterion:
            summary_list.append(
                f"stop reason({criterion.success_str()}): {criterion.get_passed()} / {criterion.get_total()} -> {criterion.get_pass_rate():.2f}%"
            )
        self._success = all(success_list)
        self._summary = ", ".join(summary_list)

    def set_frame(
        self,
        frame: 'PerceptionFrameResult',
        skip: int,
        map_to_baselink: dict,
    ) -> None:
        """Set the result of one frame from the subscribe ros message."""
        self._frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameSkip": skip,
        }
        # Add perception evaluation results
        for criterion in self.__perception_criterion:
            criterion_result = criterion.set_frame(frame)
            self._frame[criterion.name] = criterion_result
        # Add per-frame stop reason results if any
        for criterion in self.__stop_reason_criterion:
            if criterion.per_frame_results:
                self._frame[criterion.name] = criterion.per_frame_results[-1]

    def set_stop_reason_frame(self, stop_reason_data: dict) -> None:
        """Set stop reason evaluation frame result."""
        for criterion in self.__stop_reason_criterion:
            criterion_result = criterion.set_frame(stop_reason_data)
            self._frame[criterion.name] = criterion_result

    def set_info_frame(self, msg: str, skip: int) -> None:
        """Set info frame when no ground truth or no objects."""
        self._frame = {"Info": msg, "FrameSkip": skip}

    def set_warn_frame(self, msg: str, skip: int) -> None:
        """Set warning frame when invalid estimated objects."""
        self._frame = {"Warn": msg, "FrameSkip": skip}

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}

    @property
    def stop_reason_evaluations(self) -> dict[str, StopReasonEvaluationItem]:
        """Get stop reason evaluations as a dictionary."""
        return {criterion.target_reason: criterion for criterion in self.__stop_reason_criterion}

    def check_stop_reason_timeouts(self, current_time: float) -> dict[str, dict] | None:
        """Check for timeouts in all stop reason evaluations."""
        timeout_results = {}
        has_timeout = False
        
        for criterion in self.__stop_reason_criterion:
            timeout_result = criterion.check_timeout(current_time)
            if timeout_result:
                timeout_results[criterion.name] = timeout_result
                has_timeout = True
                
        return timeout_results if has_timeout else None

