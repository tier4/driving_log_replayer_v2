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

from abc import ABC
from abc import abstractmethod
import logging
from typing import TYPE_CHECKING

import numpy as np
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.evaluation.matching import MatchingMode
from perception_eval.tool.utils import filter_frame_by_distance
from perception_eval.tool.utils import filter_frame_by_region

from driving_log_replayer_v2.criteria.common import CriteriaLevel
from driving_log_replayer_v2.criteria.common import CriteriaMethod
from driving_log_replayer_v2.criteria.common import load_levels
from driving_log_replayer_v2.criteria.common import load_methods
from driving_log_replayer_v2.criteria.common import SuccessFail

if TYPE_CHECKING:
    from numbers import Number

    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult

    from driving_log_replayer_v2.perception import Filter


class CriteriaMethodImpl(ABC):
    """
    Class to define implementation for each criteria.

    Args:
    ----
        level (CriteriaLevel): Level of criteria.

    """

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__()
        self.level: CriteriaLevel = level

    def get_result(self, frame: PerceptionFrameResult) -> tuple[SuccessFail, float] | None:
        """
        Return `SuccessFail` instance from the frame result.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            SuccessFail: Success or fail.

        """
        # No ground truth and No result is considered as Not Available, return None.
        if self.has_objects(frame) is False:
            return None
        score: float = self.calculate_score(frame)
        return (
            (SuccessFail.SUCCESS, score)
            if self.level.is_valid(score, is_error=self.is_error)
            else (SuccessFail.FAIL, score)
        )

    @staticmethod
    def has_objects(frame: PerceptionFrameResult) -> bool:
        """
        Return whether the frame result contains at least one objects.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            bool: Whether the frame result has objects is.

        """
        num_success: int = frame.pass_fail_result.get_num_success()
        num_fail: int = frame.pass_fail_result.get_num_fail()
        return num_success + num_fail > 0

    @staticmethod
    @abstractmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        """
        Calculate score depending on the method.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            float: Calculated score.

        """

    @property
    @abstractmethod
    def is_error(self) -> bool:
        """
        Indicates whether this criteria calculates error or not.

        Returns
        -------
            bool: True means it is valid if score <= threshold .

        """


class NumTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_TP

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        num_success: int = frame.pass_fail_result.get_num_success()
        num_objects: int = num_success + frame.pass_fail_result.get_num_fail()
        return 100.0 * num_success / num_objects if num_objects != 0 else 100.0

    @property
    def is_error(self) -> bool:
        return False


class NumGtTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_GT_TP

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        num_success: int = frame.pass_fail_result.get_num_success()
        num_gt: int = frame.pass_fail_result.get_num_gt()

        return 100.0 * num_success / num_gt if num_gt != 0 else 100.0

    @property
    def is_error(self) -> bool:
        return False


class Label(CriteriaMethodImpl):
    name = CriteriaMethod.LABEL

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        is_label_corrects = [
            result.is_label_correct
            for result in frame.object_results
            if result.ground_truth_object is not None
        ]

        return 100.0 if len(is_label_corrects) == 0 else 100.0 * np.mean(is_label_corrects)

    @property
    def is_error(self) -> bool:
        return False


class VelocityXError(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_X_ERROR

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            if result.ground_truth_object is not None:
                err = result.estimated_object.get_velocity_error(result.ground_truth_object)
                if err is None:
                    logging.warning("Velocity is None")
                    continue
                errors.append(err[0])
        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_error(self) -> bool:
        return True


class VelocityYError(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_Y_ERROR

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            if result.ground_truth_object is not None:
                err = result.estimated_object.get_velocity_error(result.ground_truth_object)
                if err is None:
                    logging.warning("Velocity is None")
                    continue
                errors.append(err[1])
        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_error(self) -> bool:
        return True


class SpeedError(CriteriaMethodImpl):
    name = CriteriaMethod.SPEED_ERROR

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            if result.ground_truth_object is not None:
                if (
                    result.estimated_object.state.velocity is None
                    or result.ground_truth_object.state.velocity is None
                ):
                    logging.warning("Velocity is None")
                    continue
                est_norm = np.linalg.norm(result.estimated_object.state.velocity[:2])
                gt_norm = np.linalg.norm(result.ground_truth_object.state.velocity[:2])
                err = abs(gt_norm - est_norm)
                errors.append(err)
        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_error(self) -> bool:
        return True


class YawError(CriteriaMethodImpl):
    name = CriteriaMethod.YAW_ERROR

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        errors = []
        for result in frame.object_results:
            err = result.heading_error
            if err is not None:
                _, _, yaw_err = err
                errors.append(abs(yaw_err))

        return 0.0 if len(errors) == 0 else np.mean(errors)

    @property
    def is_error(self) -> bool:
        return True


class MetricsScore(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        if frame.metrics_score.evaluation_task == EvaluationTask.CLASSIFICATION2D:
            scores = [
                acc.accuracy
                for score in frame.metrics_score.classification_scores
                for acc in score.accuracies
                if not np.isnan(acc.accuracy)
            ]
        else:
            scores = [
                map_.map
                for map_ in frame.metrics_score.mean_ap_values
                if not np.isnan(map_.map) and map_.matching_mode == MatchingMode.CENTERDISTANCE
            ]

        return 100.0 * sum(scores) / len(scores) if len(scores) != 0 else 0.0

    @property
    def is_error(self) -> bool:
        return False


class MetricsScoreMAPH(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE_MAPH

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__(level)

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        assert frame.metrics_score.evaluation_task.is_3d(), "Evaluation task must be 3D for MAPH."
        scores = [
            map_.maph
            for map_ in frame.metrics_score.mean_ap_values
            if not np.isnan(map_.maph) and map_.matching_mode == MatchingMode.CENTERDISTANCE
        ]

        return 100.0 * sum(scores) / len(scores) if len(scores) != 0 else 0.0

    @property
    def is_error(self) -> bool:
        return False


class CriteriaFilter:
    def __init__(self, filters: Filter | None = None) -> None:
        self.distance_range = getattr(filters, "Distance", None)
        self.region = getattr(filters, "Region", None)

    def is_all_none(self) -> bool:
        """
        Return True if all filter params are None.

        Returns
        -------
            bool: True if all filter params are None.

        """
        return self.distance_range is None and self.region is None

    def filter_frame_result(self, frame: PerceptionFrameResult) -> PerceptionFrameResult:
        """
        Filter PerceptionFrameResult by distance range.

        If all filter params are None, do nothing and return original frame result.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result.

        Returns:
        -------
            PerceptionFrameResult: Filtered result.

        """
        if self.is_all_none():
            return frame
        if self.distance_range is not None:
            min_distance, max_distance = self.distance_range
            return filter_frame_by_distance(frame, min_distance, max_distance)
        if self.region is not None:
            x_position: tuple[Number, Number] = self.region.x_position
            y_position: tuple[Number, Number] = self.region.y_position
            return filter_frame_by_region(frame, x_position, y_position)
        error_msg = "only select one filter condition"
        raise ValueError(error_msg)


class PerceptionCriteria:
    """
    Criteria interface for perception evaluation.

    Args:
    ----
        methods (str | list[str] | CriteriaMethod | list[CriteriaMethod] | None): List of criteria method instances or names.
            If None, `CriteriaMethod.NUM_TP` is used. Defaults to None.
        levels (str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel]): Criteria level instance or name.
            If None, `CriteriaLevel.Easy` is used. Defaults to None.
        filters (Filter | None): Filter instance. Defaults to None.

    """

    def __init__(  # noqa: C901
        self,
        methods: str | list[str] | CriteriaMethod | list[CriteriaMethod] | None = None,
        levels: (
            str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel] | None
        ) = None,
        filters: Filter | None = None,
    ) -> None:
        methods = [CriteriaMethod.NUM_TP] if methods is None else self.load_methods(methods)
        levels = [CriteriaLevel.EASY] if levels is None else self.load_levels(levels)

        err_msg = f"Number of CriteriaMethod and CriteriaLevel must be same. Current methods: {methods}, levels: {levels}"
        assert len(methods) == len(levels), err_msg

        self.methods = []
        for method, level in zip(methods, levels, strict=True):
            if method == CriteriaMethod.NUM_TP:
                self.methods.append(NumTP(level))
            elif method == CriteriaMethod.NUM_GT_TP:
                self.methods.append(NumGtTP(level))
            elif method == CriteriaMethod.LABEL:
                self.methods.append(Label(level))
            elif method == CriteriaMethod.VELOCITY_X_ERROR:
                self.methods.append(VelocityXError(level))
            elif method == CriteriaMethod.VELOCITY_Y_ERROR:
                self.methods.append(VelocityYError(level))
            elif method == CriteriaMethod.SPEED_ERROR:
                self.methods.append(SpeedError(level))
            elif method == CriteriaMethod.YAW_ERROR:
                self.methods.append(YawError(level))
            elif method == CriteriaMethod.METRICS_SCORE:
                self.methods.append(MetricsScore(level))
            elif method == CriteriaMethod.METRICS_SCORE_MAPH:
                self.methods.append(MetricsScoreMAPH(level))
            else:
                error_msg: str = f"Unsupported method: {method}"
                raise NotImplementedError(error_msg)

        self.criteria_filter = CriteriaFilter(filters)

    @staticmethod
    def load_methods(
        methods_input: str | list[str] | CriteriaMethod | list[CriteriaMethod],
    ) -> list[CriteriaMethod]:
        return load_methods(methods_input)

    @staticmethod
    def load_levels(
        levels_input: str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel],
    ) -> list[CriteriaLevel]:
        return load_levels(levels_input)

    def get_result(
        self,
        frame: PerceptionFrameResult,
    ) -> (
        tuple[SuccessFail, dict[str, float], PerceptionFrameResult]
        | tuple[None, None, PerceptionFrameResult]
    ):
        """
        Return Success/Fail result from `PerceptionFrameResult`.

        Args:
        ----
            frame (PerceptionFrameResult): Frame result of perception evaluation.

        Returns:
        -------
            tuple[SuccessFail, dict[str, float], PerceptionFrameResult] | tuple[None, None, PerceptionFrameResult]:
                - SuccessFail: Overall success/fail result.
                - dict[str, float]: Calculated scores for each method.
                - PerceptionFrameResult: Filtered frame result.

        """
        ret_frame = self.criteria_filter.filter_frame_result(frame)

        result: SuccessFail = SuccessFail.SUCCESS

        scores: dict[str, float] = {}
        for method in self.methods:
            method_result = method.get_result(ret_frame)
            if method_result is None:
                return None, None, ret_frame
            success_fail, score = method_result
            scores[method.name.value] = score
            result &= success_fail
        return result, scores, ret_frame
