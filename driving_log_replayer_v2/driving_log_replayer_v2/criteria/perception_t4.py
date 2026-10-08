# Copyright (c) 2026 TIER IV.inc
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

"""Pass/fail criteria of the perception use case over a `PerceptionFrameRecord`."""

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
import logging
from typing import TYPE_CHECKING

import numpy as np

from driving_log_replayer_v2.criteria.common import CriteriaLevel
from driving_log_replayer_v2.criteria.common import CriteriaMethod
from driving_log_replayer_v2.criteria.common import load_levels
from driving_log_replayer_v2.criteria.common import load_methods
from driving_log_replayer_v2.criteria.common import SuccessFail

if TYPE_CHECKING:
    from numbers import Number

    from driving_log_replayer_v2.perception.models import Filter
    from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import (
        PerceptionFrameRecord,
    )

__all__ = (
    "CriteriaFilter",
    "CriteriaLevel",
    "CriteriaMethod",
    "PerceptionCriteria",
    "SuccessFail",
)


class CriteriaMethodImpl(ABC):
    """One criteria method evaluated on a frame record."""

    name: CriteriaMethod

    def __init__(self, level: CriteriaLevel) -> None:
        super().__init__()
        self.level: CriteriaLevel = level

    def get_result(self, frame: PerceptionFrameRecord) -> tuple[SuccessFail, float] | None:
        """Return the success/fail and the score, or None when the frame has no objects."""
        if self.has_objects(frame) is False:
            return None
        score: float = self.calculate_score(frame)
        return (
            (SuccessFail.SUCCESS, score)
            if self.level.is_valid(score, is_error=self.is_error)
            else (SuccessFail.FAIL, score)
        )

    @staticmethod
    def has_objects(frame: PerceptionFrameRecord) -> bool:
        return frame.num_success + frame.num_fail > 0

    @staticmethod
    @abstractmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        """Calculate the score of the frame."""

    @property
    @abstractmethod
    def is_error(self) -> bool:
        """True when a lower score is better."""


class NumTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_TP

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        num_objects = frame.num_success + frame.num_fail
        return 100.0 * frame.num_success / num_objects if num_objects != 0 else 100.0

    @property
    def is_error(self) -> bool:
        return False


class NumGtTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_GT_TP

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        return 100.0 * frame.num_success / frame.num_gt if frame.num_gt != 0 else 100.0

    @property
    def is_error(self) -> bool:
        return False


class Label(CriteriaMethodImpl):
    name = CriteriaMethod.LABEL

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        is_label_corrects = frame.is_label_correct()
        return 100.0 if len(is_label_corrects) == 0 else 100.0 * float(np.mean(is_label_corrects))

    @property
    def is_error(self) -> bool:
        return False


def _mean_or_zero(values: np.ndarray) -> float:
    values = values[~np.isnan(values)]
    return 0.0 if len(values) == 0 else float(np.mean(values))


class VelocityXError(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_X_ERROR

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        errors = frame.pair_errors().velocity[:, 0]
        if np.isnan(errors).any():
            logging.warning("Velocity is None")
        return _mean_or_zero(errors)

    @property
    def is_error(self) -> bool:
        return True


class VelocityYError(CriteriaMethodImpl):
    name = CriteriaMethod.VELOCITY_Y_ERROR

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        errors = frame.pair_errors().velocity[:, 1]
        if np.isnan(errors).any():
            logging.warning("Velocity is None")
        return _mean_or_zero(errors)

    @property
    def is_error(self) -> bool:
        return True


class SpeedError(CriteriaMethodImpl):
    name = CriteriaMethod.SPEED_ERROR

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        est = frame.estimation.velocity[frame.matches.tp_est][:, :2]
        gt = frame.ground_truth.velocity[frame.matches.tp_gt][:, :2]
        errors = np.abs(np.linalg.norm(gt, axis=1) - np.linalg.norm(est, axis=1))
        if np.isnan(errors).any():
            logging.warning("Velocity is None")
        return _mean_or_zero(errors)

    @property
    def is_error(self) -> bool:
        return True


class YawError(CriteriaMethodImpl):
    name = CriteriaMethod.YAW_ERROR

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        return _mean_or_zero(np.abs(frame.pair_errors().heading[:, 2]))

    @property
    def is_error(self) -> bool:
        return True


class MetricsScore(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        value = frame.frame_metrics.get("map", float("nan"))
        return 0.0 if np.isnan(value) else 100.0 * value

    @property
    def is_error(self) -> bool:
        return False


class MetricsScoreMAPH(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE_MAPH

    @staticmethod
    def calculate_score(frame: PerceptionFrameRecord) -> float:
        value = frame.frame_metrics.get("maph", float("nan"))
        return 0.0 if np.isnan(value) else 100.0 * value

    @property
    def is_error(self) -> bool:
        return False


_METHOD_IMPLS: dict[CriteriaMethod, type[CriteriaMethodImpl]] = {
    CriteriaMethod.NUM_TP: NumTP,
    CriteriaMethod.NUM_GT_TP: NumGtTP,
    CriteriaMethod.LABEL: Label,
    CriteriaMethod.VELOCITY_X_ERROR: VelocityXError,
    CriteriaMethod.VELOCITY_Y_ERROR: VelocityYError,
    CriteriaMethod.SPEED_ERROR: SpeedError,
    CriteriaMethod.YAW_ERROR: YawError,
    CriteriaMethod.METRICS_SCORE: MetricsScore,
    CriteriaMethod.METRICS_SCORE_MAPH: MetricsScoreMAPH,
}

FRAME_METRICS_METHODS: frozenset[CriteriaMethod] = frozenset(
    {CriteriaMethod.METRICS_SCORE, CriteriaMethod.METRICS_SCORE_MAPH}
)
"""Methods which need the per-frame mAP, see `PerceptionEvaluator(compute_frame_metrics=)`."""


class CriteriaFilter:
    def __init__(self, filters: Filter | None = None) -> None:
        self.distance_range = getattr(filters, "Distance", None)
        self.region = getattr(filters, "Region", None)

    def is_all_none(self) -> bool:
        return self.distance_range is None and self.region is None

    def filter_frame_result(self, frame: PerceptionFrameRecord) -> PerceptionFrameRecord:
        """Narrow the record to the distance range or the region, if one is set."""
        if self.is_all_none():
            return frame
        if self.distance_range is not None:
            min_distance, max_distance = self.distance_range
            return frame.filter_by_distance(min_distance, max_distance)
        if self.region is not None:
            return frame.filter_by_region(self.region.x_position, self.region.y_position)
        error_msg = "only select one filter condition"
        raise ValueError(error_msg)


class PerceptionCriteria:
    """
    Criteria interface of the perception use case.

    Args:
        methods (str | list[str] | CriteriaMethod | list[CriteriaMethod] | None): Criteria
            methods or their names. If None, `CriteriaMethod.NUM_TP` is used.
        levels (str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel]):
            Criteria levels, names or custom values. If None, `CriteriaLevel.EASY` is used.
        filters (Filter | None): Filter settings of the scenario.

    """

    def __init__(
        self,
        methods: str | list[str] | CriteriaMethod | list[CriteriaMethod] | None = None,
        levels: (
            str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel] | None
        ) = None,
        filters: Filter | None = None,
    ) -> None:
        methods = [CriteriaMethod.NUM_TP] if methods is None else load_methods(methods)
        levels = [CriteriaLevel.EASY] if levels is None else load_levels(levels)

        err_msg = f"Number of CriteriaMethod and CriteriaLevel must be same. Current methods: {methods}, levels: {levels}"
        assert len(methods) == len(levels), err_msg

        self.methods: list[CriteriaMethodImpl] = []
        for method, level in zip(methods, levels, strict=True):
            impl = _METHOD_IMPLS.get(method)
            if impl is None:
                error_msg = f"Unsupported method: {method}"
                raise NotImplementedError(error_msg)
            self.methods.append(impl(level))

        self.criteria_filter = CriteriaFilter(filters)

    @property
    def needs_frame_metrics(self) -> bool:
        return any(method.name in FRAME_METRICS_METHODS for method in self.methods)

    def get_result(
        self,
        frame: PerceptionFrameRecord,
    ) -> (
        tuple[SuccessFail, dict[str, float], PerceptionFrameRecord]
        | tuple[None, None, PerceptionFrameRecord]
    ):
        """
        Return the success/fail of the frame.

        Returns:
            - SuccessFail | None: Overall result, None when no method has objects to score.
            - dict[str, float] | None: Score of each method.
            - PerceptionFrameRecord: The (possibly filtered) record the scores were taken on.

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
