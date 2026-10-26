# Copyright 2026 TIER IV, Inc.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from importlib import import_module
from inspect import signature
import math
from typing import TYPE_CHECKING

from pydantic import BaseModel
from pydantic import ConfigDict
from pydantic import field_validator

if TYPE_CHECKING:
    from perception_eval.evaluation.sensing.bounding_box_scale_config import BoundingBoxScaleConfig

    from driving_log_replayer_v2.obstacle_segmentation import Conditions


class BoxScaleCondition(BaseModel):
    model_config = ConfigDict(extra="forbid")

    box_scale_0m: float | None = None
    box_scale_100m: float | None = None

    @field_validator("box_scale_0m", "box_scale_100m", mode="before")
    @classmethod
    def validate_endpoint(cls, value: object) -> float | None:
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            msg = "BoxScale values must be finite positive numbers or null"
            raise ValueError(msg)  # noqa: TRY004 - Pydantic validators must raise ValueError.
        if not math.isfinite(value) or value <= 0:
            msg = "BoxScale values must be finite positive numbers or null"
            raise ValueError(msg)
        return float(value)


def resolve_box_scales(
    conditions: Conditions,
    common_metrics: dict,
) -> tuple[BoundingBoxScaleConfig, BoundingBoxScaleConfig]:
    """Resolve D and N independently; do not mutate manager metrics."""
    try:
        config_class = import_module(
            "perception_eval.evaluation.sensing.bounding_box_scale_config"
        ).BoundingBoxScaleConfig
        manager_class = import_module(
            "perception_eval.manager.sensing_evaluation_manager"
        ).SensingEvaluationManager
        if "non_detection_box_scale" not in signature(manager_class.add_frame_result).parameters:
            raise ImportError  # noqa: TRY301
    except (ImportError, AttributeError) as error:
        msg = (
            "BoxScale separation requires perception_eval with BoundingBoxScaleConfig "
            "and SensingEvaluationManager.add_frame_result(non_detection_box_scale=...). "
            "Install the matching perception_eval revision; simulator.repos alone does not install it."
        )
        raise RuntimeError(msg) from error

    keys = ("box_scale_0m", "box_scale_100m")
    common = {key: common_metrics.get(key, 1.0) for key in keys}
    if any(value is None for value in common.values()):
        msg = "Common box_scale_0m/box_scale_100m cannot be null"
        raise ValueError(msg)
    resolved = []
    for condition in (conditions.Detection, conditions.NonDetection):
        override = condition.BoxScale if condition is not None else None
        values = {}
        for key in keys:
            value = getattr(override, key, None)
            values[key] = common[key] if value is None else value
        resolved.append(config_class(**values))
    return resolved[0], resolved[1]
