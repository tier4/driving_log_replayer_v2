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

"""
Scenario settings of the perception use case, parsed for t4perceval.

The scenario keeps the three dictionaries perception_eval used
(`PerceptionEvaluationConfig.evaluation_config_dict`, `CriticalObjectFilterConfig` and
`PerceptionPassFailConfig`). This module validates them once and turns the per-label lists
into mappings keyed by label name, which is what the t4perceval systems take.
"""

from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field
from dataclasses import fields
from numbers import Real
from typing import Any
from typing import Literal
from typing import TYPE_CHECKING

from driving_log_replayer_v2.perception.t4perceval_adapter.labels import canonical_labels
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import (
    validate_matching_label_policy,
)

if TYPE_CHECKING:
    import logging

EvaluationTask = Literal["detection", "tracking", "prediction"]
FrameId = Literal["base_link", "map"]

EVALUATION_TASKS: tuple[str, ...] = ("detection", "tracking", "prediction")
FRAME_ID_OF_TASK: dict[str, str] = {
    "detection": "base_link",
    "tracking": "map",
    "prediction": "map",
}

IGNORED_EVALUATION_KEYS: tuple[str, ...] = (
    "label_prefix",
    "count_label_number",
    "matching_class_agnostic_fps",
)
"""Keys of `evaluation_config_dict` which have no effect with t4perceval."""

_KNOWN_EVALUATION_KEYS: tuple[str, ...] = (
    "evaluation_task",
    "target_labels",
    "max_x_position",
    "max_y_position",
    "max_distance",
    "min_distance",
    "confidence_threshold",
    "target_uuids",
    "max_matchable_radii",
    "merge_similar_labels",
    "matching_label_policy",
    "allow_matching_unknown",
    "ignore_attributes",
    "center_distance_thresholds",
    "center_distance_bev_thresholds",
    "plane_distance_thresholds",
    "iou_2d_thresholds",
    "iou_3d_thresholds",
    "min_point_numbers",
    "top_ks",
    "miss_tolerance",
    "prediction_num_modes",
    "prediction_num_timesteps",
    "future_seconds",
    *IGNORED_EVALUATION_KEYS,
)

PerLabel = dict[str, float]


@dataclass(frozen=True)
class CriticalObjectFilterConfig:
    """Per-frame object filter, the former `CriticalObjectFilterConfig`."""

    target_labels: tuple[str, ...]
    max_x_position: PerLabel | None = None
    max_y_position: PerLabel | None = None
    max_distance: PerLabel | None = None
    min_distance: PerLabel | None = None
    min_num_points: dict[str, int] | None = None
    confidence_threshold: PerLabel | None = None
    target_uuids: tuple[str, ...] | None = None
    ignore_attributes: tuple[str, ...] = ()


@dataclass(frozen=True)
class PassFailConfig:
    """Pass/fail matching, the former `PerceptionPassFailConfig`."""

    target_labels: tuple[str, ...]
    matching_threshold: PerLabel | None = None
    confidence_threshold: PerLabel | None = None


@dataclass(frozen=True)
class EvaluationConfig:
    """Everything the t4perceval evaluation of one topic needs."""

    evaluation_task: EvaluationTask
    frame_id: FrameId
    target_labels: tuple[str, ...]
    merge_similar_labels: bool = False
    matching_label_policy: str = "default"
    max_x_position: float | None = None
    max_y_position: float | None = None
    max_distance: float | None = None
    min_distance: float | None = None
    confidence_threshold: float | None = None
    target_uuids: tuple[str, ...] | None = None
    ignore_attributes: tuple[str, ...] = ()
    min_num_points: dict[str, int] | None = None
    max_matchable_radii: PerLabel | None = None
    center_distance_thresholds: tuple[PerLabel, ...] = ()
    center_distance_bev_thresholds: tuple[PerLabel, ...] = ()
    plane_distance_thresholds: tuple[PerLabel, ...] = ()
    iou_2d_thresholds: tuple[PerLabel, ...] = ()
    iou_3d_thresholds: tuple[PerLabel, ...] = ()
    top_ks: tuple[int, ...] = (1, 3, 6)
    miss_tolerance: float = 2.0
    prediction_num_modes: int = 10
    prediction_num_timesteps: int = 40
    future_seconds: float = 8.0
    critical: CriticalObjectFilterConfig = field(
        default_factory=lambda: CriticalObjectFilterConfig(target_labels=())
    )
    pass_fail: PassFailConfig = field(default_factory=lambda: PassFailConfig(target_labels=()))

    @property
    def trajectory_shape(self) -> tuple[int, int] | None:
        """Pinned (num_modes, num_timesteps) of the predicted paths, None unless prediction."""
        if self.evaluation_task != "prediction":
            return None
        return (self.prediction_num_modes, self.prediction_num_timesteps)

    @property
    def threshold_families(self) -> dict[str, tuple[PerLabel, ...]]:
        """Matching mode name -> threshold sets, only the configured ones."""
        families = {
            "center_distance": self.center_distance_thresholds,
            "center_distance_bev": self.center_distance_bev_thresholds,
            "plane_distance": self.plane_distance_thresholds,
            "iou_2d": self.iou_2d_thresholds,
            "iou_3d": self.iou_3d_thresholds,
        }
        return {name: sets for name, sets in families.items() if sets}

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> EvaluationConfig:
        """Rebuild the config from `to_dict()` output (lists become tuples)."""
        data = dict(data)
        data["critical"] = CriticalObjectFilterConfig(**_tuplify(data["critical"]))
        data["pass_fail"] = PassFailConfig(**_tuplify(data["pass_fail"]))
        return cls(**_tuplify(data))


def _tuplify(data: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in data.items():
        if isinstance(value, list):
            out[key] = tuple(_tuplify(v) if isinstance(v, dict) else v for v in value)
        else:
            out[key] = value
    return out


def _as_labels(value: Any, name: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple) or not value:
        err_msg = f"{name} must be a non-empty list of labels, got {value!r}"
        raise ValueError(err_msg)
    return tuple(str(label) for label in value)


def _per_label(
    value: Any,
    labels: tuple[str, ...],
    name: str,
    *,
    cast: type = float,
) -> dict[str, Any] | None:
    """Turn a scalar or a per-label list into a label -> value mapping."""
    if value is None:
        return None
    if isinstance(value, Real):
        return dict.fromkeys(labels, cast(value))
    if isinstance(value, list | tuple):
        if len(value) != len(labels):
            err_msg = (
                f"{name} must have one value per target label ({len(labels)}), "
                f"got {len(value)}: {value!r}"
            )
            raise ValueError(err_msg)
        return {label: cast(v) for label, v in zip(labels, value, strict=True)}
    err_msg = f"{name} must be a number or a list, got {value!r}"
    raise TypeError(err_msg)


def _threshold_sets(value: Any, labels: tuple[str, ...], name: str) -> tuple[PerLabel, ...]:
    """
    Turn perception_eval threshold lists into per-label mappings.

    `[2.0, 30.0]` means two threshold sets which apply the value to every label.
    `[[1.0, 1.0], [2.0, 2.0]]` means two threshold sets with one value per label.
    """
    if value is None:
        return ()
    if isinstance(value, Real):
        return (dict.fromkeys(labels, float(value)),)
    if not isinstance(value, list | tuple):
        err_msg = f"{name} must be a number or a list, got {value!r}"
        raise TypeError(err_msg)
    if not value:
        return ()
    if all(isinstance(v, Real) for v in value):
        return tuple(dict.fromkeys(labels, float(v)) for v in value)
    return tuple(_per_label(v, labels, name) for v in value)


def _optional_float(value: Any, name: str) -> float | None:
    if value is None:
        return None
    if not isinstance(value, Real):
        err_msg = f"{name} must be a number, got {value!r}"
        raise TypeError(err_msg)
    return float(value)


def _optional_uuids(value: Any) -> tuple[str, ...] | None:
    if value is None:
        return None
    return tuple(str(uuid) for uuid in value)


def from_scenario(
    perception_evaluation_config: dict,
    critical_object_filter_config: dict,
    perception_pass_fail_config: dict,
    *,
    evaluation_task: str,
    frame_id: str,
    logger: logging.Logger | None = None,
) -> EvaluationConfig:
    """
    Parse the three scenario dictionaries.

    Args:
        perception_evaluation_config (dict): `Evaluation.PerceptionEvaluationConfig`.
        critical_object_filter_config (dict): `Evaluation.CriticalObjectFilterConfig`.
        perception_pass_fail_config (dict): `Evaluation.PerceptionPassFailConfig`.
        evaluation_task (str): detection, tracking or prediction.
        frame_id (str): base_link or map.
        logger (logging.Logger | None): Logger for the warnings about ignored keys.

    Returns:
        EvaluationConfig: Validated settings.

    """
    if evaluation_task not in EVALUATION_TASKS:
        err_msg = f"Unsupported evaluation task for t4perceval: {evaluation_task}"
        raise ValueError(err_msg)
    if frame_id not in ("base_link", "map"):
        err_msg = f"Unsupported frame id: {frame_id}"
        raise ValueError(err_msg)

    eval_dict: dict = dict(perception_evaluation_config.get("evaluation_config_dict", {}))
    for key in eval_dict:
        if key not in _KNOWN_EVALUATION_KEYS and logger is not None:
            logger.warning("Unknown key in evaluation_config_dict is ignored: %s", key)
    for key in IGNORED_EVALUATION_KEYS:
        if key in eval_dict and logger is not None:
            logger.warning("evaluation_config_dict.%s has no effect with t4perceval.", key)

    merge_similar_labels = bool(eval_dict.get("merge_similar_labels", False))
    target_labels = _as_labels(eval_dict.get("target_labels"), "target_labels")
    known = canonical_labels(merge_similar_labels=merge_similar_labels)
    unknown_targets = [label for label in target_labels if label not in known]
    if unknown_targets:
        err_msg = f"target_labels contain labels outside {known}: {unknown_targets}"
        raise ValueError(err_msg)

    policy = eval_dict.get("matching_label_policy")
    if policy is None:
        if eval_dict.get("allow_matching_unknown", False):
            err_msg = (
                "allow_matching_unknown: true (matching_label_policy: allow_unknown) is not "
                "supported with t4perceval."
            )
            raise ValueError(err_msg)
        policy = "default"
    policy = validate_matching_label_policy(str(policy))

    critical_labels = _as_labels(
        critical_object_filter_config.get("target_labels", target_labels), "target_labels"
    )
    critical = CriticalObjectFilterConfig(
        target_labels=critical_labels,
        max_x_position=_per_label(
            critical_object_filter_config.get("max_x_position_list"),
            critical_labels,
            "max_x_position_list",
        ),
        max_y_position=_per_label(
            critical_object_filter_config.get("max_y_position_list"),
            critical_labels,
            "max_y_position_list",
        ),
        max_distance=_per_label(
            critical_object_filter_config.get("max_distance_list"),
            critical_labels,
            "max_distance_list",
        ),
        min_distance=_per_label(
            critical_object_filter_config.get("min_distance_list"),
            critical_labels,
            "min_distance_list",
        ),
        min_num_points=_per_label(
            critical_object_filter_config.get("min_point_numbers"),
            critical_labels,
            "min_point_numbers",
            cast=int,
        ),
        confidence_threshold=_per_label(
            critical_object_filter_config.get("confidence_threshold_list"),
            critical_labels,
            "confidence_threshold_list",
        ),
        target_uuids=_optional_uuids(critical_object_filter_config.get("target_uuids")),
        ignore_attributes=tuple(critical_object_filter_config.get("ignore_attributes") or ()),
    )

    pass_fail_labels = _as_labels(
        perception_pass_fail_config.get("target_labels", target_labels), "target_labels"
    )
    pass_fail = PassFailConfig(
        target_labels=pass_fail_labels,
        matching_threshold=_per_label(
            perception_pass_fail_config.get("matching_threshold_list"),
            pass_fail_labels,
            "matching_threshold_list",
        ),
        confidence_threshold=_per_label(
            perception_pass_fail_config.get("confidence_threshold_list"),
            pass_fail_labels,
            "confidence_threshold_list",
        ),
    )

    top_ks = tuple(int(k) for k in eval_dict.get("top_ks", (1, 3, 6)))
    return EvaluationConfig(
        evaluation_task=evaluation_task,
        frame_id=frame_id,
        target_labels=target_labels,
        merge_similar_labels=merge_similar_labels,
        matching_label_policy=policy,
        max_x_position=_optional_float(eval_dict.get("max_x_position"), "max_x_position"),
        max_y_position=_optional_float(eval_dict.get("max_y_position"), "max_y_position"),
        max_distance=_optional_float(eval_dict.get("max_distance"), "max_distance"),
        min_distance=_optional_float(eval_dict.get("min_distance"), "min_distance"),
        confidence_threshold=_optional_float(
            eval_dict.get("confidence_threshold"), "confidence_threshold"
        ),
        target_uuids=_optional_uuids(eval_dict.get("target_uuids")),
        ignore_attributes=tuple(eval_dict.get("ignore_attributes") or ()),
        min_num_points=_per_label(
            eval_dict.get("min_point_numbers"), target_labels, "min_point_numbers", cast=int
        ),
        max_matchable_radii=_per_label(
            eval_dict.get("max_matchable_radii"), target_labels, "max_matchable_radii"
        ),
        center_distance_thresholds=_threshold_sets(
            eval_dict.get("center_distance_thresholds"),
            target_labels,
            "center_distance_thresholds",
        ),
        center_distance_bev_thresholds=_threshold_sets(
            eval_dict.get("center_distance_bev_thresholds"),
            target_labels,
            "center_distance_bev_thresholds",
        ),
        plane_distance_thresholds=_threshold_sets(
            eval_dict.get("plane_distance_thresholds"),
            target_labels,
            "plane_distance_thresholds",
        ),
        iou_2d_thresholds=_threshold_sets(
            eval_dict.get("iou_2d_thresholds"), target_labels, "iou_2d_thresholds"
        ),
        iou_3d_thresholds=_threshold_sets(
            eval_dict.get("iou_3d_thresholds"), target_labels, "iou_3d_thresholds"
        ),
        top_ks=top_ks,
        miss_tolerance=float(eval_dict.get("miss_tolerance", 2.0)),
        prediction_num_modes=int(eval_dict.get("prediction_num_modes", 10)),
        prediction_num_timesteps=int(eval_dict.get("prediction_num_timesteps", 40)),
        future_seconds=float(eval_dict.get("future_seconds", 8.0)),
        critical=critical,
        pass_fail=pass_fail,
    )


def config_field_names() -> tuple[str, ...]:
    return tuple(f.name for f in fields(EvaluationConfig))
