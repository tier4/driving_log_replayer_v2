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

"""Autoware object messages to t4perceval archetypes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import TYPE_CHECKING
import uuid as uuid_lib

from attrs import evolve
import numpy as np
from t4perceval import UNKNOWN_CLASS_ID
from t4perceval.importer.rosbag.convert import kind_of_schema
from t4perceval.importer.rosbag.convert import objects_to_columns
from t4perceval.importer.rosbag.convert import stamp_ns

if TYPE_CHECKING:
    from t4perceval import InstanceRegistry
    from t4perceval import LabelRegistry
    from t4perceval.core.archetype import Archetype

    from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig

ESTIMATION_NAMESPACE = "est"
GROUND_TRUTH_NAMESPACE = "gt"

KIND_OF_TASK: dict[str, str] = {
    "detection": "detections",
    "tracking": "trackings",
    "prediction": "predictions",
}

__all__ = (
    "ConversionContext",
    "EstimationFrame",
    "convert_objects_msg",
    "header_stamp_ns",
    "stamp_ns",
    "strip_namespace",
)


def header_stamp_ns(header: Any) -> int:
    """Return `header.stamp` as nanoseconds since the Unix epoch."""
    return stamp_ns(header.stamp)


def strip_namespace(name: str) -> str:
    """Return the dataset uuid of an interned instance name (`gt/<uuid>` -> `<uuid>`)."""
    return name.split("/", 1)[1] if "/" in name else name


@dataclass(frozen=True)
class ConversionContext:
    """What the ROS message conversion needs from the evaluator."""

    config: EvaluationConfig
    labels: LabelRegistry
    instances: InstanceRegistry


@dataclass(frozen=True)
class EstimationFrame:
    """One converted object message."""

    kind: str
    """detections, trackings or predictions."""
    archetype: Archetype
    header_frame_id: str
    uuids: tuple[str | None, ...]
    """Object uuid per row, None for DetectedObjects."""
    pose_covariance: np.ndarray
    """(N, 36) float64, the pose covariance per row."""
    twist_covariance: np.ndarray
    """(N, 36) float64, the twist covariance per row."""

    def __len__(self) -> int:
        return len(self.uuids)


def _kinematics(obj: Any, kind: str) -> tuple[Any, Any]:
    if kind == "predictions":
        return (
            obj.kinematics.initial_pose_with_covariance,
            obj.kinematics.initial_twist_with_covariance,
        )
    return obj.kinematics.pose_with_covariance, obj.kinematics.twist_with_covariance


def _covariance_rows(objects: list[Any], kind: str) -> tuple[np.ndarray, np.ndarray]:
    pose = np.zeros((len(objects), 36), dtype=np.float64)
    twist = np.zeros((len(objects), 36), dtype=np.float64)
    for row, obj in enumerate(objects):
        pose_with_cov, twist_with_cov = _kinematics(obj, kind)
        pose[row] = np.asarray(pose_with_cov.covariance, dtype=np.float64)
        twist[row] = np.asarray(twist_with_cov.covariance, dtype=np.float64)
    return pose, twist


def _uuids(objects: list[Any], kind: str) -> tuple[str | None, ...]:
    if kind == "detections":
        return tuple([None] * len(objects))
    return tuple(str(uuid_lib.UUID(bytes=bytes(bytearray(obj.object_id.uuid)))) for obj in objects)


def convert_objects_msg(msg: Any, context: ConversionContext) -> EstimationFrame | str:
    """
    Convert a DetectedObjects / TrackedObjects / PredictedObjects message.

    Args:
        msg: The ROS message.
        context (ConversionContext): Config and registries of the evaluator.

    Returns:
        EstimationFrame | str: The converted frame, or an error message when the message
            cannot be converted (invalid footprint, predicted paths exceeding the pinned shape).

    """
    try:
        kind = kind_of_schema(type(msg).__name__)
    except ValueError as error:
        return str(error)
    expected_kind = KIND_OF_TASK[context.config.evaluation_task]
    if kind != expected_kind:
        return f"Unexpected message type {type(msg).__name__} for {context.config.evaluation_task}"

    for obj in msg.objects:
        num_footprint = len(obj.shape.footprint.points)
        if 1 <= num_footprint < 3:  # noqa: PLR2004
            return f"Unexpected footprint length: {num_footprint=}"

    try:
        columns = objects_to_columns(
            msg,
            kind=kind,
            labels=context.labels,
            instances=context.instances,
            instance_namespace=ESTIMATION_NAMESPACE,
            unknown_labels="unknown",
            confidence="classification",
            velocity="always",
            trajectory=context.config.trajectory_shape,
        )
    except ValueError as error:
        return f"Failed to convert objects: {error}"

    # perception_eval evaluated a classification it did not know as `unknown`
    class_id = columns.class_id.copy()
    unknown_id = context.labels.class_id_or("unknown", UNKNOWN_CLASS_ID)
    class_id[class_id == UNKNOWN_CLASS_ID] = unknown_id
    columns = evolve(columns, class_id=class_id)

    objects = [msg.objects[int(i)] for i in columns.kept]
    pose_cov, twist_cov = _covariance_rows(objects, kind)
    return EstimationFrame(
        kind=kind,
        archetype=columns.as_archetype(kind),
        header_frame_id=str(msg.header.frame_id),
        uuids=_uuids(objects, kind),
        pose_covariance=pose_cov,
        twist_covariance=twist_cov,
    )
