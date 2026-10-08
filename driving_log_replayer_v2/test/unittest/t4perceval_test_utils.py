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

"""Builders of synthetic frame records for the t4perceval based perception tests."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from t4perceval import Detections3D
from t4perceval import LabelRegistry
from t4perceval import TimePoint
from t4perceval.transform.apply import transform_chunk
from t4perceval.transform.compose import identity

from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import MatchTable
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import ObjectTable
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import PerceptionFrameRecord
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import build_label_registry
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import ESTIMATION_KEPT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import GROUND_TRUTH_KEPT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import in_base_link

if TYPE_CHECKING:
    from collections.abc import Sequence

TIMESTAMP_NS = 1_624_157_578_750_212_000


def make_labels() -> LabelRegistry:
    return build_label_registry()


def make_table(
    positions: Sequence[Sequence[float]],
    labels: Sequence[str],
    registry: LabelRegistry,
    *,
    path: str,
    frame_index: int = 1,
    timestamp_ns: int = TIMESTAMP_NS,
    velocity: Sequence[Sequence[float]] | None = None,
    uuids: Sequence[str | None] | None = None,
    size: Sequence[float] = (1.0, 1.0, 1.0),
) -> ObjectTable:
    """Build an ObjectTable in base_link from positions and label names."""
    count = len(positions)
    detections = Detections3D(
        position=np.asarray(positions, dtype=np.float64).reshape(count, 3),
        quaternion=[[0.0, 0.0, 0.0, 1.0]] * count,
        size=[list(size)] * count,
        class_id=registry.encode(labels),
        confidence=[0.5] * count,
        velocity=None
        if velocity is None
        else np.asarray(velocity, dtype=np.float64).reshape(count, 3),
    )
    native = detections.to_chunk(
        path, at=TimePoint.at(frame=frame_index, timestamp_ns=timestamp_ns), frame_id="base_link"
    )
    base_link = transform_chunk(
        native, identity(), target_frame="base_link", entity_path=in_base_link(path)
    )
    return ObjectTable(native, base_link, tuple(uuids) if uuids else tuple([None] * count))


def make_record(
    *,
    tp: int = 0,
    fp: int = 0,
    fn: int = 0,
    label: str = "car",
    registry: LabelRegistry | None = None,
    policy: str = "default",
    frame_index: int = 1,
    frame_name: str = "12",
    timestamp_ns: int = TIMESTAMP_NS,
    tp_score: float = 0.5,
    est_velocity: Sequence[float] | None = None,
    gt_velocity: Sequence[float] | None = None,
) -> PerceptionFrameRecord:
    """
    Build a record with `tp` matched pairs, `fp` unmatched estimations and `fn` unmatched GTs.

    The i-th estimation and the i-th ground truth form the i-th TP pair, both at (i + 1, 0, 0).
    """
    registry = registry if registry is not None else make_labels()
    est_positions = [[float(i + 1), 0.0, 0.0] for i in range(tp + fp)]
    gt_positions = [[float(i + 1), 0.0, 0.0] for i in range(tp + fn)]
    estimation = make_table(
        est_positions,
        [label] * (tp + fp),
        registry,
        path=ESTIMATION_KEPT_PATH,
        frame_index=frame_index,
        timestamp_ns=timestamp_ns,
        velocity=None if est_velocity is None else [list(est_velocity)] * (tp + fp),
        uuids=[f"est-{i}" for i in range(tp + fp)],
    )
    ground_truth = make_table(
        gt_positions,
        [label] * (tp + fn),
        registry,
        path=GROUND_TRUTH_KEPT_PATH,
        frame_index=frame_index,
        timestamp_ns=timestamp_ns,
        velocity=None if gt_velocity is None else [list(gt_velocity)] * (tp + fn),
        uuids=[f"gt-{i}" for i in range(tp + fn)],
    )
    matches = MatchTable(
        est_index=np.asarray([*range(tp), *range(tp, tp + fp), *([-1] * fn)], dtype=np.int64),
        gt_index=np.asarray([*range(tp), *([-1] * fp), *range(tp, tp + fn)], dtype=np.int64),
        status=np.asarray([0] * tp + [1] * fp + [2] * fn, dtype=np.int8),
        score=np.asarray([tp_score] * tp + [np.nan] * (fp + fn), dtype=np.float64),
        threshold=np.full(tp + fp + fn, 2.0, dtype=np.float64),
    )
    return PerceptionFrameRecord(
        frame_index=frame_index,
        frame_name=frame_name,
        unix_time_ns=timestamp_ns,
        ground_truth_unix_time_ns=timestamp_ns,
        frame_id="base_link",
        labels=registry,
        policy=policy,
        estimation=estimation,
        ground_truth=ground_truth,
        matches=matches,
        frame_metrics={"map": 0.25, "maph": 0.125},
    )
