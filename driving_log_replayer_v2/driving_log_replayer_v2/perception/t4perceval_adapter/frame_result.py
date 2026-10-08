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

"""The per-frame result of the perception use case, the former `PerceptionFrameResult`."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation
from t4perceval import MatchResults
from t4perceval import TimePoint
from t4perceval.component import MatchStatus
from t4perceval.descriptors import CLASS_ID
from t4perceval.descriptors import CONFIDENCE
from t4perceval.descriptors import INSTANCE_ID
from t4perceval.descriptors import POSITION
from t4perceval.descriptors import QUATERNION
from t4perceval.descriptors import SIZE
from t4perceval.descriptors import VELOCITY

from driving_log_replayer_v2.perception.t4perceval_adapter.conversions import strip_namespace
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import restamp
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import policy_matrix
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import PASS_FAIL_MATCHING_PATH

if TYPE_CHECKING:
    from t4perceval import InstanceRegistry
    from t4perceval import LabelRegistry
    from t4perceval.core.chunk import Chunk
    from t4perceval.core.descriptor import ComponentDescriptor

__all__ = ("MatchTable", "ObjectTable", "PairErrors", "PerceptionFrameRecord")


def _column(chunk: Chunk, descriptor: ComponentDescriptor) -> np.ndarray | None:
    component = chunk.columns.get(descriptor)
    return None if component is None else np.asarray(component.values)


def _nan_rows(count: int, width: int) -> np.ndarray:
    return np.full((count, width), np.nan, dtype=np.float64)


@dataclass(frozen=True)
class ObjectTable:
    """
    The kept objects of one side of a frame.

    `native` holds the rows in the evaluation frame (base_link or map), `base_link` the same
    rows expressed in base_link. The uuids and the covariances are side tables aligned with
    the rows, because t4perceval stores neither.
    """

    native: Chunk
    base_link: Chunk
    uuids: tuple[str | None, ...]
    pose_covariance: np.ndarray | None = None
    twist_covariance: np.ndarray | None = None

    def __len__(self) -> int:
        return int(self.native.num_rows)

    @property
    def position(self) -> np.ndarray:
        return _column(self.native, POSITION)

    @property
    def quaternion(self) -> np.ndarray:
        """(N, 4) in xyzw order."""
        return _column(self.native, QUATERNION)

    @property
    def size(self) -> np.ndarray:
        """(N, 3) as (width, length, height)."""
        return _column(self.native, SIZE)

    @property
    def class_id(self) -> np.ndarray:
        return _column(self.native, CLASS_ID)

    @property
    def confidence(self) -> np.ndarray:
        return _column(self.native, CONFIDENCE)

    @property
    def instance_id(self) -> np.ndarray | None:
        return _column(self.native, INSTANCE_ID)

    @property
    def velocity(self) -> np.ndarray:
        """(N, 3) in the native frame, NaN rows when unknown."""
        velocity = _column(self.native, VELOCITY)
        return _nan_rows(len(self), 3) if velocity is None else velocity

    @property
    def position_bl(self) -> np.ndarray:
        return _column(self.base_link, POSITION)

    @property
    def quaternion_bl(self) -> np.ndarray:
        return _column(self.base_link, QUATERNION)

    @property
    def velocity_bl(self) -> np.ndarray:
        velocity = _column(self.base_link, VELOCITY)
        return _nan_rows(len(self), 3) if velocity is None else velocity

    def bev_distance(self) -> np.ndarray:
        """Distance from the ego in the xy plane, per row."""
        return np.linalg.norm(self.position_bl[:, :2], axis=1)

    def labels(self, registry: LabelRegistry) -> tuple[str, ...]:
        return registry.decode(self.class_id)

    def select(self, rows: np.ndarray) -> ObjectTable:
        rows = np.asarray(rows, dtype=np.int64)
        return ObjectTable(
            native=self.native.select(rows),
            base_link=self.base_link.select(rows),
            uuids=tuple(self.uuids[int(r)] for r in rows),
            pose_covariance=None if self.pose_covariance is None else self.pose_covariance[rows],
            twist_covariance=None if self.twist_covariance is None else self.twist_covariance[rows],
        )

    @classmethod
    def from_chunks(
        cls,
        native: Chunk,
        base_link: Chunk,
        *,
        instances: InstanceRegistry | None = None,
        uuids: tuple[str | None, ...] | None = None,
        pose_covariance: np.ndarray | None = None,
        twist_covariance: np.ndarray | None = None,
    ) -> ObjectTable:
        """Build a table; uuids are decoded from the instance ids when not given."""
        if uuids is None:
            instance_id = _column(native, INSTANCE_ID)
            if instance_id is None or instances is None:
                uuids = tuple([None] * native.num_rows)
            else:
                uuids = tuple(strip_namespace(instances.uuid(int(i))) for i in instance_id)
        return cls(native, base_link, uuids, pose_covariance, twist_covariance)


@dataclass(frozen=True)
class MatchTable:
    """The pass/fail verdicts of one frame, one row per TP pair, FP estimation or FN ground truth."""

    est_index: np.ndarray
    gt_index: np.ndarray
    status: np.ndarray
    score: np.ndarray
    threshold: np.ndarray

    @classmethod
    def from_chunk(cls, chunk: Chunk) -> MatchTable:
        results = MatchResults.from_chunk(chunk)
        return cls(
            est_index=np.asarray(results.est_index.values, dtype=np.int64),
            gt_index=np.asarray(results.gt_index.values, dtype=np.int64),
            status=np.asarray(results.match_status.values, dtype=np.int8),
            score=np.asarray(results.matching_score.values, dtype=np.float64),
            threshold=np.asarray(results.threshold.values, dtype=np.float64),
        )

    @classmethod
    def empty(cls) -> MatchTable:
        return cls(
            est_index=np.empty(0, dtype=np.int64),
            gt_index=np.empty(0, dtype=np.int64),
            status=np.empty(0, dtype=np.int8),
            score=np.empty(0, dtype=np.float64),
            threshold=np.empty(0, dtype=np.float64),
        )

    def to_results(self) -> MatchResults:
        if len(self.status) == 0:
            return MatchResults.empty()
        return MatchResults(
            est_index=self.est_index,
            gt_index=self.gt_index,
            matching_score=self.score,
            match_status=self.status,
            threshold=self.threshold,
        )

    def __len__(self) -> int:
        return len(self.status)

    @property
    def tp_rows(self) -> np.ndarray:
        return np.flatnonzero(self.status == int(MatchStatus.TP))

    @property
    def fp_rows(self) -> np.ndarray:
        return np.flatnonzero(self.status == int(MatchStatus.FP))

    @property
    def fn_rows(self) -> np.ndarray:
        return np.flatnonzero(self.status == int(MatchStatus.FN))

    @property
    def tp_est(self) -> np.ndarray:
        """Estimation rows of the TP pairs."""
        return self.est_index[self.tp_rows]

    @property
    def tp_gt(self) -> np.ndarray:
        """Ground truth rows of the TP pairs."""
        return self.gt_index[self.tp_rows]

    @property
    def tp_score(self) -> np.ndarray:
        return self.score[self.tp_rows]

    @property
    def fp_est(self) -> np.ndarray:
        return self.est_index[self.fp_rows]

    @property
    def fn_gt(self) -> np.ndarray:
        return self.gt_index[self.fn_rows]

    def select(self, rows: np.ndarray) -> MatchTable:
        return MatchTable(
            est_index=self.est_index[rows],
            gt_index=self.gt_index[rows],
            status=self.status[rows],
            score=self.score[rows],
            threshold=self.threshold[rows],
        )


def _remap(indices: np.ndarray, mapping: np.ndarray) -> np.ndarray:
    """Map row indices through `mapping`, keeping -1 (no counterpart) as -1."""
    if len(mapping) == 0:
        return np.full(len(indices), -1, dtype=np.int64)
    return np.where(indices >= 0, mapping[np.clip(indices, 0, None)], -1)


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def euler_zyx(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """(N, 3) of (roll, pitch, yaw) from xyzw quaternions."""
    if len(quaternion_xyzw) == 0:
        return np.empty((0, 3), dtype=np.float64)
    yaw_pitch_roll = Rotation.from_quat(np.asarray(quaternion_xyzw, dtype=np.float64)).as_euler(
        "ZYX"
    )
    return yaw_pitch_roll[:, ::-1]


@dataclass(frozen=True)
class PairErrors:
    """Errors of the TP pairs, ground truth minus estimation (perception_eval convention)."""

    position: np.ndarray
    """(n, 3) in the native frame."""
    position_bl: np.ndarray
    """(n, 3) in base_link."""
    heading: np.ndarray
    """(n, 3) as (roll, pitch, yaw) in [-pi, pi]."""
    velocity: np.ndarray
    """(n, 3) in the native frame, NaN when either side has no velocity."""
    velocity_bl: np.ndarray
    """(n, 3) in base_link."""
    size: np.ndarray
    """(n, 3) as (width, length, height)."""
    plane_distance: np.ndarray
    """(n,) the pass/fail matching score."""

    @property
    def bev(self) -> np.ndarray:
        """(n,) distance between the two centers in the xy plane."""
        return np.linalg.norm(self.position[:, :2], axis=1)


@dataclass(frozen=True)
class PerceptionFrameRecord:
    """
    One evaluated frame.

    Attributes:
        frame_index (int): 1-based position of the frame in the evaluated sequence (the FRAME
            timeline of the scene store).
        frame_name (str): Sample index of the matched ground truth frame in the dataset.
        unix_time_ns (int): Header stamp of the estimation.
        ground_truth_unix_time_ns (int): Timestamp of the ground truth frame.
        frame_id (str): Frame of the native tables (base_link or map).
        labels (LabelRegistry): Shared registry.
        policy (str): Label matching policy.
        estimation (ObjectTable): Kept estimations.
        ground_truth (ObjectTable): Kept ground truths.
        matches (MatchTable): Pass/fail verdicts over the kept rows.
        frame_metrics (dict[str, float]): Per-frame mAP / mAPH when they were computed.
        raw_chunks (tuple[Chunk, ...]): The unfiltered chunks of the frame (estimation, ground
            truth, ego transform), kept for the recording.

    """

    frame_index: int
    frame_name: str
    unix_time_ns: int
    ground_truth_unix_time_ns: int
    frame_id: str
    labels: LabelRegistry
    policy: str
    estimation: ObjectTable
    ground_truth: ObjectTable
    matches: MatchTable
    frame_metrics: dict[str, float]
    raw_chunks: tuple[Chunk, ...] = ()

    # -- counts -------------------------------------------------------------------------

    @property
    def num_tp(self) -> int:
        return len(self.matches.tp_rows)

    @property
    def num_fp(self) -> int:
        return len(self.matches.fp_rows)

    @property
    def num_fn(self) -> int:
        return len(self.matches.fn_rows)

    @property
    def num_gt(self) -> int:
        """Number of kept ground truths, `PassFailResult.get_num_gt()`."""
        return self.num_tp + self.num_fn

    @property
    def num_success(self) -> int:
        return self.num_tp

    @property
    def num_fail(self) -> int:
        return self.num_fp + self.num_fn

    # -- labels -------------------------------------------------------------------------

    def tp_est_labels(self) -> tuple[str, ...]:
        return self.labels.decode(self.estimation.class_id[self.matches.tp_est])

    def tp_gt_labels(self) -> tuple[str, ...]:
        return self.labels.decode(self.ground_truth.class_id[self.matches.tp_gt])

    def fp_labels(self) -> tuple[str, ...]:
        return self.labels.decode(self.estimation.class_id[self.matches.fp_est])

    def fn_labels(self) -> tuple[str, ...]:
        return self.labels.decode(self.ground_truth.class_id[self.matches.fn_gt])

    def is_label_correct(self) -> np.ndarray:
        """Whether each TP pair satisfies the label policy, `is_label_correct` of perception_eval."""
        est_class = self.estimation.class_id[self.matches.tp_est]
        gt_class = self.ground_truth.class_id[self.matches.tp_gt]
        if len(est_class) == 0:
            return np.empty(0, dtype=np.bool_)
        matrix = policy_matrix(self.policy, est_class, gt_class, self.labels)
        return np.diagonal(matrix)

    # -- errors -------------------------------------------------------------------------

    def pair_errors(self) -> PairErrors:
        est_rows = self.matches.tp_est
        gt_rows = self.matches.tp_gt
        est = self.estimation
        gt = self.ground_truth
        return PairErrors(
            position=gt.position[gt_rows] - est.position[est_rows],
            position_bl=gt.position_bl[gt_rows] - est.position_bl[est_rows],
            heading=wrap_angle(
                euler_zyx(gt.quaternion[gt_rows]) - euler_zyx(est.quaternion[est_rows])
            ),
            velocity=gt.velocity[gt_rows] - est.velocity[est_rows],
            velocity_bl=gt.velocity_bl[gt_rows] - est.velocity_bl[est_rows],
            size=gt.size[gt_rows] - est.size[est_rows],
            plane_distance=self.matches.tp_score,
        )

    # -- filtering ----------------------------------------------------------------------

    def filtered(self, *, est_keep: np.ndarray, gt_keep: np.ndarray) -> PerceptionFrameRecord:
        """
        Return the record narrowed to the rows which pass.

        A TP pair survives only when both sides pass, a FP when the estimation passes and a FN
        when the ground truth passes. This is the rule of perception_eval's
        `filter_frame_by_distance` / `filter_frame_by_region`.
        """
        est_keep = np.asarray(est_keep, dtype=np.bool_)
        gt_keep = np.asarray(gt_keep, dtype=np.bool_)
        est_rows = np.flatnonzero(est_keep)
        gt_rows = np.flatnonzero(gt_keep)
        est_map = np.full(len(est_keep), -1, dtype=np.int64)
        est_map[est_rows] = np.arange(len(est_rows))
        gt_map = np.full(len(gt_keep), -1, dtype=np.int64)
        gt_map[gt_rows] = np.arange(len(gt_rows))

        matches = self.matches
        new_est = _remap(matches.est_index, est_map)
        new_gt = _remap(matches.gt_index, gt_map)
        is_tp = matches.status == int(MatchStatus.TP)
        is_fp = matches.status == int(MatchStatus.FP)
        is_fn = matches.status == int(MatchStatus.FN)
        keep_rows = (
            (is_tp & (new_est >= 0) & (new_gt >= 0))
            | (is_fp & (new_est >= 0))
            | (is_fn & (new_gt >= 0))
        )
        new_matches = MatchTable(
            est_index=new_est[keep_rows],
            gt_index=new_gt[keep_rows],
            status=matches.status[keep_rows],
            score=matches.score[keep_rows],
            threshold=matches.threshold[keep_rows],
        )
        return replace(
            self,
            estimation=self.estimation.select(est_rows),
            ground_truth=self.ground_truth.select(gt_rows),
            matches=new_matches,
        )

    def filter_by_distance(self, min_distance: float, max_distance: float) -> PerceptionFrameRecord:
        """Keep the objects whose base_link xy distance is in `[min_distance, max_distance)`."""

        def keep(distance: np.ndarray) -> np.ndarray:
            return (distance >= min_distance) & (distance < max_distance)

        return self.filtered(
            est_keep=keep(self.estimation.bev_distance()),
            gt_keep=keep(self.ground_truth.bev_distance()),
        )

    def filter_by_region(
        self,
        x_position: tuple[float, float] | None,
        y_position: tuple[float, float] | None,
    ) -> PerceptionFrameRecord:
        """Keep the objects whose base_link position is inside the ranges (None = no limit)."""

        def keep(position: np.ndarray) -> np.ndarray:
            mask = np.ones(len(position), dtype=np.bool_)
            if x_position is not None:
                mask &= (position[:, 0] >= x_position[0]) & (position[:, 0] <= x_position[1])
            if y_position is not None:
                mask &= (position[:, 1] >= y_position[0]) & (position[:, 1] <= y_position[1])
            return mask

        return self.filtered(
            est_keep=keep(self.estimation.position_bl),
            gt_keep=keep(self.ground_truth.position_bl),
        )

    # -- chunks for the scene store ----------------------------------------------------

    def matching_chunk(self) -> Chunk:
        """Return the pass/fail verdicts as a chunk at this record's frame index."""
        return self.matches.to_results().to_chunk(
            PASS_FAIL_MATCHING_PATH,
            at=TimePoint.at(frame=self.frame_index),
            frame_id="base_link",
        )

    def kept_chunks(self) -> tuple[Chunk, ...]:
        """Return the kept entities (native and base_link, both sides) and the pass/fail chunk."""
        return (
            self.estimation.native,
            self.estimation.base_link,
            self.ground_truth.native,
            self.ground_truth.base_link,
            self.matching_chunk(),
        )

    def with_frame_index(self, frame_index: int) -> PerceptionFrameRecord:
        """Return the record moved to another frame index (used when scenes are concatenated)."""

        def move(table: ObjectTable) -> ObjectTable:
            return replace(
                table,
                native=restamp(table.native, frame_index),
                base_link=restamp(table.base_link, frame_index),
            )

        return replace(
            self,
            frame_index=frame_index,
            estimation=move(self.estimation),
            ground_truth=move(self.ground_truth),
            raw_chunks=tuple(restamp(chunk, frame_index) for chunk in self.raw_chunks),
        )
