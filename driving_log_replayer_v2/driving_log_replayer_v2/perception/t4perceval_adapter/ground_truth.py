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

"""Ground truth of a T4 dataset as a t4perceval recording, with the frame lookup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import TYPE_CHECKING

from attrs import evolve
import numpy as np
from t4perceval import FRAME
from t4perceval import TIMESTAMP
from t4perceval.core.timeline import TimeColumn
from t4perceval.descriptors import INSTANCE_ID
from t4perceval.descriptors import NUM_POINTS

from driving_log_replayer_v2.perception.t4perceval_adapter.labels import build_label_registry

if TYPE_CHECKING:
    import logging

    from t4perceval import InstanceRegistry
    from t4perceval import LabelRegistry
    from t4perceval import Recording
    from t4perceval.core.chunk import Chunk
    from t4perceval.core.descriptor import ComponentDescriptor

    from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig

GROUND_TRUTH_PATH = "/ground_truth/objects"
EGO_TRANSFORM_PATH = "/tf/base_link"
DEFAULT_TOLERANCE_NS = 75_000_000
"""Maximum time difference between an estimation and its ground truth frame (75 ms)."""

__all__ = (
    "DEFAULT_TOLERANCE_NS",
    "EGO_TRANSFORM_PATH",
    "GROUND_TRUTH_PATH",
    "GroundTruthFrame",
    "GroundTruthScene",
    "load_ground_truth",
    "restamp",
)


@dataclass(frozen=True)
class GroundTruthFrame:
    frame: int
    """Sample index in the scene (the `frame_name` of result.jsonl)."""
    timestamp_ns: int

    @property
    def frame_name(self) -> str:
        return str(self.frame)


def restamp(chunk: Chunk, frame: int) -> Chunk:
    """Return `chunk` with every partition moved to `frame` on the FRAME timeline."""
    indexes = tuple(
        TimeColumn(FRAME, np.full(len(index.times), frame, dtype=np.int64))
        if index.timeline == FRAME
        else index
        for index in chunk.indexes
    )
    return evolve(chunk, indexes=indexes)


class GroundTruthScene:
    """
    One scene of ground truth, indexed by time.

    Replaces `PerceptionEvaluationManager.get_ground_truth_now_frame`: the nearest frame within
    the tolerance is returned, and the same frame may serve several estimations.
    """

    def __init__(
        self,
        recording: Recording,
        *,
        objects_path: str = GROUND_TRUTH_PATH,
        transform_path: str = EGO_TRANSFORM_PATH,
    ) -> None:
        self._recording = recording
        self._objects_path = objects_path
        self._transform_path = transform_path
        frames: list[int] = []
        timestamps: list[int] = []
        for chunk in recording.chunks(objects_path):
            frame_index = chunk.index(FRAME)
            time_index = chunk.index(TIMESTAMP)
            if frame_index is None or time_index is None:
                err_msg = f"{objects_path} must be logged on both FRAME and TIMESTAMP timelines"
                raise ValueError(err_msg)
            frames.extend(int(f) for f in frame_index.times)
            timestamps.extend(int(t) for t in time_index.times)
        order = np.argsort(np.asarray(timestamps, dtype=np.int64), kind="stable")
        self._frames = np.asarray(frames, dtype=np.int64)[order]
        self._timestamps_ns = np.asarray(timestamps, dtype=np.int64)[order]

    @property
    def recording(self) -> Recording:
        return self._recording

    @property
    def labels(self) -> LabelRegistry:
        return self._recording.labels

    @property
    def instances(self) -> InstanceRegistry:
        return self._recording.instances

    def has_column(self, descriptor: ComponentDescriptor) -> bool:
        """Whether the ground truth objects carry `descriptor` (every chunk has the same columns)."""
        chunks = self._recording.chunks(self._objects_path)
        return bool(chunks) and descriptor in chunks[0].columns

    @property
    def has_num_points(self) -> bool:
        return self.has_column(NUM_POINTS)

    @property
    def has_instance_id(self) -> bool:
        return self.has_column(INSTANCE_ID)

    @property
    def frames(self) -> np.ndarray:
        return self._frames

    @property
    def timestamps_ns(self) -> np.ndarray:
        return self._timestamps_ns

    @property
    def num_frames(self) -> int:
        return len(self._frames)

    @property
    def has_transforms(self) -> bool:
        return bool(self._recording.chunks(self._transform_path))

    def nearest(
        self, timestamp_ns: int, *, tolerance_ns: int = DEFAULT_TOLERANCE_NS
    ) -> GroundTruthFrame | None:
        """Return the frame closest to `timestamp_ns`, or None if farther than the tolerance."""
        if self.num_frames == 0:
            return None
        position = int(np.searchsorted(self._timestamps_ns, timestamp_ns))
        candidates = [c for c in (position - 1, position) if 0 <= c < self.num_frames]
        best = min(candidates, key=lambda c: abs(int(self._timestamps_ns[c]) - timestamp_ns))
        if abs(int(self._timestamps_ns[best]) - timestamp_ns) > tolerance_ns:
            return None
        return GroundTruthFrame(int(self._frames[best]), int(self._timestamps_ns[best]))

    def objects_chunk(self, frame: int) -> Chunk:
        """Return the objects of `frame` as a single-partition chunk."""
        view = self._recording.latest_at(self._objects_path, timeline=FRAME, at=frame)
        chunk = view.to_chunk()
        if chunk.num_partitions != 1 or int(chunk.index(FRAME).times[0]) != frame:
            err_msg = f"No ground truth objects logged at frame {frame}"
            raise KeyError(err_msg)
        return chunk

    def transform_chunk(self, frame: int) -> Chunk | None:
        """Return the map -> base_link transform of `frame`, or None without transforms."""
        if not self.has_transforms:
            return None
        view = self._recording.latest_at(self._transform_path, timeline=FRAME, at=frame)
        chunk = view.to_chunk()
        if chunk.num_partitions != 1 or int(chunk.index(FRAME).times[0]) != frame:
            return None
        return chunk

    def frame_chunks(self, frame: int, *, at: int) -> list[Chunk]:
        """Return the chunks of `frame` re-stamped at evaluation frame `at`."""
        chunks = [restamp(self.objects_chunk(frame), at)]
        transform = self.transform_chunk(frame)
        if transform is not None:
            chunks.append(restamp(transform, at))
        return chunks


def _t4_importer_class() -> Any:
    from t4perceval.importer.t4 import T4Importer  # noqa: PLC0415

    return T4Importer


def _filtering_source(data_root: str, ignore_attributes: tuple[str, ...]) -> Any:
    """Return a `T4Source` which drops the boxes carrying one of `ignore_attributes`."""
    from t4perceval.importer.t4.source import T4Source  # noqa: PLC0415

    ignored = set(ignore_attributes)

    class AttributeFilteringT4Source(T4Source):
        def boxes3d(self, sample_data_token: str, **kwargs: Any) -> list[Any]:
            boxes = super().boxes3d(sample_data_token, **kwargs)
            return [
                box
                for box in boxes
                if not ignored.intersection(getattr(box.semantic_label, "attributes", ()) or ())
            ]

    return AttributeFilteringT4Source(data_root)


DEFAULT_LIDAR_CHANNEL = "LIDAR_CONCAT"


def lidar_channel(source: Any) -> str:
    """Return `LIDAR_CONCAT` when the dataset has it, otherwise its first non-camera channel."""
    channels = tuple(source.channels())
    if DEFAULT_LIDAR_CHANNEL in channels:
        return DEFAULT_LIDAR_CHANNEL
    for channel in channels:
        if not source.is_camera(channel):
            return channel
    err_msg = f"No lidar channel found in the dataset, channels: {channels}"
    raise ValueError(err_msg)


def load_ground_truth(
    t4_dataset_path: str,
    *,
    config: EvaluationConfig,
    instances: InstanceRegistry,
    labels: LabelRegistry | None = None,
    logger: logging.Logger | None = None,
) -> GroundTruthScene:
    """
    Load the first scene of a T4 dataset in the frame of the evaluation task.

    Args:
        t4_dataset_path (str): Dataset root.
        config (EvaluationConfig): Decides the frame (`coords`), the archetype and the ignored
            attributes.
        instances (InstanceRegistry): Shared registry of object identities.
        labels (LabelRegistry | None): Shared registry. Built from the dataset categories with
            `labels.build_label_registry` when None.
        logger (logging.Logger | None): Logger.

    Returns:
        GroundTruthScene: The loaded scene.

    """
    from t4perceval.importer.t4 import ImportOptions  # noqa: PLC0415
    from t4perceval.importer.t4 import SceneSelection  # noqa: PLC0415

    T4Importer = _t4_importer_class()  # noqa: N806
    ignore_attributes = tuple(
        dict.fromkeys((*config.ignore_attributes, *config.critical.ignore_attributes))
    )
    is_prediction = config.evaluation_task == "prediction"
    options = ImportOptions(
        kind_3d="predictions" if is_prediction else "trackings",
        coords=config.frame_id,
        future_seconds=config.future_seconds if is_prediction else 0.0,
        num_modes=config.prediction_num_modes if is_prediction else None,
        num_timesteps=config.prediction_num_timesteps if is_prediction else None,
        velocity="always",
        num_points="auto",
        visibility="auto",
        unknown_labels="unknown",
        instance_namespace="gt",
        strict=False,
    )
    if ignore_attributes:
        importer = T4Importer(
            _filtering_source(t4_dataset_path, ignore_attributes), options=options
        )
    else:
        importer = T4Importer.open(t4_dataset_path, options=options)
    scene_tokens = importer.scene_tokens()
    if len(scene_tokens) > 1 and logger is not None:
        logger.warning(
            "%s holds %d scenes, only the first one is evaluated.",
            t4_dataset_path,
            len(scene_tokens),
        )
    if labels is None:
        labels = build_label_registry(
            importer.label_registry().names,
            merge_similar_labels=config.merge_similar_labels,
            logger=logger,
        )
    channel = lidar_channel(importer.source)
    recording = importer.import_scene(
        labels=labels, instances=instances, selection=SceneSelection(channel_3d=channel)
    )
    scene = GroundTruthScene(recording)
    if logger is not None:
        logger.info(
            "Loaded %d ground truth frames from %s in frame '%s'.",
            scene.num_frames,
            t4_dataset_path,
            config.frame_id,
        )
    return scene
