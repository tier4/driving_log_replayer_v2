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
Scene-level metrics of the perception use case and the archive of a topic.

The frame records are concatenated into one scene store, the scene pipeline runs once over
it, and the metrics are read back into the `FinalScore` dictionary of result.jsonl. The
same store is written as a t4perceval recording (`scene_result.t4eval`) next to
`evaluation_config.json` and `frame_index.json`.
"""

from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING

import numpy as np
from t4perceval import concat_chunks
from t4perceval import ConfusionMatrix
from t4perceval import FRAME
from t4perceval import InstanceRegistry
from t4perceval import MetricValues
from t4perceval import Recording
from t4perceval import RecordingMetadata
from t4perceval import SourceInfo
from t4perceval import Store
from t4perceval import TimeRange
from t4perceval.component import ALL_CLASSES
from t4perceval.io import read_recording
from t4perceval.io import write_recording
from t4perceval.system.base import SystemContext

from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import MatchTable
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import ObjectTable
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import PerceptionFrameRecord
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import build_scene_pipeline
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import CONFUSION_MATRIX_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import DISPLACEMENT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import (
    ESTIMATION_KEPT_BASE_LINK_PATH,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import ESTIMATION_KEPT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import (
    GROUND_TRUTH_KEPT_BASE_LINK_PATH,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import GROUND_TRUTH_KEPT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import MATCHING_MODE_NAMES
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import PASS_FAIL_MATCHING_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import valid_threshold_sets

if TYPE_CHECKING:
    from collections.abc import Sequence

    from t4perceval import LabelRegistry
    from t4perceval.core.chunk import Chunk
    from t4perceval.system import Pipeline

RECORDING_DIRNAME = "scene_result.t4eval"
CONFIG_FILENAME = "evaluation_config.json"
FRAME_INDEX_FILENAME = "frame_index.json"

ERROR_METRICS: tuple[str, ...] = ("x", "y", "yaw", "length", "width", "vx", "vy", "nn_plane")
ERROR_STATISTICS: tuple[str, ...] = ("average", "rms", "std", "max", "min", "percentile_99")

__all__ = (
    "CONFIG_FILENAME",
    "ERROR_METRICS",
    "ERROR_STATISTICS",
    "FRAME_INDEX_FILENAME",
    "RECORDING_DIRNAME",
    "SceneArchive",
    "apply_statistics",
    "build_scene_store",
    "confusion_matrix_dict",
    "error_statistics",
    "final_score",
    "pass_fail_rates",
    "read_archive",
    "read_metric",
    "run_scene_pipeline",
    "to_recording",
    "write_archive",
)


# -- store -----------------------------------------------------------------------------------


def build_scene_store(records: Sequence[PerceptionFrameRecord]) -> Store:
    """Concatenate the chunks of the records into one store, one chunk per entity."""
    groups: dict[Any, list[Chunk]] = defaultdict(list)
    for record in records:
        for chunk in (*record.raw_chunks, *record.kept_chunks()):
            groups[chunk.entity_path].append(chunk)
    store = Store()
    for chunks in groups.values():
        store.send_chunk(concat_chunks(chunks))
    return store


def run_scene_pipeline(
    store: Store,
    config: EvaluationConfig,
    labels: LabelRegistry,
    instances: InstanceRegistry | None = None,
) -> Pipeline:
    """Run the scene pipeline over every frame of `store`."""
    pipeline = build_scene_pipeline(config)
    pipeline.run(
        SystemContext(store, FRAME, labels=labels, instances=instances),
        at=TimeRange.everything(),
    )
    return pipeline


# -- readers ---------------------------------------------------------------------------------


def _nan_mean(values: Sequence[float]) -> float:
    finite = [v for v in values if not np.isnan(v)]
    return float(np.mean(finite)) if finite else float("nan")


def read_metric(
    store: Store,
    path: str,
    labels: LabelRegistry,
    target_labels: Sequence[str],
) -> dict[str, float] | None:
    """
    Read a `MetricValues` entity as `{"ALL": value, "<label>": value}`.

    The per-class rows are averaged per class (over thresholds when the metric has several),
    `ALL` is the mean of the target labels that have a value. None when nothing was logged.
    """
    view = store.range(path, timeline=FRAME, time_range=TimeRange.everything())
    if not len(view):
        return None
    metric = view.materialize(MetricValues)
    per_class: dict[int, list[float]] = defaultdict(list)
    for class_id, value in zip(metric.class_id.values, metric.value.values, strict=True):
        if int(class_id) == ALL_CLASSES:
            continue
        per_class[int(class_id)].append(float(value))
    result: dict[str, float] = {"ALL": float("nan")}
    for label in target_labels:
        result[label] = _nan_mean(per_class.get(labels.class_id(label), []))
    result["ALL"] = _nan_mean([result[label] for label in target_labels])
    return result


def pass_fail_rates(
    records: Sequence[PerceptionFrameRecord], target_labels: Sequence[str]
) -> dict[str, dict[str, float]]:
    """TP / FP / FN / TN rates of the pass/fail verdicts, as `summarize_ratio` of perception_eval."""
    tp: dict[str, int] = defaultdict(int)
    fp: dict[str, int] = defaultdict(int)
    fn: dict[str, int] = defaultdict(int)
    for record in records:
        for label in record.tp_gt_labels():
            tp[label] += 1
        for label in record.fp_labels():
            fp[label] += 1
        for label in record.fn_labels():
            fn[label] += 1
    all_labels = ["ALL", *target_labels]
    rates = {status: dict.fromkeys(all_labels, 0.0) for status in ("TP", "FP", "TN", "FN")}
    for label in all_labels:
        if label == "ALL":
            num_tp, num_fp, num_fn = sum(tp.values()), sum(fp.values()), sum(fn.values())
        else:
            num_tp, num_fp, num_fn = tp[label], fp[label], fn[label]
        num_gt = num_tp + num_fn
        if num_gt == 0:
            continue
        num_det = num_tp + num_fp
        rates["TP"][label] = num_tp / num_gt
        rates["FP"][label] = num_fp / num_det if num_det != 0 else 0.0
        rates["FN"][label] = num_fn / num_gt
    return rates


def final_score(
    store: Store,
    records: Sequence[PerceptionFrameRecord],
    config: EvaluationConfig,
    labels: LabelRegistry,
) -> dict[str, dict[str, float]]:
    """Return the `FinalScore.Score` dictionary: rates, AP/APH per mode, CLEAR, displacement."""
    target_labels = config.target_labels
    is_tracking = config.evaluation_task in ("tracking", "prediction")
    metric_paths: list[tuple[str, str]] = []
    for family, sets in config.threshold_families.items():
        if not valid_threshold_sets(family, sets):
            continue
        mode = MATCHING_MODE_NAMES[family]
        metric_paths.append((f"AP({mode})", f"/metrics/map/{family}"))
        metric_paths.append((f"APH({mode})", f"/metrics/maph/{family}"))
        if is_tracking:
            metric_paths.append((f"MOTA({mode})", f"/metrics/clear/{family}/mota"))
            metric_paths.append((f"MOTP({mode})", f"/metrics/clear/{family}/motp"))
            metric_paths.append((f"IDswitch({mode})", f"/metrics/clear/{family}/id_switch"))
    if config.evaluation_task == "prediction":
        metric_paths.append(("ADE", f"{DISPLACEMENT_PATH}/ade"))
        metric_paths.append(("FDE", f"{DISPLACEMENT_PATH}/fde"))
        metric_paths.append(("MissRate", f"{DISPLACEMENT_PATH}/miss_rate"))

    score: dict[str, dict[str, float]] = pass_fail_rates(records, target_labels)
    for key, path in metric_paths:
        values = read_metric(store, path, labels, target_labels)
        if values is not None:
            score[key] = values
    return score


def apply_statistics(value: Sequence[float], statistics: str) -> float:
    """One statistic of an error array, NaN entries removed (as perception_eval's analyzer)."""
    functions = {
        "average": np.average,
        "rms": lambda v: np.sqrt(np.square(v).mean()),
        "std": np.std,
        "max": lambda v: np.max(np.abs(v)),
        "min": lambda v: np.min(np.abs(v)),
        "percentile_99": lambda v: np.percentile(np.abs(v), 99),
    }
    if statistics not in functions:
        err_msg = f"Invalid statistics: {statistics}"
        raise ValueError(err_msg)
    values = np.asarray(value, dtype=np.float64)
    values = values[~np.isnan(values)]
    if len(values) == 0:
        return float("nan")
    return float(functions[statistics](values))


def error_statistics(
    records: Sequence[PerceptionFrameRecord], target_labels: Sequence[str]
) -> dict[str, dict[str, dict[str, float]]]:
    """
    Return the `FinalScore.Error` dictionary: `{label: {statistic: {metric: value}}}`.

    Errors are ground truth minus estimation over the TP pairs, in base_link, grouped by the
    ground truth label. `nn_plane` is the plane distance of the pair.
    """
    columns: dict[str, dict[str, list[float]]] = {
        label: {metric: [] for metric in ERROR_METRICS} for label in ("ALL", *target_labels)
    }
    for record in records:
        errors = record.pair_errors()
        gt_labels = record.tp_gt_labels()
        values = {
            "x": errors.position_bl[:, 0],
            "y": errors.position_bl[:, 1],
            "yaw": errors.heading[:, 2],
            "length": errors.size[:, 1],
            "width": errors.size[:, 0],
            "vx": errors.velocity_bl[:, 0],
            "vy": errors.velocity_bl[:, 1],
            "nn_plane": errors.plane_distance,
        }
        for pair, label in enumerate(gt_labels):
            for metric, array in values.items():
                columns["ALL"][metric].append(float(array[pair]))
                if label in columns:
                    columns[label][metric].append(float(array[pair]))
    return {
        label: {
            statistic: {
                metric: apply_statistics(metrics[metric], statistic) for metric in ERROR_METRICS
            }
            for statistic in ERROR_STATISTICS
        }
        for label, metrics in columns.items()
    }


def confusion_matrix_dict(
    store: Store, labels: LabelRegistry, target_labels: Sequence[str]
) -> dict[str, dict[str, int]]:
    """Return the `FinalScore.ConfusionMatrix` dictionary: `{est_label: {gt_label: count}}`."""
    view = store.range(CONFUSION_MATRIX_PATH, timeline=FRAME, time_range=TimeRange.everything())
    if not len(view):
        return {}
    matrix = view.materialize(ConfusionMatrix).as_matrix(
        [labels.class_id(label) for label in target_labels], include_background=False
    )
    return {
        est_label: {
            gt_label: int(matrix[gt_index, est_index])
            for gt_index, gt_label in enumerate(target_labels)
        }
        for est_index, est_label in enumerate(target_labels)
    }


# -- archive ---------------------------------------------------------------------------------


def to_recording(
    store: Store,
    labels: LabelRegistry,
    instances: InstanceRegistry,
    *,
    config: EvaluationConfig,
    topic: str,
    pipeline: Pipeline | None = None,
) -> Recording:
    metadata = RecordingMetadata(
        sources=(
            SourceInfo("rosbag", topic, topic=topic, entity_path=ESTIMATION_KEPT_PATH),
            SourceInfo("t4", "", entity_path=GROUND_TRUTH_KEPT_PATH),
        ),
        pipeline=tuple(type(system).__name__ for system in pipeline) if pipeline else (),
        frame_id=config.frame_id,
        tags={"evaluation_task": config.evaluation_task},
    )
    return Recording.of(store, labels=labels, instances=instances, metadata=metadata)


def frame_index_entries(records: Sequence[PerceptionFrameRecord]) -> list[dict[str, Any]]:
    return [
        {
            "frame_index": record.frame_index,
            "frame_name": record.frame_name,
            "unix_time_ns": record.unix_time_ns,
            "ground_truth_unix_time_ns": record.ground_truth_unix_time_ns,
            "frame_id": record.frame_id,
            "policy": record.policy,
        }
        for record in records
    ]


def write_archive(
    directory: str | Path,
    *,
    store: Store,
    records: Sequence[PerceptionFrameRecord],
    config: EvaluationConfig,
    labels: LabelRegistry,
    instances: InstanceRegistry,
    topic: str,
    pipeline: Pipeline | None = None,
) -> Path:
    """Write the recording, the config and the frame index of one topic into `directory`."""
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    recording = to_recording(
        store, labels, instances, config=config, topic=topic, pipeline=pipeline
    )
    write_recording(recording, root / RECORDING_DIRNAME, exist_ok=True)
    (root / CONFIG_FILENAME).write_text(json.dumps(config.to_dict(), indent=2) + "\n")
    (root / FRAME_INDEX_FILENAME).write_text(
        json.dumps(frame_index_entries(records), indent=2) + "\n"
    )
    return root


class SceneArchive:
    """A written archive read back: the recording, the config and the frame records."""

    def __init__(
        self,
        recording: Recording,
        config: EvaluationConfig,
        records: list[PerceptionFrameRecord],
    ) -> None:
        self.recording = recording
        self.config = config
        self.records = records

    @property
    def labels(self) -> LabelRegistry:
        return self.recording.labels

    @property
    def instances(self) -> InstanceRegistry:
        return self.recording.instances


def _chunk_at(recording: Recording, path: str, frame_index: int) -> Chunk:
    return recording.range(
        path, timeline=FRAME, time_range=TimeRange.single(frame_index)
    ).to_chunk()


def records_from_recording(
    recording: Recording, frame_index: list[dict[str, Any]]
) -> list[PerceptionFrameRecord]:
    """Rebuild the frame records (without covariances) from a recording and its frame index."""
    records: list[PerceptionFrameRecord] = []
    for entry in frame_index:
        k = int(entry["frame_index"])
        estimation = ObjectTable.from_chunks(
            _chunk_at(recording, ESTIMATION_KEPT_PATH, k),
            _chunk_at(recording, ESTIMATION_KEPT_BASE_LINK_PATH, k),
            instances=recording.instances,
        )
        ground_truth = ObjectTable.from_chunks(
            _chunk_at(recording, GROUND_TRUTH_KEPT_PATH, k),
            _chunk_at(recording, GROUND_TRUTH_KEPT_BASE_LINK_PATH, k),
            instances=recording.instances,
        )
        matching = _chunk_at(recording, PASS_FAIL_MATCHING_PATH, k)
        matches = MatchTable.from_chunk(matching) if matching.num_rows else MatchTable.empty()
        records.append(
            PerceptionFrameRecord(
                frame_index=k,
                frame_name=str(entry["frame_name"]),
                unix_time_ns=int(entry["unix_time_ns"]),
                ground_truth_unix_time_ns=int(entry["ground_truth_unix_time_ns"]),
                frame_id=str(entry["frame_id"]),
                labels=recording.labels,
                policy=str(entry["policy"]),
                estimation=estimation,
                ground_truth=ground_truth,
                matches=matches,
                frame_metrics={},
            )
        )
    return records


def read_archive(directory: str | Path) -> SceneArchive:
    """Read an archive written by `write_archive`."""
    root = Path(directory)
    recording = read_recording(root / RECORDING_DIRNAME)
    config = EvaluationConfig.from_dict(json.loads((root / CONFIG_FILENAME).read_text()))
    frame_index = json.loads((root / FRAME_INDEX_FILENAME).read_text())
    return SceneArchive(recording, config, records_from_recording(recording, frame_index))
