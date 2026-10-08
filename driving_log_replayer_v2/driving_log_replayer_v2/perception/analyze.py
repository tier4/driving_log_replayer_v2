# Copyright (c) 2025 TIER IV.inc
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

"""Detailed analysis of the perception results, per distance bin, exported as a csv table."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd

from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import apply_statistics
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import build_scene_store
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import (
    confusion_matrix_dict,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import ERROR_METRICS
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import ERROR_STATISTICS
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import error_statistics
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import final_score
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import read_archive
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import RECORDING_DIRNAME
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import run_scene_pipeline

if TYPE_CHECKING:
    from collections.abc import Callable
    from collections.abc import Sequence

    from t4perceval import InstanceRegistry
    from t4perceval import LabelRegistry
    from t4perceval import Store

    from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig
    from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import (
        PerceptionFrameRecord,
    )


@dataclass(frozen=True)
class AnalysisResult:
    score: dict[str, dict[str, float]]
    """Metric name -> {"ALL" | label: value}."""
    error: dict[str, dict[str, dict[str, float]]]
    """Label -> statistic -> error metric -> value."""
    confusion_matrix: dict[str, dict[str, int]]


class PerceptionAnalysis:
    """
    Scene metrics of the evaluated frames, optionally narrowed to a distance range.

    Replaces `PerceptionAnalyzer3D`: the frame records carry everything the analysis needs, so
    a distance bin is a filtered copy of the records on which the scene pipeline runs again.
    """

    def __init__(
        self,
        records: Sequence[PerceptionFrameRecord],
        config: EvaluationConfig,
        labels: LabelRegistry,
        instances: InstanceRegistry | None = None,
        *,
        store: Store | None = None,
    ) -> None:
        self._records = list(records)
        self._config = config
        self._labels = labels
        self._instances = instances
        self._store = store

    @property
    def config(self) -> EvaluationConfig:
        return self._config

    @property
    def records(self) -> list[PerceptionFrameRecord]:
        return self._records

    @property
    def labels(self) -> LabelRegistry:
        return self._labels

    def select(self, distance: tuple[float, float] | None = None) -> list[PerceptionFrameRecord]:
        if distance is None:
            return self._records
        return [record.filter_by_distance(*distance) for record in self._records]

    def analyze(self, distance: tuple[float, float] | None = None) -> AnalysisResult:
        """Compute the score, the error statistics and the confusion matrix of the records."""
        records = self.select(distance)
        if distance is None and self._store is not None:
            store = self._store
        else:
            store = build_scene_store(records)
            run_scene_pipeline(store, self._config, self._labels, self._instances)
        target_labels = self._config.target_labels
        return AnalysisResult(
            score=final_score(store, records, self._config, self._labels),
            error=error_statistics(records, target_labels),
            confusion_matrix=confusion_matrix_dict(store, self._labels, target_labels),
        )

    def consecutive_fn_spans(
        self,
        distance: tuple[float, float] | None,
        label: str,
        statistics: Callable = np.max,
    ) -> list[float]:
        """
        One statistic per ground truth object of the spans it was consecutively missed.

        A span is the time [s] between the first and the last frame of a run of FN verdicts in
        consecutive evaluated frames. A single FN frame is not a span.
        """
        records = self.select(distance)
        fn_frames: dict[str, list[tuple[int, int]]] = {}
        for record in records:
            fn_labels = record.fn_labels()
            for row, gt_row in enumerate(record.matches.fn_gt):
                if label != "ALL" and fn_labels[row] != label:
                    continue
                uuid = record.ground_truth.uuids[int(gt_row)]
                if uuid is None:
                    continue
                fn_frames.setdefault(uuid, []).append((record.frame_index, record.unix_time_ns))

        object_spans: dict[str, list[float]] = {}
        for uuid, frames in fn_frames.items():
            frames.sort()
            spans: list[float] = []
            run_start: int | None = None
            for (frame, timestamp), (next_frame, _) in zip(
                frames, [*frames[1:], (-1, 0)], strict=True
            ):
                consecutive = next_frame == frame + 1
                if consecutive and run_start is None:
                    run_start = timestamp
                elif not consecutive and run_start is not None:
                    spans.append((timestamp - run_start) * 1e-9)
                    run_start = None
            object_spans[uuid] = spans
        return [statistics(spans) for spans in object_spans.values() if len(spans) > 0]


def analyze(
    analysis: PerceptionAnalysis,
    save_path: Path,
    max_distance: str,
    distance_interval: str,
    topic_name: str,
) -> None:
    """Analyze evaluation results in detail and export them as a flattened csv table."""
    evaluation_task = analysis.config.evaluation_task
    sample = analysis.analyze()
    if not sample.score:
        pd.DataFrame([]).to_csv(save_path.joinpath("analysis_result.csv"), index=False)
        return
    distance_range = tuple(
        (i * int(distance_interval), (i + 1) * int(distance_interval))
        for i in range(int(max_distance) // int(distance_interval))
    )
    labels = ["ALL", *analysis.config.target_labels]
    score_metrics = list(sample.score.keys())

    all_row = []
    for distance_min, distance_max in distance_range:
        analysis_result = analysis.analyze(distance=(distance_min, distance_max))
        for label in labels:
            row: dict[str, str | float] = {
                "evaluation_task": evaluation_task,
                "topic_name": topic_name,
                "label": label,
                "distance": f"{distance_min}-{distance_max}",
            }
            for score in score_metrics:
                row[score] = analysis_result.score.get(score, {}).get(label, float("nan"))
            for error in ERROR_METRICS:
                for stat in ERROR_STATISTICS:
                    row[error + "_" + stat] = (
                        analysis_result.error.get(label, {}).get(stat, {}).get(error, float("nan"))
                    )
            consecutive_fn_spans = analysis.consecutive_fn_spans(
                distance=(distance_min, distance_max), label=label, statistics=np.max
            )
            for stat in ERROR_STATISTICS:
                row["consecutive_fn_spans" + "_" + stat] = apply_statistics(
                    consecutive_fn_spans, stat
                )
            all_row.append(row)
    pd.DataFrame(all_row).fillna("nan").to_csv(
        save_path.joinpath("analysis_result.csv"), index=False
    )


def load_analysis(scene_result: Path) -> PerceptionAnalysis:
    """
    Load one archive directory, or every `scene_result.t4eval` below a directory.

    Several archives are concatenated into one analysis; their frame indices are offset so
    that the frames do not collide.
    """
    if scene_result.name == RECORDING_DIRNAME:
        scene_result = scene_result.parent
    if (scene_result / RECORDING_DIRNAME).is_dir():
        archive_dirs = [scene_result]
    else:
        archive_dirs = sorted(p.parent for p in scene_result.rglob(RECORDING_DIRNAME))
    if not archive_dirs:
        err_msg = f"No {RECORDING_DIRNAME} found under {scene_result}"
        raise ValueError(err_msg)

    archives = [read_archive(archive_dir) for archive_dir in archive_dirs]
    first = archives[0]
    if len(archives) == 1:
        return PerceptionAnalysis(first.records, first.config, first.labels, first.instances)

    records = []
    offset = 0
    for archive in archives:
        if archive.labels.fingerprint() != first.labels.fingerprint():
            err_msg = f"Label registries differ between archives: {archive_dirs}"
            raise ValueError(err_msg)
        for record in archive.records:
            records.append(record.with_frame_index(record.frame_index + offset))
        offset += max((record.frame_index for record in archive.records), default=0)
    return PerceptionAnalysis(records, first.config, first.labels, None)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze perception result")
    parser.add_argument(
        "--scene-result",
        required=True,
        type=Path,
        help=f"Archive directory of a topic (holding {RECORDING_DIRNAME}) or a directory containing several of them",
    )
    parser.add_argument(
        "--save-path", required=True, type=Path, help="Directory path to save the output csv file"
    )
    parser.add_argument("--max-distance", required=True, help="Maximum distance for analysis")
    parser.add_argument(
        "--distance-interval", required=True, help="Distance interval for analysis."
    )
    parser.add_argument("--topic-name", default="", help="Evaluated topic name")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    analysis = load_analysis(args.scene_result)
    analyze(
        analysis,
        args.save_path,
        args.max_distance,
        args.distance_interval,
        args.topic_name,
    )


if __name__ == "__main__":
    main()
