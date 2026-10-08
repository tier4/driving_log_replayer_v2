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

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import TYPE_CHECKING

from t4perceval import FRAME
from t4perceval import InstanceRegistry
from t4perceval import Store
from t4perceval import TimePoint
from t4perceval import TimeRange
from t4perceval.descriptors import MASK
from t4perceval.system.base import SystemContext

from driving_log_replayer_v2.perception.analyze import PerceptionAnalysis
from driving_log_replayer_v2.perception.t4perceval_adapter.config import from_scenario
from driving_log_replayer_v2.perception.t4perceval_adapter.conversions import ConversionContext
from driving_log_replayer_v2.perception.t4perceval_adapter.conversions import EstimationFrame
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import MatchTable
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import ObjectTable
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import PerceptionFrameRecord
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import load_ground_truth
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import build_scene_store
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import (
    confusion_matrix_dict,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import error_statistics
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import final_score
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import read_metric
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import run_scene_pipeline
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import write_archive
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import build_frame_pipeline
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import (
    ESTIMATION_KEPT_BASE_LINK_PATH,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import ESTIMATION_KEPT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import ESTIMATION_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import FRAME_METRICS_ROOT
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import (
    GROUND_TRUTH_KEPT_BASE_LINK_PATH,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import GROUND_TRUTH_KEPT_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import PASS_FAIL_MATCHING_PATH
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.evaluator import InvalidReason

if TYPE_CHECKING:
    from t4perceval.core.chunk import Chunk

    from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig
    from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import GroundTruthFrame
    from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
    from driving_log_replayer_v2.post_process.runner import ConvertedData


class PerceptionInvalidReason(InvalidReason):
    INVALID_ESTIMATED_OBJECTS = "Invalid Estimated Objects"
    NO_GROUND_TRUTH = "No Ground Truth"
    IGNORED_FRAME = "Ignored Frame"


class PerceptionEvaluator(Evaluator):
    """
    Evaluate one topic with t4perceval.

    Every evaluated estimation becomes one frame of a t4perceval store (`FRAME` = its
    position in the evaluated sequence). The ground truth frame nearest to its header stamp is
    re-logged at the same position, the frame pipeline (filters and plane distance pass/fail
    matching) runs on a store holding that frame alone, and the result is kept as a
    `PerceptionFrameRecord`. The scene metrics are computed once at the end.
    """

    def __init__(
        self,
        perception_evaluation_config: dict,
        critical_object_filter_config: dict,
        perception_pass_fail_config: dict,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topic: str,
        evaluation_task: str,
        frame_id_str: str,
        ignore_frames: IgnoreFrames,
        *,
        compute_frame_metrics: bool = True,
    ) -> None:
        super().__init__(result_archive_path, evaluation_topic)
        # instance variables
        self.__skip_counter = 0
        self.__evaluated_frame_position = 0  # 1-based position of the evaluated frames
        self.__skip_reasons: Counter[str] = Counter()
        self.__frame_id_str = frame_id_str
        self.__evaluation_topic = evaluation_topic
        self.__ignore_frames = ignore_frames
        self.__logger = self._logger
        self.__frame_results: list[PerceptionFrameRecord] = []
        self.__scored_frame_results: list[PerceptionFrameRecord] | None = None
        self.__analysis: PerceptionAnalysis | None = None
        self.__warned_header_frame_id = False

        if evaluation_task == "fp_validation":
            err_msg = (
                "fp_validation is not supported by the t4perceval based perception use case. "
                "Use the perception_fp use case instead."
            )
            raise NotImplementedError(err_msg)
        if not self.__check_evaluation_task_and_frame_id(evaluation_task):
            err_msg = (
                f"Invalid evaluation task: {evaluation_task} or frame id: {self.__frame_id_str}. "
            )
            raise ValueError(err_msg)

        self.__result_archive_w_topic_path = Path(result_archive_path).joinpath(
            evaluation_topic.lstrip("/").replace("/", ".")
        )

        self.__config: EvaluationConfig = from_scenario(
            perception_evaluation_config,
            critical_object_filter_config,
            perception_pass_fail_config,
            evaluation_task=evaluation_task,
            frame_id=frame_id_str,
            logger=self.__logger,
        )
        self.__instances = InstanceRegistry()
        self.__ground_truth = load_ground_truth(
            t4_dataset_path,
            config=self.__config,
            instances=self.__instances,
            logger=self.__logger,
        )
        self.__labels = self.__ground_truth.labels
        self.__compute_frame_metrics = compute_frame_metrics
        self.__frame_pipeline = build_frame_pipeline(
            self.__config,
            compute_frame_metrics=compute_frame_metrics,
            gt_has_num_points=self.__ground_truth.has_num_points,
            gt_has_instance_id=self.__ground_truth.has_instance_id,
        )

    # -- per frame --------------------------------------------------------------------

    @property
    def conversion_context(self) -> ConversionContext:
        """What the runner needs to convert a ROS message for this evaluator."""
        return ConversionContext(self.__config, self.__labels, self.__instances)

    def evaluate_frame(
        self,
        converted_data: ConvertedData,
    ) -> FrameResult:
        data = converted_data.data
        if not isinstance(data, EstimationFrame):
            self.__logger.warning(
                "Estimated objects is invalid for timestamp: %s (%s)",
                converted_data.header_timestamp,
                data,
            )
            return self.__skip_frame(PerceptionInvalidReason.INVALID_ESTIMATED_OBJECTS)

        ground_truth_frame = self.__ground_truth.nearest(converted_data.header_timestamp)
        if ground_truth_frame is None:
            self.__logger.warning(
                "Ground truth not found for timestamp %s", converted_data.header_timestamp
            )
            return self.__skip_frame(PerceptionInvalidReason.NO_GROUND_TRUTH)

        if (
            data.header_frame_id
            and data.header_frame_id != self.__config.frame_id
            and not self.__warned_header_frame_id
        ):
            self.__logger.warning(
                "The header frame_id '%s' differs from the evaluation frame '%s'. "
                "The objects are evaluated as expressed in '%s'.",
                data.header_frame_id,
                self.__config.frame_id,
                self.__config.frame_id,
            )
            self.__warned_header_frame_id = True

        frame_index = self.__evaluated_frame_position + 1
        frame_result = self.__evaluate(
            data, ground_truth_frame, frame_index, converted_data.header_timestamp
        )
        # NOTE: the frame result is retained (and so written into the archive) even when the
        # frame is ignored, but it is not used for the criteria or the metrics.
        self.__frame_results.append(frame_result)
        self.__evaluated_frame_position = frame_index
        if self.__ignore_frames.should_ignore(ground_truth_frame.frame, frame_index):
            self.__logger.info(
                "Frame %s (evaluated frame position %d) is ignored for evaluation. But the frame result is still added for logging.",
                ground_truth_frame.frame_name,
                frame_index,
            )
            return self.__skip_frame(PerceptionInvalidReason.IGNORED_FRAME)

        self.__logger.info(
            "Estimation header: %d, Ground truth header: %d (frame_name: %s), Difference [nano sec]: %d, "
            "Subscribe delay [nano sec]: %d",
            converted_data.header_timestamp,
            ground_truth_frame.timestamp_ns,
            ground_truth_frame.frame_name,
            converted_data.header_timestamp - ground_truth_frame.timestamp_ns,
            converted_data.subscribed_timestamp - converted_data.header_timestamp,
        )
        return FrameResult(is_valid=True, data=frame_result, skip_counter=self.__skip_counter)

    def __evaluate(
        self,
        data: EstimationFrame,
        ground_truth_frame: GroundTruthFrame,
        frame_index: int,
        timestamp_ns: int,
    ) -> PerceptionFrameRecord:
        store = Store()
        estimation_chunk = data.archetype.to_chunk(
            ESTIMATION_PATH,
            at=TimePoint.at(frame=frame_index, timestamp_ns=timestamp_ns),
            frame_id=self.__config.frame_id,
        )
        store.send_chunk(estimation_chunk)
        ground_truth_chunks = self.__ground_truth.frame_chunks(
            ground_truth_frame.frame, at=frame_index
        )
        for chunk in ground_truth_chunks:
            store.send_chunk(chunk)

        self.__frame_pipeline.run(
            SystemContext(store, FRAME, labels=self.__labels, instances=self.__instances),
            at=frame_index,
        )

        def chunk_of(path: str) -> Chunk:
            return store.range(
                path, timeline=FRAME, time_range=TimeRange.single(frame_index)
            ).to_chunk()

        kept = chunk_of(f"{ESTIMATION_PATH}/filter/critical").columns[MASK].indices()
        estimation = ObjectTable.from_chunks(
            chunk_of(ESTIMATION_KEPT_PATH),
            chunk_of(ESTIMATION_KEPT_BASE_LINK_PATH),
            uuids=tuple(data.uuids[int(row)] for row in kept),
            pose_covariance=data.pose_covariance[kept],
            twist_covariance=data.twist_covariance[kept],
        )
        ground_truth = ObjectTable.from_chunks(
            chunk_of(GROUND_TRUTH_KEPT_PATH),
            chunk_of(GROUND_TRUTH_KEPT_BASE_LINK_PATH),
            instances=self.__instances,
        )
        matching = chunk_of(PASS_FAIL_MATCHING_PATH)
        matches = MatchTable.from_chunk(matching) if matching.num_rows else MatchTable.empty()

        frame_metrics: dict[str, float] = {}
        if self.__compute_frame_metrics:
            for key in ("map", "maph"):
                values = read_metric(
                    store,
                    f"{FRAME_METRICS_ROOT}/metrics/{key}/center_distance",
                    self.__labels,
                    self.__config.target_labels,
                )
                if values is not None:
                    frame_metrics[key] = values["ALL"]

        return PerceptionFrameRecord(
            frame_index=frame_index,
            frame_name=ground_truth_frame.frame_name,
            unix_time_ns=timestamp_ns,
            ground_truth_unix_time_ns=ground_truth_frame.timestamp_ns,
            frame_id=self.__config.frame_id,
            labels=self.__labels,
            policy=self.__config.matching_label_policy,
            estimation=estimation,
            ground_truth=ground_truth,
            matches=matches,
            frame_metrics=frame_metrics,
            raw_chunks=(estimation_chunk, *ground_truth_chunks),
        )

    # -- scene ------------------------------------------------------------------------

    def get_evaluation_config(self) -> EvaluationConfig:
        return self.__config

    def get_archive_path(self) -> Path:
        return self.__result_archive_w_topic_path

    def save_frame_results(self, *, store: Store | None = None, pipeline: object = None) -> None:
        """
        Write the archive of the topic: the recording, the config and the frame index.

        Without `store`, every evaluated frame (ignored ones included) is written without the
        scene metrics. `get_evaluation_results()` passes the scored store, which holds them.
        """
        self.__logger.info("Saving frame results for topic: %s", self.__evaluation_topic)
        records = self.__frame_results if store is None else self.__scored_frame_results
        if store is None:
            store = build_scene_store(records)
        write_archive(
            self.__result_archive_w_topic_path,
            store=store,
            records=records,
            config=self.__config,
            labels=self.__labels,
            instances=self.__instances,
            topic=self.__evaluation_topic,
            pipeline=pipeline,
        )

    def get_evaluation_results(self, *, save_frame_results: bool) -> tuple[dict, dict]:
        self.__logger.info("Evaluating topic: %s", self.__evaluation_topic)
        scored = self.__ignore_tail_frames(self.__frame_results)
        scored = self.__remove_ignored_frames(scored)
        self.__scored_frame_results = scored

        num_critical_fail = sum(frame_result.num_fail for frame_result in scored)
        self.__logger.info("Number of fails for critical objects: %d", num_critical_fail)

        store = build_scene_store(scored)
        pipeline = run_scene_pipeline(store, self.__config, self.__labels, self.__instances)
        target_labels = self.__config.target_labels
        final_metrics = {
            "Score": final_score(store, scored, self.__config, self.__labels),
            "Error": error_statistics(scored, target_labels),
            "ConfusionMatrix": confusion_matrix_dict(store, self.__labels, target_labels),
        }
        self.__logger.info("final metrics result %s", final_metrics["Score"])
        self.__analysis = PerceptionAnalysis(
            scored, self.__config, self.__labels, self.__instances, store=store
        )
        if save_frame_results:
            self.save_frame_results(store=store, pipeline=pipeline)
        return final_metrics, self.__get_frame_coverage()

    def get_analyzer(self) -> PerceptionAnalysis:
        if self.__analysis is not None:
            return self.__analysis
        err_msg = "Analyzer is not available. Please call get_evaluation_results() first."
        raise RuntimeError(err_msg)

    # -- helpers ----------------------------------------------------------------------

    def __check_evaluation_task_and_frame_id(self, evaluation_task: str) -> bool:
        return (evaluation_task == "detection" and self.__frame_id_str == "base_link") or (
            evaluation_task in ("tracking", "prediction") and self.__frame_id_str == "map"
        )

    def __skip_frame(self, invalid_reason: PerceptionInvalidReason) -> FrameResult:
        """Count the skipped frame and its reason, then build the invalid FrameResult."""
        self.__skip_counter += 1
        self.__skip_reasons[invalid_reason.name] += 1
        return FrameResult(
            is_valid=False,
            invalid_reason=invalid_reason,
            skip_counter=self.__skip_counter,
        )

    def __remove_ignored_frames(
        self, frame_results: list[PerceptionFrameRecord]
    ) -> list[PerceptionFrameRecord]:
        """
        Drop the frames ignored by frame_name (`N`/`A-B`) or by position (`first:N`).

        NOTE: the ignore decision in evaluate_frame() happens after the frame was evaluated, so
        that the frame result is still retained for inspection. This method removes them from
        the list used for metrics/analysis/coverage.
        """
        return [
            frame_result
            for frame_result in frame_results
            if not self.__ignore_frames.should_ignore(
                int(frame_result.frame_name), frame_result.frame_index
            )
        ]

    def __ignore_tail_frames(
        self, frame_results: list[PerceptionFrameRecord]
    ) -> list[PerceptionFrameRecord]:
        """
        Drop the last N evaluated frames from frame_results for the `last:N` setting.

        `last:N` cannot be decided while streaming, so it is applied here, before the frame results
        are saved and before the metrics, the analysis and the coverage are computed.
        """
        last_ignore_frames = self.__ignore_frames.last
        if last_ignore_frames <= 0:
            return frame_results
        last_ignore_frames = min(last_ignore_frames, len(frame_results))
        ignored_frame_names = [
            frame_result.frame_name
            for frame_result in frame_results[len(frame_results) - last_ignore_frames :]
        ]
        self.__skip_counter += last_ignore_frames
        self.__skip_reasons[PerceptionInvalidReason.IGNORED_FRAME.name] += last_ignore_frames
        self.__logger.info(
            "Last %d evaluated frames are ignored for evaluation (frame_name: %s).",
            last_ignore_frames,
            ", ".join(ignored_frame_names),
        )
        return frame_results[: len(frame_results) - last_ignore_frames]

    def __get_frame_coverage(self) -> dict:
        """
        Get how much of the ground truth of the dataset was actually evaluated.

        Returns:
            dict: `GtFrames` is the number of ground truth frames of the dataset,
                `GtFramesEvaluated` is the number of distinct ground truth frames bound to at least
                one valid evaluated estimate, `Coverage` is their ratio and `SkipReasons` is the
                histogram of the reasons why a frame was not evaluated.

        NOTE: `last:N` is only reflected here once `get_evaluation_results()` has already run,
        same precondition as `get_analyzer()`. This method does not apply it itself.

        """
        num_gt_frames = self.__ground_truth.num_frames
        scored = (
            self.__scored_frame_results
            if self.__scored_frame_results is not None
            else self.__remove_ignored_frames(self.__frame_results)
        )
        evaluated_frame_names = {frame_result.frame_name for frame_result in scored}
        num_evaluated = len(evaluated_frame_names)
        coverage = round(num_evaluated / num_gt_frames, 4) if num_gt_frames > 0 else 0.0
        return {
            "GtFrames": num_gt_frames,
            "GtFramesEvaluated": num_evaluated,
            "Coverage": coverage,
            "SkipReasons": dict(sorted(self.__skip_reasons.items())),
        }
