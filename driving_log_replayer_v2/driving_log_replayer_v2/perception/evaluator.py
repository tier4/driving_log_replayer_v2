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
import copy
import logging
from os.path import expandvars
from pathlib import Path
import pickle
from typing import TYPE_CHECKING

from perception_eval.common import DynamicObject
from perception_eval.common.status import get_scene_rates
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.evaluation.result.perception_frame_result import get_object_status
from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer3D
from perception_eval.util.logger_config import configure_logger

from driving_log_replayer_v2.perception import advanced_metrics
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.evaluator import InvalidReason

if TYPE_CHECKING:
    from perception_eval.evaluation.metrics import MetricsScore

    from driving_log_replayer_v2.perception.runner import PerceptionEvalData
    from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
    from driving_log_replayer_v2.post_process.runner import ConvertedData


class PerceptionInvalidReason(InvalidReason):
    INVALID_ESTIMATED_OBJECTS = "Invalid Estimated Objects"
    NO_GROUND_TRUTH = "No Ground Truth"
    IGNORED_FRAME = "Ignored Frame"


class PerceptionEvaluator(Evaluator):
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
    ) -> None:
        # NOTE: this class uses the perception_eval package, so not use parent logger, which means not call super().__init__()
        # instance variables
        self.__skip_counter = 0
        self.__evaluated_frame_position = 0  # 1-based position of the evaluated frames
        self.__skip_reasons: Counter[str] = Counter()
        self.__frame_id_str = frame_id_str
        self.__critical_object_filter_config: CriticalObjectFilterConfig
        self.__frame_pass_fail_config: PerceptionPassFailConfig
        self.__evaluator: PerceptionEvaluationManager
        self.__result_archive_w_topic_path: Path
        self.__analyzer: PerceptionAnalyzer3D
        self.__logger: logging.Logger
        self.__evaluation_topic = evaluation_topic
        self.__ignore_frames = ignore_frames
        self.__ignored_frames_removed = False  # set once get_evaluation_results() dropped them
        self.__advanced_outputs_written = False
        self.__num_frame_results = 0  # before the ignored frames are removed
        self.__t4_dataset_path = t4_dataset_path
        self.__evaluation_task = evaluation_task

        perception_evaluation_config["evaluation_config_dict"]["label_prefix"] = "autoware"

        if not self.__check_evaluation_task_and_frame_id(evaluation_task):
            err_msg = (
                f"Invalid evaluation task: {evaluation_task} or frame id: {self.__frame_id_str}. "
            )
            raise ValueError(err_msg)
        perception_evaluation_config["evaluation_config_dict"]["evaluation_task"] = evaluation_task
        # raw dict, kept for the digest of the advanced detection metrics report
        self.__evaluation_config_dict: dict = copy.deepcopy(
            perception_evaluation_config["evaluation_config_dict"]
        )

        self.__result_archive_w_topic_path = Path(result_archive_path)
        self.__result_archive_w_topic_path.mkdir(exist_ok=True)
        dir_name = evaluation_topic.lstrip("/").replace("/", ".")
        self.__result_archive_w_topic_path = self.__result_archive_w_topic_path.joinpath(dir_name)
        perception_eval_log_path = self.__result_archive_w_topic_path.joinpath(
            "perception_eval_log"
        ).as_posix()

        # parameters underlying evaluation
        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=[t4_dataset_path],
            frame_id=self.__frame_id_str,
            result_root_directory=Path(
                perception_eval_log_path,
                "result",
                "{TIME}",
            ).as_posix(),
            evaluation_config_dict=perception_evaluation_config["evaluation_config_dict"],
            load_raw_data=False,
        )

        # TODO: add annotation load log
        self.__logger = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
            logger_name=dir_name,
        )

        # parameters for which to focus on
        self.__critical_object_filter_config = CriticalObjectFilterConfig(
            evaluator_config=evaluation_config,
            target_labels=critical_object_filter_config["target_labels"],
            ignore_attributes=critical_object_filter_config.get("ignore_attributes"),
            max_x_position_list=critical_object_filter_config.get("max_x_position_list"),
            max_y_position_list=critical_object_filter_config.get("max_y_position_list"),
            max_distance_list=critical_object_filter_config.get("max_distance_list"),
            min_distance_list=critical_object_filter_config.get("min_distance_list"),
            min_point_numbers=critical_object_filter_config.get("min_point_numbers"),
            confidence_threshold_list=critical_object_filter_config.get(
                "confidence_threshold_list"
            ),
            target_uuids=critical_object_filter_config.get("target_uuids"),
        )

        # parameters for deciding pass/fail
        self.__frame_pass_fail_config = PerceptionPassFailConfig(
            evaluator_config=evaluation_config,
            target_labels=perception_pass_fail_config["target_labels"],
            matching_threshold_list=perception_pass_fail_config.get("matching_threshold_list"),
            confidence_threshold_list=perception_pass_fail_config.get("confidence_threshold_list"),
        )

        self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)

    def evaluate_frame(
        self,
        converted_data: ConvertedData,
    ) -> FrameResult:
        # skip evaluation if data conversion fails
        data: PerceptionEvalData = converted_data.data
        if not (
            isinstance(data.estimated_objects, list)
            and all(isinstance(obj, DynamicObject) for obj in data.estimated_objects)
        ):
            self.__logger.warning(
                "Estimated objects is invalid for timestamp: %s", converted_data.header_timestamp
            )
            return self.__skip_frame(PerceptionInvalidReason.INVALID_ESTIMATED_OBJECTS)

        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(
            converted_data.header_timestamp,
            interpolate_ground_truth=data.interpolation,
        )

        if ground_truth_now_frame is None:
            self.__logger.warning(
                "Ground truth not found for timestamp %s", converted_data.header_timestamp
            )
            return self.__skip_frame(PerceptionInvalidReason.NO_GROUND_TRUTH)

        frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
            unix_time=converted_data.header_timestamp,
            ground_truth_now_frame=ground_truth_now_frame,
            estimated_objects=data.estimated_objects,
            critical_object_filter_config=self.__critical_object_filter_config,
            frame_pass_fail_config=self.__frame_pass_fail_config,
        )

        # NOTE: frame result is retained within `self.__evaluator` (i.e., it is also saved in the pkl).
        # However, it is not included in the results of criteria calculations or in metrics such as mAP calculated by `log`.
        self.__evaluated_frame_position += 1
        if self.__ignore_frames.should_ignore(
            int(ground_truth_now_frame.frame_name), self.__evaluated_frame_position
        ):
            self.__logger.info(
                "Frame %s (evaluated frame position %d) is ignored for evaluation. But the frame result is still added to the evaluator for logging.",
                ground_truth_now_frame.frame_name,
                self.__evaluated_frame_position,
            )
            return self.__skip_frame(PerceptionInvalidReason.IGNORED_FRAME)

        # TODO: add topic delay
        self.__logger.info(
            "Estimation header: %d, Ground truth header: %d (frame_name: %s), Difference: %d, "
            "Subscribe delay [micro sec]: %d",
            converted_data.header_timestamp,
            ground_truth_now_frame.unix_time,
            ground_truth_now_frame.frame_name,
            converted_data.header_timestamp - ground_truth_now_frame.unix_time,
            converted_data.subscribed_timestamp - converted_data.header_timestamp,
        )
        # TODO: decide whether to add skip counter or not

        return FrameResult(is_valid=True, data=frame_result, skip_counter=self.__skip_counter)

    def get_evaluation_config(self) -> PerceptionEvaluationConfig:
        return self.__evaluator.evaluator_config

    def get_archive_path(self) -> Path:
        return self.__result_archive_w_topic_path

    def save_frame_results(self) -> None:
        self.__logger.info("Saving frame results for topic: %s", self.__evaluation_topic)
        with Path(expandvars(self.__result_archive_w_topic_path.joinpath("scene_result.pkl"))).open(
            "wb"
        ) as pkl_file:
            pickle.dump(self.__evaluator.frame_results, pkl_file)
        with Path(
            expandvars(self.__result_archive_w_topic_path.joinpath("evaluation_config.pkl"))
        ).open("wb") as pkl_file:
            pickle.dump(self.__evaluator.evaluator_config, pkl_file)

    def get_evaluation_results(self, *, save_frame_results: bool) -> tuple[dict, dict]:
        self.__logger.info("Evaluating topic: %s", self.__evaluation_topic)
        if save_frame_results:
            self.save_frame_results()
        if self.__evaluator.evaluator_config.evaluation_task == "fp_validation":
            final_metrics = self.__get_fp_results()
        else:
            self.__num_frame_results = len(self.__evaluator.frame_results)
            self.__evaluator.frame_results = self.__ignore_tail_frames(
                self.__evaluator.frame_results
            )
            self.__evaluator.frame_results = self.__remove_ignored_frames(
                self.__evaluator.frame_results
            )
            self.__ignored_frames_removed = True
            scene_result = self.__get_scene_results()  # TODO: use the mAP part of this result
            # the advanced detection metrics were computed by get_scene_result(), reuse them
            self.__write_advanced_detection_outputs(
                self.__evaluator.frame_results,
                num_frame_results=self.__num_frame_results,
                scene_result=scene_result,
            )
            self.__analyzer = PerceptionAnalyzer3D(self.__evaluator.evaluator_config)
            self.__analyzer.add(self.__evaluator.frame_results)
            result = self.__analyzer.analyze()
            score_dict = result.score.to_dict() if result.score is not None else {}
            error_dict = (
                (result.error.groupby(level=0).apply(lambda df: df.xs(df.name).to_dict()).to_dict())
                if result.error is not None
                else {}
            )
            conf_mat_dict = (
                result.confusion_matrix.to_dict() if result.confusion_matrix is not None else {}
            )
            final_metrics = {
                "Score": score_dict,
                "Error": error_dict,
                "ConfusionMatrix": conf_mat_dict,
            }
        return final_metrics, self.__get_frame_coverage()

    def get_analyzer(self) -> PerceptionAnalyzer3D:
        if self.__evaluator.evaluator_config.evaluation_task == "fp_validation":
            err_msg = "Analyzer is not available for fp_validation."
            raise RuntimeError(err_msg)
        if hasattr(self, f"_{self.__class__.__name__}__analyzer"):
            return self.__analyzer
        err_msg = "Analyzer is not available. Please call get_evaluation_results() first."
        raise RuntimeError(err_msg)

    def write_advanced_detection_outputs(self) -> None:
        """
        Write the driving-aware detection metric outputs next to scene_result.pkl.

        Writes `advanced_detection_metrics.json` and `advanced_detection_samples.npz` into the
        result archive of the topic when `advanced_detection_metrics` is configured in
        `evaluation_config_dict`. Nothing is written for fp_validation or without the section.

        The frames ignored by `ignore_frames` are left out without touching `frame_results` (the
        pkl keeps them), so this can be called instead of `get_evaluation_results()`.
        """
        if self.__advanced_outputs_written:
            # get_evaluation_results() already wrote them from the same frames
            return
        frame_results = self.__evaluator.frame_results
        if self.__ignored_frames_removed:
            # get_evaluation_results() already dropped the ignored frames
            self.__write_advanced_detection_outputs(
                frame_results, num_frame_results=self.__num_frame_results
            )
            return
        kept_frame_results, ignored_frame_names = self.__split_ignored_frames(frame_results)
        self.__write_advanced_detection_outputs(
            kept_frame_results,
            num_frame_results=len(frame_results),
            ignored_frame_names=ignored_frame_names,
        )

    def __split_ignored_frames(
        self, frame_results: list[PerceptionFrameResult]
    ) -> tuple[list[PerceptionFrameResult], list[str]]:
        """
        Select the frames like `__ignore_tail_frames` + `__remove_ignored_frames`, without side effects.

        Returns:
            tuple[list[PerceptionFrameResult], list[str]]: The frames to evaluate (a new list,
                `frame_results` is not modified) and the `frame_name` of the ignored frames.

        """
        num_tail = min(max(self.__ignore_frames.last, 0), len(frame_results))
        head = frame_results[: len(frame_results) - num_tail]
        tail = frame_results[len(frame_results) - num_tail :]
        ignored_frame_names = [frame_result.frame_name for frame_result in tail]
        kept = []
        for frame_result in head:
            if int(frame_result.frame_name) in self.__ignore_frames:
                ignored_frame_names.append(frame_result.frame_name)
            else:
                kept.append(frame_result)
        return kept, ignored_frame_names

    def __write_advanced_detection_outputs(
        self,
        frame_results: list[PerceptionFrameResult],
        *,
        num_frame_results: int,
        ignored_frame_names: list[str] | None = None,
        scene_result: MetricsScore | None = None,
    ) -> None:
        """
        Compute (or reuse from `scene_result`) and write the advanced detection metric outputs.

        Args:
            frame_results (list[PerceptionFrameResult]): Frames to evaluate. Not modified.
            num_frame_results (int): Number of frame results before the ignored frames were removed.
            ignored_frame_names (list[str] | None): `frame_name` of the frames still in
                `frame_results` which must be left out (None: already removed).
            scene_result (MetricsScore | None): Scene result whose `detection_prepared` /
                `detection_metric_report` are reused when present, to run the suite only once.

        """
        if self.__evaluation_task == "fp_validation":
            return
        metrics_config = self.__evaluator.evaluator_config.metrics_config
        advanced_config = advanced_metrics.get_advanced_config(metrics_config)
        if advanced_config is None:
            return

        ignored_frame_names = list(ignored_frame_names or [])
        if ignored_frame_names:
            evaluated_frame_results = advanced_metrics.select_frame_results(
                frame_results, ignored_frame_names
            )
        else:
            evaluated_frame_results = list(frame_results)
        meta = advanced_metrics.ReportMeta(
            topic=self.__evaluation_topic,
            evaluation_task=self.__evaluation_task,
            frame_id=self.__frame_id_str,
            t4_dataset_path=self.__t4_dataset_path,
            evaluation_config_dict=self.__evaluation_config_dict,
            advanced_config=advanced_config,
            num_frame_results=num_frame_results,
            num_ignored_frames=num_frame_results - len(evaluated_frame_results),
        )
        out_dir = self.__result_archive_w_topic_path
        self.__logger.info(
            "Writing the advanced detection metric outputs for topic: %s", self.__evaluation_topic
        )
        try:
            prepared = getattr(scene_result, "detection_prepared", None)
            report = getattr(scene_result, "detection_metric_report", None)
            if prepared is None or report is None:
                computed = advanced_metrics.compute(
                    evaluated_frame_results, metrics_config, ignored_frame_names=()
                )
                prepared, report = computed if computed is not None else (None, None)
            advanced_metrics.write_outputs(out_dir, prepared=prepared, report=report, meta=meta)
            self.__advanced_outputs_written = True
        except Exception as err:
            self.__logger.exception(
                "Failed to compute the advanced detection metrics for topic: %s",
                self.__evaluation_topic,
            )
            advanced_metrics.write_outputs(
                out_dir,
                prepared=None,
                report=None,
                meta=meta,
                error=f"{type(err).__name__}: {err}",
            )
            self.__advanced_outputs_written = True

    def __check_evaluation_task_and_frame_id(self, evaluation_task: str) -> bool:
        # for fp_validation, it can be either base_link or map because it can handle DetectedObjects, TrackedObjects and PredictedObjects.
        return (
            (evaluation_task == "detection" and self.__frame_id_str == "base_link")
            or (evaluation_task in ("tracking", "prediction") and self.__frame_id_str == "map")
            or (evaluation_task == "fp_validation" and self.__frame_id_str in ("base_link", "map"))
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
        self, frame_results: list[PerceptionFrameResult]
    ) -> list[PerceptionFrameResult]:
        """
        Drop the frames ignored by frame_name (`N`/`A-B`) or by position (`first:N`).

        NOTE: the ignore decision in evaluate_frame() happens after add_frame_result(), so that the
        frame result is still retained in `frame_results` (and thus in the pkl) for inspection. This
        method removes them from the list used for metrics/analysis/coverage.
        """
        return [
            frame_result
            for frame_result in frame_results
            if int(frame_result.frame_name) not in self.__ignore_frames
        ]

    def __ignore_tail_frames(
        self, frame_results: list[PerceptionFrameResult]
    ) -> list[PerceptionFrameResult]:
        """
        Drop the last N evaluated frames from frame_results for the `last:N` setting.

        `last:N` cannot be decided while streaming, so it is applied here, before the frame results
        are saved and before the metrics, the analyzer and the coverage are computed.
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
            dict: `GtFrames` is the number of ground truth frames of the dataset window,
                `GtFramesEvaluated` is the number of distinct ground truth frames bound to at least
                one valid evaluated estimate, `Coverage` is their ratio and `SkipReasons` is the
                histogram of the reasons why a frame was not evaluated.

        NOTE: `last:N` is only reflected here once `get_evaluation_results()` has already run,
        same precondition as `get_analyzer()`. This method does not apply it itself.

        """
        num_gt_frames = len(self.__evaluator.ground_truth_frames)
        evaluated_frame_names = {
            frame_result.frame_name for frame_result in self.__evaluator.frame_results
        }
        num_evaluated = len(evaluated_frame_names)
        coverage = round(num_evaluated / num_gt_frames, 4) if num_gt_frames > 0 else 0.0
        return {
            "GtFrames": num_gt_frames,
            "GtFramesEvaluated": num_evaluated,
            "Coverage": coverage,
            "SkipReasons": dict(sorted(self.__skip_reasons.items())),
        }

    def __get_scene_results(self) -> MetricsScore:
        num_critical_fail: int = sum(
            [
                frame_result.pass_fail_result.get_num_fail()
                for frame_result in self.__evaluator.frame_results
            ],
        )
        self.__logger.info("Number of fails for critical objects: %d", num_critical_fail)

        # scene metrics score
        final_metric_score = self.__evaluator.get_scene_result()
        self.__logger.info("final metrics result %s", final_metric_score)
        return final_metric_score

    def __get_fp_results(self) -> dict:
        status_list = get_object_status(self.__evaluator.frame_results)
        gt_status = {}
        for status_info in status_list:
            tp_rate, fp_rate, tn_rate, fn_rate = status_info.get_status_rates()
            # display
            self.__logger.info(
                "uuid: %s, TP: %0.3f, FP: %0.3f, TN: %0.3f, FN: %0.3f\n Total: %s, TP: %s, FP: %s, TN: %s, FN: %s",
                status_info.uuid,
                tp_rate.rate,
                fp_rate.rate,
                tn_rate.rate,
                fn_rate.rate,
                status_info.total_frame_nums,
                status_info.tp_frame_nums,
                status_info.fp_frame_nums,
                status_info.tn_frame_nums,
                status_info.fn_frame_nums,
            )
            gt_status[status_info.uuid] = {
                "rate": {
                    "TP": tp_rate.rate,
                    "FP": fp_rate.rate,
                    "TN": tn_rate.rate,
                    "FN": fn_rate.rate,
                },
                "frame_nums": {
                    "total": status_info.total_frame_nums,
                    "TP": status_info.tp_frame_nums,
                    "FP": status_info.fp_frame_nums,
                    "TN": status_info.tn_frame_nums,
                    "FN": status_info.fn_frame_nums,
                },
            }

        scene_tp_rate, scene_fp_rate, scene_tn_rate, scene_fn_rate = get_scene_rates(status_list)
        self.__logger.info(
            "[scene] TP: %f, FP: %f, TN: %f, FN: %f",
            scene_tp_rate,
            scene_fp_rate,
            scene_tn_rate,
            scene_fn_rate,
        )
        return {
            "GroundTruthStatus": gt_status,
            "Scene": {
                "TP": scene_tp_rate,
                "FP": scene_fp_rate,
                "TN": scene_tn_rate,
                "FN": scene_fn_rate,
            },
        }
