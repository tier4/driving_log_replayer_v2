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

import logging
from os.path import expandvars
from pathlib import Path
import pickle
from typing import TYPE_CHECKING

from perception_eval.common import DynamicObject
from perception_eval.common.status import get_scene_rates
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation import get_object_status
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer3D
from perception_eval.util.logger_config import configure_logger

if TYPE_CHECKING:
    from perception_eval.evaluation.metrics import MetricsScore


class PerceptionEvaluator:
    def __init__(
        self,
        perception_evaluation_config: dict,
        critical_object_filter_config: dict,
        perception_pass_fail_config: dict,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topic: str,
        evaluation_task: str,
    ) -> None:
        self.__skip_counter = 0
        self.__frame_id_str: str
        self.__critical_object_filter_config: CriticalObjectFilterConfig
        self.__frame_pass_fail_config: PerceptionPassFailConfig
        self.__evaluator: PerceptionEvaluationManager
        self.__result_archive_w_topic_path: Path
        self.__analyzer: PerceptionAnalyzer3D
        self.__logger: logging.Logger

        perception_evaluation_config["evaluation_config_dict"]["label_prefix"] = "autoware"

        if not self.__check_evaluation_task(evaluation_task):
            err_msg = f"Invalid evaluation task: {evaluation_task}. "
            raise ValueError(err_msg)
        perception_evaluation_config["evaluation_config_dict"]["evaluation_task"] = evaluation_task

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

    def add_frame(
        self,
        estimated_objects: list[DynamicObject] | str,
        header_unix_time: int,
        subscribed_unix_time: int,
        *,
        interpolation: bool,
    ) -> tuple[PerceptionFrameResult | str, int]:
        # skip add_frame and return string (error messages) if estimated_objects conversion fails
        if not (
            isinstance(estimated_objects, list)
            and all(isinstance(obj, DynamicObject) for obj in estimated_objects)
        ):
            self.__skip_counter += 1
            self.__logger.warning(
                "Estimated objects is invalid for timestamp: %s", header_unix_time
            )
            return "Invalid Estimated Objects", self.__skip_counter

        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(
            header_unix_time,
            interpolate_ground_truth=interpolation,
        )

        if ground_truth_now_frame is None:
            self.__skip_counter += 1
            self.__logger.warning("Ground truth not found for timestamp %s", header_unix_time)
            return "No Ground Truth", self.__skip_counter

        frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
            unix_time=header_unix_time,
            ground_truth_now_frame=ground_truth_now_frame,
            estimated_objects=estimated_objects,
            critical_object_filter_config=self.__critical_object_filter_config,
            frame_pass_fail_config=self.__frame_pass_fail_config,
        )

        # TODO: add topic delay
        self.__logger.info(
            "difference between header and subscribe [micro sec]: %d",
            subscribed_unix_time - header_unix_time,
        )
        # TODO: decide whether to add skip counter or not

        return frame_result, self.__skip_counter

    def get_evaluation_config(self) -> PerceptionEvaluationConfig:
        return self.__evaluator.evaluator_config

    def get_archive_path(self) -> Path:
        return self.__result_archive_w_topic_path

    def get_evaluation_results(self, *, save_frame_results: bool) -> dict:
        if save_frame_results:
            with Path(
                expandvars(self.__result_archive_w_topic_path.joinpath("scene_result.pkl"))
            ).open("wb") as pkl_file:
                pickle.dump(self.__evaluator.frame_results, pkl_file)
            with Path(
                expandvars(self.__result_archive_w_topic_path.joinpath("evaluation_config.pkl"))
            ).open("wb") as pkl_file:
                pickle.dump(self.__evaluator.evaluator_config, pkl_file)
        if self.__evaluator.evaluator_config.evaluation_task == "fp_validation":
            final_metrics = self.__get_fp_results()
        else:
            _ = self.__get_scene_results()  # TODO: use this result
            score_dict = {}
            error_dict = {}
            conf_mat_dict = {}
            self.__analyzer = PerceptionAnalyzer3D(self.__evaluator.evaluator_config)
            self.__analyzer.add(self.__evaluator.frame_results)
            result = self.__analyzer.analyze()
            if result.score is not None:
                score_dict = result.score.to_dict()
            if result.error is not None:
                error_dict = (
                    result.error.groupby(level=0)
                    .apply(lambda df: df.xs(df.name).to_dict())
                    .to_dict()
                )
            if result.confusion_matrix is not None:
                conf_mat_dict = result.confusion_matrix.to_dict()
            final_metrics = {
                "Score": score_dict,
                "Error": error_dict,
                "ConfusionMatrix": conf_mat_dict,
            }
        return final_metrics

    def get_analyzer(self) -> PerceptionAnalyzer3D:
        if hasattr(self, f"_{self.__class__.__name__}__analyzer"):
            return self.__analyzer
        err_msg = "Analyzer is not available. Please call get_evaluation_results() first or evaluation_task is fp_validation."
        raise RuntimeError(err_msg)

    def __check_evaluation_task(self, evaluation_task: str) -> bool:
        if evaluation_task in ["detection", "fp_validation"]:
            self.__frame_id_str = "base_link"
            return True
        if evaluation_task == "tracking":
            self.__frame_id_str = "map"
            return True
        if evaluation_task == "prediction":
            self.__frame_id_str = "map"
            return False
        return False

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
