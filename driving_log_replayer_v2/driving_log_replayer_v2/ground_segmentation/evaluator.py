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

import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from driving_log_replayer_v2.ground_segmentation.models import Conditions
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.evaluator import InvalidReason
from driving_log_replayer_v2_msgs.msg import GroundSegmentationEvalResult


class GroundSegmentationInvalidReason(InvalidReason):
    NO_GROUND_TRUTH = "No Ground Truth"


class GroundSegmentationEvaluator(Evaluator):
    CLOUD_DIM = 5
    TS_DIFF_THRESH = 75000

    def __init__(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topic: str,
        conditions: Conditions,
    ) -> None:
        # additional instance variables
        self._skip_counter = 0
        self._conditions: Conditions = conditions

        super().__init__(result_archive_path, evaluation_topic)

        self._ground_label = self._conditions.ground_label
        self._obstacle_label = self._conditions.obstacle_label

        # load point cloud data
        sample_data_path = Path(t4_dataset_path, "annotation", "sample_data.json")
        sample_data = json.load(sample_data_path.open())
        sample_data = list(filter(lambda d: d["filename"].split(".")[-2] == "pcd", sample_data))

        # load gt annotation data
        lidar_seg_json_path = Path(t4_dataset_path, "annotation", "lidarseg.json")
        lidar_seg_data = json.load(lidar_seg_json_path.open())
        token_to_seg_data = {}
        for annotation_data in lidar_seg_data:
            token_to_seg_data[annotation_data["sample_data_token"]] = annotation_data

        self._ground_truth: dict[int, dict[str, np.ndarray]] = {}
        for i in range(len(sample_data)):
            raw_points_file_path = Path(
                t4_dataset_path,
                sample_data[i]["filename"],
            ).as_posix()
            raw_points = np.fromfile(raw_points_file_path, dtype=np.float32)
            token = sample_data[i]["token"]

            if token not in token_to_seg_data:
                continue
            annotation_file_path = Path(
                t4_dataset_path, token_to_seg_data[token]["filename"]
            ).as_posix()
            labels = np.fromfile(annotation_file_path, dtype=np.uint8)

            points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))

            self._ground_truth[int(sample_data[i]["timestamp"])] = {
                "points": points,
                "labels": labels,
            }

    def evaluate_frame(
        self,
        header_timestamp: int,
        subscribed_timestamp: int,
        data: np.ndarray,
    ) -> FrameResult:
        gt_frame_ts = self.__get_gt_frame_ts(header_timestamp)

        if gt_frame_ts < 0:
            self._skip_counter += 1
            self._logger.warning("No ground truth for timestamp %d microsec", header_timestamp)
            return FrameResult(
                is_valid=False,
                invalid_reason=GroundSegmentationInvalidReason.NO_GROUND_TRUTH,
                skip_counter=self._skip_counter,
            )

        # get ground truth pointcloud in this frame
        # construct kd-tree from gt cloud
        gt_frame_cloud: np.ndarray = self._ground_truth[gt_frame_ts]["points"]
        gt_frame_label: np.ndarray = self._ground_truth[gt_frame_ts]["labels"]
        kdtree = cKDTree(gt_frame_cloud[:, 0:3])

        if gt_frame_cloud.shape[0] != gt_frame_label.shape[0]:
            err_msg = (
                f"ground truth cloud and label size mismatch: "
                f"cloud points={gt_frame_cloud.shape[0]}, labels={gt_frame_label.shape[0]}"
            )
            raise ValueError(err_msg)

        # count TP+FN, TN+FP
        tp_fn = 0
        for ground_label in self._ground_label:
            tp_fn += np.count_nonzero(gt_frame_label == ground_label)

        fp_tn = 0
        for obstacle_label in self._obstacle_label:
            fp_tn += np.count_nonzero(gt_frame_label == obstacle_label)

        tn: int = 0
        fn: int = 0
        for p in data:
            _, idx = kdtree.query(p, k=1)
            if gt_frame_label[idx] in self._ground_label:
                fn += 1
            elif gt_frame_label[idx] in self._obstacle_label:
                tn += 1
        tp = tp_fn - fn
        fp = fp_tn - tn

        metrics_list = self.__compute_metrics(tp, fp, tn, fn)

        frame_result = GroundSegmentationEvalResult()
        frame_result.tp = tp
        frame_result.fp = fp
        frame_result.tn = tn
        frame_result.fn = fn
        frame_result.accuracy = metrics_list[0]
        frame_result.precision = metrics_list[1]
        frame_result.recall = metrics_list[2]
        frame_result.specificity = metrics_list[3]
        frame_result.f1_score = metrics_list[4]

        # TODO: add topic delay
        self._logger.info(
            "Estimation header: %d, Ground truth header: %d (frame_name: %s), Difference: %d, "
            "Subscribe delay [micro sec]: %d",
            header_timestamp,
            gt_frame_ts,
            "None",
            header_timestamp - gt_frame_ts,
            subscribed_timestamp - header_timestamp,
        )

        return FrameResult(is_valid=True, data=frame_result, skip_counter=self._skip_counter)

    def __get_gt_frame_ts(self, unix_time: int) -> int:
        ts_itr = iter(self._ground_truth.keys())
        ret_ts: int = int(next(ts_itr))
        min_diff: int = abs(unix_time - ret_ts)

        for _ in range(1, len(self._ground_truth)):
            sample_ts = next(ts_itr)
            diff_time = abs(unix_time - sample_ts)
            if diff_time < min_diff:
                min_diff = diff_time
                ret_ts = sample_ts

        if min_diff > self.TS_DIFF_THRESH:
            return -1

        return ret_ts

    def __compute_metrics(self, tp: int, fp: int, tn: int, fn: int) -> list[float]:
        eps = 1e-10
        accuracy = float(tp + tn) / float(tp + fp + tn + fn + eps)
        precision = float(tp) / float(tp + fp + eps)
        recall = float(tp) / float(tp + fn + eps)
        specificity = float(tn) / float(tn + fp + eps)
        f1_score = 2 * (precision * recall) / (precision + recall + eps)
        return [accuracy, precision, recall, specificity, f1_score]
