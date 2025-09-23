#!/usr/bin/env python3

# Copyright (c) 2024 TIER IV.inc
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
from rclpy.qos import qos_profile_sensor_data
import ros2_numpy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.ground_segmentation import Condition
from driving_log_replayer_v2.ground_segmentation import GroundSegmentationResult
from driving_log_replayer_v2.ground_segmentation import GroundSegmentationScenario
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2_msgs.msg import GroundSegmentationEvalResult


class GroundSegmentationEvaluator(DLREvaluatorV2):
    CLOUD_DIM = 5
    TS_DIFF_THRESH = 75000

    def __init__(self, name: str) -> None:
        super().__init__(
            name,
            GroundSegmentationScenario,
            GroundSegmentationResult,
            "/driving_log_replayer/ground_segmentation/results",
        )

        eval_condition: Condition = self._scenario.Evaluation.Conditions
        self.ground_label = eval_condition.ground_label
        self.obstacle_label = eval_condition.obstacle_label
        self.eval_target_topic = (
            self.declare_parameter(
                "evaluation_target_topic", "/perception/obstacle_segmentation/pointcloud"
            )
            .get_parameter_value()
            .string_value
        )

        # load point cloud data
        sample_data_path = Path(self._t4_dataset_paths[0], "annotation", "sample_data.json")
        sample_data = json.load(sample_data_path.open())
        sample_data = list(filter(lambda d: d["filename"].split(".")[-2] == "pcd", sample_data))

        # load gt annotation data
        lidar_seg_json_path = Path(self._t4_dataset_paths[0], "annotation", "lidarseg.json")
        lidar_seg_data = json.load(lidar_seg_json_path.open())
        token_to_seg_data = {}
        for annotation_data in lidar_seg_data:
            token_to_seg_data[annotation_data["sample_data_token"]] = annotation_data

        self.ground_truth: dict[int, dict[str, np.ndarray]] = {}
        for i in range(len(sample_data)):
            raw_points_file_path = Path(
                self._t4_dataset_paths[0],
                sample_data[i]["filename"],
            ).as_posix()
            raw_points = np.fromfile(raw_points_file_path, dtype=np.float32)
            token = sample_data[i]["token"]

            if token not in token_to_seg_data:
                continue
            annotation_file_path = Path(
                self._t4_dataset_paths[0], token_to_seg_data[token]["filename"]
            ).as_posix()
            labels = np.fromfile(annotation_file_path, dtype=np.uint8)

            points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))

            self.ground_truth[int(sample_data[i]["timestamp"])] = {
                "points": points,
                "labels": labels,
            }

        self.__sub_pointcloud = self.create_subscription(
            PointCloud2,
            self.eval_target_topic,
            self.annotated_pcd_eval_cb,
            qos_profile_sensor_data,
        )

    def annotated_pcd_eval_cb(self, msg: PointCloud2) -> None:
        header_timestamp_microsec: int = eval_conversions.unix_time_microsec_from_ros_msg(
            msg.header
        )
        gt_frame_ts = self.__get_gt_frame_ts(unix_time=header_timestamp_microsec)

        if gt_frame_ts < 0:
            return

        # get ground truth pointcloud in this frame
        # construct kd-tree from gt cloud
        gt_frame_cloud: np.ndarray = self.ground_truth[gt_frame_ts]["points"]
        gt_frame_label: np.ndarray = self.ground_truth[gt_frame_ts]["labels"]
        kdtree = cKDTree(gt_frame_cloud[:, 0:3])

        # convert ros2 pointcloud to numpy
        numpy_pcd = ros2_numpy.numpify(msg)
        pointcloud = np.zeros((numpy_pcd.shape[0], 3))
        pointcloud[:, 0] = numpy_pcd["x"]
        pointcloud[:, 1] = numpy_pcd["y"]
        pointcloud[:, 2] = numpy_pcd["z"]

        if gt_frame_cloud.shape[0] != gt_frame_label.shape[0]:
            err_msg = (
                f"ground truth cloud and label size mismatch: "
                f"cloud points={gt_frame_cloud.shape[0]}, labels={gt_frame_label.shape[0]}"
            )
            raise ValueError(err_msg)

        # count TP+FN, TN+FP
        tp_fn = 0
        for ground_label in self.ground_label:
            tp_fn += np.count_nonzero(gt_frame_label == ground_label)

        fp_tn = 0
        for obstacle_label in self.obstacle_label:
            fp_tn += np.count_nonzero(gt_frame_label == obstacle_label)

        tn: int = 0
        fn: int = 0
        for p in pointcloud:
            _, idx = kdtree.query(p, k=1)
            if gt_frame_label[idx] in self.ground_label:
                fn += 1
            elif gt_frame_label[idx] in self.obstacle_label:
                tn += 1
        tp = tp_fn - fn
        fp = fp_tn - tn

        self.get_logger().info(f"TP {tp}, FP {fp}, TN {tn}, FN {fn}")

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

        self._result.set_frame(frame_result)
        res_str = self._result_writer.write_result(self._result)
        self._pub_result.publish(String(data=res_str))

    def __get_gt_frame_ts(self, unix_time: int) -> int:
        ts_itr = iter(self.ground_truth.keys())
        ret_ts: int = int(next(ts_itr))
        min_diff: int = abs(unix_time - ret_ts)

        for _ in range(1, len(self.ground_truth)):
            sample_ts = next(ts_itr)
            diff_time = abs(unix_time - sample_ts)
            if diff_time < min_diff:
                min_diff = diff_time
                ret_ts = sample_ts

        if min_diff > self.TS_DIFF_THRESH:
            self.get_logger().warn("time diff is too big")
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


@evaluator_main
def main() -> DLREvaluatorV2:
    return GroundSegmentationEvaluator("ground_segmentation_evaluator")


if __name__ == "__main__":
    main()
