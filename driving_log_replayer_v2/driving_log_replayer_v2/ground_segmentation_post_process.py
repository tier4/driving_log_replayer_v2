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

import argparse
import json
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

import numpy as np
from sympy import true
from rclpy.clock import Clock
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions
from rosbag2_py import TopicMetadata
import ros2_numpy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from pydantic import BaseModel
# from driving_log_replayer_v2.ground_segmentation.manager import EvaluationManager
from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.ground_segmentation import GroundSegmentationResult
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.scenario import number
from driving_log_replayer_v2.scenario import load_scenario
from driving_log_replayer_v2.ground_segmentation import GroundSegmentationScenario
from driving_log_replayer_v2_msgs.msg import GroundSegmentationEvalResult
import pdb

# Import required modules for message deserialization
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


if TYPE_CHECKING:
    pass

class Condition(BaseModel):
    ground_label: list[int]
    obstacle_label: list[int]
    accuracy_min: number
    accuracy_max: number
    PassRate: number
class GroundSegmentationEvaluator:
    CLOUD_DIM = 5
    TS_DIFF_THRESH = 75000000

    def __init__(
        self,
        scenario_path: str,
        t4_dataset_paths: list[str],
        result_archive_path: str,
        evaluation_topic: str,
    ) -> None:
        
        
        self.result_archive_path = Path(result_archive_path)
        self.ground_truth = self._load_ground_truth(t4_dataset_paths)
        self.ground_truth_timestamps = list(self.ground_truth.keys())

        # Get evaluation conditions
        self.evaluation_topic = evaluation_topic
        # extract ground_label from scenario_path yaml
        self.scenario = load_scenario(Path(scenario_path), GroundSegmentationScenario)
        self.ground_label = self.scenario.Evaluation.Conditions.ground_label
        self.obstacle_label = self.scenario.Evaluation.Conditions.obstacle_label

        self.total = 0
        self.passed = 0
        self.success = False
        self.summary = ""
        self.name = "GroundSegmentation"
        self.condition: Condition = self.scenario.Evaluation.Conditions
        print(f"Evaluation conditions: {self.condition}")


    def _load_ground_truth(self, t4_dataset_paths: list[str]) -> dict[int, dict[str, np.ndarray]]:
        """Load ground truth point cloud and annotation data."""
        sample_data_path = Path(t4_dataset_paths[0], "annotation", "sample_data.json")
        sample_data = json.load(sample_data_path.open())
        sample_data = list(filter(lambda d: d["filename"].split(".")[-2] == "pcd", sample_data))

        # Load gt annotation data
        lidar_seg_json_path = Path(t4_dataset_paths[0], "annotation", "lidarseg.json")
        lidar_seg_data = json.load(lidar_seg_json_path.open())
        token_to_seg_data = {}
        for annotation_data in lidar_seg_data:
            token_to_seg_data[annotation_data["sample_data_token"]] = annotation_data

        ground_truth: dict[int, dict[str, np.ndarray]] = {}
        for i in range(len(sample_data)):
            raw_points_file_path = Path(
                t4_dataset_paths[0],
                sample_data[i]["filename"],
            ).as_posix()
            raw_points = np.fromfile(raw_points_file_path, dtype=np.float32)
            token = sample_data[i]["token"]

            if token not in token_to_seg_data:
                continue
            annotation_file_path = Path(
                t4_dataset_paths[0], token_to_seg_data[token]["filename"]
            ).as_posix()
            labels = np.fromfile(annotation_file_path, dtype=np.uint8)

            points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))

            # Convert microseconds to nanoseconds for consistency with ROS bag timestamps
            timestamp_nanosec = int(sample_data[i]["timestamp"]) * 1000
            ground_truth[timestamp_nanosec] = {
                "points": points,
                "labels": labels,
            }

        print(f"Loaded {len(ground_truth)} ground truth frames")
        return ground_truth

    def _get_gt_frame_ts(self, unix_time: int) -> int:
        """Find the closest ground truth timestamp."""
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
            return -1

        return ret_ts

    def _compute_metrics(self, tp: int, fp: int, tn: int, fn: int) -> dict[str, float]:
        """Compute evaluation metrics."""
        eps = 1e-10
        accuracy = float(tp + tn) / float(tp + fp + tn + fn + eps)
        precision = float(tp) / float(tp + fp + eps)
        recall = float(tp) / float(tp + fn + eps)
        specificity = float(tn) / float(tn + fp + eps)
        f1_score = 2 * (precision * recall) / (precision + recall + eps)
        
        return {
            "accuracy": accuracy,
            "precision": precision,
            "recall": recall,
            "specificity": specificity,
            "f1_score": f1_score,
        }

    def _evaluate_success(self, metrics: dict[str, float]) -> bool:
        """Evaluate if the result meets the success criteria."""
        if self.criteria_method not in metrics:
            return False
        return metrics[self.criteria_method] >= self.threshold

    def evaluate_pointcloud(self, msg: PointCloud2) -> dict:
        """Evaluate ground segmentation performance for a single pointcloud."""
        header_timestamp_microsec: int = eval_conversions.unix_time_microsec_from_ros_msg(
            msg.header
        )
        
        gt_frame_ts = self._get_gt_frame_ts(unix_time=header_timestamp_microsec)

        if gt_frame_ts < 0:
            return self._create_empty_result()

        # Construct kd-tree from gt cloud
        gt_frame_cloud: np.ndarray = self.ground_truth[gt_frame_ts]["points"]
        gt_frame_label: np.ndarray = self.ground_truth[gt_frame_ts]["labels"]
        kdtree = cKDTree(gt_frame_cloud[:, 0:3])

        # Convert ros2 pointcloud to numpy
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

        # Count TP+FN, TN+FP
        tp_fn = sum(np.count_nonzero(gt_frame_label == ground_label) for ground_label in self.ground_label)
        fp_tn = sum(np.count_nonzero(gt_frame_label == obstacle_label) for obstacle_label in self.obstacle_label)

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

        metrics = self._compute_metrics(tp, fp, tn, fn)
        success = self._evaluate_success(metrics)

        return {
            "tp": tp,
            "fp": fp,
            "tn": tn,
            "fn": fn,
            "success": success,
            **metrics,
        }

    def _create_empty_result(self) -> dict:
        """Create empty result when no ground truth is available."""
        return {
            "tp": 0,
            "fp": 0,
            "tn": 0,
            "fn": 0,
            "accuracy": 0.0,
            "precision": 0.0,
            "recall": 0.0,
            "specificity": 0.0,
            "f1_score": 0.0,
            "success": False,
        }
    def success_str(self) -> str:
        return "Success" if self.success else "Fail"
    
    def rate(self) -> float:
        return 0.0 if self.total == 0 else self.passed / self.total * 100.0

    def set_frame(self, msg: GroundSegmentationEvalResult, ROS_timestamp: float) -> dict:
        self.condition: Condition
        self.total += 1
        frame_success = self.condition.accuracy_min <= msg.accuracy <= self.condition.accuracy_max
        self.success = frame_success
        if frame_success:
            self.passed += 1
        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}"
        return {
            "Result": {
                "Success": frame_success,
                "Summary": f"Passed: Ground Segmentation ({'Success' if frame_success else 'Fail'}): {self.passed} / {self.total} -> {current_rate:.2f}"
            },
            "Stamp": {
                "ROS": ROS_timestamp
            },
            "Frame": {
                "GroundSegmentation": {
                    "Result": {
                        "Total": self.success_str(),
                        "Frame": "Success" if frame_success else "Fail",
                    },
                    "Info": {
                        "TP": msg.tp,
                        "FP": msg.fp,
                        "TN": msg.tn,
                        "FN": msg.fn,
                    "Accuracy": msg.accuracy,
                    "Precision": msg.precision,
                    "Recall": msg.recall,
                    "Specificity": msg.specificity,
                    "F1-score": msg.f1_score,
                    },
                },
            }
        }


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4_dataset_path: str,
    result_json_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_topic
) -> None:
    """Main evaluation function for ground segmentation."""
    
    # Parse t4_dataset_path into list
    t4_dataset_paths = t4_dataset_path.split(":") if t4_dataset_path else []
    
    # Initialize evaluator
    evaluator = GroundSegmentationEvaluator(
        scenario_path,
        t4_dataset_paths,
        result_archive_path,
        evaluation_topic,
    )

    # Initialize result writer
    result_writer = ResultWriter(
        result_json_path,
        Clock(),
        evaluator.condition,
    )

    # Save scenario.yaml for reference
    if not Path(result_archive_path).exists():
        Path(result_archive_path).mkdir(parents=True, exist_ok=True)
    shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    # Setup ROS bag reader
    storage_options = StorageOptions(uri=rosbag_dir_path, storage_id=storage)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic info
    topic_types = reader.get_all_topics_and_types()
    target_topic = evaluation_topic
    
    # Find the target topic type
    target_topic_type = None
    for topic_metadata in topic_types:
        if topic_metadata.name == target_topic:
            target_topic_type = topic_metadata.type
            break
    
    if target_topic_type != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"Expected PointCloud2 topic, got {target_topic_type}")

    print(f"Evaluating topic: {target_topic}")
    print(f"Ground truth frames: {len(evaluator.ground_truth)}")
    # print(f"Criteria: {evaluator.criteria_method} >= {evaluator.threshold}")

    # Process messages
    frame_results = []
    processed_frames = 0
    successful_frames = 0
    print("gt_timestamps:", evaluator.ground_truth_timestamps)
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name != target_topic:
            continue
        gt_timestamp = evaluator._get_gt_frame_ts(unix_time=timestamp)
        if gt_timestamp < 0:
            continue
        print(f"Processing frame at {gt_timestamp}")
        gt_frame_cloud: np.ndarray = evaluator.ground_truth[gt_timestamp]["points"]
        gt_frame_label: np.ndarray = evaluator.ground_truth[gt_timestamp]["labels"]
        kdtree = cKDTree(gt_frame_cloud[:, 0:3])

        msg_type = get_message(target_topic_type)
        msg = deserialize_message(data, msg_type)
        numpy_pcd = ros2_numpy.numpify(msg)

        pointcloud = np.zeros((numpy_pcd.shape[0], 3))
        pointcloud[:, 0] = numpy_pcd["x"]
        pointcloud[:, 1] = numpy_pcd["y"]
        pointcloud[:, 2] = numpy_pcd["z"]
        tp_fn = 0
        for ground_label in evaluator.ground_label:
            tp_fn += np.count_nonzero(gt_frame_label == ground_label)

        fp_tn = 0
        for obstacle_label in evaluator.obstacle_label:
            fp_tn += np.count_nonzero(gt_frame_label == obstacle_label)

        tn: int = 0
        fn: int = 0
        for p in pointcloud:
            _, idx = kdtree.query(p, k=1)
            if gt_frame_label[idx] in evaluator.ground_label:
                fn += 1
            elif gt_frame_label[idx] in evaluator.obstacle_label:
                tn += 1
        tp = tp_fn - fn
        fp = fp_tn - tn

        print(f"TP {tp}, FP {fp}, TN {tn}, FN {fn}")
        metrics_list = evaluator._compute_metrics(tp, fp, tn, fn)

        frame_result = GroundSegmentationEvalResult()
        frame_result.tp = tp
        frame_result.fp = fp
        frame_result.tn = tn
        frame_result.fn = fn
        frame_result.accuracy = metrics_list['accuracy']
        frame_result.precision = metrics_list['precision']
        frame_result.recall = metrics_list['recall']
        frame_result.specificity = metrics_list['specificity']
        frame_result.f1_score = metrics_list['f1_score']
        res_str = evaluator.set_frame(frame_result, ROS_timestamp=gt_timestamp / 1e9)
        res_str = result_writer.write_line(res_str)
        print(f"Result string: {res_str}")

    result_writer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate ground segmentation rosbag w/ t4dataset")
    parser.add_argument("--scenario-path", required=True, help="File path to scenario files")
    parser.add_argument(
        "--rosbag-dir-path",
        required=True,
        help="Directory path to rosbag which is outputted by Autoware",
    )
    parser.add_argument("--t4-dataset-path", required=True, help="Directory path to t4dataset")
    parser.add_argument(
        "--result-json-path", required=True, help="Output file path for the result in JSONL format"
    )
    parser.add_argument(
        "--result-archive-path", required=True, help="Output directory path for the result"
    )
    parser.add_argument(
        "--storage",
        default="sqlite3",
        help="Storage type for rosbag2",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    evaluate(
        args.scenario_path,
        args.rosbag_dir_path,
        args.t4_dataset_path,
        args.result_json_path,
        args.result_archive_path,
        args.storage,
    )


if __name__ == "__main__":
    main()