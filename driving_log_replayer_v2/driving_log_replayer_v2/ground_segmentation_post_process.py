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
from typing import TYPE_CHECKING, List, Dict, Any

import numpy as np
from rclpy.clock import Clock
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata
import ros2_numpy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from pydantic import BaseModel
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from driving_log_replayer_v2.ground_segmentation import GroundSegmentationResult, GroundSegmentationScenario
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.scenario import number, load_scenario
from driving_log_replayer_v2_msgs.msg import GroundSegmentationEvalResult

if TYPE_CHECKING:
    pass

class Condition(BaseModel):
    ground_label: List[int]
    obstacle_label: List[int]
    accuracy_min: number
    accuracy_max: number
    PassRate: number

class GroundSegmentationEvaluator:
    """Evaluator for ground segmentation results."""
    CLOUD_DIM = 5
    TS_DIFF_THRESH = 75000000

    def __init__(
        self,
        scenario_path: str,
        t4_dataset_paths: List[str],
        result_archive_path: str,
        evaluation_topic: str,
    ) -> None:
        self.result_archive_path = Path(result_archive_path)
        self.ground_truth = self._load_ground_truth(t4_dataset_paths)
        self.ground_truth_timestamps = list(self.ground_truth.keys())
        self.evaluation_topic = evaluation_topic
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

    def _load_ground_truth(self, t4_dataset_paths: List[str]) -> Dict[int, Dict[str, np.ndarray]]:
        """Load ground truth point cloud and annotation data."""
        sample_data_path = Path(t4_dataset_paths[0], "annotation", "sample_data.json")
        sample_data = json.load(sample_data_path.open())
        sample_data = [d for d in sample_data if d["filename"].split(".")[-2] == "pcd"]
        lidar_seg_json_path = Path(t4_dataset_paths[0], "annotation", "lidarseg.json")
        lidar_seg_data = json.load(lidar_seg_json_path.open())
        token_to_seg_data = {a["sample_data_token"]: a for a in lidar_seg_data}
        ground_truth = {}
        for entry in sample_data:
            raw_points_file_path = Path(t4_dataset_paths[0], entry["filename"]).as_posix()
            raw_points = np.fromfile(raw_points_file_path, dtype=np.float32)
            token = entry["token"]
            if token not in token_to_seg_data:
                continue
            annotation_file_path = Path(t4_dataset_paths[0], token_to_seg_data[token]["filename"]).as_posix()
            labels = np.fromfile(annotation_file_path, dtype=np.uint8)
            points = raw_points.reshape((-1, self.CLOUD_DIM))
            timestamp_nanosec = int(entry["timestamp"]) * 1000
            ground_truth[timestamp_nanosec] = {
                "points": points,
                "labels": labels,
            }
        print(f"Loaded {len(ground_truth)} ground truth frames")
        return ground_truth

    def _get_gt_frame_ts(self, unix_time: int) -> int:
        """Find the closest ground truth timestamp."""
        ret_ts = min(self.ground_truth.keys(), key=lambda ts: abs(unix_time - ts))
        min_diff = abs(unix_time - ret_ts)
        if min_diff > self.TS_DIFF_THRESH:
            return -1
        return ret_ts

    def _compute_metrics(self, tp: int, fp: int, tn: int, fn: int) -> Dict[str, float]:
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

    def success_str(self) -> str:
        return "Success" if self.success else "Fail"

    def rate(self) -> float:
        return 0.0 if self.total == 0 else self.passed / self.total * 100.0

    def set_frame(self, msg: GroundSegmentationEvalResult, ros_timestamp: float) -> Dict[str, Any]:
        self.total += 1
        frame_success = self.condition.accuracy_min <= msg.accuracy <= self.condition.accuracy_max
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
                "ROS": ros_timestamp
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
    evaluation_topic: str
) -> None:
    """Main evaluation function for ground segmentation."""
    t4_dataset_paths = t4_dataset_path.split(":") if t4_dataset_path else []
    evaluator = GroundSegmentationEvaluator(
        scenario_path,
        t4_dataset_paths,
        result_archive_path,
        evaluation_topic,
    )
    result_writer = ResultWriter(
        result_json_path,
        Clock(),
        evaluator.condition,
    )
    if not Path(result_archive_path).exists():
        Path(result_archive_path).mkdir(parents=True, exist_ok=True)
    shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))
    storage_options = StorageOptions(uri=rosbag_dir_path, storage_id=storage)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    target_topic_type = next((t.type for t in topic_types if t.name == evaluation_topic), None)
    if target_topic_type != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"Expected PointCloud2 topic, got {target_topic_type}")
    print(f"Evaluating topic: {evaluation_topic}")
    print(f"Ground truth frames: {len(evaluator.ground_truth)}")
    frame_results = []
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name != evaluation_topic:
            continue
        gt_timestamp = evaluator._get_gt_frame_ts(unix_time=timestamp)
        if gt_timestamp < 0:
            continue
        print(f"Processing frame at {gt_timestamp}")
        gt_frame_cloud = evaluator.ground_truth[gt_timestamp]["points"]
        gt_frame_label = evaluator.ground_truth[gt_timestamp]["labels"]
        kdtree = cKDTree(gt_frame_cloud[:, 0:3])
        msg_type = get_message(target_topic_type)
        msg = deserialize_message(data, msg_type)
        numpy_pcd = ros2_numpy.numpify(msg)
        pointcloud = np.stack((numpy_pcd["x"], numpy_pcd["y"], numpy_pcd["z"]), axis=-1)
        tp_fn = sum(np.count_nonzero(gt_frame_label == label) for label in evaluator.ground_label)
        fp_tn = sum(np.count_nonzero(gt_frame_label == label) for label in evaluator.obstacle_label)
        tn = fn = 0
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
        res_str = evaluator.set_frame(frame_result, ros_timestamp=gt_timestamp / 1e9)
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
        args.evaluation_topic,
    )


if __name__ == "__main__":
    main()