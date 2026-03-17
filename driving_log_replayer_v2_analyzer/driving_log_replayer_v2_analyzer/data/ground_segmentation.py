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
from typing import Iterator

import yaml


class JsonlParser:
    def __init__(self, input_jsonl: Path) -> None:
        self._input_jsonl = input_jsonl
        self._data = []
        with input_jsonl.open("r") as f:
            for line in f:
                try:
                    record = json.loads(line)
                    if "Frame" in record and "Ground Segmentation" in record["Frame"]:
                        self._data.append(record)
                except json.JSONDecodeError:
                    continue

    def get_evaluation_topic(self) -> str:
        metadata_path = self._input_jsonl.parent / "result_bag" / "metadata.yaml"
        if not metadata_path.exists():
            return self._input_jsonl.parent.name

        with metadata_path.open("r") as f:
            metadata = yaml.safe_load(f)
            topics = (
                metadata.get("rosbag2_bagfile_information", {})
                .get("topics_with_message_count", [])
            )
            for t in topics:
                meta = t.get("topic_metadata", {})
                if meta.get("type") == "sensor_msgs/msg/PointCloud2":
                    name = meta.get("name")
                    if "pointcloud" in name:
                        return name
        return self._input_jsonl.parent.name

    def _get_info_and_time(self) -> Iterator[tuple[dict, float]]:
        for i, record in enumerate(self._data):
            try:
                info = record["Frame"]["Ground Segmentation"]["GroundSegmentation"]["Info"]
                ros_time = record["Stamp"].get("ROS", i)
                yield info, ros_time
            except KeyError:
                continue

    def get_metrics_timeseries(self) -> list[dict]:
        time_series = []
        for info, ros_time in self._get_info_and_time():
            time_series.append(
                {
                    "x": ros_time,
                    "Accuracy": info["Accuracy"] * 100.0,
                    "Precision": info["Precision"] * 100.0,
                    "Recall": info["Recall"] * 100.0,
                    "F1-score": info["F1-score"] * 100.0,
                }
            )
        return time_series

    def get_confusion_matrix_timeseries(self) -> list[dict]:
        time_series = []
        for info, ros_time in self._get_info_and_time():
            time_series.append(
                {
                    "x": ros_time,
                    "TP": info["TP"],
                    "FP": info["FP"],
                    "TN": info["TN"],
                    "FN": info["FN"],
                }
            )
        return time_series
