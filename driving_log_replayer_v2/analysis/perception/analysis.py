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
import yaml
from dataclasses import dataclass

from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
from perception_eval.common import DynamicObject
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.metrics import MetricsScore


@dataclass
class Config:
    datasets: dict[str, str]
    topics: TopicInfo

    @classmethod
    def from_file(cls, filepath: str) -> Config:
        with open(filepath) as f:
            config: dict = yaml.safe_load(f)

        topics = TopicInfo(**config["topics"])
        return cls(config["datasets"], topics)

    @property
    def detection_topics(self) -> list[str]:
        return self.topics.detection

    @property
    def tracking_topics(self) -> list[str]:
        return self.topics.tracking


@dataclass
class TopicInfo:
    detection: list[str] = []
    tracking: list[str] = []


class RosBagReader:
    def __init__(self, bag_dir: str) -> None:
        pass

    def read_messages(self, topic: str) -> list[tuple[int, list[DynamicObject]]]:
        """Read ROS message and returns DynamicObject in perception eval.

        Returns:
            List of pairs of timestamp and frame objects.
        """
        ...

    @staticmethod
    def _to_perception_eval_object(object_msg: DetectedObjects | TrackedObjects) -> DynamicObject:
        pass


@dataclass
class SceneResultContainer:
    dataset_path: str
    scene_metrics: MetricsScore
    frame_results: list[PerceptionFrameResult]


def _load_config(task: EvaluationTask, dataset_path: str) -> PerceptionEvaluationConfig:
    pass


def _evaluate(
    task: EvaluationTask,
    dataset_path: str,
    scene_estimations: list[tuple[int, list[DynamicObject]]]
) -> SceneResultContainer:
    config = _load_config(task, dataset_path)
    manager = PerceptionEvaluationManager(config)

    frame_results: list[PerceptionFrameResult] = []
    for unix_time, frame_estimations in scene_estimations:
        frame_ground_truths = manager.get_ground_truth_now_frame(unix_time)
        frame_result = manager.add_frame_result(unix_time, frame_ground_truths, frame_estimations)  # TODO: The other configs
        frame_results.append(frame_result)

    scene_metrics = manager.get_scene_result()
    return SceneResultContainer(dataset_path, scene_metrics, frame_results)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze perception rosbags")
    parser.add_argument("config", help="Filepath to config")
    parser.add_argument("--abstract-result-path", help="Output filepath for the result in JSONL format")
    parser.add_argument("--detail-result-path", help="Output filepath for the result in CSV format")
    parser.add_argument("--rosbag-dir", help="Directory path to rosbags")
    return parser.parse_args()


def analysis(config_path: str, abstract_result_path: str, detail_result_path: str, rosbag_dir_list: str) -> None:
    config = Config.from_file(config_path)

    results: dict[EvaluationTask, dict[str, list[SceneResultContainer]]] = {
        EvaluationTask.DETECTION: {topic: [] for topic in config.detection_topics},
        EvaluationTask.TRACKING: {topic: [] for topic in config.tracking_topics},
    }

    for bag_dir, dataset_path in rosbag_dir_list:
        reader = RosBagReader(bag_dir)
        # Evaluate detection topics
        task = EvaluationTask.DETECTION
        for detection_topic in config.detection_topics:
            scene_estimations = reader.read_messages(detection_topic)
            scene_result = _evaluate(task, dataset_path, scene_estimations)
            results[task][detection_topic].append(scene_result)

        task = EvaluationTask.TRACKING
        # Evaluate tracking topics
        for tracking_topic in config.tracking_topics:
            scene_estimations = reader.read_messages(tracking_topic)
            scene_result = _evaluate(task, dataset_path, scene_estimations)
            results[task][tracking_topic].append(scene_result)
    
    # TODO: Save the results

    # TODO: analyze each scene result for the corresponding topic


def main() -> None:
    args = parse_args()
    analysis(
        args.config,
        args.abstract_result_path,
        args.detail_result_path,
        args.rosbag_dir_list,
    )


if __name__ == "__main__":
    main()
