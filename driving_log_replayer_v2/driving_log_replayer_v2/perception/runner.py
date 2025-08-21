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
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

from rclpy.clock import Clock
from rosbag2_py import TopicMetadata
from tier4_api_msgs.msg import AwapiAutowareStatus

from driving_log_replayer_v2.driving_log_replayer_v2.perception.perception.analyze import analyze
from driving_log_replayer_v2.driving_log_replayer_v2.perception.perception.manager import (
    EvaluationManager,
)
from driving_log_replayer_v2.driving_log_replayer_v2.perception.perception.processor import (
    process_perception_message,
)
from driving_log_replayer_v2.driving_log_replayer_v2.perception.stop_reason.processor import (
    process_stop_reason_message,
)
from driving_log_replayer_v2.driving_log_replayer_v2.perception.stop_reason.processor import (
    process_stop_reason_timeouts,
)
from driving_log_replayer_v2.driving_log_replayer_v2.perception.stop_reason.evaluator import (
    StopReasonEvaluator,
)
from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.perception.ros2_utils import RosBagManager
from driving_log_replayer_v2.perception.topics import load_evaluation_topics
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.result import ResultWriter

if TYPE_CHECKING:
    from perception_eval.tool import PerceptionAnalyzer3D


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4dataset_path: str,
    result_json_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_detection_topic_regex: str,
    evaluation_tracking_topic_regex: str,
    evaluation_prediction_topic_regex: str,
    analysis_max_distance: str,
    analysis_distance_interval: str,
) -> None:
    evaluation_topics = load_evaluation_topics(
        evaluation_detection_topic_regex,
        evaluation_tracking_topic_regex,
        evaluation_prediction_topic_regex,
    )

    # initialize EvaluationManager to evaluate multiple topics
    evaluator = EvaluationManager(
        scenario_path,
        t4dataset_path,
        result_archive_path,
        evaluation_topics,
    )
    if evaluator.check_scenario_error() is not None:
        error = evaluator.check_scenario_error()
        result_writer = ResultWriter(
            result_json_path,
            Clock(),
            {},
        )
        error_dict = {
            "Result": {"Success": False, "Summary": "ScenarioFormatError"},
            "Stamp": {"System": 0.0},
            "Frame": {"ErrorMsg": error.__str__()},
        }
        result_writer.write_line(error_dict)
        result_writer.close()
        raise error
    degradation_topic = evaluator.get_degradation_topic()

    # initialize ResultWriter to write result.jsonl which Evaluator will use
    result_writer = ResultWriter(
        result_json_path,
        Clock(),
        evaluator.get_evaluation_condition(),
    )
    result = PerceptionResult(evaluator.get_evaluation_condition())

    # initialize RosBagManager to read and save rosbag and write into it
    result_topic = {
        "ground_truth": "/driving_log_replayer_v2/marker/ground_truth",
        "results": "/driving_log_replayer_v2/marker/results",
    }
    additional_record_topic = [
        TopicMetadata(
            name=topic_name,
            type="visualization_msgs/MarkerArray",
            serialization_format="cdr",
            offered_qos_profiles="",
        )
        for topic_name in result_topic.values()
    ]
    rosbag_manager = RosBagManager(
        rosbag_dir_path,
        Path(result_archive_path).joinpath("result_bag").as_posix(),
        storage,
        evaluator.get_evaluation_topics(),
        additional_record_topic,
    )

    # save scenario.yaml to analyze later
    shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    # Initialize stop reason processor
    stop_reason_processor = StopReasonEvaluator(Path(result_archive_path))

    # main evaluation process
    for topic_name, msg, subscribed_ros_timestamp in rosbag_manager.read_messages():
        # See RosBagManager for `time relationships`.

        # Process stop_reason data from AwapiAutowareStatus messages
        if isinstance(msg, AwapiAutowareStatus) and process_stop_reason_message(
            msg,
            subscribed_ros_timestamp,
            stop_reason_processor,
            result,
            result_writer,
        ):
            continue

        # Process perception messages (DetectedObjects, TrackedObjects, PredictedObjects)
        process_perception_message(
            msg,
            topic_name,
            degradation_topic,
            result_topic,
            subscribed_ros_timestamp,
            evaluator,
            result,
            result_writer,
            rosbag_manager,
        )
    rosbag_manager.close_writer()
    stop_reason_processor.save_to_spreadsheet()

    # calculation of the overall evaluation like mAP, TP Rate, etc and save evaluated data.
    final_metrics: dict[str, dict] = evaluator.get_evaluation_results()
    result.set_final_metrics(final_metrics[degradation_topic])
    result_writer.write_result_with_time(result, rosbag_manager.get_last_ros_timestamp())
    result_writer.close()

    # analysis of the evaluation result and save it as csv
    if evaluator.get_degradation_evaluation_task() != "fp_validation":
        analyzers: dict[str, PerceptionAnalyzer3D] = evaluator.get_analyzers()
        # TODO: analysis other topic
        analyzer = analyzers[degradation_topic]
        save_path = evaluator.get_archive_path(degradation_topic)
        analyze(
            analyzer,
            save_path,
            analysis_max_distance,
            analysis_distance_interval,
            degradation_topic,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate perception rosbag w/ t4dataset")
    parser.add_argument("--scenario-path", required=True, help="File path to scenario files")
    parser.add_argument(
        "--rosbag-dir-path",
        required=True,
        help="Directory path to rosbag which is outputted by Autoware",
    )
    parser.add_argument("--t4dataset-path", required=True, help="Directory path to t4dataset")
    parser.add_argument(
        "--result-json-path", required=True, help="Output file path for the result in JSONL format"
    )
    parser.add_argument(
        "--result-archive-path", required=True, help="Output directory path for the result"
    )
    parser.add_argument(
        "--storage",
        required=True,
        help="Storage type for rosbag2",
    )
    parser.add_argument(
        "--evaluation-detection-topic-regex",
        default="",
        help="Regex pattern for evaluation detection topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    )
    parser.add_argument(
        "--evaluation-tracking-topic-regex",
        default="",
        help="Regex pattern for evaluation tracking topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    )
    parser.add_argument(
        "--evaluation-prediction-topic-regex",
        default="",
        help="Regex pattern for evaluation prediction topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed. If you do not want to use this feature, set it to '' or 'None'.",
    )
    parser.add_argument(
        "--analysis-max-distance",
        required=True,
        help="Maximum distance for analysis.",
    )
    parser.add_argument(
        "--analysis-distance-interval",
        required=True,
        help="Distance interval for analysis.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    evaluate(
        args.scenario_path,
        args.rosbag_dir_path,
        args.t4dataset_path,
        args.result_json_path,
        args.result_archive_path,
        args.storage,
        args.evaluation_detection_topic_regex,
        args.evaluation_tracking_topic_regex,
        args.evaluation_prediction_topic_regex,
        args.analysis_max_distance,
        args.analysis_distance_interval,
    )


if __name__ == "__main__":
    main()
