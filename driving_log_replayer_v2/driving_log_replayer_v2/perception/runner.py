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
from typing import TYPE_CHECKING

from autoware_perception_msgs.msg import DetectedObjects
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects
from perception_eval.evaluation import PerceptionFrameResult
from rclpy.clock import Clock
from rosbag2_py import TopicMetadata
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.perception.manager import EvaluationManager
from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.perception.ros2_utils import lookup_transform
from driving_log_replayer_v2.perception.ros2_utils import RosBagManager
from driving_log_replayer_v2.perception.topics import load_evaluation_topics
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.perception.analyze import analyze

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped
    from perception_eval.common.object import DynamicObject
    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.tool import PerceptionAnalyzer3D

    from driving_log_replayer_v2.perception.ros2_utils import MsgType


def convert_to_perception_eval(
    msg: MsgType,
    subscribed_ros_timestamp: int,
    evaluation_config: PerceptionEvaluationConfig,
) -> tuple[int, int, list[DynamicObject] | str]:
    header_unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
    subscribed_unix_time: int = eval_conversions.unix_time_from_ros_clock_int(
        subscribed_ros_timestamp
    )
    estimated_objects: list[DynamicObject] | str = (
        eval_conversions.list_dynamic_object_from_ros_msg(
            header_unix_time,
            msg.objects,
            evaluation_config,
        )
    )
    return header_unix_time, subscribed_unix_time, estimated_objects


def convert_to_ros_msg(
    frame: PerceptionFrameResult,
    header: Header,
) -> tuple[MarkerArray, MarkerArray]:
    marker_ground_truth = MarkerArray()
    color_success = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)

    for cnt, obj in enumerate(frame.frame_ground_truth.objects, start=1):
        bbox, uuid = eval_conversions.object_state_to_ros_box_and_uuid(
            obj.state,
            header,
            "ground_truth",
            cnt,
            color_success,
            obj.uuid,
        )
        marker_ground_truth.markers.append(bbox)
        marker_ground_truth.markers.append(uuid)

    marker_results = eval_conversions.pass_fail_result_to_ros_points_array(
        frame.pass_fail_result,
        header,
    )
    return marker_ground_truth, marker_results


def write_result(
    additional_record_topic_name: dict[str],
    result: PerceptionResult,
    result_writer: ResultWriter,
    rosbag_manager: RosBagManager,
    msg: MsgType,
    subscribed_ros_timestamp: int,
    frame_info: dict[PerceptionFrameResult | str, int],
) -> None:
    """Write result.jsonl and rosbag."""
    # NOTE: In offline evaluation using rosbag with SequentialReader(), messages are processed one-by-one.
    #       So it is impossible to get transform of future unless explicitly set the tf of future in the buffer.
    map_to_baselink: TransformStamped = lookup_transform(
        rosbag_manager.get_tf_buffer(),
        msg.header.stamp,
    )

    if isinstance(frame_info["frame"], PerceptionFrameResult):
        # handle when add_frame is success
        result.set_frame(
            **frame_info,
            map_to_baselink=DLREvaluatorV2.transform_stamped_with_euler_angle(map_to_baselink),
        )

        # this topic is written to rosbag with msg.header.timestamp, so it can show the accuracy of the result itself
        # but this topic is actually subscribed to subscribed_ros_timestamp, so ignoring delay
        marker_ground_truth, marker_results = convert_to_ros_msg(
            frame_info["frame"],
            msg.header,
        )
        rosbag_manager.write_results(
            additional_record_topic_name["ground_truth"], marker_ground_truth, msg.header.stamp
        )  # ground truth
        rosbag_manager.write_results(
            additional_record_topic_name["results"], marker_results, msg.header.stamp
        )  # results including evaluation topic and ground truth
    elif isinstance(frame_info["frame"], str):
        # handle when add_frame is fail caused by failed object conversion or no ground truth
        if frame_info["frame"] == "No Ground Truth":
            result.set_info_frame(frame_info["frame"], frame_info["skip"])
        elif frame_info["frame"] == "Invalid Estimated Objects":
            result.set_warn_frame(frame_info["frame"], frame_info["skip"])
        else:
            err_msg = f"Unknown add_frame failure: {frame_info['frame']}"
            raise TypeError(err_msg)
    else:
        err_msg = f"Unknown frame result: {frame_info['frame']}"
        raise TypeError(err_msg)
    result_writer.write_result_with_time(result, subscribed_ros_timestamp)


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4dataset_path: str,
    result_json_path: str,
    result_archive_path: str,
    evaluation_detection_topic_regex: str,
    evaluation_tracking_topic_regex: str,
    evaluation_prediction_topic_regex: str,
    evaluation_fp_validation_topic_regex: str,
) -> None:
    with open(result_json_path, "w") as result_json_file:
        result_json_file.write(scenario_path + "\n")

    evaluation_topics = load_evaluation_topics(
        evaluation_detection_topic_regex,
        evaluation_tracking_topic_regex,
        evaluation_prediction_topic_regex,
        evaluation_fp_validation_topic_regex,
    )

    evaluator = EvaluationManager(
        scenario_path,
        t4dataset_path,
        result_archive_path,
        evaluation_topics,
    )
    if evaluator.check_error() is not None:
        error = evaluator.check_error()
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

    result_writer = ResultWriter(
        result_json_path,
        Clock(),
        evaluator.get_evaluation_condition(),
    )
    result = PerceptionResult(evaluator.get_evaluation_condition())

    additional_record_topic_name = {
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
        for topic_name in additional_record_topic_name.values()
    ]
    rosbag_manager = RosBagManager(
        bag_dir=rosbag_dir_path,
        output_bag_dir=Path(result_archive_path).joinpath("result_bag").as_posix(),
        storage_type="sqlite3",
        evaluation_topic=evaluator.get_evaluation_topics(),
        additional_record_topic=additional_record_topic,
    )

    for topic_name, msg, subscribed_ros_timestamp in rosbag_manager.read_messages():
        # See RosBagManager for `time relationships`.

        if isinstance(msg, DetectedObjects):
            interpolation: bool = False
        elif isinstance(msg, TrackedObjects | PredictedObjects):
            interpolation: bool = True
        else:
            err_msg = f"Unknown message type: {type(msg)}"
            raise TypeError(err_msg)

        # convert ros to perception_eval
        header_unix_time, subscribed_unix_time, estimated_objects = convert_to_perception_eval(
            msg,
            subscribed_ros_timestamp,
            evaluator.get_evaluation_config(topic_name),
        )

        # matching process between estimated_objects and ground truth for evaluation
        frame_result, skip_counter = evaluator.add_frame(
            topic_name,
            estimated_objects,
            header_unix_time,
            subscribed_unix_time,
            interpolation=interpolation,
        )

        # write rosbag result only degradation topic
        if topic_name == degradation_topic:
            write_result(
                additional_record_topic_name,
                result,
                result_writer,
                rosbag_manager,
                msg,
                subscribed_ros_timestamp,
                {"frame": frame_result, "skip": skip_counter},
            )
    rosbag_manager.close_writer()

    # calculation of the overall evaluation like mAP, TP Rate, etc and save evaluated data.
    final_metrics: dict[str, dict] = evaluator.get_evaluation_results()
    result.set_final_metrics(final_metrics[degradation_topic])
    result_writer.write_result_with_time(result, rosbag_manager.get_last_ros_timestamp())
    result_writer.close()

    # analysis of the evaluation result
    analyzers: dict[str, PerceptionAnalyzer3D] = evaluator.get_analyzers()
    result: dict[str, dict] = {
        topic: analyze(analyzer) for topic, analyzer in analyzers.items()
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze perception rosbags")
    parser.add_argument("--scenario-path", help="Directory path to scenario files")
    parser.add_argument("--rosbag-dir-path", help="Directory path to rosbags")
    parser.add_argument("--t4dataset-path", help="Directory path to T4 dataset files")
    parser.add_argument("--result-json-path", help="Output filepath for the result in JSONL format")
    parser.add_argument(
        "--result-archive-path", help="Output filepath for the result in CSV format"
    )
    parser.add_argument(
        "--evaluation-detection-topic-regex",
        default="""\
        """,
        help="Regex pattern for evaluation detection topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed.",
    )
    parser.add_argument(
        "--evaluation-tracking-topic-regex",
        default="""\
        """,
        help="Regex pattern for evaluation tracking topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed.",
    )
    parser.add_argument(
        "--evaluation-prediction-topic-regex",
        default="""\
        """,
        help="Regex pattern for evaluation prediction topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed.",
    )
    parser.add_argument(
        "--evaluation-fp-validation-topic-regex",
        default="""\
        """,
        help="Regex pattern for evaluation fp_validation topic name. Must start with '^' and end with '$'. Wildcards (e.g. '.*', '+', '?', '[...]') are not allowed.",
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
        args.evaluation_detection_topic_regex,
        args.evaluation_tracking_topic_regex,
        args.evaluation_prediction_topic_regex,
        args.evaluation_fp_validation_topic_regex,
    )


if __name__ == "__main__":
    main()
