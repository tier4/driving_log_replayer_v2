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
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING
from typing import TypeVar

from autoware_perception_msgs.msg import DetectedObjects
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.perception.analyze import analyze
from driving_log_replayer_v2.perception.manager import PerceptionEvaluationManager
from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.perception.models import StopReasonResult
from driving_log_replayer_v2.perception.stop_reason import convert_to_stop_reason
from driving_log_replayer_v2.perception.stop_reason import StopReasonAnalyzer
from driving_log_replayer_v2.perception.stop_reason import StopReasonData
from driving_log_replayer_v2.perception.topics import load_evaluation_topics
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.post_process_evaluator import FrameResult
from driving_log_replayer_v2.post_process_runner import Runner
from driving_log_replayer_v2.post_process_runner import SimulationInfo
from driving_log_replayer_v2.post_process_runner import TopicInfo
from driving_log_replayer_v2.result import MultiResultEditor
from driving_log_replayer_v2.ros2_utils import lookup_transform
from driving_log_replayer_v2.scenario import load_condition
from driving_log_replayer_v2.scenario import load_scenario_with_exception

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped
    from perception_eval.common.object import DynamicObject
    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
    from perception_eval.tool import PerceptionAnalyzer3D


PerceptionMsgType = TypeVar("PerceptionMsgType", DetectedObjects, TrackedObjects, PredictedObjects)


@dataclass
class PerceptionEvalData:
    interpolation: bool
    estimated_objects: list[DynamicObject] | str  # str for error message


def convert_to_perception_eval(
    msg: PerceptionMsgType,
    subscribed_timestamp_nanosec: int,
    evaluation_config: PerceptionEvaluationConfig,
) -> tuple[int, int, PerceptionEvalData]:
    header_timestamp_microsec: int = eval_conversions.unix_time_microsec_from_ros_msg(msg.header)
    subscribed_timestamp_microsec: int = eval_conversions.unix_time_microsec_from_ros_time_nanosec(
        subscribed_timestamp_nanosec
    )
    estimated_objects: list[DynamicObject] | str = (
        eval_conversions.list_dynamic_object_from_ros_msg(
            header_timestamp_microsec,
            msg.objects,
            evaluation_config,
        )
    )
    if isinstance(msg, DetectedObjects):
        interpolation: bool = False
    elif isinstance(msg, TrackedObjects | PredictedObjects):
        interpolation: bool = True
    else:
        err_msg = f"Unknown message type: {type(msg)}"
        raise TypeError(err_msg)

    return (
        header_timestamp_microsec,
        subscribed_timestamp_microsec,
        PerceptionEvalData(interpolation, estimated_objects),
    )


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


class PerceptionRunner(Runner):
    def __init__(
        self,
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_json_path: str,
        result_archive_path: str,
        storage: str,
        evaluation_topics: dict[str, list[str]],
        additional_record_topics: list[TopicInfo],
        enable_analysis: str,
        analysis_max_distance: str,
        analysis_distance_interval: str,
    ) -> None:
        # additional instance variables
        self._analysis_max_distance = analysis_max_distance
        self._analysis_distance_interval = analysis_distance_interval
        self._stop_reason_analyzer: StopReasonAnalyzer | None = None

        simulation_info_list = [
            SimulationInfo(
                evaluation_manager_class=PerceptionEvaluationManager,
                result_class=PerceptionResult,
                key="perception",
                evaluation_topics=evaluation_topics,
                result_json_path=result_json_path,
            )
        ]

        # add stop_reason to SimulationInfo if stop reason criterion is defined
        scenario = load_scenario_with_exception(
            scenario_path,
            PerceptionScenario,
            result_json_path,
        )
        evaluation_condition = load_condition(scenario)
        if evaluation_condition.stop_reason_criterion is not None:
            stop_reason_result_json_path = Path(result_json_path).with_name("stop_reason.jsonl")
            simulation_info_list.append(
                SimulationInfo(
                    evaluation_manager_class=None,
                    result_class=StopReasonResult,
                    key="stop_reason",
                    evaluation_topics=["/awapi/autoware/get/status"],
                    result_json_path=stop_reason_result_json_path,
                )
            )
            self._stop_reason_analyzer = StopReasonAnalyzer(result_archive_path, "stop_reason")

        super().__init__(
            PerceptionScenario,
            simulation_info_list,
            scenario_path,
            rosbag_dir_path,
            t4_dataset_path,
            result_json_path,
            result_archive_path,
            storage,
            additional_record_topics,
            enable_analysis,
        )

    def _evaluate_frame(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> FrameResult:
        # evaluate stop reason criterion if defined
        if "stop_reason" in self._simulation and topic_name == "/awapi/autoware/get/status":
            stop_reason = convert_to_stop_reason(msg)
            return FrameResult(
                is_valid=True,
                data=stop_reason,
                skip_counter=0,
            )

        # convert ros to perception_eval
        header_timestamp_microsec, subscribed_timestamp_microsec, perception_eval_data = (
            convert_to_perception_eval(
                msg,
                subscribed_timestamp_nanosec,
                self._simulation["perception"].evaluation_manager.get_evaluation_config(topic_name),
            )
        )

        # matching process between estimated_objects and ground truth for evaluation
        return self._simulation["perception"].evaluation_manager.evaluate_frame(
            topic_name,
            header_timestamp_microsec,
            subscribed_timestamp_microsec,
            perception_eval_data,
        )

    def _write_result(
        self, frame_result: FrameResult, header: Header, subscribed_timestamp_nanosec: int
    ) -> None:
        if isinstance(frame_result.data, StopReasonData):
            # write stop reason result
            self._simulation["stop_reason"].result.set_frame(frame_result.data)
            self._simulation["stop_reason"].result_writer.write_result_with_time(
                self._simulation["stop_reason"].result, subscribed_timestamp_nanosec
            )
            return

        if frame_result.is_valid:
            # NOTE: In offline evaluation using rosbag with SequentialReader(), messages are processed one-by-one.
            #       So it is impossible to get transform of future unless explicitly set the tf of future in the buffer.
            map_to_baselink: TransformStamped = lookup_transform(
                self._rosbag_manager.get_tf_buffer(),
                header.stamp,
            )

            # handle evaluate_frame is success
            self._simulation["perception"].result.set_frame(
                frame_result.data,
                frame_result.skip_counter,
                map_to_baselink=DLREvaluatorV2.transform_stamped_with_euler_angle(map_to_baselink),
            )

            marker_ground_truth, marker_results = convert_to_ros_msg(
                frame_result.data,
                header,
            )

            # write ground truth as ROS message
            self._rosbag_manager.write_results(
                "/driving_log_replayer_v2/marker/ground_truth",
                marker_ground_truth,
                header.stamp,
            )  # ground truth
            self._rosbag_manager.write_results(
                "/driving_log_replayer_v2/marker/results", marker_results, header.stamp
            )  # results including evaluation topic and ground truth
        else:
            # handle when add_frame is fail caused by failed object conversion or no ground truth
            if frame_result.invalid_reason == "No Ground Truth":
                self._simulation["perception"].result.set_info_frame(
                    frame_result.data, frame_result.skip_counter
                )
            elif frame_result.invalid_reason == "Invalid Estimated Objects":
                self._simulation["perception"].result.set_warn_frame(
                    frame_result.data, frame_result.skip_counter
                )
            else:
                err_msg = f"Unknown invalid_reason: {frame_result.invalid_reason}"
                raise TypeError(err_msg)

        res_str = self._simulation["perception"].result_writer.write_result_with_time(
            self._simulation["perception"].result, subscribed_timestamp_nanosec
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception/results",
            String(data=res_str),
            header.stamp,
        )

    def _evaluate_on_post_process(self) -> None:
        final_metrics: dict[str, dict] = self._simulation[
            "perception"
        ].evaluation_manager.get_evaluation_results()

        perception_degradation_topic = self._degradation_topics[
            0
        ]  # head topic is perception degradation topic
        self._simulation["perception"].result.set_final_metrics(
            final_metrics[perception_degradation_topic]
        )
        res_str = self._simulation["perception"].result_writer.write_result_with_time(
            self._simulation["perception"].result,
            self._rosbag_manager.get_last_subscribed_timestamp(),
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception/results",
            String(data=res_str),
            self._rosbag_manager.get_last_subscribed_timestamp(),
        )

        if "stop_reason" in self._simulation is not None:
            result_paths = [
                self._simulation["perception"].result_writer.result_path,
                self._simulation["stop_reason"].result_writer.result_path,
            ]
            multi_result_editor = MultiResultEditor(result_paths)
            multi_result_editor.write_back_result()

    def _analysis(self) -> None:
        if (
            self._simulation["perception"].evaluation_manager.get_degradation_evaluation_task()
            != "fp_validation"
        ):
            analyzers: dict[str, PerceptionAnalyzer3D] = self._simulation[
                "perception"
            ].evaluation_manager.get_analyzer()

            # TODO: analysis other topic
            perception_degradation_topic = self._degradation_topics[
                0
            ]  # head topic is perception degradation topic
            analyzer = analyzers[perception_degradation_topic]
            save_path = self._simulation["perception"].evaluation_manager.get_archive_path(
                perception_degradation_topic
            )
            analyze(
                analyzer,
                save_path,
                self._analysis_max_distance,
                self._analysis_distance_interval,
                perception_degradation_topic,
            )
        if "stop_reason" in self._simulation:
            self._stop_reason_analyzer.save_as_csv("stop_reason.csv")


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4_dataset_path: str,
    result_json_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_detection_topic_regex: str,
    evaluation_tracking_topic_regex: str,
    evaluation_prediction_topic_regex: str,
    enable_analysis: str,
    analysis_max_distance: str,
    analysis_distance_interval: str,
) -> None:
    evaluation_topics = load_evaluation_topics(
        evaluation_detection_topic_regex,
        evaluation_tracking_topic_regex,
        evaluation_prediction_topic_regex,
    )

    additional_record_topics = [
        TopicInfo(
            name="/driving_log_replayer_v2/marker/ground_truth",
            msg_type="visualization_msgs/MarkerArray",
        ),
        TopicInfo(
            name="/driving_log_replayer_v2/marker/results",
            msg_type="visualization_msgs/MarkerArray",
        ),
        TopicInfo(
            name="/driving_log_replayer_v2/perception/results",
            msg_type="std_msgs/String",
        ),
    ]

    runner = PerceptionRunner(
        scenario_path,
        rosbag_dir_path,
        t4_dataset_path,
        result_json_path,
        result_archive_path,
        storage,
        evaluation_topics,
        additional_record_topics,
        enable_analysis,
        analysis_max_distance,
        analysis_distance_interval,
    )

    runner.evaluate()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate perception rosbag w/ t4dataset")
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
        required=True,
        help="Storage type for rosbag2 to read and write",
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
        "--enable-analysis",
        default="true",
        help="Enable analysis.",
    )
    parser.add_argument(
        "--analysis-max-distance",
        default="150",
        help="Maximum distance for analysis.",
    )
    parser.add_argument(
        "--analysis-distance-interval",
        default="150",
        help="Distance interval for analysis.",
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
        args.evaluation_detection_topic_regex,
        args.evaluation_tracking_topic_regex,
        args.evaluation_prediction_topic_regex,
        args.enable_analysis,
        args.analysis_max_distance,
        args.analysis_distance_interval,
    )


if __name__ == "__main__":
    main()
