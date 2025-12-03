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
from rosbag2_py import TopicMetadata
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.perception.analyze import analyze
from driving_log_replayer_v2.perception.evaluation_manager import PerceptionEvaluationManager
from driving_log_replayer_v2.perception.evaluator import PerceptionInvalidReason
from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.perception.planning_factor import convert_to_planning_factor
from driving_log_replayer_v2.perception.planning_factor import PlanningFactorEvalData
from driving_log_replayer_v2.perception.planning_factor import PlanningFactorEvaluationManager
from driving_log_replayer_v2.perception.topics import load_evaluation_topics
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.planning_control import PlanningFactorResult
from driving_log_replayer_v2.post_process.ros2_utils import lookup_transform
from driving_log_replayer_v2.post_process.runner import ConvertedData
from driving_log_replayer_v2.post_process.runner import Runner
from driving_log_replayer_v2.post_process.runner import UseCaseInfo

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped
    from perception_eval.common.object import DynamicObject
    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
    from perception_eval.tool import PerceptionAnalyzer3D

    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.result import ResultWriter


PerceptionMsgType = TypeVar("PerceptionMsgType", DetectedObjects, TrackedObjects, PredictedObjects)


@dataclass(frozen=True, slots=True)
class PerceptionEvalData:
    interpolation: bool
    estimated_objects: list[DynamicObject] | str  # str for error message


def convert_to_perception_eval(
    msg: PerceptionMsgType,
    subscribed_timestamp_nanosec: int,
    evaluation_config: PerceptionEvaluationConfig,
) -> ConvertedData:
    """Convert ROS message to PerceptionEvalData."""
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

    return ConvertedData(
        header_timestamp=header_timestamp_microsec,
        subscribed_timestamp=subscribed_timestamp_microsec,
        data=PerceptionEvalData(interpolation, estimated_objects),
    )


def convert_to_ros_msg(
    frame: PerceptionFrameResult,
    header: Header,
) -> tuple[MarkerArray, MarkerArray]:
    """Convert PerceptionFrameResult to ROS MarkerArray messages."""
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
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        enable_analysis: str,
        analysis_max_distance: str,
        analysis_distance_interval: str,
    ) -> None:
        # additional instance variables
        self._analysis_max_distance = analysis_max_distance
        self._analysis_distance_interval = analysis_distance_interval

        super().__init__(
            PerceptionScenario,
            scenario_path,
            rosbag_dir_path,
            t4_dataset_path,
            result_json_path,
            result_archive_path,
            storage,
            evaluation_topics_with_task,
            degradation_topic,
            enable_analysis,
        )

    def _get_use_case_info_list(
        self,
        scenario: PerceptionScenario,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        result_json_path: str,
        result_archive_path: str,
    ) -> list[UseCaseInfo]:
        use_case_info_list = [
            UseCaseInfo(
                evaluation_manager_class=PerceptionEvaluationManager,
                result_class=PerceptionResult,
                conditions=scenario.Evaluation.Conditions,
                name="perception",
                evaluation_topics_with_task=evaluation_topics_with_task,
                degradation_topic=degradation_topic,
                result_json_path=result_json_path,
            )
        ]

        # add planning_factor to UseCaseInfo if include_use_case is defined
        if scenario.include_use_case is not None:
            pf_conditions = scenario.include_use_case.Conditions.PlanningFactorConditions
            evaluation_topics = [pf_condition.topic for pf_condition in pf_conditions]
            planning_factor_result_json_path = Path(result_archive_path) / "planning_factor.jsonl"
            use_case_info_list.append(
                UseCaseInfo(
                    evaluation_manager_class=PlanningFactorEvaluationManager,
                    result_class=PlanningFactorResult,
                    conditions=pf_conditions,
                    name="planning_factor",
                    evaluation_topics_with_task={"dummy_task": evaluation_topics},
                    degradation_topic="",
                    result_json_path=planning_factor_result_json_path,
                )
            )

        return use_case_info_list

    def _get_external_record_topics(self) -> list[TopicMetadata]:
        return [
            TopicMetadata(
                name="/driving_log_replayer_v2/marker/ground_truth",
                type="visualization_msgs/MarkerArray",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
            TopicMetadata(
                name="/driving_log_replayer_v2/marker/results",
                type="visualization_msgs/MarkerArray",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
            TopicMetadata(
                name="/driving_log_replayer_v2/perception/results",
                type="std_msgs/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
            TopicMetadata(
                name="/driving_log_replayer_v2/planning_factor/results",
                type="std_msgs/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
        ]

    def is_planning_factor(self) -> bool:
        return "planning_factor" in self._use_cases

    @property
    def perc_eval_manager(self) -> PerceptionEvaluationManager:
        return self._use_cases["perception"].evaluation_manager

    @property
    def perc_result(self) -> PerceptionResult:
        return self._use_cases["perception"].result

    @property
    def perc_result_writer(self) -> ResultWriter:
        return self._use_cases["perception"].result_writer

    @property
    def planning_factor_eval_manager(self) -> PlanningFactorEvaluationManager:
        return self._use_cases["planning_factor"].evaluation_manager

    @property
    def planning_factor_result(self) -> PlanningFactorResult:
        return self._use_cases["planning_factor"].result

    @property
    def planning_factor_result_writer(self) -> ResultWriter:
        return self._use_cases["planning_factor"].result_writer

    def _convert_ros_msg_to_data(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> ConvertedData:
        # evaluate planning_factor if defined
        if self.is_planning_factor() and topic_name.startswith("/planning/planning_factors/"):
            return convert_to_planning_factor(msg, subscribed_timestamp_nanosec, topic_name)

        # convert ros to perception_eval
        return convert_to_perception_eval(
            msg,
            subscribed_timestamp_nanosec,
            self.perc_eval_manager.get_evaluation_config(topic_name),
        )

    def _write_result(
        self, frame_result: FrameResult, header: Header, subscribed_timestamp_nanosec: int
    ) -> None:
        if isinstance(frame_result.data, PlanningFactorEvalData):
            # write planning factor result
            self.planning_factor_result.set_frame(
                frame_result.data.msg, frame_result.data.topic_name
            )
            if self.planning_factor_result.frame != {}:
                res_str = self.planning_factor_result_writer.write_result_with_time(
                    self.planning_factor_result, subscribed_timestamp_nanosec
                )
                self._rosbag_manager.write_results(
                    "/driving_log_replayer_v2/planning_factor/results",
                    String(data=res_str),
                    header.stamp,
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
            self.perc_result.set_frame(
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
        # handle when add_frame is fail caused by failed object conversion or no ground truth
        elif frame_result.invalid_reason == PerceptionInvalidReason.NO_GROUND_TRUTH:
            self.perc_result.set_info_frame(frame_result.data, frame_result.skip_counter)
        elif frame_result.invalid_reason == PerceptionInvalidReason.INVALID_ESTIMATED_OBJECTS:
            self.perc_result.set_warn_frame(frame_result.data, frame_result.skip_counter)
        else:
            err_msg = f"Unknown invalid_reason: {frame_result.invalid_reason}"
            raise TypeError(err_msg)

        res_str = self.perc_result_writer.write_result_with_time(
            self.perc_result, subscribed_timestamp_nanosec
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception/results",
            String(data=res_str),
            header.stamp,
        )

    def _evaluate_on_post_process(self) -> None:
        final_metrics: dict[str, dict] = self.perc_eval_manager.get_evaluation_results()

        perception_degradation_topic = self._degradation_topics[
            0
        ]  # head topic is perception degradation topic
        self.perc_result.set_final_metrics(final_metrics[perception_degradation_topic])
        res_str = self.perc_result_writer.write_result_with_time(
            self.perc_result,
            self._rosbag_manager.get_last_subscribed_timestamp(),
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception/results",
            String(data=res_str),
            self._rosbag_manager.get_last_subscribed_timestamp(),
        )

    def _analysis(self) -> None:
        if self.perc_eval_manager.get_degradation_evaluation_task() != "fp_validation":
            analyzers: dict[str, PerceptionAnalyzer3D] = self.perc_eval_manager.get_analyzer()

            # TODO: analysis other topic
            perception_degradation_topic = self._degradation_topics[
                0
            ]  # head topic is perception degradation topic
            analyzer = analyzers[perception_degradation_topic]
            save_path = self.perc_eval_manager.get_archive_path(perception_degradation_topic)
            analyze(
                analyzer,
                save_path,
                self._analysis_max_distance,
                self._analysis_distance_interval,
                perception_degradation_topic,
            )


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
    degradation_topic: str,
    enable_analysis: str,
    analysis_max_distance: str,
    analysis_distance_interval: str,
) -> None:
    evaluation_topics_with_task = load_evaluation_topics(
        evaluation_detection_topic_regex,
        evaluation_tracking_topic_regex,
        evaluation_prediction_topic_regex,
    )

    runner = PerceptionRunner(
        scenario_path,
        rosbag_dir_path,
        t4_dataset_path,
        result_json_path,
        result_archive_path,
        storage,
        evaluation_topics_with_task,
        degradation_topic,
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
        "--degradation-topic",
        default="",
        help="Topic name for degradation information. If you do not want to override the scenario setting, set it to '' or 'None'.",
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
        args.degradation_topic,
        args.enable_analysis,
        args.analysis_max_distance,
        args.analysis_distance_interval,
    )


if __name__ == "__main__":
    main()
