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

from typing import TYPE_CHECKING

from autoware_perception_msgs.msg import DetectedObjects
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects
from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.driving_log_replayer_v2.perception.perception.manager import EvaluationManager
from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.perception.ros2_utils import lookup_transform
from driving_log_replayer_v2.perception.ros2_utils import RosBagManager
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
from driving_log_replayer_v2.result import ResultWriter

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped

    from driving_log_replayer_v2.perception.ros2_utils import MsgType


def write_result(
    result_topic: dict[str],
    result: PerceptionResult,
    result_writer: ResultWriter,
    rosbag_manager: RosBagManager,
    msg: MsgType,
    subscribed_ros_timestamp: int,
    frame_result: PerceptionFrameResult | str,
    skip_counter: int,
) -> None:
    """Write result.jsonl and rosbag."""
    # NOTE: In offline evaluation using rosbag with SequentialReader(), messages are processed one-by-one.
    #       So it is impossible to get transform of future unless explicitly set the tf of future in the buffer.
    map_to_baselink: TransformStamped = lookup_transform(
        rosbag_manager.get_tf_buffer(),
        msg.header.stamp,
    )

    if isinstance(frame_result, PerceptionFrameResult):
        # handle when add_frame is success
        result.set_frame(
            frame_result,
            skip_counter,
            map_to_baselink=DLREvaluatorV2.transform_stamped_with_euler_angle(map_to_baselink),
        )

        marker_ground_truth, marker_results = eval_conversions.convert_to_ros_msg(
            frame_result,
            msg.header,
        )

        # write ground truth as ROS message
        rosbag_manager.write_results(
            result_topic["ground_truth"], marker_ground_truth, msg.header.stamp
        )  # ground truth
        rosbag_manager.write_results(
            result_topic["results"], marker_results, msg.header.stamp
        )  # results including evaluation topic and ground truth
    elif isinstance(frame_result, str):
        # handle when add_frame is fail caused by failed object conversion or no ground truth
        if frame_result == "No Ground Truth":
            result.set_info_frame(frame_result, skip_counter)
        elif frame_result == "Invalid Estimated Objects":
            result.set_warn_frame(frame_result, skip_counter)
        else:
            err_msg = f"Unknown add_frame failure: {frame_result}"
            raise TypeError(err_msg)
    else:
        err_msg = f"Unknown frame result: {frame_result}"
        raise TypeError(err_msg)
    result_writer.write_result_with_time(result, subscribed_ros_timestamp)


def process_perception_message(
    msg: MsgType,
    topic_name: str,
    degradation_topic: str,
    result_topic: dict[str],
    subscribed_ros_timestamp: int,
    evaluator: EvaluationManager,
    result: PerceptionResult,
    result_writer: ResultWriter,
    rosbag_manager: RosBagManager,
) -> None:
    """
    Process perception messages (DetectedObjects, TrackedObjects, PredictedObjects).

    Args:
        msg: The perception message
        topic_name: Name of the topic
        unix_timestamp: Unix timestamp in seconds
        subscribed_ros_timestamp: ROS timestamp
        evaluator: Perception evaluator
        result: Perception result object
        result_writer: Writer for results
        rosbag_manager: ROS bag manager
        additional_record_topic_name: Additional topic names for recording
        degradation_topic: Name of the degradation topic

    """
    if isinstance(msg, DetectedObjects):
        interpolation: bool = False
    elif isinstance(msg, TrackedObjects | PredictedObjects):
        interpolation: bool = True
    else:
        err_msg = f"Unknown message type: {type(msg)}"
        raise TypeError(err_msg)

    # convert ros to perception_eval
    header_unix_time, subscribed_unix_time, estimated_objects = eval_conversions.convert_to_perception_eval(
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
            result_topic,
            result,
            result_writer,
            rosbag_manager,
            msg,
            subscribed_ros_timestamp,
            frame_result,
            skip_counter,
        )