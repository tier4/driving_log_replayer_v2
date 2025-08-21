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

from tier4_api_msgs.msg import AwapiAutowareStatus

from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.driving_log_replayer_v2.perception.stop_reason.evaluator import StopReasonEvaluator
from driving_log_replayer_v2.result import ResultWriter
import driving_log_replayer_v2.perception_eval_conversions as eval_conversions


def process_stop_reason_message(
    msg: AwapiAutowareStatus,
    subscribed_ros_timestamp: int,
    stop_reason_processor: StopReasonEvaluator,
    result: PerceptionResult,
    result_writer: ResultWriter,
) -> bool:
    """
    Process stop reason data from AwapiAutowareStatus messages.

    Args:
        msg: The AwapiAutowareStatus message
        unix_timestamp: Unix timestamp in seconds
        subscribed_ros_timestamp: ROS timestamp
        stop_reason_processor: Processor for stop reason data
        result: Perception result object
        result_writer: Writer for results

    Returns:
        bool: True if message was processed as stop reason, False otherwise

    """

    unix_timestamp = (
        eval_conversions.unix_time_from_ros_clock_int(subscribed_ros_timestamp) / 1e6
    )  # Convert to seconds


    timeout_results = stop_reason_processor.check_timeout(unix_timestamp)
    if timeout_results:
        result.add_timeout_results_to_frame(timeout_results)
        result_writer.write_result_with_time(result, subscribed_ros_timestamp)

    stop_reason_processor.process_message(msg, unix_timestamp)

    # Process stop reason evaluation if configured
    if hasattr(msg, "stop_reason") and hasattr(msg.stop_reason, "stop_reasons"):
        for stop_reason in msg.stop_reason.stop_reasons:
            if stop_reason.stop_factors:
                # Check if this message should be included in frame results
                should_include_in_frame = False
                for target_reason, evaluation_config in result.stop_reason_evaluations.items():
                    if evaluation_config.start_time <= unix_timestamp <= evaluation_config.end_time:
                        if evaluation_config.evaluation_type == "TP":
                            # True Positive: only include if it matches target reason
                            if stop_reason.reason == target_reason:
                                should_include_in_frame = True
                                break
                        elif evaluation_config.evaluation_type == "TN":
                            # True Negative: include all messages during evaluation window
                            # The evaluation logic will handle whether it's a success or failure
                            should_include_in_frame = True
                            break

                if should_include_in_frame:
                    stop_reason_data = {
                        "timestamp": unix_timestamp,
                        "reason": stop_reason.reason,
                        "dist_to_stop_pos": stop_reason.stop_factors[0].dist_to_stop_pose,
                    }
                    result.set_stop_reason_frame(stop_reason_data)
                    result_writer.write_result_with_time(result, subscribed_ros_timestamp)
        return True
    return False

