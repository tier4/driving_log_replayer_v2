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

import csv
import logging
from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING

# Try to import ROS messages, but don't fail if they're not available
try:
    from tier4_api_msgs.msg import AwapiAutowareStatus

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

    # Create a placeholder for type hints
    class AwapiAutowareStatus:
        pass


if TYPE_CHECKING:
    from rosidl_runtime_py.utilities import message_to_ordereddict  # noqa


class StopReasonEvaluator:
    """Process stop_reason data from /awapi/autoware/get/status topic and convert to spreadsheet format."""

    def __init__(
        self,
        output_path: "Path",
        start_time: float,
        end_time: float,
        min_distance: float,
        max_distance: float,
        tolerance_interval: float,
        evaluation_type: str,
    ) -> None:  # noqa
        """
        Initialize the evaluator.

        Args:
            output_path: Path to save the spreadsheet file

        """
        self.output_path = output_path
        self.stop_reasons_data: list[dict] = []
        self.csv_file_path = output_path / "stop_reasons.csv"

        self.start_time = start_time
        self.end_time = end_time
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.tolerance_interval = tolerance_interval
        self.evaluation_type = evaluation_type

        self.last_accepted_time = -1.0
        self.last_target_reason_time = -1.0
        self.last_check_time = -1.0
        self.passed = 0
        self.total = 0
        self.per_frame_results = []
        self.tn_success_logged = False

    # チェックにはなっている。
    def check_timeout(self, current_time: float) -> dict | None:  #  noqa
        """Check if we've exceeded the tolerance interval without receiving the target stop reason."""
        check_interval = 0.1

        # Only check if we haven't checked recently (avoid spam)
        if current_time - self.last_check_time < check_interval:  # Check at most every 0.1 seconds
            return None

        self.last_check_time = current_time

        if self.evaluation_type == "TP":
            # Only check if we're within the evaluation time window
            if not (self.start_time <= current_time <= self.end_time):
                return None
            # True Positive: Check if we haven't received the target reason for tolerance_interval seconds
            if (
                self.last_target_reason_time >= self.start_time
                and current_time - self.last_target_reason_time >= self.tolerance_interval
            ) or (
                self.last_target_reason_time == -1.0
                and current_time >= self.start_time + self.tolerance_interval
            ):
                self.total += 1
                self.success = self.rate() >= self.pass_rate
                self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

                timeout_result = {
                    "StopReason": {
                        "Result": {"Total": self.success_str(), "Frame": "Fail"},
                        "Info": {
                            "Reason": "TIMEOUT",
                            "Distance": 0.0,
                            "Timestamp": current_time,
                            "EvaluationType": self.evaluation_type,
                            "Message": f"No {self.target_reason} received for {self.tolerance_interval}s (last at {self.last_target_reason_time})",
                            "Passed": self.passed,
                            "Total": self.total,
                        },
                    },
                }
                self.per_frame_results.append(timeout_result)
                return timeout_result
        elif self.evaluation_type == "TN":
            # Only check if we're within the evaluation time window
            if not (self.start_time <= current_time):
                return None
            # True Negative: Check if we've reached the end of evaluation window without receiving the target reason
            if current_time >= self.end_time:
                # Skip if TN_SUCCESS has already been logged
                if self.tn_success_logged:
                    return None

                # For TN, reaching the end without receiving the target reason is a success
                self.total += 1
                self.passed += 1  # Success for TN: no stop event occurred
                self.success = self.rate() >= self.pass_rate
                self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {self.rate():.2f}%"

                timeout_result = {
                    "StopReason": {
                        "Result": {"Total": self.success_str(), "Frame": "Success"},
                        "Info": {
                            "Reason": "TN_SUCCESS",
                            "Distance": 0.0,
                            "Timestamp": current_time,
                            "EvaluationType": self.evaluation_type,
                            "Message": f"No {self.target_reason} received during evaluation window - TN success",
                            "Passed": self.passed,
                            "Total": self.total,
                        },
                    },
                }
                self.per_frame_results.append(timeout_result)
                self.tn_success_logged = True  # Mark as logged to prevent spam
                return timeout_result

        return None



    def process_message(self, msg: Any, timestamp: float) -> None:
        """
        Process a single AwapiAutowareStatus message.

        Args:
            msg: The AwapiAutowareStatus message
            timestamp: Unix timestamp when the message was received

        """
        if not hasattr(msg, "stop_reason") or not hasattr(msg.stop_reason, "stop_reasons"):
            return

        if not msg.stop_reason.stop_reasons:
            return

        logging.info("stop_reason loop begin")

        try:
            for idx, stop_reason in enumerate(msg.stop_reason.stop_reasons):
                reason_data = {
                    "timestamp": timestamp,
                    "idx": idx,
                    "reason": stop_reason.reason,
                    "dist_to_stop_pos": stop_reason.stop_factors[0].dist_to_stop_pose,
                    "x": stop_reason.stop_factors[0].stop_pose.position.x,
                    "y": stop_reason.stop_factors[0].stop_pose.position.y,
                    "z": stop_reason.stop_factors[0].stop_pose.position.z,
                    "qz": stop_reason.stop_factors[0].stop_pose.orientation.z,
                    "qw": stop_reason.stop_factors[0].stop_pose.orientation.w,
                }
                self.stop_reasons_data.append(reason_data)
        except AttributeError as e:
            error_msg = f"Error processing stop_reason: {e}"
            logging.exception(error_msg)
        finally:
            logging.info("stop_reason loop end")

    def save_to_spreadsheet(self) -> None:
        """Save the collected stop_reason data to a CSV file."""
        if not self.stop_reasons_data:
            return

        # Ensure output directory exists
        self.output_path.mkdir(parents=True, exist_ok=True)
        info_msg = "save_to_spreadsheet begin"
        logging.info(info_msg)

        # Write to CSV
        with Path(self.csv_file_path).open("w", newline="", encoding="utf-8") as csv_file:
            fieldnames = [
                "timestamp",
                "idx",
                "reason",
                "dist_to_stop_pos",
                "x",
                "y",
                "z",
                "qz",
                "qw",
            ]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            writer.writeheader()
            for data in self.stop_reasons_data:
                writer.writerow(data)
        info_msg = f"Stop reasons data saved to: {self.csv_file_path}"
        logging.info(info_msg)
        info_msg = f"Total stop reasons recorded: {len(self.stop_reasons_data)}"
        logging.info(info_msg)
