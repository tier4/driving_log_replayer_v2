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
from pathlib import Path
from typing import TYPE_CHECKING, Any
import logging

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
    from rosidl_runtime_py.utilities import message_to_ordereddict


class StopReasonProcessor:
    """Process stop_reason data from /awapi/autoware/get/status topic and convert to spreadsheet format."""

    def __init__(self, output_path: Path) -> None:
        """Initialize the processor.
        
        Args:
            output_path: Path to save the spreadsheet file
        """
        self.output_path = output_path
        self.stop_reasons_data: list[dict] = []
        self.csv_file_path = output_path / "stop_reasons.csv"
        
    def process_message(self, msg: Any, timestamp: float) -> None:
        """Process a single AwapiAutowareStatus message.
        
        Args:
            msg: The AwapiAutowareStatus message
            timestamp: Unix timestamp when the message was received
        """
        if not hasattr(msg, 'stop_reason') or not hasattr(msg.stop_reason, 'stop_reasons'):
            return
            
        if not msg.stop_reason.stop_reasons:
            return
            
        logging.info("stop_reason loop begin")

        try:
            for idx, stop_reason in enumerate(msg.stop_reason.stop_reasons):
                reason_data = {
                    'timestamp': timestamp,
                    'idx': idx,
                    'reason': stop_reason.reason,
                    'dist_to_stop_pos': stop_reason.stop_factors[0].dist_to_stop_pose,
                    'x': stop_reason.stop_factors[0].stop_pose.position.x,
                    'y': stop_reason.stop_factors[0].stop_pose.position.y,
                    'z': stop_reason.stop_factors[0].stop_pose.position.z,
                    'qz': stop_reason.stop_factors[0].stop_pose.orientation.z,
                    'qw': stop_reason.stop_factors[0].stop_pose.orientation.w,
                }
                self.stop_reasons_data.append(reason_data)
        except AttributeError as e:
            logging.error(f"Error processing stop_reason: {e}")
        finally:
            logging.info("stop_reason loop end")
    
    def save_to_spreadsheet(self) -> None:
        """Save the collected stop_reason data to a CSV file."""
        if not self.stop_reasons_data:
            return
            
        # Ensure output directory exists
        self.output_path.mkdir(parents=True, exist_ok=True)

        logging.info(f"save_to_spreadsheet begin")
        
        # Write to CSV
        with open(self.csv_file_path, 'w', newline='', encoding='utf-8') as csv_file:
            fieldnames = ['timestamp', 'idx', 'reason', 'dist_to_stop_pos', 'x', 'y', 'z', 'qz', 'qw']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            
            writer.writeheader()
            for data in self.stop_reasons_data:
                writer.writerow(data)
        logging.info(f"Stop reasons data saved to: {self.csv_file_path}")
        logging.info(f"Total stop reasons recorded: {len(self.stop_reasons_data)}") 