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

from typing import Any
from typing import TYPE_CHECKING

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message


if TYPE_CHECKING:
    from collections.abc import Generator


class RosBagReader:
    def __init__(self, bag_dir: str, storage_type: str, topic_list: list[str]) -> None:
        converter_options = self._get_default_converter_options()
        storage_options = StorageOptions(
            uri=bag_dir,
            storage_id=storage_type,
        )
        self._reader = SequentialReader()
        self._reader.open(storage_options, converter_options)
        self._topic_list = topic_list
        self._topic_name2type: dict[str, Any] = {
            topic.name: get_message(topic.type) for topic in self._reader.get_all_topics_and_types()
        }

    def _get_default_converter_options(self) -> ConverterOptions:
        return ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

    def get_topic_name2type(self) -> dict[str, Any]:
        return self._topic_name2type

    def read_messages(self) -> Generator[str, Any, int]:
        while self._reader.has_next():
            topic_name, msg_bytes, ros_timestamp = self._reader.read_next()
            if topic_name in self._topic_list:
                self._topic_list.remove(topic_name)
                msg = deserialize_message(msg_bytes, self._topic_name2type[topic_name])
                yield topic_name, msg, ros_timestamp
