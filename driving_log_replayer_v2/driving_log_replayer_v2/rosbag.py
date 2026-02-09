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

from pathlib import Path
from typing import Any

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import StorageFilter
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message


class RosbagReader:
    def __init__(self, bag_dir: str, topic_list: list[str] | None = None) -> None:
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        storage_options = StorageOptions(
            uri=bag_dir,
            storage_id="",  # Automatically detect storage id
        )
        self._reader = SequentialReader()
        self._reader.open(storage_options, converter_options)
        self._topic_list = topic_list if topic_list is not None else []
        self._topic_name2type: dict[str, Any] = {
            topic.name: get_message(topic.type) for topic in self._reader.get_all_topics_and_types()
        }

    def get_topic_name2type(self) -> dict[str, Any]:
        return self._topic_name2type

    def read_first_messages(self) -> list[tuple[str, Any, int]]:
        """Return the first message per topic in topic_list."""
        if not self._topic_list:
            return []
        self._reader.set_filter(StorageFilter(topics=list(self._topic_list)))
        result: list[tuple[str, Any, int]] = []
        while self._reader.has_next():
            topic_name, msg_bytes, ros_timestamp = self._reader.read_next()
            if topic_name in self._topic_list:
                self._topic_list.remove(topic_name)
                msg = deserialize_message(msg_bytes, self._topic_name2type[topic_name])
                result.append((topic_name, msg, ros_timestamp))
            if len(self._topic_list) == 0:
                break
        return result

    def read_last_messages(self) -> list[tuple[str, Any, int]]:
        """Return the last message per topic in topic_list."""
        if not self._topic_list:
            return []
        self._reader.set_filter(StorageFilter(topics=list(self._topic_list)))
        last_msgs: dict[str, tuple[bytes, int]] = {}
        while self._reader.has_next():
            topic_name, msg_bytes, ros_timestamp = self._reader.read_next()
            last_msgs[topic_name] = (msg_bytes, ros_timestamp)
        return [
            (
                topic_name,
                deserialize_message(msg_bytes, self._topic_name2type[topic_name]),
                ros_timestamp,
            )
            for topic_name, (msg_bytes, ros_timestamp) in last_msgs.items()
        ]


def create_metadata_yaml(bag_path: str) -> None:
    """
    Create metadata.yaml for the rosbag in bag_path if it does not exist.

    Args:
        bag_path (str): Path to the directory containing the rosbag files.

    """
    metadata_path = Path(bag_path).joinpath("metadata.yaml")
    if metadata_path.exists():
        return
    mcap_files = list(Path(bag_path).glob("*.mcap"))
    db3_files = list(Path(bag_path).glob("*.db3"))
    if mcap_files and db3_files:
        err_msg = f"Both mcap and sqlite3 files exist in the rosbag directory: {bag_path}"
        raise RuntimeError(err_msg)
    storage_type: str
    if mcap_files:
        storage_type = "mcap"
    elif db3_files:
        storage_type = "sqlite3"
    else:
        err_msg = f"No rosbag files found in the directory: {bag_path}"
        raise RuntimeError(err_msg)
    storage_options = StorageOptions(storage_id=storage_type, uri=Path(bag_path).as_posix())
    Reindexer().reindex(storage_options)
