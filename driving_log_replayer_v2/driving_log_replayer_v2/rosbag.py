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
from rosbag2_py import Info
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import StorageFilter
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message


class RosbagReader:
    def __init__(self, bag_dir: str, topic_list: list[str]) -> None:
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
        self._topic_list = topic_list
        self._topic_name2type = {}
        for topic in self._reader.get_all_topics_and_types():
            if topic.name in topic_list:
                self._topic_name2type[topic.name] = get_message(topic.type)

    def get_topic_name2type(self) -> dict[str, Any]:
        return self._topic_name2type

    def read_first_messages(self) -> list[tuple[str, Any, int]]:
        """Return the first message per topic in topic_list."""
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


def _detect_storage_type(bag_path: str) -> str:
    mcap_files = list(Path(bag_path).glob("*.mcap"))
    db3_files = list(Path(bag_path).glob("*.db3"))
    if mcap_files and db3_files:
        err_msg = f"Both mcap and sqlite3 files exist in the rosbag directory: {bag_path}"
        raise RuntimeError(err_msg)
    if mcap_files:
        return "mcap"
    if db3_files:
        return "sqlite3"
    err_msg = f"No rosbag files found in the directory: {bag_path}"
    raise RuntimeError(err_msg)


def _metadata_parsable(bag_path: str, storage_type: str) -> bool:
    """Return True if the existing metadata.yaml can be parsed by the local rosbag2."""
    try:
        Info().read_metadata(Path(bag_path).as_posix(), storage_type)
    except RuntimeError:
        return False
    return True


def create_metadata_yaml(bag_path: str) -> None:
    """
    Create metadata.yaml for the rosbag in bag_path, regenerating it when incompatible.

    metadata.yaml が無い場合は Reindexer で生成する。既に存在していても、ローカルの
    rosbag2 (Humble) でパースできない場合は再生成する。

    webauto pull で取得した bag は新しい rosbag2 (Jazzy 以降) が書いた metadata.yaml を
    含むことがあり、QoS プロファイルがネスト YAML 形式で格納されている。Humble の
    yaml-cpp はこれをパースできず `bad conversion` で失敗するため、Reindexer で
    互換形式の metadata.yaml に作り直す。

    Args:
        bag_path (str): Path to the directory containing the rosbag files.

    """
    metadata_path = Path(bag_path).joinpath("metadata.yaml")
    storage_type = _detect_storage_type(bag_path)
    if metadata_path.exists():
        if _metadata_parsable(bag_path, storage_type):
            return
        print(
            f"[create_metadata_yaml] existing metadata.yaml is unparsable by local rosbag2 "
            f"(likely written by a newer rosbag2); regenerating via reindex: {bag_path}",
        )
    storage_options = StorageOptions(storage_id=storage_type, uri=Path(bag_path).as_posix())
    Reindexer().reindex(storage_options)
