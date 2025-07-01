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
from typing import TypeVar

from autoware_perception_msgs.msg import DetectedObjects
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from tf2_ros import Duration
from tf2_ros import TransformException
from tier4_api_msgs.msg import AwapiAutowareStatus

if TYPE_CHECKING:
    from collections.abc import Generator

    from builtin_interfaces.msg import Time as Stamp

MsgType = TypeVar("MsgType", DetectedObjects, TrackedObjects, PredictedObjects, AwapiAutowareStatus)


class RosBagManager:
    def __init__(
        self,
        bag_dir: str,
        output_bag_dir: str,
        storage_type: str,
        evaluation_topic: list[str],
        additional_record_topic: list[TopicMetadata],
    ) -> None:
        self._reader: SequentialReader
        self._writer: SequentialWriter
        self._writer_storage_options: StorageOptions
        self._evaluate_topic: list[str]
        self._topic_name2type: dict[str, str] = {}
        self._last_ros_timestamp: int
        self._tf_buffer: Buffer

        converter_options = self._get_default_converter_options()

        # rosbag reader
        storage_options = self._get_storage_options(bag_dir, storage_type)
        self._reader = SequentialReader()
        self._reader.open(storage_options, converter_options)

        # rosbag writer
        self._writer_storage_options = self._get_storage_options(output_bag_dir, storage_type)
        self._writer = SequentialWriter()
        self._writer.open(self._writer_storage_options, converter_options)

        # writer preparation
        for topic_type in self._reader.get_all_topics_and_types():
            self._writer.create_topic(topic_type)
            self._topic_name2type[topic_type.name] = topic_type.type
        for topic_type in additional_record_topic:
            self._writer.create_topic(topic_type)

        self._evaluate_topic = evaluation_topic
        self._tf_buffer = Buffer()

    def _get_default_converter_options(self) -> ConverterOptions:
        return ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

    def _get_storage_options(self, uri: str, storage_type: str) -> StorageOptions:
        return StorageOptions(
            uri=uri,
            storage_id=storage_type,
        )

    def __del__(self) -> None:
        if hasattr(self, "_reader"):
            del self._reader
        if hasattr(self, "_writer"):
            del self._writer

    def get_tf_buffer(self) -> Buffer:
        return self._tf_buffer

    def get_last_ros_timestamp(self) -> int:
        if hasattr(self, "_last_ros_timestamp"):
            return self._last_ros_timestamp
        err_msg = "Last ros timestamp is not set."
        raise AttributeError(err_msg)

    def read_messages(self) -> Generator[str, MsgType, int]:
        """
        Describe time representations used in subscribed ROS 2 messages.

        msg.header.stamp     | builtin_interfaces.msg.Time     | base     | base
        subscribed timestamp | int                             | nanosec  | sec * 1e9 + nanosec
        clock                | rclpy.clock.Clock (nanoseconds) | nanosec  | sec * 1e9 + nanosec
        unix time            | int                             | microsec | sec * 1e6 + nanosec / 1e3
        """
        while self._reader.has_next():
            topic_name, msg_bytes, ros_timestamp = self._reader.read_next()
            self._writer.write(topic_name, msg_bytes, ros_timestamp)
            msg = deserialize_message(msg_bytes, get_message(self._topic_name2type[topic_name]))

            if topic_name == "/tf_static":
                for transform in msg.transforms:
                    self._tf_buffer.set_transform_static(transform, "rosbag_import")
            elif topic_name == "/tf":
                for transform in msg.transforms:
                    self._tf_buffer.set_transform(transform, "rosbag_import")
            elif topic_name in self._evaluate_topic:
                yield topic_name, msg, ros_timestamp
            elif topic_name == "/awapi/autoware/get/status":
                yield topic_name, msg, ros_timestamp
        self._last_ros_timestamp = ros_timestamp
        del self._reader

    def write_results(self, topic_name: str, msg: Any, timestamp: Time | int) -> None:
        if isinstance(timestamp, Time):
            ros_timestamp = timestamp.sec * pow(10, 9) + timestamp.nanosec
        elif isinstance(timestamp, int):
            ros_timestamp = timestamp
        else:
            err_msg = f"Invalid type: {type(timestamp)}"
            raise TypeError(err_msg)
        msg_bytes = serialize_message(msg)
        self._writer.write(topic_name, msg_bytes, ros_timestamp)

    def close_writer(self) -> None:
        del self._writer
        Reindexer().reindex(self._writer_storage_options)


def lookup_transform(
    tf_buffer: Buffer,
    stamp: Stamp,
    from_: str = "map",
    to: str = "base_link",
) -> TransformStamped | str:
    try:
        return tf_buffer.lookup_transform(
            from_,
            to,
            stamp,
            Duration(seconds=0.0),
        )
    except TransformException:
        return TransformStamped()
