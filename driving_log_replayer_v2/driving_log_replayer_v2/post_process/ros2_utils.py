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

from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np
import pandas as pd
from pyquaternion import Quaternion
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

if TYPE_CHECKING:
    from collections.abc import Generator

    from builtin_interfaces.msg import Time as Stamp
    from nav_msgs.msg import Odometry
    from tier4_metric_msgs.msg import MetricArray

UNIT_MAP = {
    "nanosecond": 1e-9,
    "microsecond": 1e-6,
    "millisecond": 1e-3,
    "second": 1.0,
}


class RosBagManager:
    """
    A class to manage rosbag reading and writing.

    Responsible for following items:
        Reading messages from an input rosbag.
        Writing messages to an output rosbag.
        Managing a tf2 Buffer for transform lookups.
        Storing the latest localization kinematic state.
        Analyzing and processing time via Processing Time Checker.
    """

    TF = "/tf"
    TF_STATIC = "/tf_static"
    LOCALIZATION_KINEMATIC_STATE = "/localization/kinematic_state"
    PROCESSING_TIME_CHECKER_TOPIC = "/system/processing_time_checker/metrics"

    def __init__(
        self,
        bag_dir: str,
        output_bag_dir: str,
        storage_type: str,
        evaluation_topics: list[str],
        external_record_topics: list[TopicMetadata] | None = None,
    ) -> None:
        self._reader: SequentialReader
        self._writer: SequentialWriter
        self._writer_storage_options: StorageOptions
        self._evaluation_topics: list[str]
        self._topic_name2type: dict[str, str] = {}
        self._last_subscribed_timestamp_nanosec: int
        self._tf_buffer: Buffer
        self._localization_kinematic_state: Odometry | None
        self._node_processing_time: dict[
            str, dict[str, str]
        ] = {}  # node_name -> {node_name, header_timestamp}

        converter_options = self._get_default_converter_options()

        # rosbag reader
        storage_options = self._get_storage_options(bag_dir, "")  # Automatically detect storage id
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
        if external_record_topics is not None:
            for topic_type in external_record_topics:
                self._writer.create_topic(topic_type)

        self._evaluation_topics = evaluation_topics
        self._tf_buffer = Buffer()
        self._localization_kinematic_state = None

    def _get_default_converter_options(self) -> ConverterOptions:
        """
        Get default converter options for rosbag reading and writing.

        Returns:
            ConverterOptions: The default converter options.

        """
        return ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

    def _get_storage_options(self, uri: str, storage_type: str) -> StorageOptions:
        """
        Get storage options for rosbag reading and writing.

        Args:
            uri (str): The URI of the storage.
            storage_type (str): The type of the storage.

        Returns:
            StorageOptions: The storage options.

        """
        return StorageOptions(
            uri=uri,
            storage_id=storage_type,
        )

    def __del__(self) -> None:
        """Destructor to clean up resources."""
        if hasattr(self, "_reader"):
            del self._reader
        if hasattr(self, "_writer"):
            del self._writer

    def get_tf_buffer(self) -> Buffer:
        """Get the TF buffer."""
        return self._tf_buffer

    def get_localization_kinematic_state(self) -> Odometry | None:
        """Get the latest localization kinematic state."""
        return self._localization_kinematic_state

    def get_last_subscribed_timestamp(self) -> int:
        """
        Get the last subscribed timestamp in nanoseconds.

        Returns:
            int: The last subscribed timestamp in nanoseconds.

        """
        if hasattr(self, "_last_subscribed_timestamp_nanosec"):
            return self._last_subscribed_timestamp_nanosec
        err_msg = "_last_subscribed_timestamp_nanosec is not set."
        raise AttributeError(err_msg)

    def read_messages(self) -> Generator[str, Any, int]:
        """
        Read messages from the rosbag.

        Yields:
            topic_name (str): The name of the topic.
            msg (Any): The deserialized message.
            subscribed_timestamp_nanosec (int): The subscribed timestamp in nanoseconds.

        Describe time representations.

        | Field                   | Type                             | Unit     | Representation            |
        |-------------------------|----------------------------------|----------|---------------------------|
        | msg.header.stamp        | builtin_interfaces.msg.Time      | none     | sec, nanosec (standard)   |
        | subscribed timestamp    | int                              | nanosec  | sec * 1e9 + nanosec       |
        | Clock.now().nanoseconds | int                              | nanosec  | sec * 1e9 + nanosec       |
        | nuscenes unix time      | int                              | microsec | sec * 1e6 + nanosec / 1e3 |

        """
        subscribed_timestamp_nanosec = 0
        while self._reader.has_next():
            topic_name, msg_bytes, subscribed_timestamp_nanosec = self._reader.read_next()
            self._writer.write(topic_name, msg_bytes, subscribed_timestamp_nanosec)
            msg = deserialize_message(msg_bytes, get_message(self._topic_name2type[topic_name]))

            if topic_name == self.TF_STATIC:
                for transform in msg.transforms:
                    self._tf_buffer.set_transform_static(transform, "rosbag_import")
            elif topic_name == self.TF:
                for transform in msg.transforms:
                    self._tf_buffer.set_transform(transform, "rosbag_import")
            elif topic_name == self.LOCALIZATION_KINEMATIC_STATE:
                self._localization_kinematic_state = msg
            elif topic_name == self.PROCESSING_TIME_CHECKER_TOPIC:
                self._append_processing_time_data(msg)
            elif topic_name in self._evaluation_topics:
                yield topic_name, msg, subscribed_timestamp_nanosec
        self._last_subscribed_timestamp_nanosec = subscribed_timestamp_nanosec
        self._save_node_processing_time()
        del self._reader

    def _append_processing_time_data(self, msg: MetricArray) -> None:
        """
        Append processing time data from the message.

        Args:
            msg (MetricArray): The MetricArray message containing processing time data.

        """
        header_timestamp = f"{msg.stamp.sec}.{msg.stamp.nanosec}"
        for metric in msg.metric_array:
            data = self._node_processing_time.setdefault(metric.name, {})
            data["node_name"] = metric.name
            if metric.unit not in UNIT_MAP:
                err_msg = f"Unsupported unit: {metric.unit}"
                raise ValueError(err_msg)
            data[header_timestamp] = float(metric.value) * UNIT_MAP[metric.unit]

    def _save_node_processing_time(self) -> None:
        """Save the processing time data to a CSV file."""
        data_frame = pd.DataFrame(list(self._node_processing_time.values()))
        if not data_frame.empty:
            timestamp_columns = [col for col in data_frame.columns if col != "node_name"]
            data_frame["average"] = data_frame[timestamp_columns].mean(axis=1)
            data_frame["std"] = data_frame[timestamp_columns].std(axis=1)
            data_frame["max"] = data_frame[timestamp_columns].max(axis=1)
            data_frame["min"] = data_frame[timestamp_columns].min(axis=1)
            data_frame["percentile_99"] = data_frame[timestamp_columns].quantile(0.99, axis=1)
            data_frame["percentile_95"] = data_frame[timestamp_columns].quantile(0.95, axis=1)
        data_frame.to_csv(
            Path(self._writer_storage_options.uri).parent / "processing_time.csv", index=False
        )

    def write_results(self, topic_name: str, msg: Any, subscribed_timestamp: Time | int) -> None:
        """
        Write results to the rosbag.

        Args:
            topic_name (str): The name of the topic.
            msg (Any): The message to write.
            subscribed_timestamp (Time | int): The subscribed timestamp, either as a Time object or an integer nanosecond timestamp.

        """
        if isinstance(subscribed_timestamp, Time):
            subscribed_timestamp_nanosec = (
                subscribed_timestamp.sec * pow(10, 9) + subscribed_timestamp.nanosec
            )
        elif isinstance(subscribed_timestamp, int):
            subscribed_timestamp_nanosec = subscribed_timestamp
        else:
            err_msg = f"Invalid type: {type(subscribed_timestamp)}"
            raise TypeError(err_msg)
        msg_bytes = serialize_message(msg)
        self._writer.write(topic_name, msg_bytes, subscribed_timestamp_nanosec)

    def close_writer(self) -> None:
        """Close the rosbag writer."""
        del self._writer
        Reindexer().reindex(self._writer_storage_options)


def lookup_transform(
    tf_buffer: Buffer,
    stamp: Stamp,
    from_: str = "map",
    to: str = "base_link",
) -> TransformStamped:
    try:
        return tf_buffer.lookup_transform(
            from_,
            to,
            stamp,
            Duration(seconds=0.0),
        )
    except TransformException:
        return TransformStamped()


def convert_to_homogeneous_matrix(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    quaternion = Quaternion(
        rotation.w,
        rotation.x,
        rotation.y,
        rotation.z,
    )

    matrix = np.eye(4)
    matrix[0:3, 3] = [translation.x, translation.y, translation.z]
    matrix[0:3, 0:3] = quaternion.rotation_matrix
    return matrix
