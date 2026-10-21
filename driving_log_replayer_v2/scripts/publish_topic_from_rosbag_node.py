#!/usr/bin/env python3

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

from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from driving_log_replayer_v2.rosbag import RosbagReader

# roi_*_fusion camera_info subscribers use best_effort + volatile (see fusion_node.cpp).
# Under Agnocast they only see messages published after the R2A bridge matches, so a
# one-shot transient_local publish that then shuts down never initializes projectors.
CAMERA_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PublishTopicFromRosbagNode(Node):
    TIMER_PERIOD = 1.0

    def __init__(self) -> None:
        super().__init__("publish_topic_from_rosbag_node")

        self.declare_parameter("input_bag", "")
        self.declare_parameter("publish_topic_from_rosbag", "")

        bag_dir = self.get_parameter("input_bag").get_parameter_value().string_value
        topics_with_comma = (
            self.get_parameter("publish_topic_from_rosbag").get_parameter_value().string_value
        )

        # load the topic to publish
        topic_list = topics_with_comma.split(",") if topics_with_comma != "" else []
        if len(topic_list) == 0:
            rclpy.shutdown()
            return

        # load the rosbag
        self._rosbag_reader = RosbagReader(bag_dir, topic_list)
        topic_name2type = self._rosbag_reader.get_topic_name2type()

        # create the publisher
        self._publisher_map: dict[str, rclpy.publisher.Publisher] = {}
        for topic in topic_list:
            topic_type = topic_name2type.get(topic)
            if topic_type is not None:
                self._publisher_map[topic] = self.create_publisher(
                    topic_type, topic, CAMERA_INFO_QOS
                )
            else:
                self.get_logger().warning(
                    f"Topic {topic} not found in the rosbag. Skip publishing."
                )

        # Cache first messages once. read_first_messages() consumes the topic list.
        self._cached_messages: list[tuple[str, Any]] = [
            (topic_name, msg)
            for topic_name, msg, _ in self._rosbag_reader.read_first_messages()
        ]
        if len(self._cached_messages) == 0:
            self.get_logger().warning("No messages found to publish from the rosbag.")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Republishing {len(self._cached_messages)} topic(s) at {self.TIMER_PERIOD} Hz "
            "until shutdown so Agnocast R2A bridges can deliver camera_info."
        )
        self.create_timer(self.TIMER_PERIOD, self.publish)

    def publish(self) -> None:
        for topic_name, msg in self._cached_messages:
            publisher = self._publisher_map.get(topic_name)
            if publisher is not None:
                publisher.publish(msg)


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    publish_topic_from_rosbag_node = PublishTopicFromRosbagNode()
    executor.add_node(publish_topic_from_rosbag_node)
    executor.spin()
    publish_topic_from_rosbag_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
