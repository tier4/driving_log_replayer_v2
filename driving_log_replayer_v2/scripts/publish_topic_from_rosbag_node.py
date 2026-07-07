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

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock

from driving_log_replayer_v2.rosbag import RosbagReader

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class PublishTopicFromRosbagNode(Node):
    def __init__(self) -> None:
        super().__init__("publish_topic_from_rosbag_node")

        self.declare_parameter("input_bag", "")
        self.declare_parameter("publish_topic_from_rosbag", "")

        bag_dir = self.get_parameter("input_bag").get_parameter_value().string_value
        topics_with_comma = (
            self.get_parameter("publish_topic_from_rosbag").get_parameter_value().string_value
        )

        topic_list = topics_with_comma.split(",") if topics_with_comma != "" else []
        topic_list = [topic.strip() for topic in topic_list if topic.strip()]
        if len(topic_list) == 0:
            raise RuntimeError("publish_topic_from_rosbag must specify at least one topic.")

        self._rosbag_reader = RosbagReader(bag_dir, topic_list)
        topic_name2type = self._rosbag_reader.get_topic_name2type()

        self._publisher_map: dict[str, rclpy.publisher.Publisher] = {}
        for topic in topic_list:
            topic_type = topic_name2type.get(topic)
            if topic_type is None:
                self.get_logger().error(f"Topic {topic} not found in the rosbag.")
                continue
            self._publisher_map[topic] = self.create_publisher(topic_type, topic, RELIABLE_QOS)

        self._pending_messages = self._rosbag_reader.read_all_messages()
        self._next_message_index = 0
        self.get_logger().info(
            f"Loaded {len(self._pending_messages)} messages for {len(self._publisher_map)} topics."
        )

        self.create_subscription(Clock, "/clock", self._on_clock, 10)

    def _on_clock(self, msg: Clock) -> None:
        now_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        while self._next_message_index < len(self._pending_messages):
            topic_name, ros_msg, ros_timestamp = self._pending_messages[self._next_message_index]
            if ros_timestamp > now_ns:
                break
            publisher = self._publisher_map.get(topic_name)
            if publisher is not None:
                publisher.publish(ros_msg)
            self._next_message_index += 1


def main() -> None:
    rclpy.init()
    node = PublishTopicFromRosbagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
