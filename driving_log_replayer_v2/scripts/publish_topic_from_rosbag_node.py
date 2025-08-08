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
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from driving_log_replayer_v2.publish_topic_from_rosbag import RosbagReader


class PublishTopicFromRosbagNode(Node):
    def __init__(self) -> None:
        super().__init__("publish_topic_from_rosbag_node")

        self.declare_parameter("input_bag", "")
        self.declare_parameter("storage_type", "")
        self.declare_parameter("publish_topic_from_rosbag", "")

        bag_dir = self.get_parameter("input_bag").get_parameter_value().string_value
        storage_type = self.get_parameter("storage_type").get_parameter_value().string_value
        topics_with_comma = (
            self.get_parameter("publish_topic_from_rosbag").get_parameter_value().string_value
        )

        # load the topic to publish
        topic_list = topics_with_comma.split(",") if topics_with_comma != "" else []
        if len(topic_list) == 0:
            rclpy.shutdown()

        # load the rosbag
        self._rosbag_reader = RosbagReader(bag_dir, storage_type, topic_list)
        topic_name2type = self._rosbag_reader.get_topic_name2type()

        # create the publisher
        self._publisher_map: dict[str, rclpy.publisher.Publisher] = {}
        for topic in topic_list:
            topic_type = topic_name2type.get(topic)
            if topic_type is not None:
                self._publisher_map[topic] = self.create_publisher(topic_type, topic, 10)
            else:
                self.get_logger().error(f"Topic {topic} not found in the rosbag.")

        # create timer
        self.create_timer(10, self.publish)

    def publish(self) -> None:
        for topic_name, msg, _ in self._rosbag_reader.read_messages():
            self._publisher_map[topic_name].publish(msg)
            self._clock.sleep_for(
                Duration(0.1)
            )  # sleep to wait for Autoware to process the message
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    publish_topic_from_rosbag_node = PublishTopicFromRosbagNode()
    executor.add_node(publish_topic_from_rosbag_node)
    executor.spin()
    publish_topic_from_rosbag_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
