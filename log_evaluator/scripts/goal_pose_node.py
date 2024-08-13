#!/usr/bin/env python3

# Copyright (c) 2024 TIER IV.inc
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

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from log_evaluator.pose import arg_to_goal_pose


class PoseNode(Node):
    COUNT_SHUTDOWN_NODE = 5

    def __init__(self) -> None:
        super().__init__("goal_pose")
        self.declare_parameter("goal_pose", "")

        self._goal_pose = arg_to_goal_pose(
            self.get_parameter("goal_pose").get_parameter_value().string_value,
        )

        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()
        self._count_pub = 0

        self._pub_goal_pose = self.create_publisher(
            PoseStamped,
            "/planning/mission_planning/goal",
            1,
        )

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self._timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

    def timer_cb(self) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # to debug callback use: self.get_logger().error(f"time: {self._current_time.sec}.{self._current_time.nanosec}")
        if self._current_time.sec <= 0:  # Stop PLAYER after standing for 1 second.
            return
        if self._count_pub > PoseNode.COUNT_SHUTDOWN_NODE:
            rclpy.shutdown()
        self._count_pub += 1
        self.get_logger().info(
            f"pub goal_pose time: {self._current_time.sec}.{self._current_time.nanosec}",
        )
        self._goal_pose.header.stamp = self._current_time
        self._pub_goal_pose.publish(self._goal_pose)
        self._prev_time = self._current_time


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    initial_pose_node = PoseNode()
    executor.add_node(initial_pose_node)
    executor.spin()
    initial_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
