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

from typing import TYPE_CHECKING

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from tier4_localization_msgs.srv import InitializeLocalization
from tier4_localization_msgs.srv import PoseWithCovarianceStamped as PoseWithCovarianceStampedSrv

from log_evaluator.pose import arg_to_initial_pose

if TYPE_CHECKING:
    from autoware_common_msgs.msg import ResponseStatus


class PoseNode(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose")
        self.declare_parameter("initial_pose", "")
        self.declare_parameter("direct_initial_pose", "")

        self._initial_pose_str = (
            self.get_parameter("initial_pose").get_parameter_value().string_value
        )
        self._direct_initial_pose_str = (
            self.get_parameter("direct_initial_pose").get_parameter_value().string_value
        )
        """
        self.get_logger().error(f"{type(self._initial_pose_str)=}")
        self.get_logger().error(f"{self._initial_pose_str=}")
        self.get_logger().error(f"{type(self._direct_initial_pose_str)=}")
        self.get_logger().error(f"{self._direct_initial_pose_str=}")
        """

        if self._initial_pose_str == "" and self._direct_initial_pose_str == "":
            rclpy.shutdown()

        # initial pose estimation
        self._initial_pose_running: bool = False
        self._initial_pose_success: bool = False
        if self._initial_pose_str != "":
            self._initial_pose = arg_to_initial_pose(self._initial_pose_str)
            self._initial_pose_method: int = InitializeLocalization.Request.AUTO
        if self._direct_initial_pose_str != "":
            self._initial_pose = arg_to_initial_pose(self._direct_initial_pose_str)
            self._initial_pose_method: int = InitializeLocalization.Request.DIRECT

        # The service must be up and running beforehand.
        # If you try to start this node by itself with ros2 run, it will not work.
        self._initial_pose_client = self.create_client(
            InitializeLocalization,
            "/localization/initialize",
        )
        while not self._initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("initial pose service not available, waiting again...")

        if self._initial_pose_method == InitializeLocalization.Request.AUTO:
            self._map_fit_client = self.create_client(
                PoseWithCovarianceStampedSrv,
                "/map/map_height_fitter/service",
            )
            while not self._map_fit_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning(
                    "map height fitter service not available, waiting again...",
                )

        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()

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
        self.call_initial_pose_service()
        self._prev_time = self._current_time

    def call_initial_pose_service(self) -> None:
        if self._initial_pose_success or self._initial_pose_running:
            return
        self.get_logger().info(
            f"call initial_pose time: {self._current_time.sec}.{self._current_time.nanosec}",
        )
        self._initial_pose_running = True
        self._initial_pose.header.stamp = self._current_time
        if self._initial_pose_method == InitializeLocalization.Request.AUTO:
            future_map_fit = self._map_fit_client.call_async(
                PoseWithCovarianceStampedSrv.Request(pose_with_covariance=self._initial_pose),
            )
            future_map_fit.add_done_callback(self.map_fit_cb)
        else:
            future_direct_init_pose = self._initial_pose_client.call_async(
                InitializeLocalization.Request(
                    method=self._initial_pose_method,
                    pose_with_covariance=[self._initial_pose],
                ),
            )
            future_direct_init_pose.add_done_callback(self.initial_pose_cb)

    def map_fit_cb(self, future: Future) -> None:
        result: PoseWithCovarianceStampedSrv.Response | None = future.result()
        if result is not None:
            if result.success:
                future_init_pose = self._initial_pose_client.call_async(
                    InitializeLocalization.Request(
                        method=self._initial_pose_method,
                        pose_with_covariance=[result.pose_with_covariance],
                    ),
                )
                future_init_pose.add_done_callback(self.initial_pose_cb)
            else:
                # free self._initial_pose_running when the service call fails
                self._initial_pose_running = False
                self.get_logger().warn("map_height_height service result is fail")
        else:
            # free self._initial_pose_running when the service call fails
            self._initial_pose_running = False
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def initial_pose_cb(self, future: Future) -> None:
        result: InitializeLocalization.Response | None = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            self._initial_pose_success = res_status.success
            self.get_logger().info(
                f"{self._initial_pose_success=}",
            )  # debug msg
            if self._initial_pose_success:
                rclpy.shutdown()
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")
        # free self._initial_pose_running
        self._initial_pose_running = False


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
