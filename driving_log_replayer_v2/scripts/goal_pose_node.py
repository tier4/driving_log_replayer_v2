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

from autoware_adapi_v1_msgs.srv import ClearRoute
from autoware_adapi_v1_msgs.srv import SetRoutePoints
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Header
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState

from driving_log_replayer_v2.pose import arg_to_goal_pose

if TYPE_CHECKING:
    from autoware_adapi_v1_msgs.msg import ResponseStatus


class PoseNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose")
        self.declare_parameter("goal_pose", "")
        self._goal_pose_str = self.get_parameter("goal_pose").get_parameter_value().string_value

        if self._goal_pose_str == "{}":
            rclpy.shutdown()

        self._goal_pose_running: bool = False
        self._clear_route_success: bool = False
        self._route_set_success: bool = False
        self._goal_pose = arg_to_goal_pose(self._goal_pose_str)

        self._clear_route_client = self.create_client(
            ClearRoute,
            "/api/routing/clear_route",
        )

        self._goal_pose_client = self.create_client(
            SetRoutePoints,
            "/api/routing/set_route_points",
        )

        while not self._clear_route_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("goal pose service not available, waiting again...")

        while not self._goal_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("goal pose service not available, waiting again...")

        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self._timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

        self._localization_pub = self.create_publisher(
            LocalizationInitializationState,
            #"/api/localization/initialization_state",
            "/localization/initialization_state",
            1,
        )

    def timer_cb(self) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # to debug callback use: self.get_logger().error(f"time: {self._current_time.sec}.{self._current_time.nanosec}")
        if self._current_time.sec <= 0:  # Stop PLAYER after standing for 1 second.
            return
        
        self._localization_pub.publish(
            LocalizationInitializationState(
                stamp=self._current_time,
                state=LocalizationInitializationState.INITIALIZED,
            )
        )
        self.get_logger().info(f"localization state published at {self._current_time.sec}")
        if not self._route_set_success:
            self.call_goal_pose_service()
        self._prev_time = self._current_time

    def call_goal_pose_service(self) -> None:
        if self._goal_pose_running:
            return
        self.get_logger().info(
            f"call goal_pose time: {self._current_time.sec}.{self._current_time.nanosec}",
        )
        self._goal_pose_running = True
        if self._clear_route_success:
            future_goal_pose = self._goal_pose_client.call_async(
                SetRoutePoints.Request(
                    header=Header(stamp=self._current_time, frame_id="map"),
                    goal=self._goal_pose,
                    waypoints=[],
                ),
            )
            future_goal_pose.add_done_callback(self.goal_pose_cb)
        else:
            # clear route and then call goal pose
            future_clear_route = self._clear_route_client.call_async(
                ClearRoute.Request(),
            )
            future_clear_route.add_done_callback(self.clear_route_cb)

    def goal_pose_cb(self, future: Future) -> None:
        result: SetRoutePoints.Response | None = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            self.get_logger().info(
                f"{res_status.success=}",
            )  # debug msg
            if res_status.success:
                #rclpy.shutdown()
                self._route_set_success = True
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")
        # free self._goal_pose_running
        self._goal_pose_running = False

    def clear_route_cb(self, future: Future) -> None:
        result: ClearRoute.Response | None = future.result()
        if result is not None:
            if result.status.success:
                self._clear_route_success = True
                future_goal_pose = self._goal_pose_client.call_async(
                    SetRoutePoints.Request(
                        header=Header(stamp=self._current_time, frame_id="map"),
                        goal=self._goal_pose,
                        waypoints=[],
                    ),
                )
                future_goal_pose.add_done_callback(self.goal_pose_cb)
            else:
                self._goal_pose_running = False
                self.get_logger().warn("clear route service result is fail")
        else:
            self._goal_pose_running = False
            self.get_logger().error(f"Exception for service: {future.exception()}")


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    goal_pose_node = PoseNode()
    executor.add_node(goal_pose_node)
    executor.spin()
    goal_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
