#!/usr/bin/env python3

# Copyright (c) 2026 TIER IV.inc
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

from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_system_msgs.msg import AutowareState
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from tier4_system_msgs.srv import ResetDiagGraph

if TYPE_CHECKING:
    from autoware_common_msgs.msg import ResponseStatus


class EngageNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_engage")

        self.declare_parameter("timeout_s", 80.0)
        self._timeout_s = self.get_parameter("timeout_s").get_parameter_value().double_value
        self.declare_parameter("diagnostic_reset_delay_s", 20.0)
        self._reset_delay_s = (
            self.get_parameter("diagnostic_reset_delay_s").get_parameter_value().double_value
        )

        self._current_state: int | None = None
        self._engage_running: bool = False
        self._reset_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._reset_start_time: Time | None = None
        self._reset_running = False
        self._reset_complete = False

        self._state_subscription = self.create_subscription(
            AutowareState,
            "/autoware/state",
            self.state_callback,
            10,
        )

        self._engage_client = self.create_client(
            ChangeOperationMode,
            "/api/operation_mode/change_to_autonomous",
        )
        self._reset_client = self.create_client(ResetDiagGraph, "/diagnostics_graph/reset")

        while not self._engage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("engage service not available, waiting again...")

        # Initialize start time from node initialization
        self._start_time = self.get_clock().now()
        self._current_time = Time().to_msg()

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self._timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

        self.get_logger().info(
            f"EngageNode initialized with timeout_s={self._timeout_s}, waiting for WaitingForEngage state...",
        )

    def timer_cb(self) -> None:
        """Timer callback to check timeout and call engage service."""
        current_time_obj = self.get_clock().now()
        self._current_time = current_time_obj.to_msg()
        if self._current_time.sec <= 0:
            return

        # Check timeout
        elapsed_time = (current_time_obj - self._start_time).nanoseconds / 1e9
        if elapsed_time >= self._timeout_s:
            self.get_logger().error(
                f"Engage timeout reached ({elapsed_time:.1f}s >= {self._timeout_s}s). Shutting down...",
            )
            rclpy.shutdown()
            return

        if self._current_state not in (
            AutowareState.WAITING_FOR_ROUTE,
            AutowareState.PLANNING,
            AutowareState.WAITING_FOR_ENGAGE,
        ):
            return

        # Clear startup latches once; retain latching during the actual test.
        if not self._reset_complete:
            if self._reset_start_time is not None:
                reset_elapsed = (self._reset_clock.now() - self._reset_start_time).nanoseconds / 1e9
                if reset_elapsed >= self._reset_delay_s:
                    self.call_reset_service()
            return

        # Call engage service if in WAITING_FOR_ENGAGE state
        if self._current_state == AutowareState.WAITING_FOR_ENGAGE:
            self.call_engage_service()

    def state_callback(self, msg: AutowareState) -> None:
        self._current_state = msg.state

        # Later states cover a missed WaitingForRoute message when this node starts late.
        if self._reset_start_time is None and msg.state in (
            AutowareState.WAITING_FOR_ROUTE,
            AutowareState.PLANNING,
            AutowareState.WAITING_FOR_ENGAGE,
        ):
            self._reset_start_time = self._reset_clock.now()
            self.get_logger().info(
                f"Waiting {self._reset_delay_s}s before resetting startup diagnostic latches...",
            )

        if msg.state == AutowareState.DRIVING:
            state_name = self._get_state_name(msg.state)
            self.get_logger().info(
                f"State changed to {state_name}, engage successful! Shutting down...",
            )
            rclpy.shutdown()

    def call_reset_service(self) -> None:
        if self._reset_running or self._reset_complete:
            return
        if not self._reset_client.service_is_ready():
            self.get_logger().warning(
                "Diagnostic reset service not available, waiting...", throttle_duration_sec=5.0
            )
            return
        self._reset_running = True
        self.get_logger().info("Resetting startup diagnostic latches once...")
        future = self._reset_client.call_async(ResetDiagGraph.Request())
        future.add_done_callback(self.reset_callback)

    def reset_callback(self, future: Future) -> None:
        if future.exception() is not None:
            self.get_logger().error(f"Diagnostic reset failed: {future.exception()}")
            rclpy.shutdown()
            return
        result: ResetDiagGraph.Response | None = future.result()
        if result is None or not result.status.success:
            message = result.status.message if result is not None else "No response"
            self.get_logger().error(f"Diagnostic reset failed: {message}. Shutting down...")
            rclpy.shutdown()
            return
        self._reset_complete = True
        self.get_logger().info("Diagnostic reset succeeded; continuing engagement.")

    def call_engage_service(self) -> None:
        if self._engage_running or not self._reset_complete:
            return
        self.get_logger().info(
            f"call engage service time: {self._current_time.sec}.{self._current_time.nanosec}",
        )
        self._engage_running = True
        future = self._engage_client.call_async(ChangeOperationMode.Request())
        future.add_done_callback(self.engage_callback)

    def engage_callback(self, future: Future) -> None:
        result: ChangeOperationMode.Response | None = future.result()
        if result is not None:
            res_status: ResponseStatus = result.status
            if res_status.success:
                self.get_logger().info(
                    "Engage command sent successfully, waiting for state change...",
                )
            else:
                self._engage_running = False
                self.get_logger().warning(
                    f"Engage command failed: {res_status.message} (code: {res_status.code})",
                )
        else:
            self._engage_running = False
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def _get_state_name(self, state: int) -> str:
        state_map = {
            AutowareState.INITIALIZING: "INITIALIZING",
            AutowareState.WAITING_FOR_ROUTE: "WAITING_FOR_ROUTE",
            AutowareState.PLANNING: "PLANNING",
            AutowareState.WAITING_FOR_ENGAGE: "WAITING_FOR_ENGAGE",
            AutowareState.DRIVING: "DRIVING",
            AutowareState.ARRIVED_GOAL: "ARRIVED_GOAL",
            AutowareState.FINALIZING: "FINALIZING",
        }
        return state_map.get(state, f"UNKNOWN({state})")


def main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    engage_node = EngageNode()
    executor.add_node(engage_node)
    executor.spin()
    engage_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
