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

# ruff: noqa: SLF001

from pathlib import Path
import sys
from unittest.mock import Mock

from autoware_system_msgs.msg import AutowareState
import pytest
from rclpy.task import Future
from rclpy.time import Time
from tier4_system_msgs.srv import ResetDiagGraph

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from engage_node import EngageNode  # noqa: E402
from engage_node import rclpy  # noqa: E402


@pytest.fixture
def node(monkeypatch: pytest.MonkeyPatch) -> EngageNode:
    # Exercise callbacks without starting DDS or calling a running Autoware instance.
    engage_node = object.__new__(EngageNode)
    engage_node._timeout_s = 80.0
    engage_node._start_time = Time(seconds=100)
    engage_node._current_state = None
    engage_node._engage_running = False
    engage_node._reset_delay_s = 20.0
    engage_node._reset_start_time = None
    engage_node._reset_running = False
    engage_node._reset_complete = False
    engage_node._reset_clock = Mock()
    engage_node._reset_clock.now.return_value = Time(seconds=100)
    engage_node.get_clock = Mock()
    engage_node.get_clock.return_value.now.return_value = Time(seconds=100)
    engage_node.get_logger = Mock()
    engage_node._reset_client = Mock()
    engage_node._reset_client.service_is_ready.return_value = True
    engage_node._reset_client.call_async.return_value = Future()
    engage_node._engage_client = Mock()
    engage_node._engage_client.call_async.return_value = Future()
    monkeypatch.setattr(rclpy, "shutdown", Mock())
    return engage_node


@pytest.mark.parametrize(
    "first_state",
    [AutowareState.WAITING_FOR_ROUTE, AutowareState.PLANNING, AutowareState.WAITING_FOR_ENGAGE],
)
def test_reset_once_before_engagement(node: EngageNode, first_state: int) -> None:
    node.state_callback(AutowareState(state=first_state))
    node._reset_clock.now.return_value = Time(seconds=119)
    node.state_callback(AutowareState(state=AutowareState.WAITING_FOR_ENGAGE))
    node.timer_cb()
    node.call_engage_service()
    node._reset_client.call_async.assert_not_called()
    node._engage_client.call_async.assert_not_called()

    node._reset_clock.now.return_value = Time(seconds=120)
    node.timer_cb()
    node.timer_cb()
    node._reset_client.call_async.assert_called_once()
    node._engage_client.call_async.assert_not_called()

    response = ResetDiagGraph.Response()
    response.status.success = True
    node._reset_client.call_async.return_value.set_result(response)
    node.timer_cb()
    node.timer_cb()
    node._engage_client.call_async.assert_called_once()
    node._reset_client.call_async.assert_called_once()
    rclpy.shutdown.assert_not_called()


def test_initializing_does_not_start_reset_delay(node: EngageNode) -> None:
    node.state_callback(AutowareState(state=AutowareState.INITIALIZING))
    node._reset_clock.now.return_value = Time(seconds=125)
    node.timer_cb()
    assert node._reset_start_time is None
    node._reset_client.call_async.assert_not_called()
    node._engage_client.call_async.assert_not_called()


def test_wait_for_reset_service_without_blocking(node: EngageNode) -> None:
    node.state_callback(AutowareState(state=AutowareState.WAITING_FOR_ENGAGE))
    node._reset_clock.now.return_value = Time(seconds=120)
    node._reset_client.service_is_ready.return_value = False
    node.timer_cb()
    node._reset_client.call_async.assert_not_called()
    node._engage_client.call_async.assert_not_called()

    node._reset_client.service_is_ready.return_value = True
    node.timer_cb()
    node._reset_client.call_async.assert_called_once()


@pytest.mark.parametrize("failure", ["rejected", "exception", "no_response"])
def test_failed_reset_stops_without_engaging(node: EngageNode, failure: str) -> None:
    node.state_callback(AutowareState(state=AutowareState.WAITING_FOR_ENGAGE))
    node._reset_clock.now.return_value = Time(seconds=120)
    node.timer_cb()
    future = node._reset_client.call_async.return_value
    if failure == "exception":
        future.set_exception(RuntimeError("Reset unavailable"))
    elif failure == "no_response":
        future.set_result(None)
    else:
        response = ResetDiagGraph.Response()
        response.status.message = "Reset rejected"
        future.set_result(response)
    node.timer_cb()
    assert not node._reset_complete
    node._engage_client.call_async.assert_not_called()
    node._reset_client.call_async.assert_called_once()
    rclpy.shutdown.assert_called_once()


def test_no_reset_after_driving(node: EngageNode) -> None:
    node.state_callback(AutowareState(state=AutowareState.WAITING_FOR_ROUTE))
    node._reset_clock.now.return_value = Time(seconds=120)
    node.state_callback(AutowareState(state=AutowareState.DRIVING))
    node.timer_cb()
    node._reset_client.call_async.assert_not_called()
    node._engage_client.call_async.assert_not_called()
    rclpy.shutdown.assert_called_once()


def test_timeout_still_applies_while_waiting_for_reset(node: EngageNode) -> None:
    node.state_callback(AutowareState(state=AutowareState.WAITING_FOR_ENGAGE))
    node.get_clock.return_value.now.return_value = Time(seconds=180)
    node._reset_clock.now.return_value = Time(seconds=180)
    node.timer_cb()
    node._reset_client.call_async.assert_not_called()
    node._engage_client.call_async.assert_not_called()
    rclpy.shutdown.assert_called_once()
