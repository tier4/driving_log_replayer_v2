# Copyright (c) 2023 TIER IV.inc
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

from collections.abc import Callable
from pathlib import Path
from typing import Any

from autoware_perception_msgs.msg import ObjectClassification
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import TransformStamped
from pydantic import ValidationError
from pyquaternion import Quaternion as PyQuaternion
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
import yaml

from driving_log_replayer_v2.result import PickleWriter
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.scenario import load_scenario


class DLREvaluatorV2(Node):
    COUNT_SHUTDOWN_NODE = 5

    def __init__(
        self,
        name: str,
        scenario_class: Callable,
        result_class: Callable,
        result_topic: str,
    ) -> None:
        super().__init__(name)
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_json_path", "")
        self.declare_parameter("result_archive_path", "")
        self.declare_parameter("dataset_index", "")

        self._scenario_path = self.get_parameter("scenario_path").get_parameter_value().string_value
        self._t4_dataset_paths = [
            self.get_parameter("t4_dataset_path").get_parameter_value().string_value,
        ]
        self._result_json_path = (
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )
        self._result_archive_path = Path(
            self.get_parameter("result_archive_path").get_parameter_value().string_value,
        )
        self._dataset_index = (
            self.get_parameter("dataset_index").get_parameter_value().integer_value
        )
        self._result_archive_path.mkdir(exist_ok=True)
        self._perception_eval_log_path = self._result_archive_path.parent.joinpath(
            "perception_eval_log",
        ).as_posix()

        self._scenario = None
        try:
            self._scenario = load_scenario(Path(self._scenario_path), scenario_class)
            evaluation_condition = {}
            if (
                hasattr(self._scenario.Evaluation, "Conditions")
                and self._scenario.Evaluation.Conditions is not None
            ):
                evaluation_condition = self._scenario.Evaluation.Conditions

            self._result_writer = ResultWriter(
                self._result_json_path,
                self.get_clock(),
                evaluation_condition,
            )
            self._result = result_class(evaluation_condition)
            self._pub_result = self.create_publisher(String, result_topic, 1)
        except (
            FileNotFoundError,
            PermissionError,
            yaml.YAMLError,
            ValidationError,
        ) as e:
            self.get_logger().error(f"An error occurred while loading the scenario. {e}")
            self._result_writer = ResultWriter(
                self._result_json_path,
                self.get_clock(),
                {},
            )
            error_dict = {
                "Result": {"Success": False, "Summary": "ScenarioFormatError"},
                "Stamp": {"System": 0.0},
                "Frame": {"ErrorMsg": e.__str__()},
            }
            self._result_writer.write_line(error_dict)
            self._result_writer.close()
            rclpy.shutdown()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._current_time = Time().to_msg()
        self._prev_time = Time().to_msg()
        self._clock_stop_counter = 0

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self._timer_group,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )  # wall timer

    def timer_cb(
        self,
        *,
        register_loop_func: Callable | None = None,
        register_shutdown_func: Callable | None = None,
    ) -> None:
        self._current_time = self.get_clock().now().to_msg()
        # to debug callback use: self.get_logger().error(f"time: {self._current_time.sec}.{self._current_time.nanosec}")
        if self._current_time.sec <= 0:  # Stop PLAYER after standing for 1 second.
            return
        if register_loop_func is not None:
            register_loop_func()
        self._clock_stop_counter = (
            self._clock_stop_counter + 1 if self._current_time == self._prev_time else 0
        )
        self._prev_time = self._current_time
        if self._clock_stop_counter >= DLREvaluatorV2.COUNT_SHUTDOWN_NODE:
            if register_shutdown_func is not None:
                register_shutdown_func()
            self._result_writer.close()
            rclpy.shutdown()

    def lookup_transform(
        self,
        stamp: Stamp,
        from_: str = "map",
        to: str = "base_link",
    ) -> TransformStamped:
        try:
            return self._tf_buffer.lookup_transform(
                from_,
                to,
                stamp,
                Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            return TransformStamped()

    def save_pkl(self, save_object: Any) -> None:
        PickleWriter(
            self._result_archive_path.joinpath("scene_result.pkl").as_posix(),
            save_object,
        )

    @classmethod
    def transform_stamped_with_euler_angle(cls, transform_stamped: TransformStamped) -> dict:
        tf_euler = message_to_ordereddict(transform_stamped)
        yaw, pitch, roll = PyQuaternion(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
        ).yaw_pitch_roll
        tf_euler["rotation_euler"] = {"roll": roll, "pitch": pitch, "yaw": yaw}
        return tf_euler

    @classmethod
    def get_most_probable_classification(
        cls,
        array_classification: list[ObjectClassification],
    ) -> ObjectClassification:
        index: int = array_classification.index(
            max(array_classification, key=lambda x: x.probability),
        )
        return array_classification[index]


def evaluator_main(func: Callable) -> Callable:
    def wrapper() -> None:
        rclpy.init()
        executor = MultiThreadedExecutor()
        evaluator = func()
        executor.add_node(evaluator)
        executor.spin()
        evaluator.destroy_node()
        rclpy.shutdown()

    return wrapper
