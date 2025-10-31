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

from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
import itertools
from pathlib import Path
import shutil
from typing import Any

from rclpy.clock import Clock
from rosbag2_py import TopicMetadata
from std_msgs.msg import Header
from typing_extensions import Self

from driving_log_replayer_v2.post_process.evaluation_manager import ManagerType
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.ros2_utils import RosBagManager
from driving_log_replayer_v2.result import ResultBaseType
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.scenario import load_condition
from driving_log_replayer_v2.scenario import load_scenario_with_exception
from driving_log_replayer_v2.scenario import ScenarioType


@dataclass(frozen=True, slots=True)
class TopicInfo:
    """Information of topic to be recorded from external source."""

    name: str
    msg_type: str


@dataclass(frozen=True, slots=True)
class UseCaseInfo:
    """Information of use case for evaluation."""

    evaluation_manager_class: (
        ManagerType | None
    )  # optional for only using EvaluationItem and ResultBase
    result_class: ResultBaseType
    name: str
    evaluation_topics_with_task: dict[
        str, list[str]
    ]  # If task is unnecessary, use dummy task name like {"dummy_task": [topic, ...]}
    result_json_path: str


@dataclass(frozen=True, slots=True)
class UseCase:
    """Container of evaluation manager, result, and result writer for each use case."""

    evaluation_manager: ManagerType | None  # optional for only using EvaluationItem and ResultBase
    result: ResultBaseType
    result_writer: ResultWriter
    evaluation_topics: list[str]  # only using for easy access


class UseCaseDict(dict[str, UseCase]):
    """Read-only dictionary of use cases with use case name and topic as key."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._topic_to_key: dict[str, str] = {}
        for key, use_case in self.items():
            for topic in use_case.evaluation_topics:
                if topic in self._topic_to_key:
                    err_msg = f"Duplicate topic '{topic}' found in use cases."
                    raise KeyError(err_msg)
                self._topic_to_key[topic] = key

    def __getitem__(self, key: str) -> UseCase:
        # search by use case name first
        if key in self:
            return super().__getitem__(key)
        # search by topic name next
        if key in self._topic_to_key:
            use_case_key = self._topic_to_key[key]
            return super().__getitem__(use_case_key)
        # if not found in either, raise KeyError
        raise KeyError(key)

    def __setitem__(self, _: str, __: UseCase) -> None:
        err_msg = "UseCaseDict is read-only: modification not allowed after initialization."
        raise RuntimeError(err_msg)

    def __delitem__(self, _: str) -> None:
        err_msg = "UseCaseDict is read-only: deletion not allowed after initialization."
        raise RuntimeError(err_msg)

    def clear(self) -> None:
        err_msg = "UseCaseDict is read-only: clear() not allowed."
        raise RuntimeError(err_msg)

    def pop(self, *_: Any, **__: Any) -> None:
        err_msg = "UseCaseDict is read-only: pop() not allowed."
        raise RuntimeError(err_msg)

    def popitem(self) -> None:
        err_msg = "UseCaseDict is read-only: popitem() not allowed."
        raise RuntimeError(err_msg)

    def update(self, *_: Any, **__: Any) -> None:
        err_msg = "UseCaseDict is read-only: update() not allowed."
        raise RuntimeError(err_msg)

    def setdefault(self, *_: Any, **__: Any) -> None:
        err_msg = "UseCaseDict is read-only: setdefault() not allowed."
        raise RuntimeError(err_msg)

    def __ior__(self, _: Any) -> Self:
        err_msg = "UseCaseDict is read-only: '|=' not allowed."
        raise RuntimeError(err_msg)


class Runner(ABC):
    """
    Base class for driving log replayer post-process runner.

    Responsible for following items:
        - loading scenario and condition
        - initializing subclass of EvaluationManager, ResultBase, and ResultWriter for each use case
        - reading rosbag frame by frame
        - evaluating by calling subclass of EvaluationManager
        - writing evaluation result as jsonl
        - performing analysis on the evaluation results
        - closing all resources
    """

    def __init__(
        self,
        scenario_class: ScenarioType,
        use_case_info_list: list[UseCaseInfo],
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_json_path: str,
        result_archive_path: str,
        storage: str,
        external_record_topics: list[TopicInfo],
        enable_analysis: str,
    ) -> None:
        # instance variables
        self._enable_analysis = enable_analysis
        self._use_cases: UseCaseDict[str, UseCase]
        self._degradation_topics: list[str]
        self._rosbag_manager: RosBagManager

        # load scenario and condition
        scenario = load_scenario_with_exception(
            scenario_path,
            scenario_class,
            result_json_path,
        )
        evaluation_condition = load_condition(scenario)

        # initialize evaluation manager, result, and result writer for each use case
        temp_use_cases = {}  # temporary mutable dict
        for use_case_info in use_case_info_list:
            evaluation_topics = [
                topic
                for topics in use_case_info.evaluation_topics_with_task.values()
                for topic in topics
            ]
            temp_use_cases[use_case_info.name] = UseCase(
                evaluation_manager=use_case_info.evaluation_manager_class(
                    scenario,
                    t4_dataset_path,
                    result_archive_path,
                    use_case_info.evaluation_topics_with_task,
                )
                if use_case_info.evaluation_manager_class is not None
                else None,
                result=use_case_info.result_class(evaluation_condition),
                result_writer=ResultWriter(
                    use_case_info.result_json_path, Clock(), evaluation_condition
                ),
                evaluation_topics=evaluation_topics,
            )
        self._use_cases = UseCaseDict(
            temp_use_cases
        )  # make it read-only and possible to access by topic name

        # initialize RosBagManager to read and save rosbag and write into it
        external_record_topics_metadata = [
            TopicMetadata(
                name=topic_info.name,
                type=topic_info.msg_type,
                serialization_format="cdr",
                offered_qos_profiles="",
            )
            for topic_info in external_record_topics
        ]
        evaluation_topics = [
            use_case.evaluation_manager.get_evaluation_topics()
            for use_case in self._use_cases.values()
            if use_case.evaluation_manager is not None
        ]
        evaluation_topics = list(itertools.chain.from_iterable(evaluation_topics))  # flatten list
        self._rosbag_manager = RosBagManager(
            rosbag_dir_path,
            Path(result_archive_path).joinpath("result_bag").as_posix(),
            storage,
            evaluation_topics,
            external_record_topics_metadata,
        )

        # set degradation topics
        self._degradation_topics = [
            use_case.evaluation_manager.get_degradation_topic()
            for use_case in self._use_cases.values()
            if use_case.evaluation_manager is not None
        ]

        # save scenario.yaml to check later
        shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    def evaluate(self) -> None:
        """Evaluate rosbag frame by frame."""
        for topic_name, msg, subscribed_timestamp_nanosec in self._rosbag_manager.read_messages():
            # See RosBagManager for `time relationships`.
            frame_result = self._evaluate_frame(topic_name, msg, subscribed_timestamp_nanosec)
            if topic_name in self._degradation_topics:
                self._write_result(frame_result, msg.header, subscribed_timestamp_nanosec)
        self._evaluate_on_post_process()
        self._close()
        if self._enable_analysis == "true":
            self._analysis()

    def _evaluate_frame(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> FrameResult:
        """
        Evaluate a single frame.

        Args:
            topic_name (str): Topic name of the message.
            msg (Any): The ROS message to be evaluated.
            subscribed_timestamp_nanosec (int): The timestamp when the message was subscribed, in nanoseconds.

        Returns:
            FrameResult: The result of the evaluation for the frame.

        """
        header_timestamp, subscribed_timestamp, data = self._convert_ros_msg_to_data(
            topic_name, msg, subscribed_timestamp_nanosec
        )

        evaluation_manager = self._use_cases[topic_name].evaluation_manager
        if evaluation_manager is not None:
            return evaluation_manager.evaluate_frame(
                topic_name, header_timestamp, subscribed_timestamp, data
            )
        return FrameResult(is_valid=True, data=data, skip_counter=0)

    @abstractmethod
    def _convert_ros_msg_to_data(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> tuple[int, int, Any]:
        """
        Convert a ROS message to PerceptionEvalData.

        Args:
            topic_name (str): Topic name of the message.
            msg (Any): The ROS message to be converted.
            subscribed_timestamp_nanosec (int): The timestamp when the message was subscribed, in nanoseconds.

        Returns:
            tuple[int, int, Any]: A tuple containing the header timestamp, subscribed timestamp, and converted data. Time unit is not specified.

        """
        raise NotImplementedError

    @abstractmethod
    def _write_result(
        self,
        frame_result: FrameResult,
        header: Header,
        subscribed_timestamp_nanosec: int,
    ) -> None:
        """
        Write evaluation result for a single frame.

        Args:
            frame_result (FrameResult): The result of the evaluation for the frame.
            header (Header): The header of the ROS message.
            subscribed_timestamp_nanosec (int): The timestamp when the message was subscribed, in nanoseconds.

        """
        raise NotImplementedError

    @abstractmethod
    def _evaluate_on_post_process(self) -> None:
        """Evaluate after processing all frames."""
        raise NotImplementedError

    @abstractmethod
    def _analysis(self) -> None:
        """Perform analysis on the evaluation results."""
        raise NotImplementedError

    def _close(self) -> None:
        """Close all resources."""
        for use_case in self._use_cases.values():
            use_case.result_writer.close()
        self._rosbag_manager.close_writer()
