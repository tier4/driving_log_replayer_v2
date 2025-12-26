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
from driving_log_replayer_v2.result import MultiResultEditor
from driving_log_replayer_v2.result import ResultBaseType
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.scenario import load_scenario_with_exception
from driving_log_replayer_v2.scenario import ScenarioType


@dataclass(frozen=True, slots=True)
class UseCaseInfo:
    """Information of use case for evaluation."""

    evaluation_manager_class: ManagerType
    result_class: ResultBaseType
    conditions: dict  # from scenario
    name: str
    evaluation_topics_with_task: dict[
        str, list[str]
    ]  # If task is unnecessary, use dummy task name like {"dummy_task": [topic, ...]}
    degradation_topic: str
    result_jsonl_path: str


@dataclass(frozen=True, slots=True)
class UseCase:
    """Container of evaluation manager, result, and result writer for each use case."""

    evaluation_manager: ManagerType
    result: ResultBaseType
    result_writer: ResultWriter


@dataclass(frozen=True, slots=True)
class ConvertedData:
    """Converted data for common format."""

    header_timestamp: int  # do not care time unit
    subscribed_timestamp: int  # do not care time unit
    data: Any


class UseCaseDict(dict[str, UseCase]):
    """Read-only dictionary of use cases with use case name and topic as key."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._topic_to_key: dict[str, str] = {}
        for key, use_case in self.items():
            topics = use_case.evaluation_manager.get_evaluation_topics()
            for topic in topics:
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
    Base class for post-process runner.

    Responsible for following items:
        loading scenario
        initializing subclass of EvaluationManager, ResultBase, and ResultWriter for each use case
        reading rosbag frame by frame
        evaluating by calling subclass of EvaluationManager
        writing evaluation result as jsonl
        performing analysis on the evaluation results
        closing all resources
    """

    def __init__(
        self,
        scenario_class: ScenarioType,
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_jsonl_path: str,
        result_archive_path: str,
        storage: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        enable_analysis: str,
    ) -> None:
        # instance variables
        self._enable_analysis = enable_analysis
        self._use_cases: UseCaseDict[str, UseCase]
        self._degradation_topics: list[str]
        self._rosbag_manager: RosBagManager

        # load scenario
        scenario = load_scenario_with_exception(
            scenario_path,
            scenario_class,
            result_jsonl_path,
        )

        # get use case info list
        use_case_info_list: list[UseCaseInfo] = self._get_use_case_info_list(
            scenario,
            evaluation_topics_with_task,
            degradation_topic,
            result_jsonl_path,
            result_archive_path,
        )

        # initialize evaluation manager, result, and result writer for each use case
        temp_use_cases = {}  # temporary dict
        for use_case_info in use_case_info_list:
            temp_use_cases[use_case_info.name] = UseCase(
                evaluation_manager=use_case_info.evaluation_manager_class(
                    scenario,
                    t4_dataset_path,
                    result_archive_path,
                    use_case_info.evaluation_topics_with_task,
                    use_case_info.degradation_topic,
                ),
                result=use_case_info.result_class(use_case_info.conditions),
                result_writer=ResultWriter(
                    use_case_info.result_jsonl_path, Clock(), use_case_info.conditions
                ),
            )
        self._use_cases = UseCaseDict(
            temp_use_cases
        )  # make it read-only and possible to access by topic name

        # initialize RosBagManager to read and save rosbag and write into it
        evaluation_all_topics = [
            topic
            for use_case in self._use_cases.values()
            for topic in use_case.evaluation_manager.get_evaluation_topics()
        ]
        self._rosbag_manager = RosBagManager(
            rosbag_dir_path,
            Path(result_archive_path).joinpath("result_bag").as_posix(),
            storage,
            evaluation_all_topics,
            self._get_external_record_topics(),
        )

        # set degradation topics
        self._degradation_topics = [
            topic
            for use_case in self._use_cases.values()
            for topic in use_case.evaluation_manager.get_degradation_topics()
        ]

        # save scenario.yaml to check later
        shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    @abstractmethod
    def _get_use_case_info_list(
        self,
        scenario: ScenarioType,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        result_jsonl_path: str,
        result_archive_path: str,
    ) -> list[UseCaseInfo]:
        """
        Get use case info list for each use case.

        Args:
            scenario (ScenarioType): The scenario object.
            evaluation_topics_with_task (dict[str, list[str]]): Dictionary mapping evaluation topics to their tasks.
            degradation_topic (str): Topic name for degradation information.
            result_jsonl_path (str): Path to the result jsonl file.
            result_archive_path (str): Path to the result archive directory. If necessary, use this to create result jsonl path for each use case.

        Returns:
            list[UseCaseInfo]: The list of use case info.

        """
        raise NotImplementedError

    @abstractmethod
    def _get_external_record_topics(self) -> list[TopicMetadata]:
        """
        Get the list of topics to be recorded from external source.

        Returns:
            list[TopicMetadata]: The list of TopicMetadata for external recording.

        """
        raise NotImplementedError

    def evaluate(self) -> None:
        """Evaluate rosbag frame by frame."""
        for topic_name, msg, subscribed_timestamp_nanosec in self._rosbag_manager.read_messages():
            frame_result: FrameResult = self._evaluate_frame(
                topic_name, msg, subscribed_timestamp_nanosec
            )
            if topic_name in self._degradation_topics:
                self._write_result(frame_result, msg.header, subscribed_timestamp_nanosec)
        self._evaluate_on_post_process()
        self._close()
        self._merge_results()
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
        converted_data: ConvertedData = self._convert_ros_msg_to_data(
            topic_name, msg, subscribed_timestamp_nanosec
        )
        evaluation_manager = self._use_cases[topic_name].evaluation_manager
        return evaluation_manager.evaluate_frame(topic_name, converted_data)

    @abstractmethod
    def _convert_ros_msg_to_data(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> ConvertedData:
        """
        Convert a ROS message to ConvertedData.

        Args:
            topic_name (str): Topic name of the message.
            msg (Any): The ROS message to be converted.
            subscribed_timestamp_nanosec (int): The timestamp when the message was subscribed, in nanoseconds.

        Returns:
            ConvertedData: The converted data.

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

    def _close(self) -> None:
        """Close all resources."""
        for use_case in self._use_cases.values():
            use_case.result_writer.close()
        self._rosbag_manager.close_writer()

    def _merge_results(self) -> None:
        """Merge results from all use cases into a single result. The head use case's result is used as the base."""
        if len(self._use_cases) <= 1:
            return
        result_paths = [use_case.result_writer.result_path for use_case in self._use_cases.values()]
        multi_result_editor = MultiResultEditor(result_paths)
        multi_result_editor.write_back_result()

    @abstractmethod
    def _analysis(self) -> None:
        """Perform analysis on the evaluation results."""
        raise NotImplementedError
