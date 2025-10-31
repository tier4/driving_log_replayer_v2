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

from driving_log_replayer_v2.post_process.evaluation_manager import ManagerType
from driving_log_replayer_v2.driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.result import ResultBaseType
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.post_process.ros2_utils import RosBagManager
from driving_log_replayer_v2.scenario import load_condition
from driving_log_replayer_v2.scenario import load_scenario_with_exception
from driving_log_replayer_v2.scenario import ScenarioType


@dataclass(frozen=True, slots=True)
class TopicInfo:
    name: str
    msg_type: str


@dataclass(frozen=True, slots=True)
class SimulationInfo:
    evaluation_manager_class: (
        ManagerType | None
    )  # optional for only using EvaluationItem and ResultBase
    result_class: ResultBaseType
    name: str
    evaluation_topics: dict[str, list[str]]
    result_json_path: str


@dataclass(frozen=True, slots=True)
class Simulation:
    evaluation_manager: ManagerType | None  # optional for only using EvaluationItem and ResultBase
    result: ResultBaseType
    result_writer: ResultWriter


class Runner(ABC):
    def __init__(
        self,
        scenario_class: ScenarioType,
        simulation_info_list: list[SimulationInfo],
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
        self._simulation: dict[str, Simulation]
        self._degradation_topics: list[str]
        self._rosbag_manager: RosBagManager

        # load scenario and condition
        scenario = load_scenario_with_exception(
            scenario_path,
            scenario_class,
            result_json_path,
        )
        evaluation_condition = load_condition(scenario)

        # initialize evaluation manager, result, and result writer for each simulation
        self._simulation = {}
        for simulation_info in simulation_info_list:
            self._simulation[simulation_info.name] = Simulation(
                evaluation_manager=simulation_info.evaluation_manager_class(
                    scenario,
                    t4_dataset_path,
                    result_archive_path,
                    simulation_info.evaluation_topics,
                )
                if simulation_info.evaluation_manager_class is not None
                else None,
                result=simulation_info.result_class(evaluation_condition),
                result_writer=ResultWriter(
                    simulation_info.result_json_path, Clock(), evaluation_condition
                ),
            )

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
            simulation.evaluation_manager.get_evaluation_topics()
            for simulation in self._simulation.values()
            if simulation.evaluation_manager is not None
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
            simulation.evaluation_manager.get_degradation_topic()
            for simulation in self._simulation.values()
            if simulation.evaluation_manager is not None
        ]

        # save scenario.yaml to check later
        shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    def evaluate(self) -> None:
        for topic_name, msg, subscribed_timestamp_nanosec in self._rosbag_manager.read_messages():
            # See RosBagManager for `time relationships`.
            frame_result = self._evaluate_frame(topic_name, msg, subscribed_timestamp_nanosec)
            if topic_name in self._degradation_topics:
                self._write_result(frame_result, msg.header, subscribed_timestamp_nanosec)
        self._evaluate_on_post_process()
        self._close()
        if self._enable_analysis == "true":
            self._analysis()

    @abstractmethod
    def _evaluate_frame(
        self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int
    ) -> FrameResult:
        raise NotImplementedError

    @abstractmethod
    def _write_result(
        self,
        frame_result: FrameResult,
        header: Header,
        subscribed_timestamp_nanosec: int,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def _evaluate_on_post_process(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _analysis(self) -> None:
        raise NotImplementedError

    def _close(self) -> None:
        for simulation in self._simulation.values():
            simulation.result_writer.close()
        self._rosbag_manager.close_writer()
