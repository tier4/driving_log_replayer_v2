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

from pathlib import Path
import shutil
from typing import TypeVar
from typing import Any
from dataclasses import dataclass

from std_msgs.msg import Header
from autoware_perception_msgs.msg import DetectedObjects
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects
from rclpy.clock import Clock
from rosbag2_py import TopicMetadata
from driving_log_replayer_v2.result import ResultWriter
from driving_log_replayer_v2.ros2_utils import RosBagManager
from driving_log_replayer_v2.evaluation_manager import EvaluationManager
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.post_process_evaluator import FrameResult

PerceptionMsgType = TypeVar("PerceptionMsgType", DetectedObjects, TrackedObjects, PredictedObjects)
ManagerType = TypeVar("ManagerType", bound=EvaluationManager)
ResultBaseType = TypeVar("ResultBaseType", bound=ResultBase)


@dataclass
class TopicInfo:
    name: str
    msg_type: str


class Runner(ABC):
    def __init__(
        self,
        manager_class: ManagerType,
        result_class: ResultBaseType,
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_json_path: str,
        result_archive_path: str,
        storage: str,
        evaluation_topics: dict[str, list[str]],
        enable_analysis: str,
        additional_record_topics: list[TopicInfo],
    ) -> None:
        self._enable_analysis = enable_analysis
        self._manager: ManagerType
        self._degradation_topic: str
        self._result_writer: ResultWriter
        self._result: ResultBaseType
        self._rosbag_manager: RosBagManager

        # initialize manager
        self._manager = manager_class(
            scenario_path,
            t4_dataset_path,
            result_archive_path,
            evaluation_topics,
        )
        if self._manager.check_scenario_error() is not None:
            error = self._manager.check_scenario_error()
            result_writer = ResultWriter(
                result_json_path,
                Clock(),
                {},
            )
            error_dict = {
                "Result": {"Success": False, "Summary": "ScenarioFormatError"},
                "Stamp": {"System": 0.0},
                "Frame": {"ErrorMsg": error.__str__()},
            }
            result_writer.write_line(error_dict)
            result_writer.close()
            raise error
        self._degradation_topic = self._manager.get_degradation_topic()

        # initialize ResultWriter and PerceptionResult to write result.jsonl which Manager will use
        self._result_writer = ResultWriter(
            result_json_path,
            Clock(),
            self._manager.get_evaluation_condition()
        )
        self._result = result_class(self._manager.get_evaluation_condition())

        # initialize RosBagManager to read and save rosbag and write into it
        additional_record_topics_metadata = [
            TopicMetadata(
                name=topic_info.name,
                type=topic_info.msg_type,
                serialization_format="cdr",
                offered_qos_profiles="",
            )
            for topic_info in additional_record_topics
        ]
        self._rosbag_manager = RosBagManager(
            rosbag_dir_path,
            Path(result_archive_path).joinpath("result_bag").as_posix(),
            storage,
            self._manager.get_evaluation_topics(),
            additional_record_topics_metadata,
        )

        # save scenario.yaml to check later
        shutil.copy(scenario_path, Path(result_archive_path).joinpath("scenario.yaml"))

    def evaluate(self) -> None:
        for topic_name, msg, subscribed_timestamp_nanosec in self._rosbag_manager.read_messages():
            # See RosBagManager for `time relationships`.
            frame_result = self._evaluate_frame(
                topic_name,
                msg,
                subscribed_timestamp_nanosec
            )
            if topic_name == self._degradation_topic:
                self._write_result(frame_result, msg.header, subscribed_timestamp_nanosec)
        self._evaluate_on_post_process()
        self._close()
        if self._enable_analysis == "true":
            self._analysis()

    @abstractmethod
    def _evaluate_frame(self, topic_name: str, msg: Any, subscribed_timestamp_nanosec: int) -> FrameResult:
        raise NotImplementedError

    @abstractmethod
    def _write_result(self, frame_result: FrameResult, header: Header, subscribed_timestamp_nanosec: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def _evaluate_on_post_process(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _analysis(self) -> None:
        raise NotImplementedError

    def _close(self) -> None:
        self._result_writer.close()
        self._rosbag_manager.close_writer()
