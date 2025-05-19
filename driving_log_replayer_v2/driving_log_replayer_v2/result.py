# Copyright (c) 2022 TIER IV.inc
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
from os.path import expandvars
from pathlib import Path
import pickle
from typing import Any

from pydantic import BaseModel
from rclpy.clock import Clock
from rclpy.clock import ClockType
import simplejson as json


def get_sample_result_path(
    use_case_name: str,
    result_file_name: str = "result.json",
) -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(
        get_package_share_directory("driving_log_replayer_v2"),
        "sample",
        use_case_name,
        result_file_name,
    )


@dataclass
class EvaluationItem(ABC):
    name: str
    # If condition is None, this evaluation item is not used.
    condition: BaseModel | None = None
    total: int = 0
    passed: int = 0
    summary: str = "NotTested"
    success: bool = False
    no_gt_no_obj: int = 0  # for perception, perception_2d, traffic_light

    def success_str(self) -> str:
        return "Success" if self.success else "Fail"

    def rate(self) -> float:
        return 0.0 if self.total == 0 else self.passed / self.total * 100.0

    @abstractmethod
    def set_frame(self) -> dict:
        return {}


class ResultBase(ABC):
    def __init__(self) -> None:
        self._success = False
        self._summary = "NoData"
        self._frame = {}

    @property
    def success(self) -> bool:
        return self._success

    @property
    def summary(self) -> str:
        return self._summary

    @property
    def frame(self) -> dict:
        return self._frame

    @abstractmethod
    def update(self) -> None:
        """Update success and summary."""

    @abstractmethod
    def set_frame(self) -> None:
        """Set the result of one frame from the subscribe ros message."""


class ResultWriter:
    def __init__(
        self,
        result_json_path: str,
        ros_clock: Clock,
        condition: BaseModel | dict,
    ) -> None:
        self._result_path = self.create_jsonl_path(result_json_path)
        self._result_file = self._result_path.open("w")
        self._ros_clock = ros_clock
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.write_condition(condition)
        self.write_line(self.get_header())

    @property
    def result_path(self) -> Path:
        return self._result_path

    def create_jsonl_path(self, result_json_path: str) -> Path:
        # For compatibility with previous versions.
        # If a json file name is passed, replace it with the filename + jsonl
        original_path = Path(expandvars(result_json_path))
        return original_path.parent.joinpath(original_path.stem + ".jsonl")

    def close(self) -> None:
        if not self._result_file.closed:
            self._result_file.close()

    def delete_result_file(self) -> None:
        self._result_path.unlink()

    def write_line(self, write_obj: Any) -> None:
        str_record = json.dumps(write_obj, ignore_nan=True) + "\n"
        self._result_file.write(str_record)

    def write_result(self, result: ResultBase) -> None:
        self.write_line(self.get_result(result, None))

    def write_result_with_time(self, result: ResultBase, ros_timestamp: int) -> None:
        self.write_line(self.get_result(result, ros_timestamp))

    def write_condition(self, condition: BaseModel | dict, *, updated: bool = False) -> None:
        condition_dict = condition if isinstance(condition, dict) else condition.model_dump()
        key = "Condition"
        if updated:
            key = "UpdatedCondition"
        self.write_line({key: condition_dict})

    def get_header(self) -> dict:
        system_time = self._system_clock.now()
        return {
            "Result": {"Success": False, "Summary": "NoData"},
            "Stamp": {"System": system_time.nanoseconds / pow(10, 9)},
            "Frame": {},
        }

    def get_result(self, result: ResultBase, ros_timestamp: int | None = None) -> dict:
        system_time = self._system_clock.now()
        time_dict = {"System": system_time.nanoseconds / pow(10, 9)}
        if ros_timestamp is not None:
            time_dict["ROS"] = ros_timestamp / pow(10, 9)
        elif self._ros_clock.ros_time_is_active:
            ros_time = self._ros_clock.now()
            time_dict["ROS"] = ros_time.nanoseconds / pow(10, 9)
        else:
            time_dict["ROS"] = 0.0

        return {
            "Result": {"Success": result.success, "Summary": result.summary},
            "Stamp": time_dict,
            "Frame": result.frame,
        }


class PickleWriter:
    def __init__(self, out_pkl_path: str, write_object: Any) -> None:
        with Path(expandvars(out_pkl_path)).open("wb") as pkl_file:
            pickle.dump(write_object, pkl_file)


class ResultEditor:
    def __init__(self, result_jsonl_path: str) -> None:
        self._result_path = Path(expandvars(result_jsonl_path))
        self._result_file = self._result_path.open("r+")
        self._last_result = self.load_last_result()
        self.success: bool = self._last_result["Result"]["Success"]
        self.summary: str = self._last_result["Result"]["Summary"]

    def __enter__(self) -> "ResultEditor":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def load_last_result(self) -> dict:
        last_line = ""
        for line in self._result_file:
            last_line = line
        if not last_line:
            return {"Result": {"Success": False, "Summary": "NoData"}}
        return json.loads(last_line)

    def close(self) -> None:
        if not self._result_file.closed:
            self._result_file.close()

    def add_result(self, write_obj: Any) -> None:
        self._result_file.seek(0, 2)  # end of file
        result_str = json.dumps(write_obj, ignore_nan=True) + "\n"
        self._result_file.write(result_str)
        self._result_file.flush()  # Ensure data is written to disk


class MultiResultEditor:
    def __init__(self, result_jsonl_paths: list[str]) -> None:
        self._result_jsonl_paths = result_jsonl_paths
        self.success = True
        self.summary = "MergedSummary"
        for result_jsonl_path in result_jsonl_paths:
            with ResultEditor(result_jsonl_path) as result:
                if not result.success:
                    self.success = False
                self.summary += "-" + result.summary

    def write_back_result(self) -> None:
        with ResultEditor(self._result_jsonl_paths[0]) as main_result_file:
            final_result = {
                "Result": {"Success": self.success, "Summary": self.summary},
                "Stamp": {},
                "Frame": {},
            }
            main_result_file.add_result(final_result)
