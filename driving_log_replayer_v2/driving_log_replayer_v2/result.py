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
from collections.abc import Iterator
from dataclasses import dataclass
from os.path import expandvars
from pathlib import Path
import pickle
from typing import Any
from typing import TYPE_CHECKING
from typing import TypeVar

from ament_index_python.packages import get_package_share_directory
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from pydantic import BaseModel
from rclpy.clock import Clock
from rclpy.clock import ClockType
import simplejson as json

if TYPE_CHECKING:
    from rclpy.time import Time


def get_sample_result_path(
    use_case_name: str,
    result_file_name: str = "result.json",
) -> Path:
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
    time_out: int = 0  # for perception

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


class DummyResult(ResultBase):
    def __init__(self, condition: any) -> None:  # noqa
        super().__init__()

    def update(self) -> None:
        pass

    def set_frame(self) -> None:
        pass


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

    def write_line(self, write_obj: Any) -> str:
        str_record = json.dumps(write_obj, ignore_nan=True)
        self._result_file.write(str_record + "\n")
        return str_record

    def write_result(self, result: ResultBase) -> str:
        return self.write_line(self.get_result(result, None))

    def write_result_with_time(self, result: ResultBase, timestamp_nanosec: int) -> str:
        return self.write_line(self.get_result(result, timestamp_nanosec))

    def write_condition(self, condition: BaseModel | dict | list, *, updated: bool = False) -> str:
        if isinstance(condition, list):
            # Convert list of BaseModel objects to list of dictionaries
            condition_dict = [
                item.model_dump() if isinstance(item, BaseModel) else item for item in condition
            ]
        elif isinstance(condition, BaseModel):
            condition_dict = condition.model_dump()
        else:
            condition_dict = condition
        key = "Condition"
        if updated:
            key = "UpdatedCondition"
        write_obj = {key: condition_dict}
        return self.write_line(write_obj)

    def get_header(self) -> dict:
        system_time = self._system_clock.now()
        return {
            "Result": {"Success": False, "Summary": "NoData"},
            "Stamp": {"System": system_time.nanoseconds / pow(10, 9)},
            "Frame": {},
        }

    def get_result(self, result: ResultBase, timestamp_nanosec: int | None = None) -> dict:
        # timestamp_nanosec includes both sec and nanosec parts
        system_time: Time = self._system_clock.now()
        time_dict = {"System": system_time.nanoseconds / pow(10, 9)}
        if timestamp_nanosec is not None:
            time_dict["ROS"] = timestamp_nanosec / pow(10, 9)
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

    def __exit__(self, typ, exc, tb) -> None:  # noqa
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


ResultBaseType = TypeVar("ResultBaseType", bound=ResultBase)


@dataclass
class PlotConfig:
    method: str
    axes: dict[str, Any]
    plot: dict[str, Any]


class AnalysisData(ABC):
    def __init__(self) -> None:
        self._data: list[Any] = []

    # TODO: Define the type of data(=line in result.jsonl) more specifically.
    @abstractmethod
    def append(self, data: dict) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_plot_config(self) -> PlotConfig:
        raise NotImplementedError

    @property
    def name(self) -> str:
        return self.__class__.__name__

    @property
    def data(self) -> list[Any]:
        return self._data


AnalysisDataType = TypeVar("AnalysisDataType", bound=AnalysisData)


class Success(AnalysisData):
    def append(self, data: dict) -> None:
        self._data.append(int(data["Result"]["Success"]))

    def get_plot_config(self) -> PlotConfig:
        return PlotConfig(
            method="step",
            axes={
                "title": "Success over Timestamp",
                "xlabel": "Timestamp [s]",
                "ylabel": "Success",
                "ylim": (0.0, 1.0),
                "yticks": [0, 1],
            },
            plot={
                "where": "post",
                "linewidth": 3,
            },
        )


class AnalysisDataCollection:
    def __init__(self) -> None:
        self.data: list[AnalysisDataType] = [
            Success(),
        ]

    def append(self, data: dict) -> None:
        for analysis in self.data:
            analysis.append(data)

    def __len__(self) -> int:
        return len(self.data)

    def __iter__(self) -> Iterator[AnalysisDataType]:
        return iter(self.data)


class ResultAnalyzer:
    FIG_SIZE = (8, 8)

    def __init__(self, result_jsonl_path: str | Path, output_path: str | Path) -> None:
        self._result_jsonl_path = Path(expandvars(result_jsonl_path))
        self._output_path = Path(expandvars(output_path))
        self._timestamp: list[float] = []
        self._data: AnalysisDataCollection = AnalysisDataCollection()

    def analyze_results(self) -> None:
        with self._result_jsonl_path.open("r") as result_file:
            for line in result_file:
                result_data = json.loads(line)
                self._run_by_line(result_data)
        self._run_on_post_process()

    def _run_by_line(self, result_data: dict) -> None:
        if set(result_data.keys()) != {"Result", "Stamp", "Frame"}:
            return
        if "ROS" not in result_data["Stamp"]:
            return
        self._timestamp.append(result_data["Stamp"]["ROS"])
        self._data.append(result_data)

    def _run_on_post_process(self) -> None:
        num_analyses = len(self._data)
        fig = plt.figure(figsize=(self.FIG_SIZE[0] * num_analyses, self.FIG_SIZE[1]))

        for index, analysis in enumerate(self._data):
            config = analysis.get_plot_config()
            # create axes
            axes = fig.add_subplot(
                1,  # nrows
                num_analyses,  # ncols
                index + 1,  # index
                **config.axes,
            )
            # plot data
            getattr(axes, config.method)(
                self._timestamp,
                analysis.data,
                label=analysis.name,
                **config.plot,
            )
            self._set_x_axis_format(axes)
        self._set_fig_format(fig)

    def _set_x_axis_format(self, axes: Axes) -> None:
        axes.ticklabel_format(style="plain", axis="x", useOffset=False)
        axes.xaxis.set_major_locator(mticker.MaxNLocator(6, integer=True))

    def _set_fig_format(self, fig: Figure) -> None:
        fig.suptitle("Driving Log Replayer V2 Result Analysis")
        fig.legend()
        fig.tight_layout()
        fig.savefig(self._output_path.joinpath("result_jsonl.png"))
        plt.close()
