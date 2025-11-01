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

from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path

import pandas as pd
from tier4_api_msgs.msg import AwapiAutowareStatus


@dataclass
class Position:
    x: float
    y: float
    z: float


@dataclass
class Orientation:
    x: float
    y: float
    z: float
    w: float


@dataclass
class Reason:
    index: int
    reason: str
    dist_to_stop_pose: float
    position: Position
    orientation: Orientation


@dataclass
class StopReasonData:
    seconds: int
    nanoseconds: int
    reasons: list[Reason]


def convert_to_stop_reason(
    msg: AwapiAutowareStatus, subscribed_timestamp_nanosec: int
) -> tuple[int, int, StopReasonData]:
    stop_reason = msg.stop_reason
    header_timestamp_nanosec = (
        stop_reason.header.stamp.sec * 10**9 + stop_reason.header.stamp.nanosec
    )
    return (
        header_timestamp_nanosec,
        subscribed_timestamp_nanosec,
        StopReasonData(
            seconds=stop_reason.header.stamp.sec,
            nanoseconds=stop_reason.header.stamp.nanosec,
            reasons=[
                Reason(
                    index=i,
                    reason=reason.reason,
                    dist_to_stop_pose=reason.stop_factors[0].dist_to_stop_pose,
                    position=Position(
                        x=reason.stop_factors[0].stop_pose.position.x,
                        y=reason.stop_factors[0].stop_pose.position.y,
                        z=reason.stop_factors[0].stop_pose.position.z,
                    ),
                    orientation=Orientation(
                        x=reason.stop_factors[0].stop_pose.orientation.x,
                        y=reason.stop_factors[0].stop_pose.orientation.y,
                        z=reason.stop_factors[0].stop_pose.orientation.z,
                        w=reason.stop_factors[0].stop_pose.orientation.w,
                    ),
                )
                for i, reason in enumerate(stop_reason.stop_reasons)
            ],
        ),
    )


class StopReasonAnalyzer:
    def __init__(self, result_archive_path: str, dir_name: str) -> None:
        self._data: list[StopReasonData] = []
        self._result_archive_path_w_dir_name = Path(result_archive_path).joinpath(dir_name)
        self._result_archive_path_w_dir_name.mkdir(parents=True, exist_ok=True)

    def append(self, stop_reason: StopReasonData) -> None:
        self._data.append(stop_reason)

    def save_as_csv(self, file_name: str) -> None:
        if not file_name.endswith(".csv"):
            err_msg = f"Invalid file extension: {file_name}. '.csv' is required."
            raise ValueError(err_msg)

        data_dict = [asdict(data) for data in self._data]
        pd.DataFrame(data_dict).to_csv(
            self._result_archive_path_w_dir_name.joinpath(file_name), index=False
        )
