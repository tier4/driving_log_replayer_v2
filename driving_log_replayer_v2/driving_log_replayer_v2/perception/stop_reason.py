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

from pathlib import Path

import pandas as pd
from tier4_api_msgs.msg import AwapiAutowareStatus
from dataclasses import dataclass
from dataclasses import asdict


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


def convert_to_stop_reason(msg: AwapiAutowareStatus) -> StopReasonData:
    stop_reason = msg.stop_reason
    return StopReasonData(
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
            ) for i, reason in enumerate(stop_reason.stop_reasons)
        ],
    )

class StopReasonAnalyzer:
    def __init__(self) -> None:
        self._data: list[StopReasonData] = []

    def append(self, stop_reason: StopReasonData) -> None:
        self._data.append(stop_reason)

    def save_as_csv(self, save_path: Path) -> None:
        data_dict = [asdict(data) for data in self._data]
        pd.DataFrame(data_dict).to_csv(save_path, index=False)
