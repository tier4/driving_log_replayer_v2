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

from __future__ import annotations

from dataclasses import dataclass
from sys import float_info
from typing import Any
from typing import ClassVar
from typing import Literal

import numpy as np
from perception_eval.common.object import DynamicObject
from pydantic import BaseModel
from pydantic import conlist
from pydantic import Field
from pydantic import field_validator
from pydantic import model_validator
from shapely.geometry import Polygon
from shapely.vectorized import contains

from driving_log_replayer_v2.result import EvaluationItem
from driving_log_replayer_v2.result import ResultBase
from driving_log_replayer_v2.scenario import number
from driving_log_replayer_v2.scenario import Scenario

PerceptionFPData = list[DynamicObject] | np.ndarray


@dataclass(slots=True)
class Polygon3D:
    XYZ_DIM: ClassVar[int] = 3  # number of coordinates per vertex (x, y, z)

    geom: Polygon
    frame_id: Literal["map", "base_link"]
    map_to_base_link: np.ndarray | None = None
    base_link_to_map: np.ndarray | None = None
    z_min: float | None = None
    z_max: float | None = None

    @classmethod
    def from_args(
        cls,
        *args: Any,
        frame_id: Literal["map", "base_link"],
        z_min: float | None = None,
        z_max: float | None = None,
        map_to_base_link: np.ndarray | None = None,
        base_link_to_map: np.ndarray | None = None,
        **kwargs: Any,
    ) -> Polygon3D:
        return cls(
            geom=Polygon(*args, **kwargs),
            frame_id=frame_id,
            z_min=z_min,
            z_max=z_max,
            map_to_base_link=map_to_base_link,
            base_link_to_map=base_link_to_map,
        )

    def __getattr__(self, name: str) -> Any:
        return getattr(self.geom, name)

    def convert_map_to_base_link(self) -> None:
        center_z_in_map: float
        center_z_in_base_link: float

        if self.frame_id != "map":
            err_msg = "convert_map_to_base_link can be called only when frame_id is 'map'"
            raise ValueError(err_msg)
        if self.map_to_base_link is None:
            err_msg = "map_to_base_link is required for convert_map_to_base_link"
            raise ValueError(err_msg)

        geom = np.array(self.geom.exterior.coords)
        if geom.shape[1] != Polygon3D.XYZ_DIM:
            err_msg = f"polygon requires {self.XYZ_DIM} elements"
            raise ValueError(err_msg)
        center_z_in_map = np.mean(geom[:, 2])

        geom_homogeneous = np.hstack([geom, np.ones((geom.shape[0], 1))])
        geom_in_base_link_homogeneous = (self.map_to_base_link @ geom_homogeneous.T).T
        geom_in_base_link = geom_in_base_link_homogeneous[:, :3]
        center_z_in_base_link = np.mean(geom_in_base_link[:, 2])

        self.geom = Polygon(geom_in_base_link)
        self.frame_id = "base_link"
        if self.z_min is not None:
            self.z_min = self.z_min - center_z_in_map + center_z_in_base_link
        if self.z_max is not None:
            self.z_max = self.z_max - center_z_in_map + center_z_in_base_link


class NonDetectionArea(BaseModel):
    # map frame
    # z value represents height from map origin to the ground
    polygon_2d: list[conlist(number, min_length=3, max_length=3)]
    z_min: number | None = None
    z_max: number | None = None

    @field_validator("polygon_2d")
    @classmethod
    def is_clockwise(cls, v: list[list[number]]) -> list | None:
        v_float = []
        if len(v) < 3:  # noqa
            err_msg = "polygon requires 3 or more elements"
            raise ValueError(err_msg)
        check_clock_wise: float = 0.0
        # convert int value to float
        for p_2d in v:
            v_float.append([float(p_2d[0]), float(p_2d[1]), float(p_2d[2])])
        for i, _ in enumerate(v_float):
            p1 = v_float[i]
            p2 = v_float[(i + 1) % len(v_float)]
            check_clock_wise += (p2[0] - p1[0]) * (p2[1] + p1[1])
        if check_clock_wise <= 0.0:
            err_msg = "polygon_2d is not clockwise"
            raise ValueError(err_msg)
        return v_float

    @field_validator("z_min", "z_max")
    @classmethod
    def to_float_or_none(cls, v: number | None) -> float | None:
        return float(v) if v is not None else None


class TimeStamp(BaseModel):
    start: float = Field(0.0, ge=0.0)
    end: float = Field(float_info.max, ge=0.0)

    @model_validator(mode="after")
    def validate_start_end(self) -> TimeStamp:
        err_msg = "end must be a greater number than start"

        if self.end < self.start:
            raise ValueError(err_msg)
        return self

    def is_valid(self, float_time: float) -> bool:
        return self.start <= float_time <= self.end


class Criteria(BaseModel):
    criteria_name: str | None = None
    PassRate: number
    non_detection_area: NonDetectionArea
    timestamp: list[TimeStamp] | None
    topic_type: list[Literal["bbox", "pointcloud"]]

    def get_type(self) -> tuple[type, ...]:
        return tuple(DynamicObject if topic == "bbox" else np.ndarray for topic in self.topic_type)

    def is_valid_topic_type(self, data: PerceptionFPData) -> bool:
        if isinstance(data, list) and len(data) > 0 and isinstance(data[0], DynamicObject):
            data_type = DynamicObject
        elif isinstance(data, np.ndarray):
            data_type = np.ndarray
        else:
            err_msg = "data must be either list of DynamicObject or numpy array"
            raise ValueError(err_msg)
        return data_type in self.get_type()


class Conditions(BaseModel):
    Criterion: list[Criteria]


class Evaluation(BaseModel):
    UseCaseName: Literal["perception_fp"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions
    Datasets: list[dict]


class PerceptionFPScenario(Scenario):
    Evaluation: Evaluation


@dataclass
class PerceptionFP(EvaluationItem):
    name: str = "Perception FP"

    def set_frame(
        self,
        timestamp: float,
        frame_id: str,
        data: PerceptionFPData,
        map_to_base_link: np.ndarray,
    ) -> dict | None:
        if self.condition.timestamp is not None and not any(
            condition_timestamp.is_valid(timestamp)
            for condition_timestamp in self.condition.timestamp
        ):
            self._non_detection_area = self.get_non_detection_area_with_transform(
                map_to_base_link, frame_id
            )
            self._fp_objects = []
            return {"Info": "Not in evaluation timestamp range"}

        if not self.condition.is_valid_topic_type(data):
            self._non_detection_area = self.get_non_detection_area_with_transform(
                map_to_base_link, frame_id
            )
            self._fp_objects = []
            return {"Info": "Not in evaluation topic type"}

        self.total += 1

        is_in_non_detection_area = self.is_in_non_detection_area(frame_id, data, map_to_base_link)

        if not is_in_non_detection_area:
            self.passed += 1

        current_rate = self.rate()
        self.success = current_rate >= self.condition.PassRate
        self.summary = f"{self.name} ({self.success_str()}): {self.passed} / {self.total} -> {current_rate:.2f}%"

        return {
            "PassFail": {
                "Result": {
                    "Total": self.success_str(),
                    "Frame": "Success" if not is_in_non_detection_area else "Fail",
                },
                "Info": {},
            },
        }

    def is_in_non_detection_area(
        self, frame_id: str, data: PerceptionFPData, map_to_base_link: np.ndarray
    ) -> bool:
        # boundary is not included
        exist = False
        non_detection_area: Polygon3D = self.get_non_detection_area_with_transform(
            map_to_base_link, frame_id
        )
        if isinstance(data, list):
            # TODO: use prepare function of shapely for performance improvement
            fp: list[DynamicObject] = []
            for bbox in data:
                corners: np.ndarray = bbox.get_corners()  # shape: (N, 3)
                is_in_z = np.any(
                    (
                        corners[:, 2] > non_detection_area.z_min
                        if non_detection_area.z_min is not None
                        else True
                    )
                    & (
                        corners[:, 2] < non_detection_area.z_max
                        if non_detection_area.z_max is not None
                        else True
                    )
                )
                is_in_xy = np.any(
                    contains(
                        non_detection_area.geom, corners[:, 0], corners[:, 1]
                    )  # or contains(non_detection_area, Polygon(corners[:, 0:2]))
                )
                if is_in_z and is_in_xy:
                    fp.append(bbox)
            exist = len(fp) != 0
            fp_objects = fp
        elif isinstance(data, np.ndarray):
            # check whether points exist in non-detection area z value
            pointcloud: np.ndarray = data
            pointcloud_in_z = (
                pointcloud[pointcloud[:, 2] > non_detection_area.z_min]
                if non_detection_area.z_min is not None
                else pointcloud
            )
            pointcloud_in_z = (
                pointcloud_in_z[pointcloud_in_z[:, 2] < non_detection_area.z_max]
                if non_detection_area.z_max is not None
                else pointcloud_in_z
            )
            # check whether points exist in non-detection area xy value
            pointcloud_in_non_detection_area = pointcloud_in_z[
                contains(non_detection_area.geom, pointcloud_in_z[:, 0], pointcloud_in_z[:, 1])
            ]
            exist = pointcloud_in_non_detection_area.size != 0
            fp_objects = pointcloud_in_non_detection_area
        # TODO: consider better way to store debug objects
        self._non_detection_area = non_detection_area
        self._fp_objects = fp_objects
        return exist

    def get_non_detection_area_with_transform(
        self, map_to_base_link: np.ndarray, frame_id: str
    ) -> Polygon3D:
        non_detection_area: Polygon3D = Polygon3D.from_args(
            self.condition.non_detection_area.polygon_2d,
            frame_id="map",
            map_to_base_link=map_to_base_link,
            z_min=self.condition.non_detection_area.z_min,
            z_max=self.condition.non_detection_area.z_max,
        )
        if frame_id == "base_link":
            non_detection_area.convert_map_to_base_link()
        elif frame_id == "map":
            pass
        else:
            err_msg = f"Unsupported frame_id: {frame_id}"
            raise ValueError(err_msg)
        return non_detection_area

    def get_non_detection_area(self) -> Polygon3D:
        return self._non_detection_area

    def get_fp_objects(self) -> PerceptionFPData:
        return self._fp_objects


class PerceptionFPResult(ResultBase):
    def __init__(self, condition: Conditions) -> None:
        super().__init__()
        self.__perception_fp_criterion: list[PerceptionFP] = []
        for i, criteria in enumerate(condition.Criterion):
            criteria_name = (
                criteria.criteria_name if criteria.criteria_name is not None else f"criteria_{i}"
            )
            self.__perception_fp_criterion.append(
                PerceptionFP(name=criteria_name, condition=criteria)
            )

    def update(self) -> None:
        all_summary: list[str] = []
        all_success: list[bool] = []
        for criterion in self.__perception_fp_criterion:
            tmp_success = criterion.success
            prefix_str = "Passed: " if tmp_success else "Failed: "
            all_summary.append(prefix_str + criterion.summary)
            all_success.append(tmp_success)
        self._summary = ", ".join(all_summary)
        self._success = all(all_success)

    def set_frame(
        self,
        timestamp: float,
        frame_id: str,
        data: PerceptionFPData,
        skip: int,
        map_to_base_link: np.ndarray,
    ) -> None:
        self._frame = {"FrameSkip": skip}
        for criterion in self.__perception_fp_criterion:
            self._frame[criterion.name] = criterion.set_frame(
                timestamp, frame_id, data, map_to_base_link
            )
        self.update()

    def set_warn_frame(self, data: str, skip: int) -> None:
        self._frame = {"Warning": data, "FrameSkip": skip}

    def get_non_detection_area(self) -> dict[str, Polygon3D]:
        non_detection_areas: dict[str, Polygon3D] = {}
        for criterion in self.__perception_fp_criterion:
            non_detection_areas[criterion.name] = criterion.get_non_detection_area()
        return non_detection_areas

    def get_fp_objects(self) -> dict[str, PerceptionFPData]:
        fp_objects: dict[str, PerceptionFPData] = {}
        for criterion in self.__perception_fp_criterion:
            fp_objects[criterion.name] = criterion.get_fp_objects()
        return fp_objects
