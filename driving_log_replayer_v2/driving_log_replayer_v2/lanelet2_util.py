# Copyright (c) 2024 TIER IV.inc
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

from typing import Any

import lanelet2  # isort:skip
import autoware_lanelet2_extension_python.regulatory_elements  # noqa: F401 # isort:skip # needed to register regulatory elements
from pathlib import Path
import warnings

from autoware_lanelet2_extension_python._autoware_lanelet2_extension_python_boost_python_projection import (
    TransverseMercatorProjector,
)
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_extension_python.utility import query
from lanelet2.core import Lanelet
from lanelet2.projection import UtmProjector
from pydantic import BaseModel
from shapely.geometry import Polygon
import yaml


class MapOrigin(BaseModel):
    latitude: float
    longitude: float


class MapProjectorInfo(BaseModel):
    projector_type: str
    map_origin: MapOrigin | None = None
    scale_factor: float | None = None


def load_map(map_path: str) -> lanelet2.core.LaneletMap:
    projector_path = Path(map_path).parent / "map_projector_info.yaml"

    if projector_path.exists():
        with projector_path.open(mode="r") as f:
            data = yaml.safe_load(f)
        map_projector_info = MapProjectorInfo(**data)

        projector_type = map_projector_info.projector_type

        if projector_type == "MGRS":
            projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
        elif projector_type == "LocalCartesianUTM":
            lat = map_projector_info.map_origin.latitude
            lon = map_projector_info.map_origin.longitude
            origin = lanelet2.io.Origin(lat, lon)
            projection = UtmProjector(origin)
        elif projector_type == "TransverseMercator":
            lat = map_projector_info.map_origin.latitude
            lon = map_projector_info.map_origin.longitude
            _ = map_projector_info.scale_factor  # scale_factor is currently unused
            origin = lanelet2.io.Origin(lat, lon)
            # NOTE: Currently, autoware_lanelet2_extension_python does not support scale_factor parameter.
            # https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension_python/src/projection.cpp#L48
            projection = TransverseMercatorProjector(origin)
        else:
            err_msg = f"Unsupported projector type or projector_type is not set: {projector_type}"
            raise ValueError(err_msg)
    else:
        warnings.warn(
            "No map_projector_info found. Using MGRSProjector with origin (0.0, 0.0).", stacklevel=2
        )
        projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))

    return lanelet2.io.load(map_path, projection)


def load_all_lanelets(map_path: str) -> Any:
    lanelet_map = load_map(map_path)
    return query.laneletLayer(lanelet_map)


def road_lanelets_from_file(map_path: str) -> Any:
    # return type autoware_lanelet2_extension_python._autoware_lanelet2_extension_python_boost_python_utility.lanelet::ConstLanelets
    return query.roadLanelets(load_all_lanelets(map_path))


def traffic_light_from_file(map_path: str) -> list:
    return query.trafficLights(load_all_lanelets(map_path))


def to_shapely_polygon(lanelet: Lanelet) -> Polygon:
    points: list[float, float] = []
    for p_2d in lanelet.polygon2d():
        points.append([p_2d.x, p_2d.y])
    return Polygon(points)
