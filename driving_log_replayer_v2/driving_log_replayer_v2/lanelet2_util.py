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
import yaml
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_extension_python.utility import query
from lanelet2.core import Lanelet
from lanelet2.projection import UtmProjector
from shapely.geometry import Polygon
from pathlib import Path

def load_map(map_path: str) -> lanelet2.core.LaneletMap:
    map_dir = Path(map_path).parent
    projector_path = map_dir / "map_projector_info.yaml"

    if projector_path.exists():
        with open(projector_path, "r") as f:
            projector_config = yaml.safe_load(f)

        projector_type = projector_config.get("projector_type", "MGRS")

        if projector_type == "MGRS":
            projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
        else:
            try:
                lat = projector_config["map_origin"]["latitude"]
                lon = projector_config["map_origin"]["longitude"]
            except KeyError as e:
                raise ValueError(f"map_projector_info.yaml is missing required key for UTM: {e}")
            origin = lanelet2.io.Origin(lat, lon)
            projection = UtmProjector(origin)
    else:
       projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))

    return lanelet2.io.load(map_path, projection)

def load_map2(map_path: str) -> lanelet2.core.LaneletMap:
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
