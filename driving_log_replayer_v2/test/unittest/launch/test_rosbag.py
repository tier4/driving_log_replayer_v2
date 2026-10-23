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

from unittest.mock import patch

# rosbag.py resolves the package share directory at import time.
with patch("ament_index_python.packages.get_package_share_directory", return_value="/tmp"):  # noqa: S108
    from driving_log_replayer_v2.launch.rosbag import component_defined_remap

CONCATENATED_POINTCLOUD_REMAP = (
    "/sensing/lidar/concatenated/pointcloud:=/unused/sensing/lidar/concatenated/pointcloud"
)


def test_component_defined_remap_sensing_true() -> None:
    conf = {"with_autoware": "true", "sensing": "true"}
    assert component_defined_remap(conf) == [CONCATENATED_POINTCLOUD_REMAP]


def test_component_defined_remap_sensing_not_set() -> None:
    # sensing is launched by default
    conf = {"with_autoware": "true"}
    assert component_defined_remap(conf) == [CONCATENATED_POINTCLOUD_REMAP]


def test_component_defined_remap_sensing_false() -> None:
    conf = {"with_autoware": "true", "sensing": "false"}
    assert component_defined_remap(conf) == []


def test_component_defined_remap_without_autoware() -> None:
    conf = {"with_autoware": "false", "sensing": "true"}
    assert component_defined_remap(conf) == []
