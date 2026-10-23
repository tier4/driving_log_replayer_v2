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
    from driving_log_replayer_v2.launch.rosbag import user_defined_remap

CONCAT_TOPIC = "/sensing/lidar/concatenated/pointcloud"
CONCAT_REMAP = f"{CONCAT_TOPIC}:=/unused{CONCAT_TOPIC}"
OTHER_REMAP = "/perception/objects:=/unused/perception/objects"


def _make_conf(with_autoware: str, sensing: str | None) -> dict:
    conf = {
        "with_autoware": with_autoware,
        "remap_arg": f"{CONCAT_TOPIC},/perception/objects",
        "remap_profile": "",
        "publish_profile": "",
    }
    if sensing is not None:
        conf["sensing"] = sensing
    return conf


def test_user_defined_remap_sensing_true() -> None:
    assert user_defined_remap(_make_conf("true", "true")) == [CONCAT_REMAP, OTHER_REMAP]


def test_user_defined_remap_sensing_false_excludes_concat() -> None:
    assert user_defined_remap(_make_conf("true", "false")) == [OTHER_REMAP]


def test_user_defined_remap_without_autoware_keeps_concat() -> None:
    # sensing is not evaluated when autoware is not launched
    assert user_defined_remap(_make_conf("false", None)) == [CONCAT_REMAP, OTHER_REMAP]
    assert user_defined_remap(_make_conf("false", "false")) == [CONCAT_REMAP, OTHER_REMAP]
