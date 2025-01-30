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

from driving_log_replayer_v2.pose import pose_str_to_dict


def test_pose_str_to_dict() -> None:
    pose_str = (
        '{"position": {"x": 1, "y": 2, "z": 3}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}'
    )

    pose_dict = pose_str_to_dict(pose_str)
    # The line below doesn't throw AssertionError
    assert 3.0 == 3  # noqa
    # Need to check the value type, not the value
    assert isinstance(pose_dict["position"]["x"], float)
    assert pose_dict["position"]["x"] == 1.0
    assert pose_dict["position"]["x"] == 1  # This also doesn't throw AssertionError
    assert isinstance(pose_dict["position"]["y"], float)
    assert isinstance(pose_dict["position"]["z"], float)
    assert isinstance(pose_dict["orientation"]["x"], float)
    assert isinstance(pose_dict["orientation"]["y"], float)
    assert isinstance(pose_dict["orientation"]["z"], float)
    assert isinstance(pose_dict["orientation"]["w"], float)
