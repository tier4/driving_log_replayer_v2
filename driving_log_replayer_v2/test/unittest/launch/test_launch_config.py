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

from importlib import import_module


def test_load_perception_config() -> None:
    module_name = "driving_log_replayer_v2.launch.perception"
    launch_config = import_module(module_name)
    assert (
        launch_config.RECORD_TOPIC
        == "^/tf$|^/tf_static$|^/diagnostics$|^/sensing/camera/.*|^/sensing/lidar/concatenated/pointcloud$|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*|^/perception/object_recognition/prediction/map_based_prediction/debug/pipeline_latency_ms$|^/perception/object_recognition/.*/objects$|^/perception/object_recognition/objects$|^/perception/object_recognition/detection/objects_before_filter$|^/sensing/.*detected_objects$|^/sensing/.*tracked_objects$"
    )
    assert launch_config.AUTOWARE_DISABLE == {
        "localization": "false",
        "planning": "false",
        "control": "false",
    }
