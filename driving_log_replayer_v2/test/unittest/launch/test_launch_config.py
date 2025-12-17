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
        == "^/tf$|^/tf_static$|^/diagnostics$|^/system/processing_time_checker/metrics$|^/awapi/autoware/get/status$|^/sensing/camera/.*/compressed$|^/sensing/camera/.*/camera_info$|^/sensing/lidar/concatenated/pointcloud$|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*|^/perception/object_recognition/prediction/map_based_prediction/debug/pipeline_latency_ms$|^/perception/object_recognition/.*/objects$|^/perception/object_recognition/objects$|^/perception/object_recognition/detection/rois[0-9]+$|^/perception/object_recognition/detection/objects_before_filter$|^/sensing/.*detected_objects$|^/sensing/.*tracked_objects$|^/map/vector_map_marker$|^/localization/kinematic_state$|^/planning/planning_factors/.*"
    )
    arg_disable = launch_config.AUTOWARE_DISABLE
    assert callable(arg_disable)
    arg_disable = arg_disable({})
    assert arg_disable == {
        "localization": "false",
        "planning": "false",
        "control": "false",
    }


def test_load_timestep_based_trajectory_config() -> None:
    module_name = "driving_log_replayer_v2.launch.time_step_based_trajectory"
    launch_config = import_module(module_name)
    # Check RECORD_TOPIC contains expected topics
    assert "^/tf$" in launch_config.RECORD_TOPIC
    assert "^/tf_static$" in launch_config.RECORD_TOPIC
    assert "^/diagnostics$" in launch_config.RECORD_TOPIC
    assert "^/localization/kinematic_state$" in launch_config.RECORD_TOPIC
    assert "^/planning/trajectory_generator/.*" in launch_config.RECORD_TOPIC
    # Check AUTOWARE_DISABLE
    assert launch_config.AUTOWARE_DISABLE == {
        "localization": "false",
        "perception": "false",
        "control": "false",
    }
    # Check AUTOWARE_ARGS
    assert launch_config.AUTOWARE_ARGS == {}
    # Check NODE_PARAMS is empty dict
    assert launch_config.NODE_PARAMS == {}
    # Check USE_CASE_ARGS is empty list
    assert launch_config.USE_CASE_ARGS == []
