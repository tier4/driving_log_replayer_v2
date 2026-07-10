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

from pathlib import Path

import yaml

CONFIG_DIR = Path(__file__).resolve().parents[3] / "config" / "record"


def _load_record_patterns(profile_name: str) -> list[str]:
    with (CONFIG_DIR / f"{profile_name}.yaml").open() as f:
        return yaml.safe_load(f)["record"]


def test_perception_record_profile() -> None:
    patterns = _load_record_patterns("perception")
    assert "|".join(patterns) == (
        "^/tf$|^/tf_static$|^/diagnostics$|^/system/processing_time_checker/metrics$"
        "|^/awapi/autoware/get/status$|^/sensing/camera/.*/compressed$"
        "|^/sensing/camera/.*/camera_info$|^/sensing/lidar/concatenated/pointcloud$"
        "|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$"
        "|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*"
        "|^/perception/object_recognition/prediction/map_based_prediction/debug/pipeline_latency_ms$"
        "|^/perception/object_recognition/.*/objects$|^/perception/object_recognition/objects$"
        "|^/perception/object_recognition/detection/rois[0-9]+$"
        "|^/perception/object_recognition/detection/objects_before_filter$"
        "|^/sensing/.*detected_objects$|^/sensing/.*tracked_objects$"
        "|^/map/vector_map_marker$|^/localization/kinematic_state$"
        "|^/planning/planning_factors/.*"
        "|^/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/predicted_objects$"
    )


def test_time_step_based_trajectory_record_profile() -> None:
    patterns = _load_record_patterns("time_step_based_trajectory")
    regex = "|".join(patterns)
    assert "^/tf$" in regex
    assert "^/tf_static$" in regex
    assert "^/diagnostics$" in regex
    assert "^/localization/kinematic_state$" in regex
    assert "^/planning/.*$" in regex
    assert "^/control/.*$" not in regex
