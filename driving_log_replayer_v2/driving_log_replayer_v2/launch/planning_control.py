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

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

RECORD_TOPIC = """^/tf$\
|^/tf_static$\
|^/parameter_events$\
|^/diagnostics$\
|^/map/vector_map_marker$\
|^/map/pointcloud_map$\
|^/sensing/camera/.*/compressed$\
|^/sensing/camera/.*/camera_info$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/objects$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/traffic_light_recognition/traffic_signals$\
|^/perception/occupancy_grid_map/map$\
|^/perception/obstacle_segmentation/pointcloud$\
|^/system/v2x/virtual_traffic_light_states$\
|^/localization/kinematic_state$\
|^/localization/initialization_state$\
|^/localization/pose_with_covariance$\
|^/localization/acceleration$\
|^/planning/.*$\
|^/control/.*$\
|^/system/processing_time_checker/metrics$\
|^/driving_log_replayer/.*\
"""

AUTOWARE_DISABLE = {
    "localization": "false",
}

AUTOWARE_ARGS = {"use_aeb_autoware_state_check": "false"}

NODE_PARAMS: dict[str, LaunchConfiguration] = {}

USE_CASE_ARGS: list[DeclareLaunchArgument] = []
