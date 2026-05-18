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

from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from driving_log_replayer_v2.launch.argument import add_use_case_arguments
from driving_log_replayer_v2.launch.use_case import launch_evaluator_node

# ── DLR2 use case config interface ──────────────────────────────────────────
# This use case does not replay a bag through Autoware; instead the evaluator
# node directly orchestrates make_lite + scenario_test_runner + compare_logs.

RECORD_TOPIC = ""  # no bag recording

# All Autoware components are disabled; the custom launcher skips launch_autoware.
AUTOWARE_DISABLE: dict = {}

AUTOWARE_ARGS: dict = {}

NODE_PARAMS: dict[str, LaunchConfiguration] = {
    "vehicle_model_normal": LaunchConfiguration("vehicle_model_normal"),
    "vehicle_model_godot": LaunchConfiguration("vehicle_model_godot"),
    "sensor_model_sim": LaunchConfiguration("sensor_model_sim"),
    "godot_executable": LaunchConfiguration("godot_executable"),
    "scenario_test_runner_scenario": LaunchConfiguration("scenario_test_runner_scenario"),
    # t4_dataset_path/map を compare_logs.py の地図解決に渡す（argument.py:241 が自動設定）
    "map_path": LaunchConfiguration("map_path"),
}

USE_CASE_ARGS: list[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        "vehicle_model_normal",
        default_value="best_model",
        description="Vehicle model name used for the normal simulator run.",
    ),
    DeclareLaunchArgument(
        "vehicle_model_godot",
        default_value="j6_gen2_godot",
        description="Vehicle model name used for the Godot simulator run.",
    ),
    DeclareLaunchArgument(
        "sensor_model_sim",
        default_value="aip_x2_gen2",
        description="Sensor model name used for both simulator runs.",
    ),
    DeclareLaunchArgument(
        "godot_executable",
        default_value="/home/autoware/godot_autoware_simulator.x86_64",
        description="Path to the Godot simulator executable. Sim-godot step is skipped if missing.",
    ),
    DeclareLaunchArgument(
        "scenario_test_runner_scenario",
        default_value="",
        description="Absolute path to the scenario_test_runner scenario YAML used for sim_normal/sim_godot steps. Required when running real_log_sim_comparison.",
    ),
]


# ── Custom launcher (called from simulation.launch.py) ──────────────────────

def launch_real_log_sim_comparison() -> list:
    """Return launch actions for the real_log_sim_comparison use case.

    Skips Autoware, bag player, and bag recorder.  Only the evaluator node
    (which orchestrates the full analysis pipeline) is launched.
    """
    return [
        OpaqueFunction(function=add_use_case_arguments),
        OpaqueFunction(function=launch_evaluator_node),
    ]
