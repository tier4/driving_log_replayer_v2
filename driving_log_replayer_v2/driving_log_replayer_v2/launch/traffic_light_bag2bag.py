# Copyright (c) 2026 TIER IV.inc
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

import json
from pathlib import Path
import re
import shutil
import signal
import tempfile

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import ProcessExited
from launch.events.process import SignalProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
import yaml

from driving_log_replayer_v2.launch.argument import add_use_case_arguments

MAP_TOPIC = "/map/vector_map"
MAP_TIMEOUT_S = 60.0


def camera_numbers(launch_file: str) -> str:
    """Return the camera_namespaces default of launch_file as --camera of the runner: [camera4, camera5] -> 4,5."""
    context = LaunchContext()
    entities = AnyLaunchDescriptionSource(launch_file).get_launch_description(context).entities
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == "camera_namespaces":
            namespaces = yaml.safe_load(perform_substitutions(context, entity.default_value))
            return ",".join(re.sub(r"\D", "", namespace) for namespace in namespaces)
    msg = f"{launch_file} declares no camera_namespaces argument"
    raise ValueError(msg)


def run_bag2bag(context: LaunchContext) -> list:
    """
    Write result_bag with the bag2bag runner in place of Autoware and ros2 bag play / record.

    The runner reads the map from a rosbag, so this first writes map_bag: the production map nodes
    load the map of t4_dataset and publish /map/vector_map, and ros2 bag record records it.
    """
    conf = context.launch_configurations

    # The post process replaces this result with the evaluation of result_bag. It is a failure, so
    # that the case fails when the runner or the post process does not finish.
    with Path(conf["result_jsonl_path"]).open("w") as f:
        json.dump(
            {
                "Result": {"Success": False, "Summary": "bag2bag did not finish"},
                "Stamp": {"System": 0.0},
                "Frame": {},
            },
            f,
        )

    map_dir = Path(conf["t4_dataset_path"], "map")
    tmp_dir = tempfile.mkdtemp(prefix="traffic_light_map_bag_")
    map_bag = Path(tmp_dir, "map_bag")

    # The production map nodes publish the map of map_dir.
    map_projection = ExecuteProcess(
        cmd=[
            "ros2", "launch", "autoware_map_projection_loader", "map_projection_loader.launch.xml",
            f"map_projector_info_path:={map_dir / 'map_projector_info.yaml'}",
            f"lanelet2_map_path:={map_dir / 'lanelet2_map.osm'}",
        ],
        name="map_projection_loader",
    )  # fmt: skip
    map_loader = ExecuteProcess(
        cmd=[
            "ros2", "launch", "autoware_map_loader", "lanelet2_map_loader.launch.xml",
            f"lanelet2_map_path:={map_dir / 'lanelet2_map.osm'}",
        ],
        name="lanelet2_map_loader",
    )  # fmt: skip
    map_recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", conf["storage"], "-o", map_bag.as_posix(), MAP_TOPIC],
        name="map_bag_recorder",
    )
    # Waits until the map nodes publish the map, to stop them and ros2 bag record after that.
    wait_until_map_published = ExecuteProcess(
        cmd=[
            "ros2", "topic", "echo", "--once", "--no-arr",
            "--qos-durability", "transient_local", "--qos-reliability", "reliable",
            MAP_TOPIC, "autoware_map_msgs/msg/LaneletMapBin",
        ],
        name="map_echo",
    )  # fmt: skip

    def stop(action: ExecuteProcess) -> EmitEvent:
        return EmitEvent(
            event=SignalProcess(signal_number=signal.SIGINT, process_matcher=matches_action(action))
        )

    def stop_nodes(_event: ProcessExited, _context: LaunchContext) -> list:
        # ros2 bag record subscribes after it discovers the topic, later than echo
        return [
            TimerAction(
                period=1.0, actions=[stop(map_recorder), stop(map_loader), stop(map_projection)]
            )
        ]

    bag2bag_runner = ExecuteProcess(
        cmd=[
            "ros2", "run", "autoware_traffic_light_pipeline", "traffic_light_pipeline_bag2bag_runner",
            "--input-bag", Path(conf["t4_dataset_path"], "input_bag").as_posix(),
            "--input-bag", map_bag.as_posix(),
            "--output-bag", conf["result_bag_path"],
            "--camera", camera_numbers(conf["bag2bag_camera_launch"]),
            "--ml-model-path", conf["bag2bag_ml_model_path"],
            "--recognition-config", conf["bag2bag_recognition_config"],
            "--fusion-config", conf["bag2bag_fusion_config"],
        ],
        output="screen",
        name="traffic_light_bag2bag",
        additional_env={"OMP_WAIT_POLICY": "passive"},
    )  # fmt: skip

    return [
        map_projection,
        map_loader,
        map_recorder,
        wait_until_map_published,
        TimerAction(period=MAP_TIMEOUT_S, actions=[stop(wait_until_map_published)]),
        RegisterEventHandler(
            OnProcessExit(target_action=wait_until_map_published, on_exit=stop_nodes)
        ),
        RegisterEventHandler(OnProcessExit(target_action=map_recorder, on_exit=[bag2bag_runner])),
        RegisterEventHandler(
            OnProcessExit(target_action=bag2bag_runner, on_exit=lambda *_: shutil.rmtree(tmp_dir))
        ),
    ]


def launch_traffic_light_bag2bag() -> list:
    return [
        # A separate OpaqueFunction, since the declared arguments are set in the launch configurations
        # after the OpaqueFunction that returns them.
        OpaqueFunction(function=add_use_case_arguments),
        OpaqueFunction(function=run_bag2bag),
    ]
