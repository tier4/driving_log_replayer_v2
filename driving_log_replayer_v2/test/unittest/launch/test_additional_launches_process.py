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

"""Exercise real ros2 launch children without requiring a DDS participant or Autoware."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from driving_log_replayer_v2.additional_launches import FAILURE_NAME
from driving_log_replayer_v2.additional_launches import finalize_additional_launch_failure
from driving_log_replayer_v2.additional_launches import MANIFEST_NAME

HELPER = """from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "error":
        raise RuntimeError("intentional child launch error")
    if mode == "empty":
        return []
    marker = LaunchConfiguration("marker").perform(context)
    value = LaunchConfiguration("value").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context)
    code = (
        "import json, os, signal, sys, time; from pathlib import Path; "
        "signal.signal(signal.SIGINT, lambda *args: "
        "(Path(sys.argv[1] + '.stopped').write_text('stopped'), sys.exit(0))); "
        "Path(sys.argv[1]).write_text(json.dumps("
        "{'pid': os.getpid(), 'value': sys.argv[2], 'use_sim_time': sys.argv[3]})); "
        + ("sys.exit(7)" if mode == "required_exit" else "time.sleep(30)")
    )
    return [ExecuteProcess(cmd=[sys.executable, "-u", "-c", code, marker, value, sim_time],
                           on_exit=Shutdown(reason="required helper exited"))]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode"), DeclareLaunchArgument("marker"),
        DeclareLaunchArgument("value"), DeclareLaunchArgument("use_sim_time"),
        OpaqueFunction(function=setup),
    ])
"""

PARENT = """import sys
from pathlib import Path
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, TimerAction
from driving_log_replayer_v2.launch.additional_launches import launch_additional_launches
from driving_log_replayer_v2.shutdown_once import ShutdownOnce

directory = Path(sys.argv[1])
service = LaunchService()
service.context.launch_configurations.update(
    use_case="obstacle_segmentation", record_only="false",
    scenario_path=str(directory / "scenario.yaml"),
    additional_launches=(directory / "launches.json").read_text(),
    result_archive_path=str(directory / "archive"),
)
def stop_when_ready(context):
    if (directory / "child").is_file():
        return [ShutdownOnce(reason="test evaluation completed")]
    return [TimerAction(period=0.05, actions=[OpaqueFunction(function=stop_when_ready)])]

actions = [OpaqueFunction(function=launch_additional_launches)]
if sys.argv[2] == "normal":
    actions.append(OpaqueFunction(function=stop_when_ready))
actions.append(TimerAction(period=10.0, actions=[ShutdownOnce(reason="test watchdog")]))
service.include_launch_description(LaunchDescription(actions))
sys.exit(service.run())
"""


@pytest.fixture
def setup_run(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    prefix = tmp_path / "install"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "test_additional_launch").touch()
    launch_dir = prefix / "share" / "test_additional_launch" / "launch"
    launch_dir.mkdir(parents=True)
    (launch_dir / "helper.launch.py").write_text(HELPER)
    monkeypatch.setenv(
        "AMENT_PREFIX_PATH", str(prefix) + os.pathsep + os.getenv("AMENT_PREFIX_PATH", "")
    )
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    return tmp_path


def write_scenario(directory: Path, mode: str, *, sibling: bool = False) -> None:
    def specification(name: str, launch_mode: str) -> dict:
        return {
            "package": "test_additional_launch",
            "file": "helper.launch.py",
            "arguments": {
                "mode": launch_mode,
                "marker": str(directory / name),
                "value": "space ; $(not_a_command) \\ literal",
                "use_sim_time": "true",
            },
        }

    launches = [specification("child", mode)]
    if sibling:
        launches.insert(0, specification("sibling", "hold"))
    (directory / "launches.json").write_text(json.dumps(launches))
    (directory / "scenario.yaml").write_text(
        yaml.safe_dump(
            {
                "ScenarioFormatVersion": "3.3.0",
                "Evaluation": {"UseCaseName": "obstacle_segmentation"},
            }
        )
    )


@pytest.mark.parametrize("mode", ["empty", "error", "required_exit"])
def test_real_child_exit_is_failure(setup_run: Path, mode: str) -> None:
    directory = setup_run
    write_scenario(directory, mode, sibling=True)
    subprocess.run(
        [sys.executable, "-c", PARENT, str(directory), "failure"], check=True, timeout=30
    )
    failure = json.loads((directory / "archive" / FAILURE_NAME).read_text())
    assert failure["launch"]["arguments"]["mode"] == mode
    assert finalize_additional_launch_failure(
        str(directory / "scenario.yaml"),
        str(directory / "archive"),
        str(directory / "result.jsonl"),
    )
    assert json.loads((directory / "result.jsonl").read_text().splitlines()[-1])["Result"] == {
        "Success": False,
        "Summary": "AdditionalLaunchFailed",
    }
    # Any helper that reached its ready marker must also have received shutdown.
    if (directory / "sibling").exists():
        assert (directory / "sibling.stopped").exists()


def test_real_normal_shutdown_stops_child_without_failure(setup_run: Path) -> None:
    directory = setup_run
    write_scenario(directory, "hold")

    subprocess.run([sys.executable, "-c", PARENT, str(directory), "normal"], check=True, timeout=30)
    assert (directory / "child.stopped").exists()
    actual = json.loads((directory / "child").read_text())
    assert actual["value"] == "space ; $(not_a_command) \\ literal"
    assert actual["use_sim_time"] == "true"
    assert not (directory / "archive" / FAILURE_NAME).exists()
    assert (directory / "archive" / MANIFEST_NAME).is_file()
