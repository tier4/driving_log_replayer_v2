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

import importlib.util
import json
from pathlib import Path
import re
from types import SimpleNamespace
from unittest.mock import Mock

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from launch import LaunchContext
import pytest
import yaml

from driving_log_replayer_v2.evaluation_topic import CONFIG_KEY
from driving_log_replayer_v2.evaluation_topic import DEFAULT_POINTCLOUD_TOPIC
from driving_log_replayer_v2.evaluation_topic import DIAGNOSTIC_WARNING_INTERVAL_SEC
from driving_log_replayer_v2.evaluation_topic import EvaluationTopic
from driving_log_replayer_v2.evaluation_topic import parse_evaluation_topic
from driving_log_replayer_v2.evaluation_topic import TopicRateDiagnosticState
from driving_log_replayer_v2.evaluation_topic import TopicRateMonitor
from driving_log_replayer_v2.launch import profile
from driving_log_replayer_v2.launch.argument import ensure_arg_compatibility
from driving_log_replayer_v2.launch.rosbag import launch_bag_recorder
from driving_log_replayer_v2.launch.use_case import launch_evaluator_node
from driving_log_replayer_v2.obstacle_segmentation import ObstacleSegmentationScenario
from driving_log_replayer_v2.scenario import Scenario
from driving_log_replayer_v2_analyzer.data import DistType
from driving_log_replayer_v2_analyzer.data.obstacle_segmentation import JsonlParser

TARGET = "/perception/radar_lidar_fusion/debug/appended_radar_points"
MONITOR = {"module": "perception", "node_name_suffix": "obstacle_segmentation_pointcloud"}


def conventional_scenario() -> dict:
    return {
        "ScenarioFormatVersion": "3.2.0",
        "ScenarioName": "target",
        "ScenarioDescription": "target",
        "SensorModel": "sample_sensor_kit",
        "VehicleModel": "sample_vehicle",
        "Evaluation": {"UseCaseName": "obstacle_segmentation"},
    }


@pytest.mark.parametrize("version", ["3.0.0", "3.1.0", "3.2.0", "3.3.0"])
def test_launch_settings_do_not_require_scenario_version(version: str) -> None:
    scenario = conventional_scenario()
    scenario["ScenarioFormatVersion"] = version
    Scenario(**scenario)
    target = parse_evaluation_topic(
        {"use_case": "obstacle_segmentation", "evaluation_topic": TARGET}
    )
    assert target.topic == TARGET
    assert target.topic_rate is None


@pytest.mark.parametrize(
    ("topic", "rate", "expected_topic", "expected_monitor"),
    [
        ("", "legacy", None, None),
        (TARGET, "legacy", TARGET, None),
        (TARGET, "null", TARGET, None),
        ("", "null", DEFAULT_POINTCLOUD_TOPIC, None),
        ("", json.dumps(MONITOR), DEFAULT_POINTCLOUD_TOPIC, MONITOR),
        (TARGET, json.dumps(MONITOR), TARGET, MONITOR),
    ],
)
def test_defaults_and_explicit_null(
    topic: str, rate: str, expected_topic: str | None, expected_monitor: dict | None
) -> None:
    target = parse_evaluation_topic(
        {
            "use_case": "obstacle_segmentation",
            "evaluation_topic": topic,
            "evaluation_topic_rate": rate,
        }
    )
    if expected_topic is None:
        assert target is None
    else:
        assert target.topic == expected_topic
        actual = target.topic_rate.model_dump() if target.topic_rate is not None else None
        assert actual == expected_monitor


@pytest.mark.parametrize(
    "settings",
    [
        {"evaluation_topic": "relative"},
        {"evaluation_topic": "~/private"},
        {"evaluation_topic": "/bad*"},
        {"evaluation_topic": "/a:=/b"},
        {"evaluation_topic_rate": "{}"},
        {"evaluation_topic_rate": "false"},
        {"evaluation_topic_rate": "not-json"},
        {"evaluation_topic_rate": json.dumps({"module": "perception"})},
        {"evaluation_topic_rate": json.dumps({**MONITOR, "extra": True})},
        {"evaluation_topic_rate": json.dumps({**MONITOR, "module": ""})},
    ],
)
def test_invalid_launch_settings(settings: dict) -> None:
    with pytest.raises(ValueError, match=r"topic|validation|JSON|Expecting"):
        parse_evaluation_topic({"use_case": "obstacle_segmentation", **settings})


@pytest.mark.parametrize("use_case", ["localization", "perception", "ndt_convergence"])
def test_reject_other_use_cases(use_case: str) -> None:
    with pytest.raises(ValueError, match="only obstacle_segmentation"):
        parse_evaluation_topic({"use_case": use_case, "evaluation_topic": TARGET})
    assert parse_evaluation_topic({"use_case": use_case}) is None


def test_reject_record_only() -> None:
    with pytest.raises(ValueError, match="record_only=false"):
        parse_evaluation_topic(
            {"use_case": "obstacle_segmentation", "evaluation_topic": TARGET}, record_only=True
        )


@pytest.mark.parametrize(
    "old_setting",
    [
        {"Simulation": {"AdditionalLaunches": [{"package": "pkg", "file": "file.launch.xml"}]}},
        {
            "Evaluation": {
                "UseCaseName": "obstacle_segmentation",
                "EvaluationTopic": {"topic": TARGET},
            }
        },
    ],
)
def test_reject_removed_scenario_settings(old_setting: dict) -> None:
    with pytest.raises(ValueError, match="Move"):
        Scenario(**{**conventional_scenario(), **old_setting})


def test_validation_before_dataset_access(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(conventional_scenario()))
    monkeypatch.setattr(
        "driving_log_replayer_v2.launch.argument.prepare_paths",
        lambda _: (path, tmp_path, tmp_path),
    )
    context = LaunchContext()
    context.launch_configurations.update(
        dataset_dir="",
        t4_dataset_path="",
        remap_arg="",
        remap_profile="",
        record_only="false",
        evaluation_topic="relative",
    )
    with pytest.raises(ValueError, match="topic"):
        ensure_arg_compatibility(context)


def test_typed_scenario_remains_unmodified() -> None:
    path = Path(__file__).parents[3] / "sample/obstacle_segmentation/scenario.yaml"
    raw = yaml.safe_load(path.read_text())
    scenario = ObstacleSegmentationScenario(**raw)
    assert scenario.Evaluation.UseCaseFormatVersion == "0.3.0"
    assert "EvaluationTopic" not in scenario.Evaluation.model_dump()


def diagnostic(level: int, stamp: float = 10.0, *, name: str | None = None) -> DiagnosticArray:
    msg = DiagnosticArray()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    msg.status = [
        DiagnosticStatus(
            name=name or TopicRateMonitor(**MONITOR).diagnostic_name, level=bytes([level])
        )
    ]
    return msg


def state() -> TopicRateDiagnosticState:
    return TopicRateDiagnosticState(TopicRateMonitor(**MONITOR))


@pytest.mark.parametrize(
    ("level", "healthy", "label"),
    [(0, True, "OK"), (1, True, "WARN"), (2, False, "ERROR"), (3, False, "STALE")],
)
def test_diagnostic_levels(level: int, *, healthy: bool, label: str) -> None:
    rate = state()
    msg = diagnostic(level)
    msg.status.insert(0, DiagnosticStatus(name="unrelated", level=bytes([0])))
    rate.update(msg, 10.0)
    result, metadata = rate.snapshot(10.5)
    assert result is healthy
    assert metadata["state"] == label


def test_disabled_is_skipped_even_with_errors() -> None:
    rate = TopicRateDiagnosticState(None)
    rate.update(diagnostic(2), 10.0)
    assert rate.snapshot(10.0) == (None, {"enabled": False, "state": "Skipped"})


def test_not_received_and_unrelated_diagnostics() -> None:
    rate = state()
    rate.update(diagnostic(0, name="other"), 10.0)
    assert rate.snapshot(10.0)[0] is False
    assert rate.snapshot(10.0)[1]["reason"] == "NotReceived"


def test_timeout_unrelated_and_pause() -> None:
    rate = state()
    rate.update(diagnostic(0), 10.0)
    assert rate.snapshot(10.0)[0] is True
    assert rate.snapshot(10.0)[0] is True  # Paused ROS time.
    rate.update(diagnostic(0, stamp=11.0, name="other"), 11.0)
    assert rate.snapshot(12.01) == (
        False,
        {
            "enabled": True,
            "diagnostic_name": TopicRateMonitor(**MONITOR).diagnostic_name,
            "state": "Invalid",
            "reason": "Expired",
        },
    )


def test_rollback_clears_ok() -> None:
    rate = state()
    rate.update(diagnostic(0), 10.0)
    assert rate.snapshot(10.0)[0] is True
    assert rate.snapshot(9.0)[1]["reason"] == "NotReceived"


def test_older_ok_does_not_replace_error_and_can_recover() -> None:
    rate = state()
    rate.update(diagnostic(2, stamp=11), 11)
    rate.update(diagnostic(0, stamp=10), 11.1)
    assert rate.snapshot(11.1)[0] is False
    rate.update(diagnostic(0, stamp=12), 12)
    assert rate.snapshot(12)[0] is True  # No permanent failure latch.


def test_duplicate_names_and_equal_stamp_use_worst_level() -> None:
    rate = state()
    msg = diagnostic(0)
    msg.status.extend(diagnostic(2).status)
    rate.update(msg, 10.0)
    rate.update(diagnostic(0), 10.0)
    assert rate.snapshot(10.0)[0] is False


@pytest.mark.parametrize("stamp", [0, 11])
def test_invalid_stamp_does_not_establish_health(stamp: float) -> None:
    rate = state()
    rate.update(diagnostic(0, stamp=stamp), 10)
    assert rate.snapshot(10)[0] is False


@pytest.mark.parametrize("monitored", [False, True])
@pytest.mark.parametrize("base", ["", "^/tf$|^/custom$", "^/diagnostics$|^/perception/.*"])
def test_recording_union(monkeypatch: pytest.MonkeyPatch, *, monitored: bool, base: str) -> None:
    target = EvaluationTopic(topic=TARGET, topic_rate=MONITOR if monitored else None)
    monkeypatch.setattr(profile, "build_base_record_topic_regex", lambda _: base)
    regex = profile.build_record_topic_regex({CONFIG_KEY: target.model_dump_json()})
    assert re.fullmatch(regex, TARGET)
    assert bool(re.fullmatch(regex, "/diagnostics")) is (
        monitored or bool(re.fullmatch(base, "/diagnostics"))
    )
    assert not re.fullmatch(regex, "/unused" + TARGET)
    assert not re.fullmatch(regex, "/unrelated")
    if base == "^/tf$|^/custom$":
        assert re.fullmatch(regex, "/tf")
        assert re.fullmatch(regex, "/custom")
        assert not re.fullmatch(regex, DEFAULT_POINTCLOUD_TOPIC)
    if not base:
        assert not re.fullmatch(regex, TARGET + "_suffix")


def test_evaluator_remap_and_legacy(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("driving_log_replayer_v2.launch.use_case.Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(
        use_case="obstacle_segmentation",
        record_only="false",
        scenario_path="s",
        t4_dataset_path="d",
        result_jsonl_path="r",
        result_archive_path="a",
        dataset_index="0",
        input_bag="b",
    )
    assert launch_evaluator_node(context)[0]["remappings"] == []
    context.launch_configurations[CONFIG_KEY] = json.dumps({"topic": TARGET})
    action = launch_evaluator_node(context)[0]
    assert action["remappings"] == [(DEFAULT_POINTCLOUD_TOPIC, TARGET)]
    assert action["executable"] == "obstacle_segmentation_evaluator_node.py"
    serialized = action["parameters"][0]["evaluation_topic_config"].evaluate(context)
    assert json.loads(serialized)["topic"] == TARGET


def test_actual_recorder_command_uses_configured_union(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    commands = []
    monkeypatch.setattr(
        "driving_log_replayer_v2.launch.rosbag.ExecuteProcess",
        lambda **kwargs: commands.append(kwargs["cmd"]),
    )
    context = LaunchContext()
    context.launch_configurations.update(
        use_case="obstacle_segmentation",
        storage="mcap",
        result_bag_path=str(tmp_path / "bag"),
        override_topics_regex="^/tf$",
        **{CONFIG_KEY: EvaluationTopic(topic=TARGET, topic_rate=MONITOR).model_dump_json()},
    )
    launch_bag_recorder(context)
    command = commands[0]
    pattern = command[command.index("-e") + 1]
    for topic in (TARGET, "/diagnostics", "/tf"):
        assert re.fullmatch(pattern, topic)
    assert not re.fullmatch(pattern, DEFAULT_POINTCLOUD_TOPIC)
    assert "--use-sim-time" in command


def test_analyzer_preserves_skipped_and_legacy_output(tmp_path: Path) -> None:
    path = tmp_path / "result.jsonl"
    row = {
        "Frame": {
            "FrameName": "1",
            "TopicRate": None,
            "TopicRateEvaluation": {"enabled": False, "state": "Skipped"},
        }
    }
    path.write_text(json.dumps(row) + "\n")
    parser = JsonlParser(path, SimpleNamespace(overhang_from_baselink=0), DistType.EUCLID)
    destination = tmp_path / "rate.json"
    parser.save_topic_rate_evaluations(destination)
    data = json.loads(destination.read_text())
    assert data[0]["topic_rate"] is None
    assert data[0]["state"] == "Skipped"
    assert data[0]["enabled"] is False
    del row["Frame"]["TopicRateEvaluation"]
    row["Frame"]["TopicRate"] = "True"
    path.write_text(json.dumps(row) + "\n")
    parser = JsonlParser(path, SimpleNamespace(overhang_from_baselink=0), DistType.EUCLID)
    legacy_destination = tmp_path / "legacy-rate.json"
    parser.save_topic_rate_evaluations(legacy_destination)
    assert not legacy_destination.exists()


def test_public_arguments_forward_as_single_values() -> None:
    path = Path(__file__).parents[2] / "launch/driving_log_replayer_v2.launch.py"
    spec = importlib.util.spec_from_file_location("public_launch_arguments", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    launch_json = json.dumps(
        [
            {
                "package": "pkg",
                "file": "helper.launch.py",
                "arguments": {"value": "space ; $(literal)"},
            }
        ]
    )
    context.launch_configurations.update(
        additional_launches=launch_json, evaluation_topic=TARGET, evaluation_topic_rate="null"
    )
    forwarded = module.parse_launch_arguments(context)
    assert f"additional_launches:={launch_json}" in forwarded
    assert "evaluation_topic_rate:=null" in forwarded
    assert f"evaluation_topic:={TARGET}" in forwarded


def test_missing_diagnostic_warns_and_reports_invalid() -> None:
    path = Path(__file__).parents[2] / "scripts/obstacle_segmentation_evaluator_node.py"
    spec = importlib.util.spec_from_file_location("configured_rate_evaluator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    logger = Mock()
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    evaluator = SimpleNamespace(
        _ObstacleSegmentationEvaluator__rate_state=state(),
        get_clock=lambda: clock,
        get_logger=lambda: logger,
    )
    healthy, metadata = module.ObstacleSegmentationEvaluator.current_topic_rate(evaluator)
    assert healthy is False
    assert metadata["state"] == "Invalid"
    assert metadata["reason"] == "NotReceived"
    assert "TopicRate Invalid" in logger.warning.call_args.args[0]
    assert (
        logger.warning.call_args.kwargs["throttle_duration_sec"] == DIAGNOSTIC_WARNING_INTERVAL_SEC
    )


def test_legacy_diagnostic_behavior_is_preserved() -> None:
    path = Path(__file__).parents[2] / "scripts/obstacle_segmentation_evaluator_node.py"
    spec = importlib.util.spec_from_file_location("legacy_rate_evaluator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    evaluator = SimpleNamespace(
        _ObstacleSegmentationEvaluator__rate_state=None,
        _ObstacleSegmentationEvaluator__topic_rate=False,
    )
    msg = diagnostic(0)
    msg.status.insert(0, DiagnosticStatus(name="unrelated"))
    module.ObstacleSegmentationEvaluator.diag_cb(evaluator, msg)
    assert module.ObstacleSegmentationEvaluator.current_topic_rate(evaluator) == (False, None)
    msg.status = msg.status[1:]
    module.ObstacleSegmentationEvaluator.diag_cb(evaluator, msg)
    assert module.ObstacleSegmentationEvaluator.current_topic_rate(evaluator) == (True, None)
