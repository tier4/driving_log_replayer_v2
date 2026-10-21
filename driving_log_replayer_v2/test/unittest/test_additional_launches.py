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

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from pydantic import ValidationError
import pytest
import yaml

from driving_log_replayer_v2.additional_launches import AdditionalLaunch
from driving_log_replayer_v2.additional_launches import FAILURE_NAME
from driving_log_replayer_v2.additional_launches import finalize_additional_launch_failure
from driving_log_replayer_v2.additional_launches import initialize_run
from driving_log_replayer_v2.additional_launches import MANIFEST_NAME
from driving_log_replayer_v2.additional_launches import parse_additional_launches
from driving_log_replayer_v2.additional_launches import resolve_additional_launches
from driving_log_replayer_v2.additional_launches import write_json_atomically
from driving_log_replayer_v2.launch.additional_launches import AdditionalLaunchRun
from driving_log_replayer_v2.launch.additional_launches import launch_additional_launches
from driving_log_replayer_v2.launch.argument import ensure_arg_compatibility
from driving_log_replayer_v2.scenario import Scenario

SAMPLE_SCENARIOS = sorted((Path(__file__).parents[3] / "sample").glob("**/scenario*.yaml"))


@pytest.fixture
def scenario() -> dict:
    return {
        "ScenarioFormatVersion": "3.3.0",
        "ScenarioName": "additional-launch-test",
        "ScenarioDescription": "additional-launch-test",
        "SensorModel": "sample_sensor_kit",
        "VehicleModel": "sample_vehicle",
        "Evaluation": {"UseCaseName": "obstacle_segmentation", "UseCaseFormatVersion": "0.3.0"},
    }


@pytest.fixture
def package(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    share = tmp_path / "share" / "test_additional_launch"
    (share / "launch").mkdir(parents=True)
    (share / "launch" / "helper.launch.py").write_text("# test launch\n")
    (share / "package.xml").write_text("<package><version>1.2.3</version></package>")
    monkeypatch.setattr(
        "driving_log_replayer_v2.additional_launches.get_package_share_directory",
        lambda _: str(share),
    )
    return share


def configured_scenario(scenario: dict, **overrides: object) -> dict:
    scenario["additional_launches"] = json.dumps(
        [{"package": "test_additional_launch", "file": "helper.launch.py", **overrides}]
    )
    scenario["use_case"] = scenario["Evaluation"]["UseCaseName"]
    return scenario


@pytest.mark.parametrize("version", ["3.0.0", "3.1.0", "3.2.0", "3.3.0"])
@pytest.mark.parametrize("simulation", [None, {}, {"AdditionalLaunches": []}])
def test_legacy_and_empty_settings(scenario: dict, version: str, simulation: dict | None) -> None:
    scenario.update(ScenarioFormatVersion=version, Simulation=simulation)
    for use_case in ["obstacle_segmentation", "localization", "perception", "diagnostics"]:
        scenario["Evaluation"]["UseCaseName"] = use_case
        assert Scenario(**scenario).Evaluation == scenario["Evaluation"]
        assert parse_additional_launches(scenario) == []


@pytest.mark.parametrize(
    "overrides",
    [
        {"package": "../pkg"},
        {"file": "/absolute/helper.launch.py"},
        {"file": "../helper.launch.py"},
        {"file": "helper.py"},
        {"arguments": {"use_sim_time": True}},
        {"arguments": {"rate": 0.5}},
        {"arguments": {"--bad": "true"}},
        {"arguments": {"value": "\x00"}},
        {"arguments": None},
        {"typo": "value"},
    ],
)
def test_reject_invalid_entry(scenario: dict, overrides: dict) -> None:
    configured_scenario(scenario, **overrides)
    with pytest.raises(ValidationError):
        parse_additional_launches(scenario)


@pytest.mark.parametrize("settings", [{"AdditionalLaunches": None}, {"AdditionalLaunch": []}, None])
def test_reject_invalid_launch_json(scenario: dict, settings: object) -> None:
    scenario["additional_launches"] = json.dumps(settings)
    with pytest.raises(ValidationError):
        parse_additional_launches(scenario)


def test_support_constraints(scenario: dict) -> None:
    configured_scenario(scenario)
    with pytest.raises(ValueError, match="record_only=false"):
        parse_additional_launches(scenario, record_only=True)
    scenario["ScenarioFormatVersion"] = "3.2.0"
    assert parse_additional_launches(scenario)
    scenario["ScenarioFormatVersion"] = "3.3.0"
    scenario["use_case"] = "localization"
    with pytest.raises(ValueError, match="obstacle_segmentation"):
        parse_additional_launches(scenario)


def test_duplicate_entries(scenario: dict) -> None:
    configured_scenario(scenario)
    scenario["additional_launches"] = json.dumps(json.loads(scenario["additional_launches"]) * 2)
    with pytest.raises(ValidationError, match="identical"):
        parse_additional_launches(scenario)


def test_resolve_and_literal_arguments(scenario: dict, package: Path) -> None:
    configured_scenario(
        scenario, arguments={"use_sim_time": "true", "value": "$(touch /tmp/no); a b"}
    )
    launches = resolve_additional_launches(parse_additional_launches(scenario))
    item = launches[0]
    assert item.path == package / "launch" / "helper.launch.py"
    assert item.command()[-2:] == ["use_sim_time:=true", "value:=$(touch /tmp/no); a b"]
    assert item.manifest()["package_version"] == "1.2.3"
    assert item.manifest()["sha256"] == hashlib.sha256(item.path.read_bytes()).hexdigest()


@pytest.mark.parametrize(
    "path", SAMPLE_SCENARIOS, ids=lambda path: str(path.relative_to(path.parents[1]))
)
def test_existing_sample_scenarios(path: Path) -> None:
    scenario = yaml.safe_load(path.read_text())
    assert Scenario(**scenario).ScenarioFormatVersion == scenario["ScenarioFormatVersion"]


def test_obstacle_use_case_accepts_new_settings() -> None:
    obstacle_scenario = importlib.import_module(
        "driving_log_replayer_v2.obstacle_segmentation"
    ).ObstacleSegmentationScenario

    path = Path(__file__).parents[3] / "sample" / "obstacle_segmentation" / "scenario.yaml"
    scenario = yaml.safe_load(path.read_text())
    scenario["ScenarioFormatVersion"] = "3.3.0"
    configured_scenario(scenario)
    model = obstacle_scenario(**scenario)
    assert model.Evaluation.UseCaseFormatVersion == "0.3.0"
    assert parse_additional_launches(scenario)[0].package == "test_additional_launch"


def test_missing_package() -> None:
    with pytest.raises(PackageNotFoundError):
        resolve_additional_launches(
            [AdditionalLaunch(package="dlr_nonexistent_test_package", file="helper.launch.py")]
        )


def test_launch_validation_precedes_dataset_and_autoware(
    scenario: dict, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    configured_scenario(scenario, file="../escape.launch.py")
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario))
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
        additional_launches=scenario["additional_launches"],
    )
    # No Evaluation.Datasets or bag exists: reject the extension before touching either.
    with pytest.raises(ValidationError, match="relative launch"):
        ensure_arg_compatibility(context)


def test_missing_file_and_symlink_escape(package: Path, tmp_path: Path) -> None:
    specification = AdditionalLaunch(package="pkg", file="missing.launch.xml")
    with pytest.raises(ValueError, match="existing file"):
        resolve_additional_launches([specification])
    outside = tmp_path / "outside.launch.xml"
    outside.write_text("<launch/>")
    (package / "launch" / specification.file).symlink_to(outside)
    with pytest.raises(ValueError, match="existing file"):
        resolve_additional_launches([specification])


def test_launch_directory_symlink_install(package: Path, tmp_path: Path) -> None:
    source = tmp_path / "source_launch"
    (package / "launch").rename(source)
    (package / "launch").symlink_to(source, target_is_directory=True)
    resolved = resolve_additional_launches(
        [AdditionalLaunch(package="pkg", file="helper.launch.py")]
    )
    assert resolved[0].path == source / "helper.launch.py"


def test_launch_file_symlink_install(package: Path, tmp_path: Path) -> None:
    source = tmp_path / "source_package"
    (source / "launch").mkdir(parents=True)
    source_file = source / "launch" / "helper.launch.py"
    source_file.write_text("# source launch\n")
    (package / "package.xml").rename(source / "package.xml")
    (package / "package.xml").symlink_to(source / "package.xml")
    installed_file = package / "launch" / "helper.launch.py"
    installed_file.unlink()
    installed_file.symlink_to(source_file)
    specification = AdditionalLaunch(package="pkg", file="helper.launch.py")
    resolved = resolve_additional_launches([specification])
    assert resolved[0].path == source_file
    assert resolved[0].manifest()["package_version"] == "1.2.3"

    outside = tmp_path / "outside.launch.py"
    outside.write_text("# outside the source package\n")
    source_file.unlink()
    source_file.symlink_to(outside)
    with pytest.raises(ValueError, match="existing file"):
        resolve_additional_launches([specification])


def test_recursive_launch(package: Path) -> None:
    (package / "launch" / "simulation.launch.py").write_text("# recursive")
    with pytest.raises(ValueError, match="recursively"):
        resolve_additional_launches(
            [AdditionalLaunch(package="driving_log_replayer_v2", file="simulation.launch.py")]
        )


def test_unconfigured_runner_does_not_resolve_packages(
    scenario: dict, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def unexpected_lookup(_: str) -> None:
        pytest.fail("Unconfigured run looked up a package")

    monkeypatch.setattr(
        "driving_log_replayer_v2.additional_launches.get_package_share_directory", unexpected_lookup
    )
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario))
    context = LaunchContext()
    context.launch_configurations.update(
        use_case="obstacle_segmentation", scenario_path=str(scenario_path), record_only="false"
    )
    assert launch_additional_launches(context) == []
    context.launch_configurations.update(use_case="localization", scenario_path="/does/not/exist")
    assert launch_additional_launches(context) == []


@pytest.mark.parametrize("returncode", [0, 1, -15])
@pytest.mark.usefixtures("package")
def test_exit_marks_failure_and_post_process_overrides_success(
    scenario: dict, tmp_path: Path, returncode: int
) -> None:
    configured_scenario(scenario)
    launches = resolve_additional_launches(parse_additional_launches(scenario))
    archive = tmp_path / "result_archive"
    run_id = initialize_run(archive, launches)
    state = AdditionalLaunchRun(archive, run_id)
    event = SimpleNamespace(returncode=returncode, process_name="test_helper")
    actions = state.on_exit(launches[0], event, LaunchContext())
    assert len(actions) == 1
    assert state.on_exit(launches[0], event, LaunchContext()) == []
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario))
    result = tmp_path / "result.jsonl"
    result.write_text(json.dumps({"Result": {"Success": True}, "Frame": {}, "Stamp": {}}) + "\n")
    assert finalize_additional_launch_failure(str(scenario_path), str(archive), str(result))
    records = [json.loads(line) for line in result.read_text().splitlines()]
    assert records[0]["Result"]["Success"]
    assert records[-1]["Result"] == {"Success": False, "Summary": "AdditionalLaunchFailed"}
    assert records[-1]["Frame"]["ErrorMsg"]["returncode"] == returncode


@pytest.mark.usefixtures("package")
def test_shutdown_is_not_failure(tmp_path: Path) -> None:
    launches = resolve_additional_launches(
        [AdditionalLaunch(package="pkg", file="helper.launch.py")]
    )
    state = AdditionalLaunchRun(tmp_path, "run")
    context = LaunchContext()
    state.on_shutdown(None, context)
    assert state.on_exit(launches[0], SimpleNamespace(), context) == []
    assert not (tmp_path / FAILURE_NAME).exists()


@pytest.mark.usefixtures("package")
def test_empty_result_failure_and_stale_marker(scenario: dict, tmp_path: Path) -> None:
    configured_scenario(scenario)
    launches = resolve_additional_launches(parse_additional_launches(scenario))
    run_id = initialize_run(tmp_path, launches)
    failure = {"run_id": "old", "returncode": 1}
    write_json_atomically(tmp_path / FAILURE_NAME, failure)
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario))
    result_path = tmp_path / "result.jsonl"
    assert not finalize_additional_launch_failure(
        str(scenario_path), str(tmp_path), str(result_path)
    )
    failure["run_id"] = run_id
    write_json_atomically(tmp_path / FAILURE_NAME, failure)
    assert finalize_additional_launch_failure(str(scenario_path), str(tmp_path), str(result_path))
    assert json.loads(result_path.read_text().splitlines()[-1])["Result"]["Success"] is False
    initialize_run(tmp_path, launches)
    assert not (tmp_path / FAILURE_NAME).exists()
    assert json.loads((tmp_path / MANIFEST_NAME).read_text())["run_id"] != run_id


@pytest.mark.usefixtures("package")
def test_post_process_failure_precedes_missing_bag(
    scenario: dict, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    configured_scenario(scenario)
    launches = resolve_additional_launches(parse_additional_launches(scenario))
    run_id = initialize_run(tmp_path, launches)
    write_json_atomically(tmp_path / FAILURE_NAME, {"run_id": run_id, "returncode": 1})
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario))
    path = Path(__file__).parents[2] / "launch" / "post_process.launch.py"
    spec = importlib.util.spec_from_file_location("additional_launch_post_process", path)
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolve their module through sys.modules.
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        use_case="obstacle_segmentation",
        scenario_path=str(scenario_path),
        result_archive_path=str(tmp_path),
        result_jsonl_path=str(tmp_path / "result.jsonl"),
        result_bag_path=str(tmp_path / "missing_bag"),
        additional_launches=scenario["additional_launches"],
    )
    assert module.post_process(context)
    assert json.loads((tmp_path / "result.jsonl").read_text().splitlines()[-1])["Result"] == {
        "Success": False,
        "Summary": "AdditionalLaunchFailed",
    }
