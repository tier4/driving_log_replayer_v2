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

"""Scenario contracts and run artifacts for optional, externally supplied launches."""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time
from typing import Annotated
from uuid import uuid4
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from pydantic import BaseModel
from pydantic import ConfigDict
from pydantic import Field
from pydantic import field_validator
from pydantic import model_validator
from pydantic import StrictStr
from pydantic import StringConstraints
import yaml

Identifier = Annotated[StrictStr, StringConstraints(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")]
MANIFEST_NAME = "additional_launches.json"
FAILURE_NAME = "additional_launch_failure.json"


class AdditionalLaunch(BaseModel):
    model_config = ConfigDict(extra="forbid")

    package: Annotated[StrictStr, StringConstraints(pattern=r"^[A-Za-z][A-Za-z0-9_]*$")]
    file: StrictStr
    arguments: dict[Identifier, StrictStr] = Field(default_factory=dict)

    @field_validator("file")
    @classmethod
    def validate_file(cls, value: str) -> str:
        path = Path(value)
        if (
            path.is_absolute()
            or ".." in path.parts
            or "\x00" in value
            or not value.endswith((".launch.py", ".launch.xml", ".launch.yaml", ".launch.yml"))
        ):
            msg = "file must be a relative launch file below the package's launch directory"
            raise ValueError(msg)
        return value

    @field_validator("arguments")
    @classmethod
    def validate_arguments(cls, value: dict[str, str]) -> dict[str, str]:
        if any("\x00" in argument for argument in value.values()):
            msg = "launch arguments cannot contain NUL characters"
            raise ValueError(msg)
        return value


class SimulationSettings(BaseModel):
    model_config = ConfigDict(extra="forbid")

    AdditionalLaunches: list[AdditionalLaunch] = Field(default_factory=list)

    @model_validator(mode="after")
    def unique_launches(self) -> "SimulationSettings":
        keys = [
            (item.package, item.file, tuple(sorted(item.arguments.items())))
            for item in self.AdditionalLaunches
        ]
        if len(keys) != len(set(keys)):
            msg = "identical AdditionalLaunches entries are not allowed"
            raise ValueError(msg)
        return self


def validate_launch_support(
    version: str, use_case: str, launches: list[AdditionalLaunch], *, record_only: bool = False
) -> None:
    if not launches:
        return
    if version != "3.3.0":
        msg = "Simulation.AdditionalLaunches requires ScenarioFormatVersion 3.3.0"
        raise ValueError(msg)
    if use_case != "obstacle_segmentation" or record_only:
        msg = "AdditionalLaunches currently supports obstacle_segmentation with record_only=false"
        raise ValueError(msg)


def parse_additional_launches(
    scenario: dict, *, record_only: bool = False
) -> list[AdditionalLaunch]:
    simulation = scenario.get("Simulation")
    if simulation is None:
        return []
    settings = SimulationSettings.model_validate(simulation)
    validate_launch_support(
        scenario.get("ScenarioFormatVersion", ""),
        scenario.get("Evaluation", {}).get("UseCaseName", ""),
        settings.AdditionalLaunches,
        record_only=record_only,
    )
    return settings.AdditionalLaunches


@dataclass(frozen=True)
class ResolvedLaunch:
    specification: AdditionalLaunch
    path: Path
    package_version: str | None

    def command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            str(self.path),
            *(f"{key}:={value}" for key, value in self.specification.arguments.items()),
        ]

    def manifest(self) -> dict:
        return {
            **self.specification.model_dump(),
            "resolved_file": str(self.path),
            "sha256": hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "package_version": self.package_version,
        }


def resolve_additional_launches(launches: list[AdditionalLaunch]) -> list[ResolvedLaunch]:
    resolved = []
    seen = set()
    for specification in launches:
        share = Path(get_package_share_directory(specification.package)).resolve()
        # Colcon --symlink-install may make share/launch a symlink into the source tree.
        launch_dir = (share / "launch").resolve()
        path = (launch_dir / specification.file).resolve()
        if not path.is_relative_to(launch_dir) or not path.is_file():
            msg = f"Additional launch must be an existing file below {launch_dir}: {path}"
            raise ValueError(msg)
        if specification.package == "driving_log_replayer_v2" and path.name in {
            "driving_log_replayer_v2.launch.py",
            "simulation.launch.py",
        }:
            msg = "AdditionalLaunches cannot recursively launch DLR"
            raise ValueError(msg)
        key = (path, tuple(sorted(specification.arguments.items())))
        if key in seen:
            msg = f"Duplicate resolved additional launch: {path}"
            raise ValueError(msg)
        seen.add(key)
        package_xml = share / "package.xml"
        # Read installed package metadata, not XML supplied in the scenario.
        version = None
        if package_xml.is_file():
            version = ET.parse(package_xml).getroot().findtext("version")  # noqa: S314
        resolved.append(ResolvedLaunch(specification, path, version))
    return resolved


def load_additional_launches(
    scenario_path: str, *, record_only: bool = False
) -> list[ResolvedLaunch]:
    with Path(scenario_path).open() as stream:
        scenario = yaml.safe_load(stream)
    return resolve_additional_launches(parse_additional_launches(scenario, record_only=record_only))


def write_json_atomically(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        temporary_path.write_text(json.dumps(value, indent=2) + "\n")
        temporary_path.replace(path)
    finally:
        temporary_path.unlink(missing_ok=True)


def initialize_run(archive_path: Path, launches: list[ResolvedLaunch]) -> str:
    run_id = uuid4().hex
    write_json_atomically(
        archive_path / MANIFEST_NAME,
        {"run_id": run_id, "launches": [item.manifest() for item in launches]},
    )
    (archive_path / FAILURE_NAME).unlink(missing_ok=True)
    return run_id


def finalize_additional_launch_failure(
    scenario_path: str, archive_path: str, result_path: str
) -> bool:
    """Append a terminal failure after all simulation writers have stopped, before bag analysis."""
    with Path(scenario_path).open() as stream:
        scenario = yaml.safe_load(stream)
    if not parse_additional_launches(scenario):
        return False
    archive = Path(archive_path)
    failure_path = archive / FAILURE_NAME
    if not failure_path.is_file():
        return False
    failure = json.loads(failure_path.read_text())
    manifest = json.loads((archive / MANIFEST_NAME).read_text())
    if failure["run_id"] != manifest["run_id"]:
        return False
    result = Path(result_path)
    result.parent.mkdir(parents=True, exist_ok=True)
    needs_newline = False
    if result.is_file() and result.stat().st_size:
        with result.open("rb") as previous:
            previous.seek(-1, 2)
            needs_newline = previous.read(1) != b"\n"
    # Keep all evaluation frames. A System-only terminal record is not plotted as a sensor frame.
    with result.open("a+") as stream:
        stream.seek(0, 2)
        if stream.tell() == 0:
            stream.write(
                json.dumps({"Condition": scenario["Evaluation"].get("Conditions", {})}) + "\n"
            )
        elif needs_newline:
            stream.write("\n")
        stream.write(
            json.dumps(
                {
                    "Result": {"Success": False, "Summary": "AdditionalLaunchFailed"},
                    "Stamp": {"System": time.time()},
                    "Frame": {"ErrorMsg": failure},
                }
            )
            + "\n"
        )
    return True
