# Copyright (c) 2022 TIER IV.inc
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

from collections.abc import Callable
from pathlib import Path
from typing import Any
from typing import Literal
from typing import TypeVar

from ament_index_python.packages import get_package_share_directory
from pydantic import BaseModel
from pydantic import ValidationError
from rclpy.clock import Clock
import yaml

from driving_log_replayer_v2.result import ResultWriter

number = int | float


class Scenario(BaseModel):
    ScenarioFormatVersion: Literal["3.0.0", "3.1.0", "3.2.0"]
    ScenarioName: str
    ScenarioDescription: str
    SensorModel: str
    VehicleModel: str
    VehicleId: str | None = None
    Evaluation: dict
    include_use_case: dict | None = None
    publish_profile: str | None = None


def load_scenario(scenario_path: Path, scenario_class: Callable) -> Any:
    if scenario_path.is_symlink():
        scenario_path = scenario_path.resolve()
    with scenario_path.open() as scenario_file:
        return scenario_class(**yaml.safe_load(scenario_file))


def load_sample_scenario(
    use_case_name: str,
    scenario_class: Callable,
    scenario_name: str = "scenario.yaml",
) -> Any:
    sample_scenario_path = Path(
        get_package_share_directory("driving_log_replayer_v2"),
        "sample",
        use_case_name,
        scenario_name,
    )
    return load_scenario(sample_scenario_path, scenario_class)


ScenarioType = TypeVar("ScenarioType", bound=Scenario)


def load_scenario_with_exception(
    scenario_path: str, scenario_class: ScenarioType, result_json_path: str
) -> ScenarioType:
    try:
        return load_scenario(Path(scenario_path), scenario_class)
    except (FileNotFoundError, PermissionError, yaml.YAMLError, ValidationError) as e:
        result_writer = ResultWriter(
            result_json_path,
            Clock(),
            {},
        )
        error_dict = {
            "Result": {"Success": False, "Summary": "ScenarioFormatError"},
            "Stamp": {"System": 0.0},
            "Frame": {"ErrorMsg": e.__str__()},
        }
        result_writer.write_line(error_dict)
        result_writer.close()
        raise


def load_condition(scenario: ScenarioType) -> dict:
    if hasattr(scenario.Evaluation, "Conditions") and scenario.Evaluation.Conditions is not None:
        return scenario.Evaluation.Conditions
    return {}
