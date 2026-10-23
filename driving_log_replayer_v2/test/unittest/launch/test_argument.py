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

import json

from driving_log_replayer_v2.launch.argument import check_launch_component


def _make_conf(use_case: str, scenario_autoware_args: dict, **cli_args: str) -> dict:
    return {
        "with_autoware": "true",
        "use_case": use_case,
        "scenario_autoware_args": json.dumps(scenario_autoware_args),
        **cli_args,
    }


def test_check_launch_component_without_autoware() -> None:
    conf = _make_conf("diagnostics", {})
    conf["with_autoware"] = "false"
    assert check_launch_component(conf) == {"autoware": "false"}


def test_check_launch_component_default() -> None:
    # diagnostics has no AUTOWARE_DISABLE
    conf = _make_conf("diagnostics", {})
    assert check_launch_component(conf) == {
        "sensing": "true",
        "localization": "true",
        "perception": "true",
        "planning": "true",
        "control": "true",
    }


def test_check_launch_component_scenario_bool_to_str() -> None:
    conf = _make_conf("diagnostics", {"sensing": False, "planning": True})
    result = check_launch_component(conf)
    assert result["sensing"] == "false"
    assert result["planning"] == "true"
    assert conf["sensing"] == "false"
    assert conf["planning"] == "true"
    # not specified
    assert result["control"] == "true"
    assert "control" not in conf


def test_check_launch_component_scenario_over_launch_config() -> None:
    # eagleye: AUTOWARE_DISABLE = perception, planning, control are false
    conf = _make_conf("eagleye", {"perception": True})
    result = check_launch_component(conf)
    assert result["perception"] == "true"  # scenario file wins
    assert result["planning"] == "false"  # launch_config.py
    assert result["control"] == "false"  # launch_config.py


def test_check_launch_component_cli_over_scenario() -> None:
    conf = _make_conf("eagleye", {"perception": True, "sensing": False}, perception="false")
    result = check_launch_component(conf)
    assert result["perception"] == "false"  # CLI wins
    assert result["sensing"] == "false"  # scenario file
