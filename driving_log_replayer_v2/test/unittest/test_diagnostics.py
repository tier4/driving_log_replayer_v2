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

from typing import Literal

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pydantic import ValidationError
import pytest

from driving_log_replayer_v2.diagnostics import Conditions
from driving_log_replayer_v2.diagnostics import DiagCondition
from driving_log_replayer_v2.diagnostics import DiagnosticsResult
from driving_log_replayer_v2.diagnostics import DiagnosticsScenario
from driving_log_replayer_v2.diagnostics import get_diagnostic_level_string
from driving_log_replayer_v2.diagnostics import stamp_to_float
from driving_log_replayer_v2.diagnostics import StartEnd
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: DiagnosticsScenario = load_sample_scenario("diagnostics", DiagnosticsScenario)
    assert scenario.ScenarioName == "sample_diagnostics"
    assert scenario.Evaluation.Conditions.DiagConditions[0].condition_type == "all_of"


def test_stamp_to_float() -> None:
    stamp = Time(sec=1234567890, nanosec=123456789)
    assert stamp_to_float(stamp) == 1234567890.123456789  # noqa


def test_get_diagnostic_level_string() -> None:
    assert get_diagnostic_level_string(DiagnosticStatus(level=DiagnosticStatus.OK)) == "OK"
    assert get_diagnostic_level_string(DiagnosticStatus(level=DiagnosticStatus.WARN)) == "WARN"


def test_start_end_validation() -> None:
    with pytest.raises(ValidationError):
        StartEnd(start=10.0, end=1.0)


def test_start_end_match_condition() -> None:
    start_end = StartEnd(start=10.0, end=20.0)
    assert start_end.match_condition(12.0)
    assert start_end.match_condition(2.0) is False
    assert start_end.match_condition(22.0) is False


def test_target_hardware_ids() -> None:
    diag_cond1 = DiagCondition(
        hardware_id="control_validator",
        name="control_validator: control_validation_rolling_back",
        level=["OK"],
        time=StartEnd(start=0.0, end=1.0),
        condition_type="all_of",
    )
    diag_cond2 = DiagCondition(
        hardware_id="topic_state_monitor",
        name="topic_state_monitor",
        level=["OK"],
        time=StartEnd(start=0.0, end=1.0),
        condition_type="all_of",
    )
    cond = Conditions(DiagConditions=[diag_cond1, diag_cond2])
    assert cond.target_hardware_ids == ["control_validator", "topic_state_monitor"]
