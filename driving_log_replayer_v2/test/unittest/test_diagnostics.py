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
from pydantic import ValidationError
import pytest
from std_msgs.msg import Header

from driving_log_replayer_v2.diagnostics import Conditions
from driving_log_replayer_v2.diagnostics import Diag
from driving_log_replayer_v2.diagnostics import DiagCondition
from driving_log_replayer_v2.diagnostics import DiagnosticsScenario
from driving_log_replayer_v2.diagnostics import get_diagnostic_level_string
from driving_log_replayer_v2.diagnostics import stamp_to_float
from driving_log_replayer_v2.diagnostics import StartEnd
from driving_log_replayer_v2.scenario import load_sample_scenario

TARGET_HARDWARE_ID = "control_validator"
TARGET_NAME = "control_validator: control_validation_rolling_back"


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


def create_condition(
    condition_type: Literal[
        "any_of",
        "all_of",
        "duration_larger_than",
        "duration_less_than",
        "percentage_larger_than",
        "percentage_less_than",
    ],
    *,
    hardware_id: str = TARGET_HARDWARE_ID,
) -> DiagCondition:
    return DiagCondition(
        hardware_id=hardware_id,
        name=TARGET_NAME,
        level=["OK"],
        time=StartEnd(start=0.0, end=1.0),
        condition_type=condition_type,
    )


def test_target_hardware_ids() -> None:
    diag_cond1 = create_condition("all_of")
    diag_cond2 = create_condition("any_of", hardware_id="topic_state_monitor")
    cond = Conditions(DiagConditions=[diag_cond1, diag_cond2])
    assert cond.target_hardware_ids == [TARGET_HARDWARE_ID, "topic_state_monitor"]


def create_diag_msg(
    *, match_level: bool = True, match_name: bool = True, match_time: bool = True
) -> DiagnosticArray:
    status = DiagnosticStatus(
        level=DiagnosticStatus.OK if match_level else DiagnosticStatus.WARN,
        name=TARGET_NAME if match_name else "module_not_matched",
    )
    header = Header(stamp=Time(sec=0, nanosec=5)) if match_time else Header(stamp=Time(sec=2))
    return DiagnosticArray(header=header, status=[status])


def test_diag_not_match_name() -> None:
    evaluation_item = Diag("Condition_0", create_condition("all_of"))
    diag_msg = create_diag_msg(match_name=False)
    assert evaluation_item.set_frame(diag_msg) is None


def test_diag_not_match_time() -> None:
    evaluation_item = Diag("Condition_0", create_condition("all_of"))
    diag_msg = create_diag_msg(match_time=False)
    assert evaluation_item.set_frame(diag_msg) is None


def test_diag_success_any_of() -> None:
    evaluation_item = Diag("Condition_0", create_condition("any_of"))
    diag_msg = create_diag_msg()
    frame_dict = evaluation_item.set_frame(diag_msg)
    assert evaluation_item.success is True
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "TotalPassed": 1,
            "Level": "OK",
            "ConsecutiveDuration": 0.0,
        },
    }
    diag_msg = create_diag_msg(match_level=False)
    frame_dict = evaluation_item.set_frame(diag_msg)  # any_of is OK if one of them succeeds.
    assert evaluation_item.success is True
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Fail"},
        "Info": {
            "TotalPassed": 1,
            "Level": "WARN",
            "ConsecutiveDuration": 0.0,
        },
    }


def test_diag_fail_any_of() -> None:
    evaluation_item = Diag("Condition_0", create_condition("any_of"))
    diag_msg = create_diag_msg(match_level=False)
    frame_dict = evaluation_item.set_frame(diag_msg)
    assert evaluation_item.success is False
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "TotalPassed": 0,
            "Level": "WARN",
            "ConsecutiveDuration": 0.0,
        },
    }


def test_diag_fail_all_of() -> None:
    evaluation_item = Diag("Condition_0", create_condition("all_of"))
    diag_msg = create_diag_msg()
    frame_dict = evaluation_item.set_frame(diag_msg)
    assert evaluation_item.success is True
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "TotalPassed": 1,
            "Level": "OK",
            "ConsecutiveDuration": 0.0,
        },
    }
    diag_msg = create_diag_msg(match_level=False)
    frame_dict = evaluation_item.set_frame(diag_msg)  # ALL_OF is not allowed if even one fails.
    assert evaluation_item.success is False
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "TotalPassed": 1,
            "Level": "WARN",
            "ConsecutiveDuration": 0.0,
        },
    }
