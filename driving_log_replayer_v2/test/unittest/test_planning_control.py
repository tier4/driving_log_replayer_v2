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
from pydantic import ValidationError
import pytest
from tier4_metric_msgs.msg import Metric as MetricMsg
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.planning_control import Metric
from driving_log_replayer_v2.planning_control import MetricCondition
from driving_log_replayer_v2.planning_control import MinMax
from driving_log_replayer_v2.planning_control import PlanningControlScenario
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PlanningControlScenario = load_sample_scenario(
        "planning_control",
        PlanningControlScenario,
    )
    assert scenario.ScenarioName == "sample_planning_control"
    assert scenario.Evaluation.Conditions.MetricConditions[0].condition_type == "all_of"


def test_min_max_validation() -> None:
    with pytest.raises(ValidationError):
        MinMax(min=3.0, max=1.0)


def create_condition(
    value_type: Literal["number", "string"],
    condition_type: Literal["any_of", "all_of"],
    judgement: Literal["positive", "negative"] = "positive",
) -> MetricCondition:
    return MetricCondition(
        condition_name="acceleration_check",
        topic="/control/control_evaluator/metrics",
        metric_name="acceleration",
        time={"start": 0.0, "end": 10.0},
        value_type=value_type,
        value_range=MinMax(min=0.0, max=1.0),
        value_target="0.0",
        condition_type=condition_type,
        judgement=judgement,
    )


def create_metric_array_msg(
    *,
    value: str = "0.0",
    is_out_of_range: bool = False,
    is_not_target: bool = False,
) -> MetricArray:
    if is_out_of_range or is_not_target:
        value = "2.0"
    return MetricArray(
        stamp=Time(sec=1, nanosec=0),
        metric_array=[MetricMsg(name="acceleration", unit="", value=value)],
    )


def test_metrics_number() -> None:
    condition = create_condition(value_type="number", condition_type="any_of")
    evaluation_item: Metric = Metric(name="control_0", condition=condition)
    metric_array_msg = create_metric_array_msg()
    assert evaluation_item.set_frame(metric_array_msg) == {
        "Info": {"MetricName": "acceleration", "MetricValue": 0.0},
        "Result": {"Frame": "Success", "Total": "Success"},
    }
    assert evaluation_item.success is True


def test_metrics_number_out_range() -> None:
    condition = create_condition(value_type="number", condition_type="any_of")
    evaluation_item: Metric = Metric(name="control_0", condition=condition)
    metric_array_msg = create_metric_array_msg(is_out_of_range=True)
    assert evaluation_item.set_frame(metric_array_msg) == {
        "Info": {"MetricName": "acceleration", "MetricValue": 2.0},
        "Result": {"Frame": "Fail", "Total": "Fail"},
    }
    assert evaluation_item.success is False


def test_metrics_string() -> None:
    condition = create_condition(value_type="string", condition_type="any_of")
    evaluation_item: Metric = Metric(name="control_0", condition=condition)
    metric_array_msg = create_metric_array_msg()
    assert evaluation_item.set_frame(metric_array_msg) == {
        "Info": {"MetricName": "acceleration", "MetricValue": "0.0"},
        "Result": {"Frame": "Success", "Total": "Success"},
    }
    assert evaluation_item.success is True


def test_metrics_string_not_target() -> None:
    condition = create_condition(value_type="string", condition_type="any_of")
    evaluation_item: Metric = Metric(name="control_0", condition=condition)
    metric_array_msg = create_metric_array_msg(is_not_target=True)
    assert evaluation_item.set_frame(metric_array_msg) == {
        "Info": {"MetricName": "acceleration", "MetricValue": "2.0"},
        "Result": {"Frame": "Fail", "Total": "Fail"},
    }
    assert evaluation_item.success is False


def test_metrics_success_any_of() -> None:
    condition = create_condition(value_type="number", condition_type="any_of")
    evaluation_item: Metric = Metric(name="control_0", condition=condition)
    metric_array_msg = create_metric_array_msg()
    frame_dict = evaluation_item.set_frame(metric_array_msg)
    assert evaluation_item.success is True
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {"MetricName": "acceleration", "MetricValue": 0.0},
    }

    metric_array_msg = create_metric_array_msg(is_out_of_range=True)
    frame_dict = evaluation_item.set_frame(
        metric_array_msg
    )  # any_of is OK if one of them succeeds.
    assert evaluation_item.success is True
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Fail"},
        "Info": {"MetricName": "acceleration", "MetricValue": 2.0},
    }


def test_metrics_fail_all_of() -> None:
    condition = create_condition(value_type="number", condition_type="all_of")
    evaluation_item: Metric = Metric(name="control_0", condition=condition)
    metric_array_msg = create_metric_array_msg()
    frame_dict = evaluation_item.set_frame(metric_array_msg)
    assert evaluation_item.success is True
    assert frame_dict == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {"MetricName": "acceleration", "MetricValue": 0.0},
    }

    metric_array_msg = create_metric_array_msg(is_out_of_range=True)
    frame_dict = evaluation_item.set_frame(
        metric_array_msg
    )  # # all_of is not allowed if even one fails.
    assert evaluation_item.success is False
    assert frame_dict == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {"MetricName": "acceleration", "MetricValue": 2.0},
    }
