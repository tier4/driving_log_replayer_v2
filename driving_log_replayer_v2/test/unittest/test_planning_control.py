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

from autoware_internal_planning_msgs.msg import ControlPoint
from autoware_internal_planning_msgs.msg import PlanningFactor as PlanningFactorMsg
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from pydantic import ValidationError
import pytest
from std_msgs.msg import Header
from tier4_metric_msgs.msg import Metric as MetricMsg
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.planning_control import Metric
from driving_log_replayer_v2.planning_control import MetricCondition
from driving_log_replayer_v2.planning_control import MinMax
from driving_log_replayer_v2.planning_control import PlanningControlScenario
from driving_log_replayer_v2.planning_control import PlanningFactor
from driving_log_replayer_v2.planning_control import PlanningFactorCondition
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


def create_planning_factor_condition(
    *,
    behavior: list[str] | None = None,
    duration: MinMax | None = None,
    judgement: Literal["positive", "negative"] = "positive",
) -> PlanningFactorCondition:
    return PlanningFactorCondition(
        condition_name="pf_check",
        topic="/planning/planning_factors/obstacle_stop",
        time={"start": 0.0, "end": 100.0},
        condition_type="any_of",
        behavior=behavior,
        duration=duration,
        judgement=judgement,
    )


def create_planning_factor_array_msg(
    *,
    stamp_sec: int = 1,
    stamp_nanosec: int = 0,
    behavior: int = PlanningFactorMsg.STOP,
    factor_count: int = 1,
) -> PlanningFactorArray:
    control_point = ControlPoint()
    control_point.distance = 10.0
    control_point.velocity = 20.0
    control_point.pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(),
    )

    factors = []
    for _ in range(factor_count):
        factor = PlanningFactorMsg()
        factor.behavior = behavior
        factor.control_points = [control_point]
        factors.append(factor)

    msg = PlanningFactorArray()
    msg.header = Header(stamp=Time(sec=stamp_sec, nanosec=stamp_nanosec))
    msg.factors = factors
    return msg


def test_planning_factor_duration_first_frame() -> None:
    condition = create_planning_factor_condition(duration=MinMax(min=0.1, max=10.0))
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    result = evaluation_item.set_frame(create_planning_factor_array_msg(), None)

    assert result is not None
    assert result["Result"]["Frame"] == "Success"
    assert result["Info"]["Factor_0"]["Duration"] == pytest.approx(0.1)


def test_planning_factor_duration_accumulates() -> None:
    condition = create_planning_factor_condition(duration=MinMax(min=0.0, max=10.0))
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    evaluation_item.set_frame(create_planning_factor_array_msg(stamp_sec=1), None)
    result = evaluation_item.set_frame(
        create_planning_factor_array_msg(stamp_sec=1, stamp_nanosec=300_000_000), None
    )

    assert result is not None
    assert result["Info"]["Factor_0"]["Duration"] == pytest.approx(0.3)


def test_planning_factor_duration_empty_factors_resets_session() -> None:
    condition = create_planning_factor_condition(duration=MinMax(min=0.1, max=10.0))
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    evaluation_item.set_frame(
        create_planning_factor_array_msg(stamp_sec=1, stamp_nanosec=500_000_000), None
    )
    empty_msg = create_planning_factor_array_msg(stamp_sec=2)
    empty_msg.factors = []
    evaluation_item.set_frame(empty_msg, None)
    result = evaluation_item.set_frame(create_planning_factor_array_msg(stamp_sec=3), None)

    assert result is not None
    assert result["Info"]["Factor_0"]["Duration"] == pytest.approx(0.1)


def test_planning_factor_duration_gap_resets_session() -> None:
    condition = create_planning_factor_condition(duration=MinMax(min=0.1, max=10.0))
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    evaluation_item.set_frame(create_planning_factor_array_msg(stamp_sec=1), None)
    result = evaluation_item.set_frame(create_planning_factor_array_msg(stamp_sec=2), None)

    assert result is not None
    assert result["Info"]["Factor_0"]["Duration"] == pytest.approx(0.1)


def test_planning_factor_duration_advances_without_behavior_match() -> None:
    condition = create_planning_factor_condition(
        behavior=["STOP"],
        duration=MinMax(min=0.0, max=10.0),
    )
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    evaluation_item.set_frame(
        create_planning_factor_array_msg(stamp_sec=1, behavior=PlanningFactorMsg.SLOW_DOWN),
        None,
    )
    result = evaluation_item.set_frame(
        create_planning_factor_array_msg(
            stamp_sec=1,
            stamp_nanosec=400_000_000,
            behavior=PlanningFactorMsg.SLOW_DOWN,
        ),
        None,
    )

    assert result is not None
    assert result["Result"]["Frame"] == "Fail"
    assert result["Info"]["Factor_0"]["Duration"] == pytest.approx(0.4)


def test_planning_factor_duration_out_of_range() -> None:
    condition = create_planning_factor_condition(duration=MinMax(min=1.0, max=10.0))
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    result = evaluation_item.set_frame(create_planning_factor_array_msg(), None)

    assert result is not None
    assert result["Result"]["Frame"] == "Fail"
    assert result["Info"]["Factor_0"]["Duration"] == pytest.approx(0.1)


def test_planning_factor_duration_negative_judgement() -> None:
    condition = create_planning_factor_condition(
        duration=MinMax(min=1.0, max=10.0),
        judgement="negative",
    )
    evaluation_item = PlanningFactor(name="pf_0", condition=condition)

    result = evaluation_item.set_frame(create_planning_factor_array_msg(), None)

    assert result is not None
    assert result["Result"]["Frame"] == "Success"
