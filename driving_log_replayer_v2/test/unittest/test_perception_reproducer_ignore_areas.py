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

import sys
from pathlib import Path

import pytest

from driving_log_replayer_v2.perception_reproducer import ConditionGroup
from driving_log_replayer_v2.perception_reproducer import MetricCondition
from driving_log_replayer_v2.perception_reproducer import is_ego_in_any_area
from driving_log_replayer_v2.perception_reproducer import is_ego_in_area
from driving_log_replayer_v2.planning_control import Area
from driving_log_replayer_v2.planning_control import MinMax

pytest.importorskip("builtin_interfaces")
pytest.importorskip("tier4_metric_msgs")

from builtin_interfaces.msg import Time  # noqa: E402
from tier4_metric_msgs.msg import Metric as MetricMsg  # noqa: E402
from tier4_metric_msgs.msg import MetricArray  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from perception_reproducer_evaluator_node import ConditionGroupEvaluator  # noqa: E402
from perception_reproducer_evaluator_node import EvaluatorState  # noqa: E402


def _inside_area() -> Area:
    return Area(x=0.0, y=0.0, range=5.0, area_condition="inside")


def _metric_condition() -> MetricCondition:
    return MetricCondition(
        condition_class="metric",
        condition_name="check_metric",
        topic="/test/metrics",
        metric_name="acceleration",
        condition_type="all_of",
        judgement="positive",
        value_type="number",
        value_range=MinMax(min=0.0, max=0.1),
    )


def _metric_group(
    group_name: str,
    *,
    ignore_areas: list[Area] | None = None,
    nested: ConditionGroup | None = None,
) -> ConditionGroup:
    condition_list: list = [_metric_condition()]
    if nested is not None:
        condition_list.append(nested)
    return ConditionGroup(
        group_name=group_name,
        group_type="all_of",
        ignore_areas=ignore_areas or [],
        condition_list=condition_list,
    )


def _evaluator_state(*, x: float | None = None, y: float | None = None) -> EvaluatorState:
    return EvaluatorState(all_evaluators={}, timeout_s=120.0, latest_ego_x=x, latest_ego_y=y)


def _fail_metric_msg() -> MetricArray:
    return MetricArray(
        stamp=Time(sec=1, nanosec=0),
        metric_array=[MetricMsg(name="acceleration", unit="", value="2.0")],
    )


def test_parent_keeps_nested_fail_when_nested_ignored() -> None:
    """Ignored nested must still contribute frozen success=False to parent all_of."""
    child = _metric_group("child", ignore_areas=[_inside_area()])
    parent = ConditionGroupEvaluator(
        ConditionGroup(
            group_name="parent",
            group_type="all_of",
            condition_list=[child],
        ),
        _evaluator_state(x=10.0, y=0.0),
    )
    parent.activate()

    parent.nested_evaluators[0].set_metric_frame(_fail_metric_msg(), "/test/metrics")
    parent.evaluate()
    assert parent.nested_evaluators[0].success is False
    assert parent.success is False

    state = parent.state
    state.latest_ego_x = 0.0
    state.latest_ego_y = 0.0
    parent.evaluate()

    child_evaluator = parent.nested_evaluators[0]
    assert child_evaluator.is_ignored()
    assert child_evaluator.success is False
    assert child_evaluator.summary == {"Status": "Ignored"}
    assert parent.success is False
    assert parent.summary["Status"] == "Failed"
    assert parent.summary["child"] == {"Status": "Ignored"}


def test_is_ego_in_area_inside() -> None:
    area = _inside_area()
    assert is_ego_in_area(0.0, 0.0, area)
    assert is_ego_in_area(5.0, 0.0, area)
    assert not is_ego_in_area(5.1, 0.0, area)
