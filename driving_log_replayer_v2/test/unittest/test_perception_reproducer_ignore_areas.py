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


def _outside_area() -> Area:
    return Area(x=0.0, y=0.0, range=5.0, area_condition="outside")


def _metric_group(
    group_name: str,
    *,
    ignore_areas: list[Area] | None = None,
    nested: ConditionGroup | None = None,
) -> ConditionGroup:
    condition_list: list = [
        MetricCondition(
            condition_class="metric",
            condition_name="check_metric",
            topic="/test/metrics",
            metric_name="acceleration",
            condition_type="all_of",
            judgement="positive",
            value_type="number",
            value_range=MinMax(min=0.0, max=0.1),
        )
    ]
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


def test_is_ego_in_area_inside() -> None:
    area = _inside_area()
    assert is_ego_in_area(0.0, 0.0, area)
    assert is_ego_in_area(5.0, 0.0, area)
    assert not is_ego_in_area(5.1, 0.0, area)


def test_is_ego_in_area_outside() -> None:
    area = _outside_area()
    assert not is_ego_in_area(0.0, 0.0, area)
    assert is_ego_in_area(5.1, 0.0, area)


def test_is_ego_in_any_area() -> None:
    areas = [_inside_area(), Area(x=100.0, y=0.0, range=2.0)]
    assert is_ego_in_any_area(0.0, 0.0, areas)
    assert is_ego_in_any_area(100.0, 0.0, areas)
    assert not is_ego_in_any_area(50.0, 0.0, areas)


def test_is_ignored_no_ignore_areas() -> None:
    evaluator = ConditionGroupEvaluator(
        _metric_group("group"), _evaluator_state(x=0.0, y=0.0)
    )
    assert not evaluator.is_ignored()


def test_is_ignored_own_area() -> None:
    evaluator = ConditionGroupEvaluator(
        _metric_group("group", ignore_areas=[_inside_area()]),
        _evaluator_state(x=0.0, y=0.0),
    )
    assert evaluator.is_ignored()
    assert not ConditionGroupEvaluator(
        _metric_group("group", ignore_areas=[_inside_area()]),
        _evaluator_state(x=10.0, y=0.0),
    ).is_ignored()


def test_is_ignored_parent_cascade() -> None:
    child = _metric_group("child")
    parent = ConditionGroupEvaluator(
        _metric_group("parent", ignore_areas=[_inside_area()], nested=child),
        _evaluator_state(x=0.0, y=0.0),
    )
    child_evaluator = parent.nested_evaluators[0]
    assert parent.is_ignored()
    assert child_evaluator.is_ignored()


def test_evaluate_skips_when_ignored() -> None:
    evaluator = ConditionGroupEvaluator(
        _metric_group("group", ignore_areas=[_inside_area()]),
        _evaluator_state(x=0.0, y=0.0),
    )
    evaluator.activate()
    evaluator.set_metric_frame(_fail_metric_msg(), "/test/metrics")
    evaluator.evaluate()
    assert evaluator.success is None
    assert evaluator.triggered is False
    assert evaluator.summary == {"Status": "Ignored"}


def test_evaluate_resumes_after_leaving_ignore_area() -> None:
    state = _evaluator_state(x=10.0, y=0.0)
    evaluator = ConditionGroupEvaluator(
        _metric_group("group", ignore_areas=[_inside_area()]),
        state,
    )
    evaluator.activate()
    evaluator.set_metric_frame(_fail_metric_msg(), "/test/metrics")
    evaluator.evaluate()
    assert evaluator.success is False

    state.latest_ego_x = 0.0
    state.latest_ego_y = 0.0
    evaluator.evaluate()
    assert evaluator.summary == {"Status": "Ignored"}
    assert evaluator.success is False

    state.latest_ego_x = 10.0
    state.latest_ego_y = 0.0
    evaluator.set_metric_frame(_fail_metric_msg(), "/test/metrics")
    evaluator.evaluate()
    assert evaluator.success is False


def test_set_metric_frame_skips_when_ignored() -> None:
    evaluator = ConditionGroupEvaluator(
        _metric_group("group", ignore_areas=[_inside_area()]),
        _evaluator_state(x=0.0, y=0.0),
    )
    evaluator.activate()
    updated = evaluator.set_metric_frame(_fail_metric_msg(), "/test/metrics")
    assert not updated
    assert evaluator.metric_result is not None
    assert evaluator.metric_result.frame == {}
