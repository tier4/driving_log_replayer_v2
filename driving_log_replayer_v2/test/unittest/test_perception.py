# Copyright (c) 2023 TIER IV.inc
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

import pytest
from t4perceval_test_utils import make_record

from driving_log_replayer_v2.perception.models import Conditions
from driving_log_replayer_v2.perception.models import Criteria
from driving_log_replayer_v2.perception.models import Filter
from driving_log_replayer_v2.perception.models import Perception
from driving_log_replayer_v2.perception.models import PerceptionResult
from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaMethod == "num_gt_tp"
    assert scenario.Evaluation.Conditions.Criterion[1].CriteriaLevel == "easy"
    assert (
        scenario.include_use_case.Conditions.PlanningFactorConditions[0].topic
        == "/planning/planning_factors/obstacle_stop"
    )
    assert scenario.include_use_case.Conditions.PlanningFactorConditions[0].behavior == ["STOP"]


def test_scenario_criteria_custom_level() -> None:
    scenario: PerceptionScenario = load_sample_scenario(
        "perception",
        PerceptionScenario,
        "scenario.criteria.custom.yaml",
    )
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaMethod == [
        "metrics_score",
        "metrics_score_maph",
    ]
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaLevel == [10.0, 10.0]
    assert scenario.Evaluation.Conditions.Criterion[0].Filter.Distance is None


def test_filter_distance_omit_upper_limit() -> None:
    filter_condition = Filter(Distance="1.0-")
    assert filter_condition.Distance[0] == 1.0
    assert filter_condition.Distance[1] == sys.float_info.max


def test_filter_distance_is_not_number() -> None:
    with pytest.raises(ValueError):  # noqa
        Filter(Distance="a-b")


def test_filter_distance_element_is_not_two() -> None:
    with pytest.raises(ValueError):  # noqa
        Filter(Distance="1.0-2.0-3.0")


def test_filter_distance_min_max_reversed() -> None:
    with pytest.raises(ValueError):  # noqa
        Filter(Distance="2.0-1.0")


@pytest.fixture
def create_tp_normal() -> Perception:
    return Perception(
        name="criteria0",
        condition=Criteria(
            PassRate=95.0,
            CriteriaMethod="num_tp",
            CriteriaLevel="normal",
            Filter=Filter(Distance=None),
        ),
        total=99,
        passed=94,
    )


@pytest.fixture
def create_tp_hard() -> Perception:
    return Perception(
        name="criteria0",
        condition=Criteria(
            PassRate=95.0,
            CriteriaMethod="num_tp",
            CriteriaLevel="hard",
            Filter=Filter(Distance=None),
        ),
        total=99,
        passed=94,
    )


def test_perception_fail_has_no_object(create_tp_normal: Perception) -> None:
    evaluation_item = create_tp_normal
    # no tp, fp or fn objects
    frame_dict = evaluation_item.set_frame(make_record())
    # check total is not changed (skip count)
    assert evaluation_item.total == 99  # noqa
    assert evaluation_item.success is True  # default is True
    assert frame_dict == {"NoGTNoObj": 1}


def test_perception_success_tp_normal(create_tp_normal: Perception) -> None:
    evaluation_item = create_tp_normal
    # score 50.0 >= NORMAL(50.0)
    frame_dict = evaluation_item.set_frame(make_record(tp=5, fp=5))
    assert evaluation_item.success is True
    assert evaluation_item.summary == "criteria0 (Success): 95 / 100 -> 95.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "TP": "5 [car, car, car, car, car]",
            "FP": "5 [car, car, car, car, car]",
            "FN": "0 []",
            "TN": "null",
        },
    }
    assert frame_dict["Scores"] == {"num_tp": 50.0}
    assert len(frame_dict["Objects"]) == 15  # noqa: PLR2004


def test_perception_fail_tp_normal(create_tp_normal: Perception) -> None:
    evaluation_item = create_tp_normal
    # score 33.3 < NORMAL(50.0)
    frame_dict = evaluation_item.set_frame(make_record(tp=5, fp=10))
    assert evaluation_item.success is False
    assert evaluation_item.summary == "criteria0 (Fail): 94 / 100 -> 94.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "TP": "5 [car, car, car, car, car]",
            "FP": "10 [car, car, car, car, car, car, car, car, car, car]",
            "FN": "0 []",
            "TN": "null",
        },
    }
    # only check PassFail part because Scores will be 3.3333...


def test_perception_fail_tp_hard(create_tp_hard: Perception) -> None:
    evaluation_item = create_tp_hard
    # score 50.0 < HARD(75.0)
    frame_dict = evaluation_item.set_frame(make_record(tp=5, fp=5))
    assert evaluation_item.success is False
    assert evaluation_item.summary == "criteria0 (Fail): 94 / 100 -> 94.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Fail", "Frame": "Fail"},
        "Info": {
            "TP": "5 [car, car, car, car, car]",
            "FP": "5 [car, car, car, car, car]",
            "FN": "0 []",
            "TN": "null",
        },
    }
    assert frame_dict["Scores"] == {"num_tp": 50.0}


@pytest.fixture
def create_perception_result() -> PerceptionResult:
    condition = Conditions(
        Criterion=[
            Criteria(
                PassRate=95.0,
                CriteriaMethod="num_tp",
                CriteriaLevel="normal",
                Filter=Filter(Distance=None),
            )
        ]
    )
    return PerceptionResult(condition)


def test_perception_result_set_frame(create_perception_result: PerceptionResult) -> None:
    """Test that a frame line carries the ego pose, the frame name and the criteria."""
    result = create_perception_result
    result.set_frame(make_record(tp=1, frame_name="12"), 2, map_to_baselink={"dummy": 1})
    assert result.frame["Ego"] == {"TransformStamped": {"dummy": 1}}
    assert result.frame["FrameName"] == "12"
    assert result.frame["FrameSkip"] == 2  # noqa: PLR2004
    assert result.frame["criteria_0"]["PassFail"]["Result"] == {
        "Total": "Success",
        "Frame": "Success",
    }
    assert result.success is True


def test_perception_result_info_frame(create_perception_result: PerceptionResult) -> None:
    """Test that the skip line carries the reason why the frame was not evaluated."""
    result = create_perception_result

    result.set_info_frame({"Reason": "NO_GROUND_TRUTH"}, 3)
    assert result.frame == {"Info": {"Reason": "NO_GROUND_TRUTH"}, "FrameSkip": 3}

    result.set_info_frame({"Reason": "IGNORED_FRAME"}, 4)
    assert result.frame == {"Info": {"Reason": "IGNORED_FRAME"}, "FrameSkip": 4}

    result.set_warn_frame({"Reason": "INVALID_ESTIMATED_OBJECTS"}, 5)
    assert result.frame == {"Warning": {"Reason": "INVALID_ESTIMATED_OBJECTS"}, "FrameSkip": 5}


def test_perception_result_final_metrics(create_perception_result: PerceptionResult) -> None:
    """Test that the coverage is reported next to the metrics, without changing FinalScore."""
    result = create_perception_result

    final_metrics = {"Score": {}, "Error": {}, "ConfusionMatrix": {}}
    result.set_final_metrics(final_metrics)
    assert result.frame == {"FinalScore": final_metrics}

    frame_coverage = {
        "GtFrames": 320,
        "GtFramesEvaluated": 300,
        "Coverage": 0.9375,
        "SkipReasons": {"IGNORED_FRAME": 3, "NO_GROUND_TRUTH": 17},
    }
    result.set_final_metrics(final_metrics, frame_coverage)
    assert result.frame == {"FinalScore": final_metrics, **frame_coverage}
