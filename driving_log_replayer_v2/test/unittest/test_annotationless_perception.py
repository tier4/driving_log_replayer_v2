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

from collections.abc import Callable
from typing import Literal

import pytest

from driving_log_replayer_v2.annotationless_perception import AnnotationlessPerceptionScenario
from driving_log_replayer_v2.annotationless_perception import DiagValue
from driving_log_replayer_v2.annotationless_perception import LowerUpper
from driving_log_replayer_v2.annotationless_perception import ObjectMetrics
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: AnnotationlessPerceptionScenario = load_sample_scenario(
        "annotationless_perception",
        AnnotationlessPerceptionScenario,
    )
    assert scenario.ScenarioName == "sample_annotationless_perception"
    assert scenario.Evaluation.Conditions.ClassConditions["BUS"] == {
        "yaw_rate": DiagValue(mean=LowerUpper(lower=0, upper=0.01)),
    }


@pytest.fixture
def create_obj_metrics() -> ObjectMetrics:
    condition = {
        "lateral_deviation": DiagValue(
            min=LowerUpper(lower=0.2, upper=1.05),
            max=LowerUpper(lower=0, upper=10.5),
            mean=LowerUpper(lower=4.75, upper=5.25),
        ),
    }
    return ObjectMetrics(
        name="CAR",
        condition=condition,
        received_data={"lateral_deviation": {"min": 1.0, "max": 0.0, "mean": 45.0, "total_sum": 9}},
    )


def test_deviation_success(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is True


def test_deviation_fail_lower_limit_min(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    status_dict = {"lateral_deviation": {"min": 0.1, "max": 10.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_upper_limit_max(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 12.0, "mean": 5.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_upper_limit_mean(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    # sum_mean = 45.0 + 8.0 = 53.0 average = 53.0 / 10 = 5.3 > 5.0*1.05 = 5.25
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 8.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_deviation_fail_lower_limit(create_obj_metrics: Callable) -> None:
    evaluation_item: ObjectMetrics = create_obj_metrics
    # sum_mean = 45.0 + 2.0 = 47.0 average = 47.0 / 10 = 4.7 < 5.0*0.95 = 4.75
    status_dict = {"lateral_deviation": {"min": 1.0, "max": 10.0, "mean": 2.0}}
    evaluation_item.set_frame(status_dict)
    assert evaluation_item.success is False


def test_create_literal_from_tuple() -> None:
    class_tuple = (
        "UNKNOWN",
        "CAR",
        "TRUCK",
        "BUS",
        "TRAILER",
        "MOTORCYCLE",
        "BICYCLE",
        "PEDESTRIAN",
    )
    class_literal = Literal[
        "UNKNOWN",
        "CAR",
        "TRUCK",
        "BUS",
        "TRAILER",
        "MOTORCYCLE",
        "BICYCLE",
        "PEDESTRIAN",
    ]

    literal_using_tuple = Literal[class_tuple]
    assert literal_using_tuple == class_literal
