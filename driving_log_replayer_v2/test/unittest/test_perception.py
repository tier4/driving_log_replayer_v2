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

from perception_eval.common import DynamicObject
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.metrics import MetricsScoreConfig
from perception_eval.evaluation.result.object_result import DynamicObjectWithPerceptionResult
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
from pyquaternion import Quaternion
import pytest
from tier4_api_msgs.msg import AwapiAutowareStatus
from tier4_planning_msgs.msg import StopFactor
from tier4_planning_msgs.msg import StopReason as msgStopReason

from driving_log_replayer_v2.perception.models import Criteria
from driving_log_replayer_v2.perception.models import Filter
from driving_log_replayer_v2.perception.models import Perception
from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.perception.models import StopReason
from driving_log_replayer_v2.perception.models import StopReasonCriteria
from driving_log_replayer_v2.perception.stop_reason import convert_to_stop_reason
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    assert scenario.Evaluation.Conditions.Criterion[0].CriteriaMethod == "num_gt_tp"
    assert scenario.Evaluation.Conditions.Criterion[1].CriteriaLevel == "easy"
    assert scenario.Evaluation.Conditions.stop_reason_criterion[0].tolerance_interval == 1.0
    assert scenario.Evaluation.Conditions.stop_reason_criterion[1].evaluation_type == "non_stop"


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


def test_stop_reason_time_range() -> None:
    stop_reason_criteria = StopReasonCriteria(
        time_range="0-",
        pass_rate=80.0,
        tolerance_interval=1.0,
        evaluation_type="stop",
        condition=[
            {"reason": "ObstacleStop", "base_stop_line_dist": "0.0,10.0"},
        ],
    )
    assert stop_reason_criteria.time_range[0] == 0
    assert stop_reason_criteria.time_range[1] == (1 << 63) - 1


@pytest.fixture
def create_frame_result() -> PerceptionFrameResult:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation_config_dict = scenario.Evaluation.PerceptionEvaluationConfig[
        "evaluation_config_dict"
    ]
    critical_object_filter_config = scenario.Evaluation.CriticalObjectFilterConfig
    perception_pass_fail_config = scenario.Evaluation.PerceptionPassFailConfig
    evaluation_config_dict["label_prefix"] = "autoware"
    m_params: dict = {
        "target_labels": evaluation_config_dict["target_labels"],
        "center_distance_thresholds": evaluation_config_dict.get("center_distance_thresholds"),
        "plane_distance_thresholds": evaluation_config_dict.get("plane_distance_thresholds"),
        "iou_2d_thresholds": evaluation_config_dict.get("iou_2d_thresholds"),
        "iou_3d_thresholds": evaluation_config_dict.get("iou_3d_thresholds"),
    }
    evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
        dataset_paths=["/tmp/dlr"],  # noqa
        frame_id="base_link",
        result_root_directory="/tmp/dlr/result/{TIME}",  # noqa
        evaluation_config_dict=evaluation_config_dict,
        load_raw_data=False,
    )

    return PerceptionFrameResult(
        object_results=None,
        nuscene_object_results=[],
        frame_ground_truth=FrameGroundTruth(123, "12", []),
        metrics_config=MetricsScoreConfig(
            EvaluationTask.DETECTION,
            **m_params,
        ),
        critical_object_filter_config=CriticalObjectFilterConfig(
            evaluation_config,
            critical_object_filter_config["target_labels"],
            max_x_position_list=critical_object_filter_config["max_x_position_list"],
            max_y_position_list=critical_object_filter_config["max_y_position_list"],
        ),
        frame_pass_fail_config=PerceptionPassFailConfig(
            evaluation_config,
            perception_pass_fail_config["target_labels"],
            matching_threshold_list=perception_pass_fail_config["matching_threshold_list"],
        ),
        unix_time=123,
        target_labels=[AutowareLabel.CAR],
    )


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


@pytest.fixture
def create_dynamic_object() -> DynamicObjectWithPerceptionResult:
    dynamic_obj = DynamicObject(
        123,
        FrameID.BASE_LINK,
        (1.0, 2.0, 3.0),
        Quaternion(),
        Shape(ShapeType.BOUNDING_BOX, (1.0, 1.0, 1.0)),
        (1.0, 2.0, 3.0),
        0.5,
        Label(AutowareLabel.CAR, "car"),
    )
    return DynamicObjectWithPerceptionResult(dynamic_obj, None, True)  # noqa


@pytest.fixture
def create_awapi_autoware_status_msg() -> AwapiAutowareStatus:
    stop_factor = StopFactor()
    stop_factor.stop_pose.position.x = 1.0
    stop_factor.stop_pose.position.y = 2.0
    stop_factor.stop_pose.position.z = 3.0
    stop_factor.stop_pose.orientation.x = 0.0
    stop_factor.stop_pose.orientation.y = 0.0
    stop_factor.stop_pose.orientation.z = 0.0
    stop_factor.stop_pose.orientation.w = 1.0
    stop_factor.dist_to_stop_pose = 4.0

    stop_reason = msgStopReason()
    stop_reason.reason = "ObstacleStop"
    stop_reason.stop_factors.append(stop_factor)

    msg = AwapiAutowareStatus()
    msg.stop_reason.header.stamp.sec = 1000
    msg.stop_reason.header.stamp.nanosec = 0
    msg.stop_reason.stop_reasons.append(stop_reason)
    return msg


@pytest.fixture
def create_stop_reason() -> StopReason:
    return StopReason(
        name="criteria0",
        condition=StopReasonCriteria(
            time_range="900-1100",
            pass_rate=95.0,
            tolerance_interval=1.0,
            evaluation_type="stop",
            condition=[{"reason": "ObstacleStop", "base_stop_line_dist": "0.0-10.0"}],
        ),
        total=99,
        passed=94,
    )


def test_perception_fail_has_no_object(
    create_tp_normal: Perception,
    create_frame_result: PerceptionFrameResult,
) -> None:
    evaluation_item = create_tp_normal
    result = create_frame_result
    # add no tp_object_results, fp_object_results
    frame_dict = evaluation_item.set_frame(result)
    # check total is not changed (skip count)
    assert evaluation_item.total == 99  # noqa
    assert evaluation_item.success is True  # default is True
    assert frame_dict == {"NoGTNoObj": 1}


def test_perception_success_tp_normal(
    create_tp_normal: Perception,
    create_frame_result: PerceptionFrameResult,
    create_dynamic_object: DynamicObjectWithPerceptionResult,
) -> None:
    evaluation_item = create_tp_normal
    result = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    result.pass_fail_result.tp_object_results = tp_objects_results
    result.pass_fail_result.fp_object_results = fp_objects_results
    # score 50.0 >= NORMAL(50.0)
    frame_dict = evaluation_item.set_frame(result)
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


def test_perception_fail_tp_normal(
    create_tp_normal: Perception,
    create_frame_result: PerceptionFrameResult,
    create_dynamic_object: DynamicObjectWithPerceptionResult,
) -> None:
    evaluation_item = create_tp_normal
    result = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(10)
    ]
    result.pass_fail_result.tp_object_results = tp_objects_results
    result.pass_fail_result.fp_object_results = fp_objects_results
    # score 33.3 < NORMAL(50.0)
    frame_dict = evaluation_item.set_frame(result)
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


def test_perception_fail_tp_hard(
    create_tp_hard: Perception,
    create_frame_result: PerceptionFrameResult,
    create_dynamic_object: DynamicObjectWithPerceptionResult,
) -> None:
    evaluation_item = create_tp_hard
    result = create_frame_result
    tp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    fp_objects_results: list[DynamicObjectWithPerceptionResult] = [
        create_dynamic_object for i in range(5)
    ]
    result.pass_fail_result.tp_object_results = tp_objects_results
    result.pass_fail_result.fp_object_results = fp_objects_results
    # score 50.0 < HARD(75.0)
    frame_dict = evaluation_item.set_frame(result)
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


def test_stop_reason_obstacle_stop(
    create_awapi_autoware_status_msg: AwapiAutowareStatus,
    create_stop_reason: StopReason,
) -> None:
    stop_reason = convert_to_stop_reason(create_awapi_autoware_status_msg)
    evaluation_item = create_stop_reason
    frame_dict = evaluation_item.set_frame(stop_reason)
    assert evaluation_item.success is True
    assert evaluation_item.summary == "criteria0 (Success): 95 / 100 -> 95.00%"
    assert frame_dict["PassFail"] == {
        "Result": {"Total": "Success", "Frame": "Success"},
        "Info": {
            "Reason": ["ObstacleStop"],
            "Distance": [4.0],
            "Timestamp": 1000,
        },
    }


def test_stop_reason_timeout(
    create_awapi_autoware_status_msg: AwapiAutowareStatus,
    create_stop_reason: StopReason,
) -> None:
    # change time to out of range
    create_awapi_autoware_status_msg.stop_reason.header.stamp.sec = 50
    stop_reason = convert_to_stop_reason(create_awapi_autoware_status_msg)
    evaluation_item = create_stop_reason
    frame_dict = evaluation_item.set_frame(stop_reason)
    assert evaluation_item.success is True  # default is True
    assert frame_dict == {"Timeout": 1}
