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

from perception_eval.common import DynamicObject2D
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.evaluation_task import EvaluationTask
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.schema import FrameID
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.metrics import MetricsScoreConfig
from perception_eval.evaluation.result.object_result import DynamicObjectWithPerceptionResult
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
import pytest

from driving_log_replayer_v2.perception_2d import Conditions
from driving_log_replayer_v2.perception_2d import Perception
from driving_log_replayer_v2.perception_2d import Perception2DScenario
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: Perception2DScenario = load_sample_scenario("perception_2d", Perception2DScenario)
    assert scenario.Evaluation.Conditions.TargetCameras["cam_front"] == 0


@pytest.fixture
def create_frame_result() -> PerceptionFrameResult:
    target_labels = ["car", "truck", "bicycle", "pedestrian", "motorbike"]
    evaluation_config_dict = {
        "evaluation_task": "detection2d",
        "center_distance_thresholds": [100, 200],
        "iou_2d_thresholds": [0.5],
        "target_labels": target_labels,
        "allow_matching_unknown": True,
        "merge_similar_labels": False,
        "label_prefix": "autoware",
        "count_label_number": True,
    }
    m_params: dict = {
        "target_labels": target_labels,
        "center_distance_thresholds": evaluation_config_dict.get("center_distance_thresholds"),
        "plane_distance_thresholds": evaluation_config_dict.get("plane_distance_thresholds"),
        "iou_2d_thresholds": evaluation_config_dict.get("iou_2d_thresholds"),
        "iou_3d_thresholds": evaluation_config_dict.get("iou_3d_thresholds"),
    }
    evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
        dataset_paths=["/tmp/dlr"],  # noqa
        frame_id=["cam_front"],
        result_root_directory="/tmp/dlr/result/{TIME}",  # noqa
        evaluation_config_dict=evaluation_config_dict,
        load_raw_data=False,
    )

    return PerceptionFrameResult(
        object_results=[],
        frame_ground_truth=FrameGroundTruth(123, "12", []),
        metrics_config=MetricsScoreConfig(
            EvaluationTask.DETECTION2D,
            **m_params,
        ),
        critical_object_filter_config=CriticalObjectFilterConfig(
            evaluation_config,
            target_labels,
        ),
        frame_pass_fail_config=PerceptionPassFailConfig(evaluation_config, target_labels),
        unix_time=123,
        target_labels=[AutowareLabel.CAR],
    )


@pytest.fixture
def create_tp_normal() -> Perception:
    return Perception(
        name="cam_front",
        condition=Conditions(
            PassRate=95.0,
            CriteriaMethod="num_tp",
            CriteriaLevel="normal",
            TargetCameras={"cam_front": 0},
        ),
        total=99,
        passed=94,
    )


@pytest.fixture
def create_tp_hard() -> Perception:
    return Perception(
        name="cam_front",
        condition=Conditions(
            PassRate=95.0,
            CriteriaMethod="num_tp",
            CriteriaLevel="hard",
            TargetCameras={"cam_front": 0},
        ),
        total=99,
        passed=94,
    )


@pytest.fixture
def create_dynamic_object() -> DynamicObjectWithPerceptionResult:
    dynamic_obj_2d = DynamicObject2D(
        123,
        FrameID.CAM_FRONT,
        0.5,
        Label(AutowareLabel.CAR, "12"),
    )
    return DynamicObjectWithPerceptionResult(dynamic_obj_2d, None, True)  # noqa


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
    assert evaluation_item.summary == "cam_front (Success): 95 / 100 -> 95.00%"
    assert frame_dict == {
        "PassFail": {
            "Result": {"Total": "Success", "Frame": "Success"},
            "Info": {
                "TP": 5,
                "FP": 5,
                "FN": 0,
            },
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
    assert evaluation_item.summary == "cam_front (Fail): 94 / 100 -> 94.00%"
    assert frame_dict == {
        "PassFail": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {
                "TP": 5,
                "FP": 10,
                "FN": 0,
            },
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
    assert evaluation_item.summary == "cam_front (Fail): 94 / 100 -> 94.00%"
    assert frame_dict == {
        "PassFail": {
            "Result": {"Total": "Fail", "Frame": "Fail"},
            "Info": {
                "TP": 5,
                "FP": 5,
                "FN": 0,
            },
        },
    }
