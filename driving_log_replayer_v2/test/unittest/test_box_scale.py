# Copyright 2026 TIER IV, Inc.
# SPDX-License-Identifier: Apache-2.0

from copy import deepcopy
from importlib import import_module
from types import SimpleNamespace

import numpy as np
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.object import DynamicObject
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from perception_eval.manager.sensing_evaluation_manager import SensingEvaluationManager
from pydantic import ValidationError
from pyquaternion import Quaternion
import pytest
from std_msgs.msg import Header

from driving_log_replayer_v2.box_scale import BoxScaleCondition
from driving_log_replayer_v2.box_scale import resolve_box_scales
from driving_log_replayer_v2.obstacle_segmentation import get_sensing_frame_config
from driving_log_replayer_v2.obstacle_segmentation import ObstacleSegmentationResult
from driving_log_replayer_v2.obstacle_segmentation import ObstacleSegmentationScenario
from driving_log_replayer_v2.scenario import load_sample_scenario

POINT_THRESHOLD = 3
OVERRIDE_SCALE = 2


@pytest.fixture
def scenario_dict() -> dict:
    result = load_sample_scenario(
        "obstacle_segmentation", ObstacleSegmentationScenario
    ).model_dump()
    result["Evaluation"]["UseCaseFormatVersion"] = "0.4.0"
    return result


@pytest.mark.parametrize("override", [None, {}, {"box_scale_0m": None}, {"box_scale_100m": None}])
def test_omitted_null_and_empty_endpoints_use_common_values(
    scenario_dict: dict, override: dict | None
) -> None:
    evaluation = scenario_dict["Evaluation"]
    evaluation["Conditions"]["Detection"]["BoxScale"] = override
    scenario = ObstacleSegmentationScenario.model_validate(scenario_dict)
    metrics = {"box_scale_0m": 1.3, "box_scale_100m": 1.7}
    before = deepcopy(metrics)
    d, n = resolve_box_scales(scenario.Evaluation.Conditions, metrics)
    assert (d.box_scale_0m, d.box_scale_100m) == (1.3, 1.7)
    assert d == n
    assert metrics == before


@pytest.mark.parametrize("disabled", [None, "Detection", "NonDetection", "both"])
def test_partial_overrides_are_independent_and_disabled_conditions_use_common(
    scenario_dict: dict,
    disabled: str | None,
) -> None:
    conditions = scenario_dict["Evaluation"]["Conditions"]
    conditions["Detection"]["BoxScale"] = {"box_scale_0m": 2}
    conditions["NonDetection"]["BoxScale"] = {"box_scale_100m": 3}
    for name in ("Detection", "NonDetection"):
        if disabled in (name, "both"):
            conditions[name] = None
    scenario = ObstacleSegmentationScenario.model_validate(scenario_dict)
    d, n = resolve_box_scales(scenario.Evaluation.Conditions, {})
    assert d.box_scale_0m == (1 if disabled in ("Detection", "both") else 2)
    assert d.box_scale_100m == 1
    assert n.box_scale_0m == 1
    assert n.box_scale_100m == (1 if disabled in ("NonDetection", "both") else 3)


@pytest.mark.parametrize("value", [True, False, "1.2", "", float("nan"), float("inf"), 0, -1])
@pytest.mark.parametrize("key", ["box_scale_0m", "box_scale_100m"])
def test_invalid_individual_scale_is_rejected(key: str, value: object) -> None:
    with pytest.raises(ValidationError):
        BoxScaleCondition.model_validate({key: value})


def test_unknown_scale_key_is_rejected() -> None:
    with pytest.raises(ValidationError):
        BoxScaleCondition.model_validate({"box_scale_10m": 2})


@pytest.mark.parametrize("override", [{}, {"box_scale_0m": 2}])
def test_individual_scale_requires_usecase_040(scenario_dict: dict, override: dict) -> None:
    scenario_dict["Evaluation"]["UseCaseFormatVersion"] = "0.3.0"
    scenario_dict["Evaluation"]["Conditions"]["Detection"]["BoxScale"] = override
    with pytest.raises(ValidationError, match=r"UseCaseFormatVersion: 0\.4\.0"):
        ObstacleSegmentationScenario.model_validate(scenario_dict)


@pytest.mark.parametrize("version", ["0.3.0", "0.4.0"])
def test_legacy_configuration_without_individual_values_is_supported(
    scenario_dict: dict, version: str
) -> None:
    scenario_dict["Evaluation"]["UseCaseFormatVersion"] = version
    scenario = ObstacleSegmentationScenario.model_validate(scenario_dict)
    d, n = resolve_box_scales(
        scenario.Evaluation.Conditions, {"box_scale_0m": 0, "box_scale_100m": 0}
    )
    assert d == n
    serialized = scenario.Evaluation.Conditions.model_dump()
    assert all(
        condition is None or "BoxScale" not in condition for condition in serialized.values()
    )
    assert d.box_scale_0m == 0  # Do not tighten the existing common-value validation.


def test_missing_common_keys_use_library_defaults_and_null_common_is_error(
    scenario_dict: dict,
) -> None:
    conditions = ObstacleSegmentationScenario.model_validate(scenario_dict).Evaluation.Conditions
    d, n = resolve_box_scales(conditions, {})
    assert d.box_scale_0m == d.box_scale_100m == n.box_scale_0m == n.box_scale_100m == 1
    with pytest.raises(ValueError, match="cannot be null"):
        resolve_box_scales(conditions, {"box_scale_0m": None})


@pytest.mark.parametrize("disabled", [False, True])
def test_null_bbox_config_still_builds_config_and_preserves_common_targets(
    scenario_dict: dict,
    *,
    disabled: bool,
) -> None:
    evaluation = scenario_dict["Evaluation"]
    evaluation["Conditions"]["Detection"]["BoundingBoxConfig"] = None
    evaluation["Conditions"]["Detection"]["BoxScale"] = {"box_scale_0m": 2}
    if disabled:
        evaluation["Conditions"]["Detection"] = None
    scenario = ObstacleSegmentationScenario.model_validate(scenario_dict)
    d, _ = resolve_box_scales(scenario.Evaluation.Conditions, {})
    ok, config = get_sensing_frame_config(
        Header(),
        scenario,
        detection_scale=d,
        common_filtering={"target_uuids": ["shared-target"]},
        common_metrics={"min_points_threshold": 3},
    )
    assert ok
    assert config.target_uuids == ["shared-target"]
    assert config.box_scale_0m == (1 if disabled else 2)
    assert config.min_points_threshold == POINT_THRESHOLD


@pytest.mark.parametrize(("stamp", "expected"), [(9, False), (10, True), (20, True), (21, False)])
def test_uuid_time_window_still_skips_entire_frame(
    scenario_dict: dict, stamp: int, *, expected: bool
) -> None:
    scenario_dict["Evaluation"]["Conditions"]["Detection"]["BoundingBoxConfig"] = [
        {"target": {"Start": 10, "End": 20}}
    ]
    scenario_dict["Evaluation"]["Conditions"]["Detection"]["BoxScale"] = {"box_scale_0m": 2}
    scenario = ObstacleSegmentationScenario.model_validate(scenario_dict)
    header = Header()
    header.stamp.sec = stamp
    ok, config = get_sensing_frame_config(header, scenario)
    assert ok is expected
    if expected:
        assert config.target_uuids == ["target"]
        assert config.box_scale_0m == OVERRIDE_SCALE
    else:
        assert config is None


def test_old_dependency_is_rejected_explicitly(
    scenario_dict: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    conditions = ObstacleSegmentationScenario.model_validate(scenario_dict).Evaluation.Conditions
    module = import_module("driving_log_replayer_v2.box_scale")
    real_import = module.import_module

    def old_api(name: str) -> object:
        if name.endswith("sensing_evaluation_manager"):
            return SimpleNamespace(
                SensingEvaluationManager=SimpleNamespace(add_frame_result=lambda: None)
            )
        return real_import(name)

    monkeypatch.setattr(module, "import_module", old_api)
    with pytest.raises(RuntimeError, match="matching perception_eval revision"):
        resolve_box_scales(conditions, {})


@pytest.mark.parametrize(
    ("d", "n", "expected_detection", "expected_non_detection"),
    [(2, 1, "Success", "Fail"), (1, 2, "Fail", "Success")],
)
@pytest.mark.parametrize("topic_rate", [True, None])
def test_scenario_to_manager_to_dlr_result_keeps_scales_independent(
    scenario_dict: dict,
    d: int,
    n: int,
    expected_detection: str,
    expected_non_detection: str,
    *,
    topic_rate: bool | None,
) -> None:
    conditions = scenario_dict["Evaluation"]["Conditions"]
    conditions["Detection"]["BoundingBoxConfig"] = None
    conditions["Detection"]["BoxScale"] = {"box_scale_0m": d, "box_scale_100m": d}
    conditions["NonDetection"]["BoxScale"] = {"box_scale_0m": n, "box_scale_100m": n}
    scenario = ObstacleSegmentationScenario.model_validate(scenario_dict)
    common = {"box_scale_0m": 3.0, "box_scale_100m": 3.0, "min_points_threshold": 1}
    d_scale, n_scale = resolve_box_scales(scenario.Evaluation.Conditions, common)
    _, frame_config = get_sensing_frame_config(
        Header(), scenario, detection_scale=d_scale, common_metrics=common
    )
    obj = DynamicObject(
        unix_time=1,
        frame_id=FrameID.BASE_LINK,
        position=(50.0, 0.0, 0.0),
        orientation=Quaternion(),
        shape=Shape(ShapeType.BOUNDING_BOX, (2.0, 2.0, 2.0)),
        velocity=(0.0, 0.0, 0.0),
        semantic_score=1.0,
        semantic_label=Label(AutowareLabel.CAR, "car"),
        pointcloud_num=1,
        uuid="target",
    )
    manager = SensingEvaluationManager.__new__(SensingEvaluationManager)
    manager.evaluator_config = SimpleNamespace(
        metrics_params=common, filtering_params={"target_uuids": None}
    )
    manager.frame_results = []
    area = [
        [100.0, 5.0, 2.0],
        [0.0, 5.0, 2.0],
        [0.0, -5.0, 2.0],
        [100.0, -5.0, 2.0],
        [100.0, 5.0, -2.0],
        [0.0, 5.0, -2.0],
        [0.0, -5.0, -2.0],
        [100.0, -5.0, -2.0],
    ]
    frame = manager.add_frame_result(
        1,
        FrameGroundTruth(1, "1", [obj]),
        np.array([[51.5, 0.0, 0.0]]),
        [area],
        frame_config,
        n_scale,
    )
    result = ObstacleSegmentationResult(scenario.Evaluation.Conditions)
    result.set_frame(frame, 0, {}, [], Header(), topic_rate=topic_rate)
    assert result.frame["Detection"]["Result"]["Frame"] == expected_detection
    assert result.frame["NonDetection"]["Result"]["Frame"] == expected_non_detection
    assert result.frame["TopicRate"] == ("True" if topic_rate is True else None)
