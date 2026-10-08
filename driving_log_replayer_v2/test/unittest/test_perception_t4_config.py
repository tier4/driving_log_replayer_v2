# Copyright (c) 2026 TIER IV.inc
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

import logging

import pytest

from driving_log_replayer_v2.perception.models import PerceptionScenario
from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig
from driving_log_replayer_v2.perception.t4perceval_adapter.config import from_scenario
from driving_log_replayer_v2.scenario import load_sample_scenario


class ListHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__()
        self.messages: list[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.messages.append(record.getMessage())


def make_logger(name: str) -> tuple[logging.Logger, ListHandler]:
    logger = logging.getLogger(name)
    logger.handlers.clear()
    handler = ListHandler()
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    return logger, handler


def load_sample_config(**overrides: object) -> EvaluationConfig:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation = scenario.Evaluation
    kwargs: dict = {
        "evaluation_task": "detection",
        "frame_id": "base_link",
        "logger": logging.getLogger("test_config"),
    }
    kwargs.update(overrides)
    return from_scenario(
        evaluation.PerceptionEvaluationConfig,
        evaluation.CriticalObjectFilterConfig,
        evaluation.PerceptionPassFailConfig,
        **kwargs,
    )


def test_sample_scenario_is_parsed() -> None:
    config = load_sample_config()
    labels = ("car", "bicycle", "pedestrian", "motorbike", "unknown")
    assert config.target_labels == labels
    assert config.matching_label_policy == "allow_unknown"
    assert config.max_x_position == 200.0  # noqa: PLR2004
    assert config.max_matchable_radii == dict(zip(labels, (5.0, 3.0, 3.0, 3.0, 3.0), strict=True))
    # nested thresholds: one set per inner list
    assert config.center_distance_thresholds == (
        dict.fromkeys(labels, 1.0),
        dict.fromkeys(labels, 2.0),
    )
    # flat thresholds: one set per value, applied to every label
    assert config.plane_distance_thresholds == (
        dict.fromkeys(labels, 2.0),
        dict.fromkeys(labels, 30.0),
    )
    assert config.iou_2d_thresholds == (dict.fromkeys(labels, 0.5),)
    assert config.min_num_points == dict.fromkeys(labels, 0)
    assert config.ignore_attributes == ("cycle_state.without_rider",)
    assert config.critical.max_x_position == dict.fromkeys(labels, 200.0)
    assert config.pass_fail.matching_threshold == dict.fromkeys(labels, 2.0)
    assert config.trajectory_shape is None
    assert set(config.threshold_families) == {
        "center_distance",
        "plane_distance",
        "iou_2d",
        "iou_3d",
    }


def test_ignored_keys_are_warned() -> None:
    logger, handler = make_logger("test_config_warnings")
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation = scenario.Evaluation
    evaluation.PerceptionEvaluationConfig["evaluation_config_dict"]["label_prefix"] = "autoware"
    evaluation.PerceptionEvaluationConfig["evaluation_config_dict"]["count_label_number"] = True
    evaluation.PerceptionEvaluationConfig["evaluation_config_dict"]["mystery"] = 1
    from_scenario(
        evaluation.PerceptionEvaluationConfig,
        evaluation.CriticalObjectFilterConfig,
        evaluation.PerceptionPassFailConfig,
        evaluation_task="detection",
        frame_id="base_link",
        logger=logger,
    )
    text = "\n".join(handler.messages)
    assert "label_prefix" in text
    assert "count_label_number" in text
    assert "mystery" in text


def test_per_label_list_length_is_checked() -> None:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation = scenario.Evaluation
    evaluation.CriticalObjectFilterConfig["max_x_position_list"] = [1.0, 2.0]
    with pytest.raises(ValueError, match="max_x_position_list"):
        from_scenario(
            evaluation.PerceptionEvaluationConfig,
            evaluation.CriticalObjectFilterConfig,
            evaluation.PerceptionPassFailConfig,
            evaluation_task="detection",
            frame_id="base_link",
        )


def test_unknown_target_label_is_rejected() -> None:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation = scenario.Evaluation
    evaluation.PerceptionEvaluationConfig["evaluation_config_dict"]["target_labels"] = [
        "car",
        "tree",
    ]
    with pytest.raises(ValueError, match="tree"):
        from_scenario(
            evaluation.PerceptionEvaluationConfig,
            {"target_labels": ["car"]},
            {"target_labels": ["car"]},
            evaluation_task="detection",
            frame_id="base_link",
        )


def test_fp_validation_is_rejected() -> None:
    with pytest.raises(ValueError, match="fp_validation"):
        load_sample_config(evaluation_task="fp_validation")


def test_allow_matching_unknown_sets_the_policy() -> None:
    scenario: PerceptionScenario = load_sample_scenario("perception", PerceptionScenario)
    evaluation = scenario.Evaluation
    eval_dict = evaluation.PerceptionEvaluationConfig["evaluation_config_dict"]
    del eval_dict["matching_label_policy"]
    eval_dict["allow_matching_unknown"] = True
    config = from_scenario(
        evaluation.PerceptionEvaluationConfig,
        evaluation.CriticalObjectFilterConfig,
        evaluation.PerceptionPassFailConfig,
        evaluation_task="tracking",
        frame_id="map",
    )
    assert config.matching_label_policy == "allow_unknown"
    assert config.frame_id == "map"


def test_config_round_trips_through_dict() -> None:
    config = load_sample_config()
    assert EvaluationConfig.from_dict(config.to_dict()) == config
