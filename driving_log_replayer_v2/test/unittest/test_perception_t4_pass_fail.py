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

"""The frame pipeline: filters, base_link transform and plane distance pass/fail."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np
import pytest
from t4perceval import FRAME
from t4perceval import InstanceRegistry
from t4perceval import LabelRegistry
from t4perceval import Store
from t4perceval import TimePoint
from t4perceval import TimeRange
from t4perceval import Trackings3D
from t4perceval import Transform3D
from t4perceval.descriptors import MASK
from t4perceval.system.base import SystemContext

from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig
from driving_log_replayer_v2.perception.t4perceval_adapter.config import from_scenario
from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import MatchTable
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import EGO_TRANSFORM_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import GROUND_TRUTH_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import build_label_registry
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import build_frame_pipeline
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import ESTIMATION_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import (
    GROUND_TRUTH_KEPT_BASE_LINK_PATH,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import PASS_FAIL_MATCHING_PATH

if TYPE_CHECKING:
    from collections.abc import Sequence

LABELS = ["car", "bicycle", "pedestrian", "motorbike", "unknown"]
FRAME_INDEX = 1


def make_config(
    *,
    evaluation_task: str = "detection",
    policy: str = "default",
    max_matchable_radii: Sequence[float] | None = None,
    matching_threshold: Sequence[float] = (2.0, 2.0, 2.0, 2.0, 2.0),
    max_x_position: float | None = 200.0,
    confidence_threshold: float | None = None,
    target_uuids: Sequence[str] | None = None,
    min_point_numbers: Sequence[int] | None = None,
) -> EvaluationConfig:
    eval_dict = {
        "evaluation_task": evaluation_task,
        "target_labels": LABELS,
        "max_x_position": max_x_position,
        "max_y_position": max_x_position,
        "matching_label_policy": policy,
        "max_matchable_radii": list(max_matchable_radii) if max_matchable_radii else None,
        "confidence_threshold": confidence_threshold,
        "target_uuids": list(target_uuids) if target_uuids else None,
        "min_point_numbers": list(min_point_numbers) if min_point_numbers else None,
        "center_distance_thresholds": [1.0, 2.0],
        "plane_distance_thresholds": [2.0],
    }
    return from_scenario(
        {"evaluation_config_dict": eval_dict},
        {"target_labels": LABELS},
        {"target_labels": LABELS, "matching_threshold_list": list(matching_threshold)},
        evaluation_task=evaluation_task,
        frame_id="base_link" if evaluation_task == "detection" else "map",
        logger=logging.getLogger("test_pass_fail"),
    )


def tracks(
    registry: LabelRegistry,
    instances: InstanceRegistry,
    positions: Sequence[Sequence[float]],
    labels: Sequence[str],
    uuids: Sequence[str],
    *,
    confidence: float = 0.9,
    num_points: Sequence[int] | None = None,
) -> Trackings3D:
    count = len(positions)
    return Trackings3D(
        position=positions,
        quaternion=[[0.0, 0.0, 0.0, 1.0]] * count,
        size=[[2.0, 4.0, 1.5]] * count,
        class_id=registry.encode(labels),
        confidence=[confidence] * count,
        instance_id=instances.encode(uuids),
        velocity=np.full((count, 3), np.nan),
        num_points=num_points,
    )


def run_frame(
    config: EvaluationConfig,
    registry: LabelRegistry,
    instances: InstanceRegistry,
    estimation: Trackings3D,
    ground_truth: Trackings3D,
    *,
    ego_translation: Sequence[float] | None = None,
    gt_has_num_points: bool = False,
) -> tuple[Store, MatchTable]:
    store = Store()
    frame_id = config.frame_id
    at = TimePoint.at(frame=FRAME_INDEX, timestamp_ns=100)
    store.log(ESTIMATION_PATH, estimation, at=at, frame_id=frame_id)
    store.log(GROUND_TRUTH_PATH, ground_truth, at=at, frame_id=frame_id)
    if ego_translation is not None:
        store.log(
            EGO_TRANSFORM_PATH,
            Transform3D(
                translation=ego_translation,
                rotation=[0.0, 0.0, 0.0, 1.0],
                child_frame_id="base_link",
            ),
            at=at,
            frame_id="map",
        )
    pipeline = build_frame_pipeline(config, gt_has_num_points=gt_has_num_points)
    pipeline.run(SystemContext(store, FRAME, labels=registry, instances=instances), at=FRAME_INDEX)
    chunk = store.range(
        PASS_FAIL_MATCHING_PATH, timeline=FRAME, time_range=TimeRange.single(FRAME_INDEX)
    ).to_chunk()
    return store, (MatchTable.from_chunk(chunk) if chunk.num_rows else MatchTable.empty())


@pytest.fixture
def registry() -> LabelRegistry:
    return build_label_registry()


def test_plane_distance_pass_fail(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    config = make_config()
    estimation = tracks(
        registry,
        instances,
        [[10.0, 0.0, 0.0], [20.0, 0.0, 0.0], [40.0, 0.0, 0.0]],
        ["car", "car", "car"],
        ["est/a", "est/b", "est/c"],
    )
    ground_truth = tracks(
        registry,
        instances,
        [[10.5, 0.0, 0.0], [23.0, 0.0, 0.0], [60.0, 0.0, 0.0]],
        ["car", "car", "car"],
        ["gt/a", "gt/b", "gt/c"],
    )
    _, matches = run_frame(config, registry, instances, estimation, ground_truth)
    # 0.5 m apart -> TP; 3 m apart -> both FP and FN at the 2 m threshold
    assert matches.tp_est.tolist() == [0]
    assert matches.tp_gt.tolist() == [0]
    assert matches.fp_est.tolist() == [1, 2]
    assert matches.fn_gt.tolist() == [1, 2]
    assert matches.tp_score[0] == pytest.approx(0.5)


def test_per_label_matching_threshold(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    # pedestrian threshold 0.3 m, car 2.0 m
    config = make_config(matching_threshold=(2.0, 2.0, 0.3, 2.0, 2.0))
    estimation = tracks(
        registry,
        instances,
        [[10.0, 0.0, 0.0], [20.0, 0.0, 0.0]],
        ["car", "pedestrian"],
        ["e0", "e1"],
    )
    ground_truth = tracks(
        registry,
        instances,
        [[10.5, 0.0, 0.0], [20.5, 0.0, 0.0]],
        ["car", "pedestrian"],
        ["g0", "g1"],
    )
    _, matches = run_frame(config, registry, instances, estimation, ground_truth)
    assert matches.tp_gt.tolist() == [0]
    assert matches.fn_gt.tolist() == [1]


def test_label_policy_default_rejects_other_labels(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    estimation = tracks(registry, instances, [[10.0, 0.0, 0.0]], ["unknown"], ["e0"])
    ground_truth = tracks(registry, instances, [[10.0, 0.0, 0.0]], ["car"], ["g0"])
    _, strict = run_frame(
        make_config(policy="default"), registry, instances, estimation, ground_truth
    )
    assert strict.tp_est.tolist() == []
    assert strict.fp_est.tolist() == [0]
    assert strict.fn_gt.tolist() == [0]
    _, loose = run_frame(
        make_config(policy="allow_any"), registry, instances, estimation, ground_truth
    )
    assert loose.tp_est.tolist() == [0]


def test_max_matchable_radii_gates_the_match(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    estimation = tracks(registry, instances, [[10.0, 0.0, 0.0]], ["car"], ["e0"])
    # same heading and size: plane distance 1.5 m but center distance 1.5 m too
    ground_truth = tracks(registry, instances, [[11.5, 0.0, 0.0]], ["car"], ["g0"])
    _, without = run_frame(make_config(), registry, instances, estimation, ground_truth)
    assert without.tp_est.tolist() == [0]
    _, gated = run_frame(
        make_config(max_matchable_radii=(1.0, 1.0, 1.0, 1.0, 1.0)),
        registry,
        instances,
        estimation,
        ground_truth,
    )
    assert gated.tp_est.tolist() == []


def test_filters_apply_in_base_link_for_map_data(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    config = make_config(evaluation_task="tracking", max_x_position=50.0)
    # ego at map x = 100: the object at map x = 120 is 20 m ahead, the one at 200 is 100 m ahead
    estimation = tracks(
        registry, instances, [[120.0, 0.0, 0.0], [200.0, 0.0, 0.0]], ["car", "car"], ["e0", "e1"]
    )
    ground_truth = tracks(
        registry, instances, [[120.5, 0.0, 0.0], [200.5, 0.0, 0.0]], ["car", "car"], ["g0", "g1"]
    )
    store, matches = run_frame(
        config, registry, instances, estimation, ground_truth, ego_translation=[100.0, 0.0, 0.0]
    )
    assert matches.tp_est.tolist() == [0]
    assert matches.fp_est.tolist() == []
    assert matches.fn_gt.tolist() == []
    kept = store.range(
        GROUND_TRUTH_KEPT_BASE_LINK_PATH, timeline=FRAME, time_range=TimeRange.single(FRAME_INDEX)
    ).to_chunk()
    assert kept.frame_id == "base_link"
    assert kept.num_rows == 1
    position = next(c for d, c in kept.columns.items() if d.component == "position").values
    assert np.allclose(position[0], [20.5, 0.0, 0.0])


def test_confidence_threshold_filters_estimations_only(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    config = make_config(confidence_threshold=0.95)
    estimation = tracks(registry, instances, [[10.0, 0.0, 0.0]], ["car"], ["e0"], confidence=0.5)
    ground_truth = tracks(registry, instances, [[10.0, 0.0, 0.0]], ["car"], ["g0"], confidence=0.5)
    _, matches = run_frame(config, registry, instances, estimation, ground_truth)
    assert matches.fp_est.tolist() == []
    assert matches.fn_gt.tolist() == [0]


def test_target_uuids_keep_only_those_ground_truths(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    config = make_config(target_uuids=("keep",))
    estimation = tracks(registry, instances, [[10.0, 0.0, 0.0]], ["car"], ["est/e0"])
    ground_truth = tracks(
        registry,
        instances,
        [[10.0, 0.0, 0.0], [30.0, 0.0, 0.0]],
        ["car", "car"],
        ["gt/keep", "gt/drop"],
    )
    _, matches = run_frame(config, registry, instances, estimation, ground_truth)
    assert matches.tp_gt.tolist() == [0]
    assert matches.fn_gt.tolist() == []


def test_min_point_numbers_per_label(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    config = make_config(min_point_numbers=(5, 0, 0, 0, 0))
    estimation = tracks(registry, instances, [], [], [])
    ground_truth = tracks(
        registry,
        instances,
        [[10.0, 0.0, 0.0], [20.0, 0.0, 0.0], [30.0, 0.0, 0.0]],
        ["car", "car", "pedestrian"],
        ["g0", "g1", "g2"],
        num_points=[3, 10, 1],
    )
    store, matches = run_frame(
        config, registry, instances, estimation, ground_truth, gt_has_num_points=True
    )
    # the car with 3 points is dropped, the pedestrian has no minimum: 2 kept ground truths
    kept = store.range(
        GROUND_TRUTH_KEPT_BASE_LINK_PATH, timeline=FRAME, time_range=TimeRange.single(FRAME_INDEX)
    ).to_chunk()
    assert kept.num_rows == 2  # noqa: PLR2004
    assert matches.fn_gt.tolist() == [0, 1]


def test_empty_frame(registry: LabelRegistry) -> None:
    instances = InstanceRegistry()
    estimation = tracks(registry, instances, [], [], [])
    ground_truth = tracks(registry, instances, [], [], [])
    store, matches = run_frame(make_config(), registry, instances, estimation, ground_truth)
    assert len(matches) == 0
    mask = store.range(
        f"{ESTIMATION_PATH}/filter/critical",
        timeline=FRAME,
        time_range=TimeRange.single(FRAME_INDEX),
    ).to_chunk()
    assert mask.columns[MASK].values.tolist() == []
