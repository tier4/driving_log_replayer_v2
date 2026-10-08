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

"""
t4perceval systems and pipelines of the perception use case.

The pipelines map the scenario settings onto the t4perceval filters, matchers and metrics:
perception_eval's label matching policy becomes `class_agnostic` of the matchers and
`max_matchable_radii` becomes their `max_matchable_distance`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from t4perceval.system import ApplyMaskSystem
from t4perceval.system import AveragePrecisionHeadingSystem
from t4perceval.system import AveragePrecisionSystem
from t4perceval.system import CenterDistanceBEVMatchingSystem
from t4perceval.system import CenterDistanceMatchingSystem
from t4perceval.system import ClearSystem
from t4perceval.system import CombineMasksSystem
from t4perceval.system import ConfusionMatrixSystem
from t4perceval.system import FilterByConfidenceSystem
from t4perceval.system import FilterByDistanceSystem
from t4perceval.system import FilterByInstanceSystem
from t4perceval.system import FilterByLabelSystem
from t4perceval.system import FilterByNumPointsSystem
from t4perceval.system import FilterByRegionSystem
from t4perceval.system import IoU3DMatchingSystem
from t4perceval.system import IoUBEVMatchingSystem
from t4perceval.system import MeanAveragePrecisionSystem
from t4perceval.system import PathDisplacementSystem
from t4perceval.system import Pipeline
from t4perceval.system import PlaneDistanceMatchingSystem
from t4perceval.system import TransformEntitySystem
from t4perceval.system.matching.threshold import Thresholds

from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import GROUND_TRUTH_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import is_class_agnostic

if TYPE_CHECKING:
    from collections.abc import Callable
    from collections.abc import Iterable

    from t4perceval.system import System
    from t4perceval.system.matching.base import MatchingSystem

    from driving_log_replayer_v2.perception.t4perceval_adapter.config import EvaluationConfig
    from driving_log_replayer_v2.perception.t4perceval_adapter.config import PerLabel

BASE_LINK = "base_link"
ESTIMATION_PATH = "/estimation/objects"
PASS_FAIL_MATCHING_PATH = "/matching/pass_fail"  # noqa: S105
CONFUSION_MATCHING_PATH = "/matching/confusion"
CONFUSION_MATRIX_PATH = "/metrics/confusion_matrix"
DISPLACEMENT_PATH = "/metrics/displacement"
FRAME_METRICS_ROOT = "/frame_metrics"

MATCHING_MODE_NAMES: dict[str, str] = {
    "center_distance": "Center Distance",
    "center_distance_bev": "Center Distance BEV",
    "plane_distance": "Plane Distance",
    "iou_2d": "IoU 2D",
    "iou_3d": "IoU 3D",
}
"""Matching family -> the name perception_eval printed in the metrics keys."""


def in_base_link(path: str) -> str:
    return f"{path}/in/{BASE_LINK}"


def kept(path: str) -> str:
    return f"{path}/kept"


ESTIMATION_KEPT_PATH = kept(ESTIMATION_PATH)
GROUND_TRUTH_KEPT_PATH = kept(GROUND_TRUTH_PATH)
ESTIMATION_KEPT_BASE_LINK_PATH = in_base_link(ESTIMATION_KEPT_PATH)
GROUND_TRUTH_KEPT_BASE_LINK_PATH = in_base_link(GROUND_TRUTH_KEPT_PATH)


def to_thresholds(values: PerLabel) -> Thresholds:
    """Per-label mapping -> `Thresholds`. The default covers labels outside the mapping."""
    return Thresholds(max(values.values()), by_class=dict(values))


def max_matchable_distance(config: EvaluationConfig) -> Thresholds | None:
    """`max_matchable_radii` as the `max_matchable_distance` of the t4perceval matchers."""
    radii = config.max_matchable_radii
    return to_thresholds(radii) if radii else None


MATCHERS: dict[str, type[MatchingSystem]] = {
    "center_distance": CenterDistanceMatchingSystem,
    "center_distance_bev": CenterDistanceBEVMatchingSystem,
    "plane_distance": PlaneDistanceMatchingSystem,
    "iou_2d": IoUBEVMatchingSystem,
    "iou_3d": IoU3DMatchingSystem,
}


def matching_sources(family: str) -> tuple[str, str]:
    """Return the (estimation, ground truth) entities a matching family reads."""
    if family == "plane_distance":
        return ESTIMATION_KEPT_BASE_LINK_PATH, GROUND_TRUTH_KEPT_BASE_LINK_PATH
    return ESTIMATION_KEPT_PATH, GROUND_TRUTH_KEPT_PATH


def valid_threshold_sets(family: str, sets: tuple[PerLabel, ...]) -> list[PerLabel]:
    """Drop the threshold sets t4perceval rejects (non-positive, or IoU outside (0, 1])."""
    is_iou = family.startswith("iou")
    valid = []
    for values in sets:
        numbers = list(values.values())
        if all(v > 0.0 for v in numbers) and (not is_iou or all(v <= 1.0 for v in numbers)):
            valid.append(values)
    return valid


# -- masks ----------------------------------------------------------------------------------


def _loosest(values: Iterable[float | None], pick: Callable[..., float]) -> float | None:
    """
    OR the bounds given by several configs (global, critical, pass/fail) into one.

    `pick` is `max` for an upper bound and `min` for a lower bound; unset bounds are skipped.
    """
    given = [value for value in values if value is not None]
    return pick(given) if given else None


def _side_masks(
    config: EvaluationConfig,
    *,
    is_estimation: bool,
    has_num_points: bool,
    has_instance_id: bool,
) -> tuple[list[System], str]:
    """
    Masks of one side (estimation or ground truth), AND-ed into `<native>/filter/critical`.

    Every filter applies one threshold to every label; when the global and the critical
    settings both bound the same quantity, a row passes if it satisfies either of them, so
    the looser bound is used.
    """
    native = ESTIMATION_PATH if is_estimation else GROUND_TRUTH_PATH
    base_link = in_base_link(native)
    critical = config.critical
    systems: list[System] = []

    # labels: the global target_labels and the critical target_labels both apply
    labels = [label for label in config.target_labels if label in critical.target_labels]
    systems.append(FilterByLabelSystem.on(native, name="target_labels", labels=labels))

    # region / distance, in base_link
    max_x = _loosest((config.max_x_position, critical.max_x_position), max)
    max_y = _loosest((config.max_y_position, critical.max_y_position), max)
    if max_x is not None or max_y is not None:
        max_xy = (
            max_x if max_x is not None else float("inf"),
            max_y if max_y is not None else float("inf"),
        )
        systems.append(FilterByRegionSystem.symmetric(base_link, name="region", max_xy=max_xy))
    max_distance = _loosest((config.max_distance, critical.max_distance), max)
    min_distance = _loosest((config.min_distance, critical.min_distance), min)
    if max_distance is not None or min_distance is not None:
        systems.append(
            FilterByDistanceSystem.on(
                base_link,
                name="distance",
                min_distance=min_distance or 0.0,
                max_distance=max_distance if max_distance is not None else float("inf"),
                bev=True,
            )
        )

    if is_estimation:
        min_confidence = _loosest(
            (
                config.confidence_threshold,
                critical.confidence_threshold,
                config.pass_fail.confidence_threshold,
            ),
            min,
        )
        if min_confidence is not None:
            systems.append(
                FilterByConfidenceSystem.on(
                    native, name="confidence", min_confidence=min_confidence
                )
            )
    else:
        target_uuids = critical.target_uuids or config.target_uuids
        if target_uuids and has_instance_id:
            systems.append(
                FilterByInstanceSystem.on(
                    native,
                    name="target_uuids",
                    instances=[f"gt/{uuid}" for uuid in target_uuids],
                )
            )
        min_num_points = _loosest((config.min_num_points, critical.min_num_points), min)
        if has_num_points and min_num_points:
            systems.append(
                FilterByNumPointsSystem.on(native, name="num_points", min_num_points=min_num_points)
            )

    target = f"{native}/filter/critical"
    systems.append(
        CombineMasksSystem.of([str(system.target) for system in systems], target, mode="all")
    )
    return systems, target


# -- pipelines -------------------------------------------------------------------------------


def _sweep(
    family: str,
    sets: list[PerLabel],
    config: EvaluationConfig,
    *,
    root: str = "",
) -> list[System]:
    """AP/APH per threshold set and their means, with targets prefixed by `root`/family."""
    matcher = MATCHERS[family]
    estimation, ground_truth = matching_sources(family)
    systems: list[System] = []
    ap_targets: list[str] = []
    aph_targets: list[str] = []
    for index, values in enumerate(sets):
        matching = f"{root}/matching/{family}/{index}"
        systems.append(
            matcher.between(
                estimation,
                ground_truth,
                target=matching,
                threshold=to_thresholds(values),
                class_agnostic=is_class_agnostic(config.matching_label_policy),
                max_matchable_distance=max_matchable_distance(config),
            )
        )
        ap_target = f"{root}/metrics/ap/{family}/{index}"
        aph_target = f"{root}/metrics/aph/{family}/{index}"
        ap_targets.append(ap_target)
        aph_targets.append(aph_target)
        systems.append(
            AveragePrecisionSystem.on(matching, estimation, ground_truth, target=ap_target)
        )
        systems.append(
            AveragePrecisionHeadingSystem.on(matching, estimation, ground_truth, target=aph_target)
        )
    systems.append(MeanAveragePrecisionSystem.of(ap_targets, target=f"{root}/metrics/map/{family}"))
    systems.append(
        MeanAveragePrecisionSystem.of(aph_targets, target=f"{root}/metrics/maph/{family}")
    )
    return systems


def build_frame_pipeline(
    config: EvaluationConfig,
    *,
    compute_frame_metrics: bool = False,
    gt_has_num_points: bool = True,
    gt_has_instance_id: bool = True,
) -> Pipeline:
    """
    Pipeline run once per evaluated frame on a store holding only that frame.

    It expresses both sides in base_link, applies the object filters, materializes the kept
    rows and runs the plane distance pass/fail matching.
    """
    systems: list[System] = [
        TransformEntitySystem.of(ESTIMATION_PATH, target_frame=BASE_LINK),
        TransformEntitySystem.of(GROUND_TRUTH_PATH, target_frame=BASE_LINK),
    ]
    est_masks, est_mask = _side_masks(
        config, is_estimation=True, has_num_points=False, has_instance_id=False
    )
    gt_masks, gt_mask = _side_masks(
        config,
        is_estimation=False,
        has_num_points=gt_has_num_points,
        has_instance_id=gt_has_instance_id,
    )
    systems.extend(est_masks)
    systems.extend(gt_masks)
    systems.extend(
        [
            ApplyMaskSystem.of(ESTIMATION_PATH, est_mask, target=ESTIMATION_KEPT_PATH),
            ApplyMaskSystem.of(
                in_base_link(ESTIMATION_PATH), est_mask, target=ESTIMATION_KEPT_BASE_LINK_PATH
            ),
            ApplyMaskSystem.of(GROUND_TRUTH_PATH, gt_mask, target=GROUND_TRUTH_KEPT_PATH),
            ApplyMaskSystem.of(
                in_base_link(GROUND_TRUTH_PATH), gt_mask, target=GROUND_TRUTH_KEPT_BASE_LINK_PATH
            ),
        ]
    )
    pass_fail_threshold = (
        to_thresholds(config.pass_fail.matching_threshold)
        if config.pass_fail.matching_threshold
        else PlaneDistanceMatchingSystem.DEFAULT_THRESHOLD
    )
    systems.append(
        PlaneDistanceMatchingSystem.between(
            ESTIMATION_KEPT_BASE_LINK_PATH,
            GROUND_TRUTH_KEPT_BASE_LINK_PATH,
            target=PASS_FAIL_MATCHING_PATH,
            threshold=pass_fail_threshold,
            class_agnostic=is_class_agnostic(config.matching_label_policy),
            max_matchable_distance=max_matchable_distance(config),
        )
    )
    if compute_frame_metrics:
        sets = valid_threshold_sets("center_distance", config.center_distance_thresholds)
        if sets:
            systems.extend(_sweep("center_distance", sets, config, root=FRAME_METRICS_ROOT))
    return Pipeline(systems)


def build_scene_pipeline(config: EvaluationConfig) -> Pipeline:
    """Pipeline run once over the whole scene store (kept entities and pass/fail matching)."""
    systems: list[System] = []
    is_tracking = config.evaluation_task in ("tracking", "prediction")
    for family, sets in config.threshold_families.items():
        valid = valid_threshold_sets(family, sets)
        if not valid:
            continue
        systems.extend(_sweep(family, valid, config))
        if is_tracking:
            estimation, ground_truth = matching_sources(family)
            systems.append(
                ClearSystem.on(
                    f"/matching/{family}/0",
                    estimation,
                    ground_truth,
                    target=f"/metrics/clear/{family}",
                )
            )
    if config.evaluation_task == "prediction" and valid_threshold_sets(
        "center_distance", config.center_distance_thresholds
    ):
        systems.append(
            PathDisplacementSystem.on(
                "/matching/center_distance/0",
                ESTIMATION_KEPT_PATH,
                GROUND_TRUTH_KEPT_PATH,
                target=DISPLACEMENT_PATH,
                top_k=max(config.top_ks) if config.top_ks else 1,
                miss_tolerance=config.miss_tolerance,
            )
        )
    pass_fail_threshold = (
        to_thresholds(config.pass_fail.matching_threshold)
        if config.pass_fail.matching_threshold
        else PlaneDistanceMatchingSystem.DEFAULT_THRESHOLD
    )
    systems.append(
        PlaneDistanceMatchingSystem.between(
            ESTIMATION_KEPT_BASE_LINK_PATH,
            GROUND_TRUTH_KEPT_BASE_LINK_PATH,
            target=CONFUSION_MATCHING_PATH,
            threshold=pass_fail_threshold,
            class_agnostic=True,
            max_matchable_distance=max_matchable_distance(config),
        )
    )
    systems.append(
        ConfusionMatrixSystem.on(
            CONFUSION_MATCHING_PATH,
            ESTIMATION_KEPT_BASE_LINK_PATH,
            GROUND_TRUTH_KEPT_BASE_LINK_PATH,
            target=CONFUSION_MATRIX_PATH,
        )
    )
    return Pipeline(systems)
