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

from typing import Any
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


def per_label_masks(
    source: str,
    name: str,
    values: dict[str, Any],
    make_mask: Callable[[str, Any], System],
) -> tuple[list[System], str]:
    """
    Build `(label AND threshold)` masks per label, OR-ed into `<source>/filter/<name>`.

    Args:
        source (str): Entity the masks describe.
        name (str): Name of the combined mask.
        values (dict[str, Any]): Label -> threshold value.
        make_mask (Callable[[str, Any], System]): Builds the threshold mask of one label from
            `(mask name, value)`.

    Returns:
        tuple[list[System], str]: The systems in run order and the combined mask path.

    """
    systems: list[System] = []
    targets: list[str] = []
    for index, (label, value) in enumerate(values.items()):
        label_mask = FilterByLabelSystem.on(source, name=f"{name}_{index}_label", labels=[label])
        value_mask = make_mask(f"{name}_{index}_value", value)
        combined = f"{source}/filter/{name}_{index}"
        systems.extend(
            [
                label_mask,
                value_mask,
                CombineMasksSystem.of(
                    [str(label_mask.target), str(value_mask.target)], combined, mode="all"
                ),
            ]
        )
        targets.append(combined)
    target = f"{source}/filter/{name}"
    systems.append(CombineMasksSystem.of(targets, target, mode="any"))
    return systems, target


def _side_masks(
    config: EvaluationConfig,
    *,
    is_estimation: bool,
    has_num_points: bool,
    has_instance_id: bool,
) -> tuple[list[System], str]:
    """Masks of one side (estimation or ground truth), AND-ed into `<native>/filter/critical`."""
    native = ESTIMATION_PATH if is_estimation else GROUND_TRUTH_PATH
    base_link = in_base_link(native)
    systems: list[System] = []
    masks: list[str] = []

    def add(system: System) -> None:
        systems.append(system)
        masks.append(str(system.target))

    def add_per_label(
        source: str, name: str, values: dict[str, Any] | None, make: Callable[[str, Any], System]
    ) -> None:
        if not values:
            return
        per_label, target = per_label_masks(source, name, values, make)
        systems.extend(per_label)
        masks.append(target)

    # labels: the global target_labels and the critical target_labels both apply
    labels = tuple(dict.fromkeys((*config.target_labels, *config.critical.target_labels)))
    add(FilterByLabelSystem.on(native, name="target_labels", labels=list(labels)))

    # region / distance, in base_link
    if config.max_x_position is not None or config.max_y_position is not None:
        max_xy = (
            config.max_x_position if config.max_x_position is not None else float("inf"),
            config.max_y_position if config.max_y_position is not None else float("inf"),
        )
        add(FilterByRegionSystem.symmetric(base_link, name="region", max_xy=max_xy))
    if config.max_distance is not None or config.min_distance is not None:
        add(
            FilterByDistanceSystem.on(
                base_link,
                name="distance",
                min_distance=config.min_distance or 0.0,
                max_distance=config.max_distance
                if config.max_distance is not None
                else float("inf"),
                bev=True,
            )
        )
    add_per_label(
        base_link,
        "critical_max_x",
        config.critical.max_x_position,
        lambda name, value: FilterByRegionSystem.symmetric(
            base_link, name=name, max_xy=(value, float("inf"))
        ),
    )
    add_per_label(
        base_link,
        "critical_max_y",
        config.critical.max_y_position,
        lambda name, value: FilterByRegionSystem.symmetric(
            base_link, name=name, max_xy=(float("inf"), value)
        ),
    )
    add_per_label(
        base_link,
        "critical_max_distance",
        config.critical.max_distance,
        lambda name, value: FilterByDistanceSystem.on(
            base_link, name=name, max_distance=value, bev=True
        ),
    )
    add_per_label(
        base_link,
        "critical_min_distance",
        config.critical.min_distance,
        lambda name, value: FilterByDistanceSystem.on(
            base_link, name=name, min_distance=value, bev=True
        ),
    )

    if is_estimation:
        if config.confidence_threshold is not None:
            add(
                FilterByConfidenceSystem.on(
                    native, name="confidence", min_confidence=config.confidence_threshold
                )
            )
        add_per_label(
            native,
            "critical_confidence",
            config.critical.confidence_threshold,
            lambda name, value: FilterByConfidenceSystem.on(
                native, name=name, min_confidence=value
            ),
        )
        add_per_label(
            native,
            "pass_fail_confidence",
            config.pass_fail.confidence_threshold,
            lambda name, value: FilterByConfidenceSystem.on(
                native, name=name, min_confidence=value
            ),
        )
    else:
        target_uuids = config.critical.target_uuids or config.target_uuids
        if target_uuids and has_instance_id:
            add(
                FilterByInstanceSystem.on(
                    native,
                    name="target_uuids",
                    instances=[f"gt/{uuid}" for uuid in target_uuids],
                )
            )
        if has_num_points:
            add_per_label(
                native,
                "min_num_points",
                config.min_num_points,
                lambda name, value: FilterByNumPointsSystem.on(
                    native, name=name, min_num_points=value
                ),
            )
            add_per_label(
                native,
                "critical_min_num_points",
                config.critical.min_num_points,
                lambda name, value: FilterByNumPointsSystem.on(
                    native, name=name, min_num_points=value
                ),
            )

    target = f"{native}/filter/critical"
    systems.append(CombineMasksSystem.of(masks, target, mode="all"))
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
