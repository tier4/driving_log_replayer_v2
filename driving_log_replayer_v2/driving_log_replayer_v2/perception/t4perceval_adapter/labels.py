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
Autoware label vocabulary for t4perceval.

perception_eval mapped every T4 category name onto an Autoware label with its
`LabelConverter`. t4perceval has no such table: a `LabelRegistry` holds canonical class
names and aliases. This module rebuilds the same table as aliases of one registry so that
the ground truth of a T4 dataset and the Autoware messages of a rosbag share one class id
per label.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from t4perceval import LabelRegistry
from t4perceval.label import ClassInfo

if TYPE_CHECKING:
    from collections.abc import Iterable
    import logging

    from t4perceval.typing import NDArrayBool
    from t4perceval.typing import NDArrayI32

AUTOWARE_LABELS: tuple[str, ...] = (
    "unknown",
    "car",
    "truck",
    "bus",
    "motorbike",
    "bicycle",
    "pedestrian",
    "animal",
    "hazard",
)
"""Canonical Autoware labels, in the order perception_eval's `AutowareLabel` declares them."""

MERGED_LABELS: tuple[str, ...] = ("unknown", "car", "bicycle", "pedestrian", "animal")
"""Labels that remain when `merge_similar_labels` is on: bus/truck -> car, motorbike -> bicycle."""

LABEL_GROUPS: dict[str, tuple[str, ...]] = {
    "vehicle": ("car", "truck", "bus"),
    "vru": ("bicycle", "motorbike", "pedestrian", "animal"),
}
"""Label groups of perception_eval's `allow_same_group` matching policy."""

MATCHING_LABEL_POLICIES: tuple[str, ...] = (
    "default",
    "allow_unknown",
    "allow_same_group",
    "allow_any",
)

# Pairs of (label, category name) shared by both merge settings.
# Copied from perception_eval.common.label._get_autoware_pairs.
_COMMON_PAIRS: tuple[tuple[str, str], ...] = (
    ("bicycle", "bicycle"),
    ("bicycle", "vehicle.bicycle"),
    ("car", "car"),
    ("car", "police_car"),
    ("car", "forklift"),
    ("car", "ambulance"),
    ("car", "kart"),
    ("car", "vehicle.car"),
    ("car", "vehicle.emergency (ambulance & police)"),
    ("car", "vehicle.police"),
    ("car", "vehicle.fire"),
    ("car", "vehicle.ambulance"),
    ("car", "other_vehicle"),
    ("pedestrian", "pedestrian"),
    ("pedestrian", "stroller"),
    ("pedestrian", "pedestrian.adult"),
    ("pedestrian", "pedestrian.child"),
    ("pedestrian", "pedestrian.construction_worker"),
    ("pedestrian", "pedestrian.personal_mobility"),
    ("pedestrian", "pedestrian.police_officer"),
    ("pedestrian", "pedestrian.stroller"),
    ("pedestrian", "pedestrian.wheelchair"),
    ("pedestrian", "police_officer"),
    ("pedestrian", "construction_worker"),
    ("pedestrian", "wheelchair"),
    ("pedestrian", "personal_mobility"),
    ("pedestrian", "other_pedestrian"),
    ("animal", "animal"),
    ("unknown", "unknown"),
    ("unknown", "static_object.bicycle rack"),
    ("unknown", "static_object.bicycle_rack"),
    # Autoware ObjectClassification names which perception_eval never saw
    ("unknown", "over_drivable"),
    ("unknown", "under_drivable"),
)

_VEHICLE_NAMES: tuple[str, ...] = (
    "bus",
    "vehicle.bus (bendy & rigid)",
    "vehicle.bus",
    "truck",
    "vehicle.truck",
    "trailer",
    "vehicle.trailer",
    "semi_trailer",
    "vehicle.construction",
    "vehicle.fire",
    "fire_truck",
    "tractor_unit",
    "construction_vehicle",
)
_BUS_NAMES: tuple[str, ...] = ("bus", "vehicle.bus (bendy & rigid)", "vehicle.bus")
_MOTORBIKE_NAMES: tuple[str, ...] = ("motorbike", "motorcycle", "vehicle.motorcycle")
_HAZARD_NAMES: tuple[str, ...] = (
    "movable_object.pushable_pullable",
    "movable_object.barrier",
    "movable_object.debris",
    "movable_object.trafficcone",
    "movable_object.traffic_cone",
    "static_object.bollard",
    "traffic_cone",
    "trafficcone",
    "barrier",
    "hazard",
)


def t4_category_aliases(*, merge_similar_labels: bool) -> dict[str, str]:
    """
    Return the category name to Autoware label table of perception_eval.

    Args:
        merge_similar_labels (bool): If True, bus/truck/trailer become `car`, motorbike becomes
            `bicycle` and the hazard-like categories become `unknown`.

    Returns:
        dict[str, str]: Category or classification name -> canonical label.

    """
    aliases: dict[str, str] = {name: label for label, name in _COMMON_PAIRS}
    if merge_similar_labels:
        aliases.update(dict.fromkeys(_VEHICLE_NAMES, "car"))
        aliases.update(dict.fromkeys(_MOTORBIKE_NAMES, "bicycle"))
        aliases.update(dict.fromkeys(_HAZARD_NAMES, "unknown"))
    else:
        aliases.update(dict.fromkeys(_VEHICLE_NAMES, "truck"))
        aliases.update(dict.fromkeys(_BUS_NAMES, "bus"))
        aliases.update(dict.fromkeys(_MOTORBIKE_NAMES, "motorbike"))
        aliases.update(dict.fromkeys(_HAZARD_NAMES, "hazard"))
    return aliases


def canonical_labels(*, merge_similar_labels: bool) -> tuple[str, ...]:
    """Return the class names of the registry for the merge setting."""
    return MERGED_LABELS if merge_similar_labels else AUTOWARE_LABELS


def build_label_registry(
    dataset_categories: Iterable[str] = (),
    *,
    merge_similar_labels: bool = False,
    logger: logging.Logger | None = None,
) -> LabelRegistry:
    """
    Build the shared label registry of the ground truth and the estimation.

    Args:
        dataset_categories (Iterable[str]): Category names of the T4 dataset. A category the
            table does not know is mapped to `unknown`, as perception_eval did, and a warning is
            logged.
        merge_similar_labels (bool): See `t4_category_aliases`.
        logger (logging.Logger | None): Logger for the warnings.

    Returns:
        LabelRegistry: Classes in `AUTOWARE_LABELS` order plus the aliases.

    """
    classes = canonical_labels(merge_similar_labels=merge_similar_labels)
    class_ids = {name: class_id for class_id, name in enumerate(classes)}
    aliases: dict[str, int] = {}
    for name, label in t4_category_aliases(merge_similar_labels=merge_similar_labels).items():
        if name in class_ids:
            continue
        aliases[name] = class_ids[label]
    for category in dataset_categories:
        if category in class_ids or category in aliases:
            continue
        if logger is not None:
            logger.warning(
                "Category '%s' is not in the Autoware label table, it is evaluated as 'unknown'.",
                category,
            )
        aliases[category] = class_ids["unknown"]
    return LabelRegistry(
        tuple(ClassInfo(class_id, name) for name, class_id in class_ids.items()),
        aliases=aliases,
    )


def validate_matching_label_policy(policy: str) -> str:
    """Return the normalized policy name or raise ValueError."""
    normalized = policy.lower()
    if normalized not in MATCHING_LABEL_POLICIES:
        err_msg = (
            f"Unsupported matching_label_policy: {policy}. "
            f"Expected one of {', '.join(MATCHING_LABEL_POLICIES)}."
        )
        raise ValueError(err_msg)
    return normalized


def _group_ids(labels: LabelRegistry) -> dict[int, int]:
    groups: dict[int, int] = {}
    for group_index, members in enumerate(LABEL_GROUPS.values()):
        for member in members:
            class_id = labels.class_id_or(member, -1)
            if class_id >= 0:
                groups[class_id] = group_index
    return groups


def policy_matrix(
    policy: str,
    est_class: NDArrayI32,
    gt_class: NDArrayI32,
    labels: LabelRegistry,
) -> NDArrayBool:
    """
    Return which (estimation, ground truth) pairs the label policy allows to match.

    Reproduces `MatchingLabelPolicy.is_matchable` of perception_eval:
        default: same label.
        allow_unknown: same label, or the estimation is `unknown` or `hazard`.
        allow_same_group: same label group, or the estimation is `unknown` or `hazard`.
        allow_any: every pair.

    Args:
        policy (str): One of `MATCHING_LABEL_POLICIES`.
        est_class (NDArrayI32): Class ids of the estimations, shape (N,).
        gt_class (NDArrayI32): Class ids of the ground truths, shape (M,).
        labels (LabelRegistry): Registry resolving the special labels.

    Returns:
        NDArrayBool: Matrix of shape (N, M).

    """
    policy = validate_matching_label_policy(policy)
    est_class = np.asarray(est_class, dtype=np.int32)
    gt_class = np.asarray(gt_class, dtype=np.int32)
    if policy == "allow_any":
        return np.ones((len(est_class), len(gt_class)), dtype=np.bool_)

    same = est_class[:, None] == gt_class[None, :]
    if policy == "default":
        return same

    wildcard_ids = [
        class_id
        for class_id in (labels.class_id_or("unknown", -1), labels.class_id_or("hazard", -1))
        if class_id >= 0
    ]
    est_wildcard = np.isin(est_class, wildcard_ids)[:, None]
    if policy == "allow_unknown":
        return same | est_wildcard

    groups = _group_ids(labels)
    est_group = np.asarray([groups.get(int(c), -1) for c in est_class], dtype=np.int64)
    gt_group = np.asarray([groups.get(int(c), -1) for c in gt_class], dtype=np.int64)
    same_group = (est_group[:, None] == gt_group[None, :]) & (est_group[:, None] >= 0)
    return same | same_group | est_wildcard
