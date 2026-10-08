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

import numpy as np
import pytest

from driving_log_replayer_v2.perception.t4perceval_adapter.labels import AUTOWARE_LABELS
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import build_label_registry
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import MERGED_LABELS
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import policy_matrix
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import t4_category_aliases


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


def test_t4_categories_map_to_autoware_labels() -> None:
    registry = build_label_registry()
    assert registry.names == AUTOWARE_LABELS
    assert registry.name(registry.class_id("vehicle.car")) == "car"
    assert registry.name(registry.class_id("pedestrian.adult")) == "pedestrian"
    assert registry.name(registry.class_id("vehicle.bus")) == "bus"
    assert registry.name(registry.class_id("vehicle.trailer")) == "truck"
    assert registry.name(registry.class_id("vehicle.motorcycle")) == "motorbike"
    assert registry.name(registry.class_id("movable_object.traffic_cone")) == "hazard"


def test_ros_classification_names_map_to_autoware_labels() -> None:
    registry = build_label_registry()
    assert registry.name(registry.class_id("motorcycle")) == "motorbike"
    assert registry.name(registry.class_id("trailer")) == "truck"
    assert registry.name(registry.class_id("over_drivable")) == "unknown"
    assert registry.name(registry.class_id("hazard")) == "hazard"


def test_merge_similar_labels() -> None:
    registry = build_label_registry(merge_similar_labels=True)
    assert registry.names == MERGED_LABELS
    assert registry.name(registry.class_id("bus")) == "car"
    assert registry.name(registry.class_id("truck")) == "car"
    assert registry.name(registry.class_id("motorbike")) == "bicycle"
    assert registry.name(registry.class_id("traffic_cone")) == "unknown"
    assert t4_category_aliases(merge_similar_labels=True)["vehicle.truck"] == "car"


def test_unknown_dataset_category_becomes_unknown() -> None:
    logger, handler = make_logger("test_labels")
    registry = build_label_registry(["vehicle.car", "weird"], logger=logger)
    assert registry.name(registry.class_id("weird")) == "unknown"
    assert any("weird" in message for message in handler.messages)


@pytest.fixture
def classes() -> dict[str, int]:
    registry = build_label_registry()
    return {name: registry.class_id(name) for name in registry.names}


def test_policy_default_is_same_label(classes: dict[str, int]) -> None:
    registry = build_label_registry()
    est = np.asarray([classes["car"], classes["unknown"]], dtype=np.int32)
    gt = np.asarray([classes["car"], classes["truck"]], dtype=np.int32)
    expected = np.asarray([[True, False], [False, False]])
    assert np.array_equal(policy_matrix("default", est, gt, registry), expected)


def test_policy_allow_unknown(classes: dict[str, int]) -> None:
    registry = build_label_registry()
    est = np.asarray([classes["car"], classes["unknown"], classes["hazard"]], dtype=np.int32)
    gt = np.asarray([classes["car"], classes["truck"]], dtype=np.int32)
    expected = np.asarray([[True, False], [True, True], [True, True]])
    assert np.array_equal(policy_matrix("allow_unknown", est, gt, registry), expected)


def test_policy_allow_same_group(classes: dict[str, int]) -> None:
    registry = build_label_registry()
    est = np.asarray([classes["car"], classes["bicycle"], classes["unknown"]], dtype=np.int32)
    gt = np.asarray([classes["truck"], classes["pedestrian"], classes["animal"]], dtype=np.int32)
    expected = np.asarray([[True, False, False], [False, True, True], [True, True, True]])
    assert np.array_equal(policy_matrix("allow_same_group", est, gt, registry), expected)


def test_policy_allow_any(classes: dict[str, int]) -> None:
    registry = build_label_registry()
    est = np.asarray([classes["car"]], dtype=np.int32)
    gt = np.asarray([classes["truck"], classes["pedestrian"]], dtype=np.int32)
    assert policy_matrix("allow_any", est, gt, registry).all()


def test_policy_rejects_unknown_name() -> None:
    registry = build_label_registry()
    with pytest.raises(ValueError, match="matching_label_policy"):
        policy_matrix("strict", np.zeros(1, np.int32), np.zeros(1, np.int32), registry)
