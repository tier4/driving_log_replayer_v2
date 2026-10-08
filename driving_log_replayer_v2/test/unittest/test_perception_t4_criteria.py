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

# NOTE: CriteriaLevel.CUSTOM holds one value per process, so every custom level below is 1.0.

import pytest
from t4perceval_test_utils import make_record

from driving_log_replayer_v2.criteria.perception_t4 import PerceptionCriteria
from driving_log_replayer_v2.criteria.perception_t4 import SuccessFail
from driving_log_replayer_v2.perception.models import Filter


def test_num_tp() -> None:
    criteria = PerceptionCriteria(methods="num_tp", levels="normal")
    result, scores, _ = criteria.get_result(make_record(tp=5, fp=5))
    assert result == SuccessFail.SUCCESS
    assert scores == {"num_tp": 50.0}
    result, scores, _ = criteria.get_result(make_record(tp=5, fp=10, fn=1))
    assert result == SuccessFail.FAIL
    assert scores["num_tp"] == pytest.approx(100.0 * 5 / 16)


def test_num_gt_tp() -> None:
    criteria = PerceptionCriteria(methods="num_gt_tp", levels="hard")
    result, scores, _ = criteria.get_result(make_record(tp=3, fp=10, fn=1))
    assert result == SuccessFail.SUCCESS
    assert scores == {"num_gt_tp": 75.0}


def test_no_objects_is_not_available() -> None:
    criteria = PerceptionCriteria(methods="num_tp", levels="easy")
    assert criteria.get_result(make_record())[:2] == (None, None)


def test_label() -> None:
    criteria = PerceptionCriteria(methods="label", levels="perfect")
    result, scores, _ = criteria.get_result(make_record(tp=2))
    assert result == SuccessFail.SUCCESS
    assert scores == {"label": 100.0}


def test_velocity_and_speed_errors() -> None:
    criteria = PerceptionCriteria(
        methods=["velocity_x_error", "velocity_y_error", "speed_error"],
        levels=[1.0, 1.0, 1.0],
    )
    record = make_record(tp=2, est_velocity=(1.0, 0.0, 0.0), gt_velocity=(1.5, 0.5, 0.0))
    result, scores, _ = criteria.get_result(record)
    assert result == SuccessFail.SUCCESS
    assert scores["velocity_x_error"] == pytest.approx(0.5)
    assert scores["velocity_y_error"] == pytest.approx(0.5)
    assert scores["speed_error"] == pytest.approx((1.5**2 + 0.5**2) ** 0.5 - 1.0)


def test_velocity_error_without_velocity_is_zero() -> None:
    criteria = PerceptionCriteria(methods="velocity_x_error", levels=1.0)
    _, scores, _ = criteria.get_result(make_record(tp=2))
    assert scores == {"velocity_x_error": 0.0}


def test_yaw_error() -> None:
    criteria = PerceptionCriteria(methods="yaw_error", levels=1.0)
    result, scores, _ = criteria.get_result(make_record(tp=2))
    assert result == SuccessFail.SUCCESS
    assert scores == {"yaw_error": 0.0}


def test_metrics_score_uses_the_frame_map() -> None:
    criteria = PerceptionCriteria(
        methods=["metrics_score", "metrics_score_maph"], levels=[1.0, 1.0]
    )
    assert criteria.needs_frame_metrics is True
    result, scores, _ = criteria.get_result(make_record(tp=1))
    assert result == SuccessFail.SUCCESS
    assert scores == {"metrics_score": 25.0, "metrics_score_maph": 12.5}


def test_distance_filter_drops_far_objects() -> None:
    # TP pairs at x = 1..3, FP at x = 4, 5 and FN at x = 4
    criteria = PerceptionCriteria(
        methods="num_tp", levels="easy", filters=Filter(Distance="0.0-3.5", Region=None)
    )
    result, scores, filtered = criteria.get_result(make_record(tp=3, fp=2, fn=1))
    assert result == SuccessFail.SUCCESS
    assert (filtered.num_tp, filtered.num_fp, filtered.num_fn) == (3, 0, 0)
    assert scores == {"num_tp": 100.0}


def test_region_filter() -> None:
    criteria = PerceptionCriteria(
        methods="num_gt_tp",
        levels="easy",
        filters=Filter(Distance=None, Region={"x_position": "2.5,10.0", "y_position": None}),
    )
    _, _, filtered = criteria.get_result(make_record(tp=3, fp=1, fn=1))
    # objects at x = 3, 4 (fp) and 4 (fn) remain
    assert (filtered.num_tp, filtered.num_fp, filtered.num_fn) == (1, 1, 1)


def test_methods_and_levels_must_pair() -> None:
    with pytest.raises(AssertionError):
        PerceptionCriteria(methods=["num_tp", "label"], levels="easy")
