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

import numpy as np
import pytest
from std_msgs.msg import Header
from t4perceval_test_utils import make_record

from driving_log_replayer_v2.perception.t4perceval_adapter.writers import FrameDescriptionWriter
from driving_log_replayer_v2.perception.t4perceval_adapter.writers import ground_truth_markers
from driving_log_replayer_v2.perception.t4perceval_adapter.writers import result_markers
from driving_log_replayer_v2.perception.t4perceval_adapter.writers import summarize_pass_fail


def test_summarize_pass_fail() -> None:
    assert summarize_pass_fail(make_record(tp=2, fp=1, fn=3)) == {
        "TP": "2 [car, car]",
        "FP": "1 [car]",
        "FN": "3 [car, car, car]",
        "TN": "null",
    }
    assert summarize_pass_fail(make_record()) == {
        "TP": "0 []",
        "FP": "0 []",
        "FN": "0 []",
        "TN": "null",
    }


def test_objects_description_order_and_schema() -> None:
    record = make_record(
        tp=2, fp=1, fn=1, est_velocity=(1.0, 0.0, 0.0), gt_velocity=(2.0, 0.0, 0.0)
    )
    objects = FrameDescriptionWriter.extract_pass_fail_objects_description(record)
    assert [(o["status"], o["object_type"]) for o in objects] == [
        ("TP", "GT"),
        ("TP", "GT"),
        ("FN", "GT"),
        ("TP", "EST"),
        ("TP", "EST"),
        ("FP", "EST"),
    ]
    for obj in objects:
        assert FrameDescriptionWriter.is_object_structure_valid(obj)
    gt_tp = objects[0]
    assert gt_tp["label"] == "car"
    assert gt_tp["uuid"] == "gt-0"
    assert gt_tp["position"] == {"x": 1.0, "y": 0.0, "z": 0.0}
    assert gt_tp["distance_from_ego"] == pytest.approx(1.0)
    assert gt_tp["orientation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    assert gt_tp["shape"] == {"x": 1.0, "y": 1.0, "z": 1.0}
    assert gt_tp["velocity_error"] == {"x": 1.0, "y": 0.0, "z": 0.0}
    assert gt_tp["bev_error"] == 0.0
    assert gt_tp["pose_covariance"] == []
    fp = objects[-1]
    assert fp["uuid"] == "est-2"
    assert fp["pose_error"] is None
    assert fp["bev_error"] is None


def test_objects_description_with_nan_velocity_is_valid() -> None:
    objects = FrameDescriptionWriter.extract_pass_fail_objects_description(make_record(tp=1))
    assert np.isnan(objects[0]["velocity"]["x"])
    assert np.isnan(objects[0]["velocity_error"]["x"])


def test_markers() -> None:
    record = make_record(tp=2, fp=1, fn=1)
    header = Header(frame_id="base_link")
    ground_truth = ground_truth_markers(record, header)
    assert [m.ns for m in ground_truth.markers] == ["ground_truth", "ground_truth_uuid"] * 3
    assert ground_truth.markers[1].text == "car\ngt-0"
    results = result_markers(record, header)
    namespaces = [m.ns for m in results.markers]
    assert namespaces == (
        ["tp_est", "tp_est_score"] * 2
        + ["tp_gt", "tp_gt_score"] * 2
        + ["fp", "fp_score", "fn", "fn_score"]
    )
    tp_est_box = results.markers[0]
    assert tp_est_box.pose.position.x == 1.0
    assert tp_est_box.pose.orientation.w == 1.0
    assert "PD: 0.50" in results.markers[1].text
    assert "IoU3D: 1.00" in results.markers[1].text
