# Copyright (c) 2025 TIER IV.inc
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

from __future__ import annotations

from collections import Counter
import logging
from types import SimpleNamespace
from typing import TYPE_CHECKING

import numpy as np
from t4perceval import Detections3D
from t4perceval_test_utils import make_labels
from t4perceval_test_utils import make_record

from driving_log_replayer_v2.perception.evaluator import PerceptionEvaluator
from driving_log_replayer_v2.perception.evaluator import PerceptionInvalidReason
from driving_log_replayer_v2.perception.t4perceval_adapter.conversions import EstimationFrame
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import GroundTruthFrame
from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
from driving_log_replayer_v2.post_process.evaluation_manager import parse_ignore_frames

if TYPE_CHECKING:
    from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import (
        PerceptionFrameRecord,
    )

FRAME_INTERVAL: int = 100_000_000  # [ns], 10 Hz
BASE_TIME: int = 1_624_157_578_750_212_000


class FakeGroundTruthScene:
    """Minimal stand-in for GroundTruthScene: frames at exact timestamps, no objects."""

    def __init__(self, num_gt_frames: int) -> None:
        self.frames = [
            GroundTruthFrame(i, BASE_TIME + i * FRAME_INTERVAL) for i in range(num_gt_frames)
        ]
        self.num_frames = num_gt_frames

    def nearest(self, timestamp_ns: int) -> GroundTruthFrame | None:
        for frame in self.frames:
            if frame.timestamp_ns == timestamp_ns:
                return frame
        return None


def create_evaluator(
    ignore_frames: IgnoreFrames, num_gt_frames: int = 5
) -> tuple[PerceptionEvaluator, list[str]]:
    """
    Create a PerceptionEvaluator without loading a t4_dataset.

    The constructor needs a real t4_dataset and writes log files, so only the instance variables
    used by evaluate_frame() / get_frame_coverage() are set here, and the per-frame evaluation is
    replaced by a stub which records the evaluated frame names.
    """
    evaluator = PerceptionEvaluator.__new__(PerceptionEvaluator)
    added_frame_names: list[str] = []
    registry = make_labels()

    def fake_evaluate(
        data: EstimationFrame,  # noqa: ARG001
        ground_truth_frame: GroundTruthFrame,
        frame_index: int,
        timestamp_ns: int,
    ) -> PerceptionFrameRecord:
        added_frame_names.append(ground_truth_frame.frame_name)
        return make_record(
            registry=registry,
            frame_index=frame_index,
            frame_name=ground_truth_frame.frame_name,
            timestamp_ns=timestamp_ns,
        )

    prefix = "_PerceptionEvaluator__"
    for name, value in {
        "skip_counter": 0,
        "evaluated_frame_position": 0,
        "skip_reasons": Counter(),
        "ground_truth": FakeGroundTruthScene(num_gt_frames),
        "ignore_frames": ignore_frames,
        "logger": logging.getLogger("test_perception_evaluator"),
        "frame_results": [],
        "scored_frame_results": None,
        "warned_header_frame_id": False,
        "config": SimpleNamespace(frame_id="base_link"),
        "evaluation_topic": "/perception/object_recognition/detection/objects",
        "evaluate": fake_evaluate,
    }.items():
        setattr(evaluator, prefix + name, value)
    return evaluator, added_frame_names


def create_converted_data(frame_index: int) -> SimpleNamespace:
    header_timestamp = BASE_TIME + frame_index * FRAME_INTERVAL
    archetype = Detections3D(
        position=np.empty((0, 3)),
        quaternion=np.empty((0, 4)),
        size=np.empty((0, 3)),
        class_id=np.empty(0, dtype=np.int32),
        confidence=np.empty(0),
    )
    return SimpleNamespace(
        header_timestamp=header_timestamp,
        subscribed_timestamp=header_timestamp + 1000,
        data=EstimationFrame(
            kind="detections",
            archetype=archetype,
            header_frame_id="base_link",
            uuids=(),
            pose_covariance=np.empty((0, 36)),
            twist_covariance=np.empty((0, 36)),
        ),
    )


def _frame_results(evaluator: PerceptionEvaluator) -> list:
    prefix = "_PerceptionEvaluator__"
    return getattr(evaluator, prefix + "frame_results")


def _apply_tail_ignore(evaluator: PerceptionEvaluator) -> list:
    """Simulate what get_evaluation_results() does to frame_results before __get_frame_coverage()."""
    prefix = "_PerceptionEvaluator__"
    ignore_tail_frames = getattr(evaluator, prefix + "ignore_tail_frames")
    scored = ignore_tail_frames(_frame_results(evaluator))
    setattr(evaluator, prefix + "scored_frame_results", scored)
    return scored


def _get_frame_coverage(evaluator: PerceptionEvaluator) -> dict:
    """get_frame_coverage() is a private helper of get_evaluation_results(); call it directly."""
    prefix = "_PerceptionEvaluator__"
    return getattr(evaluator, prefix + "get_frame_coverage")()


def _remove_ignored_frames(evaluator: PerceptionEvaluator, frame_results: list) -> list:
    """Call the private helper get_evaluation_results() uses to drop N/A-B/first:N ignored frames."""
    prefix = "_PerceptionEvaluator__"
    return getattr(evaluator, prefix + "remove_ignored_frames")(frame_results)


def test_ignored_frame_is_still_evaluated_and_kept_for_the_archive() -> None:
    """
    Ignored frames are still kept in frame_results, so they land in the archive.

    They are only excluded from metrics/analysis later, by __remove_ignored_frames() in
    get_evaluation_results().
    """
    evaluator, added_frame_names = create_evaluator(parse_ignore_frames("1,3"))

    results = [evaluator.evaluate_frame(create_converted_data(i)) for i in range(5)]

    assert [result.is_valid for result in results] == [True, False, True, False, True]
    assert results[1].invalid_reason == PerceptionInvalidReason.IGNORED_FRAME
    assert results[3].invalid_reason == PerceptionInvalidReason.IGNORED_FRAME
    assert added_frame_names == ["0", "1", "2", "3", "4"]
    assert [frame.frame_name for frame in _frame_results(evaluator)] == ["0", "1", "2", "3", "4"]
    assert [frame.frame_index for frame in _frame_results(evaluator)] == [1, 2, 3, 4, 5]
    # but excluded from the set get_evaluation_results() scores
    scored = _remove_ignored_frames(evaluator, _frame_results(evaluator))
    assert [frame.frame_name for frame in scored] == ["0", "2", "4"]


def test_first_n_ignores_by_position_not_by_frame_name() -> None:
    """first:N drops the first N evaluated frames, whatever their dataset index is."""
    evaluator, added_frame_names = create_evaluator(parse_ignore_frames("first:2"))

    results = [evaluator.evaluate_frame(create_converted_data(i)) for i in range(1, 5)]

    assert [result.is_valid for result in results] == [False, False, True, True]
    assert added_frame_names == ["1", "2", "3", "4"]
    scored = _remove_ignored_frames(evaluator, _frame_results(evaluator))
    assert [frame.frame_name for frame in scored] == ["3", "4"]


def test_skip_reason_is_reported_for_every_skip() -> None:
    evaluator, _ = create_evaluator(parse_ignore_frames("1"))

    # frame index 99 is not in the dataset -> no ground truth
    no_gt = evaluator.evaluate_frame(create_converted_data(99))

    invalid = create_converted_data(0)
    invalid.data = "Unexpected footprint length: num_footprint=2"
    invalid_result = evaluator.evaluate_frame(invalid)

    ignored = evaluator.evaluate_frame(create_converted_data(1))

    assert [result.invalid_reason for result in (no_gt, invalid_result, ignored)] == [
        PerceptionInvalidReason.NO_GROUND_TRUTH,
        PerceptionInvalidReason.INVALID_ESTIMATED_OBJECTS,
        PerceptionInvalidReason.IGNORED_FRAME,
    ]
    # every skipped frame is counted, whatever its reason is
    assert [result.skip_counter for result in (no_gt, invalid_result, ignored)] == [1, 2, 3]

    assert _get_frame_coverage(evaluator)["SkipReasons"] == {
        "IGNORED_FRAME": 1,
        "INVALID_ESTIMATED_OBJECTS": 1,
        "NO_GROUND_TRUTH": 1,
    }


def test_frame_coverage_reports_the_scored_denominator() -> None:
    evaluator, _ = create_evaluator(IgnoreFrames(), num_gt_frames=5)

    for i in (0, 1, 2):
        evaluator.evaluate_frame(create_converted_data(i))
    # a second estimate bound to the same ground truth frame must not be counted twice
    evaluator.evaluate_frame(create_converted_data(2))
    # no ground truth for these
    evaluator.evaluate_frame(create_converted_data(99))

    assert _get_frame_coverage(evaluator) == {
        "GtFrames": 5,
        "GtFramesEvaluated": 3,
        "Coverage": 0.6,
        "SkipReasons": {"NO_GROUND_TRUTH": 1},
    }


def test_last_n_frames_are_removed_before_the_coverage_and_the_metrics() -> None:
    """last:N is applied on the frame results, before the archive, the metrics and the coverage."""
    evaluator, _ = create_evaluator(parse_ignore_frames("last:2"), num_gt_frames=5)

    for i in range(5):
        evaluator.evaluate_frame(create_converted_data(i))
    assert [frame.frame_name for frame in _frame_results(evaluator)] == ["0", "1", "2", "3", "4"]

    # get_evaluation_results() applies last:N to frame_results before get_frame_coverage() reads it
    scored = _apply_tail_ignore(evaluator)
    assert [frame.frame_name for frame in scored] == ["0", "1", "2"]

    coverage = _get_frame_coverage(evaluator)
    assert coverage == {
        "GtFrames": 5,
        "GtFramesEvaluated": 3,
        "Coverage": 0.6,
        "SkipReasons": {"IGNORED_FRAME": 2},
    }

    # get_frame_coverage() does not mutate state, so calling it again must not change the result
    assert _get_frame_coverage(evaluator) == coverage


def test_no_gt_frame_gives_zero_coverage() -> None:
    evaluator, _ = create_evaluator(IgnoreFrames(), num_gt_frames=0)
    assert _get_frame_coverage(evaluator) == {
        "GtFrames": 0,
        "GtFramesEvaluated": 0,
        "Coverage": 0.0,
        "SkipReasons": {},
    }
