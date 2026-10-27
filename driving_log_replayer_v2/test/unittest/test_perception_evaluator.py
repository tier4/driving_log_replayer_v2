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

from driving_log_replayer_v2.perception.evaluator import PerceptionEvaluator
from driving_log_replayer_v2.perception.evaluator import PerceptionInvalidReason
from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
from driving_log_replayer_v2.post_process.evaluation_manager import parse_ignore_frames

FRAME_INTERVAL: int = 100000  # [us], 10 Hz
BASE_TIME: int = 1624157578750212


class FakeEvaluationManager:
    """Minimal stand-in for perception_eval's PerceptionEvaluationManager."""

    def __init__(self, num_gt_frames: int) -> None:
        self.ground_truth_frames = [
            SimpleNamespace(frame_name=str(i), unix_time=BASE_TIME + i * FRAME_INTERVAL)
            for i in range(num_gt_frames)
        ]
        self.frame_results = []
        self.added_frame_names: list[str] = []

    def get_ground_truth_now_frame(
        self,
        unix_time: int,
        *,
        interpolate_ground_truth: bool = False,  # noqa: ARG002
    ) -> SimpleNamespace | None:
        for frame in self.ground_truth_frames:
            if frame.unix_time == unix_time:
                return frame
        return None

    def add_frame_result(
        self,
        unix_time: int,
        ground_truth_now_frame: SimpleNamespace,
        estimated_objects: list,  # noqa: ARG002
        critical_object_filter_config: object,  # noqa: ARG002
        frame_pass_fail_config: object,  # noqa: ARG002
    ) -> SimpleNamespace:
        self.added_frame_names.append(ground_truth_now_frame.frame_name)
        frame_result = SimpleNamespace(
            frame_name=ground_truth_now_frame.frame_name, unix_time=unix_time
        )
        self.frame_results.append(frame_result)
        return frame_result


def create_evaluator(
    ignore_frames: IgnoreFrames, num_gt_frames: int = 5
) -> tuple[PerceptionEvaluator, FakeEvaluationManager]:
    """
    Create a PerceptionEvaluator without loading a t4_dataset.

    The constructor needs a real t4_dataset and writes log files, so only the instance variables
    used by evaluate_frame() / get_frame_coverage() are set here.
    """
    evaluator = PerceptionEvaluator.__new__(PerceptionEvaluator)
    inner_evaluator = FakeEvaluationManager(num_gt_frames)
    prefix = "_PerceptionEvaluator__"
    for name, value in {
        "skip_counter": 0,
        "evaluated_frame_position": 0,
        "skip_reasons": Counter(),
        "evaluator": inner_evaluator,
        "ignore_frames": ignore_frames,
        "logger": logging.getLogger("test_perception_evaluator"),
        "critical_object_filter_config": None,
        "frame_pass_fail_config": None,
        "evaluation_topic": "/perception/object_recognition/detection/objects",
    }.items():
        setattr(evaluator, prefix + name, value)
    return evaluator, inner_evaluator


def create_converted_data(frame_index: int) -> SimpleNamespace:
    header_timestamp = BASE_TIME + frame_index * FRAME_INTERVAL
    return SimpleNamespace(
        header_timestamp=header_timestamp,
        subscribed_timestamp=header_timestamp + 1000,
        data=SimpleNamespace(estimated_objects=[], interpolation=False),
    )


def test_ignored_frame_still_reaches_add_frame_result_for_the_pkl() -> None:
    """
    Ignored frames are still added to frame_results, kept for the pkl.

    They are only excluded from metrics/analysis later, by __remove_ignored_frames() in
    get_evaluation_results().
    """
    evaluator, inner_evaluator = create_evaluator(parse_ignore_frames("1,3"))

    results = [evaluator.evaluate_frame(create_converted_data(i)) for i in range(5)]

    assert [result.is_valid for result in results] == [True, False, True, False, True]
    assert results[1].invalid_reason == PerceptionInvalidReason.IGNORED_FRAME
    assert results[3].invalid_reason == PerceptionInvalidReason.IGNORED_FRAME
    # ignored frames are still added, so they are kept in the pkl
    assert inner_evaluator.added_frame_names == ["0", "1", "2", "3", "4"]
    assert [frame.frame_name for frame in inner_evaluator.frame_results] == [
        "0",
        "1",
        "2",
        "3",
        "4",
    ]
    # but excluded from the set get_evaluation_results() scores
    scored = _remove_ignored_frames(evaluator, inner_evaluator.frame_results)
    assert [frame.frame_name for frame in scored] == ["0", "2", "4"]


def test_first_n_ignores_by_position_not_by_frame_name() -> None:
    """first:N drops the first N evaluated frames, whatever their dataset index is."""
    evaluator, inner_evaluator = create_evaluator(parse_ignore_frames("first:2"))

    results = [evaluator.evaluate_frame(create_converted_data(i)) for i in range(1, 5)]

    assert [result.is_valid for result in results] == [False, False, True, True]
    # ignored frames are still added (kept for the pkl); only their position matters, not frame_name
    assert inner_evaluator.added_frame_names == ["1", "2", "3", "4"]


def test_skip_reason_is_reported_for_every_skip() -> None:
    evaluator, _ = create_evaluator(parse_ignore_frames("1"))

    # frame index 99 is not in the dataset -> no ground truth
    no_gt = evaluator.evaluate_frame(create_converted_data(99))

    invalid = create_converted_data(0)
    invalid.data.estimated_objects = None
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


def _apply_tail_ignore(evaluator: PerceptionEvaluator) -> None:
    """Simulate what get_evaluation_results() does to frame_results before __get_frame_coverage()."""
    prefix = "_PerceptionEvaluator__"
    inner = getattr(evaluator, prefix + "evaluator")
    ignore_tail_frames = getattr(evaluator, prefix + "ignore_tail_frames")
    inner.frame_results = ignore_tail_frames(inner.frame_results)


def _get_frame_coverage(evaluator: PerceptionEvaluator) -> dict:
    """get_frame_coverage() is now a private helper of get_evaluation_results(); call it directly."""
    prefix = "_PerceptionEvaluator__"
    return getattr(evaluator, prefix + "get_frame_coverage")()


def _remove_ignored_frames(evaluator: PerceptionEvaluator, frame_results: list) -> list:
    """Call the private helper get_evaluation_results() uses to drop N/A-B/first:N ignored frames."""
    prefix = "_PerceptionEvaluator__"
    return getattr(evaluator, prefix + "remove_ignored_frames")(frame_results)


def test_last_n_frames_are_removed_before_the_coverage_and_the_metrics() -> None:
    """last:N is applied on the frame results, before the pkl, the metrics and the coverage."""
    evaluator, inner_evaluator = create_evaluator(parse_ignore_frames("last:2"), num_gt_frames=5)

    for i in range(5):
        evaluator.evaluate_frame(create_converted_data(i))
    assert [frame.frame_name for frame in inner_evaluator.frame_results] == [
        "0",
        "1",
        "2",
        "3",
        "4",
    ]

    # get_evaluation_results() applies last:N to frame_results before get_frame_coverage() reads it
    _apply_tail_ignore(evaluator)
    assert [frame.frame_name for frame in inner_evaluator.frame_results] == ["0", "1", "2"]

    coverage = _get_frame_coverage(evaluator)
    assert coverage == {
        "GtFrames": 5,
        "GtFramesEvaluated": 3,
        "Coverage": 0.6,
        "SkipReasons": {"IGNORED_FRAME": 2},
    }

    # get_frame_coverage() no longer mutates state, so calling it again must not change the result
    assert _get_frame_coverage(evaluator) == coverage


def test_no_gt_frame_gives_zero_coverage() -> None:
    evaluator, _ = create_evaluator(IgnoreFrames(), num_gt_frames=0)
    assert _get_frame_coverage(evaluator) == {
        "GtFrames": 0,
        "GtFramesEvaluated": 0,
        "Coverage": 0.0,
        "SkipReasons": {},
    }
