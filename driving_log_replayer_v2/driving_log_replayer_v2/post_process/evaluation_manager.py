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

from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
from typing import Any
from typing import TYPE_CHECKING
from typing import TypeVar

if TYPE_CHECKING:
    from collections.abc import Iterator

    from driving_log_replayer_v2.post_process.evaluator import EvaluatorType
    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.post_process.runner import ConvertedData
    from driving_log_replayer_v2.scenario import ScenarioType

POSITION_SEPARATOR = ":"
FIRST_KEYWORD = "first"
LAST_KEYWORD = "last"


@dataclass(frozen=True)
class IgnoreFrames:
    """
    Frames to be excluded from the evaluation.

    Attributes:
        frame_names (tuple[int, ...]): Dataset sample indices (`frame_name`) to be ignored.
        first (int): Number of first evaluated frames to be ignored (`first:N`).
        last (int): Number of last evaluated frames to be ignored (`last:N`).

    NOTE: `frame_names` is matched against the dataset sample index, while `first` / `last` are
    matched against the position of the frame in the sequence of evaluated frames.

    """

    frame_names: tuple[int, ...] = ()
    first: int = 0
    last: int = 0

    def __contains__(self, frame_name: int) -> bool:
        """Keep the `frame_name in ignore_frames` usage of the previous list[int] interface."""
        return frame_name in self.frame_names

    def __iter__(self) -> Iterator[int]:
        return iter(self.frame_names)

    def __len__(self) -> int:
        return len(self.frame_names)

    def __bool__(self) -> bool:
        return bool(self.frame_names) or self.first > 0 or self.last > 0

    def should_ignore(self, frame_name: int, position: int) -> bool:
        """
        Whether the frame is ignored by `frame_name` or by `first:N`.

        Args:
            frame_name (int): Dataset sample index of the frame.
            position (int): 1-based position of the frame in the sequence of evaluated frames.

        Returns:
            bool: True if the frame must be excluded from the evaluation.

        NOTE: `last:N` cannot be decided while streaming, it is applied in post-processing.

        """
        return frame_name in self.frame_names or position <= self.first


def _parse_position_token(item: str) -> tuple[str, int]:
    keyword, _, num_str = item.partition(POSITION_SEPARATOR)
    keyword = keyword.strip().lower()
    num_str = num_str.strip()
    if keyword not in (FIRST_KEYWORD, LAST_KEYWORD) or not num_str.isdigit() or int(num_str) <= 0:
        err_msg = (
            f"Invalid position format in ignore_frames: '{item}'. "
            f"Expected format is '{FIRST_KEYWORD}:N' or '{LAST_KEYWORD}:N' where N is a positive integer."
        )
        raise ValueError(err_msg)
    return keyword, int(num_str)


def parse_ignore_frames(ignore_frames: str) -> IgnoreFrames:
    """
    Parse the ignore_frames setting.

    Supported tokens, separated by comma:
        `N`: dataset sample index (frame_name) of the frame to ignore.
        `A-B`: closed range of dataset sample indices to ignore.
        `first:N`: first N evaluated frames.
        `last:N`: last N evaluated frames.

    Args:
        ignore_frames (str): ignore_frames setting, e.g. "0,3-5,first:2,last:1".

    Returns:
        IgnoreFrames: Parsed setting.

    """
    ignore_set: set[int] = set()
    first = 0
    last = 0
    if not ignore_frames or ignore_frames == "None":
        return IgnoreFrames()

    for raw_item in ignore_frames.split(","):
        item = raw_item.strip()
        if not item:
            continue

        if POSITION_SEPARATOR in item:
            keyword, num = _parse_position_token(item)
            if keyword == FIRST_KEYWORD:
                first = max(first, num)
            else:
                last = max(last, num)

        elif "-" in item:
            # Split the range into start and end
            start_str, end_str = item.split("-", 1)

            # Check if both extracted strings are digits
            if start_str.isdigit() and end_str.isdigit():
                start, end = int(start_str), int(end_str)
                ignore_set.update(range(start, end + 1))
            else:
                err_msg = (
                    f"Invalid range format in ignore_frames: '{item}'. "
                    "Expected format is 'start-end' where start and end are integers."
                )
                raise ValueError(err_msg)

        elif item.isdigit():
            # Check if the single string is composed of digits
            ignore_set.add(int(item))

        else:
            err_msg = (
                f"Invalid integer format in ignore_frames: '{item}'. Expected a single integer."
            )
            raise ValueError(err_msg)

    return IgnoreFrames(frame_names=tuple(sorted(ignore_set)), first=first, last=last)


class TailIgnoreBuffer:
    """
    Delay the writing of the last N valid frames so that they can be ignored (`last:N`).

    `last:N` is not known while streaming: the last N evaluated frames are only known once the
    rosbag is fully read. Instead of writing the result line and re-writing result.jsonl
    afterwards, the writing of the frames is delayed until it is known that they are not part of
    the last N valid frames. Invalid frames (skipped frames) are kept in the buffer as well so
    that the order of the result lines is preserved.
    """

    def __init__(self, num_ignore: int) -> None:
        self._num_ignore = max(num_ignore, 0)
        self._buffer: list[tuple[Any, bool]] = []  # (item, is_valid)

    @property
    def enabled(self) -> bool:
        return self._num_ignore > 0

    def _num_valid(self) -> int:
        return sum(1 for _, is_valid in self._buffer if is_valid)

    def push(self, item: Any, *, is_valid: bool) -> list[Any]:
        """
        Push a frame and get back the frames which can be written now.

        Args:
            item (Any): Frame to be written later.
            is_valid (bool): Whether the frame is a valid evaluated frame.

        Returns:
            list[Any]: Frames which are not part of the last N valid frames, in order.

        """
        self._buffer.append((item, is_valid))
        writable: list[Any] = []
        while self._num_valid() > self._num_ignore:
            # write the head of the buffer until one valid frame has been released
            while self._buffer:
                buffered_item, buffered_is_valid = self._buffer.pop(0)
                writable.append(buffered_item)
                if buffered_is_valid:
                    break
        return writable

    def flush(self) -> list[tuple[Any, bool]]:
        """Get the remaining frames, i.e. the last N valid frames and the skipped frames after them."""
        remaining = self._buffer
        self._buffer = []
        return remaining


class EvaluationManager(ABC):
    """
    Base class for evaluation manager.

    Responsible for following items:
        initializing subclass of Evaluator for each evaluation topic
        managing evaluation process for each frame
    """

    def __init__(
        self,
        scenario: ScenarioType,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        ignore_frames: str,
    ) -> None:
        # instance variables
        self._scenario: ScenarioType = scenario
        self._evaluators: dict[str, EvaluatorType]
        self._degradation_topics: list[str]

        # argument has higher priority than scenario setting
        if not ignore_frames:
            parsed_ignore_frames = parse_ignore_frames(
                getattr(scenario.Evaluation, "ignore_frames", "")
            )
        else:
            parsed_ignore_frames = parse_ignore_frames(ignore_frames)

        self._ignore_frames: IgnoreFrames = parsed_ignore_frames

        self._set_evaluators(
            t4_dataset_path, result_archive_path, evaluation_topics_with_task, parsed_ignore_frames
        )
        self._set_degradation_topics(degradation_topic)

    @abstractmethod
    def _set_evaluators(
        self,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topics_with_task: dict[str, list[str]],
        ignore_frames: IgnoreFrames,
    ) -> None:
        """
        Set evaluators for each evaluation topic.

        Args:
            t4_dataset_path (str): Path to T4 dataset.
            result_archive_path (str): Path to result archive.
            evaluation_topics_with_task (dict[str, list[str]]): Dictionary mapping evaluation topics to their tasks.
            ignore_frames (IgnoreFrames): Frames to be ignored during evaluation.

        """
        raise NotImplementedError

    @abstractmethod
    def _set_degradation_topics(self, degradation_topic: str) -> None:
        """
        Set the degradation topic for the evaluation manager.

        Args:
            degradation_topic (str): Topic name for degradation information.

        """
        raise NotImplementedError

    def evaluate_frame(
        self,
        topic_name: str,
        converted_data: ConvertedData,
    ) -> FrameResult:
        """
        Evaluate a frame for a given topic.

        Args:
            topic_name (str): Name of the topic to evaluate.
            data (ConvertedData): Data to be evaluated.

        Returns:
            FrameResult: The result of the frame evaluation.

        """
        evaluator = self._evaluators[topic_name]
        return evaluator.evaluate_frame(converted_data)

    def get_degradation_topics(self) -> list[str]:
        """Get the degradation topics for the evaluation manager."""
        return self._degradation_topics

    def get_evaluation_topics(self) -> list[str]:
        """Get the evaluation topics for the evaluation manager."""
        return list(self._evaluators.keys())

    def get_ignore_frames(self) -> IgnoreFrames:
        """Get the parsed ignore_frames setting used by the evaluators."""
        return self._ignore_frames


ManagerType = TypeVar("ManagerType", bound=EvaluationManager)
