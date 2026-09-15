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

import pytest

from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
from driving_log_replayer_v2.post_process.evaluation_manager import parse_ignore_frames
from driving_log_replayer_v2.post_process.evaluation_manager import TailIgnoreBuffer


def test_parse_ignore_frames_empty() -> None:
    for value in ("", "None", " , "):
        parsed = parse_ignore_frames(value)
        assert parsed == IgnoreFrames()
        assert not parsed
        assert list(parsed) == []


def test_parse_ignore_frames_index_and_range() -> None:
    parsed = parse_ignore_frames("3, 0-4, 10, 10")
    assert parsed == IgnoreFrames(frame_names=(0, 1, 2, 3, 4, 10))
    assert parsed
    # keep the list[int] like interface of the previous implementation
    assert 10 in parsed  # noqa
    assert 11 not in parsed  # noqa
    assert list(parsed) == [0, 1, 2, 3, 4, 10]
    assert len(parsed) == len(parsed.frame_names)


def test_parse_ignore_frames_position_tokens() -> None:
    assert parse_ignore_frames("first:2,last:3") == IgnoreFrames(first=2, last=3)

    assert parse_ignore_frames("0-4,10,first:1,last:2") == IgnoreFrames(
        frame_names=(0, 1, 2, 3, 4, 10), first=1, last=2
    )

    # the largest value wins if the token is used twice
    assert parse_ignore_frames("first:1,first:5,last:2,LAST:7") == IgnoreFrames(first=5, last=7)


@pytest.mark.parametrize(
    "ignore_frames",
    [
        "first",
        "first:",
        "first:0",
        "first:-1",
        "first:a",
        "last:1.5",
        "head:1",
        "0-a",
        "abc",
    ],
)
def test_parse_ignore_frames_invalid(ignore_frames: str) -> None:
    with pytest.raises(ValueError):  # noqa
        parse_ignore_frames(ignore_frames)


def test_ignore_frames_should_ignore() -> None:
    ignore_frames = parse_ignore_frames("10-11,first:2")
    # ignored by first:N, position is 1-based
    assert ignore_frames.should_ignore(frame_name=0, position=1)
    assert ignore_frames.should_ignore(frame_name=1, position=2)
    assert not ignore_frames.should_ignore(frame_name=2, position=3)
    # ignored by frame_name
    assert ignore_frames.should_ignore(frame_name=10, position=11)
    assert ignore_frames.should_ignore(frame_name=11, position=12)
    assert not ignore_frames.should_ignore(frame_name=12, position=13)
    # last:N is not decided here
    assert not parse_ignore_frames("last:3").should_ignore(frame_name=100, position=100)


def test_tail_ignore_buffer_disabled() -> None:
    buffer = TailIgnoreBuffer(0)
    assert not buffer.enabled
    assert TailIgnoreBuffer(-1).enabled is False


def test_tail_ignore_buffer_keeps_last_valid_frames() -> None:
    buffer = TailIgnoreBuffer(2)
    assert buffer.enabled

    written = []
    for i in range(5):
        written += buffer.push(i, is_valid=True)

    # the last 2 valid frames are still buffered
    assert written == [0, 1, 2]
    assert buffer.flush() == [(3, True), (4, True)]
    # the buffer is empty after the flush
    assert buffer.flush() == []


def test_tail_ignore_buffer_keeps_order_with_skipped_frames() -> None:
    """Skipped frames do not count for last:N but must keep their position in result.jsonl."""
    buffer = TailIgnoreBuffer(1)
    frames = [
        ("valid0", True),
        ("skip0", False),
        ("valid1", True),
        ("skip1", False),
        ("skip2", False),
        ("valid2", True),
        ("skip3", False),
    ]

    written = []
    for item, is_valid in frames:
        written += buffer.push(item, is_valid=is_valid)

    assert written == ["valid0", "skip0", "valid1"]
    # the last valid frame and the skipped frames after it are left for the post process
    assert buffer.flush() == [
        ("skip1", False),
        ("skip2", False),
        ("valid2", True),
        ("skip3", False),
    ]


def test_tail_ignore_buffer_fewer_frames_than_ignored() -> None:
    buffer = TailIgnoreBuffer(5)
    written = []
    for i in range(3):
        written += buffer.push(i, is_valid=True)
    assert written == []
    assert buffer.flush() == [(0, True), (1, True), (2, True)]
