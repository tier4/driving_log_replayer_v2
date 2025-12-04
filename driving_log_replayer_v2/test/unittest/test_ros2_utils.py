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

from driving_log_replayer_v2.post_process.ros2_utils import convert_to_sec


def test_nanosecond_to_sec() -> None:
    assert convert_to_sec("1500000000.123", "nanosecond") == "1.500000000123"

def test_microsecond_to_sec() -> None:
    assert convert_to_sec("1500123.456", "microsecond") == "1.500123456"

def test_millisecond_to_sec() -> None:
    assert convert_to_sec("1500.123", "millisecond") == "1.500123"

def test_sec_to_sec() -> None:
    assert convert_to_sec("15.123", "second") == "15.123"

def test_zero_value() -> None:
    assert convert_to_sec("0.0", "second") == "0"

def test_remove_leading_and_trailing_zeros() -> None:
    assert convert_to_sec("0001500.123000", "millisecond") == "1.500123"

def test_no_decimal_point() -> None:
    assert convert_to_sec("1500", "millisecond") == "1.5"

def test_unknown_unit_to_sec() -> None:
    with pytest.raises(ValueError):
        convert_to_sec("1500.123", "picosecond")  # noqa
