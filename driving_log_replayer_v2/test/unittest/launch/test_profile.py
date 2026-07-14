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

from pathlib import Path
import sys
from unittest.mock import MagicMock

import pytest

sys.modules.setdefault("ament_index_python", MagicMock())
sys.modules.setdefault("ament_index_python.packages", MagicMock())

from driving_log_replayer_v2.launch import profile  # noqa: E402

PACKAGE_SHARE = Path(__file__).resolve().parents[3]


@pytest.fixture(autouse=True)
def mock_package_share(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        profile,
        "get_package_share_directory",
        lambda _package_name: PACKAGE_SHARE.as_posix(),
    )


def test_get_record_profile_name_defaults_to_use_case() -> None:
    conf = {"record_profile": "", "use_case": "planning_control"}
    assert profile.get_record_profile_name(conf) == "planning_control"


def test_get_record_profile_name_uses_explicit_profile() -> None:
    conf = {"record_profile": "custom_profile", "use_case": "planning_control"}
    assert profile.get_record_profile_name(conf) == "custom_profile"


def test_load_record_patterns_perception() -> None:
    patterns = profile.load_record_patterns("perception")
    assert patterns[0] == "^/tf$"
    assert "^/planning/planning_factors/.*" in patterns


def test_load_record_patterns_ndt_convergence_is_empty() -> None:
    assert profile.load_record_patterns("ndt_convergence") == []


def test_load_record_patterns_missing_profile_raises() -> None:
    with pytest.raises(FileNotFoundError, match="Record profile not found"):
        profile.load_record_patterns("nonexistent_profile")


def test_build_record_topic_regex_defaults_to_use_case_profile() -> None:
    conf = {
        "override_topics_regex": "",
        "record_profile": "",
        "use_case": "time_step_based_trajectory",
    }
    regex = profile.build_record_topic_regex(conf)
    assert "^/tf$" in regex
    assert "^/planning/.*$" in regex
    assert "^/control/.*$" not in regex


def test_build_record_topic_regex_uses_record_profile() -> None:
    conf = {
        "override_topics_regex": "",
        "record_profile": "diagnostics",
        "use_case": "planning_control",
    }
    regex = profile.build_record_topic_regex(conf)
    assert (
        regex
        == "^/tf$|^/diagnostics$|^/system/processing_time_checker/metrics$|^/driving_log_replayer/.*"
    )


def test_build_record_topic_regex_override_has_highest_priority() -> None:
    conf = {
        "override_topics_regex": "^/tf$|^/custom$",
        "record_profile": "diagnostics",
        "use_case": "planning_control",
    }
    assert profile.build_record_topic_regex(conf) == "^/tf$|^/custom$"


def test_build_record_topic_regex_ndt_convergence_is_empty() -> None:
    conf = {
        "override_topics_regex": "",
        "record_profile": "",
        "use_case": "ndt_convergence",
    }
    assert profile.build_record_topic_regex(conf) == ""


def test_load_publish_remap_topics_planning_control() -> None:
    topics = profile.load_publish_remap_topics("planning_control", "publish")
    assert "/tf" in topics
    assert "/planning/trajectory" in topics
    assert len(topics) > 0


def test_load_publish_remap_topics_missing_profile_returns_empty() -> None:
    assert profile.load_publish_remap_topics("nonexistent_profile", "publish") == []
    assert profile.load_publish_remap_topics("", "publish") == []


def test_load_publish_remap_topics_invalid_profile_type_raises() -> None:
    with pytest.raises(ValueError, match="Invalid profile_type"):
        profile.load_publish_remap_topics("planning_control", "invalid")


def test_load_publish_remap_topics_non_mapping_raises(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    profile_file = tmp_path / "bad_profile.yaml"
    profile_file.write_text("- not\n- a\n- mapping\n", encoding="utf-8")
    monkeypatch.setattr(profile, "_get_profile_path", lambda *_args: profile_file)
    with pytest.raises(TypeError, match="must be a mapping"):
        profile.load_publish_remap_topics("bad_profile", "publish")
