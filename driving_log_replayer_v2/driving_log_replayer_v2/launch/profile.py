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

from ament_index_python.packages import get_package_share_directory
import yaml


def _get_profile_path(profile_name: str, config_subdir: str) -> Path:
    profile_file = Path(
        get_package_share_directory("driving_log_replayer_v2"),
        "config",
        config_subdir,
        f"{profile_name}.yaml",
    )
    if profile_file.is_symlink():
        profile_file = profile_file.resolve()
    return profile_file


def _load_profile_yaml(profile_name: str, config_subdir: str) -> dict | None:
    if not profile_name:
        return None

    profile_file = _get_profile_path(profile_name, config_subdir)
    if not profile_file.exists():
        return None

    with profile_file.open("r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        err_msg = f"Profile '{profile_name}' must be a mapping: {profile_file}"
        raise TypeError(err_msg)
    return data


def load_publish_remap_topics(profile_name: str, profile_type: str) -> list[str]:
    if not profile_name:
        return []

    if profile_type not in ("publish", "remap"):
        err_msg = f"Invalid profile_type: {profile_type}"
        raise ValueError(err_msg)
    profile_dict = _load_profile_yaml(profile_name, "publish_and_remap")
    if profile_dict is None:
        return []
    return profile_dict.get(profile_type, [])


def get_record_profile_name(conf: dict) -> str:
    if conf.get("record_profile", "") != "":
        return conf["record_profile"]
    return conf["use_case"]


def load_record_patterns(profile_name: str) -> list[str]:
    if not profile_name:
        return []

    profile_file = _get_profile_path(profile_name, "record")
    if not profile_file.exists():
        err_msg = f"Record profile not found: {profile_file}"
        raise FileNotFoundError(err_msg)

    with profile_file.open("r") as f:
        profile_dict = yaml.safe_load(f) or {}

    record_patterns = profile_dict.get("record")
    if record_patterns is None:
        err_msg = f"Record profile '{profile_name}' has no 'record' field: {profile_file}"
        raise ValueError(err_msg)
    if not isinstance(record_patterns, list):
        err_msg = f"Record profile '{profile_name}' 'record' field must be a list: {profile_file}"
        raise TypeError(err_msg)
    return [str(pattern) for pattern in record_patterns]


def build_record_topic_regex(conf: dict) -> str:
    if conf.get("override_topics_regex", "") != "":
        return conf["override_topics_regex"]

    patterns = load_record_patterns(get_record_profile_name(conf))
    return "|".join(patterns)
