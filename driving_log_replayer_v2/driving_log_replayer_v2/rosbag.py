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

from rosbag2_py import Reindexer
from rosbag2_py import StorageOptions


def create_metadata_yaml(bag_path: str) -> None:
    """
    Create metadata.yaml for the rosbag in bag_path if it does not exist.

    Args:
        bag_path (str): Path to the directory containing the rosbag files.

    """
    metadata_path = Path(bag_path).joinpath("metadata.yaml")
    if metadata_path.exists():
        return
    mcap_files = list(Path(bag_path).glob("*.mcap"))
    db3_files = list(Path(bag_path).glob("*.db3"))
    if mcap_files and db3_files:
        err_msg = f"Both mcap and sqlite3 files exist in the rosbag directory: {bag_path}"
        raise RuntimeError(err_msg)
    storage_type: str
    if mcap_files:
        storage_type = "mcap"
    elif db3_files:
        storage_type = "sqlite3"
    else:
        err_msg = f"No rosbag files found in the directory: {bag_path}"
        raise RuntimeError(err_msg)
    storage_options = StorageOptions(storage_id=storage_type, uri=Path(bag_path).as_posix())
    Reindexer().reindex(storage_options)
