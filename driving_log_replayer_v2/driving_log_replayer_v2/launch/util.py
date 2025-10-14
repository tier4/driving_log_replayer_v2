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

import json
from pathlib import Path

import rosbag2_py


def output_dummy_result_jsonl(result_json_path_str: str, summary: str = "RecordOnlyMode") -> None:
    jsonl_path_str = result_json_path_str + "l"
    dummy_result = {
        "Result": {"Success": True, "Summary": summary},
        "Stamp": {"System": 0.0},
        "Frame": {},
    }
    with Path(jsonl_path_str).open("w") as f:
        json.dump(dummy_result, f)


def output_dummy_result_bag(result_bag_path_str: str, storage_type: str) -> None:
    result_bag_path = Path(result_bag_path_str)

    storage_options = rosbag2_py.StorageOptions(uri=str(result_bag_path), storage_id=storage_type)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
