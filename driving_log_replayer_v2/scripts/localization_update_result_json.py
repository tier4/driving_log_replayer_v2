#!/usr/bin/env python3

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

import argparse
import json
from pathlib import Path

import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_jsonl_path", type=Path)
    parser.add_argument("result_archive_path", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    result_jsonl_path = args.result_jsonl_path
    result_archive_path = args.result_archive_path

    with result_jsonl_path.open("r") as f:
        result_data = [json.loads(line) for line in f.readlines()]

    relative_pose_summary_tsv_path = (
        result_archive_path
        / "compare_trajectories/localization__kinematic_state_result/relative_pose_summary.tsv"
    )
    df = pd.read_csv(relative_pose_summary_tsv_path, sep="\t")

    final_success = True
    final_summary = ""

    error_mean = df["error_mean"].item()
    ERROR_MEAN_THRESHOLD = 0.5
    if error_mean > ERROR_MEAN_THRESHOLD:
        final_success = False
        final_summary += f"{error_mean=} is too large."
    else:
        final_summary += f"{error_mean=}"

    result_data.append(
        {
            "Result": {
                "Success": final_success,
                "Summary": final_summary,
            }
        }
    )

    with result_jsonl_path.open("w") as f:
        for entry in result_data:
            f.write(json.dumps(entry) + "\n")
