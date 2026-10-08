#!/usr/bin/env python3

# Copyright (c) 2024 TIER IV.inc
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

"""Aggregate the perception results of several scenes into one database_result.json."""

import argparse
from pathlib import Path

import simplejson as json

from driving_log_replayer_v2.perception.analyze import load_analysis


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Aggregate the scene_result.t4eval archives found below a directory"
    )
    parser.add_argument(
        "-r",
        "--result_root_directory",
        required=True,
        help="Directory below which the scene_result.t4eval archives are searched",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output json path, defaults to <result_root_directory>/database_result.json",
    )
    args = parser.parse_args()

    root = Path(args.result_root_directory)
    analysis = load_analysis(root)
    result = analysis.analyze()
    output = Path(args.output) if args.output else root.joinpath("database_result.json")
    with output.open("w") as f:
        json.dump(
            {
                "Score": result.score,
                "Error": result.error,
                "ConfusionMatrix": result.confusion_matrix,
            },
            f,
            ignore_nan=True,
            indent=2,
        )
    print(f"Wrote {output}")  # noqa: T201


if __name__ == "__main__":
    main()
