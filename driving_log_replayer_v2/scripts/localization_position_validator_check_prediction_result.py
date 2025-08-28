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

"""Script for evaluating the position validator's output with annotation."""

import argparse
import json
from pathlib import Path

from pydantic import BaseModel

from driving_log_replayer_v2.scenario import load_scenario
from driving_log_replayer_v2.scenario import number
from driving_log_replayer_v2.scenario import Scenario


class CheckPredictionCondition(BaseModel):
    PassRate: number


class Conditions(BaseModel):
    CheckPrediction: CheckPredictionCondition


class AnnotationFormat(BaseModel):
    Timestamp: number
    ExpectedValidity: bool


class Evaluation(BaseModel):
    Conditions: Conditions
    Annotation: list[AnnotationFormat]


class LocalizationPositionValidatorScenario(Scenario):
    Evaluation: Evaluation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("scenario_path", type=Path)

    return parser.parse_args()


def compare_with_annotation(
    result_data: dict[str, any], annotation_data: AnnotationFormat, pass_rate: float
) -> tuple[bool, str]:
    annotation_len = len(annotation_data)
    if annotation_len == 0:
        return False, "No annotation was provided"

    # sort the data just in case
    # I don't think there is a use case for unordered timestamps
    annotation_data = sorted(annotation_data, key=lambda x: x.Timestamp)

    record_count = 0
    record_skip_count = 0
    success_count = 0
    annotation_index = 0
    current_annotation = annotation_data[annotation_index]
    first_annotation_ts = current_annotation.Timestamp

    for result_json in result_data:
        if "CheckInputAndOutputCount" not in result_json["Frame"]:
            record_skip_count += 1
            continue

        record_timestamp = result_json["Stamp"]["ROS"]

        # skip if prediction's timestamp is early than annotation
        if record_timestamp < first_annotation_ts:
            record_skip_count += 1
            continue

        # update the annotation if needed
        if annotation_index + 1 < annotation_len:
            next_annotation = annotation_data[annotation_index + 1]
            if record_timestamp >= next_annotation.Timestamp:
                annotation_index += 1
                current_annotation = next_annotation

        label = current_annotation.ExpectedValidity
        pred = result_json["Frame"]["CheckInputAndOutputCount"]["Prediction"]["is_position_valid"]

        record_count += 1
        if pred == label:
            success_count += 1

    # in percent
    success_rate = (success_count / record_count) * 100.0 if record_count > 0 else 0

    return success_rate >= pass_rate, (
        f"Correct prediction: {success_count}/{record_count} ({success_rate:.2f}%), "
        f"Pass rate for prediction: {pass_rate}, Skipped record: {record_skip_count}"
    )


def append_final_result(
    *, result_dir: Path, prediction_result: bool, prediction_summary: str
) -> None:
    result_jsonl_path = result_dir / "result.jsonl"
    with result_jsonl_path.open("r") as f:
        result_data = [json.loads(line) for line in f.readlines()]

    if len(result_data) > 0:
        # take "AND" among with the result and last data in the result.jsonl
        final_result = result_data[-1]
        final_result["Result"]["Success"] = final_result["Result"]["Success"] and prediction_result

        final_result["Result"]["Summary"] = (
            final_result["Result"]["Summary"] + ", " + prediction_summary
        )

        with result_jsonl_path.open("a") as f:
            f.write(json.dumps(final_result) + "\n")


def check_prediction_result(result_dir: Path, scenario_path: Path) -> None:
    result_jsonl_path = result_dir / "result.jsonl"

    result_data = []
    with Path(result_jsonl_path).open("r") as result_file:
        for each_json_data in result_file:
            record = json.loads(each_json_data)

            # skip the record does not have result in the key
            # usually, first line contains "Conditions"
            if "Result" in record:
                result_data.append(record)

    scenario_data = load_scenario(scenario_path, LocalizationPositionValidatorScenario)
    annotation_data = scenario_data.Evaluation.Annotation
    pass_rate = scenario_data.Evaluation.Conditions.CheckPrediction.PassRate

    result, summary = compare_with_annotation(result_data, annotation_data, pass_rate)

    append_final_result(result_dir, result, summary)


if __name__ == "__main__":
    args = parse_args()

    # output summary.json
    check_prediction_result(args.result_dir, args.scenario_path)
