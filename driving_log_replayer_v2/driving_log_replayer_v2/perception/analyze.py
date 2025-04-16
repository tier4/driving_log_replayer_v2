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
from pathlib import Path
import pickle

import pandas as pd
from perception_eval.tool import PerceptionAnalyzer3D

DISTANCE_RANGE = tuple(i * 10 for i in range(21))


def analyze(topic_name: str, analyzer: PerceptionAnalyzer3D, save_path: Path) -> None:
    """Analyze evaluation results in detail and export them as a flattened csv table."""
    evaluation_task = analyzer.config.evaluation_task
    # decide the columns to be used
    sample = analyzer.analyze()
    if sample.score is not None and sample.error is not None:
        distance_range = DISTANCE_RANGE
        labels = sample.score.index.to_list()
        score_metrics = sample.score.columns.to_list()
        error_metrics = sample.error.index.get_level_values(1).unique().tolist()
        error_statistics = sample.error.columns.to_list()
    else:
        output_table = [
            {
                "evaluation_task": evaluation_task,
                "topic_name": topic_name,
            }
        ]
        pd.DataFrame(output_table).to_csv(save_path.joinpath("analysis_result.csv"), index=False)
        return

    output_table = []
    for i in range(len(distance_range) - 1):
        analysis_result = analyzer.analyze(distance=(distance_range[i], distance_range[i + 1]))
        for label in labels:
            # make base table
            row: dict[str, str | float] = {
                "evaluation_task": evaluation_task,
                "topic_name": topic_name,
                "label": label,
                "distance": f"{distance_range[i]}-{distance_range[i + 1]}",
            }

            # add each score as column
            score_result = analysis_result.score.to_dict()
            for score in score_metrics:
                row[score] = score_result[score][label]

            # add each error as column w/ statistics
            error_result = analysis_result.error.to_dict()
            for error in error_metrics:
                for stat in error_statistics:
                    row[error + "_" + stat] = error_result[stat][(label, error)]

            output_table.append(row)
    pd.DataFrame(output_table).to_csv(save_path.joinpath("analysis_result.csv"), index=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze perception result")
    parser.add_argument(
        "--evaluation-config-path", type=Path, help="File path to evaluation_config.pkl"
    )
    parser.add_argument(
        "--scene-result", type=Path, help="File or Directory path to scene_result.pkl"
    )
    parser.add_argument("--save-path", type=Path, help="Directory path to save the output csv file")
    parser.add_argument("--topic-name", default="", help="Evaluated topic name")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with args.evaluation_config_path.open("rb") as f:
        evaluation_config = pickle.load(f)

    if args.scene_result.is_dir():
        scene_result_list = list(args.scene_result.rglob("scene_result.pkl"))
        all_scene_result = []
        for scene_result in scene_result_list:
            with scene_result.open("rb") as f:
                frame_result = pickle.load(f)
                all_scene_result.extend(frame_result)
    elif args.scene_result.name == "scene_result.pkl":
        with args.scene_result.open("rb") as f:
            all_scene_result = pickle.load(f)
    else:
        err_msg = f"Invalid scene result path: {args.scene_result}"
        raise ValueError(err_msg)

    analyzer = PerceptionAnalyzer3D(evaluation_config)
    analyzer.add(all_scene_result)

    analyze(
        args.topic_name,
        analyzer,
        args.save_path,
    )


if __name__ == "__main__":
    main()
