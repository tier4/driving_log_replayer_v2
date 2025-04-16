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
    sample = analyzer.analyze()
    if sample.score is not None and sample.error is not None:
        # analysis setting
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
        pd.DataFrame(output_table).to_csv(save_path.joinpath("output_table.csv"), index=False)
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
    pd.DataFrame(output_table).to_csv(save_path.joinpath("output_table.csv"), index=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze perception result")
    parser.add_argument(
        "--evaluation-config-path", help="File path to scenario evaluation_config.pkl"
    )
    parser.add_argument("--scene-result-path", help="File path to scenario scene_result.pkl")
    parser.add_argument("--topic-name", default="", help="Evaluated topic name")
    parser.add_argument("--save-path", help="Directory path to save the output csv file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with Path.open(Path(args.evaluation_config_path), "rb") as f:
        evaluation_config = pickle.load(f)

    analyzer = PerceptionAnalyzer3D(evaluation_config)
    analyzer.add_from_pkl(args.scene_result_path)

    analyze(
        args.topic_name,
        analyzer,
        Path(args.save_path),
    )


if __name__ == "__main__":
    main()
