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
from collections.abc import Callable
from pathlib import Path
import pickle

import numpy as np
import pandas as pd
from perception_eval.common.status import MatchingStatus
from perception_eval.tool import PerceptionAnalyzer3D
from perception_eval.tool.utils import PlotAxes


def analyze(
    analyzer: PerceptionAnalyzer3D,
    save_path: Path,
    max_distance: str,
    distance_interval: str,
    topic_name: str,
) -> None:
    """Analyze evaluation results in detail and export them as a flattened csv table."""
    evaluation_task = analyzer.config.evaluation_task
    # decide the columns to be used
    sample = analyzer.analyze()
    if sample.score is not None and sample.error is not None:
        distance_range = tuple(
            (i * int(distance_interval), (i + 1) * int(distance_interval))
            for i in range(int(max_distance) // int(distance_interval))
        )
        labels = sample.score.index.to_list()
        score_metrics = sample.score.columns.to_list()
        error_metrics = sample.error.index.get_level_values(1).unique().tolist()
        error_statistics = sample.error.columns.to_list()
    else:
        pd.DataFrame([]).to_csv(save_path.joinpath("analysis_result.csv"), index=False)
        return

    all_row = []
    for distance_min, distance_max in distance_range:
        analysis_result = analyzer.analyze(distance=(distance_min, distance_max))
        for label in labels:
            # make base table
            row: dict[str, str | float] = {
                "evaluation_task": evaluation_task,
                "topic_name": topic_name,
                "label": label,
                "distance": f"{distance_min}-{distance_max}",
            }

            # add each score as column or nan if target object does not exist
            score_result: pd.DataFrame = analysis_result.score
            for score in score_metrics:
                row[score] = (
                    score_result.loc[label, score] if score_result is not None else float("nan")
                )

            # add each error w/ statistics as column or nan if target object does not exist
            error_result: pd.DataFrame = analysis_result.error
            for error in error_metrics:
                for stat in error_statistics:
                    row[error + "_" + stat] = (
                        error_result.loc[(label, error), stat]
                        if error_result is not None
                        else float("nan")
                    )

            # add consecutive fn spans w/ statistics as column or nan if target object does not exist
            consecutive_fn_spans = get_consecutive_fn_spans(
                analyzer, distance=(distance_min, distance_max), label=label, statistics=np.max
            )
            for stat in error_statistics:
                row["consecutive_fn_spans" + "_" + stat] = apply_statistics(
                    consecutive_fn_spans, stat
                )

            all_row.append(row)
    all_row = pd.DataFrame(all_row)
    all_row.fillna("nan").to_csv(save_path.joinpath("analysis_result.csv"), index=False)


def get_consecutive_fn_spans(
    analyzer: PerceptionAnalyzer3D,
    distance: tuple[int, int],
    label: list[str] | str,
    statistics: Callable,
) -> list[float]:
    df = get_df(analyzer, distance=distance, columns={"label": label})

    if len(df) == 0:
        return []

    # calculate each consecutive fn spans of objects
    object_spans: dict[str, list[float]] = {}
    for uuid in pd.unique(df.loc[pd.IndexSlice[:, "ground_truth"], "uuid"]):
        if uuid is None:
            continue

        fn_df = df.loc[(df["uuid"] == uuid) & (df["status"] == "FN")]
        frame_diff_list = np.diff(fn_df["frame"], append=0)
        timestamp_list = fn_df["timestamp"].to_list()

        # check in time series
        # NOTE: only one fn frame is not considered
        consecutive_fn_count: int = 0
        spans: list[float] = []
        start_timestamp: float
        for frame_diff, timestamp in zip(frame_diff_list, timestamp_list, strict=True):
            # status is consecutive fn
            if frame_diff == 1.0:
                if consecutive_fn_count == 0:
                    start_timestamp = timestamp
                consecutive_fn_count += 1
            # status is start of new fn and handle the last consecutive fn
            else:
                if consecutive_fn_count > 0:
                    spans.append((timestamp - start_timestamp) * 1e-6)
                consecutive_fn_count = 0

        object_spans[uuid] = spans
    # calculate statistics of each consecutive fn spans of objects
    return [statistics(each_span) for each_span in object_spans.values() if len(each_span) > 0]


def apply_statistics(value: list[float], statistics: str) -> float:
    if len(value) == 0:
        return float("nan")

    value = np.array(value)
    value = value[~np.isnan(value)]

    if statistics == "average":
        value = np.average(value)
    elif statistics == "rms":
        value = np.sqrt(np.square(value).mean())
    elif statistics == "std":
        value = np.std(value)
    elif statistics == "max":
        value = np.max(np.abs(value))
    elif statistics == "min":
        value = np.min(np.abs(value))
    elif statistics == "percentile_99":
        value = np.percentile(np.abs(value), 99)
    else:
        err_msg = f"Invalid statistics: {statistics}"
        raise ValueError(err_msg)
    return value


def get_df(
    analyzer: PerceptionAnalyzer3D,
    distance: tuple[int, int] | None,
    columns: dict[str, list[str] | str] | None,
) -> pd.DataFrame:
    # extract matching data in the specified columns
    df = analyzer.get(**columns) if columns is not None else analyzer.get()
    return analyzer.filter_by_distance(df=df, distance=distance) if distance is not None else df


def plot(
    analyzer: PerceptionAnalyzer3D,
) -> None:
    for column in [
        "x",
        "y",
        "yaw",
        "width",
        "length",
        "bev_area",
        "vx",
        "vy",
        "v_yaw",
        "speed",
        "nn_plane",
        "distance",
    ]:
        analyzer.plot_error(
            mode=PlotAxes.FRAME,
            columns=column,
        )
    analyzer.plot_num_object(
        mode=PlotAxes.FRAME,
    )
    analyzer.plot_ratio(
        mode=PlotAxes.FRAME,
        status=MatchingStatus.TP,
    )
    analyzer.plot_ratio(
        mode=PlotAxes.FRAME,
        status=MatchingStatus.FP,
    )
    analyzer.plot_ratio(
        mode=PlotAxes.FRAME,
        status=MatchingStatus.FN,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze perception result")
    parser.add_argument(
        "--evaluation-config-path",
        required=True,
        type=Path,
        help="File path to evaluation_config.pkl",
    )
    parser.add_argument(
        "--scene-result",
        required=True,
        type=Path,
        help="File or Directory path to scene_result.pkl",
    )
    parser.add_argument(
        "--save-path", required=True, type=Path, help="Directory path to save the output csv file"
    )
    parser.add_argument("--max-distance", required=True, help="Maximum distance for analysis")
    parser.add_argument(
        "--distance-interval", required=True, help="Distance interval for analysis."
    )
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
        analyzer,
        args.save_path,
        args.max_distance,
        args.distance_interval,
        args.topic_name,
    )

    plot(analyzer)


if __name__ == "__main__":
    main()
