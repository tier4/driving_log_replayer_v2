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

from pathlib import Path

from driving_log_replayer_v2_analyzer.data.ground_segmentation import JsonlParser
from driving_log_replayer_v2_analyzer.plot.line_plot import LinePlot


def visualize(input_jsonl: Path, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load result.jsonl
    parser = JsonlParser(input_jsonl)

    # Plot Metrics Time Series
    metrics_data = parser.get_metrics_timeseries()
    if metrics_data:
        metrics_plot = LinePlot()
        for metric in ["Accuracy", "Precision", "Recall", "F1-score"]:
            data = [[d["x"], d[metric], metric] for d in metrics_data]
            metrics_plot.add_data(data, legend=metric)

        metrics_plot.plot(
            title="Ground Segmentation Metrics Time Series",
            xlabel="Time [s]",
            ylabel="Score [%]",
            markers=True,
        )
        metrics_plot.save_plot(output_dir / "ground_segmentation_metrics")
    else:
        print("No metrics data found in the input file.")

    # Plot Confusion Matrix Elements Time Series
    cm_data = parser.get_confusion_matrix_timeseries()
    if cm_data:
        cm_plot = LinePlot()
        for element in ["TP", "FP", "TN", "FN"]:
            data = [[d["x"], d[element], element] for d in cm_data]
            cm_plot.add_data(data, legend=element)

        cm_plot.plot(
            title="Ground Segmentation Confusion Matrix Elements",
            xlabel="Time [s]",
            ylabel="Count",
            markers=True,
        )
        cm_plot.save_plot(output_dir / "ground_segmentation_confusion_matrix")
    else:
        print("No confusion matrix data found in the input file.")
