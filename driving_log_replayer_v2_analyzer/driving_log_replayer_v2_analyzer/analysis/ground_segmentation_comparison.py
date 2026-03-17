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


def visualize_comparison(jsonl_paths: list[Path], legends: list[str], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    metrics_plot = LinePlot()
    cm_plot = LinePlot()

    has_metrics_data = False
    has_cm_data = False

    for jsonl_path, original_legend in zip(jsonl_paths, legends):
        parser = JsonlParser(jsonl_path)
        
        # If legend is default (parent folder name), use the requested format: topic_name_folder_name
        if original_legend == jsonl_path.parent.name:
            full_topic_name = parser.get_evaluation_topic()
            # Shorten the topic name for the legend (e.g., .../obstacle_segmentation/ptv3/pointcloud)
            topic_parts = full_topic_name.strip("/").split("/")
            if len(topic_parts) > 3:
                topic_name = f".../{'/'.join(topic_parts[-3:])}"
            else:
                topic_name = full_topic_name
            legend = f"{topic_name}_{original_legend}"
        else:
            legend = original_legend

        # Plot Metrics Time Series (Focusing on Accuracy for comparison)
        metrics_data = parser.get_metrics_timeseries()
        if metrics_data:
            has_metrics_data = True
            # For comparison, we usually want to see one key metric like Accuracy from different runs
            data = [[d["x"], d["Accuracy"], legend] for d in metrics_data]
            metrics_plot.add_data(data, legend=legend)

        # Plot Confusion Matrix Elements (Focusing on TP for comparison)
        cm_data = parser.get_confusion_matrix_timeseries()
        if cm_data:
            has_cm_data = True
            data = [[d["x"], d["TP"], legend] for d in cm_data]
            cm_plot.add_data(data, legend=legend)

    if has_metrics_data:
        metrics_plot.plot(
            title="Ground Segmentation Accuracy Comparison",
            xlabel="Time [s]",
            ylabel="Accuracy [%]",
            markers=True,
        )
        metrics_plot.save_plot(output_dir / "ground_segmentation_accuracy_comparison")

    if has_cm_data:
        cm_plot.plot(
            title="Ground Segmentation TP Comparison",
            xlabel="Time [s]",
            ylabel="Count",
            markers=True,
        )
        cm_plot.save_plot(output_dir / "ground_segmentation_tp_comparison")
