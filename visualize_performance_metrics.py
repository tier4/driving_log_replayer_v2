#!/usr/bin/env python3
"""
Performance metrics visualization script.

Reads PTv3 and CenterPoint debug topics from a ROS 2 bag and produces
time-series plots, a Gantt chart, and a GPU contention timeline.
"""

import argparse
import csv
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
except ImportError:
    print("Error: rosbags library is not installed.")
    print("Install with: pip3 install rosbags")
    sys.exit(1)


# --------------------------------------------------------------------------- #
# Topic definitions
# --------------------------------------------------------------------------- #

PTV3_TOPICS = {
    "pipeline_latency_ms": "/perception/obstacle_segmentation/ptv3/debug/pipeline_latency_ms",
    "total_ms":            "/perception/obstacle_segmentation/ptv3/debug/processing_time/total_ms",
    "preprocess_ms":       "/perception/obstacle_segmentation/ptv3/debug/processing_time/preprocess_ms",
    "inference_ms":        "/perception/obstacle_segmentation/ptv3/debug/processing_time/inference_ms",
    "postprocess_ms":      "/perception/obstacle_segmentation/ptv3/debug/processing_time/postprocess_ms",
}

# CenterPoint topic prefix; overridable with --cp-prefix
_CP_DEFAULT_PREFIX = "/perception/object_recognition/detection/centerpoint/lidar_centerpoint"

CP_TOPICS = {
    "pipeline_latency_ms": f"{_CP_DEFAULT_PREFIX}/debug/pipeline_latency_ms",
    "processing_time_ms":  f"{_CP_DEFAULT_PREFIX}/debug/processing_time_ms",
    "preprocess_ms":       f"{_CP_DEFAULT_PREFIX}/debug/processing_time/preprocess_ms",
    "inference_ms":        f"{_CP_DEFAULT_PREFIX}/debug/processing_time/inference_ms",
    "postprocess_ms":      f"{_CP_DEFAULT_PREFIX}/debug/processing_time/postprocess_ms",
}

ALL_TOPICS = list(PTV3_TOPICS.values()) + list(CP_TOPICS.values())

STAGE_COLORS = {
    "preprocess":  "#4CAF50",   # green
    "inference":   "#F44336",   # red (GPU occupied)
    "postprocess": "#2196F3",   # blue
}

MODEL_COLORS = {
    "ptv3":        {"preprocess": "#80CBC4", "inference": "#EF5350", "postprocess": "#90CAF9"},
    "centerpoint": {"preprocess": "#A5D6A7", "inference": "#FF8A65", "postprocess": "#CE93D8"},
}


# --------------------------------------------------------------------------- #
# Data classes
# --------------------------------------------------------------------------- #

@dataclass
class ExecutionWindow:
    """Reconstructed execution window for one frame (all times in seconds)."""
    publish_time: float
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float

    @property
    def postprocess_end(self) -> float:
        return self.publish_time

    @property
    def postprocess_start(self) -> float:
        return self.publish_time - self.postprocess_ms / 1000.0

    @property
    def inference_end(self) -> float:
        return self.postprocess_start

    @property
    def inference_start(self) -> float:
        return self.inference_end - self.inference_ms / 1000.0

    @property
    def preprocess_end(self) -> float:
        return self.inference_start

    @property
    def preprocess_start(self) -> float:
        return self.preprocess_end - self.preprocess_ms / 1000.0

    @property
    def total_start(self) -> float:
        return self.preprocess_start


class PerformanceMetrics:
    """Stores time-series data for one dataset."""

    def __init__(self, name: str):
        self.name = name
        self.data: Dict[str, List[Tuple[float, float]]] = {t: [] for t in ALL_TOPICS}

    def add_data(self, topic: str, timestamp: float, value: float):
        if topic in self.data:
            self.data[topic].append((timestamp, value))

    def get_values(self, topic: str) -> Tuple[np.ndarray, np.ndarray]:
        if not self.data.get(topic):
            return np.array([]), np.array([])
        ts, vs = zip(*self.data[topic])
        return np.array(ts), np.array(vs)

    def get_statistics(self, topic: str) -> dict:
        _, vs = self.get_values(topic)
        if len(vs) == 0:
            return {"count": 0, "mean": 0, "std": 0, "min": 0, "max": 0, "median": 0}
        return {
            "count": len(vs),
            "mean":   float(np.mean(vs)),
            "std":    float(np.std(vs)),
            "min":    float(np.min(vs)),
            "max":    float(np.max(vs)),
            "median": float(np.median(vs)),
        }

    def reconstruct_windows(self, topics: Dict[str, str]) -> List[ExecutionWindow]:
        """Pair preprocess/inference/postprocess time-series by index and return ExecutionWindow list."""
        _, pre_vs = self.get_values(topics["preprocess_ms"])
        _, inf_vs = self.get_values(topics["inference_ms"])
        pos_ts, pos_vs = self.get_values(topics["postprocess_ms"])

        n = min(len(pre_vs), len(inf_vs), len(pos_vs))
        if n == 0:
            return []

        windows = []
        for i in range(n):
            # publish_time = rosbag timestamp of postprocess topic (closest to processing end)
            windows.append(ExecutionWindow(
                publish_time=pos_ts[i],
                preprocess_ms=pre_vs[i],
                inference_ms=inf_vs[i],
                postprocess_ms=pos_vs[i],
            ))
        return windows

    def get_input_latency(self, model_name: str, topics: Dict[str, str]) -> Tuple[np.ndarray, np.ndarray]:
        """Calculate input_latency = pipeline_latency - total_processing_time."""
        p_ts, p_vs = self.get_values(topics["pipeline_latency_ms"])
        t_key = "total_ms" if model_name == "PTv3" else "processing_time_ms"
        t_ts, t_vs = self.get_values(topics[t_key])

        # Align by timestamp (approximate)
        if len(p_vs) == 0 or len(t_vs) == 0:
            return np.array([]), np.array([])
        
        # Simple index-based alignment assuming 1:1 message ratio
        n = min(len(p_vs), len(t_vs))
        input_latencies = p_vs[:n] - t_vs[:n]
        return p_ts[:n], input_latencies

# --------------------------------------------------------------------------- #
# GPU contention analysis
# --------------------------------------------------------------------------- #

def plot_contention_impact(
    metrics: PerformanceMetrics,
    cp_windows: List[ExecutionWindow],
    ptv3_windows: List[ExecutionWindow],
    output_dir: Path,
):
    """Compare inference_ms and pipeline_latency for contended vs non-contended frames."""
    if not cp_windows or not ptv3_windows:
        return

    overlaps = detect_inference_overlaps(cp_windows, ptv3_windows)
    
    def is_contended(window: ExecutionWindow):
        for s, e in overlaps:
            if max(window.inference_start, s) < min(window.inference_end, e):
                return True
        return False

    results = {}
    for name, windows, topics in [
        ("PTv3", ptv3_windows, PTV3_TOPICS),
        ("CenterPoint", cp_windows, CP_TOPICS)
    ]:
        if not windows: continue
        
        contended_inf = [w.inference_ms for w in windows if is_contended(w)]
        normal_inf    = [w.inference_ms for w in windows if not is_contended(w)]
        
        # Get pipeline latency
        _, p_vs = metrics.get_values(topics["pipeline_latency_ms"])
        # Align pipeline latency with windows (assuming 1:1)
        p_vs = p_vs[:len(windows)]
        contended_pipe = [p_vs[i] for i, w in enumerate(windows) if is_contended(w)]
        normal_pipe    = [p_vs[i] for i, w in enumerate(windows) if not is_contended(w)]

        results[name] = {
            "inf": (normal_inf, contended_inf),
            "pipe": (normal_pipe, contended_pipe)
        }

    # Plotting
    fig, axes = plt.subplots(len(results), 2, figsize=(14, 5 * len(results)))
    if len(results) == 1: axes = [axes]

    for i, (model, data) in enumerate(results.items()):
        # Inference comparison
        ax_inf = axes[i][0]
        ax_inf.boxplot(data["inf"], labels=["Normal", "Contended"])
        ax_inf.set_title(f"{model} - Inference Time Impact")
        ax_inf.set_ylabel("ms")
        ax_inf.grid(True, alpha=0.3)

        # Pipeline latency comparison
        ax_pipe = axes[i][1]
        ax_pipe.boxplot(data["pipe"], labels=["Normal", "Contended"])
        ax_pipe.set_title(f"{model} - Pipeline Latency Impact")
        ax_pipe.set_ylabel("ms")
        ax_pipe.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f"contention_impact_{metrics.name}.png", dpi=150)
    plt.close()


# --------------------------------------------------------------------------- #
# ROS bag reading
# --------------------------------------------------------------------------- #

def read_rosbag_metrics(bag_path: Path, name: str, cp_prefix: str) -> Optional[PerformanceMetrics]:
    """Read metrics for all models from a ROS 2 bag."""
    # Apply CenterPoint prefix override
    cp_topics = {
        k: v.replace(_CP_DEFAULT_PREFIX, cp_prefix)
        for k, v in CP_TOPICS.items()
    }
    target_topics = list(PTV3_TOPICS.values()) + list(cp_topics.values())

    if not bag_path.exists():
        print(f"Warning: bag file not found: {bag_path}")
        return None

    metrics = PerformanceMetrics(name)
    # Register cp_prefix-adjusted topics in metrics.data
    for t in list(cp_topics.values()):
        if t not in metrics.data:
            metrics.data[t] = []

    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        float64_stamped_def = "builtin_interfaces/Time stamp\nfloat64 data\n"
        types = get_types_from_msg(
            float64_stamped_def, "autoware_internal_debug_msgs/msg/Float64Stamped"
        )
        typestore.register(types)

        bag_dir = bag_path.parent
        if not (bag_dir / "metadata.yaml").exists():
            print(f"  Warning: metadata.yaml not found in: {bag_dir}")
            return None

        with Reader(bag_dir) as reader:
            connections = [c for c in reader.connections if c.topic in target_topics]
            if not connections:
                print(f"Warning: no target topics found in: {bag_path}")
                return None

            for conn, timestamp, rawdata in reader.messages(connections=connections):
                try:
                    msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                    metrics.add_data(conn.topic, timestamp / 1e9, msg.data)
                except Exception as e:
                    print(f"Deserialization failed: {e}")

        print(f"Loaded: {name}")
        for t in target_topics:
            n = len(metrics.data.get(t, []))
            if n > 0:
                print(f"  {t.split('/')[-1]}: {n} msgs")

        return metrics

    except Exception as e:
        print(f"Error: failed to read {bag_path}: {e}")
        return None


# --------------------------------------------------------------------------- #
# GPU contention detection
# --------------------------------------------------------------------------- #

def detect_inference_overlaps(
    cp_windows: List[ExecutionWindow],
    ptv3_windows: List[ExecutionWindow],
) -> List[Tuple[float, float]]:
    """Return time intervals where CenterPoint and PTv3 inference overlap."""
    overlaps = []
    for cp in cp_windows:
        for ptv3 in ptv3_windows:
            start = max(cp.inference_start, ptv3.inference_start)
            end   = min(cp.inference_end,   ptv3.inference_end)
            if start < end:
                overlaps.append((start, end))
    return overlaps


# --------------------------------------------------------------------------- #
# Gantt chart
# --------------------------------------------------------------------------- #

def plot_gantt_chart(
    cp_windows: List[ExecutionWindow],
    ptv3_windows: List[ExecutionWindow],
    output_dir: Path,
    window_sec: float = 1.0,
    start_offset_sec: Optional[float] = None,
    dataset_name: str = "",
):
    """Draw execution windows as a Gantt chart and highlight GPU contention.

    Layout (top to bottom):
      Row 2  CenterPoint  - preprocess (green) / inference (red) / postprocess (blue) bars
      Row 1  PTv3         - same
      Row 0  GPU busy     - filled where either model is running inference
                            both models simultaneously -> red (contention)
                            only one model           -> light orange
    x-axis is in milliseconds; each frame is annotated with its index.
    """
    if not cp_windows and not ptv3_windows:
        print("Warning: no data to draw Gantt chart")
        return

    all_windows = cp_windows + ptv3_windows
    t_global_start = min(w.total_start for w in all_windows)
    t_global_end   = max(w.postprocess_end for w in all_windows)

    if start_offset_sec is None:
        mid = (t_global_start + t_global_end) / 2.0
        t_start = mid - window_sec / 2.0
    else:
        t_start = t_global_start + start_offset_sec
    t_end = t_start + window_sec

    cp_vis   = [w for w in cp_windows   if w.postprocess_end >= t_start and w.total_start <= t_end]
    ptv3_vis = [w for w in ptv3_windows if w.postprocess_end >= t_start and w.total_start <= t_end]
    overlaps = detect_inference_overlaps(cp_vis, ptv3_vis)

    # helper: convert x-axis to milliseconds
    def ms(t: float) -> float:
        return (t - t_start) * 1000.0

    window_ms = window_sec * 1000.0

    # ---- Layout ----
    # ax_main: CenterPoint / PTv3 bars (upper 2 rows)
    # ax_gpu:  GPU busy indicator (bottom row)
    _, (ax_main, ax_gpu) = plt.subplots(
        2, 1, figsize=(18, 6),
        gridspec_kw={"height_ratios": [3, 1]},
        sharex=True,
    )

    Y_CP   = 2.0
    Y_PTV3 = 1.0
    BAR_H  = 0.55

    # ---- Draw CenterPoint / PTv3 bars ----
    def draw_windows(ax, windows, y, colors):
        for i, w in enumerate(windows):
            for stage, (s, e) in [
                ("preprocess",  (w.preprocess_start,  w.preprocess_end)),
                ("inference",   (w.inference_start,   w.inference_end)),
                ("postprocess", (w.postprocess_start, w.postprocess_end)),
            ]:
                s_ms = max(ms(s), 0.0)
                e_ms = min(ms(e), window_ms)
                if s_ms >= e_ms:
                    continue
                ax.barh(y, e_ms - s_ms, left=s_ms, height=BAR_H,
                        color=colors[stage], alpha=0.9,
                        linewidth=0.5, edgecolor="white", align="center")
                # Label inference bar with duration if wide enough
                if stage == "inference" and (e_ms - s_ms) > window_ms * 0.03:
                    ax.text(
                        s_ms + (e_ms - s_ms) / 2, y,
                        f"{w.inference_ms:.0f}ms",
                        ha="center", va="center",
                        fontsize=7, color="white", fontweight="bold",
                    )
            # Frame index near preprocess start
            fn_x = ms(w.preprocess_start)
            if 0 <= fn_x <= window_ms:
                ax.text(fn_x + 1, y + BAR_H / 2 + 0.05,
                        f"#{i}", fontsize=6, color="gray", va="bottom")

    draw_windows(ax_main, cp_vis,   Y_CP,   MODEL_COLORS["centerpoint"])
    draw_windows(ax_main, ptv3_vis, Y_PTV3, MODEL_COLORS["ptv3"])

    # ---- GPU busy indicator ----
    # PTv3 only (light orange)
    for w in ptv3_vis:
        s_ms = max(ms(w.inference_start), 0.0)
        e_ms = min(ms(w.inference_end), window_ms)
        if s_ms < e_ms:
            ax_gpu.barh(0.5, e_ms - s_ms, left=s_ms, height=0.6,
                        color="#FFB74D", alpha=0.5, align="center")
    # CP only (light orange)
    for w in cp_vis:
        s_ms = max(ms(w.inference_start), 0.0)
        e_ms = min(ms(w.inference_end), window_ms)
        if s_ms < e_ms:
            ax_gpu.barh(0.5, e_ms - s_ms, left=s_ms, height=0.6,
                        color="#FFB74D", alpha=0.5, align="center")
    # Contention (red, drawn on top)
    for (s, e) in overlaps:
        s_ms = max(ms(s), 0.0)
        e_ms = min(ms(e), window_ms)
        if s_ms < e_ms:
            ax_gpu.barh(0.5, e_ms - s_ms, left=s_ms, height=0.6,
                        color="red", alpha=0.7, align="center")

    # ---- Axis / label settings ----
    ax_main.set_yticks([Y_PTV3, Y_CP])
    ax_main.set_yticklabels(["PTv3", "CenterPoint"], fontsize=12)
    ax_main.set_ylim(0.5, 2.7)
    ax_main.grid(True, axis="x", alpha=0.3, linestyle="--")

    ax_gpu.set_yticks([0.5])
    ax_gpu.set_yticklabels(["GPU\nbusy"], fontsize=9)
    ax_gpu.set_ylim(0.0, 1.0)
    ax_gpu.set_xlabel("Time [ms]  (relative to window start)", fontsize=11)
    ax_gpu.grid(True, axis="x", alpha=0.3, linestyle="--")

    # x-axis ticks: ~50 ms intervals
    tick_interval = max(10, int(window_ms / 20 / 10) * 10)
    ax_gpu.set_xticks(np.arange(0, window_ms + tick_interval, tick_interval))

    title = (
        f"GPU Execution Windows — {dataset_name}\n"
        f"window: +{t_start - t_global_start:.1f}s ~ +{t_end - t_global_start:.1f}s  "
        f"(CP {len(cp_vis)} frames, PTv3 {len(ptv3_vis)} frames, "
        f"contention {len(overlaps)} regions)"
    )
    ax_main.set_title(title, fontsize=12, fontweight="bold")

    # ---- Legend ----
    legend_handles = [
        mpatches.Patch(color=MODEL_COLORS["centerpoint"]["preprocess"],  label="preprocess (CPU+GPU memcpy)"),
        mpatches.Patch(color=MODEL_COLORS["centerpoint"]["inference"],   label="inference (TensorRT / GPU occupied)"),
        mpatches.Patch(color=MODEL_COLORS["centerpoint"]["postprocess"], label="postprocess (CPU)"),
        mpatches.Patch(color="#FFB74D", alpha=0.6,                       label="GPU busy (one model only)"),
        mpatches.Patch(color="red",     alpha=0.7,                       label="GPU contention (both models simultaneously)"),
    ]
    ax_main.legend(handles=legend_handles, loc="upper right", fontsize=8, ncol=1,
                   framealpha=0.9)

    # ---- How to read annotation ----
    note = (
        "Right edge of bar = publish time\n"
        "Bar width = stage duration\n"
        "White label = inference_ms"
    )
    ax_main.text(0.01, 0.97, note, transform=ax_main.transAxes,
                 fontsize=7, va="top", color="#555555",
                 bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))

    plt.tight_layout()
    safe_name = dataset_name.replace("/", "_").replace(" ", "_")
    output_file = output_dir / f"gantt_gpu_contention_{safe_name}.png"
    plt.savefig(output_file, dpi=150)
    plt.close()
    print(f"Saved: {output_file}")


# --------------------------------------------------------------------------- #
# GPU contention timeline
# --------------------------------------------------------------------------- #

def plot_gpu_contention_timeline(
    cp_windows: List[ExecutionWindow],
    ptv3_windows: List[ExecutionWindow],
    output_dir: Path,
    dataset_name: str = "",
    bin_sec: float = 1.0,
):
    """Visualize GPU contention over the full run."""
    if not cp_windows or not ptv3_windows:
        print("Warning: insufficient data to draw GPU contention timeline")
        return

    all_windows = cp_windows + ptv3_windows
    t0 = min(w.total_start for w in all_windows)

    def rel(t):
        return t - t0

    overlaps = detect_inference_overlaps(cp_windows, ptv3_windows)

    # --- Top: inference_ms time-series + contention highlight ---
    axes = plt.subplots(3, 1, figsize=(16, 10), sharex=False)[1]

    ax1 = axes[0]
    cp_inf_ts   = np.array([rel(w.inference_end) for w in cp_windows])
    cp_inf_ms   = np.array([w.inference_ms for w in cp_windows])
    ptv3_inf_ts = np.array([rel(w.inference_end) for w in ptv3_windows])
    ptv3_inf_ms = np.array([w.inference_ms for w in ptv3_windows])

    ax1.plot(cp_inf_ts,   cp_inf_ms,   color=MODEL_COLORS["centerpoint"]["inference"],
             alpha=0.8, linewidth=1.2, label="CenterPoint inference_ms")
    ax1.plot(ptv3_inf_ts, ptv3_inf_ms, color=MODEL_COLORS["ptv3"]["inference"],
             alpha=0.8, linewidth=1.2, label="PTv3 inference_ms")

    for (s, e) in overlaps:
        ax1.axvspan(rel(s), rel(e), alpha=0.2, color="red")

    ax1.set_ylabel("inference_ms", fontsize=11)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_title(
        f"GPU Contention Timeline - {dataset_name}" if dataset_name else "GPU Contention Timeline",
        fontsize=14, fontweight="bold"
    )

    # --- Middle: total_ms time-series ---
    ax2 = axes[1]
    for windows, _, color, label in [
        (cp_windows,   CP_TOPICS,   MODEL_COLORS["centerpoint"]["inference"], "CenterPoint pipeline_latency_ms"),
        (ptv3_windows, PTV3_TOPICS, MODEL_COLORS["ptv3"]["inference"],        "PTv3 pipeline_latency_ms"),
    ]:
        ts = np.array([rel(w.postprocess_end) for w in windows])
        # Approximate total latency as sum of stage durations
        lat = np.array([w.preprocess_ms + w.inference_ms + w.postprocess_ms for w in windows])
        ax2.plot(ts, lat, color=color, alpha=0.8, linewidth=1.2, label=label)

    for (s, e) in overlaps:
        ax2.axvspan(rel(s), rel(e), alpha=0.2, color="red")

    ax2.set_ylabel("total_ms (approx latency)", fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # --- Bottom: contention rate per bin ---
    ax3 = axes[2]

    t_max = max(rel(w.postprocess_end) for w in all_windows)
    bins = np.arange(0, t_max + bin_sec, bin_sec)
    contention_duration = np.zeros(len(bins) - 1)

    for (s, e) in overlaps:
        rs, re = rel(s), rel(e)
        for i in range(len(bins) - 1):
            bs, be = bins[i], bins[i + 1]
            overlap = min(re, be) - max(rs, bs)
            if overlap > 0:
                contention_duration[i] += overlap

    contention_pct = contention_duration / bin_sec * 100.0
    ax3.bar(bins[:-1], contention_pct, width=bin_sec * 0.9,
            color="red", alpha=0.6, align="edge")
    ax3.set_xlabel("Time [s]", fontsize=11)
    ax3.set_ylabel("GPU contention [%]", fontsize=11)
    ax3.set_ylim(0, 105)
    ax3.grid(True, alpha=0.3, axis="y")

    # display overall contention rate
    total_contention_sec = sum(e - s for s, e in overlaps)
    total_sec = t_max
    ax3.set_title(
        f"GPU Contention Rate per {bin_sec}s bin  "
        f"(Total: {total_contention_sec:.2f}s / {total_sec:.1f}s = "
        f"{total_contention_sec / total_sec * 100:.1f}%)",
        fontsize=11
    )

    plt.tight_layout()
    safe_name = dataset_name.replace("/", "_").replace(" ", "_")
    output_file = output_dir / f"gpu_contention_timeline_{safe_name}.png"
    plt.savefig(output_file, dpi=150)
    plt.close()
    print(f"Saved: {output_file}")


# --------------------------------------------------------------------------- #
# CSV Export
# --------------------------------------------------------------------------- #

def export_metrics_to_csv(metrics_list: List[PerformanceMetrics], output_dir: Path, cp_prefix: str):
    """Export all time-series and statistics to unified master CSV files."""
    cp_topics = {k: v.replace(_CP_DEFAULT_PREFIX, cp_prefix) for k, v in CP_TOPICS.items()}
    
    # 1. Export Unified Raw Metrics
    raw_csv_path = output_dir / "all_raw_metrics.csv"
    raw_header = [
        "dataset", "model", "timestamp_sec", "relative_time_sec", 
        "pipeline_latency_ms", "node_processing_time_ms", 
        "preprocess_ms", "inference_ms", "postprocess_ms", "input_latency_ms"
    ]
    
    with open(raw_csv_path, "w", newline="") as f_raw:
        writer_raw = csv.writer(f_raw)
        writer_raw.writerow(raw_header)
        
        for metrics in metrics_list:
            for model_name, topics in [("PTv3", PTV3_TOPICS), ("CenterPoint", cp_topics)]:
                t_key = "total_ms" if model_name == "PTv3" else "processing_time_ms"
                columns = ["pipeline_latency_ms", t_key, "preprocess_ms", "inference_ms", "postprocess_ms"]
                
                data_map = {}
                all_timestamps = set()
                for col in columns:
                    ts, vs = metrics.get_values(topics[col])
                    data_map[col] = {t: v for t, v in zip(ts, vs)}
                    all_timestamps.update(ts)
                
                ts_lat, vs_lat = metrics.get_input_latency(model_name, topics)
                data_map["input_latency_ms"] = {t: v for t, v in zip(ts_lat, vs_lat)}
                all_timestamps.update(ts_lat)

                sorted_ts = sorted(list(all_timestamps))
                t0 = sorted_ts[0] if sorted_ts else 0
                
                for t in sorted_ts:
                    row = [metrics.name, model_name, t, t - t0]
                    # Fill standard columns
                    for col in columns:
                        row.append(data_map.get(col, {}).get(t, ""))
                    # Fill input_latency
                    row.append(data_map.get("input_latency_ms", {}).get(t, ""))
                    writer_raw.writerow(row)

    print(f"Exported unified raw metrics: {raw_csv_path}")

    # 2. Export Unified Statistics Summary
    stats_csv_path = output_dir / "all_statistics.csv"
    with open(stats_csv_path, "w", newline="") as f_stats:
        writer_stats = csv.writer(f_stats)
        writer_stats.writerow(["dataset", "model", "metric", "count", "mean", "std", "min", "max", "median"])
        
        for metrics in metrics_list:
            for model_name, topics in [("PTv3", PTV3_TOPICS), ("CenterPoint", cp_topics)]:
                for key, topic in topics.items():
                    s = metrics.get_statistics(topic)
                    writer_stats.writerow([metrics.name, model_name, key, s["count"], s["mean"], s["std"], s["min"], s["max"], s["median"]])
                
                # Add input_latency stats
                _, vs_lat = metrics.get_input_latency(model_name, topics)
                if len(vs_lat) > 0:
                    writer_stats.writerow([
                        metrics.name, model_name, "input_latency_ms", len(vs_lat),
                        np.mean(vs_lat), np.std(vs_lat), np.min(vs_lat), np.max(vs_lat), np.median(vs_lat)
                    ])
    print(f"Exported unified statistics: {stats_csv_path}")

def plot_metrics(metrics_list: List[PerformanceMetrics], output_dir: Path, cp_prefix: str):
    """Output time-series and statistics graphs."""
    cp_topics = {k: v.replace(_CP_DEFAULT_PREFIX, cp_prefix) for k, v in CP_TOPICS.items()}

    output_dir.mkdir(parents=True, exist_ok=True)

    stage_topic_groups = {
        "PTv3":        PTV3_TOPICS,
        "CenterPoint": cp_topics,
    }

    # --- Stage time-series per model ---
    for model_name, topics in stage_topic_groups.items():
        stage_keys = ["preprocess_ms", "inference_ms", "postprocess_ms"]

        # Skip if no data for this model
        has_any = any(
            len(metrics.get_values(topics[k])[0]) > 0
            for metrics in metrics_list
            for k in stage_keys
        )
        if not has_any:
            continue

        axes = plt.subplots(len(stage_keys), 1, figsize=(14, 10), sharex=False)[1]
        for ax, key in zip(axes, stage_keys):
            topic = topics[key]
            has_data = False
            for metrics in metrics_list:
                ts, vs = metrics.get_values(topic)
                if len(ts) == 0:
                    continue
                ts = ts - ts[0]
                ax.plot(ts, vs, label=metrics.name, alpha=0.8, linewidth=1.2)
                has_data = True
            ax.set_ylabel(f"{key} [ms]", fontsize=10)
            if has_data:
                ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        axes[0].set_title(f"{model_name} - Stage Processing Times", fontsize=13, fontweight="bold")
        axes[-1].set_xlabel("Time [s]", fontsize=11)
        plt.tight_layout()
        safe = model_name.lower().replace(" ", "_")
        plt.savefig(output_dir / f"stage_timings_{safe}.png", dpi=150)
        plt.close()
        print(f"Saved: {output_dir}/stage_timings_{safe}.png")

    # --- All-metrics overlay time-series (per dataset) ---
    for metrics in metrics_list:
        all_plot_topics = list(PTV3_TOPICS.values())
        available = [(t, t.split("/")[-1]) for t in all_plot_topics if len(metrics.get_values(t)[0]) > 0]
        if not available:
            continue

        colors = plt.cm.tab10(np.linspace(0, 1, len(available)))
        plt.figure(figsize=(14, 8))
        
        # Collect all values to calculate a robust Y-limit
        all_values = []
        for (topic, label), color in zip(available, colors):
            ts, vs = metrics.get_values(topic)
            if len(vs) == 0: continue
            ts = ts - ts[0]
            
            # Use only values within 98th percentile for Y-limit calculation
            all_values.extend(vs)
            plt.plot(ts, vs, label=label, alpha=0.8, linewidth=1.5, color=color)

        if all_values:
            # Set Y-limit to 98th percentile * 1.2 to cut off initial spikes
            y_upper = np.percentile(all_values, 98) * 1.2
            plt.ylim(0, y_upper)
            plt.text(0.01, 0.98, f"Note: Y-axis clipped at 98th percentile ({y_upper:.1f}ms)", 
                     transform=plt.gca().transAxes, fontsize=9, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))

        plt.xlabel("Time [s]", fontsize=13)
        plt.ylabel("Value [ms]", fontsize=13)
        plt.title(f"All Performance Metrics (Outliers Clipped) - {metrics.name}", fontsize=15, fontweight="bold")
        plt.legend(loc="upper right", fontsize=10, framealpha=0.9)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        safe_name = metrics.name.replace("/", "_").replace(" ", "_")
        output_file = output_dir / f"all_metrics_overlay_{safe_name}.png"
        plt.savefig(output_file, dpi=150)
        plt.close()
        print(f"Saved: {output_file}")

    # --- Statistics summary (text) ---
    output_file = output_dir / "statistics_summary.txt"
    with open(output_file, "w") as f:
        f.write("=== Performance Metrics Statistics Summary ===\n\n")
        for metrics in metrics_list:
            f.write(f"Dataset: {metrics.name}\n")
            f.write("-" * 80 + "\n")
            for model_name, topics in stage_topic_groups.items():
                f.write(f"\n[{model_name}]\n")
                for key, topic in topics.items():
                    stats = metrics.get_statistics(topic)
                    f.write(f"  {key}: count={stats['count']}")
                    if stats["count"] > 0:
                        f.write(
                            f"  mean={stats['mean']:.2f}ms  std={stats['std']:.2f}ms"
                            f"  min={stats['min']:.2f}ms  max={stats['max']:.2f}ms"
                        )
                    f.write("\n")
            f.write("\n" + "=" * 80 + "\n\n")
    print(f"Saved: {output_file}")


# --------------------------------------------------------------------------- #
# Utilities
# --------------------------------------------------------------------------- #

def find_result_bag(directory: Path) -> Optional[Path]:
    result_bag_dir = directory / "result_bag"
    if not result_bag_dir.exists():
        return None
    mcap_files = list(result_bag_dir.glob("*.mcap"))
    return mcap_files[0] if mcap_files else None


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(
        description="Visualize performance metrics from a ROS bag"
    )
    parser.add_argument("input_dirs", nargs="+", type=str,
                        help="Input directory paths (one or more)")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output directory path")
    parser.add_argument("--names", nargs="+", type=str,
                        help="Names for each dataset")
    parser.add_argument("--cp-prefix", type=str, default=_CP_DEFAULT_PREFIX,
                        help=f"CenterPoint topic prefix (default: {_CP_DEFAULT_PREFIX})")
    parser.add_argument("--gantt-window", type=float, default=5.0,
                        help="Time window to display in Gantt chart [s] (default: 5.0)")
    parser.add_argument("--gantt-start", type=float, default=None,
                        help="Start offset for Gantt chart [s] (default: near center)")
    parser.add_argument("--contention-bin", type=float, default=1.0,
                        help="Bin width for contention rate aggregation [s] (default: 1.0)")
    args = parser.parse_args()

    input_paths = [Path(d) for d in args.input_dirs]
    for path in input_paths:
        if not path.exists():
            print(f"Error: directory does not exist: {path}")
            sys.exit(1)

    if args.output:
        output_dir = Path(args.output)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path("comparison_results") / timestamp

    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_dir}\n")

    dataset_names = args.names if args.names else [p.name for p in input_paths]
    if args.names and len(args.names) != len(input_paths):
        print("Error: number of --names entries does not match number of input directories")
        sys.exit(1)

    # apply prefix to CenterPoint topics
    cp_topics = {k: v.replace(_CP_DEFAULT_PREFIX, args.cp_prefix) for k, v in CP_TOPICS.items()}

    metrics_list = []
    for path, name in zip(input_paths, dataset_names):
        print(f"Processing: {path} (name: {name})")
        bag_file = find_result_bag(path)
        if bag_file is None:
            print(f"  Warning: result_bag not found: {path}")
            continue
        metrics = read_rosbag_metrics(bag_file, name, args.cp_prefix)
        if metrics is not None:
            metrics_list.append(metrics)

    if not metrics_list:
        print("\nError: no valid data could be loaded")
        sys.exit(1)

    print(f"\nLoaded datasets: {len(metrics_list)}")
    print("\nRunning visualization...")

    # existing time-series and statistics graphs
    plot_metrics(metrics_list, output_dir, args.cp_prefix)

    # export metrics to CSV
    export_metrics_to_csv(metrics_list, output_dir, args.cp_prefix)

    # generate Gantt + contention timeline per dataset
    for metrics in metrics_list:
        cp_windows   = metrics.reconstruct_windows(cp_topics)
        ptv3_windows = metrics.reconstruct_windows(PTV3_TOPICS)

        print(f"\n[{metrics.name}] CenterPoint: {len(cp_windows)} frames, PTv3: {len(ptv3_windows)} frames")

        if cp_windows and ptv3_windows:
            overlaps = detect_inference_overlaps(cp_windows, ptv3_windows)
            total_sec = sum(e - s for s, e in overlaps)
            print(f"  GPU contention intervals: {len(overlaps)}, total {total_sec:.3f}s")

            plot_gantt_chart(
                cp_windows, ptv3_windows, output_dir,
                window_sec=args.gantt_window,
                start_offset_sec=args.gantt_start,
                dataset_name=metrics.name,
            )
            plot_gpu_contention_timeline(
                cp_windows, ptv3_windows, output_dir,
                dataset_name=metrics.name,
                bin_sec=args.contention_bin,
            )
            # Add contention impact analysis
            plot_contention_impact(metrics, cp_windows, ptv3_windows, output_dir)
        else:
            if not cp_windows:
                print(f"  Warning: no CenterPoint stage timing data available")
            if not ptv3_windows:
                print(f"  Warning: no PTv3 stage timing data available")

        # Plot Input Latency Breakdown
        plt.figure(figsize=(14, 6))
        for model_name, topics in [("PTv3", PTV3_TOPICS), ("CenterPoint", cp_topics)]:
            ts, input_lat = metrics.get_input_latency(model_name, topics)
            if len(ts) > 0:
                plt.plot(ts - ts[0], input_lat, label=f"{model_name} Input Latency")
        plt.title(f"Input Latency Breakdown - {metrics.name}")
        plt.xlabel("Time [s]")
        plt.ylabel("ms")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(output_dir / f"input_latency_{metrics.name.replace('/', '_')}.png", dpi=150)
        plt.close()

    print(f"\nDone! Output directory: {output_dir}")


if __name__ == "__main__":
    main()
