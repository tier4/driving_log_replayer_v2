#!/usr/bin/env python3
"""rosbag (db3 または mcap) から必要トピックだけを抽出して lite bag を作る.

Usage:
    python3 step0_make_lite.py --kind real --input <bag_dir> --output lite/real.lite
    python3 step0_make_lite.py --kind sim  --input <bag_dir> --output lite/sim_godot.lite
    python3 step0_make_lite.py --kind sim  --input <bag_dir> --output lite/sim_normal.lite

--input にはロスバッグのディレクトリ（db3 / mcap どちらでも可）または単一 .mcap ファイルを渡す。
--output は rosbag2 bag ディレクトリとして出力される。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shutil
import sys
from typing import Any

import rosbag2_py
import yaml

from driving_log_replayer_v2.rosbag import create_metadata_yaml
from driving_log_replayer_v2.real_log_sim_comparison.lib._validation import (
    MissingRequiredDataError,
    RequiredTopic,
    REQUIRED_REAL_LITE_TOPICS,
    REQUIRED_SIM_LITE_TOPICS,
    validate_required_topics,
)

TOPICS_BY_KIND: dict[str, set[str]] = {
    "real": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/vehicle/status/gear_status",
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/control/command/control_cmd",
        "/control/command/gear_cmd",
        # DiffusionPlanner出力軌跡（シムとの直接比較用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 追跡物体（社会的コンテキスト有無の確認用）
        "/perception/object_recognition/tracking/objects",
        "/perception/object_recognition/detection/objects",
        "/perception/traffic_light_recognition/traffic_signals",
        # 最終プランニング軌跡（optimizer後段の出力を確認）
        "/planning/trajectory",
    },
    "sim": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/vehicle/status/gear_status",
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/control/trajectory_follower/control_cmd",
        # post-gate制御指令（実機 /control/command/control_cmd と同一段での比較用）
        "/control/command/control_cmd",
        "/control/command/gear_cmd",
        # DiffusionPlanner出力軌跡（速度プロファイル分析用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 交通信号状態（DiffusionPlannerへの入力トピック）
        "/perception/traffic_light_recognition/traffic_signals",
    },
}

REQUIRED_TOPICS_BY_KIND: dict[str, list[RequiredTopic]] = {
    "real": list(REQUIRED_REAL_LITE_TOPICS),
    "sim": list(REQUIRED_SIM_LITE_TOPICS),
}


@dataclass(slots=True)
class BagStats:
    duration_s: float
    n_written: int
    per_topic: dict[str, int]
    missing_topics: list[str]


def _open_reader(input_path: Path) -> rosbag2_py.SequentialReader:
    if input_path.is_dir():
        create_metadata_yaml(str(input_path))
        storage_id = ""
    else:
        storage_id = "mcap"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def _bag_size_bytes(input_path: Path) -> int:
    if input_path.is_dir():
        return sum(f.stat().st_size for f in input_path.rglob("*") if f.is_file())
    return input_path.stat().st_size


def _load_extra_topics(topics_yaml: Path | None, kind: str) -> set[str]:
    if topics_yaml is None:
        return set()
    with topics_yaml.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("topics-yaml は real/sim をキーに持つ mapping である必要があります")
    topics = data.get(kind) or []
    if topics is None:
        return set()
    if not isinstance(topics, list):
        raise ValueError(f"topics-yaml の {kind} は list である必要があります")
    return {str(topic) for topic in topics}


def filter_bag(input_path: Path, output_dir: Path, topics: set[str]) -> BagStats:
    """rosbag（ディレクトリまたは単一 .mcap）からトピックを絞り込んで rosbag2 bag ディレクトリに書き出す.

    戻り値: 診断用統計 BagStats(duration_s, n_written, per_topic, missing_topics)。
    本ツールは AUTONOMOUS による時間カットをしない (全区間コピー)。AUTONOMOUS 窓の
    切り出しは後段 (step4/step5 の find_autonomous_start) が担う。
    """
    reader = _open_reader(input_path)
    topic_type_map: dict[str, str] = {
        t.name: t.type for t in reader.get_all_topics_and_types() if t.name in topics
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.parent.mkdir(parents=True, exist_ok=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    registered: set[str] = set()
    per_topic: dict[str, int] = {}
    t_min: int | None = None
    t_max: int | None = None
    while reader.has_next():
        topic_name, msg_bytes, timestamp = reader.read_next()
        if topic_name not in topic_type_map:
            continue
        if topic_name not in registered:
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic_name,
                    type=topic_type_map[topic_name],
                    serialization_format="cdr",
                )
            )
            registered.add(topic_name)
        writer.write(topic_name, msg_bytes, timestamp)
        per_topic[topic_name] = per_topic.get(topic_name, 0) + 1
        if t_min is None or timestamp < t_min:
            t_min = timestamp
        if t_max is None or timestamp > t_max:
            t_max = timestamp

    del writer
    duration_s = (t_max - t_min) / 1e9 if (t_min is not None and t_max is not None) else 0.0
    return BagStats(
        duration_s=duration_s,
        n_written=sum(per_topic.values()),
        per_topic=per_topic,
        missing_topics=sorted(topics - set(per_topic.keys())),
    )


def _configure_warnings(verbose: bool) -> None:
    if not verbose:
        import warnings

        warnings.simplefilter("ignore")


def _print_verbose(verbose: bool, *args: Any, **kwargs: Any) -> None:
    if verbose:
        print(*args, **kwargs)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="rosbag トピックフィルタ — lite bag を生成")
    parser.add_argument(
        "--kind",
        choices=["real", "sim"],
        required=True,
        help="ログの種別 (real=実機, sim=シミュレータ)",
    )
    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="入力ロスバッグのディレクトリ、または単一 .mcap ファイルパス",
    )
    parser.add_argument("--output", required=True, type=Path, help="出力 lite bag ディレクトリパス")
    parser.add_argument(
        "--topics-yaml",
        type=Path,
        default=None,
        help=(
            "追加トピックリストを含む YAML ファイルのパス（省略時は既定トピックのみ）。"
            "形式: real: [topic, ...] および/または sim: [topic, ...]"
        ),
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="詳細情報を出力する",
    )
    args = parser.parse_args(argv)

    _configure_warnings(args.verbose)

    if not args.input.exists():
        parser.error(f"入力が見つかりません: {args.input}")

    try:
        topics = set(TOPICS_BY_KIND[args.kind]) | _load_extra_topics(args.topics_yaml, args.kind)
    except Exception as exc:
        print(f"WARNING: topics-yaml 読み込み失敗: {exc}", file=sys.stderr)
        topics = set(TOPICS_BY_KIND[args.kind])
    if args.topics_yaml is not None and topics != TOPICS_BY_KIND[args.kind]:
        _print_verbose(args.verbose, f"追加トピック ({args.kind}): {sorted(topics - TOPICS_BY_KIND[args.kind])}")

    _print_verbose(args.verbose, f"種別  : {args.kind}")
    _print_verbose(args.verbose, f"入力  : {args.input} ({_bag_size_bytes(args.input) / 1024 / 1024:.0f} MB)")
    _print_verbose(args.verbose, f"トピック: {sorted(topics)}")

    stats = filter_bag(args.input, args.output, topics)

    total = _bag_size_bytes(args.output)
    _print_verbose(args.verbose, f"  書き込み完了: {args.output} ({total / 1024 / 1024:.1f} MB)")
    # 診断出力: lite bag の時間範囲・件数・欠落トピックを明示し、短すぎる/空の bag を可視化。
    _print_verbose(args.verbose, f"  lite bag 全長: {stats.duration_s:.1f} s, 総メッセージ: {stats.n_written}")
    if args.verbose:
        for topic in sorted(stats.per_topic):
            print(f"    {topic}: {stats.per_topic[topic]}")
    if stats.missing_topics:
        _print_verbose(args.verbose, f"  WARNING: 入力 bag に存在しなかったトピック: {stats.missing_topics}", file=sys.stderr)
    try:
        validate_required_topics(
            set(stats.per_topic),
            REQUIRED_TOPICS_BY_KIND[args.kind],
            context=f"step0_make_lite --kind {args.kind}",
            regeneration_hint="必須トピックを含む input_bag から lite bag を再生成してください。",
        )
    except MissingRequiredDataError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    if stats.n_written == 0:
        _print_verbose(args.verbose, "  WARNING: 出力 lite bag が空です (トピック名不一致または入力 bag が空の可能性)", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
