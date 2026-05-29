#!/usr/bin/env python3
"""
rosbag (db3 または mcap) から三方比較に必要なトピックだけを抽出して lite bag を生成する.

Usage:
    python3 step1_make_lite.py --kind real --input <bag_dir>  --output lite/real.lite
    python3 step1_make_lite.py --kind sim  --input <bag_dir>  --output lite/sim_godot.lite
    python3 step1_make_lite.py --kind sim  --input <bag_dir>  --output lite/sim_normal.lite

--input にはロスバッグのディレクトリ（db3 / mcap どちらでも可）または単一 .mcap ファイルを渡す。
--output は rosbag2 bag ディレクトリとして出力される。
"""

import argparse
from pathlib import Path
import shutil
import sys

import rosbag2_py
import yaml

from driving_log_replayer_v2.rosbag import create_metadata_yaml

TOPICS: dict[str, set[str]] = {
    "real": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/control/command/control_cmd",
        # DiffusionPlanner出力軌跡（シムとの直接比較用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 追跡物体（社会的コンテキスト有無の確認用）
        "/perception/object_recognition/tracking/objects",
        # 最終プランニング軌跡（optimizer後段の出力を確認）
        "/planning/trajectory",
    },
    "sim": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/control/trajectory_follower/control_cmd",
        # post-gate制御指令（実機 /control/command/control_cmd と同一段での比較用）
        "/control/command/control_cmd",
        # DiffusionPlanner出力軌跡（速度プロファイル分析用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 交通信号状態（DiffusionPlannerへの入力トピック）
        "/perception/traffic_light_recognition/traffic_signals",
    },
}


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


def filter_bag(input_path: Path, output_dir: Path, topics: set[str]) -> None:
    """rosbag（ディレクトリまたは単一 .mcap）からトピックを絞り込んで rosbag2 bag ディレクトリに書き出す."""
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

    del writer


def main() -> None:
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
    args = parser.parse_args()

    if not args.input.exists():
        parser.error(f"入力が見つかりません: {args.input}")

    topics = set(TOPICS[args.kind])

    if args.topics_yaml is not None:
        try:
            with open(args.topics_yaml, encoding="utf-8") as f:
                extra = yaml.safe_load(f) or {}
            additional = extra.get(args.kind, [])
            if additional:
                topics |= set(additional)
                print(f"追加トピック ({args.kind}): {sorted(additional)}")
        except Exception as e:
            print(f"WARNING: topics-yaml 読み込み失敗: {e}", file=sys.stderr)

    in_size = (
        sum(f.stat().st_size for f in args.input.rglob("*") if f.is_file())
        if args.input.is_dir()
        else args.input.stat().st_size
    )
    print(f"種別  : {args.kind}")
    print(f"入力  : {args.input} ({in_size / 1024 / 1024:.0f} MB)")
    print(f"トピック: {sorted(topics)}")

    filter_bag(args.input, args.output, topics)

    total = sum(f.stat().st_size for f in args.output.rglob("*") if f.is_file())
    print(f"  書き込み完了: {args.output} ({total / 1024 / 1024:.1f} MB)")


if __name__ == "__main__":
    main()
