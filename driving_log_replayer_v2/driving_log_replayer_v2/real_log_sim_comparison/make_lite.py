#!/usr/bin/env python3
"""rosbag (db3 または mcap) から三方比較に必要なトピックだけを抽出して lite/*.mcap を生成する.

Usage:
    python3 make_lite.py --kind real --input <bag_dir>  --output lite/real.lite.mcap
    python3 make_lite.py --kind sim  --input <bag_dir>  --output lite/sim_godot.lite.mcap
    python3 make_lite.py --kind sim  --input <bag_dir>  --output lite/sim_normal.lite.mcap

--input にはロスバッグのディレクトリ（db3 / mcap どちらでも可）を渡す。
単一の .mcap ファイルパスを渡した場合は後方互換のためそのまま処理する。
"""

import argparse
from pathlib import Path

from mcap.writer import Writer

TOPICS: dict[str, set[str]] = {
    "real": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/sub/localization/kinematic_state",
        "/sub/localization/acceleration",
        "/sub/control/command/control_cmd",
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
        # post-gate制御指令（実機 /sub/control/command/control_cmd と同一段での比較用）
        "/control/command/control_cmd",
        # DiffusionPlanner出力軌跡（速度プロファイル分析用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 交通信号状態（DiffusionPlannerへの入力トピック）
        "/perception/traffic_light_recognition/traffic_signals",
    },
}


def filter_bag(input_dir: Path, output_path: Path, topics: set[str]) -> None:
    """rosbag ディレクトリ（db3 / mcap）からトピックを絞り込んで単一 MCAP ファイルに書き出す.

    rosbag2_py.SequentialReader で読み込み（storage_id="" で形式を自動検出）、
    mcap.writer.Writer で単一ファイル出力する。
    """
    import rosbag2_py

    # metadata.yaml がない場合は Reindexer で生成する
    if not (input_dir / "metadata.yaml").exists():
        rosbag2_py.Reindexer().reindex(
            rosbag2_py.StorageOptions(uri=str(input_dir), storage_id="")
        )

    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(input_dir), storage_id=""), converter)

    # 対象トピックの型名マップ（schema 登録に使用）
    topic_type_map: dict[str, str] = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
        if t.name in topics
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as fout:
        writer = Writer(fout)
        writer.start(profile="ros2", library="make_lite")

        schema_ids: dict[str, int] = {}   # type_name → schema_id
        channel_ids: dict[str, int] = {}  # topic_name → channel_id

        while reader.has_next():
            topic_name, msg_bytes, timestamp = reader.read_next()
            if topic_name not in topic_type_map:
                continue

            type_name = topic_type_map[topic_name]
            if type_name not in schema_ids:
                # encoding="ros2msg" + name=型名 で mcap_ros2.DecoderFactory が
                # ローカルの rosidl から型定義を解決してデコードできる
                schema_ids[type_name] = writer.register_schema(
                    name=type_name,
                    encoding="ros2msg",
                    data=b"",
                )
            if topic_name not in channel_ids:
                channel_ids[topic_name] = writer.register_channel(
                    topic=topic_name,
                    message_encoding="cdr",
                    schema_id=schema_ids[type_name],
                    metadata={},
                )
            writer.add_message(
                channel_id=channel_ids[topic_name],
                log_time=timestamp,
                data=msg_bytes,
                publish_time=timestamp,
                sequence=0,
            )
        writer.finish()


def filter_mcap(input_path: Path, output_path: Path, topics: set[str]) -> None:
    """単一 MCAP ファイルからトピックを絞り込んで別の単一 MCAP ファイルに書き出す（後方互換）."""
    from mcap.reader import make_reader

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(input_path, "rb") as fin, open(output_path, "wb") as fout:
        reader = make_reader(fin)
        writer = Writer(fout)
        writer.start(profile="", library="make_lite")

        schema_ids: dict[int, int] = {}   # 旧 schema_id → 新 schema_id
        channel_ids: dict[int, int] = {}  # 旧 channel_id → 新 channel_id

        for schema, channel, message in reader.iter_messages(topics=list(topics)):
            if schema is not None and schema.id not in schema_ids:
                schema_ids[schema.id] = writer.register_schema(
                    name=schema.name,
                    encoding=schema.encoding,
                    data=schema.data,
                )

            if channel.id not in channel_ids:
                new_schema_id = schema_ids.get(schema.id, 0) if schema else 0
                channel_ids[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=new_schema_id,
                    metadata=channel.metadata,
                )

            writer.add_message(
                channel_id=channel_ids[channel.id],
                log_time=message.log_time,
                data=message.data,
                publish_time=message.publish_time,
                sequence=message.sequence,
            )

        writer.finish()


def main() -> None:
    parser = argparse.ArgumentParser(description="rosbag トピックフィルタ — lite mcap を生成")
    parser.add_argument("--kind", choices=["real", "sim"], required=True,
                        help="ログの種別 (real=実機, sim=シミュレータ)")
    parser.add_argument("--input", required=True, type=Path,
                        help="入力ロスバッグのディレクトリ、または単一 .mcap ファイルパス")
    parser.add_argument("--output", required=True, type=Path,
                        help="出力 lite mcap ファイルパス")
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
            import yaml as _yaml
            with open(args.topics_yaml, encoding="utf-8") as f:
                extra = _yaml.safe_load(f) or {}
            additional = extra.get(args.kind, [])
            if additional:
                topics |= set(additional)
                print(f"追加トピック ({args.kind}): {sorted(additional)}")
        except Exception as e:
            import sys
            print(f"WARNING: topics-yaml 読み込み失敗: {e}", file=sys.stderr)

    print(f"種別  : {args.kind}")
    print(f"入力  : {args.input} ({args.input.stat().st_size / 1024 / 1024:.0f} MB)")
    print(f"トピック: {sorted(topics)}")

    if args.input.is_dir():
        filter_bag(args.input, args.output, topics)
    else:
        # 後方互換: 単一 .mcap ファイルが渡された場合
        filter_mcap(args.input, args.output, topics)

    size_mb = args.output.stat().st_size / 1024 / 1024
    print(f"  書き込み完了: {args.output} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
