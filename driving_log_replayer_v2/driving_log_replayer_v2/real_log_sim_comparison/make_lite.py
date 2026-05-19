#!/usr/bin/env python3
"""rosbag (db3 または mcap) から三方比較に必要なトピックだけを抽出して lite bag を生成する.

Usage:
    python3 make_lite.py --kind real --input <bag_dir>  --output lite/real.lite
    python3 make_lite.py --kind sim  --input <bag_dir>  --output lite/sim_godot.lite
    python3 make_lite.py --kind sim  --input <bag_dir>  --output lite/sim_normal.lite

--input にはロスバッグのディレクトリ（db3 / mcap どちらでも可）を渡す。
単一の .mcap ファイルパスを渡した場合は後方互換のためそのまま処理する。
--output は rosbag2 bag ディレクトリとして出力される。
"""

import argparse
from pathlib import Path

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


def _open_reader(input_path: Path):
    """入力パス（bag ディレクトリ or 単一 .mcap）から SequentialReader を返す。

    単一 .mcap の場合は一時ディレクトリにシンボリックリンクを張って Reindex してから開く。
    呼び出し元は返り値の (reader, cleanup) を使い終わったら cleanup() を呼ぶこと。
    """
    import tempfile

    import rosbag2_py

    if input_path.is_dir():
        if not (input_path / "metadata.yaml").exists():
            rosbag2_py.Reindexer().reindex(
                rosbag2_py.StorageOptions(uri=str(input_path), storage_id="")
            )
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(input_path), storage_id=""),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        return reader, lambda: None
    else:
        # 単一 .mcap ファイル: 一時ディレクトリにシンボリックリンクを作成して Reindex
        tmp = tempfile.mkdtemp()
        link = Path(tmp) / input_path.name
        link.symlink_to(input_path.resolve())
        rosbag2_py.Reindexer().reindex(
            rosbag2_py.StorageOptions(uri=tmp, storage_id="mcap")
        )
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=tmp, storage_id=""),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        import shutil
        return reader, lambda: shutil.rmtree(tmp, ignore_errors=True)


def _write_filtered_bag(reader, topic_type_map: dict, output_dir: Path) -> None:
    """reader から topic_type_map のトピックを絞り込んで output_dir に bag を書き出す。"""
    import shutil

    import rosbag2_py

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
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic_name,
                type=topic_type_map[topic_name],
                serialization_format="cdr",
            ))
            registered.add(topic_name)
        writer.write(topic_name, msg_bytes, timestamp)

    del writer  # flush & close


def filter_bag(input_dir: Path, output_dir: Path, topics: set[str]) -> None:
    """rosbag ディレクトリ（db3 / mcap）からトピックを絞り込んで rosbag2 bag ディレクトリに書き出す."""
    import rosbag2_py

    reader, cleanup = _open_reader(input_dir)
    try:
        topic_type_map: dict[str, str] = {
            t.name: t.type
            for t in reader.get_all_topics_and_types()
            if t.name in topics
        }
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        _write_filtered_bag(reader, topic_type_map, output_dir)
    finally:
        cleanup()


def filter_mcap(input_path: Path, output_dir: Path, topics: set[str]) -> None:
    """単一 MCAP ファイルからトピックを絞り込んで rosbag2 bag ディレクトリに書き出す（後方互換）."""
    import rosbag2_py

    reader, cleanup = _open_reader(input_path)
    try:
        topic_type_map: dict[str, str] = {
            t.name: t.type
            for t in reader.get_all_topics_and_types()
            if t.name in topics
        }
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        _write_filtered_bag(reader, topic_type_map, output_dir)
    finally:
        cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description="rosbag トピックフィルタ — lite bag を生成")
    parser.add_argument("--kind", choices=["real", "sim"], required=True,
                        help="ログの種別 (real=実機, sim=シミュレータ)")
    parser.add_argument("--input", required=True, type=Path,
                        help="入力ロスバッグのディレクトリ、または単一 .mcap ファイルパス")
    parser.add_argument("--output", required=True, type=Path,
                        help="出力 lite bag ディレクトリパス")
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

    in_size = sum(f.stat().st_size for f in args.input.rglob("*") if f.is_file()) if args.input.is_dir() else args.input.stat().st_size
    print(f"種別  : {args.kind}")
    print(f"入力  : {args.input} ({in_size / 1024 / 1024:.0f} MB)")
    print(f"トピック: {sorted(topics)}")

    if args.input.is_dir():
        filter_bag(args.input, args.output, topics)
    else:
        filter_mcap(args.input, args.output, topics)

    total = sum(f.stat().st_size for f in args.output.rglob("*") if f.is_file())
    print(f"  書き込み完了: {args.output} ({total / 1024 / 1024:.1f} MB)")


if __name__ == "__main__":
    main()
