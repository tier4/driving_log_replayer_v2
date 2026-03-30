#!/usr/bin/env python3
"""
パフォーマンスメトリクスの可視化スクリプト

ROSバッグからPTV3のパフォーマンスメトリクスを抽出し、時系列グラフとして可視化します。
複数のデータセットを比較することもできます。
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np

# ROSバッグ読み込み用のライブラリをインポート
try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
except ImportError:
    print("エラー: rosbags ライブラリがインストールされていません")
    print("インストールコマンド: pip3 install rosbags")
    sys.exit(1)


# 可視化対象のトピック
TARGET_TOPICS = [
    "/perception/obstacle_segmentation/ptv3/debug/pipeline_latency_ms",
    "/perception/obstacle_segmentation/ptv3/debug/processing_time/inference_ms",
    "/perception/obstacle_segmentation/ptv3/debug/processing_time/postprocess_ms",
    "/perception/obstacle_segmentation/ptv3/debug/processing_time/preprocess_ms",
    "/perception/obstacle_segmentation/ptv3/debug/processing_time/total_ms",
]


class PerformanceMetrics:
    """パフォーマンスメトリクスを格納するクラス"""

    def __init__(self, name: str):
        self.name = name
        self.data: Dict[str, List[tuple]] = {topic: [] for topic in TARGET_TOPICS}

    def add_data(self, topic: str, timestamp: float, value: float):
        """データポイントを追加"""
        if topic in self.data:
            self.data[topic].append((timestamp, value))

    def get_statistics(self, topic: str) -> Dict[str, float]:
        """統計情報を取得"""
        if topic not in self.data or len(self.data[topic]) == 0:
            return {
                "count": 0,
                "mean": 0,
                "std": 0,
                "min": 0,
                "max": 0,
                "median": 0,
            }

        values = [v for _, v in self.data[topic]]
        return {
            "count": len(values),
            "mean": np.mean(values),
            "std": np.std(values),
            "min": np.min(values),
            "max": np.max(values),
            "median": np.median(values),
        }


def read_rosbag_metrics(bag_path: Path, name: str) -> Optional[PerformanceMetrics]:
    """ROSバッグからメトリクスを読み取る"""

    if not bag_path.exists():
        print(f"警告: バッグファイルが見つかりません: {bag_path}")
        return None

    metrics = PerformanceMetrics(name)

    try:
        # タイプストアを作成
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        # カスタムメッセージタイプを登録
        # autoware_internal_debug_msgs/msg/Float64Stamped
        float64_stamped_def = """
builtin_interfaces/Time stamp
float64 data
"""
        types = get_types_from_msg(
            float64_stamped_def,
            'autoware_internal_debug_msgs/msg/Float64Stamped'
        )
        typestore.register(types)

        # metadata.yaml の有無を確認
        bag_dir = bag_path.parent
        metadata_path = bag_dir / "metadata.yaml"

        if not metadata_path.exists():
            # metadata.yaml がない場合は、mcapファイルを直接読む
            print(f"  注: metadata.yaml が見つかりません。mcapファイルを直接読み込みます")
            # rosbags 0.11.0 では直接 mcap ファイルをサポートしていない可能性があるため、
            # この場合はスキップします
            print(f"  警告: metadata.yaml が必要です: {bag_dir}")
            return None

        with Reader(bag_dir) as reader:
            # トピックの接続情報を取得
            connections = [x for x in reader.connections if x.topic in TARGET_TOPICS]

            if not connections:
                print(f"警告: 対象トピックが見つかりません: {bag_path}")
                return None

            # メッセージを読み取る
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                try:
                    # 新しいAPIを使用してデシリアライズ
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    # Float64Stampedメッセージからデータを取得
                    value = msg.data
                    # タイムスタンプを秒単位に変換
                    time_sec = timestamp / 1e9
                    metrics.add_data(connection.topic, time_sec, value)
                except Exception as e:
                    print(f"メッセージのデシリアライズに失敗: {e}")
                    continue

            print(f"読み込み完了: {name}")
            for topic in TARGET_TOPICS:
                count = len(metrics.data[topic])
                if count > 0:
                    print(f"  {topic}: {count} メッセージ")

            return metrics

    except Exception as e:
        print(f"エラー: バッグファイルの読み込みに失敗: {bag_path}")
        print(f"  詳細: {e}")
        return None


def plot_metrics(metrics_list: List[PerformanceMetrics], output_dir: Path):
    """メトリクスを可視化"""

    output_dir.mkdir(parents=True, exist_ok=True)

    # 全メトリクスを重ねた統合時系列グラフ（データセット毎）
    for metrics in metrics_list:
        plt.figure(figsize=(14, 8))

        # 色のパレットを定義
        colors = plt.cm.tab10(np.linspace(0, 1, len(TARGET_TOPICS)))

        has_data = False
        for idx, topic in enumerate(TARGET_TOPICS):
            if len(metrics.data[topic]) == 0:
                continue

            has_data = True
            timestamps, values = zip(*metrics.data[topic])
            # 最初のタイムスタンプを0とする
            timestamps = np.array(timestamps) - timestamps[0]
            topic_name = topic.split("/")[-1]
            plt.plot(timestamps, values, label=topic_name, alpha=0.8, linewidth=1.5, color=colors[idx])

        if has_data:
            plt.xlabel("Time [s]", fontsize=13)
            plt.ylabel("Value [ms]", fontsize=13)
            plt.title(f"All Performance Metrics - {metrics.name}", fontsize=15, fontweight='bold')
            plt.legend(loc='best', fontsize=10, framealpha=0.9)
            plt.grid(True, alpha=0.3)
            plt.tight_layout()

            # ファイル名をサニタイズ
            safe_name = metrics.name.replace("/", "_").replace(" ", "_")
            output_file = output_dir / f"all_metrics_overlay_{safe_name}.png"
            plt.savefig(output_file, dpi=150)
            plt.close()
            print(f"保存: {output_file}")
        else:
            plt.close()

    # 複数データセット比較用: 全メトリクスを重ねたグラフ（全データセット比較）
    if len(metrics_list) > 1:
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        axes = axes.flatten()

        for idx, topic in enumerate(TARGET_TOPICS):
            ax = axes[idx]
            topic_name = topic.split("/")[-1]

            for metrics in metrics_list:
                if len(metrics.data[topic]) == 0:
                    continue

                timestamps, values = zip(*metrics.data[topic])
                timestamps = np.array(timestamps) - timestamps[0]
                ax.plot(timestamps, values, label=metrics.name, alpha=0.7, linewidth=1.5)

            ax.set_xlabel("Time [s]", fontsize=10)
            ax.set_ylabel("Value [ms]", fontsize=10)
            ax.set_title(topic_name, fontsize=11, fontweight='bold')
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        # 最後のサブプロット（6番目）を非表示にする
        axes[5].set_visible(False)

        plt.tight_layout()
        output_file = output_dir / "all_datasets_comparison_timeseries.png"
        plt.savefig(output_file, dpi=150)
        plt.close()
        print(f"保存: {output_file}")

    # トピックごとにグラフを作成
    for topic in TARGET_TOPICS:
        topic_name = topic.split("/")[-1]

        # 時系列グラフ
        plt.figure(figsize=(12, 6))

        for metrics in metrics_list:
            if len(metrics.data[topic]) == 0:
                continue

            timestamps, values = zip(*metrics.data[topic])
            # 最初のタイムスタンプを0とする
            timestamps = np.array(timestamps) - timestamps[0]
            plt.plot(timestamps, values, label=metrics.name, alpha=0.7, linewidth=1)

        plt.xlabel("Time [s]", fontsize=12)
        plt.ylabel("Value [ms]", fontsize=12)
        plt.title(f"{topic_name}", fontsize=14)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        output_file = output_dir / f"{topic_name}_timeseries.png"
        plt.savefig(output_file, dpi=150)
        plt.close()
        print(f"保存: {output_file}")

    # 統計サマリーのグラフ（平均値と標準偏差）
    # 5つのメトリクスなので、2行3列のレイアウトで最後の1つは空白
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()

    for idx, topic in enumerate(TARGET_TOPICS):
        ax = axes[idx]
        topic_name = topic.split("/")[-1]

        names = []
        means = []
        stds = []

        for metrics in metrics_list:
            stats = metrics.get_statistics(topic)
            if stats["count"] > 0:
                names.append(metrics.name)
                means.append(stats["mean"])
                stds.append(stats["std"])

        if names:
            x_pos = np.arange(len(names))
            ax.bar(x_pos, means, yerr=stds, capsize=5, alpha=0.7)
            ax.set_xticks(x_pos)
            ax.set_xticklabels(names, rotation=45, ha="right", fontsize=9)
            ax.set_ylabel("Value [ms]", fontsize=10)
            ax.set_title(topic_name, fontsize=11)
            ax.grid(True, alpha=0.3, axis='y')

    # 最後のサブプロット（6番目）を非表示にする
    axes[5].set_visible(False)

    plt.tight_layout()
    output_file = output_dir / "statistics_summary.png"
    plt.savefig(output_file, dpi=150)
    plt.close()
    print(f"保存: {output_file}")

    # ボックスプロット
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()

    for idx, topic in enumerate(TARGET_TOPICS):
        ax = axes[idx]
        topic_name = topic.split("/")[-1]

        data_list = []
        labels = []

        for metrics in metrics_list:
            if len(metrics.data[topic]) > 0:
                values = [v for _, v in metrics.data[topic]]
                data_list.append(values)
                labels.append(metrics.name)

        if data_list:
            bp = ax.boxplot(data_list, labels=labels, patch_artist=True)
            for patch in bp['boxes']:
                patch.set_facecolor('lightblue')
                patch.set_alpha(0.7)
            ax.set_ylabel("Value [ms]", fontsize=10)
            ax.set_title(topic_name, fontsize=11)
            ax.tick_params(axis='x', rotation=45, labelsize=9)
            ax.grid(True, alpha=0.3, axis='y')

    # 最後のサブプロット（6番目）を非表示にする
    axes[5].set_visible(False)

    plt.tight_layout()
    output_file = output_dir / "boxplot_comparison.png"
    plt.savefig(output_file, dpi=150)
    plt.close()
    print(f"保存: {output_file}")

    # 統計情報をテキストファイルに出力
    output_file = output_dir / "statistics_summary.txt"
    with open(output_file, "w") as f:
        f.write("=== パフォーマンスメトリクス統計サマリー ===\n\n")

        for metrics in metrics_list:
            f.write(f"データセット: {metrics.name}\n")
            f.write("-" * 80 + "\n")

            for topic in TARGET_TOPICS:
                stats = metrics.get_statistics(topic)
                topic_name = topic.split("/")[-1]
                f.write(f"\n{topic_name}:\n")
                f.write(f"  データ数: {stats['count']}\n")
                if stats['count'] > 0:
                    f.write(f"  平均値: {stats['mean']:.3f} ms\n")
                    f.write(f"  標準偏差: {stats['std']:.3f} ms\n")
                    f.write(f"  最小値: {stats['min']:.3f} ms\n")
                    f.write(f"  最大値: {stats['max']:.3f} ms\n")
                    f.write(f"  中央値: {stats['median']:.3f} ms\n")

            f.write("\n" + "=" * 80 + "\n\n")

    print(f"保存: {output_file}")


def find_result_bag(directory: Path) -> Optional[Path]:
    """result_bag ディレクトリ内の .mcap ファイルを探す"""
    result_bag_dir = directory / "result_bag"

    if not result_bag_dir.exists():
        return None

    mcap_files = list(result_bag_dir.glob("*.mcap"))
    if mcap_files:
        return mcap_files[0]

    return None


def main():
    parser = argparse.ArgumentParser(
        description="ROSバッグからパフォーマンスメトリクスを可視化します"
    )
    parser.add_argument(
        "input_dirs",
        nargs="+",
        type=str,
        help="入力ディレクトリのパス（複数指定可能）",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="出力ディレクトリのパス（デフォルト: comparison_results_YYYYMMDD_HHMMSS）",
    )
    parser.add_argument(
        "--names",
        nargs="+",
        type=str,
        help="各データセットの名前（指定しない場合はディレクトリ名を使用）",
    )

    args = parser.parse_args()

    # 入力ディレクトリの検証
    input_paths = [Path(d) for d in args.input_dirs]

    for path in input_paths:
        if not path.exists():
            print(f"エラー: ディレクトリが存在しません: {path}")
            sys.exit(1)

    # 出力ディレクトリの設定
    if args.output:
        output_dir = Path(args.output)
    else:
        # デフォルト: comparison_results/<timestamp>
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path("comparison_results") / timestamp

    # 出力ディレクトリを作成
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"出力ディレクトリ: {output_dir}")
    print()

    # データセット名の設定
    if args.names:
        if len(args.names) != len(input_paths):
            print("エラー: --names で指定した名前の数が入力ディレクトリの数と一致しません")
            sys.exit(1)
        dataset_names = args.names
    else:
        dataset_names = [path.name for path in input_paths]

    # メトリクスの読み込み
    metrics_list = []

    for path, name in zip(input_paths, dataset_names):
        print(f"\n処理中: {path} (名前: {name})")

        bag_file = find_result_bag(path)
        if bag_file is None:
            print(f"警告: result_bag が見つかりません: {path}")
            continue

        metrics = read_rosbag_metrics(bag_file, name)
        if metrics is not None:
            metrics_list.append(metrics)

    if not metrics_list:
        print("\nエラー: 有効なデータが読み込めませんでした")
        sys.exit(1)

    print(f"\n読み込んだデータセット数: {len(metrics_list)}")

    # 可視化
    print(f"\n可視化を実行中...")
    plot_metrics(metrics_list, output_dir)

    print(f"\n完了! 出力ディレクトリ: {output_dir}")


if __name__ == "__main__":
    main()

