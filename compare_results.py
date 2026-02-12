#!/usr/bin/env python3

"""
複数のresult.jsonlファイルを比較して可視化するスクリプト（任意の数に対応）
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# 日本語フォントの設定
matplotlib.rcParams['font.family'] = 'DejaVu Sans'

# マーカーとカラーのリスト（多数のデータセットに対応）
MARKERS = ['o', 's', '^', 'D', 'v', '<', '>', 'p', '*', 'h', 'H', '+', 'x', 'd', '|', '_']
COLORS = plt.cm.tab10.colors + plt.cm.tab20.colors


def load_jsonl(file_path: Path) -> List[Dict]:
    """JSONLファイルを読み込む"""
    data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.strip():
                data.append(json.loads(line))
    return data


def extract_ground_segmentation_metrics(data: List[Dict]) -> Dict[str, List]:
    """Ground Segmentationのメトリクスを抽出"""
    metrics = {
        'frame_numbers': [],
        'accuracy': [],
        'precision': [],
        'recall': [],
        'specificity': [],
        'f1_score': [],
        'tp': [],
        'fp': [],
        'tn': [],
        'fn': [],
        'ros_time': [],
    }

    frame_count = 0
    for entry in data:
        # Condition行はスキップ
        if 'Condition' in entry:
            continue

        frame = entry.get('Frame', {})
        ground_seg = frame.get('Ground Segmentation', {})
        gs_result = ground_seg.get('GroundSegmentation', {})

        if gs_result:
            info = gs_result.get('Info', {})
            if info:
                metrics['frame_numbers'].append(frame_count)
                metrics['accuracy'].append(info.get('Accuracy', None))
                metrics['precision'].append(info.get('Precision', None))
                metrics['recall'].append(info.get('Recall', None))
                metrics['specificity'].append(info.get('Specificity', None))
                metrics['f1_score'].append(info.get('F1-score', None))
                metrics['tp'].append(info.get('TP', None))
                metrics['fp'].append(info.get('FP', None))
                metrics['tn'].append(info.get('TN', None))
                metrics['fn'].append(info.get('FN', None))

                # ROS時間を取得
                ros_time = entry.get('Stamp', {}).get('ROS', None)
                metrics['ros_time'].append(ros_time)
                frame_count += 1

    return metrics


def create_comparison_plots(metrics_list: List[Dict[str, List]], labels: List[str], output_path: Path):
    """比較プロットを作成（任意の数の結果を比較）"""

    num_datasets = len(metrics_list)
    print(f"  {num_datasets} データセットを比較中...")

    # メトリクス名とラベルのマッピング
    metric_configs = [
        ('accuracy', 'Accuracy', 'Accuracy'),
        ('precision', 'Precision', 'Precision'),
        ('recall', 'Recall', 'Recall'),
        ('specificity', 'Specificity', 'Specificity'),
        ('f1_score', 'F1-score', 'F1-score'),
    ]

    # メトリクス比較プロット
    fig, axes = plt.subplots(len(metric_configs), 1, figsize=(14, 3 * len(metric_configs)))
    if len(metric_configs) == 1:
        axes = [axes]

    for idx, (key, title, ylabel) in enumerate(metric_configs):
        ax = axes[idx]

        for i, (metrics, label) in enumerate(zip(metrics_list, labels)):
            frames = np.array(metrics['frame_numbers'])
            values = np.array(metrics[key])

            # None値を除外
            valid = ~np.isnan(values) if values.dtype == float else np.array([v is not None for v in values])

            if np.any(valid):
                marker = MARKERS[i % len(MARKERS)]
                color = COLORS[i % len(COLORS)]
                ax.plot(frames[valid], values[valid], marker=marker, linestyle='-',
                       label=label, linewidth=2, markersize=4, alpha=0.8, color=color)

        ax.set_xlabel('Frame Number', fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(f'{title} Comparison ({num_datasets} datasets)', fontsize=12, fontweight='bold')
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=-1)

    plt.tight_layout()
    plt.savefig(output_path / 'metrics_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()

    # TP, FP, TN, FN, Accuracyの比較プロット
    confusion_metrics = [
        ('tp', 'True Positive (TP)', 'Count'),
        ('fp', 'False Positive (FP)', 'Count'),
        ('tn', 'True Negative (TN)', 'Count'),
        ('fn', 'False Negative (FN)', 'Count'),
        ('accuracy', 'Accuracy', 'Accuracy'),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(20, 10))
    axes = axes.flatten()

    for idx, (key, title, ylabel) in enumerate(confusion_metrics):
        ax = axes[idx]

        for i, (metrics, label) in enumerate(zip(metrics_list, labels)):
            frames = np.array(metrics['frame_numbers'])
            values = np.array(metrics[key])

            # None値を除外
            if key == 'accuracy':
                valid = ~np.isnan(values) if values.dtype == float else np.array([v is not None for v in values])
            else:
                valid = np.array([v is not None for v in values])

            if np.any(valid):
                marker = MARKERS[i % len(MARKERS)]
                color = COLORS[i % len(COLORS)]
                ax.plot(frames[valid], values[valid], marker=marker, linestyle='-',
                       label=label, linewidth=2, markersize=4, alpha=0.8, color=color)

        ax.set_xlabel('Frame Number', fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=-1)

    # 6つ目のプロット（未使用）を非表示にする
    if len(confusion_metrics) < len(axes):
        axes[len(confusion_metrics)].axis('off')

    plt.tight_layout()
    plt.savefig(output_path / 'confusion_matrix_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()

    # 統計サマリー
    print_summary(metrics_list, labels, output_path)


def print_summary(metrics_list: List[Dict[str, List]], labels: List[str], output_path: Path):
    """統計サマリーを出力（任意の数の結果を比較）"""

    def get_stats(values):
        """統計値を計算"""
        valid_values = [v for v in values if v is not None]
        if not valid_values:
            return None, None, None, None
        arr = np.array(valid_values)
        return np.mean(arr), np.std(arr), np.min(arr), np.max(arr)

    num_datasets = len(metrics_list)

    summary_lines = []
    summary_lines.append("=" * 80)
    summary_lines.append(f"比較結果サマリー ({num_datasets} データセット)")
    summary_lines.append("=" * 80)
    summary_lines.append("")

    metric_names = [
        ('accuracy', 'Accuracy'),
        ('precision', 'Precision'),
        ('recall', 'Recall'),
        ('specificity', 'Specificity'),
        ('f1_score', 'F1-score'),
    ]

    for key, name in metric_names:
        summary_lines.append(f"{name}:")

        stats_list = []
        for i, (metrics, label) in enumerate(zip(metrics_list, labels)):
            values = metrics[key]
            mean, std, min_val, max_val = get_stats(values)
            stats_list.append((label, mean, std, min_val, max_val))

            summary_lines.append(f"  {label}:")
            if mean is not None:
                summary_lines.append(f"    平均: {mean:.4f}, 標準偏差: {std:.4f}, 最小: {min_val:.4f}, 最大: {max_val:.4f}")
            else:
                summary_lines.append(f"    データなし")

        # 全ての組み合わせで差分を計算（最初のデータセットを基準に）
        if len(stats_list) > 1:
            summary_lines.append(f"  差分 (基準: {stats_list[0][0]}):")
            base_mean = stats_list[0][1]
            if base_mean is not None:
                for i in range(1, len(stats_list)):
                    label_i, mean_i, _, _, _ = stats_list[i]
                    if mean_i is not None:
                        diff = mean_i - base_mean
                        percent = (diff / base_mean * 100) if base_mean != 0 else 0
                        summary_lines.append(f"    {label_i}: {diff:+.4f} ({percent:+.2f}%)")

        summary_lines.append("")

    # 混同行列の要素も追加
    summary_lines.append("=" * 80)
    summary_lines.append("混同行列要素の統計")
    summary_lines.append("=" * 80)
    summary_lines.append("")

    confusion_keys = [
        ('tp', 'True Positive (TP)'),
        ('fp', 'False Positive (FP)'),
        ('tn', 'True Negative (TN)'),
        ('fn', 'False Negative (FN)'),
    ]

    for key, name in confusion_keys:
        summary_lines.append(f"{name}:")

        for metrics, label in zip(metrics_list, labels):
            values = metrics[key]
            mean, std, min_val, max_val = get_stats(values)

            summary_lines.append(f"  {label}:")
            if mean is not None:
                summary_lines.append(f"    平均: {mean:.1f}, 標準偏差: {std:.1f}, 最小: {min_val:.0f}, 最大: {max_val:.0f}")
            else:
                summary_lines.append(f"    データなし")

        summary_lines.append("")

    summary_text = "\n".join(summary_lines)
    print(summary_text)

    # ファイルに保存
    with open(output_path / 'summary.txt', 'w', encoding='utf-8') as f:
        f.write(summary_text)


def main():
    parser = argparse.ArgumentParser(
        description='複数のresult.jsonlファイルを比較して可視化する（任意の数に対応）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # 2つのファイルを比較
  %(prog)s file1.jsonl file2.jsonl --label1 "Model A" --label2 "Model B"

  # 5つのファイルを比較
  %(prog)s file1.jsonl file2.jsonl file3.jsonl file4.jsonl file5.jsonl \\
    --label1 "Original" --label2 "Grid0.05" --label3 "Grid0.10" \\
    --label4 "Concerto" --label5 "Concerto_Raw"
        """
    )
    parser.add_argument(
        'files',
        type=Path,
        nargs='+',
        help='比較するresult.jsonlファイルのパス（2つ以上）',
    )
    parser.add_argument(
        '--labels',
        type=str,
        nargs='+',
        default=None,
        help='各ファイルのラベル（files と同じ数を指定、デフォルト: File 1, File 2, ...）',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('comparison_output'),
        help='出力ディレクトリ（デフォルト: comparison_output）',
    )

    args = parser.parse_args()

    # 最低2つのファイルが必要
    if len(args.files) < 2:
        parser.error("少なくとも2つのファイルを指定してください")

    # ラベルの処理
    if args.labels is None:
        labels = [f"File {i+1}" for i in range(len(args.files))]
    else:
        if len(args.labels) != len(args.files):
            parser.error(f"ラベルの数 ({len(args.labels)}) がファイルの数 ({len(args.files)}) と一致しません")
        labels = args.labels

    # 出力ディレクトリを作成
    args.output.mkdir(parents=True, exist_ok=True)

    print(f"{'='*80}")
    print(f"{len(args.files)} データセットの比較を開始")
    print(f"{'='*80}")

    # 全てのファイルを読み込み、メトリクスを抽出
    metrics_list = []
    for i, (file_path, label) in enumerate(zip(args.files, labels)):
        print(f"\n[{i+1}/{len(args.files)}] 読み込み中: {file_path}")
        data = load_jsonl(file_path)

        print(f"  メトリクスを抽出中...")
        metrics = extract_ground_segmentation_metrics(data)
        print(f"  ✓ {label}: {len(metrics['frame_numbers'])} フレーム")

        metrics_list.append(metrics)

    # プロットを作成
    print(f"\n{'='*80}")
    print("可視化中...")
    print(f"{'='*80}")
    create_comparison_plots(metrics_list, labels, args.output)

    print(f"\n{'='*80}")
    print(f"✓ 結果を {args.output} に保存しました:")
    print(f"{'='*80}")
    print(f"  - metrics_comparison.png: メトリクス比較グラフ ({len(args.files)} データセット)")
    print(f"  - confusion_matrix_comparison.png: 混同行列要素比較グラフ")
    print(f"  - summary.txt: 統計サマリー")
    print("")


if __name__ == '__main__':
    main()
