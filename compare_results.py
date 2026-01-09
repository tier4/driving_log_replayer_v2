#!/usr/bin/env python3

"""
2つまたは3つのresult.jsonlファイルを比較して可視化するスクリプト
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# 日本語フォントの設定
matplotlib.rcParams['font.family'] = 'DejaVu Sans'


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


def create_comparison_plots(metrics1: Dict[str, List], metrics2: Dict[str, List],
                           label1: str, label2: str, output_path: Path,
                           metrics3: Optional[Dict[str, List]] = None,
                           label3: Optional[str] = None):
    """比較プロットを作成（2つまたは3つの結果を比較）"""

    # プロット用のデータを準備
    frames1 = np.array(metrics1['frame_numbers'])
    frames2 = np.array(metrics2['frame_numbers'])
    has_third = metrics3 is not None and label3 is not None
    if has_third:
        frames3 = np.array(metrics3['frame_numbers'])

    # メトリクス名とラベルのマッピング
    metric_configs = [
        ('accuracy', 'Accuracy', 'Accuracy'),
        ('precision', 'Precision', 'Precision'),
        ('recall', 'Recall', 'Recall'),
        ('specificity', 'Specificity', 'Specificity'),
        ('f1_score', 'F1-score', 'F1-score'),
    ]

    # メトリクス比較プロット
    fig, axes = plt.subplots(len(metric_configs), 1, figsize=(12, 3 * len(metric_configs)))
    if len(metric_configs) == 1:
        axes = [axes]

    for idx, (key, title, ylabel) in enumerate(metric_configs):
        ax = axes[idx]

        values1 = np.array(metrics1[key])
        values2 = np.array(metrics2[key])

        # None値を除外
        valid1 = ~np.isnan(values1) if values1.dtype == float else np.array([v is not None for v in values1])
        valid2 = ~np.isnan(values2) if values2.dtype == float else np.array([v is not None for v in values2])

        if np.any(valid1):
            ax.plot(frames1[valid1], values1[valid1], 'o-', label=label1, linewidth=2, markersize=4)
        if np.any(valid2):
            ax.plot(frames2[valid2], values2[valid2], 's-', label=label2, linewidth=2, markersize=4, alpha=0.7)

        if has_third:
            values3 = np.array(metrics3[key])
            valid3 = ~np.isnan(values3) if values3.dtype == float else np.array([v is not None for v in values3])
            if np.any(valid3):
                ax.plot(frames3[valid3], values3[valid3], '^-', label=label3, linewidth=2, markersize=4, alpha=0.7)

        ax.set_xlabel('Frame Number', fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(f'{title} Comparison', fontsize=12, fontweight='bold')
        ax.legend(loc='best')
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

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()

    for idx, (key, title, ylabel) in enumerate(confusion_metrics):
        ax = axes[idx]

        values1 = np.array(metrics1[key])
        values2 = np.array(metrics2[key])

        # None値を除外
        if key == 'accuracy':
            # Accuracyはfloat型なので、NaNチェック
            valid1 = ~np.isnan(values1) if values1.dtype == float else np.array([v is not None for v in values1])
            valid2 = ~np.isnan(values2) if values2.dtype == float else np.array([v is not None for v in values2])
        else:
            valid1 = np.array([v is not None for v in values1])
            valid2 = np.array([v is not None for v in values2])

        if np.any(valid1):
            ax.plot(frames1[valid1], values1[valid1], 'o-', label=label1, linewidth=2, markersize=4)
        if np.any(valid2):
            ax.plot(frames2[valid2], values2[valid2], 's-', label=label2, linewidth=2, markersize=4, alpha=0.7)

        if has_third:
            values3 = np.array(metrics3[key])
            if key == 'accuracy':
                valid3 = ~np.isnan(values3) if values3.dtype == float else np.array([v is not None for v in values3])
            else:
                valid3 = np.array([v is not None for v in values3])
            if np.any(valid3):
                ax.plot(frames3[valid3], values3[valid3], '^-', label=label3, linewidth=2, markersize=4, alpha=0.7)

        ax.set_xlabel('Frame Number', fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=-1)

    # 6つ目のプロット（未使用）を非表示にする
    if len(confusion_metrics) < len(axes):
        axes[len(confusion_metrics)].axis('off')

    plt.tight_layout()
    plt.savefig(output_path / 'confusion_matrix_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()

    # 統計サマリー
    print_summary(metrics1, metrics2, label1, label2, output_path, metrics3, label3)


def print_summary(metrics1: Dict[str, List], metrics2: Dict[str, List],
                  label1: str, label2: str, output_path: Path,
                  metrics3: Optional[Dict[str, List]] = None,
                  label3: Optional[str] = None):
    """統計サマリーを出力（2つまたは3つの結果を比較）"""

    def get_stats(values):
        """統計値を計算"""
        valid_values = [v for v in values if v is not None]
        if not valid_values:
            return None, None, None, None
        arr = np.array(valid_values)
        return np.mean(arr), np.std(arr), np.min(arr), np.max(arr)

    has_third = metrics3 is not None and label3 is not None

    summary_lines = []
    summary_lines.append("=" * 80)
    summary_lines.append("比較結果サマリー")
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
        values1 = metrics1[key]
        values2 = metrics2[key]

        mean1, std1, min1, max1 = get_stats(values1)
        mean2, std2, min2, max2 = get_stats(values2)

        summary_lines.append(f"{name}:")
        summary_lines.append(f"  {label1}:")
        if mean1 is not None:
            summary_lines.append(f"    平均: {mean1:.4f}, 標準偏差: {std1:.4f}, 最小: {min1:.4f}, 最大: {max1:.4f}")
        else:
            summary_lines.append(f"    データなし")

        summary_lines.append(f"  {label2}:")
        if mean2 is not None:
            summary_lines.append(f"    平均: {mean2:.4f}, 標準偏差: {std2:.4f}, 最小: {min2:.4f}, 最大: {max2:.4f}")
        else:
            summary_lines.append(f"    データなし")

        if has_third:
            values3 = metrics3[key]
            mean3, std3, min3, max3 = get_stats(values3)
            summary_lines.append(f"  {label3}:")
            if mean3 is not None:
                summary_lines.append(f"    平均: {mean3:.4f}, 標準偏差: {std3:.4f}, 最小: {min3:.4f}, 最大: {max3:.4f}")
            else:
                summary_lines.append(f"    データなし")

        if mean1 is not None and mean2 is not None:
            diff = mean2 - mean1
            summary_lines.append(f"  差分 ({label2} - {label1}): {diff:+.4f}")

        if has_third and mean1 is not None and mean3 is not None:
            diff3 = mean3 - mean1
            summary_lines.append(f"  差分 ({label3} - {label1}): {diff3:+.4f}")

        if has_third and mean2 is not None and mean3 is not None:
            diff23 = mean3 - mean2
            summary_lines.append(f"  差分 ({label3} - {label2}): {diff23:+.4f}")

        summary_lines.append("")

    summary_text = "\n".join(summary_lines)
    print(summary_text)

    # ファイルに保存
    with open(output_path / 'summary.txt', 'w', encoding='utf-8') as f:
        f.write(summary_text)


def main():
    parser = argparse.ArgumentParser(
        description='2つまたは3つのresult.jsonlファイルを比較して可視化する',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        'file1',
        type=Path,
        help='比較する1つ目のresult.jsonlファイルのパス',
    )
    parser.add_argument(
        'file2',
        type=Path,
        help='比較する2つ目のresult.jsonlファイルのパス',
    )
    parser.add_argument(
        'file3',
        type=Path,
        nargs='?',
        default=None,
        help='比較する3つ目のresult.jsonlファイルのパス（オプション）',
    )
    parser.add_argument(
        '--label1',
        type=str,
        default='File 1',
        help='1つ目のファイルのラベル（デフォルト: File 1）',
    )
    parser.add_argument(
        '--label2',
        type=str,
        default='File 2',
        help='2つ目のファイルのラベル（デフォルト: File 2）',
    )
    parser.add_argument(
        '--label3',
        type=str,
        default=None,
        help='3つ目のファイルのラベル（file3が指定された場合に使用）',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('comparison_output'),
        help='出力ディレクトリ（デフォルト: comparison_output）',
    )

    args = parser.parse_args()

    # 出力ディレクトリを作成
    args.output.mkdir(parents=True, exist_ok=True)

    # ファイルを読み込む
    print(f"読み込み中: {args.file1}")
    data1 = load_jsonl(args.file1)
    print(f"読み込み中: {args.file2}")
    data2 = load_jsonl(args.file2)

    # メトリクスを抽出
    print("メトリクスを抽出中...")
    metrics1 = extract_ground_segmentation_metrics(data1)
    metrics2 = extract_ground_segmentation_metrics(data2)

    print(f"File 1: {len(metrics1['frame_numbers'])} フレーム")
    print(f"File 2: {len(metrics2['frame_numbers'])} フレーム")

    # 3つ目のファイルが指定されている場合
    metrics3 = None
    label3 = None
    if args.file3 is not None:
        print(f"読み込み中: {args.file3}")
        data3 = load_jsonl(args.file3)
        metrics3 = extract_ground_segmentation_metrics(data3)
        print(f"File 3: {len(metrics3['frame_numbers'])} フレーム")
        label3 = args.label3 if args.label3 else 'File 3'

    # プロットを作成
    print("可視化中...")
    create_comparison_plots(metrics1, metrics2, args.label1, args.label2, args.output,
                           metrics3, label3)

    print(f"\n結果を {args.output} に保存しました:")
    print(f"  - metrics_comparison.png: メトリクス比較グラフ")
    print(f"  - confusion_matrix_comparison.png: 混同行列要素比較グラフ")
    print(f"  - summary.txt: 統計サマリー")


if __name__ == '__main__':
    main()

