# driving_log_replayer_v2 Analyzer

driving_log_replayer_v2 Analyzer is a package that analyses result files.

## Supported Use Cases

- [obstacle_segmentation](https://github.com/tier4/driving_log_replayer_v2/blob/develop/docs/use_case/obstacle_segmentation.ja.md)
- [ground_segmentation](https://github.com/tier4/driving_log_replayer_v2/blob/develop/docs/use_case/ground_segmentation.ja.md)

## Requirements

Same as driving_log_replayer_v2

## Install

This package is installed with driving_log_replayer_v2_cli

## How to use

### General Usage

```shell
dlr2-analyzer analysis ${use-case-name} ${result.jsonl_path} [-o ${output_dir}]
```

### Ground Segmentation Analysis

To visualize the performance metrics of ground segmentation:

```shell
dlr2-analyzer analysis ground-segmentation ${result.jsonl_path} -o ${output_dir}
```

**Outputs:**

- `ground_segmentation_metrics.png/html`: Time-series plots of Accuracy, Precision, Recall, and F1-score (in %).
- `ground_segmentation_confusion_matrix.png/html`: Time-series plots of TP, FP, TN, and FN counts.

### Obstacle Segmentation Analysis

```shell
dlr2-analyzer analysis obstacle-segmentation ${result.jsonl_path} [-o ${output_dir}] [-c ${config_path}] [-d ${dist_type}]
```
