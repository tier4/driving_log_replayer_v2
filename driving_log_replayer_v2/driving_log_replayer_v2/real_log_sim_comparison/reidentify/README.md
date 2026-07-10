# reidentify

`real_log_sim_comparison` の車両モデル再同定パイプライン。
複数データセットの `real.lite` / `reidentify_cache.csv` から物理パラメータを同定し、
検証レポートとリリース用 `simulator_model.param.yaml` を生成する。

## 正規入口

Makefile 入口:

```bash
make reidentify MULTI_BATCH_ROOT=<collection_dir> LOCAL_SCENARIO=<scenario.yaml>
```

個別ステップ:

```bash
make reidentify_extract
make reidentify_fit_lon
make reidentify_fit_steer
make reidentify_fit_merge
make reidentify_report
make reidentify_release
make reidentify_apply
make reidentify_closed_loop
```

Python 入口:

```bash
python3 -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.pipeline \
    --collection-dir <collection_dir> \
    --scenario <scenario.yaml> \
    --case current \
    --only fit_lon,fit_steer,fit_merge,report,release
```

`--only` は `extract,fit_lon,fit_steer,fit_merge,report,release` のカンマ区切り。
CSV cache が既にある場合は `extract` を含めないことで ROS 依存を読み込まない。

## 成果物

成果物は `<collection_dir>/reidentify/` 配下に集約する。

```text
reidentify/
├── phase1_acc.yaml
├── phase2_steer.yaml
├── tuned_params.yaml
├── physical_validity_report.html
├── physical_validity_release_note.html
├── metrics_cache.csv
├── simulator_model.param.yaml
└── tuned_scenario.yaml
```

`tuned_params.yaml` の YAML schema は既存パイプラインと同じ。

## 処理ステップ

1. `extract.py`: `real.lite` から dataset ごとの `reidentify_cache.csv` を生成する。
2. `fit_lon.py`: 縦方向の一次遅れ、むだ時間、加速度scaleを直接同定する。
3. `fit_steer.py`: 操舵一次遅れ、むだ時間、bias、scale、`k_us` を直接同定する。
4. `fit_merge.py`: 直接同定値を warm-start / passthrough にして Optuna で統合最適化する。
5. `physical_validity_report.py`: HTML検証レポートとリリースノートを生成する。
6. `release_params.py`: `tuned_params.yaml` を `simulator_model.param.yaml` の `v100` に反映する。

## 依存境界

`extract.py` だけが `rclpy`, `rosbag2_py`, `rosidl_runtime_py` に依存する。
`fit_lon.py` 以降は `reidentify_cache.csv` を入力にし、`pandas` / `numpy` / `ctypes`
と既存の車両モデル共有ライブラリで動く。

`fit_merge.py` は `rollout.py` 経由で `libvehicle_model_wrapper.so` を使う。
これは closed-loop 実simと同じ車両モデル実装で N-step ロールアウトを評価するため。
