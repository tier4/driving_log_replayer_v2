# reidentify

実機 rosbag から車両モデルを再同定する専用パイプラインです。正規入口はリポジトリ直下の
`make reidentify` だけです。sim、closed-loop、Web.Auto、個別ステップ実行は扱いません。

## 実行

```bash
make reidentify

# 入力を上書きする場合
make reidentify \
  ROOT=/path/to/collection \
  SCENARIO=/path/to/scenario.yaml \
  INPUT_PARAM=/path/to/simulator_model.param.yaml
```

既定入力は、従来と同じ `ROOT=/home/kotaroyoshimoto/data/openloop_j6_16_onwards`、
`SCENARIO=sample/scenario.yaml`、description package の `simulator_model.param.yaml` です。
`MULTI_BATCH_ROOT`、`LOCAL_SCENARIO`、`LOCAL_SAMPLE_DIR`、
`REAL_LOG_SIM_COMPARISON_JOBS` も従来互換のフォールバックとして利用できます。

任意変数は `N_TRIALS=640`、`N_JOBS=32`、`FORCE=0` です。
ROS 2 環境をすでに読み込んでいる場合や単体テストでは `NO_SETUP=1` を指定できます。
`FORCE=1` の場合だけ既存の CSV cache を再生成します。

## 入力

collection は次の構造にします。`input_bag` は rosbag2 の DB3 または MCAP です。

```text
<ROOT>/
└── datasets/
    ├── <dataset-id>/
    │   └── input_bag/
    └── <dataset-id>/
        └── input_bag/
```

`SCENARIO` には `Evaluation.Conditions.models` の `baseline` と同定対象の `current`
を定義します。`current.vehicle_model_type` は
`delay_steer_acc_geared_for_diffusion_planner` 固定です。最小例は `sample/scenario.yaml` です。

`INPUT_PARAM` は `vehicle_model_type: DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER`
を選択し、`delay_steer_acc_geared_for_diffusion_planner` の versioned block を持つ必要があります。
リリース時は versioned パラメータを `v100` に、`steer_rate_lim`、`wheel_base` などの
global パラメータをトップレベルに反映し、レポートで評価したモデルと一致させます。

## 固定パイプライン

1. `extract`: 必要な7トピックを dataset ごとの `reidentify_cache.csv` に変換する。
2. `fit_lon`: 縦方向の時定数、むだ時間、scale を直接同定する。
3. `fit_steer`: 操舵応答、bias、scale、`k_us` を直接同定する。
4. `fit_merge`: 直接同定値を warm-start にして統合最適化し、`comparison_models` の全モデルの N-step 指標を確定する。`current` は tuned として出力する。探索スコア用の N は従来の5点を維持し、レポート用には N=1〜100 を出力する。
5. `release`: tuned params を入力パラメータ YAML の `v100` に反映する。
6. `report`: 全工程の成果物、確定済み N-step 指標、物理的妥当性検証を統合した最終 HTML を生成する。N-step ロールアウトは再実行しない。

## 成果物

```text
<ROOT>/reidentify/
├── phase1_acc.yaml
├── phase2_steer.yaml
├── tuned_params.yaml
├── metrics.csv
├── report.html
└── simulator_model.param.yaml
```

`reidentify_cache.csv` が存在する dataset は通常再利用されます。不完全な dataset は理由付きで
除外され、有効な dataset が0件の場合はパイプライン全体が失敗します。
