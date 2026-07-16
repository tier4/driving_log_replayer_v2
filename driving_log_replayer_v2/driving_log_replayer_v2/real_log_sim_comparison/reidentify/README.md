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

任意変数は `N_TRIALS=160`、`N_JOBS=32`、`FORCE=0` です。
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

`SCENARIO` には `Evaluation.Conditions.models` の `baseline`（必須）と、fit を使う場合は
fit 対象ケース（既定 `current`）を定義します。fit 対象の `vehicle_model_type` は
`delay_steer_acc_geared_for_diffusion_planner` 固定です。実行ステージは
`Evaluation.Conditions.fit: {target, stages}` と `Evaluation.Conditions.release: {model, version}`
で制御します。最小例は `sample/scenario.yaml` です。

`INPUT_PARAM` は release 指定がある場合のみ必須で、
`vehicle_model_type: DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER` を選択し、
`delay_steer_acc_geared_for_diffusion_planner` の versioned block を持つ必要があります。
リリース時は versioned パラメータを `v<N>` に、`steer_rate_lim`、`wheel_base` などの
global パラメータをトップレベルに反映し、レポートで評価したモデルと一致させます。

## パイプライン（scenario 駆動）

1. `extract`: 必要な7トピックを dataset ごとの `reidentify_cache.csv` に変換する（常時）。
2. `fit_lon` / `fit_steer` / `fit_xy`: `fit.stages` の direct-fit 連続プレフィックスだけ実行し、
   縦方向 τ/むだ時間/scale・操舵応答/bias/scale/`k_us`・`xy_heading_rate_coeff` を直接同定する。
3. `fit_merge`: `fit.stages` に `merge` があるときだけ実行。直接同定値（または fit 対象ケース）を
   warm-start に統合最適化し、fit 出力を `tuned` として確定する。
4. `evaluate`: `merge` を含まない（または fit なしの）場合に実行する評価専用ステージ。
   `merge` の有無に関わらず、`comparison_models` 全モデル（+ fit 出力 `tuned`）の N-step 指標を
   `metrics.csv` に確定する。探索スコア用 N は5点、レポート用は N=1〜300。
5. `release`: `release` 指定があるときだけ実行。指定ケースまたは `tuned` を入力 YAML の `v<N>` に反映する。
6. `report`: 全工程の成果物、確定済み N-step 指標、物理的妥当性検証を統合した最終 HTML を生成する。
   7 章にリリースされた param 値（版スロット + global 制限）のテーブルを含む。N-step ロールアウトは再実行しない。

## 成果物

```text
<ROOT>/reidentify/
├── phase1_acc.yaml
├── phase2_steer.yaml
├── phase3_xy.yaml
├── tuned_params.yaml
├── metrics.csv
├── report.html
└── simulator_model.param.yaml
```

`reidentify_cache.csv` が存在する dataset は通常再利用されます。不完全な dataset は理由付きで
除外され、有効な dataset が0件の場合はパイプライン全体が失敗します。
