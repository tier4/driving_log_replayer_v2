# 車両モデル再同定

このディレクトリは、複数の実機 rosbag からシミュレータの車両モデルを再同定するための
単一用途ツールです。旧 real/sim 比較、closed-loop sim、Web.Auto evaluator、解析バッチは
正規ワークフローに含みません。

## クイックスタート

従来のローカル環境では、既定値を使ってそのまま実行できます。

```bash
make reidentify
```

別の入力を使う場合は、各 dataset の raw rosbag を
`<ROOT>/datasets/<id>/input_bag` に配置して上書きします。

```bash
make reidentify \
  ROOT=/path/to/collection \
  SCENARIO=$(pwd)/sample/scenario.yaml \
  INPUT_PARAM=/path/to/simulator_model.param.yaml
```

入力変数と既定値は次のとおりです。環境変数または `make` のコマンドラインで上書きできます。

- `ROOT`: `/home/kotaroyoshimoto/data/openloop_j6_16_onwards`
- `SCENARIO`: `sample/scenario.yaml`
- `INPUT_PARAM`: `<WS_ROOT>/src/description/vehicle/j6_gen2_description/j6_gen2_description/config/simulator_model.param.yaml`

従来の環境変数も互換入力として利用できます。`MULTI_BATCH_ROOT` は `ROOT`、
`LOCAL_SCENARIO` は `SCENARIO`、`LOCAL_SAMPLE_DIR` はその配下の `scenario.yaml`、
`REAL_LOG_SIM_COMPARISON_JOBS` は `N_JOBS` のフォールバックになります。
新しい変数が設定されている場合は新しい変数を優先します。

任意変数は `N_TRIALS=640`、`N_JOBS=32`、`FORCE=0` です。
既存の `reidentify_cache.csv` を作り直す場合だけ `FORCE=1` を指定します。
通常はワークスペースの `install/setup.bash` を自動で読み込みます。すでに必要な環境が
設定済みの場合は `NO_SETUP=1` を利用できます。

## 入出力

```text
<ROOT>/
├── datasets/
│   ├── <dataset-id-a>/
│   │   └── input_bag/             # rosbag2 DB3/MCAP
│   └── <dataset-id-b>/
│       └── input_bag/
└── reidentify/
    ├── phase1_acc.yaml
    ├── phase2_steer.yaml
    ├── tuned_params.yaml
    ├── metrics.csv
    ├── report.html
    ├── physical_validity.html
    └── simulator_model.param.yaml
```

初回は raw bag から、同定に必要なトピックだけを dataset ごとの
`reidentify_cache.csv` に抽出します。2回目以降は cache を再利用します。

## 処理内容

`make reidentify` は次の処理を必ずこの順で実行します。

```text
raw bag -> CSV cache -> fit_lon -> fit_steer -> fit_merge -> report -> release YAML
```

`fit_merge` が baseline/tuned の dataset別・horizon別 N-step 指標を `metrics.csv` に一度だけ
書き出します。最適化スコアは従来どおり N=10/20/30/40/100 で算出し、レポート用指標は
データが保証される N=1〜100 を1刻みで出力します。`report.html` はメトリクスごとの誤差推移を
その CSV から描画するだけなので、同じロールアウトを再計算しません。

`physical_validity.html` は同じ CSV キャッシュを一度だけ読み、縦方向・操舵・ヨー (`k_us`)・
x/y 方程式残差を独立に評価します。scenario の
`Evaluation.Conditions.comparison_models` に比較するケース名を列挙します（例:
`[baseline, v1, v1_rk4, current]`）。重複・未定義ケースはエラーで、`baseline` と
`current` は必須です。

各モデルは宣言済みパラメータで固定 RMSE 評価され、baseline 比・サンプル数・使用パラメータを
比較表と分布で表示します。`baseline` と `v1` は確定済みモデルのためフィットせず、
`v1_*` を含むその他のモデルと `current` だけに従来のフィット診断を表示します。
`current` のみ scenario の `params` に `tuned_params.yaml` の `params` を上書きマージして使い、
それ以外の比較モデルは scenario に書かれた値をそのまま使います。

必須topicがない、データが空、有効区間が短すぎる dataset は理由を表示して除外します。
同定可能な dataset が1件も残らない場合はエラー終了します。

scenario の最小契約と成果物の詳細は [`reidentify/README.md`](reidentify/README.md) を参照してください。
