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

任意変数は `N_TRIALS=160`、`N_JOBS=32`、`FORCE=0` です。
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
    ├── phase3_xy.yaml
    ├── tuned_params.yaml
    ├── metrics.csv
    ├── report.html
    └── simulator_model.param.yaml
```

初回は raw bag から、同定に必要なトピックだけを dataset ごとの
`reidentify_cache.csv` に抽出します。2回目以降は cache を再利用します。

## 処理内容

`make reidentify` は scenario の設定に従って次のステージを順に実行します。

```text
raw bag -> CSV cache -> [fit_lon -> fit_steer -> fit_xy] -> [fit_merge | evaluate] -> [release YAML] -> report.html
```

どのステージを実行するかは scenario の `Evaluation.Conditions.fit` / `release` で制御します。

fit を完走済みで report だけ作り直したい場合（scenario 表示の調整・report 段階だけ失敗した実行の
回収など）は、高価な再フィットなしで `make report` を実行します（既存の `tuned_params.yaml` /
`metrics.csv` から `report.html` のみ再生成。ROS 非依存なので `NO_SETUP=1` でも可）。

- `fit: {target: <case>, stages: [lon, steer, xy, merge]}` — `target` は同定の初期値/対象ケース
  （既定 `current`）。`stages` は `lon -> steer -> xy` の**連続する先頭部分**に加えて任意で `merge`
  を続けられます（歯抜け不可）。省略時は全ステージ、`[]` で fit を行わず固定比較のみになります。
  `merge` を含まない場合は Optuna 統合最適化を行わず、直接同定の結果（あれば）を評価します。
  fit 出力は `tuned` として比較表・リリース対象になります（ハードコードの `current` 特別扱いは廃止）。
- `release: {model: <case>|tuned, version: <N>}` — 指定時のみ release ステージが動き、そのケース
  （または fit 出力 `tuned`）を `simulator_model.param.yaml` の `v<N>` スロットへ書き `version: <N>`
  で選択します。省略すると release（YAML 出力）はスキップされ、`--input-param` も不要です。

`fit_merge` が `comparison_models` の全モデル（`current` は `tuned` 表記）の dataset別・horizon別 N-step 指標を `metrics.csv` に一度だけ
書き出します。最適化スコアは yaw/縦/横 を N=10/30/70/150/300 で、steer/ax の
アクチュエータ項を N=10/30 で算出して合算します（steer/ax の open-loop 誤差は
N≈20 までに定常値へ飽和するプラトー特性を持つため、過渡とプラトーの代表 2 点で足ります。
定義式は report.html の「評価関数の定義」を参照）。worst 側テールは単一最悪値（max）でなく
CVaR@90%（正規化比の上位 10% の平均）で評価し、正規化フロアは baseline 誤差分布の p10 を
horizon 別に採用します（objective v3）。レポート用指標は
データが保証される N=1〜300 を1刻みで出力します。`report.html` はメトリクスごとの誤差推移を
その CSV から描画し、同じ文書内で CSV キャッシュを一度だけ読んで縦方向・操舵・ヨー (`k_us`)・
x/y 方程式残差も評価します。N-step 指標のロールアウトは再計算しません。scenario の
`Evaluation.Conditions.comparison_models` に比較するケース名を列挙します（例:
`[baseline, v1, v2, current]`）。重複・未定義ケースはエラーで、`baseline` のみ必須です。
fit 出力 `tuned` を比較表・グラフに載せたい場合は fit 対象ケース（既定 `current`）を
comparison_models に**明記**します。明記しなければ `tuned` は比較には出ません（fit は実行され、
結果は 6 章「Final parameters」および `tuned_params.yaml` に残り、`release: {model: tuned}` で
リリースも可能です）。

各モデルは宣言済みパラメータで固定 RMSE 評価され、baseline 比・サンプル数・使用パラメータを
比較表と分布で表示します。comparison_models に明記した fit 対象ケースだけは scenario の `params` に
`tuned_params.yaml` の `params` を上書きマージして `tuned` として表示し、それ以外の比較モデルは
scenario に書かれた値をそのまま使います。

必須topicがない、データが空、有効区間が短すぎる dataset は理由を表示して除外します。
同定可能な dataset が1件も残らない場合はエラー終了します。

## リリース対象の指定

release ステージは scenario に `Evaluation.Conditions.release: {model: <case>|tuned, version: <N>}`
が指定された場合のみ実行され、そのケース（または fit 出力 `tuned`）のパラメータを
`v<N>` スロットへ書き `version: <N>` で選択します。自動（magic）な既定リリースはありません。
release を省略すると release ステージ（`simulator_model.param.yaml` 出力）はスキップされ、
`--input-param` も不要です。
例: `release: {model: v2, version: 2}` — プラトー補正 + k_us 更新済みの固定ケース v2 を
リリース候補として出力します。`release: {model: tuned, version: 100}` なら統合最適化の結果を出します。
固定ケースの再リリースは入力 YAML に別内容の `v<N>` があるとエラー（同一内容は冪等に許可）。
`tuned` は Optuna の再現性が保証されないため冪等ガードを外し、既存スロットを上書きします。

## スケーリングのプラトー同定（2段構成）

各系統の直接同定は「τ・むだ時間を動的励起データの最小二乗で決定 → scaling factor を
プラトー（N-step rollout の定常誤差、既定 N=30）の最小化で決定」の2段で行います。
同定コアは rollout の正式評価を使う `fit_plateau.fit_scaling_channels` の1本で、
パイプライン（`make reidentify`）の fit_lon / fit_steer が τ/delay 確定後に自チャネルの
scaling をこのコアで決め直します（独立したツール実行はありません）。
steer 終端状態は steer 系のみ、ax 終端状態は acc 系のみに依存するため、目的関数は
`J_steer(steer scaling)` と `J_ax(acc scaling)` の**独立な1次元フィット**に分離されます
（`steer_bias` はモデル構造上 steer プラトーに影響しないため対象外）。プラトー特性・GT 整備
（`*_savgol` 平滑化）などの方法論解説は `report.html` の 1 章（記号と運動方程式）と
系統別の 3・4 章にまとめています。

scenario の最小契約と成果物の詳細は [`reidentify/README.md`](reidentify/README.md) を参照してください。

## 残差分析キャンペーン（make analyze）

v3 以降の構造仮説を証拠付きで確定させるための分析ステージ群です。`make reidentify` の
成果物・挙動には影響せず、成果物は `<ROOT>/reidentify/analysis/`（analysis.html ほか）に
出力されます。

```bash
make analyze                     # 既定: split,pitch-sign,traces,conditioned,regime,steady,tail,etfe,report
make analyze ANALYZE_STAGES=oracle,report   # 重い per-dataset oracle のみ追加実行
```

- `split` — dataset_id の sha1 による dev/holdout 決定的分割（分析・fit は dev、最終判定は holdout 併用）
- `pitch-sign` — コースト区間回帰による pitch 符号規約の検証（slope 給電の前提）
- `traces` — per-start 署名付き N-step 終端誤差の抽出（`rollout.eval_rollout_terminal_errors`）
- `conditioned` — 特徴量条件付き残差（2 段集計 + bootstrap CI + BH 補正）と反実仮想 RMSE 予測
- `regime` — 縦レジーム分割評価（brake / coast / throttle 別の RMSE・バイアス）
- `steady` — 定常 Hammerstein マップ（静的ゲインの非線形・非対称の直接推定）
- `oracle` — per-dataset scaling oracle（構造欠落 vs 個体差の切り分け、重い）
- `tail` — CVaR テールの走行条件特性（metrics.csv から、追加 rollout なし）
- `etfe` — cmd→achieved 周波数応答の適合性チェック（閉ループバイアスのため同定には不使用）

## 採用ゲート（reidentify.gate）

v2 採用時に手作業だったセル別非劣化判定（mean/cvar × horizon × 指標が参照モデル比
+2% 以内）を成文化した CLI です。

```bash
# metrics.csv からのリリース級判定（dense horizon）
python3 -m ...reidentify.gate metrics --metrics <ROOT>/reidentify/metrics.csv \
  --candidate tuned --reference v2

# パラメータ上書き候補の速報スクリーニング（score horizon のみ、直接 rollout）
python3 -m ...reidentify.gate screen --root <ROOT> --scenario sample/scenario.yaml \
  --case v2 --set brake_scaling_factor=0.8 --slope-source pitch --n-jobs 32 --splits dev
```

## SLOPE_ACCX 給電（slope_source）と v3 構造項

scenario の各 case は `slope_source: none|pitch` を持てます。`pitch` のとき rollout は
`+g·sin(pitch_lf)`（localization pitch の <0.1 Hz 成分、符号はコースト回帰で検証済み）を
SLOPE_ACCX に給電します。実機経路（simple_planning_simulator の
`enable_road_slope_simulation`）は実勾配を給電しているため、`none`（従来）は
「無勾配で同定し勾配ありの sim にリリースする」系統不整合を含む点に注意してください。

モデルには v3 構造項として `lon_drag_c0` / `lon_drag_c2`（走行抵抗、VX 右辺）、
`brake_time_constant` / `brake_scaling_factor`（減速指令時の非対称応答、<=0 は対称値継承の
センチネル）が追加されています（両シミュレータ実装 + wrapper `_v3`/`_v2` 新シンボル、
中立値で v2 と bit 一致）。探索・検証域の SSOT は `reidentify/parameter_constraints.py`。
