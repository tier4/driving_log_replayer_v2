# Real Log / Sim Comparisonの評価

実機走行ログとシミュレーション走行ログを同一ルートで比較し、Autoware のプランニング・制御性能の乖離を定量化するユースケース。
具体的には、速度応答、操舵応答、軌跡精度の差分を統計値（RMSE 等）と可視化プロットで提示する。

> **現状の実装範囲**
> 評価ノードは現状、実機ログのみを対象とした分析パイプラインを実行する。
> 将来的に `sim_normal` / `sim_godot` を含む三方比較（実機 + シム2系統）に拡張される設計だが、`sim_scenario.yaml` は TODO 状態であり、シム側の lite bag 生成・比較は本ドキュメント執筆時点では未配線である。
> 詳細は末尾の「[将来拡張：シム側パイプライン](#将来拡張シム側パイプライン)」を参照。

サンプルシナリオ: [sample/real_log_sim_comparison/scenario.yaml](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/real_log_sim_comparison/scenario.yaml)（x2_dev/2231 テレポート駅→日本科学未来館）。

## 評価方法

本ユースケースは他のユースケースと異なり、launch から **Autoware・bag player・bag recorder のいずれも起動しない**。
評価ノード `real_log_sim_comparison_evaluator` が以下のパイプラインを subprocess で直接実行する。

1. **lite bag 生成** — `make_lite`（`--kind real`）で実機 mcap/db3 から比較対象トピックのみを抽出し、`lite/real.lite` を生成する。
2. **比較プロット・レポート生成** — `compare_logs` で実機ログから速度・加速度・操舵・軌跡の比較プロット（PNG/PDF）と Markdown レポートを生成する。
3. **per-step delta 解析（best-effort）** — `analyze_curve2_per_step` で指定カーブのステップ単位 delta 解析を生成する。失敗してもステップ 1/2 の成果は維持される。

成否判定はパイプラインの例外有無で決まる。
全ステップ（best-effort のステップ 3 を除く）が完走すれば `result.jsonl` に `Success: true` が記録され、いずれかの subprocess が非ゼロ終了またはタイムアウトすると `Success: false` と Python traceback が記録される。

## シナリオ設定（Conditions）

シナリオ YAML の `Evaluation.Conditions` で以下のキーを認識する（いずれも任意）。

| キー                 | 内容                                                                                                  |
| -------------------- | ----------------------------------------------------------------------------------------------------- |
| `scenario_name`      | 図タイトルに表示するシナリオ名。未指定時は `ScenarioName` を使用。                                    |
| `curve_config_yaml`  | カーブ別解析設定 YAML への相対パス（`scenario.yaml` 基準）または絶対パス。空文字でカーブ別解析をスキップ。 |

`curve_config_yaml` は以下の構造で記述する（[`sample/real_log_sim_comparison/curve_config_miraikan.yaml`](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/real_log_sim_comparison/curve_config_miraikan.yaml) 参照）。

- `curve_centers`: 地図座標系でのカーブ中心リスト（`label`, `cx`, `cy`, `margin`）。`curves_closeup` プロット用。
- `curve2_index`: `curve_centers` 内で per-step delta 解析の対象とするカーブのインデックス（0 始まり）。
- `curve2_window`: カーブ② 直前の一時停止を検出する時刻窓 `[start, end]`（AUTONOMOUS 開始からの経過秒）。

## 評価対象トピック（抽出対象）

評価ノードは ROS のトピックを subscribe しない。`make_lite` が rosbag を読み込み、以下のトピックのみを lite bag に書き出す。

### 実機ログ（`TOPICS["real"]`）

| Topic name                                                                                              | 用途                                       |
| ------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| `/system/operation_mode/state`                                                                          | AUTONOMOUS 区間の切り出し                  |
| `/vehicle/status/velocity_status`                                                                       | 速度応答                                   |
| `/vehicle/status/steering_status`                                                                       | 操舵応答                                   |
| `/localization/kinematic_state`                                                                         | 自車位置・軌跡                             |
| `/localization/acceleration`                                                                            | 加速度応答                                 |
| `/control/command/control_cmd`                                                                          | 制御指令（post-gate）                      |
| `/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory`  | DiffusionPlanner 出力軌跡（シムと直接比較） |
| `/perception/object_recognition/tracking/objects`                                                       | 追跡物体（社会的コンテキストの確認）       |
| `/planning/trajectory`                                                                                  | 最終プランニング軌跡（optimizer 後段出力） |

### シミュレーションログ（`TOPICS["sim"]`、将来拡張用）

現状の評価ノードは sim 側 lite bag を生成しないが、`make_lite.py` の `TOPICS["sim"]` は以下を抽出する設計になっている。

| Topic name                                                                                              | 用途                                       |
| ------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| `/system/operation_mode/state`                                                                          | AUTONOMOUS 区間の切り出し                  |
| `/vehicle/status/velocity_status`                                                                       | 速度応答                                   |
| `/vehicle/status/steering_status`                                                                       | 操舵応答                                   |
| `/localization/kinematic_state`                                                                         | 自車位置・軌跡                             |
| `/localization/acceleration`                                                                            | 加速度応答                                 |
| `/control/trajectory_follower/control_cmd`                                                              | trajectory_follower の制御指令             |
| `/control/command/control_cmd`                                                                          | 制御指令（post-gate、実機との同段比較用）  |
| `/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory`  | DiffusionPlanner 出力軌跡（速度プロファイル分析用） |
| `/perception/traffic_light_recognition/traffic_signals`                                                 | 交通信号状態（DiffusionPlanner 入力）      |

## 評価結果の出力先

result_archive 配下に以下の成果物が生成される。

### `lite/real.lite/`

`make_lite` が抽出した実機 lite bag（rosbag2 mcap）。将来拡張時は `lite/sim_normal.lite/`, `lite/sim_godot.lite/` が併置される。

### `comparison/report.md`

Markdown 形式の比較レポート。以下の表を含む。

- 完走時間（AUTONOMOUS 開始～停止）
- 速度統計（`VelocityReport.longitudinal_velocity` の平均・最大・標準偏差）
- 速度 RMSE（指令 vs 応答）
- ステアリング RMSE（指令 vs 応答）
- 軌跡乖離（実機との最近傍距離。将来拡張時にシム軌跡を対象に算出）

### `comparison/figures/`

PNG / PDF の比較プロットが出力される。

| ファイル                           | 内容                                  |
| ---------------------------------- | ------------------------------------- |
| `velocity.{png,pdf}`               | 速度指令 vs 応答                      |
| `acceleration.{png,pdf}`           | 加速度指令 vs 応答                    |
| `steering.{png,pdf}`               | 操舵指令 vs 応答                      |
| `steer_response.{png,pdf}`         | ステアリング応答特性                  |
| `trajectory_with_map.{png,pdf}`    | 地図背景上での軌跡重ね合わせ          |
| `curves_closeup.{png,pdf}`         | カーブ別の詳細拡大                    |
| `curve2_analysis.{png,pdf}`        | カーブ② 全体解析                      |
| `curve2_steering_detail.{png,pdf}` | カーブ② 操舵詳細                      |
| `curve2_yaw_steer.{png,pdf}`       | カーブ② ヨーレート・操舵関係          |

### `comparison/curve2_per_step/`

`analyze_curve2_per_step` による per-step delta 解析の成果物（CSV・図）。ステップ 3 が失敗した場合は出力されない。

### `result_bag_path/`

post_process の `create_metadata_yaml` を通すためのプレースホルダ mcap が事前に書き込まれる。
本ユースケースは実走 bag を録らないが、後段の post_process が `result_bag_path` の存在を要求するための互換用ファイルである。

### `result.jsonl`

`output_dir/result.jsonl` に出力される。Evaluator はこの最終行を参照して成否を判定する。

```json
{"Result": {"Success": true, "Summary": "Success"}, "Stamp": {"System": 0.0}, "Frame": {}}
```

例外発生時は `Summary` に Python の traceback 文字列が格納される。

## simulation

本ユースケースは Autoware を起動しないため、`logging_simulator.launch` も `planning_simulator.launch` も使用しない。
launch は `add_use_case_arguments` と `launch_evaluator_node` のみを実行する。

評価ノードに渡される主な launch 引数:

- `t4_dataset_path`: 実機 rosbag を `input_bag/` 配下に含むデータセットルート
- `map_path`: `lanelet2_map.osm` を含む地図ディレクトリ（軌跡プロットの地図背景に使用。なければ地図なしで描画）
- `result_jsonl_path`, `result_archive_path`, `result_bag_path`, `scenario_path`: 共通の出力・入力パス

### 入力 rosbag に必要な要素

- ディレクトリ形式の rosbag2（db3 / mcap いずれも可）または単一 `.mcap` ファイルを `t4_dataset_path/input_bag/` 配下に配置する。
- 上掲の「[実機ログ（`TOPICS["real"]`）](#実機ログtopicsreal)」のトピックが録られていること。

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/real_log_sim_comparison/scenario.yaml)を参照。

### カーブ別解析設定

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/real_log_sim_comparison/curve_config_miraikan.yaml)を参照。

### 評価結果フォーマット

`result.jsonl` の最終行に以下の構造が記録される。

```json
{
  "Result": {"Success": <bool>, "Summary": <str>},
  "Stamp": {"System": 0.0},
  "Frame": {}
}
```

`Success` がパイプライン例外有無、`Summary` が `"Success"` または Python traceback 文字列。

## 将来拡張：シム側パイプライン

`sample/real_log_sim_comparison/scenario.yaml` の `ScenarioDescription` には `Real-log lite-fication + sim_normal + sim_godot + compare_logs analysis` と記載されており、`make_lite.py` の `TOPICS["sim"]` もシムログ抽出用に定義済みである。
ただし以下の理由により、現状は実機ログのみが処理対象である。

- `sample/real_log_sim_comparison/sim_scenario.yaml` は TODO コメントのみで内容が未充填。
- 評価ノード `real_log_sim_comparison_evaluator` は `make_lite --kind sim` を呼び出さず、`compare_logs` も sim lite bag が存在しないため実機側のみのプロットを生成する。

三方比較が有効化されると、`result_archive_path/lite/` 配下に `sim_normal.lite/` および `sim_godot.lite/` が追加生成され、`comparison/report.md` および `comparison/figures/` 配下のプロットに実機・sim_normal・sim_godot の三系統が現れる構成となる。
