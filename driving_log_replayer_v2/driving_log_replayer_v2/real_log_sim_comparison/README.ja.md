# Real Log / Sim Comparisonの評価

実機走行ログ (T4 dataset の `input_bag/` rosbag) を **SSOT** とし、同じルートを
Autoware + シミュレータで closed-loop 再現して、実機との乖離を統計値と図で比較する
ユースケース。速度応答・操舵応答・軌跡精度・per-step delta を可視化する。

## ファイル構成 (本ディレクトリに全集約)

リポジトリの他ユースケースとは異なり、real_log_sim_comparison は本ディレクトリ配下に
ソース・evaluator_node・Makefile・sample・README を全部集約している。

| パス (本ディレクトリからの相対) | 内容 |
|---|---|
| `evaluator_node.py` | パイプラインを orchestrate する ROS2 ノード。`lib/driving_log_replayer_v2/real_log_sim_comparison_evaluator_node.py` に install される (CMakeLists で `RENAME` 互換) |
| `Makefile` | `make local_cloud_run` でローカル実行 (詳細は `sample/local/README.ja.md`) |
| `step1_make_lite.py` / `step2_bag_to_scenario.py` / `step3_run_sims.py` / `step4_compare_logs.py` / `step5_analyze_per_step.py` / `step6_analyze_cases.py` | 6 段階パイプラインの各 stage 実装 (ファイル名先頭の `stepN_` が実行順) |
| `lib/_*.py` | 共有ユーティリティ・内部設定 (cases_config / sim_runs_config / runtime_config) と utility (io, events, map, params)。stage 実装から `from .lib._x import` で参照 |
| `tools/*.py` | パイプライン外の手動診断・解析ツール (compare_curve2 / compare_dp_trajectory / diagnose_curve2 / diagnose_departure / analyze_real_curve2)。`python3 -m ...real_log_sim_comparison.tools.<name>` で単体実行 |
| `sample/cloud/` | cloud Web.Auto evaluator 向けサンプル (`scenario.yaml`, `cases.yaml`, `sim_runs.yaml`) |
| `sample/local/` | ローカル実行向けサンプル + 手順 README (`sample/local/README.ja.md`) |
| `sample/curve_config_miraikan.yaml` | カーブ別解析設定 (cloud / local 共通参照。サンプルでは scenario.yaml で参照しない) |

> **build 後の install パス**
> CMakeLists が `share/driving_log_replayer_v2/sample/real_log_sim_comparison{,_local}/`
> に sample を install する (cloud Web.Auto evaluator 互換のため旧パスを維持)。

サンプルシナリオ: `sample/cloud/scenario.yaml`。`Datasets[0]` の UUID が
**SSOT (実機 rosbag を含む annotation-dataset)** となり、6 段階すべての入口になる。

## 評価方法

本ユースケースは他のユースケースと異なり、launch から **Autoware・bag player・bag recorder のいずれも起動しない**。
評価ノード `real_log_sim_comparison_evaluator` が以下の 6 段階パイプラインを subprocess で直接実行する。

| Stage | 名称 | 入力 | 出力 (`result_archive/` 配下) | 実行回数 |
|---|---|---|---|---|
| 1 | 実機ログ抽出 (`step1_make_lite --kind real`) | `input_bag/*.{mcap,db3}` | `lite/real.lite/` | 1 |
| 2 | scenario 自動生成 (`step2_bag_to_scenario`) | `input_bag/` + map | `scenarios/auto_scenario.yaml` (OpenSCENARIO) | 1 |
| 3 | closed-loop シム実行 (`step3_run_sims`) | `auto_scenario.yaml` + `sim_runs.yaml` の 1 run | `lite/<run_tag>.lite/` | N (sim_runs) |
| 4 | 実機 + sim 比較解析 (`step4_compare_logs`) | `lite/{real, <run_tag>}.lite/` 群 | `comparison/{figures/, report.md}` (N-way 重ね描き) | 1 |
| 5 | VehicleModel per-step 解析 (`step5_analyze_per_step`) | `lite/real.lite/` + `cases.yaml` の 1 ケース | `comparison/per_step/<tag>/` | N (cases) |
| 6 | ケース集約解析 (`step6_analyze_cases`) | `per_step/<tag>/per_step_delta.csv` 群 | `comparison/cases/{overlay/, cases_summary.md}` | 1 |

成否判定はパイプラインの例外有無で決まる。
全 stage が完走すれば `result.jsonl` に `Success: true` が記録され、いずれかの subprocess が非ゼロ終了またはタイムアウトすると `Success: false` と Python traceback が記録される。

## シナリオ設定（Conditions）

シナリオ YAML の `Evaluation.Conditions` で以下のキーを認識する。

| キー                 | 必須/任意 | 内容                                                                                                  |
| -------------------- | --- | ----------------------------------------------------------------------------------------------------- |
| `scenario_name`      | 任意 | 図タイトルに表示するシナリオ名。未指定時は `ScenarioName` を使用。                                    |
| `curve_config_yaml`  | 任意 | カーブ別解析設定 YAML への相対パス (`scenario.yaml` 基準) または絶対パス。空文字でカーブ別解析をスキップ。 |
| `cases_config`       | **必須** | Stage 5/6 (VehicleModel per-step 解析 + 集約) の `cases.yaml` への相対パス。 |
| `sim_runs_config`    | **必須** | Stage 3/4 (closed-loop sim + N-way 比較) の `sim_runs.yaml` への相対パス。 |

`cases.yaml` / `sim_runs.yaml` の書式と使える `vehicle_model` の種別は
`sample/local/README.ja.md` を参照。

`curve_config_yaml` は以下の構造で記述する (`sample/curve_config_miraikan.yaml` 参照)。

- `curve_centers`: 地図座標系でのカーブ中心リスト (`label`, `cx`, `cy`, `margin`)。`curves_closeup` プロット用。
- `curve2_index`: `curve_centers` 内で per-step delta 解析の対象とするカーブのインデックス (0 始まり)。
- `curve2_window`: カーブ② 直前の一時停止を検出する時刻窓 `[start, end]` (AUTONOMOUS 開始からの経過秒)。

## 評価対象トピック（抽出対象）

評価ノードは ROS のトピックを subscribe しない。`step1_make_lite` が rosbag を読み込み、以下のトピックのみを lite bag に書き出す。

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

### シミュレーションログ（`TOPICS["sim"]`）

Stage 3 (`step3_run_sims`) が `scenario_test_runner` で sim を回した結果の rosbag から、
`step1_make_lite.py` の `TOPICS["sim"]` で以下を抽出して `lite/<run_tag>.lite/` を生成する。

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

`step1_make_lite` が抽出した実機 lite bag（rosbag2 mcap）。将来拡張時は `lite/sim_normal.lite/`, `lite/sim_godot.lite/` が併置される。

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
| `trajectory_with_map.{png,pdf}`    | 地図背景上での軌跡重ね合わせ          |
| `curves_closeup.{png,pdf}`         | カーブ別の詳細拡大                    |
| `curve{N}_analysis.{png,pdf}`        | 指定カーブ 全体解析（{N}=`plot_curves[*].index + 1`）   |
| `curve{N}_steering_detail.{png,pdf}` | 指定カーブ 操舵詳細                                     |
| `curve{N}_yaw_steer.{png,pdf}`       | 指定カーブ ヨーレート・操舵関係                         |
| `curve{N}_steer_response.{png,pdf}`  | 指定カーブ ステアリング応答特性                         |

> `curve{N}_*` 系は `curve_config.yaml::plot_curves` で対象カーブを切り替え可能。<br>
> 未指定なら `curve2_index` のカーブだけが生成される（既定は `curve2_*` の 4 枚）。

### `comparison/per_step/<case_tag>/`

`step5_analyze_per_step` (Stage 5) によるケース別 per-step delta 解析の成果物。
1 ケースあたり CSV (`per_step_delta.csv`) + 図 8 枚 + `summary.txt` を生成。
ケースは `cases.yaml` で定義する (Stage 5/6 の入力)。

### `comparison/cases/`

`step6_analyze_cases` (Stage 6) による全ケース集約解析の成果物。

| ファイル | 内容 |
|---|---|
| `cases_summary.md` | tag × (RMSE err_steer / err_ds_long / err_ds_lat) の Markdown 表 |
| `overlay/cascade_error_overlay.png` | 全ケースを 1 枚に重ね描き (段階的誤差) |
| `overlay/error_timeseries_overlay.png` | 全ケースを 1 枚に重ね描き (誤差時系列) |

---

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

`sample/cloud/scenario.yaml` を参照。

### カーブ別解析設定

`sample/curve_config_miraikan.yaml` を参照 (現状サンプルは scenario.yaml で参照しない)。

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

## ローカル実行

webauto で T4 dataset を pull 済みの環境で `make local_cloud_run` 一発で 6 段階すべて
走る。手順詳細・トラブルシュート・Makefile 変数の上書き例は
[`sample/local/README.ja.md`](sample/local/README.ja.md) を参照。
