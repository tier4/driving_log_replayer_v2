# Perception Reproducerの評価

rosbagから再現された感知データを用いた閉ループ条件下で、指定されたルートを完走できるか評価する。
perception_reproducerノードは現在の自車位置に基づいてrosbagから感知オブジェクトを再現し、閉ループテストを可能にする。

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`perception_reproducer_evaluator_node`)と `planning_simulator.launch`、`perception_reproducer`ノードを立ち上げる
2. `perception_reproducer`ノードは現在の自車位置に基づいてrosbagから感知オブジェクトを読み取り、パブリッシュする
3. AutowareがDRIVING状態（ENGAGE）に入ると車両が走行を開始する
4. 評価ノードが topic を subscribe して、各基準を満たしているかを判定して結果をファイルに記録する
5. すべてのpass条件が満たされるか、いずれかのfail条件がトリガーされると自動で launch が終了して評価が終了する

## 評価結果

### Pass条件

`pass_conditions`内のすべての条件グループが一度満たされると、テストが成功して終了する。

#### ego_kinematic_trigger

自車両が指定されたエリアに到達したかを評価する。

- シナリオにarea条件がある場合、自車両のx,y位置がシナリオで指定したx,y座標からrangeの範囲に入っている。
- シナリオにvelocity条件がある場合、自車両の速度がシナリオで指定した範囲に入っている。
- シナリオにacceleration条件がある場合、自車両の加速度がシナリオで指定した範囲に入っている。

#### time_wait_trigger

指定された待機時間が経過したかを評価する。

- アクティベーションからの経過時間が`wait_seconds`に達すると条件が満たされる。

#### condition_group

ネストされた条件グループがサポートされている。グループは`start_at`と`end_at`で設定でき、いつアクティブになるかを制御できる。

### Fail条件

`fail_conditions`内のいずれかの条件グループが一度満たされないと、テストが失敗し、`terminated_after_fail_s`秒後に終了する。

#### metric

メトリクスを評価する。planning_controlのユースケースと同じ。

#### diagnostic

診断ステータスを評価する。diagnosticsのユースケースと同じ。

#### planning_factor

計画因子を評価する。planning_controlのユースケースと同じ。

#### ego_kinematic

アクティブ期間中、自車両の運動学条件（位置、速度、加速度）を継続的に評価する。

- シナリオにarea条件がある場合、自車両のx,y位置がシナリオで指定したx,y座標からrangeの範囲内/外にある。
- シナリオにvelocity条件がある場合、自車両の速度がシナリオで指定した範囲に入っている。
- シナリオにacceleration条件がある場合、自車両の加速度がシナリオで指定した範囲に入っている。

#### condition_group

ネストされた条件グループがサポートされている。グループは`start_at`と`end_at`で設定でき、いつアクティブになるかを制御できる。

## 評価結果の出力先ファイル

perception_reproducerにおいては、以下の3つのファイルにそれぞれresult.jsonlが作成される。
result.jsonlは必ず出力されるが、pass_result.jsonlとfail_result.jsonlはシナリオで指定した場合にのみ出力される

### result.jsonl

output_dir/result.jsonlに出力される。
passとfail評価からまとめされた評価結果が記述される。

Evaluatorで実行する場合は、このファイルの最終行が参照されて成否が決定される。
このため、pass_result.jsonlとfail_result.jsonlの結果をマージした最終的な成否の情報がpost_processで書き込まれる。

### pass_result.jsonl

output_dir/result_archive/pass_result.jsonlに出力される。
pass条件の評価結果が記述される。

### fail_result.jsonl

output_dir/result_archive/fail_result.jsonlに出力される。
fail条件の評価結果が記述される。

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name                           | Data type                                               |
| ------------------------------------ | ------------------------------------------------------- |
| /localization/kinematic_state        | nav_msgs/msg/Odometry                                   |
| /localization/acceleration           | geometry_msgs/msg/AccelWithCovarianceStamped            |
| /control/control_evaluator/metrics   | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_evaluator/metrics | tier4_metric_msg/msg/MetricArray                        |
| /system/processing_time/metrics      | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_factors/\*\*      | autoware_internal_planning_msgs/msg/PlanningFactorArray |
| /diagnostics                         | diagnostic_msgs/msg/DiagnosticArray                     |
| /autoware/state                      | autoware_system_msgs/msg/AutowareState                  |

Published topics:

| Topic name                                               | Data type           |
| -------------------------------------------------------- | ------------------- |
| /driving_log_replayer/perception_reproducer/results      | std_msgs/msg/String |
| /driving_log_replayer/perception_reproducer/pass_results | std_msgs/msg/String |
| /driving_log_replayer/perception_reproducer/fail_results | std_msgs/msg/String |

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれてはいけない topic

[perception_reproducer](https://github.com/autowarefoundation/autoware_tools/tree/main/planning/planning_debug_tools#perception-reproducer)を参照。

### Planning Simulator

このユースケースは、車両が実際に走行できる閉ループテストを可能にするため、`logging_simulator.launch`の代わりに`planning_simulator.launch`を使用する。

### Perception Reproducerノード

`planning_debug_tools`パッケージの`perception_reproducer`ノードは、現在の自車位置に基づいてrosbagから感知オブジェクトを再現する。
詳細については、[perception_reproducer](https://github.com/autowarefoundation/autoware_tools/tree/main/planning/planning_debug_tools#perception-reproducer)を参照。

設定オプション：

- `noise`: 感知オブジェクトにノイズを適用する
- `reproduce_cool_down`: 再パブリッシュのクールダウン時間（デフォルト: 80.0）
- `tracked_object`: 追跡オブジェクトをパブリッシュする
- `search_radius`: rosbagの自車オドメトリメッセージを検索する検索半径（デフォルト: 1.5）

### ルート設定方法

走行ルートはシナリオYAMLのデータセットレベルにある `route_method` フィールドで設定できる。
`route_method` が `play_route_from_rosbag` に設定されている場合、`--pub-route` フラグが `perception_reproducer` ノードに渡される。

`route_method` の詳細については[シナリオフォーマット](../scenario_format/index.ja.md#route_method)を参照。

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_reproducer/scenario.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_reproducer/result.jsonl)参照
