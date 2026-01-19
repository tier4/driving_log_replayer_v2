# Planning Controlの評価

Metrics、PlanningFactorsが指定の条件で出力されているか評価する。
include_use_caseでdiagnosticsを指定すれば、diagnosticsの評価も可能。

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`planning_control_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、metrics型、PlanningFactor型のメッセージを出力する
3. 評価ノードが topic を subscribe して、各基準を満たしているかを判定して結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

## 評価結果

### Metric

[Metric.msg](https://github.com/autowarefoundation/autoware_internal_msgs/blob/main/autoware_internal_metric_msgs/msg/Metric.msg)が利用されているtopicを利用する。
主に、`/control/control_evaluator/metrics`, `/planning/planning_evaluator/metrics`, `/system/processing_time/metrics`を想定している。
評価対象となるtopic内の`name`は`metric_name`にて指定する。
以下条件を評価できる。

- 指定メトリクスがシナリオ指定の範囲内か
- 指定メトリクスがシナリオ指定の値となるか

#### Metric正常(judgement: positive)

`value_type=number`の場合に、メトリクスTopic中の指定metricが`value_range`の範囲に入ってると正常となる。
`value_type=string`の場合に、メトリクスTopic中の指定metricが`value_target`と一致すると正常となる。

#### Metric正常(judgement: negative)

`value_type=number`の場合に、メトリクスTopic中の指定metricが`value_range`の範囲外であると正常となる。
`value_type=string`の場合に、メトリクスTopic中の指定metricが`value_target`と一致しないと正常となる。

#### Metric正常異常

Metric正常の条件を満たさないとき

### PlanningFactor

`/planning/planning_factors/**`のtopicを利用する。評価対象のtopicはシナリオファイルでtopic名を指定する。
以下の条件を評価できる。

- PlanningFactorのcontrol_pointの位置がシナリオに指定された条件を満たすか
- PlanningFactorのbehaviorが指定のbehaviorになってるか

#### PlanningFactor正常(judgement: positive)

`/planning/planning_factors/**`が以下の条件を全部満たす場合に正常となる。

- シナリオにarea条件がある場合、control_points[0].poseのx,yの位置がシナリオで指定したx,y座標からrangeの範囲に入っている。
- シナリオにbehavior条件がある場合、planning_factorのbehaviorがシナリオで指定したbehaviorにある。
- シナリオにdistance条件がある場合、planning_factorのdistance(Egoからcontrol_pointまでの距離)がシナリオで指定した範囲に入っている。
- シナリオにvelocity条件がある場合、planning_factorのvelocity(control_pointでの速度)がシナリオで指定した範囲に入っている。
- シナリオにtime_to_wall条件がある場合、planning_factorのtime_to_wall(現在の速度でcontrol_pointに到達するまでの時間)がシナリオで指定した範囲に入っている。
- シナリオにacceleration_to_wall条件がある場合、planning_factorのacceleration_to_wall(現在の速度でcontrol_pointに到達するために必要な加速度)がシナリオで指定した範囲に入っている。

#### PlanningFactor正常(judgement: negative)

`/planning/planning_factors/**`が以下の任意条件を満たさない場合に正常となる。

- シナリオにarea条件がある場合、control_points[0].poseのx,yの位置がシナリオで指定したx,y座標からrangeの範囲に入っている。
- シナリオにbehavior条件がある場合、planning_factorのbehaviorがシナリオで指定したbehaviorにある。
- シナリオにdistance条件がある場合、planning_factorのdistance(Egoからcontrol_pointまでの距離)がシナリオで指定した範囲に入っている。
- シナリオにvelocity条件がある場合、planning_factorのvelocity(control_pointでの速度)がシナリオで指定した範囲に入っている。
- シナリオにtime_to_wall条件がある場合、planning_factorのtime_to_wall(現在の速度でcontrol_pointに到達するまでの時間)がシナリオで指定した範囲に入っている。
- シナリオにacceleration_to_wall条件がある場合、planning_factorのacceleration_to_wall(現在の速度でcontrol_pointに到達するために必要な加速度)がシナリオで指定した範囲に入っている。

#### PlanningFactor異常

PlanningFactor正常の条件を満たさないとき

## 評価結果の出力先ファイル

planning_controlにおいては、以下の3つのファイルにそれぞれresult.jsonlが作成される。
result.jsonlは必ず出力されるが、planning_factor_result.jsonlとmetric_result.jsonlとdiag_result.jsonlはシナリオで指定した場合にのみ出力される

### result.jsonl

output_dir/result.jsonlに出力される。
planning_factorとmetricとdiag評価からまとめされた評価結果が記述される。

Evaluatorで実行する場合は、このファイルの最終行が参照されて成否が決定される。
このため、planning_factor_result.jsonlとmetric_result.jsonlとdiag_result.jsonlの結果をマージした最終的な成否の情報がpost_processで書き込まれる。

## planning_factor_result.jsonl

output_dir/result_archive/planning_factor_result.jsonlに出力される。
planning_factorの評価結果が記述される。

## metric_result.jsonl

output_dir/result_archive/metric_result.jsonlに出力される。
metricsの評価結果が記述される。

## diag_result.jsonl

output_dir/result_archive/diag_result.jsonlに出力される。
diagnosticsの評価結果が記述される。

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name                           | Data type                                               |
| ------------------------------------ | ------------------------------------------------------- |
| /control/control_evaluator/metrics   | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_evaluator/metrics | tier4_metric_msg/msg/MetricArray                        |
| /system/processing_time/metrics      | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_factors/\*\*      | autoware_internal_planning_msgs/msg/PlanningFactorArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/scenario.yaml)参照　#TODO

### 評価結果フォーマット

#### metric

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/result.json)参照　#TODO

#### planning_factor

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/planning_factor_result.json)参照 #TODO

#### diagnostics

diagnosticsのユースケースと同じ
