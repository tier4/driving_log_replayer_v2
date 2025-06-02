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

`/control/autonomous_emergency_braking/metrics`と`/control/control_evaluator/metrics`を利用する。
`/control/autonomous_emergency_braking/metrics`がシナリオで指定されたvalueになっているかを評価する。
シナリオでレーン条件を記述した場合は、`/control/control_evaluator/metrics`から取得できるレーンが条件を満たした場合に評価される。
評価の条件を満たさない場合は、ログも出力されない。

`/planning/planning_factors/**`のtopicを利用する。評価対象のtopicはシナリオファイルでtopic名を指定する。
PlanningFactorのcontrol_pointの位置がシナリオに指定された条件を満たすかを評価する。

### Metric正常

`/control/control_evaluator/metrics`のvalueがシナリオ指定の値と一致した場合に正常となる。
ただし、`none`が指定された場合は、topicのmetric_arrayが空配列の場合にnoneと判断する。
kinematic_conditionを指定した場合は追加で、kinematic_stateが条件を満たしている必要がある。

### Metric正常異常

Metric正常の条件を満たさないとき

### PlanningFactor正常(judgement: positive)

`/planning/planning_factors/**`のcontrol_points[0].poseのx,yの位置がシナリオで指定したx,y座標からrangeの範囲に入っている場合に正常となる。

### PlanningFactor正常(judgement: negative)

`/planning/planning_factors/**`のcontrol_points[0].poseのx,yの位置がシナリオで指定したx,y座標からrangeの範囲に入っていない場合に正常となる。

### PlanningFactor異常

PlanningFactor正常の条件を満たさないとき

## 評価結果の出力先ファイル

planning_controlにおいては、以下の3つのファイルにそれぞれresult.jsonlが作成される。
result.jsonlは必ず出力されるが、planning_factor_result.jsonlとdiag_result.jsonlはシナリオで指定した場合にのみ出力される

### result.jsonl

output_dir/result.jsonlに出力される。
metricの評価結果が記述される。

Evaluatorで実行する場合は、このファイルの最終行が参照されて成否が決定される。
このため、planning_factor_result.jsonlとdiag_result.jsonlの結果をマージした最終的な成否の情報がpost_processで書き込まれる。

## planning_factor_result.jsonl

output_dir/result_archive/planning_factor_result.jsonlに出力される。
planning_factorの評価結果が記述される。

## diag_result.jsonl

output_dir/result_archive/diag_result.jsonlに出力される。
diagnosticsの評価結果が記述される。

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name                                    | Data type                                               |
| --------------------------------------------- | ------------------------------------------------------- |
| /control/control_evaluator/metrics            | tier4_metric_msg/msg/MetricArray                        |
| /control/autonomous_emergency_braking/metrics | tier4_metric_msg/msg/DiagnosticArray                    |
| /planning/planning_factors/\*\*               | autoware_internal_planning_msgs/msg/PlanningFactorArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## logging_simulator.launch に渡す引数

- localization: false

bagの中に入っている、/sensing/lidar/concatenated/pointcloudを利用する場合は、launchの引数にsensing:=falseを追加する
perception、planningも同様にbagから出力する場合は、launchの引数にperception:=false planning:=falseを追加する

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${planning_control_scenario_path} sensing:=false perception:=false planning:=false
```

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

| topic 名                               | データ型                                     |
| -------------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus                   | can_msgs/msg/Frame                           |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                        |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                  |
| /tf                                    | tf2_msgs/msg/TFMessage                       |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute      |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                               | データ型                                            |
| -------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                               |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped        |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                         |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute             |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/scenario.yaml)参照

### 評価結果フォーマット

#### metric

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/result.json)参照

以下に、それぞれの評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

planning と controlで設定した全ての評価条件で成功している場合に成功と判定される。

```json
{
  "Frame": {
    "[Planning|Control]_CONDITION_INDEX": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "TotalPassed": "評価条件をパスしたtopicの総数",
        "Decision": "取得したtopicのdecision",
        "LaneInfo": "[lane_id, s, t]",
        "KinematicState": "[vel, acc, jerk]"
      }
    }
  }
}
```

#### planning_factor

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/planning_factor_result.json)参照

以下に、それぞれの評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

PlanningFactorのすべての評価条件で成功している場合に成功と判定される。

```json
{
  "Frame": {
    "TopicName": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "Distance": "control_pointの座標とシナリオに指定された座標の距離",
        "ControlPointPoseX": "control_pointのposeのx座標",
        "ControlPointPoseY": "control_pointのposeのy座標"
      }
    }
  }
}
```

#### diagnostics

diagnosticsのユースケースと同じ
