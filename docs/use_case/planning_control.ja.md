# Planning Controlの評価

Metricsが指定の条件で出力されているか評価する

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`planning_control_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、metrics型のメッセージを出力する
3. 評価ノードが topic を subscribe して、各基準を満たしているかを判定して結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

## 評価結果

`/control/autonomous_emergency_braking/metrics`と`/control/control_evaluator/metrics`を利用する。
`/control/autonomous_emergency_braking/metrics`がシナリオで指定されたvalueになっているかを評価する。
シナリオでレーン条件を記述した場合は、`/control/control_evaluator/metrics`から取得できるレーンが条件を満たした場合に評価される。
評価の条件を満たさない場合は、ログも出力されない。

### 正常

`/control/control_evaluator/metrics`のvalueがシナリオ指定の値と一致した場合に正常となる。
ただし、`none`が指定された場合は、topicのmetric_arrayが空配列の場合にnoneと判断する。
kinematic_conditionを指定した場合は追加で、kinematic_stateが条件を満たしている必要がある。

### 異常

正常の条件を満たさないとき

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name                                    | Data type                              |
| --------------------------------------------- | -------------------------------------- |
| /control/control_evaluator/metrics            | tier4_metric_msg::msg::MetricArray     |
| /control/autonomous_emergency_braking/metrics | tier4_metric_msg::msg::DiagnosticArray |

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
