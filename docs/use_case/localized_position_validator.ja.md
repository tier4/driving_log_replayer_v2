# Localized position validator の評価

Localized position validator の動作を評価する。
想定としては、学習したモデルの重みの変更等に伴ってデグレ等が起きていないかを確認する。

## Annotation データ

評価のために必要となる Annotation (Ground Truth) データは以下のように scenario.yaml 経由で与えられる．

### Annotation のフォーマット

シナリオの中の `Evaluation` の下に以下のような形で指定する。

```yaml
Evaluation:
  Annotation:
    - Timestamp: 1649138854.971764
      ExpectedValidity: true
    - Timestamp: 1649138915.570511
      ExpectedValidity: false
    - ...
```

時系列順にソートされた後に、 Timestamp の時刻から次の Timestamp の時刻前まで ExpectedValidity が正解のラベルとして設定される。  
次の Timestamp がない場合は、終了までそのラベルが設定される。

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`localized_position_validator_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを Autoware が受け取って、自己位置推定を行う
3. 評価ノードが topic を subscribe して、 localized position validator の入力と出力の数、ラベルと予測との正答率が基準を満たしているかを判定して結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

### 評価時の注意点

シミュレーションの結果次第では予測の Timestamp が前後する恐れがある。  
ラベルのスイッチがある場合（前述の `Annotation のフォーマット` のような true/false が入れ替わる場合）、 正答率にノイズが入る可能性がある。

## 評価結果

localized position validator の出力 topic の subscribe 1 回につき、以下に記述する判定結果が出力される。  
また、 logging simulator 終了後に Annotation と予測との比較が行われ、前者の最終的な判定結果と合わせて最終的な判定結果を出力する。

### 正常

入力に対する出力トピックの数がシナリオに記述した `Evaluation.Conditions.CheckInputAndOutputCount.PassRate` 以上の場合、かつ予測の正答率がシナリオに記述した `Evaluation.Conditions.CheckPrediction.PassRate` 以上の場合。

### 異常

正常の条件を満たさないとき、異常とする。

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| topic 名                                                     | データ型                                                           |
| ------------------------------------------------------------ | ------------------------------------------------------------------ |
| /localization/pose_estimator/points_aligned                  | sensor_msgs::msg::PointCloud2                                      |
| /localization/localized_position_validator/validation_result | tier4_localization_msgs::msg::LocalizedPositionValidatorPrediction |

## logging_simulator.launch に渡す引数

- perception: false
- planning: false
- control: false
- pose_source: ndt
- twist_source: gyro_odom

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

車両の ECU の CAN と、使用している sensor の topic が必要。
以下は例であり、違うセンサーを使っている場合は適宜読み替える。

LiDAR が複数ついている場合は、搭載されているすべての LiDAR の packets を含める。

| topic 名                           | データ型                                     |
| ---------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus               | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                               | データ型                                       |
| -------------------------------------- | ---------------------------------------------- |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped   |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                      |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                          |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                            |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                     |
| /vehicle/status/control_mode           | autoware_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_vehicle_msgs/msg/VelocityReport       |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない。

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localized_position_validator/scenario.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localized_position_validator/result.json)参照

`CheckInputAndOutputCount` の最終的な結果とその後に行われる Annotation と予測（`Prediction`）との比較結果の２つをパスしていると result は true でそれ以外は false となる。

以下に、評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

```json
{
  "CheckInputAndOutputCount": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
    "Prediction": {
      "is_position_valid": "localized position validator の予測 (true / false)"
    },
    "Info": {
      "InputCount": "入力トピックの数",
      "OutputCount": "出力トピックの数"
    }
  }
}
```
