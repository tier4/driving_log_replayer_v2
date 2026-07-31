# NDT 自己位置推定の評価

ユースケース `localization` では NDT による Autoware の自己位置推定が安定して動作しているかを評価する。
Driving Log Replayer は Autoware の `logging_simulator` を用いてシナリオを再生し、シミュレーション中およびシミュレーション後に評価を行っている。
本ページはシミュレーションと評価がどのように行われているかを記述する。

## シミュレーション詳細

本セクションではシミュレーションの詳細や実行するために必要なものを記述する。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localization/scenario.ja.yaml)参照

### logging_simulator.launch に渡す引数

- perception: false
- planning: false
- control: false
- pose_source: ndt
- twist_source: gyro_odom

### 入力 rosbag に含まれるべき topic

入力 rosbag には Autoware の localization コンポーネントを動かすのに必要なトピックが含まれている必要がある。

ECU CAN のデータを用いる場合は以下のトピックを rosbag に含める必要がある。

| Topic name                         | Data type                                    |
| ---------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus               | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |

また、ECU の CAN を使わずに vehicle 系のトピックで代用しても良い

| Topic name                             | Data type                                      |
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

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

### localization_evaluator_node

Driving Log Replayer は `logging_simulator` と共に `localization_evaluator_node` を launch し、評価に必要なデータを収集している。

`localization_evaluator_node` は以下のトピックをサブスクライブする。

| Topic name                                                           | Data type                                         |
| -------------------------------------------------------------------- | ------------------------------------------------- |
| /diagnostics                                                         | diagnostic_msgs::msg::DiagnosticArray             |
| /localization/pose_estimator/transform_probability                   | autoware_internal_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/nearest_voxel_transformation_likelihood | autoware_internal_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/initial_to_result_relative_pose         | geometry_msgs::msg::PoseStamped                   |
| /localization/pose_estimator/exe_time_ms                             | autoware_internal_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/iteration_num                           | autoware_internal_debug_msgs::msg::Int32Stamped   |
| /tf                                                                  | tf2_msgs/msg/TFMessage                            |
| /localization/util/downsample/pointcloud                             | sensor_msgs::msg::PointCloud2                     |
| /localization/pose_estimator/points_aligned                          | sensor_msgs::msg::PointCloud2                     |

また、`localization_evaluator_node` は以下のトピックをパブリッシュする。

| Topic name                                             | Data type                      |
| ------------------------------------------------------ | ------------------------------ |
| /driving_log_replayer_v2/localization/lateral_distance | example_interfaces/msg/Float64 |

また、`localization_evaluator_node` は以下のサービスをコールする。

| Service name             | Data type                                                      |
| ------------------------ | -------------------------------------------------------------- |
| /localization/initialize | autoware_internal_localization_msgs/srv/InitializeLocalization |

---

## 評価内容詳細

具体的には以下を評価する。

- NDT の可用性
- NDT の収束性
- NDT の信頼度
- 参照軌跡（位置姿勢・速度・加速度）との乖離
- diagnostics のエラー率
- diagnostics の立ち上がり/立ち下がりタイミング

Driving Log Replayer を launch すると以下が実行され、評価される。

1. 評価ノード(`localization_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、自己位置推定を行う
3. 評価ノードが topic を subscribe して、NDT の信頼度、収束性、可用性が基準を満たしているかを判定して結果をファイルに記録する。
4. rosbag の再生が終了すると `logging_simulator` も終了し、集計された結果ファイルを用いて参照軌跡との乖離、diagnostics のエラー率や立ち上がり/立ち下がりタイミングを [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) を用いて評価する。
5. 上記評価結果の出力後、自動的にプログラムが終了する。

全評価項目が Success と判定されたとき、本評価は Success と判定される。

以下は各評価項目の評価方法の詳細である。

### NDT の可用性

全ての localization シナリオおいて本評価項目は基本的に評価されるが、シナリオファイルの `Conditions` に `availability` が定義されておりかつ `enable` が `false` と設定されたときのみ実行されない。

NDTの可用性を評価するために、下記のようなケースを `logging_simulator` 中に検知する。

- Runtime error等により `pointcloud_preprocessor` が落ちている（これにより、 `ndt_scan_matcher` へのLiDARスキャンが送信されなくなる）
- Runtime error等により `ndt_scan_matcher` が落ちている

そのために下記の出力が定期的に出力されているかどうかを、Component State MonitorというAutoware内のパッケージを間接的に利用することによって評価する。

- /localization/pose_estimator/exe_time_ms

なお、NDTの出力トピックの中で `/localization/pose_estimator/exe_time_ms` が選ばれたのは、「トピックが定期的に出力されている」ことを確認しやすいからである。例えば `/localization/pose_estimator/pose` は NVTL や TP などのスコアが低い場合も出力されないので、出力を監視するだけでは `ndt_scan_matcher` の可用性を判定することが難しく本目的には適さない。

`logging_simulator` の最後まで `/localization/pose_estimator/exe_time_ms` トピックが確認できれば本評価は Success、確認できなかれば Fail と判定される。

### NDT の収束性

本評価項目はシナリオファイルの `Conditions` に `Convergence` が定義されているとき評価される。

```yaml
Evaluation:
  Conditions:
    Convergence:
      AllowableDistance: 0.2 # Lateral distance to be considered convergence
      AllowableExeTimeMs: 100.0 # If the NDT computation time is less than or equal to this value, it is considered successful.
      AllowableIterationNum: 30 # If the number of NDT calculations is less than or equal to this value, it is considered a success.
      PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
```

以下 3 つの topic を `logging_simulator` 中に評価する。

| Topic name                                                   | Success condition                            |
| ------------------------------------------------------------ | -------------------------------------------- |
| /localization/pose_estimator/initial_to_result_relative_pose | value of lateral factor <= AllowableDistance |
| /localization/pose_estimator/exe_time_ms                     | value <= AllowableExeTimeMs                  |
| /localization/pose_estimator/iteration_num                   | value <= AllowableIterationNum               |

あるフレームにおいて、上記の 3 条件が全て満たされた場合に NDT が収束したとして判定される。シミュレーションを通じてこの収束率が `PassRate` 以上であれば本評価は Success、`PassRate` 未満であれば Fail と判定される。

### NDT の信頼度

本評価項目はシナリオファイルの `Conditions` に `Reliability` が定義されているとき評価される。

```yaml
Evaluation:
  Conditions:
    Reliability:
      Method: NVTL # NVTL or TP which method to use for evaluation
      AllowableLikelihood: 2.3 # If above this value, the localization reliability value is considered normal.
      NGCount: 10 # If the reliability value is lower than the threshold value for more than this number in the sequence. the evaluation is considered to have failed.
```

以下の 2 つの topic のうち、シナリオで指定した方を `logging_simulator` 中に評価する。

| Method | Topic Name                                                           | Success Condition            |
| ------ | -------------------------------------------------------------------- | ---------------------------- |
| TP     | /localization/pose_estimator/transform_probability                   | value >= AllowableLikelihood |
| NVTL   | /localization/pose_estimator/nearest_voxel_transformation_likelihood | value >= AllowableLikelihood |

上記トピックが示す数値が `AllowableLikelihood` 以上であれば NDT の結果は信頼できるものとして評価し、下回っていれば異常とみなして評価する。シミュレーションを通じて TP もしくは NVTL が `NGCount` 回連続で閾値を下回って場合は Fail、そうでない場合は Success と判定される。

### 参照軌跡との乖離

全ての localization シナリオおいて、本評価項目は基本的に評価される。

`logging_simulator` 終了後 `autoware_localization_evaluation_scripts` を通じて、位置姿勢・速度・加速度の参照軌跡と実際のシミュレーション結果に乖離がないかを評価する。具体的な乖離の定義など、詳細な処理は [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) を参照すること。

評価できる軌跡の要素は

- 位置
- 姿勢
- 進行方向の速度
- 角速度
- 進行方向の加速度

の 5 点であり、シナリオファイル中の `OverallCriteriaMask` を定義することで評価をするかしないかを決定できる。（`mean_relative_*` の項目）もしも `OverallCriteriaMask` の定義がない場合は全ての要素に対して評価を行う。全ての評価項目で乖離がなかった場合は Success と判定され、そうでない場合は Fail と判定される。

```yaml
Evaluation:
  Conditions:
    OverallCriteriaMask: # Toggle the mask below to perform or not to perform evaluation of the according criteria. The evaluator will automatically set all to `true` if this block is not defined.
      mean_relative_position: true
      mean_relative_angle: true
      mean_relative_linear_velocity: true
      mean_relative_angular_velocity: true
      mean_relative_acceleration: true
      diagnostics_not_ok_rate: true
```

### diagnostics のエラー率

全ての localization シナリオおいて、本評価項目は基本的に評価される。

`logging_simulator` 終了後 `autoware_localization_evaluation_scripts` を通じて、localization に関連する diagnostics が想定以上の ERROR を出していないかを評価する。具体的には以下の名前の diagnostics について評価する。

- ndt_scan_matcher: scan_matching_status
- [ndt_scan_matcher: scan_matching_status](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/README.md#scan_matching_status)
- [localization: ekf_localizer](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ekf_localizer/README.md#diagnostics): pose_no_update_count[^*]
- [localization: ekf_localizer](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ekf_localizer/README.md#diagnostics): twist_no_update_count[^*]
- [localization: ekf_localizer](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ekf_localizer/README.md#diagnostics): cov_ellipse_long_axis_size[^*]
- [localization: ekf_localizer](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ekf_localizer/README.md#diagnostics): cov_ellipse_lateral_direction_size[^*]
- [localization_error_monitor: ellipse_error_status](https://github.com/autowarefoundation/autoware_universe/blob/main/localization/autoware_localization_error_monitor/README.md#purpose)
- [localization: pose_instability_detector](https://github.com/autowarefoundation/autoware_universe/blob/main/localization/autoware_pose_instability_detector/README.md#autoware_pose_instability_detector)

[^*]: この4つは旧バージョンでは"localization: ekf_localizer"に統合されている。

詳細な処理は [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) の README を参照すること。

全ての diagnostics において、ERROR 率が十分低かった場合は Success と判定され、そうでない場合は Fail と判定される。

また、シナリオファイル中の `OverallCriteriaMask` の定義によっては本評価の無効化ができる。（`diagnostics_not_ok_rate` の項目）もしも `OverallCriteriaMask` の定義がない場合はデフォルトで評価を行う。

```yaml
Evaluation:
  Conditions:
    OverallCriteriaMask: # Toggle the mask below to perform or not to perform evaluation of the according criteria. The evaluator will automatically set all to `true` if this block is not defined.
      mean_relative_position: true
      mean_relative_angle: true
      mean_relative_linear_velocity: true
      mean_relative_angular_velocity: true
      mean_relative_acceleration: true
      diagnostics_not_ok_rate: true
```

### diagnostics の立ち上がり/立ち下がりタイミング

本評価項目はシナリオファイルの `Conditions` に `DiagnosticsFlagCheck` が定義されているとき評価される。

```yaml
Evaluation:
  Conditions:
    DiagnosticsFlagCheck:
      pose_is_passed_delay_gate: # name of the diagnostics to check
        flag: rise # which flag to detect (`rise` or `fall`)
        at_sec: 113 # The `sec` part of the expected time to rise/fall
        at_nanosec: 750000000 # The `nanosec` part of the expected time to rise/fall
      pose_no_update_count:
        flag: rise
        at_sec: 117
        at_nanosec: 900000000
```

`logging_simulator` 終了後 `autoware_localization_evaluation_scripts` を通じて、localization に関連する diagnostics が想定タイミングで立ち上がるもしくは立ち下がるかどうかを評価する。詳細な処理は [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) の README を参照すること。

diagnostics の指定の項目が `at_sec` と `at_nanosec` で定義される想定時間の +/- 0.2 秒の間に立ち上がった/立ち下がった場合に Success と判定され、そうでない場合は Fail と判定される。想定時間よりも早くに立ち上がり/立ち下がりがあった場合にも Fail と判定される。

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localization/result.json)参照

以下に、それぞれの評価の例を記述する。

可用性の結果(Frame の中に Availability 項目がある場合)

```json
{
  "Availability": {
    "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Warn" },
    "Info": {}
  }
}
```

収束性の結果(Frame の中に Convergence 項目がある場合)

```json
{
  "Convergence": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
    "Info": {
      "LateralDistance": "initial_to_result_relative_pose.pose.position.y",
      "HorizontalDistance": "initial_to_result_relative_pose.pose.positionの水平距離。参考値",
      "ExeTimeMs": "ndtの計算にかかった時間",
      "IterationNum": "ndtの再計算回数"
    }
  }
}
```

信頼度の結果(Frame に Reliability の項目がある場合)

```json
{
  "Reliability": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
    "Info": {
      "Value": {
        "stamp": {
          "sec": "stampの秒",
          "nanosec": "stampのnano秒"
        },
        "data": "NVTL or TPの値"
      },
      "Reference": {
        "stamp": {
          "sec": "stampの秒",
          "nanosec": "stampのnano秒"
        },
        "data": "評価に使用しなかった尤度。参考値。ValueがNVTLならTPが入る"
      }
    }
  }
}
```

NDTの可用性・収束性・信頼度の最終結果および参照軌跡との乖離、diagnostics のエラー率、diagnostics の立ち上がり/立ち下がりタイミングの結果はまとめて result.json の最終フレームに記載される。

```json
{
  "Result": {
    "Success": false,
    "Summary": "Failed: Convergence (Fail): 570 / 632 -> 90.19%, Reliability (Fail): NVTL Sequential NG Count: 10 (Total Test: 632, Average: 2.46835, StdDev: 0.16043), NDT Availability (Success): NDT available, mean_position_norm=0.166 [m]|mean_angle_norm=0.032 [deg]|mean_linear_velocity_norm=0.003 [m/s]|mean_angular_velocity_norm=0.001 [rad/s]|localization__ekf_localizer 13.447 [%] is too large.|localization__pose_instability_detector 0.000 [%]|localization_error_monitor__ellipse_error_status 13.447 [%] is too large.|ndt_scan_matcher__scan_matching_status 57.177 [%] is too large.|Diagnostics flag 'pose_is_passed_delay_gate' OK.|Diagnostics flag 'pose_no_update_count' OK."
  }
}
```
