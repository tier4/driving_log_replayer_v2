# 認識機能の False Positive 評価

Autoware の認識機能(perception)の認識結果から指定された領域(polygon)内に指定された点群または bbox が存在していないかを評価する。

Autoware の実行時には perception topic を保存する。その後、後処理の中で評価を行う。

## 事前準備

perception の[事前準備](/docs/use_case/perception.ja.md#事前準備)を参考

## 評価方法

launch を立ち上げると以下が順に実行され、評価される。

1. `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. rosbag から出力されたセンサーデータを Autoware が受け取り、perception モジュールが認識を行う
3. そこで出力された対象の topic を保存しておく
4. rosbag の再生が終了した後、保存した rosbag を1つのメッセージごとにparseさせ対象の topic を評価する

## 評価結果

pass/fail を判定する topic の subscribe 1 回につき、以下に記述する判定結果が出力される。

### 正常

- `non_detection_area`で指定した領域内に点群または bbox が非接触であること。(境界上の場合は非接触となる)
- また、`PassRate >= 正常数 / 全受信数 * 100`の条件を満たすとき、ResultのTotalがSuccessになる。

### 異常

正常の条件を満たさない場合

### 評価スキップ

以下の場合に、FrameSkipに1足す処理のみ行う。
FrameSkipは評価をskipした回数のカウンタ。

- 受信したobjectのfootprint.pointsの数が1か2の場合(この条件はperception_evalが更新されたらなくなる予定)

## 評価スクリプトが使用する Topic 名とデータ型

受信できるtopic型

| データ型                           |
| ---------------------------------- |
| sensor_msgs/msg/PointCloud2        |
| autoware_msgs/msg/DetectedObjects  |
| autoware_msgs/msg/TrackedObjects   |
| autoware_msgs/msg/PredictedObjects |

また、評価を通じて得られた結果を rosbag に書き込む。

| topic 名                                                  | データ型                           |
| --------------------------------------------------------- | ---------------------------------- |
| /driving_log_replayer_v2/perception_fp/results            | std_msgs/msg/String                |
| /driving_log_replayer_v2/perception_fp/non_detection_area | visualization_msgs/msg/MarkerArray |
| /driving_log_replayer_v2/perception_fp/fp_objects         | visualization_msgs/msg/MarkerArray |

## logging_simulator.launch に渡す引数

- localization: false
- planning: false
- control: false

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

t4_dataset で必要なトピックが含まれていること

車両の ECU の CAN と、使用している sensor の topic が必要
以下は例であり、違うセンサーを使っている場合は適宜読み替える。
CAMERA が複数ついている場合は、搭載されているすべての camera_info と image_rect_color_compressed を含める。

| topic 名                                             | データ型                        |
| ---------------------------------------------------- | ------------------------------- |
| /pacmod/from_can_bus                                 | can_msgs/msg/Frame              |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo      |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2     |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan      |
| /tf                                                  | tf2_msgs/msg/TFMessage          |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                                             | データ型                                       |
| ---------------------------------------------------- | ---------------------------------------------- |
| /pacmod/from_can_bus                                 | can_msgs/msg/Frame                             |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                     |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage                |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                    |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                     |
| /tf                                                  | tf2_msgs/msg/TFMessage                         |
| /vehicle/status/control_mode                         | autoware_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status                          | autoware_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status                      | autoware_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status               | autoware_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status                      | autoware_vehicle_msgs/msg/VelocityReport       |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_fp/scenario.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_fp/result.json)参照

perception では、シナリオに指定した条件で perception_eval が評価した結果を各 frame 毎に出力する。
全てのデータを流し終わったあとに、最終的なメトリクスを計算しているため、最終行だけ、他の行と形式が異なる。

以下に、各フレームのフォーマットとメトリクスのフォーマットを示す。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

各フレームのフォーマット

```json
{
  "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
  "Stamp": {
    "System": 123456789.123456,
    "ROS": 234567891.234567
  },
  "Frame": {
    "FrameSkip": "評価が飛ばされた回数の合計。",
    "criteria_name": {
      "PassFail": {
        "Result": {
          "Total": "Success or Fail",
          "Frame": "Success or Fail"
        },
        "Info": {}
      }
    },
    "criteria_name": {
      "PassFail": {
        "Result": {
          "Total": "Success or Fail",
          "Frame": "Success or Fail"
        },
        "Info": {}
      }
    },
    "criteria_name": {
      "Info": "Not in evaluation timestamp range"
    }
  }
}
```

警告のフォーマット

```json
{
  "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
  "Stamp": {
    "System": 123456789.123456,
    "ROS": 234567891.234567
  },
  "Frame": {
    "Warning": "警告のメッセージ",
    "FrameSkip": "評価が飛ばされた回数の合計。footprint.pointsの数が1か2の場合に発生する"
  }
}
```
