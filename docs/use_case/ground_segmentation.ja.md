# 地面点群除去の評価

入力点群に対して地面点群をセグメンテーションし、除去する機能について評価する。

## Ground Truthデータ

評価のために必要となるGround Truthデータは以下の2種類の方法で与えることが可能であり、それぞれシナリオの`Evaluation.Conditions.Method`を変更することにより使用できる。

### annotated_rosbag

bagデータに含まれる点群データに、セマンティックラベルを表すフィールドを持たせる方法。

地面点群除去前後の topic を同期 subscribe し、地面ラベルを持つ点数の比較により精度評価を行う。

本評価基盤では、セマンティックラベルは`INT32`型の`entity_id`フィールドに記述されていることが前提となっている。

### annotated_pcd

データセットとして与える点群データ(`~/driving_log_replayer_v2/ground_segmentation/dataset/data/LIDAR_CONCAT/*.pnd.bin`)に、セマンティックラベルを表すフィールドを持たせる方法。

地面点群除去処理後の点群と、pcd.binファイルに含まれる点群同士を比較し、処理後点群が持つラベルを見ることで精度評価を行う。

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`ground_segmentation_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、perceptionモジュール内で地面点群除去を行う
3. 評価ノードが topic を subscribe して、地面点群の除去精度などについて評価し結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

### 評価時の注意点

- **annotated_rosbagモード**  
   [autoware.universeのsensingモジュール](https://github.com/autowarefoundation/autoware.universe/blob/main/sensing/autoware_pointcloud_preprocessor/src/filter.cpp#L386-L394)を以下のように書き換える必要がある。

  ```diff
    if (utils::is_data_layout_compatible_with_point_xyzi(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy "
        "code/data");
    }

  - return;
  + //return;
  }
  ```

- **annotated_pcdモード**  
   評価処理に時間がかかるため、rosbagの再生レートを下げる必要がある。
  `ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_file} play_rate:=0.1`

## 評価結果

topic の subscribe 1 回につき、以下に記述する判定結果が出力される。

### 正常

評価により得られたAccuracyがシナリオに記述されている`Evaluation.Conditions.accuracy_min`以上の場合、正常とする。

### 異常

正常の条件を満たさないとき、異常とする。

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| topic 名                                                  | データ型                    |
| --------------------------------------------------------- | --------------------------- |
| /sensing/lidar/concatenated/pointcloud 　　               | sensor_msgs/msg/PointCloud2 |
| /perception/obstacle_segmentation/single_frame/pointcloud | sensor_msgs/msg/PointCloud2 |

**注:`/perception/obstacle_segmentation/single_frame/pointcloud`topicは、launch引数`evaluation_target_topic`で変更可能である。**

Published topics:

| topic 名 | データ型 |
| -------- | -------- |
| -        | -        |

## logging_simulator.launch に渡す引数

- localization: false
- planning: false
- control: false
- perception_mode: lidar

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

| topic 名                               | データ型                    |
| -------------------------------------- | --------------------------- |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2 |
| /tf                                    | tf2_msgs/msg/TFMessage      |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/ground_segmentation/scenario.ja.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/ground_segmentation/result.json)参照

ground_segmentation では、Accuracy、Precision、Recall、Specificity、F1-scoreを評価した結果を各 frame 毎に出力する。

以下に、評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

```json
{
  "GroundSegmentation": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
    "Info": {
      "TP": "地面点として認識された地面点の数",
      "FP": "地面点として認識された障害物点の数",
      "TN": "障害物点として認識された障害物点の数",
      "FN": "障害物点として認識された地面点の数",
      "Accuracy": "Accuracyの値",
      "Precision": "Precisionの値",
      "Recall": "Recallの値",
      "Specificity": "Specificitの値",
      "F1-score": "F1-scoreの値"
    }
  }
}
```
