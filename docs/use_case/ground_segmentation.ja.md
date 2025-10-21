# 地面点群除去の評価

入力点群に対して地面点群をセグメンテーションし、除去する機能について評価する。

## Ground Truthデータ

評価のために必要となるGround Truthデータは以下の方法で与えられる．

### annotated_pcd

3D Semantic Segmentationのアノテーションが含まれる t4_datasetを用いる方法。([format](https://github.com/tier4/tier4_perception_dataset/blob/main/docs/t4_format_3d_detailed.md#3d-lidarseg-annotation-format-in-t4-format))

1. 地面除去された点群と t4_dataset内の`/sensing/lidar/concatenated/pointcloud`に相当する点群(`dataset/data/LIDAR_CONCAT/*.pcd.bin`)を最近傍探索でマッチングさせる
2. その点群が地面であるのか障害物であるのかチェックする

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launchコマンド `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、perceptionモジュール内で地面点群除去を行う
3. 出力トピックを保存用rosbagに保存する
4. rosbagの再生が終わった後、メッセージ一つ一つをパースしてターゲットとなるトピックを評価する

### 評価時の注意点

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

| topic 名                                     | データ型                    |
| -------------------------------------------- | --------------------------- |
| /sensing/lidar/concatenated/pointcloud 　　  | sensor_msgs/msg/PointCloud2 |
| /perception/obstacle_segmentation/pointcloud | sensor_msgs/msg/PointCloud2 |

**注:`/perception/obstacle_segmentation/pointcloud`topicは、launch引数`evaluation_target_topic`で変更可能である。**

Published topics:

| topic 名 | データ型 |
| -------- | -------- |
| -        | -        |

## logging_simulator.launch に渡す引数

- localization: false
- planning: false
- control: false
- sensing: false
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
      "Specificity": "Specificityの値",
      "F1-score": "F1-scoreの値"
    }
  }
}
```
