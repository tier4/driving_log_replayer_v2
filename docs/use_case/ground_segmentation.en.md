# Evaluate Ground Segmentation

Evaluate the performance of the Ground Segmentation sub-component in Autoware, which is responsible for segmenting and removing ground points from the input point cloud.

## Ground Truth data

The Ground Truth data required for evaluation can be provided using the following two methods, and each can be used by changing the `Evaluation.Conditions.Method` of the scenario.

### annotated_pcd

This method involves using the point cloud data provided as a dataset (`dataset/data/LIDAR_CONCAT/\*.pcd.bin`) together with the corresponding label data (`dataset/lidarseg/\*.bin`).

Compare the point cloud after ground removal with the point cloud in the `pcd.bin` file, and evaluate accuracy by examining the labels associated with the points in the processed point cloud.

## Evaluation method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`ground_segmentation_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data input from previously prepared rosbag and performs ground point cloud removal within the perception module.
3. Evaluation node subscribes to Autoware's output topics, evaluates the accuracy of ground point cloud removal, and dumps the results to a file
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### Points to note during evaluation

- **annotated_pcd mode**
  Since the evaluation process takes time, the playback rate of the rosbag needs to be reduced.
  Example:
  `ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_file} play_rate:=0.1`

## Evaluation result

The results are calculated for each subscription. The format and available states are described below.

### Normal

If the Accuracy obtained through the evaluation meets or exceeds the Evaluation.Conditions.accuracy_min specified in the scenario, it is assumed to be normal.

### Error

When the normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| topic name                                   | Data type                   |
| -------------------------------------------- | --------------------------- |
| /sensing/lidar/concatenated/pointcloud 　　  | sensor_msgs/msg/PointCloud2 |
| /perception/obstacle_segmentation/pointcloud | sensor_msgs/msg/PointCloud2 |

**NOTE: the`/perception/obstacle_segmentation/pointcloud`topic can be modified by changing the `evaluation_target_topic` launch argument.**

Published topics:

| topic name | Data type |
| ---------- | --------- |
| -          | -         |

## Arguments passed to logging_simulator.launch

- localization: false
- planning: false
- control: false
- sensing: false
- perception_mode: lidar

## simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

| topic name                             | Data type                   |
| -------------------------------------- | --------------------------- |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2 |
| /tf                                    | tf2_msgs/msg/TFMessage      |

### Topics that must not be included in the input rosbag

| topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/ground_segmentation/scenario.ja.yaml)

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/ground_segmentation/result.json)

In ground segmentation, the evaluation results for Accuracy, Precision, Recall, Specificity, and F1-score are output for each frame.

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

```json
{
  "GroundSegmentation": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
    "Info": {
      "TP": "The number of ground points recognized as ground",
      "FP": "The number of obstacle points recognized as ground",
      "TN": "The number of obstacle points recognized as obstacle",
      "FN": "The number of ground points recognized as obstacle",
      "Accuracy": "Accuracy value",
      "Precision": "Precision value",
      "Recall": "Recall value",
      "Specificity": "Specificity value",
      "F1-score": "F1-score value`"
    }
  }
}
```
