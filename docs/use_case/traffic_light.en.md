# Evaluate perception(traffic light)

The performance of Autoware's recognition function (perception) is evaluated by calculating mAP (mean Average Precision) and other indices from the recognition results.

Run the perception module and pass the output perception topic to the evaluation library for evaluation.

Currently, only the evaluation of `classification2d` is supported.

## Preparation

In perception evaluation, machine-learning pre-trained models are used.
If the model is not prepared in advance, Autoware will not output recognition results.
If no evaluation results are produced, check to see if this has been done correctly.

### Downloading Model Files

Models are downloaded during Autoware setup.
The method of downloading models depends on the version of Autoware you are using, so check which method is used.
The following patterns exist.

#### Download with ansible

When you run the ansible setup script, you will see `Download artifacts? [y/N]`, type `y` and press enter (Autoware foundation's main branch use this method)
<https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/tasks/main.yaml>

#### Automatically downloaded when the package is built

If you are using a slightly older Autoware.universe, this is the one to use, until the commit hash of `13b96ad3c636389b32fea3a47dfb7cfb7813cadc`.
[traffic_light_classifier/CMakeList.txt](https://github.com/autowarefoundation/autoware.universe/blob/13b96ad3c636389b32fea3a47dfb7cfb7813cadc/perception/traffic_light_classifier/CMakeLists.txt#L113-L119)

### Conversion of model files

The downloaded onnx file is not to be used as-is, but to be converted to a TensorRT engine file for use.
A conversion command is available, so source the autoware workspace and execute the command.

Let's assume that autoware is installed in `$HOME/autoware`.

```shell
source $HOME/autoware/install/setup.bash
ros2 launch traffic_light_classifier traffic_light_classifier.launch.xml use_gpu:=true  build_only:=true
ros2 launch traffic_light_fine_detector traffic_light_fine_detector.launch.xml build_only:=true
```

When the conversion command finishes, the engine file is output.
The output destination changes according to the model download method, so check that the output is in the appropriate directory.

#### Download with ansible

The following file is output.

```shell
$HOME/autoware_data/traffic_light_classifier/traffic_light_classifier_mobilenetv2_batch_6.fp16-batch6.engine
$HOME/autoware_data/traffic_light_fine_detector/tlr_yolox_s_batch_6.fp16-batch6.engine
```

#### Automatic download at package build time

The following file is output.

```shell
$HOME/autoware/install/traffic_light_classifier/share/traffic_light_classifier/data/traffic_light_classifier_mobilenetv2_batch_6.fp16-batch6.engine
$HOME/autoware/install/traffic_light_fine_detector/share/traffic_light_fine_detector/data/tlr_yolox_s_batch_6.fp16-batch6.engine
```

### (For evaluation on a single PC) rewrite parameters in the launch file

set the value of `traffic_light_recognition/fusion_only` `true` in the file `autoware.universe/launch/tier4_perception_launch/launch/perception.launch.xml`
<https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_perception_launch/launch/perception.launch.xml#L79>

In the main branch of Autoware Foundation's Autoware, it is set to `false`, but in the case of Autoware used in actual vehicles, it may be set to `true`.
Because `true` is a setting that recognition results are sent from another computer, when evaluating with a single PC, it should be set back to `false` before execution.

## Evaluation method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`traffic_light_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and outputs camera, and the perception module performs recognition.
3. The evaluation node subscribes to `/perception/traffic_light_recognition/traffic_signals` and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation results

The results are calculated for each subscription. The format and available states are described below.

### Perception Normal

When the following conditions are satisfied by executing the evaluation function of perception_eval

1. frame_result.pass_fail_result contains at least one object (`tp_object_results ! = [] and fp_object_results ! = [] and fn_objects ! = []`)
2. no object fail (`frame_result.pass_fail_result.get_fail_object_num() == 0`)

### Perception Error

The perception evaluation output is marked as `Error` when condition for `Normal` is not met.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                            | Data type                                    |
| ----------------------------------------------------- | -------------------------------------------- |
| /perception/traffic_light_recognition/traffic_signals | tier4_perception_msgs/msg/TrafficSignalArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| -          | -         |

## Arguments passed to logging_simulator.launch

- localization: false
- planning: false
- control: false

**NOTE: The `tf` in the bag is used to align the localization during annotation and simulation. Therefore, localization is invalid.**

## Dependent libraries

The perception evaluation step bases on the [perception_eval](https://github.com/tier4/autoware_perception_evaluation) library.

### Division of roles of driving_log_replayer_v2 with dependent libraries

`driving_log_replayer_v2` package is in charge of the connection with ROS. The actual perception evaluation is conducted in [perception_eval](https://github.com/tier4/autoware_perception_evaluation) library.
The [perception_eval](https://github.com/tier4/autoware_perception_evaluation) is a ROS-independent library, it cannot receive ROS objects. Also, ROS timestamps use nanoseconds while the `t4_dataset` format is based on milliseconds (because it uses `nuScenes`), so the values must be properly converted before using the library's functions.

`driving_log_replayer_v2` subscribes the topic output from the perception module of Autoware, converts it to the data format defined in [perception_eval](https://github.com/tier4/autoware_perception_evaluation), and passes it on.
It is also responsible for publishing and visualizing the evaluation results from [perception_eval](https://github.com/tier4/autoware_perception_evaluation) on proper ROS topic.

[perception_eval](https://github.com/tier4/autoware_perception_evaluation) is in charge of the part that compares the detection results passed from `driving_log_replayer_v2` with ground truth data, calculates the index, and outputs the results.

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in `t4_dataset` format.

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.
The following example shows the topic list available in evaluation input rosbag when multiple LiDARs and Cameras are used in a real-world vehicle configuration.

/sensing/lidar/concatenated/pointcloud is used when sensing:=false is added to the launch argument.

If there is more than one CAMERA, include all on-board camera_info and image_rect_color_compressed.

| Topic name                                           | Data type                                    |
| ---------------------------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus                                 | can_msgs/msg/Frame                           |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                   |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage              |
| /sensing/camera/traffic_light/camera_info            | sensor_msgs/msg/CameraInfo                   |
| /sensing/camera/traffic_light/image_raw/compressed   | sensor_msgs/msg/CompressedImage              |
| /sensing/gnss/ublox/fix_velocity                     | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix                      | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt                           | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw                        | sensor_msgs/msg/Imu                          |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                  |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                   |
| /tf                                                  | tf2_msgs/msg/TFMessage                       |

The vehicle topics can be included instead of CAN.

| Topic name                                           | Data type                                      |
| ---------------------------------------------------- | ---------------------------------------------- |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                     |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage                |
| /sensing/camera/traffic_light/camera_info            | sensor_msgs/msg/CameraInfo                     |
| /sensing/camera/traffic_light/image_raw/compressed   | sensor_msgs/msg/CompressedImage                |
| /sensing/gnss/ublox/fix_velocity                     | geometry_msgs/msg/TwistWithCovarianceStamped   |
| /sensing/gnss/ublox/nav_sat_fix                      | sensor_msgs/msg/NavSatFix                      |
| /sensing/gnss/ublox/navpvt                           | ublox_msgs/msg/NavPVT                          |
| /sensing/imu/tamagawa/imu_raw                        | sensor_msgs/msg/Imu                            |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                    |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                     |
| /tf                                                  | tf2_msgs/msg/TFMessage                         |
| /vehicle/status/control_mode                         | autoware_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status                          | autoware_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status                      | autoware_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status               | autoware_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status                      | autoware_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

There are two types of evaluation: use case evaluation and database evaluation.
Use case evaluation is performed on a single dataset, while database evaluation uses multiple datasets and takes the average of the results for each dataset.

In the database evaluation, the `vehicle_id` should be able to be set for each data set, since the calibration values may change.
Also, it is necessary to set whether or not to activate the sensing module.

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/traffic_light/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/traffic_light/result.json).

The evaluation results by [perception_eval](https://github.com/tier4/autoware_perception_evaluation) under the conditions specified in the scenario are output for each frame.
Only the final line has a different format from the other lines since the final metrics are calculated after all data has been flushed.

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Format of each frame:

```json
{
  "Frame": {
    "FrameName": "Frame number of t4_dataset used for evaluation",
    "FrameSkip": "Number of times that an object was requested to be evaluated but the evaluation was skipped because there was no ground truth in the dataset within 75msec",
    "PassFail": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "TP": "Number of TPs",
        "FP": "Number of FPs",
        "FN": "Number of FNs"
      }
    }
  }
}
```

Metrics Data Format:

```json
{
  "Frame": {
    "FinalScore": {
      "Score": {
        "TP": {
          "ALL": "TP rate for all labels",
          "label0": "TP rate of label0",
          "label1": "TP rate of label1"
        },
        "FP": {
          "ALL": "FP rate for all labels",
          "label0": "FP rate of label0",
          "label1": "FP rate of label1"
        },
        "FN": {
          "ALL": "FN rate for all labels",
          "label0": "FN rate of label0",
          "label1": "FN rate of label1"
        },
        "TN": {
          "ALL": "TN rate for all labels",
          "label0": "TN rate of label0",
          "label1": "TN rate of label1"
        },
        "Accuracy": {
          "ALL": "Accuracy for all labels",
          "label0": "Accuracy of label0",
          "label1": "Accuracy of label1"
        },
        "Precision": {
          "ALL": "Precision for all labels",
          "label0": "Precision of label0",
          "label1": "Precision of label1"
        },
        "Recall": {
          "ALL": "Recall for all labels",
          "label0": "Recall of label0",
          "label1": "Recall of label1"
        },
        "F1score": {
          "ALL": "F1score for all labels",
          "label0": "F1score of label0",
          "label1": "F1score of label1"
        }
      },
      "ConfusionMatrix": {
        "label0(GroundTruth)": {
          "label0(Prediction)": "value",
          "label1(Prediction)": "value"
        },
        "label1(GroundTruth)": {
          "label0(Prediction)": "value",
          "label1(Prediction)": "value"
        }
      }
    }
  }
}
```

### pickle file

In database evaluation, it is necessary to replay multiple rosbags, but due to the ROS specification, it is impossible to use multiple bags in a single launch.
Since one rosbag, i.e., one `t4_dataset`, requires one launch, it is necessary to execute as many launches as the number of datasets contained in the database evaluation.

Since database evaluation cannot be done in a single launch, perception outputs a file `scene_result.pkl` in addition to `result.jsonl` file.
A pickle file is a python object saved as a file, PerceptionEvaluationManager.frame_results of [perception_eval](https://github.com/tier4/autoware_perception_evaluation).
The dataset evaluation can be performed by reading all the objects recorded in the pickle file and outputting the index of the dataset's average.

### Result file of database evaluation

In the case of a database evaluation with multiple datasets in the scenario, a file named `database_result.json` is output to the results directory.

The format is the same as the [Metrics Data Format](#evaluation-result-format).
