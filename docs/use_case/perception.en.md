# Evaluate perception

The performance of Autoware's recognition function (perception) is evaluated by calculating mAP (mean Average Precision) and other indices from the recognition results.

The perception topic is saved when Autoware is executed. The evaluation is then performed during post-processing.

The topic for pass/fail is based on the evaluation_task described in scenario.yaml. The topic to be analyzed can be specified from terminal arguments. If not specified, the default value is used.

In addition, you can use planning_factor on planning_control. Please see [this documents](/docs/use_case/planning_control.en.md).

## Preparation

In perception evaluation, machine learning pre-trained models are used.
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
[lidar_centerpoint/CMakeList.txt](https://github.com/autowarefoundation/autoware.universe/blob/13b96ad3c636389b32fea3a47dfb7cfb7813cadc/perception/lidar_centerpoint/CMakeLists.txt#L112-L118)

### Conversion of model files

The downloaded onnx file is not to be used as-is, but to be converted to a TensorRT engine file for use.
A conversion command is available, so source the autoware workspace and execute the command.

Let's assume that autoware is installed in `$HOME/autoware`.

```shell
source $HOME/autoware/install/setup.bash
ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml build_only:=true
```

When the conversion command finishes, the engine file is output.
The output destination changes according to the model download method, so check that the output is in the appropriate directory.

#### Download with ansible

An example of the use of autowarefoundation's autoware.universe is shown below.

The following file is output.

```shell
$HOME/autoware_data/lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.engine
$HOME/autoware_data/lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.engine
```

#### Automatic download at package build time

The following file is output.

```shell
$HOME/autoware/install/lidar_centerpoint/share/lidar_centerpoint/data/pts_backbone_neck_head_centerpoint_tiny.engine
$HOME/autoware/install/lidar_centerpoint/share/lidar_centerpoint/data/pts_voxel_encoder_centerpoint_tiny.engine
```

## Evaluation method

First, complete the setup procedure described in [Setup Instructions](/docs/quick_start/setup.en.md).

Once the setup is finished, user can start the perception evaluation using the sample rosbag provided at `~/driving_log_replayer_v2/sample_dataset`. with the command:

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
    scenario_path:=$HOME/driving_log_replayer_v2/perception.yaml \
    sensing:=false
```

or

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
    scenario_path:=$HOME/driving_log_replayer_v2/perception.yaml \
    remap_arg:="/sensing/lidar/top/velodyne_packets,/sensing/lidar/left/velodyne_packets,/sensing/lidar/right/velodyne_packets"
```

> [!NOTE]  
> sample rosbag includes packets to produce pointcloud and /sensing/lidar/concatenated/pointcloud. So it is necessary to either remap or not activate `sensing` to avoid topic duplication.

This command will perform the following steps:

1. launch the commands `logging_simulator.launch` and `ros2 bag play`
2. Autoware receives the sensor data output from the rosbag and the perception module recognizes it
3. Record the output topics in the bag file
4. After rosbag playback is finished, parse the saved rosbag one message at a time and evaluate the target topic.

## Evaluation results

The results are calculated for each subscription to judge pass/fail. The format and available states are described below.

### Perception Normal

Satisfy Criteria in the Criterion tag of the scenario.

The scenario.yaml of the sample is as follows,

```yaml
Criterion:
  - PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
    CriteriaMethod: num_gt_tp # refer https://github.com/tier4/driving_log_replayer_v2/blob/develop/driving_log_replayer_v2/driving_log_replayer_v2/criteria/perception.py#L136-L152
    CriteriaLevel: hard # Level of criteria (perfect/hard/normal/easy, or custom value 0.0-100.0)
    Filter:
      Distance: 0.0-50.0 # [m] null [Do not filter by distance] or lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is 1.7976931348623157e+308]
  - PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
    CriteriaMethod: num_gt_tp # refer https://github.com/tier4/driving_log_replayer_v2/blob/develop/driving_log_replayer_v2/driving_log_replayer_v2/criteria/perception.py#L136-L152
    CriteriaLevel: easy # Level of criteria (perfect/hard/normal/easy, or custom value 0.0-100.0)
    Filter:
      Distance: 50.0- # [m] null [Do not filter by distance] or lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is 1.7976931348623157e+308]
```

- For each subscription of topic to judge pass/fail, the number of objects in tp is hard (75.0%) or more for objects at a distance of 0.0-50.0[m]. Frame of Result becomes Success.
- For one subscription of topic to judge pass/fail, the number of objects in tp is easy (25.0%) or more for objects at a distance of 50.0-1.7976931348623157e+308[m]. Frame of Result becomes Success.
- If the condition `PassRate >= Normal / Total Received * 100` is satisfied, the Total of Result becomes Success.

### Perception Error

The perception evaluation output is marked as `Error` when condition for `Normal` is not met.

### Skipping evaluation

Only add 1 to FrameSkip in the following cases.
FrameSkip is a counter for the number of times evaluation is skipped.

- No Ground Truth exists within 75msec before or after the received object's header time. The nearest Ground Truth frame is used, the Ground Truth is not interpolated to the header time.
- If the number of footprint.points of the received object is 1 or 2
- If the predicted paths of the received object exceed the shape set by `prediction_num_modes` / `prediction_num_timesteps` (prediction only)

### Skipping evaluation(NoGTNoObject)

- When the Ground Truth and the recognition objects are filtered by the filter condition and not evaluated (when the content of the evaluation result PassFail object is empty).

### ignore_frames

Frames specified in `ignore_frames` are excluded from evaluation and are not included in the analysis results. However, they are added to the `frame results` (i.e., they remain in the `scene_result.t4eval` recording). They are counted in `FrameSkip` and reported in result.jsonl as `{"Info": {"Reason": "IGNORED_FRAME"}}`.

`ignore_frames` can be set in the scenario (`Evaluation.ignore_frames`) or as a launch argument, which has the higher priority. The value is a comma-separated list of the following tokens.

| Token     | Meaning                                                                        | Example   |
| --------- | ------------------------------------------------------------------------------ | --------- |
| `N`       | Frame whose t4_dataset frame index (`FrameName`) is N.                         | `3`       |
| `A-B`     | Frames whose t4_dataset frame index is between A and B (both included).        | `0-4`     |
| `first:N` | First N evaluated frames, by position in the sequence of the evaluated frames. | `first:2` |
| `last:N`  | Last N evaluated frames, by position in the sequence of the evaluated frames.  | `last:1`  |

e.g. `ignore_frames:="0-4,10,first:1,last:2"`

`N` and `A-B` are matched against the frame index of the dataset, while `first:N` and `last:N` are matched against the position of the frame in the sequence of the frames which would be evaluated, so they can be used without knowing the frame indices of the dataset. They are typically used to drop the head and the tail of the scene where the ego vehicle or the perception module is not settled yet.

`last:N` cannot be decided while the rosbag is read, so the writing of the last N evaluated frames is delayed until the end of the rosbag, and they are then written as ignored frames. The frames ignored by `last:N` are removed from the frame results before the metrics, the analysis and the coverage are computed.

## Topic name and data type used by evaluation script

The topic to determine pass/fail is based on the evaluation_task defined in scenario.yaml.

| evaluation_task | Data type                                    |
| --------------- | -------------------------------------------- |
| detection       | autoware_perception_msgs/msg/DetectedObjects |
| tracking        | autoware_perception_msgs/msg/TrackedObjects  |
| prediction      | TBD                                          |
| fp_validation   | autoware_perception_msgs/msg/DetectedObjects |

The topic to be analyzed, independent of pass/fail, can be defined with the terminal argument USE_CASE_ARGS.

| Arguments                            | Data type                                    |
| ------------------------------------ | -------------------------------------------- |
| evaluation_detection_topic_regex     | autoware_perception_msgs/msg/DetectedObjects |
| evaluation_tracking_topic_regex      | autoware_perception_msgs/msg/TrackedObjects  |
| evaluation_prediction_topic_regex    | TBD                                          |
| evaluation_fp_validation_topic_regex | autoware_perception_msgs/msg/DetectedObjects |

The results obtained through the evaluation are also written in rosbag.

| Topic name                                   | Data type                          |
| -------------------------------------------- | ---------------------------------- |
| /driving_log_replayer_v2/marker/ground_truth | visualization_msgs/msg/MarkerArray |
| /driving_log_replayer_v2/marker/results      | visualization_msgs/msg/MarkerArray |

## Arguments passed to logging_simulator.launch

- localization: false
- planning: false
- control: false

**NOTE: The `tf` in the bag is used to align the localization during annotation and simulation. Therefore, localization is invalid.**

## Dependent libraries

The perception evaluation step bases on the [t4perceval](https://github.com/ktro2828/t4perceval) library.
It is installed with pip from `requirements/<distro>.txt` when the package is built, together with `t4-devkit`, which loads the `t4_dataset`.

### Division of roles of driving_log_replayer_v2 with dependent libraries

`driving_log_replayer_v2` package is in charge of the part of the relationship with ROS, the object filters of the scenario and the part that determines pass/fail. The matching of objects and the metrics are computed by [t4perceval](https://github.com/ktro2828/t4perceval).
[t4perceval](https://github.com/ktro2828/t4perceval) is a ROS-independent library which stores objects as columns and evaluates them with systems. The adapter in `driving_log_replayer_v2/perception/t4perceval_adapter` converts the Autoware object messages into its archetypes, loads the `t4_dataset` with the Autoware label table of `perception_eval`, builds the filter and matching systems from the scenario, and reads the results back.

`driving_log_replayer_v2` subscribes the topic output from the perception module of Autoware, converts it, and evaluates every message against the nearest Ground Truth frame.
It is also responsible for publishing and visualizing the evaluation results on proper ROS topic.

The following settings of `evaluation_config_dict` have no effect with [t4perceval](https://github.com/ktro2828/t4perceval) and only log a warning: `label_prefix`, `count_label_number`, `matching_class_agnostic_fps`. The object filters apply one threshold to every label: a per-label list such as `min_point_numbers`, `max_x_position_list` or `confidence_threshold_list` must repeat the same value, and when the global and the critical settings bound the same quantity an object passes if it satisfies either (the looser one is used). `matching_label_policy` supports only `default` and `allow_any`; `allow_unknown`, `allow_same_group` and `allow_matching_unknown: true` are rejected. `max_matchable_radii` is applied as the `max_matchable_distance` of the t4perceval matchers, the maximum center distance of a match. The optional keys `prediction_num_modes` (default 10), `prediction_num_timesteps` (default 40) and `future_seconds` (default 8.0) fix the shape of the predicted paths for the prediction task.

Note that `fp_validation` is not available in this use case anymore, use the `perception_fp` use case instead.

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in `t4_dataset` format.

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.

If more than one CAMERA is attached, all camera_info and image_rect_color_compressed should be included.
In addition, /sensing/lidar/concatenated/pointcloud is remapped to avoid duplication depending on true or false of sensing.

| Topic name                                           | Data type                       |
| ---------------------------------------------------- | ------------------------------- |
| /pacmod/from_can_bus                                 | can_msgs/msg/Frame              |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo      |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2     |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan      |
| /tf                                                  | tf2_msgs/msg/TFMessage          |

The vehicle topics can be included instead of CAN.

| Topic name                                           | Data type                                      |
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

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception/result.json).

The evaluation results by [t4perceval](https://github.com/ktro2828/t4perceval) under the conditions specified in the scenario are output for each frame.
Only the final line has a different format from the other lines since the final metrics are calculated after all data has been flushed.

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Format of each frame:

```json
{
  "Frame": {
    "FrameName": "Frame number of t4_dataset used for evaluation",
    "FrameSkip": "The total number of times the evaluation was skipped, which occurs when the evaluation of an object is requested but there is no Ground Truth in the dataset within 75msec, or when the number of footprint.points is 1 or 2.",
    "criteria0": {
      // result of criteria 0, If the Ground Truth and recognition objects exist
      "PassFail": {
        "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
        "Info": {
          "TP": "Number of filtered objects determined to be TP",
          "FP": "Number of filtered objects determined to be FP",
          "FN": "Number of filtered objects determined to be FN"
        },
        "Objects": {
          // Evaluated objects information. See the [json schema](../../driving_log_replayer_v2/config/perception/object_output_schema.json) for details.
        }
      }
    },
    "criteria1": {
      // result of criteria 1. If the Ground Truth and the recognition objects do not exist
      "NoGTNoObj": "Number of times that the Ground Truth and the recognition objects were filtered and could not be evaluated."
    }
  }
}
```

Information Data Format:

```json
{
  "Frame": {
    "Info": {
      "Reason": "Why the frame was not evaluated. NO_GROUND_TRUTH: there is no Ground Truth within 75msec of the header time of the received objects. IGNORED_FRAME: the frame is excluded by the ignore_frames setting."
    },
    "FrameSkip": "Total number of times the evaluation was skipped. This occurs when you request the evaluation of an object but there is no ground truth value within 75msec in the dataset or footprint.points is 1 or 2."
  }
}
```

Warning Data Format:

```json
{
  "Frame": {
    "Warning": {
      "Reason": "Why the frame was not evaluated. INVALID_ESTIMATED_OBJECTS: the received objects could not be converted, e.g. the number of footprint.points is 1 or 2."
    },
    "FrameSkip": "The total number of times the evaluation was skipped, which occurs when the evaluation of an object is requested but there is no Ground Truth in the dataset within 75msec, or when the number of footprint.points is 1 or 2."
  }
}
```

Objects Data Format:

See [json schema](../../driving_log_replayer_v2/config/perception/object_output_schema.json)

Metrics Data Format:

When the `evaluation_task` is detection or tracking

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
        "AP(Center Distance)": {
          "ALL": "AP(Center Distance) rate for all labels",
          "label0": "AP(Center Distance) rate of label0",
          "label1": "AP(Center Distance) rate of label1"
        },
        "APH(Center Distance)": {
          "ALL": "APH(Center Distance) rate for all labels",
          "label0": "APH(Center Distance) rate of label0",
          "label1": "APH(Center Distance) rate of label1"
        },
        "AP(IoU 2D)": {
          "ALL": "AP(IoU 2D) rate for all labels",
          "label0": "AP(IoU 2D) rate of label0",
          "label1": "AP(IoU 2D) rate of label1"
        },
        "APH(IoU 2D)": {
          "ALL": "APH(IoU 2D) rate for all labels",
          "label0": "APH(IoU 2D) rate of label0",
          "label1": "APH(IoU 2D) rate of label1"
        },
        "AP(IoU 3D)": {
          "ALL": "AP(IoU 3D) rate for all labels",
          "label0": "AP(IoU 3D) rate of label0",
          "label1": "AP(IoU 3D) rate of label1"
        },
        "APH(IoU 3D)": {
          "ALL": "APH(IoU 3D) rate for all labels",
          "label0": "APH(IoU 3D) rate of label0",
          "label1": "APH(IoU 3D) rate of label1"
        },
        "AP(Plane Distance)": {
          "ALL": "AP(Plane Distance) rate for all labels",
          "label0": "AP(Plane Distance) rate of label0",
          "label1": "AP(Plane Distance) rate of label1"
        },
        "APH(Plane Distance)": {
          "ALL": "APH(Plane Distance) rate for all labels",
          "label0": "APH(Plane Distance) rate of label0",
          "label1": "APH(Plane Distance) rate of label1"
        }
      },
      "MOTA(Center Distance)": {
        "ALL": "MOTA for all labels, tracking and prediction only",
        "label0": "MOTA of label0"
      },
      "MOTP(Center Distance)": {
        "ALL": "MOTP for all labels, tracking and prediction only",
        "label0": "MOTP of label0"
      },
      "IDswitch(Center Distance)": {
        "ALL": "ID switches for all labels, tracking and prediction only",
        "label0": "ID switches of label0"
      },
      "ADE": { "ALL": "average displacement error, prediction only", "label0": "ADE of label0" },
      "FDE": { "ALL": "final displacement error, prediction only", "label0": "FDE of label0" },
      "MissRate": { "ALL": "miss rate, prediction only", "label0": "miss rate of label0" },
      "Error": {
        "ALL": {
          "average": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "rms": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "std": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "max": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "min": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          }
        },
        "label0": "Error metrics for the label0"
      }
    }
  }
}
```

When the `evaluation_task` is fp_validation

```json
{
  "Frame": {
    "FinalScore": {
      "GroundTruthStatus": {
        "UUID": {
          "rate": {
            "TP": "TP rate of the displyed UUID",
            "FP": "FP rate of the displyed UUID",
            "TN": "TN rate of the displyed UUID",
            "FN": "FN rate of the displyed UUID"
          },
          "frame_nums": {
            "total": "List of frame numbers, which GT is evaluated",
            "TP": "List of frame numbers, which GT is evaluated as TP",
            "FP": "List of frame numbers, which GT is evaluated as FP",
            "TN": "List of frame numbers, which GT is evaluated as TN",
            "FN": "List of frame numbers, which GT is evaluated as FN"
          }
        }
      },
      "Scene": {
        "TP": "TP rate of the scene",
        "FP": "FP rate of the scene",
        "TN": "TN rate of the scene",
        "FN": "FN rate of the scene"
      }
    }
  }
}
```

Ground Truth Coverage:

The final line also reports how much of the Ground Truth of the dataset was actually evaluated, for the degradation topic. It is written next to `FinalScore` and does not change `Result.Summary`.

```json
{
  "Frame": {
    "FinalScore": {},
    "GtFrames": "Number of Ground Truth frames of the dataset window",
    "GtFramesEvaluated": "Number of distinct Ground Truth frames which were bound to at least one valid evaluated estimate",
    "Coverage": "GtFramesEvaluated / GtFrames, rounded to 4 decimals",
    "SkipReasons": {
      "INVALID_ESTIMATED_OBJECTS": "Number of frames skipped because the received objects could not be converted",
      "NO_GROUND_TRUTH": "Number of frames skipped because no Ground Truth was found within 75msec",
      "IGNORED_FRAME": "Number of frames excluded by the ignore_frames setting"
    }
  }
}
```

A `Coverage` lower than expected means that Ground Truth frames of the dataset were never scored, e.g. because no objects message was published close enough to the annotation, so neither the TP nor the FN of those frames are in the metrics.

### Recording of the scene

In database evaluation, it is necessary to replay multiple rosbags, but due to the ROS specification, it is impossible to use multiple bags in a single launch.
Since one rosbag, i.e., one `t4_dataset`, requires one launch, it is necessary to execute as many launches as the number of datasets contained in the database evaluation.

Since database evaluation cannot be done in a single launch, perception outputs the following files in the archive directory of each evaluated topic, in addition to `result.jsonl` file.

- `scene_result.t4eval/`: the [t4perceval](https://github.com/ktro2828/t4perceval) recording of the scene (the estimations, the Ground Truth, the pass/fail verdicts and the metrics as parquet files plus `manifest.json`).
- `evaluation_config.json`: the parsed evaluation settings.
- `frame_index.json`: the frame name and the timestamps of every evaluated frame.
- `analysis_result.csv`: the metrics per distance range.

The dataset evaluation can be performed by reading every recording and outputting the index of the dataset's average with `perception_database_result.py -r <directory>`.
The earlier `scene_result.pkl` / `evaluation_config.pkl` files of `perception_eval` are not written anymore and cannot be read.

### Result file of database evaluation

In the case of a database evaluation with multiple datasets in the scenario, a file named `database_result.json` is output to the results directory.

The format is the same as the [Metrics Data Format](#evaluation-result-format).
