# False Positive Evaluation of perception

Evaluates whether specified pointcloud or bbox exist within the specified polygon based on the recognition results from Autoware's perception function.

During Autoware execution, save the perception topic. Subsequently, perform evaluation during post-processing.

## Preparation

Refer to the [preparation](/docs/use_case/perception.en.md#Preparation) for perception.

## Evaluation method

This command will perform the following steps:

1. launch the commands logging_simulator.launch and ros2 bag play
2. Autoware receives the sensor data output from the rosbag and the perception module recognizes it
3. Record the output topics in the bag file
4. After rosbag playback is finished, parse the saved rosbag one message at a time and evaluate the target topic.

## Evaluation results

The results are calculated for each subscription to judge pass/fail. The format and available states are described below.

### Normal

- The pointcloud or bbox must be non-contact within the area specified by `non_detection_area`. (If on the boundary, it is considered non-contact.)
- If the condition `PassRate >= Normal / Total Received * 100` is satisfied, the Total of Result becomes Success.

### Error

When normal conditions are not met

### Skipping evaluation

In the following cases, only increment FrameSkip by 1.
FrameSkip is a counter for the number of times evaluation was skipped.

- When the number of points in the received object's footprint is 1 or 2 (this condition is scheduled to be removed when perception_eval is updated)

## Topic name and data type used by evaluation script

受信できるtopic型

| Data type                          |
| ---------------------------------- |
| sensor_msgs/msg/PointCloud2        |
| autoware_msgs/msg/DetectedObjects  |
| autoware_msgs/msg/TrackedObjects   |
| autoware_msgs/msg/PredictedObjects |

Additionally, the results obtained through evaluation are written to a rosbag.

| topic name                                                | Data type                          |
| --------------------------------------------------------- | ---------------------------------- |
| /driving_log_replayer_v2/perception_fp/results            | std_msgs/msg/String                |
| /driving_log_replayer_v2/perception_fp/non_detection_area | visualization_msgs/msg/MarkerArray |
| /driving_log_replayer_v2/perception_fp/fp_objects         | visualization_msgs/msg/MarkerArray |

## Arguments passed to logging_simulator.launch

- localization: false
- planning: false
- control: false

## Simulation

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

## Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_fp/scenario.yaml)

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_fp/result.json)

In perception, the results evaluated by perception_eval under the conditions specified in the scenario are output for each frame.
Since the final metrics are calculated after all data has been processed, only the last row differs in format from the others.

Below is the format for each frame and the format for the metrics.
**Note: Common elements already explained in the result file format are omitted.**

Format of each frame:

```json
{
  "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
  "Stamp": {
    "System": 123456789.123456,
    "ROS": 234567891.234567
  },
  "Frame": {
    "FrameSkip": "Total number of times the rating was skipped.",
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

Warning Data Format:

```json
{
  "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
  "Stamp": {
    "System": 123456789.123456,
    "ROS": 234567891.234567
  },
  "Frame": {
    "Warning": "Warning message",
    "FrameSkip": "Total number of times evaluations were skipped. Occurs when the number of footprint.points is either 1 or 2."
  }
}
```
