# Evaluate Annotationless Perception

Evaluate Autoware's recognition features (perception) without annotations using the perception_online_evaluator.

Requires Autoware with the following PR features.
<https://github.com/autowarefoundation/autoware.universe/pull/6556>

## Evaluation method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`annotationless_perception_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and the perception module performs recognition.
3. The perception_online_evaluator publishes diagnostic topic to `/perception/perception_online_evaluator/metrics`
4. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
5. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation results

The output topic of perception_online_evaluator is in the form of the following sample.
[topic sample](https://github.com/tier4/log_evaluator/blob/main/sample/annotationless_perception/diag_topic.txt)

For each subscription, the following judgment results are output for each recognition class.

If all classes are normal, the test is successful.

### Normal

The following two values specified in the scenario or launch argument are used to judge

- Threshold
- PassRange(Coefficient to correct threshold)

Success or failure is determined for each status.name in `/perception/perception_online_evaluator/metrics` according to the following rules.
Items for which no threshold is set (min, max, mean) are always judged as normal. Only those items for which a threshold is specified are subject to evaluation.

#### min

If `threshold * lower_limit` <= `minimum value of min` <= `threshold * upper_limit`, it is assumed to be normal.

#### max

If `threshold * lower_limit` <= `maximum value of max` <= `threshold * upper_limit`, it is assumed to be normal.

Lower limit recommended to be 0.0

#### mean

If `threshold * lower_limit` <= `average value of mean` <= `threshold * upper_limit`, it is assumed to be normal.

#### metric_value

If `threshold * lower_limit` <= `value of metric_value` <= `threshold * upper_limit`, it is assumed to be normal.
metric_value is determined by the current topic value only and does not update the values of min, max, and mean metrics.

An illustration is shown below.

![metrics](./images/annotationless_metrics.drawio.svg)

### Error

When the normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                      | Data type                             |
| ----------------------------------------------- | ------------------------------------- |
| /perception/perception_online_evaluator/metrics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## Arguments passed to logging_simulator.launch

- localization: false
- planning: false
- control: false
- use_perception_online_evaluator: true

## simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

| Topic name                             | Data type                                    |
| -------------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus                   | can_msgs/msg/Frame                           |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                        |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                          |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                  |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                   |
| /tf                                    | tf2_msgs/msg/TFMessage                       |

The vehicle topics can be included instead of CAN.

| Topic name                             | Data type                                           |
| -------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                               |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped        |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                           |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                               |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                                 |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                         |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                          |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/annotationless_perception/scenario.yaml)

### Evaluation Result Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/annotationless_perception/result.json)

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

```json
{
  "Frame": {
    "Ego": {},
    "OBJECT_CLASSIFICATION": {
      // Recognized class
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" }, // The results for Total and Frame are the same. The same values are output to make the data structure the same as other evaluations.
      "Info": {
        "name_min_max_mean": { "min": "min value", "max": "max value", "mean": "average value" },
        "name_metric_value": { "metric_value": "value"},
        ...
      },
      "Metrics": {
        "name_min_max_mean": {
          "min": "Minimum value of min",
          "max": "Maximum value of max",
          "mean": "Average value of mean"
        },
        ...
      }
    }
  }
}
```

See the figure below for the meaning of items

![lateral_deviation](./images/lateral_deviation.png)

![predicted_path_deviation](./images/predicted_path_deviation.png)
