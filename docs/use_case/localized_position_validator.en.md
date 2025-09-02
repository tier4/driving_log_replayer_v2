# Evaluate localized position validator

Evaluate whether localized position validator works as expected.

## Annotation

An annotation (ground truth) required for evaluation will be provided via scenario.yaml as follows.

### Annotation format

The annotation must be provided in the following format:

```yaml
Evaluation:
  Annotation:
    - Timestamp: 1649138854.971764
      ExpectedValidity: true
    - Timestamp: 1649138915.570511
      ExpectedValidity: false
    - ...
```

After being sorted in chronological order, ExpectedValidity is assigned as the correct label from the time of a given Timestamp until just before the time of the next Timestamp.
If there is no subsequent Timestamp, the label remains assigned until the end.

## Evaluation method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`localized_position_validator_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data input from previously prepared rosbag and performs localization estimation
3. Evaluation node subscribes to Autoware's output topics, checks whether the number of inputs and outputs of the validator, as well as the accuracy between labels and predictions, meet the criteria, and dumps the results to a file
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### Points to note during evaluation

Depending on the simulation results, the timestamps of predictions may shift.
When there is a label switch (as in the previously mentioned Annotation format, where true/false values are swapped), the accuracy may be subject to noise.

## Evaluation Result

For each subscription to the localized position validator's output topic, the evaluation results described below are produced.
Additionally, after the simulator finishes, a comparison between the annotations and predictions is performed, and the final evaluation result is produced.

### Normal

When the number of output topics corresponding to the inputs meets or exceeds `Evaluation.Conditions.CheckInputAndOutputCount.PassRate` specified in the scenario, and the prediction accuracy meets or exceeds `Evaluation.Conditions.CheckPrediction.PassRate` specified in the scenario.

### Error

When the normal condition is not met.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                                   | Data type                                                          |
| ------------------------------------------------------------ | ------------------------------------------------------------------ |
| /localization/pose_estimator/points_aligned                  | sensor_msgs::msg::PointCloud2                                      |
| /localization/localized_position_validator/validation_result | tier4_localization_msgs::msg::LocalizedPositionValidatorPrediction |

## Arguments passed to logging_simulator.launch

- perception: false
- planning: false
- control: false
- pose_source: ndt
- twist_source: gyro_odom

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.
The following example shows the topic list available in evaluation input rosbag when multiple LiDARs are used in a real-world vehicle configuration.

| Topic name                         | Data type                                    |
| ---------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus               | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |

The vehicle topics can be included instead of CAN.

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

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the `--clock` option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localized_position_validator/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localized_position_validator/result.json).

The result is true if both the final outcome of CheckInputAndOutputCount and the subsequent comparison between annotations and predictions pass; otherwise, it is false.

Examples of each evaluation are described below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Result example:

```json
{
  "CheckInputAndOutputCount": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
    "Prediction": {
      "is_position_valid": "Prediction of localized position validator (true / false)"
    },
    "Info": {
      "InputCount": "Number of input topics",
      "OutputCount": "Number of output topics"
    }
  }
}
```
