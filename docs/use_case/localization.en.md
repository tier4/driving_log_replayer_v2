# Evaluate NDT estimation

Use case `localization` will evaluate the performance of NDT localization in Autoware.
Driving Log Replayer runs the `logging_simulator` of Autoware to replay the scenario, and evaluation will run during and after the simulation.
This pages explains how the simulation and the evaluation works for this use case.

## Simulation Details

This section explains the details and requirements to run the simulation.

### Scenario Format

See [the sample scenario file](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localization/scenario.yaml).

### Arguments passed to logging_simulator.launch

- perception: false
- planning: false
- control: false
- pose_source: ndt
- twist_source: gyro_odom

### Topic to be included in the input rosbag

The input rosbag requires topics to function the localization component of Autoware.

If you use CAN data from the ECU, you will need the following topics in your rosbag.

| Topic name                         | Data type                                    |
| ---------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus               | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |

Or, you can use vehicle topics instead of CAN data.

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

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it outputs twice so it should not be included in the bag.

### localization_evaluator_node

Driving Log Replayer also launches a `localization_evaluator_node` with the `logging_simulator` to collect data for the evaluation.

The `localization_evaluator_node` subscribes the following topics.

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

The `localization_evaluator_node` publishes the following topics.

| Topic name                                             | Data type                      |
| ------------------------------------------------------ | ------------------------------ |
| /driving_log_replayer_v2/localization/lateral_distance | example_interfaces/msg/Float64 |

The `localization_evaluator_node` calls the following services.

| Service name             | Data type                                                      |
| ------------------------ | -------------------------------------------------------------- |
| /localization/initialize | autoware_internal_localization_msgs/srv/InitializeLocalization |

---

## Evaluation details

This use case can evaluate the following contents.

- NDT Availability
- NDT Convergence
- NDT Reliability
- Difference from the reference trajectory (position, twist and acceleration)
- Error rate of diagnostics
- Rise/fall timing of the diagnostics

When you launch Driving Log Replayer, the evaluation will start with the procedure below.

1. Launch the `localization_evaluator_node`, `logging_simulator.launch` and `ros2 bag play` command
2. Autoware performs localization by the sensor data from the rosbag
3. The `localization_evaluator_node` evaluates the NDT Availability, Convergence and Reliability meanwhile and record the results into a file.
4. After the logging simulation is done, [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) will start and evaluate the difference from the reference trajectory, error rate of diagnostics and the rise/fall timing of the diagnostics.
5. Summarize all results and then the program finishes automatically.

If all the evaluation items turn to be Success, the entire evaluation is judged as Success.

The following sections explain the details of each evaluation item.

### NDT Availability

This evaluation basically starts for any kind of scenario, but it doesn't only if you defined a `availability` condition in the `Conditions` of the scenario and set its `enable` parameter as `false`.

Driving Log Replayer will evaluate the NDT Availability through `logging_simulator` by detecting these two cases.

- The `pointcloud_preprocessor` is failing due to such like runtime error. (Then the `ndt_scan_matcher` cannot subscribe LiDAR scan points.)
- The `ndt_scan_matcher` itself is failing due to such like runtime error.

This can be done by monitoring the following topic using an Autoware package Component State Monitor.

- /localization/pose_estimator/exe_time_ms

We use `/localization/pose_estimator/exe_time_ms` since this topic is being published constantly. For example, `/localization/pose_estimator/pose` will not be published when the NVTL or TP is lower than the score threshold and does not suit this purpose.

This evaluation is judged as Success if `/localization/pose_estimator/exe_time_ms` is alive until the end of the `logging_simulator`, Fail if not.

### NDT Convergence

This evaluation starts if a `Convergence` condition is defined in the `Conditions` of the scenario.

```yaml
Evaluation:
  Conditions:
    Convergence:
      AllowableDistance: 0.2 # Lateral distance to be considered convergence
      AllowableExeTimeMs: 100.0 # If the NDT computation time is less than or equal to this value, it is considered successful.
      AllowableIterationNum: 30 # If the number of NDT calculations is less than or equal to this value, it is considered a success.
      PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
```

This evaluation monitors these three topics through the `logging_simulator`.

| Topic name                                                   | Success condition                            |
| ------------------------------------------------------------ | -------------------------------------------- |
| /localization/pose_estimator/initial_to_result_relative_pose | value of lateral factor <= AllowableDistance |
| /localization/pose_estimator/exe_time_ms                     | value <= AllowableExeTimeMs                  |
| /localization/pose_estimator/iteration_num                   | value <= AllowableIterationNum               |

If all three conditions have been passed, the NDT will be marked as converged for that time frame. If the rate of convergence is equal or larger than the `PassRate`, this evaluation is judged as Success.

### NDT Reliability

This evaluation starts if a `Reliability` condition is defined in the `Conditions` of the scenario.

```yaml
Evaluation:
  Conditions:
    Reliability:
      Method: NVTL # NVTL or TP which method to use for evaluation
      AllowableLikelihood: 2.3 # If above this value, the localization reliability value is considered normal.
      NGCount: 10 # If the reliability value is lower than the threshold value for more than this number in the sequence. the evaluation is considered to have failed.
```

This evaluation monitors the topic selected in the scenario during the `logging_simulator`.

| Method | Topic Name                                                           | Success Condition            |
| ------ | -------------------------------------------------------------------- | ---------------------------- |
| TP     | /localization/pose_estimator/transform_probability                   | value >= AllowableLikelihood |
| NVTL   | /localization/pose_estimator/nearest_voxel_transformation_likelihood | value >= AllowableLikelihood |

If the value of the topic is equal or larger than the `AllowableLikelihood`, the NDT is assumed to be reliable. If the TP or NVTL is below `AllowableLikelihood` for a consecutive `NGCount` times in the simulation, the evaluation is judged as Fail, if not Success.

### Difference from the reference trajectory

This evaluation basically starts for any kind of scenario.

After `logging_simulator` is done, the position, twist, and acceleration difference from the reference trajectory will be evaluated. See the actual implementation of [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) for details of such like the definition of the difference.

The difference of the following factors will be evaluated

- Position
- Orientation
- Linear Velocity
- Angular Velocity
- Linear Acceleration

You can choose which factor to evaluate or not by defining a `OverallCriteriaMask` in the scenario file and set true or false to `mean_relative_*`. If `OverallCriteriaMask` is not set in the scenario, the evaluator will set all factors to true by default. When all if the factors have small difference the evaluation is judged as Success, and Fail if not.

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

### Error rate of diagnostics

This evaluation basically starts for any kind of scenario.

After the `logging_simulator` is done `autoware_localization_evaluation_scripts` evaluates whether localization related diagnostics (listed below) are not publishing errors with a high rate.

- ndt_scan_matcher: scan_matching_status
- localization: ekf_localizer
- localization_error_monitor: ellipse_error_status
- localization: pose_instability_detector

See the actual implementation of [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) for details.

If the error rate is small enough for all diagnostics, the evaluation is judged as Success, and Fail if not.

You can disable the evaluation of this item by making the `diagnostics_not_ok_rate` in the `OverallCriteriaMask` false. If `OverallCriteriaMask` is not set in the scenario, the evaluator will set it to true by default.

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

### Rise/fall timing of the diagnostics

This evaluation starts if a `DiagnosticsFlagCheck` condition is defined in the `Conditions` of the scenario.

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

After the `logging_simulator` is done `autoware_localization_evaluation_scripts` evaluates whether localization related diagnostics rise or fall at an expected timing. See the actual implementation of [`autoware_localization_evaluation_scripts`](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) for details.

If all the specified diagnostics have risen/fallen within +/- 0.2 seconds of the expected time (defined by `at_sec` and `at_nanosec`), the evaluation is judged as Success, and Fail if not. Also if the flag has risen/fallen much earlier than expected, the evaluation is judged as Fail.

### Evaluation Result Format

See [the sample file](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/localization/result.json).

For NDT Availability, Convergence and Reliability you can have the following result frames

Availability Result

```json
{
  "Availability": {
    "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Warn" },
    "Info": {}
  }
}
```

Convergence Result

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

Reliability Result

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

For the difference from the reference trajectory, Error rate of diagnostics, and Rise/fall timing of the diagnostics, all results will be summarized in the bottom the result.json file.

```json
{
  "Result": {
    "Success": false,
    "Summary": "Failed: Convergence (Fail): 570 / 632 -> 90.19%, Reliability (Fail): NVTL Sequential NG Count: 10 (Total Test: 632, Average: 2.46835, StdDev: 0.16043), NDT Availability (Success): NDT available, mean_position_norm=0.166 [m]|mean_angle_norm=0.032 [deg]|mean_linear_velocity_norm=0.003 [m/s]|mean_angular_velocity_norm=0.001 [rad/s]|localization__ekf_localizer 13.447 [%] is too large.|localization__pose_instability_detector 0.000 [%]|localization_error_monitor__ellipse_error_status 13.447 [%] is too large.|ndt_scan_matcher__scan_matching_status 57.177 [%] is too large.|Diagnostics flag 'pose_is_passed_delay_gate' OK.|Diagnostics flag 'pose_no_update_count' OK."
  }
}
```
