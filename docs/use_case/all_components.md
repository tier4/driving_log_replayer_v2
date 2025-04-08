# Evaluate All Components

Run all components of Autoware to test whether the topic is output correctly and whether the output rate is stable.

## Execution method

No evaluation is performed, only recording the BAG file for post-analysis.

Launching the file executes the following steps:

1. Execute launch of `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and the each component outputs topics.
3. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the bag record is terminated.

## Arguments passed to logging_simulator.launch

- sensing: true
- perception: true
- planning: true
- control: true
- localization: true
- pose_source: ndt
- twist_source: gyro_odom

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

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/all_components.yaml)

### Evaluation Result Format

The contents of the result file are always the same.

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/all_components/result.json)
