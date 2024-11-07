# Evaluate Planning Control

Evaluate Metrics are output at specified times and conditions

## Evaluation Method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`planning_control_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from bag and publishes a metrics type message
3. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

This node uses `/control/autonomous_emergency_braking/metrics` and `/control/control_evaluator/metrics`.
Evaluate if `/control/autonomous_emergency_braking/metrics` is the value specified in the scenario.
If a lane condition is described in the scenario, it is evaluated when a lane that can be obtained from `/control/control_evaluator/metrics` satisfies the condition.
If the conditions for evaluation are not met, no log is output.

### Normal

Normal if the value in `/control/control_evaluator/metrics` matches the value specified in the scenario.
Though, if `none` is specified, it is judged as `none` if the metric_array of topic is an empty array.
If kinematic_condition is specified, additionally, kinematic_state must meet the condition.

### Error

When the normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                    | Data type                              |
| --------------------------------------------- | -------------------------------------- |
| /control/control_evaluator/metrics            | tier4_metric_msg::msg::MetricArray     |
| /control/autonomous_emergency_braking/metrics | tier4_metric_msg::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## Arguments passed to logging_simulator.launch

- localization: false

To use /sensing/lidar/concatenated/pointcloud in the bag, add sensing:=false to the launch argument.
If you want to use perception and planning from the bag as well, add “perception:=false planning:=false” to the “launch” argument.

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${planning_control_scenario_path} sensing:=false perception:=false planning:=false
```

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

| Topic name                             | Data type                                    |
| -------------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus                   | can_msgs/msg/Frame                           |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                        |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                  |
| /tf                                    | tf2_msgs/msg/TFMessage                       |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute      |

The vehicle topics can be included instead of CAN.

| Topic name                             | Data type                                           |
| -------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                               |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped        |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                         |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute             |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| --------   | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/result.json).

The result format is shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Success is determined when all evaluation conditions set in planning and control are met.

```json
{
  "Frame": {
    "[Planning|Control]_CONDITION_INDEX": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "TotalPassed": "Total number of topics that passed the evaluation criteria",
        "Decision": "Decision of the acquired TOPIC",
        "LaneInfo": "[lane_id, s, t]",
        "KinematicState": "[vel, acc, jerk]"
      }
    }
  }
}
```
