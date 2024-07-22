# Evaluate Planning Control

Evaluate whether Planning / Control metrics are output at specified times and conditions

## Evaluation Method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`planning_control_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and the perception module performs recognition.
3. Using the results of perception, Autoware output Metrics to `/diagnostic/planning_evaluator/metrics` for planning and `/diagnostic/control_evaluator/metrics` for control.
4. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
5. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

It is evaluated when status[0].name of the topic matches the module name specified in the scenario and status[0].value[0].key is a decision.
If a lane condition is described in the scenario, it is evaluated when the lane condition is also satisfied.
If the conditions for evaluation are not met, no log is output.

### Normal

Normal if status[0].values[0].value matches the decision in the scenario.
If kinematic_condition is specified, additionally, kinematic_state must meet the condition.

### Error

When the normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                             | Data type                             |
| -------------------------------------- | ------------------------------------- |
| /diagnostic/control_evaluator/metrics  | diagnostic_msgs::msg::DiagnosticArray |
| /diagnostic/planning_evaluator/metrics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.

- localization: false

### Arguments specified in the scenario or launch command

- sensing: It can be disabled by specifying LaunchSensing: false in the scenario. Or specify sensing:=false in the launch command
- perception: It can be disabled by specifying LaunchPerception: false in the scenario. Or specify perception:=false in the launch command
- planning: It can be disabled by specifying LaunchPlanning: false in the scenario. Or specify planning:=false in the launch command

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

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/scenario.ja.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/result.json).

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
