# Evaluate Diagnostics

Evaluates whether diagnostics are at a specified level at a specified time.

A similar evaluation is performance_diag, which is specialized for LiDAR.
diagnostics_evaluator_node has only a simple function to evaluate level, but it supports arbitrary status.name.

## Evaluation Method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`diagnostics_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and publishes `/diagnostics`.
3. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

If msg.status[0].hardware_id of the received msg matches the hardware_id specified in the scenario and msg.header.stamp meets the time specified in the scenario, it is evaluated.
If the conditions for evaluation are not met, no log is output.

### Normal

There exists a status in msg.status that satisfies the name and level specified in the scenario.

### Error

When the normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name   | Data type                             |
| ------------ | ------------------------------------- |
| /diagnostics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## Arguments passed to logging_simulator.launch

No (launch with default argument)

To use /sensing/lidar/concatenated/pointcloud in the bag, add sensing:=false to the launch argument.
If you want to use perception and planning from the bag as well, add “perception:=false planning:=false” to the “launch” argument.

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${daignostics_scenario_path} sensing:=false perception:=false planning:=false
```

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Which topic is needed depends on what you want to do.

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| --------   | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/diagnostics/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/diagnostics/result.json).

The result format is shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Success is determined when all evaluation conditions are met.

```json
{
  "Frame": {
    "Condition_IDNEX": {
      "Result": {"Total": "Success or Fail", "Frame": "Success or Fail"},
      "Info": {
        "TotalPassed": "Total number of topics that passed the evaluation criteria",
        "Level": "Level of acquired status"
      }
    }
  }
}
```
