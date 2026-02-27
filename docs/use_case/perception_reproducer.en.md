# Evaluate Perception Reproducer

Evaluate if the vehicle can complete the specified route under closed-loop conditions with perception data reproduced from rosbag.
The perception_reproducer node reproduces perception objects from rosbag based on the current ego position, enabling closed-loop testing.

## Evaluation Method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`perception_reproducer_evaluator_node`), `planning_simulator.launch` file and `perception_reproducer` node
2. The `perception_reproducer` node reads perception objects from rosbag based on the current ego position and publishes them
3. The vehicle starts driving when Autoware enters DRIVING state (ENGAGE)
4. The evaluation node subscribes to the topics and evaluates if each criterion is met. The result is dumped into a file.
5. When all pass conditions are met or any fail condition is triggered, the launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

### Pass Conditions

When all condition groups in `pass_conditions` meet ONCE, the test passes and terminates.

#### ego_kinematic_trigger

Evaluate if the ego vehicle reaches the specified area.

- If there is an area condition in the scenario, the x,y position of ego is within the range from the x,y coordinates specified in the scenario.
- If there is a velocity condition in the scenario, the ego's velocity is within the range specified in the scenario.
- If there is an acceleration condition in the scenario, the ego's acceleration is within the range specified in the scenario.

#### time_wait_trigger

Evaluate if the specified wait time has elapsed.

- The condition is met when the elapsed time from activation reaches `wait_seconds`.

#### condition_group

Nested condition groups are supported. The group can be configured with `start_at` and `end_at` to control when it becomes active.

### Fail Conditions

When any condition group in `fail_conditions` does NOT meet ONCE, the test fails and terminates after `terminated_after_fail_s` seconds.

#### metric

Evaluate metrics. Same as the planning_control use case.

#### diagnostic

Evaluate diagnostics status. Same as the diagnostics use case.

#### planning_factor

Evaluate planning factors. Same as the planning_control use case.

#### ego_kinematic

Evaluate ego kinematic conditions (position, velocity, acceleration) continuously during the active period.

- If there is an area condition in the scenario, the x,y position of ego is within/outside the range from the x,y coordinates specified in the scenario.
- If there is a velocity condition in the scenario, the ego's velocity is within the range specified in the scenario.
- If there is an acceleration condition in the scenario, the ego's acceleration is within the range specified in the scenario.

#### condition_group

Nested condition groups are supported. The group can be configured with `start_at` and `end_at` to control when it becomes active.

## Output File for Evaluation Results

In perception_reproducer, result.jsonl is created in the following three files.
result.jsonl is always output, but pass_result.jsonl and fail_result.jsonl are only output when specified in the scenario.

### result.jsonl

Output to output_dir/result.jsonl.
Contains summarized results of pass and fail evaluations.

When running with Evaluator, the success/failure is determined by referencing the last line of this file.
Therefore, the final success/failure information that merges the results of pass_result.jsonl and fail_result.jsonl is written in post_process.

### pass_result.jsonl

Output to output_dir/result_archive/pass_result.jsonl.
Contains pass condition evaluation results.

### fail_result.jsonl

Output to output_dir/result_archive/fail_result.jsonl.
Contains fail condition evaluation results.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                           | Data type                                               |
| ------------------------------------ | ------------------------------------------------------- |
| /localization/kinematic_state        | nav_msgs/msg/Odometry                                   |
| /localization/acceleration           | geometry_msgs/msg/AccelWithCovarianceStamped            |
| /control/control_evaluator/metrics   | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_evaluator/metrics | tier4_metric_msg/msg/MetricArray                        |
| /system/processing_time/metrics      | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_factors/\*\*      | autoware_internal_planning_msgs/msg/PlanningFactorArray |
| /diagnostics                         | diagnostic_msgs/msg/DiagnosticArray                     |
| /autoware/state                      | autoware_system_msgs/msg/AutowareState                  |

Published topics:

| Topic name                                               | Data type           |
| -------------------------------------------------------- | ------------------- |
| /driving_log_replayer/perception_reproducer/results      | std_msgs/msg/String |
| /driving_log_replayer/perception_reproducer/pass_results | std_msgs/msg/String |
| /driving_log_replayer/perception_reproducer/fail_results | std_msgs/msg/String |

## About simulation

State the information required to run the simulation.

### Topics that must not be included in the input rosbag

See [perception_reproducer](https://github.com/autowarefoundation/autoware_tools/tree/main/planning/planning_debug_tools#perception-reproducer).

### Planning Simulator

This use case uses `planning_simulator.launch` instead of `logging_simulator.launch` to enable closed-loop testing where the vehicle can actually drive.

### Perception Reproducer Node

The `perception_reproducer` node from `planning_debug_tools` package reproduces perception objects from rosbag based on the current ego position.
For more details, see [perception_reproducer](https://github.com/autowarefoundation/autoware_tools/tree/main/planning/planning_debug_tools#perception-reproducer).

Configuration options:

- `noise`: Apply perception noise to the objects
- `reproduce_cool_down`: Cool down time for republishing (default: 80.0)
- `tracked_object`: Publish tracked object
- `search_radius`: Search radius for searching rosbag's ego odom messages (default: 1.5)

### Route Method

The driving route can be configured via the `route_method` field at the dataset level in the scenario YAML.
When `route_method` is set to `play_route_from_rosbag`, the `--pub-route` flag is passed to the `perception_reproducer` node.

See [scenario format](../scenario_format/index.en.md#route_method) for details on `route_method`.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_reproducer/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/perception_reproducer/result.jsonl).
