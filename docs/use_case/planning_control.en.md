# Evaluate Planning Control

Evaluate if Metrics and PlanningFactors are output under specified conditions.
If diagnostics is specified in include_use_case, diagnostics evaluation is also possible.

## Evaluation Method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`planning_control_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from bag and publishes metrics type and PlanningFactor type messages
3. The evaluation node subscribes to the topics and evaluates if each criterion is met. The result is dumped into a file.
4. When the playback of the rosbag is finished, the launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

### Metric

Use topics that utilize [Metric.msg](https://github.com/autowarefoundation/autoware_internal_msgs/blob/main/autoware_internal_metric_msgs/msg/Metric.msg).
Primarily intended for `/control/control_evaluator/metrics`, `/planning/planning_evaluator/metrics`, and `/system/processing_time/metrics`.
The `name` within the topic being evaluated is specified by `module_name`.
The following conditions can be evaluated:

- Whether the specified `msg.value` falls outside the range specified by the scenario
- Whether the specified `msg.value` matches the value specified by the scenario

#### Metric Normal

The condition passes successfully when the `msg.value` of the specified topic falls within the scenario's specified range or matches the value.
`all_of` requires the condition to pass at all times during the scenario, while `any_of` requires the condition to pass at least once during the scenario.

#### Metric Error

When the Metric Normal condition is not met

### PlanningFactor

#### PlanningFactor Normal(judgement: positive)

Normal if `/planning/planning_factors/**` meets all of the following conditions:

- If there is an area condition in the scenario, the x,y position of control_points[0].pose is within the range from the x,y coordinates specified in the scenario.
- If there is a behavior condition in the scenario, the planning_factor's behavior matches the behavior specified in the scenario.
- If there is a distance condition in the scenario, the planning_factor's distance (distance from Ego to control_point) is within the range specified in the scenario.

#### PlanningFactor Normal(judgement: negative)

Normal if `/planning/planning_factors/**` does not meet any of the following conditions:

- If there is an area condition in the scenario, the x,y position of control_points[0].pose is within the range from the x,y coordinates specified in the scenario.
- If there is a behavior condition in the scenario, the planning_factor's behavior matches the behavior specified in the scenario.
- If there is a distance condition in the scenario, the planning_factor's distance (distance from Ego to control_point) is within the range specified in the scenario.

#### PlanningFactor Error

When the PlanningFactor Normal condition is not met

## Output File for Evaluation Results

In planning_control, result.jsonl is created in the following three files.
result.jsonl is always output, but planning_factor_result.jsonl, metric_result.jsonl and diag_result.jsonl are only output when specified in the scenario.

### result.jsonl

Output to output_dir/result.jsonl.
Contains metric evaluation results.

When running with Evaluator, the success/failure is determined by referencing the last line of this file.
Therefore, the final success/failure information that merges the results of planning_factor_result.jsonl, metric_result.jsonl and diag_result.jsonl is written in post_process.

### planning_factor_result.jsonl

Output to output_dir/result_archive/planning_factor_result.jsonl.
Contains planning_factor evaluation results.

## metric_result.jsonl

Output to output_dir/result_archive/metric_result.jsonl.
Contains the evaluation results for metrics.

### diag_result.jsonl

Output to output_dir/result_archive/diag_result.jsonl.
Contains diagnostics evaluation results.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                    | Data type                                               |
| --------------------------------------------- | ------------------------------------------------------- |
| /control/control_evaluator/metrics            | tier4_metric_msg/msg/MetricArray                        |
| /planning/planning_evaluator/metrics          | tier4_metric_msg/msg/MetricArray                        |
| /system/processing_time/metrics               | tier4_metric_msg/msg/MetricArray                        |
| /control/autonomous_emergency_braking/metrics | tier4_metric_msg/msg/DiagnosticArray                    |
| /planning/planning_factors/\*\*               | autoware_internal_planning_msgs/msg/PlanningFactorArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## About simulation

State the information required to run the simulation.

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/scenario.yaml).

### Evaluation Result Format

#### metric

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/result.json).

#### planning_factor

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/planning_control/planning_factor_result.json).

#### diagnostics

Same as the diagnostics use case
