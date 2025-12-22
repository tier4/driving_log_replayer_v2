# Evaluate Time Step Based Trajectory

Evaluates the self-vehicle trajectories output by Autoware. Currently, no evaluation against specific criteria is performed (TBD). Analysis is conducted solely via `autoware_planning_data_analyzer` in `autoware_tools` during post-processing.

## Arguments Passed to `logging_simulator.launch`

- perception: false
- control: false
- localization: false

## Simulation

State the information required to run the simulation.

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/time_step_based_trajectory/scenario.yaml)

### Evaluation Result Format

The contents of the result file are always the same.

See [sample](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/time_step_based_trajectory/result.json)
