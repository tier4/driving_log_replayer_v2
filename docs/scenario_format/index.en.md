# driving_log_replayer_v2 Scenario Format Definition

This section describes the scenario format used in driving_log_replayer_v2.

## Notes on the format

- Keys are defined in CamelCase until Scenario Format Version 3.0
- Due to scenario validation using pydantic, snake_case is recommended for newly added keys from Scenario Format Version 3.1
- Unless otherwise specified, the coordinate system is `map` coordinate system.
- Unless otherwise specified, the following unit system is used.

```shell
Distance: m
Velocity: m/s
acceleration: m/s^2
Time: s
```

## Samples

Sample scenarios are stored in the [sample](https://github.com/tier4/driving_log_replayer_v2/tree/develop/sample) folder.

## Format

The basic structure is as follows. Details of each key are described below.

### driving_log_replayer_v2 Scenario Format version 3.x.x

```yaml
ScenarioFormatVersion: 3.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
topics_profile: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary # refer use case
  Datasets:
    - DatasetName:
        VehicleId: String
include_use_case:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary
```

### ScenarioFormatVersion

Describe the version information of the scenario format. Use the semantic version.

Current Version is 3.1.0

Minor versions are updated each time the format is updated.

### ScenarioName

Describes the name of the scenario, used as the display name of the scenario on the Autoware Evaluator.

### ScenarioDescription

Describes a scenario, used as a scenario description on the Autoware Evaluator.

### SensorModel

Specify `sensor_model` as argument in `autoware_launch/launch/logging_simulator.launch.xml`

### VehicleModel

Specify `vehicle_model` as an argument in `autoware_launch/launch/logging_simulator.launch.xml`

### topics_profile

(Optional) Specify the name of the topics profile to control which topics are published during simulation. The profile file should be located at `config/topics/{profile_name}.yaml`, such as `planning_control`. If not specified, all available topics will be published.

### Evaluation

Define the evaluation conditions for the simulation.

#### UseCaseName

Specify an evaluation program.

The evaluation is executed by calling the evaluator node with the name specified here.

#### UseCaseFormatVersion

Describe the version information of the use case format. The semantic version shall be used.
Until the major version becomes 1, the minor version is updated every time the format is updated.
The initial version is 0.1.0.

#### Conditions

Specify conditions that can be set for each use case.

Refer to each [use case](../use_case/index.en.md) for the conditions that can be specified.

#### Datasets

Multiple datasets can be described, but they can be used only when the same evaluation conditions are used for multiple datasets.
If multiple datasets are described, the index of the dataset to be used must be passed as the launch argument.
The index starts with the number 0.
If there is only one dataset, dataset_index:=0 may be used.

```shell
# If the number of datasets described in the scenario is 1. dataset_index:=0 can be omitted.
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} [dataset_index:=0]

# If the number of datasets described in the scenario is more than one
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} dataset_index:=${index_number}
```

#### DatasetName

dataset name of t4_dataset

#### VehicleId

Specify `vehicle_id` as an argument in `autoware_launch/launch/logging_simulator.launch.xml`

If you don't know `vehicle_id`, set `default`.

### include_use_case

Use this when you want to perform evaluation with a different use case simultaneously with the use case specified in Evaluation.
Note that the nodes for the use cases specified here will not be automatically started.

Each use case's evaluator node needs to add processing to evaluate with the conditions specified in include_use_case.
Since the evaluator processes the last line of result.jsonl to determine success or failure, it is necessary to merge the results of result.jsonl from Evaluation and result.jsonl output from include_use_case in post_process.

Currently, the functionality to evaluate diagnostics in planning_control has been implemented.
