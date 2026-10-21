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
publish_profile: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary # refer use case
  Datasets:
    - DatasetName:
        VehicleId: String
        InitialPose: Dictionary # optional; omit this key or use {} to disable the pose
        DirectInitialPose: Dictionary # optional; omit this key or use {} to disable the pose
        GoalPose: Dictionary # optional; omit this key or use {} to disable the pose
        goal_method: String # optional, default: set_goal_from_scenario
        time_offset: Dictionary # optional, default: {}
include_use_case:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary
```

### ScenarioFormatVersion

Describe the version information of the scenario format. Use the semantic version.

The latest supported version is 3.3.0. Versions 3.0.0, 3.1.0, and 3.2.0 remain supported.

Minor versions are updated each time the format is updated.

### ScenarioName

Describes the name of the scenario, used as the display name of the scenario on the Autoware Evaluator.

### ScenarioDescription

Describes a scenario, used as a scenario description on the Autoware Evaluator.

### SensorModel

Specify `sensor_model` as argument in `autoware_launch/launch/logging_simulator.launch.xml`

### VehicleModel

Specify `vehicle_model` as an argument in `autoware_launch/launch/logging_simulator.launch.xml`

### publish_profile

(Optional) Specify the name of the topics profile to control which topics are published during simulation. The profile file should be located at `config/publish_and_remap/{profile_name}.yaml`, such as `planning_control`. If not specified, all available topics will be published.

### Simulation.AdditionalLaunches

Optional in ScenarioFormatVersion 3.3.0. Start additional, long-running launch processes alongside Autoware without adding hardware-specific DLR arguments. Initially supported only for `obstacle_segmentation` with `record_only:=false`.

```yaml
ScenarioFormatVersion: 3.3.0
Simulation:
  AdditionalLaunches:
    - package: aip_hd785_gen2_perception_launch
      file: radars_replay.launch.xml
      arguments:
        use_sim_time: "true"
```

This radar example requires a separately installed replay-only launch in the AIP package; DLR does not supply it or depend on that package. Keep the existing `Evaluation` section, including `UseCaseFormatVersion: 0.3.0`, for obstacle segmentation. Evaluation-topic remapping is a separate feature and is not implemented by this setting. Both extensions are planned to share scenario version 3.3.0; this branch implements only additional launches.

- `package` and `file` are required. Resolve `file` below the installed package's `launch/` directory; nested directories and `.launch.py`, `.launch.xml`, `.launch.yaml`, and `.launch.yml` are supported. Absolute paths, `..`, escaping symlinks, missing files/packages, identical entries, and direct recursive DLR launches are rejected before simulation.
- `arguments` defaults to `{}`. Keys are launch-argument names and values must be quoted strings, including booleans and numbers. Arguments are passed as separate command arguments without a shell. The additional launch must declare and apply its own `use_sim_time`; the parent's `SetParameter` does not cross the process boundary.
- Omit `Simulation`, use `null`/`{}`, or omit/set `AdditionalLaunches: []` to retain existing behavior. `AdditionalLaunches: null` and unknown keys in these new settings are invalid. A nonempty list on an older scenario version, another use case, or record-only mode is rejected. `with_autoware:=false` is allowed; the caller supplies the other pipeline nodes.
- Additional launches are requested before the evaluator/player, but process creation does not guarantee node readiness. The existing `play_delay` is unchanged. Verify subscriptions before bag playback and allow sufficient startup time; no readiness handshake or automatic restart is provided.
- Any additional launch that exits before DLR shutdown, even with exit code 0, fails the run and stops the simulation. Each supplied launch must shut itself down if an essential child node dies. DLR cannot detect an individual node failure while its containing launch stays alive. Normal DLR shutdown signals the child launches and uses ROS launch's standard termination timeouts.
- `result_archive/additional_launches.json` records a run ID, resolved launch paths, SHA-256, package versions, and supplied arguments. Unexpected exits produce `additional_launch_failure.json`. Post-processing preserves existing frames and appends a terminal `AdditionalLaunchFailed` result, even when no result bag exists; it skips bag/plot analysis for that failed run. Run IDs prevent a previous run's failure marker from overriding the current run.
- This setting does not change bag publication/remapping/recording profiles, evaluation subscriptions, or TopicRate diagnostics. Configure recording separately if the additional outputs are needed. Launch arguments are recorded in plain text, so do not pass secrets in them.

See [the example scenario](https://github.com/tier4/driving_log_replayer_v2/blob/develop/sample/obstacle_segmentation/scenario_additional_launches.yaml). The standard `driving_log_replayer_v2.launch.py` entry point and existing CLI remain unchanged. Before using a new scenario on Evaluator, deploy a DLR image that supports 3.3.0 and verify that the scenario service preserves the new fields.

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

#### InitialPose

(Optional) Specify the initial pose for calling the `/localization/initialize` service.

```yaml
InitialPose:
  position:
    x: float
    y: float
    z: float
  orientation:
    x: float
    y: float
    z: float
    w: float
```

#### DirectInitialPose

(Optional) Specify the initial pose for calling the `/localization/initialize` service. For the difference with InitialPose, refer to [autoware_pose_initializer](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_pose_initializer). If both `InitialPose` and `DirectInitialPose` are specified, `DirectInitialPose` takes precedence and `InitialPose` is ignored; it is recommended to specify at most one of them.

```yaml
DirectInitialPose:
  position:
    x: float
    y: float
    z: float
  orientation:
    x: float
    y: float
    z: float
    w: float
```

#### GoalPose

(Optional) Specify the goal pose for calling the `/api/routing/set_route_points` service.

```yaml
GoalPose:
  position:
    x: float
    y: float
    z: float
  orientation:
    x: float
    y: float
    z: float
    w: float
```

#### goal_method

(Optional) Specifies how the goal pose is set.

| Value                    | Description                                                                                                                            |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------- |
| `set_goal_from_scenario` | (Default) Use `GoalPose` from scenario to set goal. Skip setting goal if `GoalPose` is not specified.                                  |
| `set_goal_from_rosbag`   | Read the last `/localization/kinematic_state` from rosbag and use its position as the goal. Even if `GoalPose` is specified ignore it. |

#### time_offset

(Optional) Specify the start time and duration of rosbag playback. The start time is specified as a relative time from the start of the rosbag, not based on timestamp. Place it at the same level as `GoalPose` in the dataset section of the scenario YAML.

```yaml
time_offset:
  start: float # the start time [s]
  duration: float # the duration [s]
```

### include_use_case

Use this when you want to perform evaluation with a different use case simultaneously with the use case specified in Evaluation.
Note that the nodes for the use cases specified here will not be automatically started.

Each use case's evaluator node needs to add processing to evaluate with the conditions specified in include_use_case.
Since the evaluator processes the last line of result.jsonl to determine success or failure, it is necessary to merge the results of result.jsonl from Evaluation and result.jsonl output from include_use_case in post_process.

Currently, the functionality to evaluate diagnostics in planning_control has been implemented.
