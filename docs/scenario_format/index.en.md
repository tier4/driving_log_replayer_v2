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

### Execution settings are launch arguments

Configure additional launches, evaluated pointcloud and existing diagnostics using `ros2 launch`
arguments locally and the same string parameters under `.webauto-ci.yml`
`simulations[].simulator.parameters` in Evaluator. These are not scenario fields and require no
scenario/use-case format bump. Existing scenario versions 3.0.0–3.3.0 remain usable.

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  additional_launches:='[{"package":"aip_hd785_gen2_perception_launch","file":"radars_replay.launch.xml","arguments":{"use_sim_time":"true"}}]' \
  evaluation_topic:=/perception/radar_lidar_fusion/debug/appended_radar_points \
  evaluation_topic_rate:='null'
```

Supply your normal dataset arguments as well. The generic additional launch facility does not
supply the AIP radar launch or introduce an AIP or dedicated-monitor dependency.

| Argument | Default | Meaning |
| --- | --- | --- |
| additional_launches | JSON string `[]` | Array of package/file/arguments objects; argument values must be strings |
| evaluation_topic | empty string | Fully qualified evaluated PointCloud2 topic, also recorded |
| evaluation_topic_rate | internal sentinel `legacy` | JSON object containing module/node_name_suffix, or JSON `null` to disable |

Both evaluation arguments omitted retain legacy subscription, fixed diagnostic, results and recording.
An explicit topic with rate omitted/null disables rate evaluation. Explicit null rate without a topic
uses the original pointcloud with monitoring disabled. A rate object without a topic monitors the
original pointcloud; with a topic it monitors the specified diagnostic for that target.

```bash
evaluation_topic_rate:='{"module":"perception","node_name_suffix":"obstacle_segmentation_pointcloud"}'
```

The diagnostic name is `topic_state_monitor_{node_name_suffix}: {module}_topic_status`.
No monitor is created and no diagnostic name is guessed from the topic. Match the actual Autoware config.
All diagnostic statuses are searched; duplicate names use the worst level. OK/WARN pass, ERROR/STALE fail.
Hz thresholds remain in Autoware. Missing or expired (>2 seconds ROS time) diagnostics log throttled
warnings (5 seconds) and yield `TopicRateEvaluation.state: Invalid` with reason NotReceived/Expired.
The rate condition fails; it never silently switches to Skipped. Unrelated/old samples do not refresh
health, ROS clock rollback clears it, and paused ROS time does not advance expiry.
Disabled monitoring writes `TopicRate: null` and `{enabled: false, state: Skipped}` metadata and
excludes only the rate condition. Pointcloud checks and PassRate still apply; no fake OK graph is published.
There is no permanent failure latch. NoData and pointcloud-stopped final handling remain unchanged.

Nondefault execution settings support obstacle_segmentation with record_only=false only.
Launch creation precedes evaluator/player but does not guarantee readiness: configure play_delay
and pass use_sim_time to additional launches when appropriate. Any additional launch exit, including
code 0 before normal shutdown, fails the run. Essential-node failures must stop the containing launch.
Run IDs, resolved paths, SHA256 and arguments are archived in additional_launches.json, failures in
additional_launch_failure.json. Normal shutdown and stale failure IDs are excluded.

Recording selects its base as before (override_topics_regex > explicit record_profile > use-case profile)
and adds the exact evaluated topic, plus /diagnostics when monitoring is enabled. No extra record argument
is required. The addition also applies to explicit overrides; original profile topics remain.
Default invocations retain their regex and empty-profile recording skip. Other cluster/debug topics
require explicit recording rules. Recording patterns do not guarantee publishers, QoS or all messages.
Storage/I/O can increase with pre/post-fusion pointclouds. Bag-player /unused remapping is unchanged.

Nonempty former Simulation.AdditionalLaunches and nonnull Evaluation.EvaluationTopic scenario settings
are rejected with migration instructions instead of silently ignored. Empty/null old fields are no-ops.
Public arguments are forwarded individually through pre/simulation/post and revalidated there.
The evaluator receives normalized JSON as a string ROS parameter, shared with recorder configuration.
Pin the feature-capable build and verify startup logs. The analyzer saves configured rate states to
3_topic_rate_evaluation.json; its callback-frequency plot is reference data, distinct from diagnostic verdicts.
See sample/obstacle_segmentation/launch_arguments.md for examples.
