# Command

The driving_log_replayer_v2 can be started by specifying the scenario path.

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} [output_dir:=${output_dir} dataset_dir:=${dataset_dir}]
```

## Run driving_log_replayer_v2 with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.

## Run driving_log_replayer_v2 with driving-log-replayer-v2-cli

Using CLI, multiple scenarios can be executed consecutively with a single command input.

For example, suppose that multiple scenarios are placed in subdirectories (SCENARIO_DIR1, SCENARIO_DIR2...) under SCENARIO_ROOT as shown below.

```shell
SCENARIO_ROOT
├── SCENARIO_DIR1
│   ├── out # output directyory
│   └── SCENARIO_DIR1_DATASET0 # t4_dataset
│   │  ├── annotation
│   │  ├── data
│   │  ├── input_bag
│   │  ├── map
│   │  └── status.json
│   └── SCENARIO_DIR1_DATASET1 # t4_dataset
│   │  └── ...
│   │  ...
│   └── scenario.yaml  # scenario fixed name
│
├── SCENARIO_DIR2
│   ├── out # output directyory
│   └── SCNERIO_DIR2_DATASET0 # t4_dataset
│   │  ├── annotation
│   │  ├── data
│   │  ├── input_bag
│   │  ├── map
│   │  └── status.json
│   └── scenario.yaml  # scenario fixed name
...
```

To run multiple scenarios in the above SCENARIO_ROOT with the ros2 launch command, you need to hit the command multiple times as follows.

```shell
source ${AUTOWARE_ROOT}/install/setup.bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2 scenario_path:=${SCENARIO_ROOT}/SCENARIO_DIR1/scenario.yaml dataset_index:=0 # If you have multiple datasets
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2 scenario_path:=${SCENARIO_ROOT}/SCENARIO_DIR1/scenario.yaml dataset_index:=1 # If you have multiple datasets
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2 scenario_path:=${SCENARIO_ROOT}/SCENARIO_DIR2/scenario.yaml
...
```

If you use cli, you only need the following command

```shell
source ${AUTOWARE_ROOT}/install/setup.bash
dlr2 simulation run ${SCENARIO_ROOT}
```

### cli installation

You can install cli with the following command.

```shell
# install
pipx install git+https://github.com/tier4/driving_log_replayer_v2.git

# upgrade
pipx upgrade driving-log-replayer-v2

# uninstall
pipx uninstall driving-log-replayer-v2
```

### cli limitation

The following limitations apply because of the automatic search for scenario files in CLI.

- The scenario file must be named scenario.yaml
- scenario.yaml must exist in a subdirectory under SCENARIO_ROOT (no sub-sub-directories allowed)

### cli output

Using CLI not only shortens the command input, but also increases the number of files output.

The following is an example of the output destination when using CLI.
The simulation run command and the console log are output as files.
This is used to run multiple tests in a row and debug only the tests that show errors later.

```shell
OUTPUT_LATEST
├── 0 # result of Datasets[0]
│   ├── result.jsonl
│   ├── result_archive_path
│   └── result_bag
│       ├── metadata.yaml
│       └── result_bag_0.db3
├── 1 # result of Datasets[1]
│   ├── result.jsonl
│   ├── result_archive_path
│   └── result_bag
│       ├── metadata.yaml
│       └── result_bag_0.db3
├── console.log # Logs displayed in the console as a file
└── run.bash # simulation run command
```
