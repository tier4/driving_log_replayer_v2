# Copyright (c) 2025 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import datetime
from importlib import import_module
import json
from pathlib import Path
import subprocess

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
import yaml


def create_output_dir(output_dir_str: str, scenario_path: Path) -> Path:
    if output_dir_str != "":
        output_dir = Path(output_dir_str)
        output_dir.mkdir(exist_ok=True, parents=True)
        return output_dir
    # create output_dir in scenario path
    time_now = datetime.datetime.now().strftime("%Y-%m%d-%H%M%S")  # noqa
    output_dir = scenario_path.parent.joinpath("out", time_now)
    output_dir.mkdir(exist_ok=True, parents=True)
    symlink_dst = output_dir.parent.joinpath("latest").as_posix()
    update_symlink = ["ln", "-snf", output_dir.as_posix(), symlink_dst]
    subprocess.run(update_symlink, check=False)
    return output_dir


def get_dataset_index(idx_str: str, dataset_length: int) -> int | str:
    if idx_str == "":  # default value
        if dataset_length == 1:
            return 0
        return "You need to set dataset_index"
    try:
        idx_int = int(idx_str)
        if idx_int < 0 or idx_int > dataset_length:
            return f"dataset_index {idx_int} not in index range"
    except ValueError:
        return f"cannot parser dataset_index {idx_str}"
    else:
        return idx_int


def extract_index_from_id(t4_dataset_id: str, datasets: list[dict]) -> int | str:
    for idx, dataset_dict in enumerate(datasets):
        for dataset_id in dataset_dict:
            if t4_dataset_id == dataset_id:
                # this block is for local usage
                return idx
    return "index not found"


def check_launch_component(conf: dict) -> dict:
    if conf["with_autoware"] != "true":
        return {"autoware": "false"}
    launch_config = import_module(f"driving_log_replayer_v2.launch.{conf['use_case']}")
    arg_disable = launch_config.AUTOWARE_DISABLE
    # update autoware component launch or not
    autoware_components = ["sensing", "localization", "perception", "planning", "control"]
    launch_component = {}
    for component in autoware_components:
        # argument has higher priority than the launch_config.py setting.
        if conf.get(component) is None and arg_disable.get(component) is not None:
            conf[component] = arg_disable[component]
        launch_component[component] = conf.get(component, "true")
    return launch_component


def get_launch_arguments() -> list:
    """
    Set and return launch argument.

    scenario_path
    output_dir
    dataset_dir
    dataset_index
    t4_dataset_path
    play_rate
    play_delay
    with_autoware
    record_only
    override_topics_regex
    storage
    remap_arg
    remap_profile
    """
    launch_arguments = []

    def add_launch_arg(
        name: str,
        default_value: str | None = None,
        description: str = "",
    ) -> None:
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description),
        )

    add_launch_arg("scenario_path", description="scenario file path")
    add_launch_arg(
        "output_dir",
        default_value="",
        description="Directory to output evaluation results. If omitted, the out/${datetime} directory is created in the same directory as scenario. Mount in read-write mode when using docker",
    )
    add_launch_arg(
        "dataset_dir",
        default_value="",
        description="Directory where the dataset is located. If not specified, the directory where the scenario is located.",
    )
    add_launch_arg(
        "dataset_index",
        default_value="",
        description="index number of dataset. t4_dataset_path = dataset_dir.joinpath(Datasets[dataset_index])",
    )
    add_launch_arg(
        "t4_dataset_path",
        default_value="",
        description="Set t4_dataset_path directly. Compatible with v1. Mutually exclusive with dataset_dir.",
    )
    add_launch_arg(
        "t4_dataset_id",
        default_value="",
        description="Required when passing t4_dataset_path. Specify the Datasets[i].key",
    )
    add_launch_arg("play_rate", default_value="1.0", description="ros2 bag play rate")
    add_launch_arg("play_delay", default_value="10.0", description="ros2 bag play delay")
    add_launch_arg(
        "with_autoware",
        default_value="true",
        description="Whether to launch Autoware or not. set false if Autoware is started on a different PC.",
    )
    add_launch_arg(
        "record_only",
        default_value="false",
        description="Do only bag record without starting evaluator node",
    )
    add_launch_arg(
        "override_topics_regex",
        default_value="",
        description="use allowlist. Ex: override_topics_regex:=\^/tf\$\|/sensing/lidar/concatenated/pointcloud\|\^/perception/.\*/objects\$",  # noqa
    )
    add_launch_arg(
        "storage",
        default_value="sqlite3",  # Settings are adjusted to ros distro standards. Currently autoware is humble, so use sqlite3. Change to mcap when updated to jazzy.
        description="select storage type mcap or sqlite3",
    )
    add_launch_arg(
        "remap_arg",
        default_value="",
        description="use comma separated string. Ex: remap_arg:=/tf,/sensing/lidar/concatenated/pointcloud",
    )
    add_launch_arg(
        "remap_profile",
        default_value="",
        description="Specify the name of the profile. config/remap/{profile_name}.yaml. Ex: remap_profile:=x2",
    )

    return launch_arguments


def is_arg_valid(conf: dict) -> LogInfo | None:
    # check conf
    if conf["dataset_dir"] != "" and conf["t4_dataset_path"] != "":
        return [
            LogInfo(
                msg="Both dataset_dir and t4_dataset_path are specified. Only one of them can be specified."
            )
        ]
    if conf["t4_dataset_path"] != "" and conf["t4_dataset_id"] == "":
        return [LogInfo(msg="t4_dataset_id is required when passing t4_dataset_path.")]
    if conf["remap_arg"] != "" and conf["remap_profile"] != "":
        return [
            LogInfo(
                msg="Both remap_arg and remap_profile are specified. Only one of them can be specified."
            )
        ]
    return None


def load_scenario(scenario_path: Path) -> dict:
    with scenario_path.open() as scenario_file:
        return yaml.safe_load(scenario_file)


def get_dataset_index_from_conf(conf: dict, datasets: list[dict]) -> int | str:
    if conf["t4_dataset_path"] != "":
        return extract_index_from_id(conf["t4_dataset_id"], datasets)
    return get_dataset_index(conf["dataset_index"], len(datasets))


def update_conf_with_dataset_info(
    conf: dict,
    t4_dataset_path: Path,
    yaml_obj: dict,
    dataset_info: dict,
    output_dir: Path,
) -> None:
    v = dataset_info
    conf["vehicle_id"] = v["VehicleId"]
    conf["initial_pose"] = json.dumps(v.get("InitialPose", {}))
    conf["direct_initial_pose"] = json.dumps(v.get("DirectInitialPose", {}))
    conf["goal_pose"] = json.dumps(v.get("GoalPose", {}))
    conf["t4_dataset_path"] = t4_dataset_path.as_posix()
    conf["vehicle_model"] = yaml_obj["VehicleModel"]
    conf["sensor_model"] = yaml_obj["SensorModel"]
    conf["map_path"] = t4_dataset_path.joinpath("map").as_posix()
    conf["input_bag"] = t4_dataset_path.joinpath("input_bag").as_posix()
    conf["result_json_path"] = output_dir.joinpath("result.json").as_posix()
    conf["result_bag_path"] = output_dir.joinpath("result_bag").as_posix()
    conf["result_archive_path"] = output_dir.joinpath("result_archive").as_posix()
    conf["use_case"] = yaml_obj["Evaluation"]["UseCaseName"]

    if conf["use_case"] == "dlr_all":
        conf["record_only"] = "true"


def prepare_paths(conf: dict) -> tuple[Path, Path, Path]:
    scenario_path = Path(conf["scenario_path"])
    dataset_dir = scenario_path.parent if conf["dataset_dir"] == "" else Path(conf["dataset_dir"])
    output_dir = create_output_dir(conf["output_dir"], scenario_path)
    conf["output_dir"] = output_dir.as_posix()
    return scenario_path, dataset_dir, output_dir


def ensure_arg_compatibility(context: LaunchContext) -> list:
    conf = context.launch_configurations
    is_valid = is_arg_valid(conf)
    if is_valid is not None:
        return is_valid

    scenario_path, dataset_dir, output_dir = prepare_paths(conf)
    yaml_obj = load_scenario(scenario_path)
    datasets = yaml_obj["Evaluation"]["Datasets"]
    dataset_index = get_dataset_index_from_conf(conf, datasets)
    if isinstance(dataset_dir, str):
        return [LogInfo(msg=dataset_index)]

    k, v = next(iter(datasets[dataset_index].items()))
    t4_dataset_path = (
        Path(conf["t4_dataset_path"]) if conf["t4_dataset_path"] != "" else dataset_dir.joinpath(k)
    )  # Do not update if t4_dataset_path is set by argument. If not, create t4_dataset_path from data_dir
    update_conf_with_dataset_info(conf, t4_dataset_path, yaml_obj, v, output_dir)

    return [
        LogInfo(
            msg=f"{t4_dataset_path=}, {dataset_index=}, {output_dir=}, use_case={conf['use_case']}",
        ),
        LogInfo(
            msg=f"{check_launch_component(conf)=}",
        ),
        LogInfo(
            msg=f"{conf.get('initial_pose')=}, {conf.get('direct_initial_pose')=}",
        ),
        LogInfo(
            msg=f"{conf.get('goal_pose')=}",
        ),
    ]


def add_use_case_arguments(context: LaunchContext) -> list:
    conf = context.launch_configurations
    launch_config = import_module(f"driving_log_replayer_v2.launch.{conf['use_case']}")
    use_case_launch_arg: list = launch_config.USE_CASE_ARGS
    if len(use_case_launch_arg) == 0:
        return [LogInfo(msg="no use case launch argument")]
    return use_case_launch_arg
