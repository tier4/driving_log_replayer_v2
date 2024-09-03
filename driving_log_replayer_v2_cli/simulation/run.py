import datetime
from pathlib import Path
import subprocess

import termcolor
import yaml

from driving_log_replayer_v2_cli.core.result import display_all
from driving_log_replayer_v2_cli.core.shell import run_with_log


def run(scenario_root_directory: Path, launch_args: list[str]) -> None:
    for scenario_dir in scenario_root_directory.glob("*"):
        # open scenario file and make ros2 launch command
        if not scenario_dir.is_dir():
            continue
        scenario_file = scenario_dir.joinpath("scenario.yaml")
        if not scenario_file.exists():
            termcolor.cprint(f"{scenario_file=} does not exist", "yellow")
            continue
        output_case = create_output_dir_by_time(scenario_dir.joinpath("out"))
        launch_cmd = None
        launch_arg_dict = args_to_dict(launch_args)
        launch_cmd = create_launch_cmd(
            scenario_file,
            output_case,
            launch_arg_dict,
        )

        if launch_cmd is None:
            continue

        # save command as bash script
        run_script = output_case.joinpath("run.bash")
        with run_script.open("w") as f:
            f.write(launch_cmd)

        # run simulation
        cmd = ["/bin/bash", run_script.as_posix()]
        try:
            run_with_log(cmd, output_case.joinpath("console.log"))
        except KeyboardInterrupt:
            termcolor.cprint("Simulation execution canceled by Ctrl+C", "red")
            break

    # display result
    display_all(scenario_root_directory, "latest")  # latestのsymlinkが貼られている


def create_output_dir_by_time(base_path: Path) -> Path:
    output_dir_by_time = base_path.joinpath(
        datetime.datetime.now().strftime("%Y-%m%d-%H%M%S")  # noqa
    )
    output_dir_by_time.mkdir(exist_ok=True)
    symlink_dst = base_path.joinpath("latest").as_posix()
    update_symlink = ["ln", "-snf", output_dir_by_time.as_posix(), symlink_dst]
    subprocess.run(update_symlink, check=False)
    return output_dir_by_time


def args_to_dict(launch_args: list[str]) -> dict[str, str]:
    launch_arg_dict = {}
    for l_arg in launch_args:
        try:
            key, value = l_arg.split(":=")
            launch_arg_dict[key] = value
        except ValueError:
            # invalid argument
            termcolor.cprint(
                f"{l_arg} is ignored because it is invalid",
                "red",
            )
    return launch_arg_dict


def launch_dict_to_str(launch_arg_dict: dict) -> str:
    rtn_str = ""
    for k, v in launch_arg_dict.items():
        if isinstance(v, str) and ("{" in v or "[" in v):
            rtn_str += f" '{k}:={v}'"
        else:
            rtn_str += f" {k}:={v}"
    return rtn_str


def clean_up_cmd() -> str:
    # echo return value of ros2 launch (0: ok, others: ng)
    # kill zombie ros2 process
    # kill rviz
    # sleep 1 sec
    # new line
    return """
echo \"exit status: $?\"
pgrep ros | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep ros | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
sleep 1
"""


def create_launch_cmd(
    scenario_path: Path,
    output_dir: Path,
    launch_args_dict: dict[str, str],
) -> str | None:
    launch_command_for_all_dataset = ""
    with scenario_path.open("r") as f:
        yaml_obj = yaml.safe_load(f)

    dataset_count = len(yaml_obj["Evaluation"]["Datasets"])
    for dataset_index in range(dataset_count):
        output_dataset = output_dir.joinpath(str(dataset_index))
        output_dataset.mkdir()
        launch_command = "ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py"
        launch_arg_dict_dataset = {
            "scenario_path": scenario_path.as_posix(),
            "output_dir": output_dataset.as_posix(),
            "dataset_index": dataset_index,
        }
        launch_arg_dict_dataset.update(launch_args_dict)
        launch_command += launch_dict_to_str(launch_arg_dict_dataset) + "\n"
        launch_command += clean_up_cmd()
        launch_command_for_all_dataset += launch_command
    return launch_command_for_all_dataset
