from pathlib import Path

import simplejson as json
import termcolor


def load_last_result(result_path: Path) -> dict:
    if not result_path.exists():
        return {}

    try:
        with result_path.open() as jsonl_file:
            # Read the last line efficiently
            for line in jsonl_file:  # noqa
                pass
            return json.loads(line)
    except (json.JSONDecodeError, UnboundLocalError):
        return {}


def load_final_metrics(result_path: Path) -> dict:
    return load_last_result(result_path).get("Frame", {}).get("FinalMetrics", {})


def display(result_path: Path) -> None:
    print("--------------------------------------------------")  # noqa
    last_result = load_last_result(result_path)
    if last_result:
        # dict is not empty
        if last_result["Result"]["Success"]:
            result = "TestResult: Passed"
            color = "green"
        else:
            result = "TestResult: Failed"
            color = "red"
        termcolor.cprint(result, color)
        termcolor.cprint(last_result["Result"]["Summary"], color)


def display_all(output_directory: Path, target_dir_name: str) -> None:
    result_paths = output_directory.glob(f"**/{target_dir_name}/**/result.jsonl")
    for result_path in result_paths:
        termcolor.cprint(f"{result_path.as_posix()}", "white")
        display(result_path)
