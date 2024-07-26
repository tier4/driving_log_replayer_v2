import argparse
from pathlib import Path

import yaml


def convert_scenario(scenario_path: Path) -> None:
    scenario_file = scenario_path.open("r+")
    yaml_obj = yaml.safe_load(scenario_file)

    if yaml_obj["Evaluation"]["UseCaseName"] != "annotationless_perception":
        print(f"{scenario_path} does not require conversion")  # noqa
        scenario_file.close()
        return

    if (
        yaml_obj["ScenarioFormatVersion"] != "3.0.0"
        or yaml_obj["Evaluation"]["UseCaseFormatVersion"] != "1.0.0"
    ):
        print(f"{scenario_path} does not match format version")  # noqa
        scenario_file.close()
        return

    yaml_obj["Evaluation"]["UseCaseFormatVersion"] = "2.0.0"

    class_cond = yaml_obj["Evaluation"]["Conditions"]["ClassConditions"]

    for k, v in class_cond.items():
        pass_range_dict = expand_pass_range(v["PassRange"])
        for name, value in v["Threshold"].items():
            threshold_values = {}
            for m_k, m_v in value.items():
                threshold_values[m_k] = {
                    "lower": m_v * pass_range_dict[m_k][0],
                    "upper": m_v * pass_range_dict[m_k][1],
                }
            class_cond[k] |= {name: threshold_values}
        class_cond[k].pop("Threshold")
        class_cond[k].pop("PassRange")

    # 既存の内容を消す
    scenario_file.seek(0)
    scenario_file.truncate()

    # 更新済みの内容を書き込む
    yaml.safe_dump(yaml_obj, scenario_file, sort_keys=False)
    scenario_file.close()


def expand_pass_range(pass_range_dict: dict) -> dict:
    rtn_dict = {}
    for k, v in pass_range_dict.items():
        s_lower, s_upper = v.split("-")
        rtn_dict[k] = [float(s_lower), float(s_upper)]
    return rtn_dict


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "data_directory",
        help="data_directory of log_evaluator",
    )
    args = parser.parse_args()
    scenario_paths = Path(args.data_directory).resolve().glob("**/scenario*.y*ml")  # yaml or yml
    for scenario_path in sorted(scenario_paths):
        print(f"convert {scenario_path}")  # noqa
        convert_scenario(scenario_path)


if __name__ == "__main__":
    main()
