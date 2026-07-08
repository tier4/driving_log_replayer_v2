#!/usr/bin/env python3
"""[Step4c] 同定済みパラメータのリリース用 YAML 生成。"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

from .settings import RELEASE_MODEL_KEY


def release(input_param: Path, tuned_params: Path, out_dir: Path) -> Path:
    """`tuned_params.yaml` の params を `input_param` の `v100` として書き込む。"""
    if not input_param.is_file():
        raise FileNotFoundError(f"Input parameter file not found: {input_param}")
    if not tuned_params.is_file():
        raise FileNotFoundError(f"Tuned parameters file not found: {tuned_params}")
    out_dir.mkdir(parents=True, exist_ok=True)

    with tuned_params.open("r", encoding="utf-8") as f:
        tuned_data = yaml.safe_load(f)
    params = tuned_data.get("params")
    if not isinstance(params, dict):
        raise ValueError("Tuned params YAML must contain a 'params' dictionary.")

    with input_param.open("r", encoding="utf-8") as f:
        param_data = yaml.safe_load(f)

    try:
        ros_params = param_data["/**"]["ros__parameters"]
    except KeyError as exc:
        raise KeyError("Could not find '/**' -> 'ros__parameters' in the input YAML.") from exc
    if RELEASE_MODEL_KEY not in ros_params:
        raise KeyError(f"Could not find '{RELEASE_MODEL_KEY}' in ros__parameters.")

    model_params = ros_params[RELEASE_MODEL_KEY]
    if not isinstance(model_params, dict):
        raise ValueError(f"'{RELEASE_MODEL_KEY}' is not a dictionary.")

    model_params["version"] = 100
    model_params["v100"] = params

    out_file = out_dir / input_param.name
    with out_file.open("w", encoding="utf-8") as f:
        yaml.safe_dump(param_data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    print(f"[INFO] Successfully generated release model parameter at: {out_file}")
    return out_file


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate release model parameters by merging tuned params into simulator_model.param.yaml"
    )
    parser.add_argument("--input-param", required=True, type=Path)
    parser.add_argument("--tuned-params", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    try:
        release(args.input_param, args.tuned_params, args.out_dir)
    except Exception as e:  # noqa: BLE001
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
