#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
import yaml

def main():
    parser = argparse.ArgumentParser(
        description="Generate release model parameters by merging tuned params into simulator_model.param.yaml"
    )
    parser.add_argument("--input-param", required=True, type=Path, help="Original simulator_model.param.yaml")
    parser.add_argument("--tuned-params", required=True, type=Path, help="Tuned parameters YAML (tuned_params.yaml)")
    parser.add_argument("--out-dir", required=True, type=Path, help="Output directory to copy and update the yaml")
    args = parser.parse_args()

    if not args.input_param.is_file():
        print(f"ERROR: Input parameter file not found: {args.input_param}", file=sys.stderr)
        sys.exit(1)

    if not args.tuned_params.is_file():
        print(f"ERROR: Tuned parameters file not found: {args.tuned_params}", file=sys.stderr)
        sys.exit(1)

    if not args.out_dir.is_dir():
        print(f"ERROR: Output directory not found: {args.out_dir}", file=sys.stderr)
        sys.exit(1)

    # Output file path
    out_file = args.out_dir / args.input_param.name

    # Load tuned parameters
    try:
        with open(args.tuned_params, "r", encoding="utf-8") as f:
            tuned_data = yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load tuned params: {e}", file=sys.stderr)
        sys.exit(1)

    tuned_params = tuned_data.get("params")
    if not isinstance(tuned_params, dict):
        print("ERROR: Tuned params YAML must contain a 'params' dictionary.", file=sys.stderr)
        sys.exit(1)

    # Load original parameter file
    try:
        with open(args.input_param, "r", encoding="utf-8") as f:
            param_data = yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load input parameter file: {e}", file=sys.stderr)
        sys.exit(1)

    # Verify and modify parameters
    try:
        ros_params = param_data["/**"]["ros__parameters"]
    except KeyError:
        print("ERROR: Could not find '/**' -> 'ros__parameters' in the input YAML.", file=sys.stderr)
        sys.exit(1)

    model_key = "delay_steer_acc_geared_for_diffusion_planner"
    if model_key not in ros_params:
        print(f"ERROR: Could not find '{model_key}' in ros__parameters.", file=sys.stderr)
        sys.exit(1)

    model_params = ros_params[model_key]
    if not isinstance(model_params, dict):
        print(f"ERROR: '{model_key}' is not a dictionary.", file=sys.stderr)
        sys.exit(1)

    # Update version to 100
    model_params["version"] = 100
    # Add v100
    model_params["v100"] = tuned_params

    # Write output file
    try:
        with open(out_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(param_data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        print(f"[INFO] Successfully generated release model parameter at: {out_file}")
    except Exception as e:
        print(f"ERROR: Failed to write output parameter file: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
