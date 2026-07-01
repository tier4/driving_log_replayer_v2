#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
import yaml

def main():
    parser = argparse.ArgumentParser(
        description="Apply tuned vehicle model parameters to scenario.yaml"
    )
    parser.add_argument("--scenario", required=True, type=Path, help="Input scenario.yaml")
    parser.add_argument("--params", required=True, type=Path, help="Input tuned_params.yaml")
    parser.add_argument("--out", type=Path, help="Output scenario.yaml (default: overwrite input)")
    parser.add_argument("--model", default="best_normal", help="Model name to apply parameters to")
    args = parser.parse_args()

    if not args.scenario.is_file():
        print(f"ERROR: Scenario file not found: {args.scenario}", file=sys.stderr)
        sys.exit(1)

    if not args.params.is_file():
        print(f"ERROR: Tuned params file not found: {args.params}", file=sys.stderr)
        sys.exit(1)

    # Load YAML files
    try:
        with open(args.scenario, "r", encoding="utf-8") as f:
            scenario = yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load scenario YAML: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        with open(args.params, "r", encoding="utf-8") as f:
            params_data = yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load tuned params YAML: {e}", file=sys.stderr)
        sys.exit(1)

    # Extraction of params dict
    tuned_params = params_data.get("params")
    if not isinstance(tuned_params, dict):
        print("ERROR: Tuned params YAML must contain a 'params' dictionary.", file=sys.stderr)
        sys.exit(1)

    # Locate models section
    try:
        models = scenario["Evaluation"]["Conditions"]["models"]
    except KeyError:
        print("ERROR: Could not find Evaluation.Conditions.models in scenario", file=sys.stderr)
        sys.exit(1)

    # Update main model (e.g. best_normal)
    if args.model in models:
        if not isinstance(models[args.model], dict):
            models[args.model] = {}
        if "params" not in models[args.model] or not isinstance(models[args.model]["params"], dict):
            models[args.model]["params"] = {}
        
        # Apply parameters
        for k, v in tuned_params.items():
            models[args.model]["params"][k] = v
        print(f"[INFO] Applied tuned parameters to '{args.model}': {tuned_params}")
    else:
        print(f"[WARN] Target model '{args.model}' not found in scenario.yaml models registry")

    # Update best_normal_substep if present
    substep_model = f"{args.model}_substep"
    if substep_model in models:
        if not isinstance(models[substep_model], dict):
            models[substep_model] = {}
        if "params" not in models[substep_model] or not isinstance(models[substep_model]["params"], dict):
            models[substep_model]["params"] = {}
        
        # Apply parameters while preserving n_substep if it exists
        n_substep = models[substep_model]["params"].get("n_substep", 10)
        for k, v in tuned_params.items():
            models[substep_model]["params"][k] = v
        models[substep_model]["params"]["n_substep"] = n_substep
        print(f"[INFO] Applied tuned parameters to '{substep_model}' (n_substep={n_substep})")

    # Write output
    out_path = args.out if args.out else args.scenario
    out_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(scenario, f, allow_unicode=True, sort_keys=False)
        print(f"[INFO] Wrote updated scenario to: {out_path}")
    except Exception as e:
        print(f"ERROR: Failed to save scenario YAML: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
