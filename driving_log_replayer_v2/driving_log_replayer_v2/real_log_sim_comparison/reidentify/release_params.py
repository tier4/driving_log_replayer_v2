#!/usr/bin/env python3
"""同定済みパラメータのリリース用 YAML 生成。"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from ..lib._vehicle_models import get_vehicle_model_spec
from .settings import RELEASE_MODEL_KEY
from .settings import TARGET_MODEL_TYPE

_GLOBAL_PARAM_KEYS = {
    "vel_lim": "vel_lim",
    "vel_rate_lim": "vel_rate_lim",
    "steer_lim": "steer_lim",
    "steer_rate_lim": "steer_rate_lim",
    "wheelbase": "wheel_base",
}


def _load_input_document(input_param: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    if not input_param.is_file():
        raise FileNotFoundError(f"Input parameter file not found: {input_param}")
    document = yaml.safe_load(input_param.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("Input parameter YAML must contain a mapping.")
    try:
        ros_params = document["/**"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise KeyError("Could not find '/**' -> 'ros__parameters' in the input YAML.") from exc
    if not isinstance(ros_params, dict):
        raise ValueError("'/**' -> 'ros__parameters' must be a mapping.")

    spec = get_vehicle_model_spec(TARGET_MODEL_TYPE)
    if ros_params.get("vehicle_model_type") != spec.sim_enum:
        raise ValueError(
            f"vehicle_model_type must be {spec.sim_enum!r} for reidentify release."
        )
    model_params = ros_params.get(RELEASE_MODEL_KEY)
    if not isinstance(model_params, dict):
        raise KeyError(f"Could not find mapping '{RELEASE_MODEL_KEY}' in ros__parameters.")
    return document, ros_params


def validate_input(input_param: Path) -> None:
    """Fail fast when the release base is not the fixed target model YAML."""
    _load_input_document(Path(input_param))


def release(input_param: Path, tuned_params: Path, out_dir: Path) -> Path:
    """評価済みパラメータを target model の global 値と ``v100`` に反映する。"""
    if not tuned_params.is_file():
        raise FileNotFoundError(f"Tuned parameters file not found: {tuned_params}")
    out_dir.mkdir(parents=True, exist_ok=True)

    tuned_data = yaml.safe_load(tuned_params.read_text(encoding="utf-8"))
    if not isinstance(tuned_data, dict):
        raise ValueError("Tuned params YAML must contain a mapping.")
    params = tuned_data.get("params")
    if not isinstance(params, dict):
        raise ValueError("Tuned params YAML must contain a 'params' dictionary.")
    metadata = tuned_data.get("metadata") or {}
    if metadata.get("vehicle_model_type") != TARGET_MODEL_TYPE:
        raise ValueError(
            f"Tuned params must target vehicle_model_type={TARGET_MODEL_TYPE!r}."
        )

    param_data, ros_params = _load_input_document(Path(input_param))
    model_params = ros_params[RELEASE_MODEL_KEY]
    spec = get_vehicle_model_spec(TARGET_MODEL_TYPE)
    required = spec.namespaced_param_keys | _GLOBAL_PARAM_KEYS.keys()
    missing = required - params.keys()
    if missing:
        raise ValueError(f"Tuned params are missing target model keys: {sorted(missing)}")

    for model_key, ros_key in _GLOBAL_PARAM_KEYS.items():
        ros_params[ros_key] = params[model_key]

    model_params["version"] = 100
    model_params["v100"] = {
        key: params[key] for key in sorted(spec.namespaced_param_keys)
    }

    out_file = out_dir / "simulator_model.param.yaml"
    with out_file.open("w", encoding="utf-8") as f:
        yaml.safe_dump(param_data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    print(f"[INFO] Successfully generated release model parameter at: {out_file}")
    return out_file
