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


def _load_tuned_release_params(tuned_params: Path) -> dict[str, Any]:
    """tuned_params.yaml から release 対象パラメータを検証付きで読み込む。"""
    if not tuned_params.is_file():
        raise FileNotFoundError(f"Tuned parameters file not found: {tuned_params}")
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
    return params


def _load_case_release_params(scenario: Path, model_name: str) -> dict[str, Any]:
    """scenario ケースのパラメータを rollout 既定値へマージして完全な param 集合にする。

    scenario ケースは vel_lim 等の global 制限値を持たないため、評価パイプラインと
    同じ既定 (rollout.build_params) を土台にマージする。
    """
    from ..lib._vehicle_models import merge_vehicle_model_params  # noqa: PLC0415
    from . import rollout  # noqa: PLC0415 (遅延 import で release の基本経路を軽く保つ)
    from .model_config import load_model_config  # noqa: PLC0415

    case = load_model_config(scenario).find_case(model_name)
    return merge_vehicle_model_params(
        rollout.build_params(), dict(case.params), TARGET_MODEL_TYPE,
    )


def release(
    input_param: Path, tuned_params: Path, out_dir: Path, *, scenario: Path | None = None,
) -> Path:
    """同定/指定パラメータを target model の global 値とバージョンスロットへ反映する。

    scenario の ``Evaluation.Conditions.release`` ({model, version}) が指定されている
    場合は、そのケースのパラメータを ``v{version}`` スロットに書き ``version`` で選択する
    (tuned の v100 は書かない)。未指定時は従来どおり tuned → ``v100``。
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    release_spec = None
    if scenario is not None:
        from .model_config import load_model_config  # noqa: PLC0415

        release_spec = load_model_config(scenario).release

    if release_spec is not None:
        params = _load_case_release_params(scenario, release_spec.model)
        version = release_spec.version
        print(
            f"[INFO] release: scenario ケース '{release_spec.model}' を "
            f"v{version} としてリリースします (tuned は含めません)"
        )
    else:
        params = _load_tuned_release_params(tuned_params)
        version = 100

    param_data, ros_params = _load_input_document(Path(input_param))
    model_params = ros_params[RELEASE_MODEL_KEY]
    spec = get_vehicle_model_spec(TARGET_MODEL_TYPE)
    required = spec.namespaced_param_keys | _GLOBAL_PARAM_KEYS.keys()
    missing = required - params.keys()
    if missing:
        raise ValueError(f"Release params are missing target model keys: {sorted(missing)}")

    release_slot = {key: params[key] for key in sorted(spec.namespaced_param_keys)}

    # 指定リリースでは既存の確定バージョン (入力 YAML の v1 等) を黙って潰さない。
    # ただし同一内容の再リリース (リリース適用済み入力でのパイプライン再実行) は
    # 何も変えないため冪等として許可する。tuned の v100 は候補スロットとして従来どおり上書き可。
    if release_spec is not None:
        existing_slot = model_params.get(f"v{version}")
        if existing_slot is not None and existing_slot != release_slot:
            raise ValueError(
                f"Input already contains 'v{version}' with different values; "
                "choose an unused release.version."
            )

    for model_key, ros_key in _GLOBAL_PARAM_KEYS.items():
        ros_params[ros_key] = params[model_key]

    model_params["version"] = version
    model_params[f"v{version}"] = release_slot

    out_file = out_dir / "simulator_model.param.yaml"
    with out_file.open("w", encoding="utf-8") as f:
        yaml.safe_dump(param_data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    print(f"[INFO] Successfully generated release model parameter at: {out_file}")
    return out_file
