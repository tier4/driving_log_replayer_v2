"""Minimal model configuration used by the reidentify pipeline."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re

import yaml

from ..lib._accel_source import normalize_accel_source
from ..lib._steer_source import normalize_steer_source
from ..lib._vehicle_models import SUPPORTED_VMT
from .settings import BASELINE_MODEL_NAME, TARGET_MODEL_NAME, TARGET_MODEL_TYPE

_SAFE_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")


@dataclass(frozen=True)
class ModelSpec:
    name: str
    vehicle_model_type: str
    acceleration_source: str
    steering_source: str
    params: dict


@dataclass(frozen=True)
class ModelConfig:
    models: dict[str, ModelSpec]
    comparison_models: tuple[str, ...]

    def find_case(self, name: str) -> ModelSpec:
        try:
            return self.models[name]
        except KeyError as exc:
            raise ValueError(
                f"model {name!r} が scenario にありません。定義済み: {sorted(self.models)}"
            ) from exc


def load_model_config(path: str | Path) -> ModelConfig:
    """Load model data and the physical-validity comparison contract."""
    scenario = Path(path)
    if not scenario.is_file():
        raise FileNotFoundError(f"scenario が見つかりません: {scenario}")
    document = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{scenario}: top-level は mapping である必要があります")
    evaluation = document.get("Evaluation")
    if not isinstance(evaluation, dict):
        raise ValueError(f"{scenario}: Evaluation は mapping である必要があります")
    conditions = evaluation.get("Conditions")
    if not isinstance(conditions, dict):
        raise ValueError(f"{scenario}: Evaluation.Conditions は mapping である必要があります")
    raw_models = conditions.get("models")
    if not isinstance(raw_models, dict) or not raw_models:
        raise ValueError(f"{scenario}: Evaluation.Conditions.models が必要です")

    models: dict[str, ModelSpec] = {}
    for raw_name, raw_spec in raw_models.items():
        name = str(raw_name)
        if not _SAFE_NAME.fullmatch(name):
            raise ValueError(f"{scenario}: 不正な model 名です: {name!r}")
        if not isinstance(raw_spec, dict):
            raise ValueError(f"{scenario}: models.{name} は mapping である必要があります")
        model_type = str(raw_spec.get("vehicle_model_type") or "")
        if model_type not in SUPPORTED_VMT:
            raise ValueError(
                f"{scenario}: models.{name}.vehicle_model_type={model_type!r} は未対応です"
            )
        params = raw_spec.get("params") or {}
        if not isinstance(params, dict):
            raise ValueError(f"{scenario}: models.{name}.params は mapping である必要があります")
        models[name] = ModelSpec(
            name=name,
            vehicle_model_type=model_type,
            acceleration_source=normalize_accel_source(
                raw_spec.get("acceleration_source"), default="accel"
            ),
            steering_source=normalize_steer_source(
                raw_spec.get("steering_source"), default="steer"
            ),
            params=dict(params),
        )

    missing = {BASELINE_MODEL_NAME, TARGET_MODEL_NAME} - models.keys()
    if missing:
        raise ValueError(f"{scenario}: models に必須定義がありません: {sorted(missing)}")
    target = models[TARGET_MODEL_NAME]
    if target.vehicle_model_type != TARGET_MODEL_TYPE:
        raise ValueError(
            f"{scenario}: models.{TARGET_MODEL_NAME}.vehicle_model_type は "
            f"{TARGET_MODEL_TYPE!r} である必要があります"
        )
    for name in (BASELINE_MODEL_NAME, TARGET_MODEL_NAME):
        wheelbase = models[name].params.get("wheelbase")
        valid_wheelbase = (
            isinstance(wheelbase, (int, float))
            and not isinstance(wheelbase, bool)
            and math.isfinite(float(wheelbase))
            and float(wheelbase) > 0.0
        )
        if not valid_wheelbase:
            raise ValueError(f"{scenario}: models.{name}.params.wheelbase に正の数値が必要です")
    raw_comparison_models = conditions.get("comparison_models")
    if not isinstance(raw_comparison_models, list) or not raw_comparison_models:
        raise ValueError(f"{scenario}: Evaluation.Conditions.comparison_models が必要です")
    comparison_models = tuple(str(name) for name in raw_comparison_models)
    if len(set(comparison_models)) != len(comparison_models):
        raise ValueError(f"{scenario}: comparison_models に重複があります")
    undefined = [name for name in comparison_models if name not in models]
    if undefined:
        raise ValueError(f"{scenario}: comparison_models に未定義モデルがあります: {undefined}")
    comparison_missing = {BASELINE_MODEL_NAME, TARGET_MODEL_NAME} - set(comparison_models)
    if comparison_missing:
        raise ValueError(
            f"{scenario}: comparison_models に必須モデルがありません: {sorted(comparison_missing)}"
        )
    return ModelConfig(models=models, comparison_models=comparison_models)


def resolve_baseline_model(config: ModelConfig) -> tuple[str, dict, str]:
    baseline = config.models[BASELINE_MODEL_NAME]
    return baseline.vehicle_model_type, dict(baseline.params), baseline.name
