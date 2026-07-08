"""scenario.yaml から車両ホイールベースを読む薄いヘルパー。"""
from __future__ import annotations

from pathlib import Path

import yaml

from ..lib._params_utils import load_sim_params
from .settings import DEFAULT_WHEELBASE


def resolve_wheelbase(scenario: Path | None, case: str) -> float:
    """指定 case の wheelbase を返す。未指定なら車両既定値を使う。"""
    if scenario is not None and Path(scenario).exists():
        with Path(scenario).open("r") as f:
            scen = yaml.safe_load(f)
        models = scen.get("Evaluation", {}).get("Conditions", {}).get("models", {})
        wb = models.get(case, {}).get("wheelbase")
        if wb:
            return float(wb)
    base = load_sim_params()
    return float(base.get("wheelbase", base.get("wheel_base", DEFAULT_WHEELBASE)))
