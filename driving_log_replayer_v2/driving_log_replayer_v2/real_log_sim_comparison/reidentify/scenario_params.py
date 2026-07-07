"""scenario.yaml から車両ホイールベースを読む薄いヘルパー (ROS フリー)。

scenario.yaml の models 定義全体の検証は `lib/_models_config.py` (SSOT) が
担うが、fit_steer.py が必要とするのは 1 ケースの wheelbase だけなので、
physical_tuning.py と同じ寛容な (フォールバックチェーン付き) 読み方をここに
切り出す。wheelbase の最終フォールバックは `lib/_params_utils.load_sim_params()`
(j6_gen2_description/config を実読込する ROS フリーな SSOT)。
"""
from __future__ import annotations

from pathlib import Path

import yaml

from ..lib._params_utils import load_sim_params

_CASE_FALLBACK_ORDER = ("current", "best_normal", "case_normal", "normal", "baseline")


def resolve_wheelbase(scenario: Path | None, case: str) -> float:
    """`case` → `_CASE_FALLBACK_ORDER` の順に scenario.yaml の wheelbase を探し、
    見つからなければ vehicle_info.param.yaml の既定値にフォールバックする。
    """
    if scenario is not None and Path(scenario).exists():
        with Path(scenario).open("r") as f:
            scen = yaml.safe_load(f)
        models = scen.get("Evaluation", {}).get("Conditions", {}).get("models", {})
        for name in (case, *_CASE_FALLBACK_ORDER):
            wb = models.get(name, {}).get("wheelbase")
            if wb:
                return float(wb)
    base = load_sim_params()
    return float(base.get("wheelbase", base.get("wheel_base", 4.76012)))
