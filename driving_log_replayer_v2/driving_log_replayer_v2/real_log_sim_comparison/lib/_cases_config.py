"""cases の dataclass + loader (scenario.yaml 経由).

設定は scenario.yaml の Conditions.models / cases / overlay に統合された。
load_cases_config() は scenario.yaml のパスを受け取り、_models_config.load_models_doc()
に委譲して CasesConfig を構築する。

schema (scenario.yaml の Conditions ブロック):

    models:
      <name>:
        vehicle_model_type: <str>      # open-loop VehicleModel クラス名 (必須)
        params: {<key>: <scalar>}      # load_sim_params() の base に上書きする dict (任意)
        # vehicle_model / sim 専用フィールドは open-loop では無視
      ...

    cases: [<name>, ...]               # vehicle_model_type が必須
    overlay:
      reference_tag: <str>
      plots: [cascade_error, error_timeseries]
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ._models_config import OverlaySpec, load_models_doc


@dataclass
class CaseSpec:
    """1 ケースの定義 (open-loop VehicleModel 解析用)."""

    tag: str
    vehicle_model_type: str            # VehicleModel クラス名 (旧フィールド名: vehicle_model)
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class CasesConfig:
    cases: list[CaseSpec]
    overlay: OverlaySpec = field(default_factory=OverlaySpec)

    def find_case(self, tag: str) -> CaseSpec:
        for c in self.cases:
            if c.tag == tag:
                return c
        raise KeyError(
            f"tag={tag!r} が Conditions.cases に見つかりません。"
            f"定義済 tag: {[c.tag for c in self.cases]}"
        )

    @property
    def tags(self) -> list[str]:
        return [c.tag for c in self.cases]


def load_cases_config(scenario_path: str | Path) -> CasesConfig:
    """scenario.yaml を読み込み CasesConfig を返す。

    Args:
        scenario_path: scenario.yaml のパス (Conditions.models / cases / overlay を含む)

    Returns:
        CasesConfig: cases リストと overlay 設定

    Raises:
        FileNotFoundError: scenario.yaml が存在しない
        ValueError: models / cases の定義が不正
    """
    doc = load_models_doc(scenario_path)
    cases: list[CaseSpec] = []
    for name in doc.cases_list:
        m = doc.models[name]
        # vehicle_model_type は load_models_doc で None でないことが保証済み
        cases.append(CaseSpec(
            tag=name,
            vehicle_model_type=m.vehicle_model_type,  # type: ignore[arg-type]
            params=dict(m.params),
        ))
    return CasesConfig(cases=cases, overlay=doc.overlay)
