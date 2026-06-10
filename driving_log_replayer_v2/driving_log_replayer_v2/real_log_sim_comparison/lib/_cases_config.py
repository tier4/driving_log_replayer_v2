"""cases.yaml の dataclass + loader.

cases.yaml は VehicleModel 解析 (Stage 3) を複数ケースで回すための定義ファイル。
各ケースは (vehicle_model タイプ, params dict) の組で識別され、tag で出力先を分ける。
Stage 4 (step6_analyze_cases) は overlay セクションを参照する。

schema:

    cases:
      - tag: <str>                       # 出力ディレクトリ名 nstep/<tag>/
        vehicle_model: <str>             # "delay_steer_acc_geared_wo_fall_guard" | "ideal_steer_acc"
        params:                          # load_sim_params() の base に上書きする dict
          wheelbase: 4.76012
          steer_bias: 0.01
          ...
    overlay:
      reference_tag: <str>               # 基準ケース (任意)
      plots:                             # 重ね描き対象の nstep plot 名リスト
        - cascade_error
        - error_timeseries
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
import warnings
from typing import Any

import yaml


SUPPORTED_MODEL_TYPES: frozenset[str] = frozenset(
    {"delay_steer_acc_geared_wo_fall_guard", "ideal_steer_acc", "taiga_dyn", "taiga_x"}
)

# vm_create_* / VehicleModel.__init__ が消費する物理パラメータ名。typo 検出用。
_KNOWN_PARAM_KEYS: frozenset[str] = frozenset({
    "vel_lim", "steer_lim", "vel_rate_lim", "steer_rate_lim",
    "wheelbase", "acc_time_delay", "acc_time_constant",
    "steer_time_delay", "steer_time_constant",
    "steer_dead_band", "steer_bias",
    "debug_acc_scaling_factor", "debug_steer_scaling_factor",
    "k_us", "sub_dt",
    # taiga_dyn / taiga_x dynamic-bicycle physical parameters
    "mass", "inertia_z", "lf", "lr",
    "cornering_stiffness_front", "cornering_stiffness_rear", "vx_min_dyn",
    # taiga_x (PhysX backend) specific parameters
    "track_width", "cg_offset_x", "max_accel", "max_brake", "wheel_radius",
    "taiga_x_fixed_dt",
})

# tag は OUT_DIR = .../nstep/<tag>/ に直接連結されるため安全文字のみ許可。
_TAG_PATTERN = re.compile(r"^[A-Za-z0-9_\-.]+$")


@dataclass
class CaseSpec:
    """1 ケースの定義 (cases.yaml の cases[i])."""

    tag: str
    vehicle_model: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class OverlaySpec:
    """cases.yaml の overlay セクション."""

    reference_tag: str | None = None
    plots: list[str] = field(default_factory=lambda: ["cascade_error", "error_timeseries"])


@dataclass
class CasesConfig:
    cases: list[CaseSpec]
    overlay: OverlaySpec = field(default_factory=OverlaySpec)

    def find_case(self, tag: str) -> CaseSpec:
        for c in self.cases:
            if c.tag == tag:
                return c
        raise KeyError(
            f"tag={tag!r} が cases.yaml に見つかりません。"
            f"定義済 tag: {[c.tag for c in self.cases]}"
        )

    @property
    def tags(self) -> list[str]:
        return [c.tag for c in self.cases]


def load_cases_config(path: str | Path) -> CasesConfig:
    """cases.yaml を読み込み CasesConfig を返す。

    必須: cases (≥1 要素、各 tag は一意 / 安全文字のみ、vehicle_model は SUPPORTED_MODEL_TYPES)。
    任意: overlay (省略時はデフォルト = cascade_error + error_timeseries を重ね描き)。
    typo 検出: params に _KNOWN_PARAM_KEYS 外のキーがあれば warn (黙って無視を防ぐ)。
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"cases.yaml が見つかりません: {p}")

    with p.open(encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}

    raw_cases = doc.get("cases")
    if not raw_cases or not isinstance(raw_cases, list):
        raise ValueError(f"{p}: 'cases:' が空またはリストではありません")

    cases: list[CaseSpec] = []
    seen_tags: set[str] = set()
    for i, entry in enumerate(raw_cases):
        if not isinstance(entry, dict):
            raise ValueError(f"{p}: cases[{i}] が dict ではありません")
        tag = entry.get("tag")
        model = entry.get("vehicle_model")
        if not tag or not isinstance(tag, str):
            raise ValueError(f"{p}: cases[{i}].tag が未指定または文字列ではありません")
        if not _TAG_PATTERN.match(tag):
            raise ValueError(
                f"{p}: cases[{i}].tag={tag!r} は安全文字 [A-Za-z0-9_.-] のみ許可 "
                "(path traversal / 不正ディレクトリ名を防ぐ)"
            )
        if tag in seen_tags:
            raise ValueError(f"{p}: tag={tag!r} が重複しています")
        seen_tags.add(tag)
        if not model or model not in SUPPORTED_MODEL_TYPES:
            raise ValueError(
                f"{p}: cases[{i}].vehicle_model={model!r} は未対応。"
                f"対応: {sorted(SUPPORTED_MODEL_TYPES)}"
            )
        raw_params = entry.get("params")
        if raw_params is None:
            params = {}
        elif not isinstance(raw_params, dict):
            raise ValueError(f"{p}: cases[{i}].params が dict ではありません")
        else:
            params = dict(raw_params)
        unknown = set(params.keys()) - _KNOWN_PARAM_KEYS
        if unknown:
            warnings.warn(
                f"{p}: cases[{i}].params に未知キー {sorted(unknown)} が含まれます "
                f"(typo の可能性。既知キー: {sorted(_KNOWN_PARAM_KEYS)})"
            )
        cases.append(CaseSpec(tag=tag, vehicle_model=model, params=params))

    overlay_raw = doc.get("overlay") or {}
    overlay = OverlaySpec(
        reference_tag=overlay_raw.get("reference_tag"),
        plots=list(overlay_raw.get("plots") or ["cascade_error", "error_timeseries"]),
    )

    return CasesConfig(cases=cases, overlay=overlay)
