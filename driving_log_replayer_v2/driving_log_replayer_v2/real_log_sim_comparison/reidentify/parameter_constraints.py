"""Physical parameter constraints shared by the reidentification merge step."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

FIT_LON = "fit_lon"
FIT_STEER = "fit_steer"
FIT_XY = "fit_xy"
FIT_MERGE = "fit_merge"


@dataclass(frozen=True)
class ParameterConstraint:
    """A physical domain and, when applicable, an Optuna search domain."""

    name: str
    reason: str
    bounds: tuple[float | None, float | None] | None = None
    minimum_inclusive: bool = True
    maximum_inclusive: bool = True
    search_bounds: tuple[float, float] | None = None
    search_candidates: tuple[float, ...] | None = None
    direct_fit_bounds: tuple[float, float] | None = None
    direct_fit_candidates: tuple[float, ...] | None = None
    default: float | None = None
    optimization_stages: frozenset[str] = frozenset()

    def __post_init__(self) -> None:
        if self.search_bounds is not None and self.search_candidates is not None:
            raise ValueError(f"{self.name}: search_bounds と search_candidates は併用できません")
        if self.search_bounds is not None and self.bounds is not None:
            raise ValueError(f"{self.name}: search_bounds と bounds は二重指定できません")
        if self.search_candidates is not None and self.bounds is not None:
            raise ValueError(f"{self.name}: search_candidates と bounds は二重指定できません")
        if self.search_candidates is not None and not self.search_candidates:
            raise ValueError(f"{self.name}: search_candidates は空にできません")
        if self.direct_fit_bounds is not None and self.direct_fit_candidates is not None:
            raise ValueError(
                f"{self.name}: direct_fit_bounds と direct_fit_candidates は併用できません"
            )
        if self.direct_fit_candidates is not None and not self.direct_fit_candidates:
            raise ValueError(f"{self.name}: direct_fit_candidates は空にできません")
        if self.default is not None and not self.contains(self.default):
            raise ValueError(f"{self.name}: default が許容域 {self.range_text()} 外です")

    @property
    def allowed_bounds(self) -> tuple[float | None, float | None]:
        """Return the single source of truth for the accepted numeric interval."""
        if self.search_bounds is not None:
            return self.search_bounds
        if self.search_candidates is not None:
            return min(self.search_candidates), max(self.search_candidates)
        return self.bounds or (None, None)

    def contains(self, value: float) -> bool:
        if not math.isfinite(value):
            return False
        minimum, maximum = self.allowed_bounds
        if minimum is not None:
            if value < minimum or (value == minimum and not self.minimum_inclusive):
                return False
        if maximum is not None:
            if value > maximum or (value == maximum and not self.maximum_inclusive):
                return False
        return True

    def clamp(self, value: float) -> float:
        """Clamp a numeric search candidate into this physical domain."""
        if not math.isfinite(value):
            raise ValueError(f"{self.name} の探索候補が有限数値ではありません: {value!r}")
        result = value
        minimum, maximum = self.allowed_bounds
        if minimum is not None and result < minimum:
            result = minimum
        if maximum is not None and result > maximum:
            result = maximum
        if not self.contains(result):
            raise ValueError(
                f"{self.name} の物理範囲 {self.range_text()} に clamp できません: {value!r}"
            )
        return result

    def range_text(self) -> str:
        left = "[" if self.minimum_inclusive else "("
        right = "]" if self.maximum_inclusive else ")"
        minimum, maximum = self.allowed_bounds
        lower = "-∞" if minimum is None else f"{minimum:g}"
        upper = "∞" if maximum is None else f"{maximum:g}"
        return f"{left}{lower}, {upper}{right}"

    def audit_record(self, value: Any) -> dict[str, Any]:
        record: dict[str, Any] = {
            "value": value,
            "allowed_range": {
                "minimum": self.allowed_bounds[0],
                "maximum": self.allowed_bounds[1],
                "minimum_inclusive": self.minimum_inclusive,
                "maximum_inclusive": self.maximum_inclusive,
            },
            "reason": self.reason,
            "status": "within_allowed_range",
            "optimization_targets": sorted(self.optimization_stages),
        }
        if self.search_candidates is not None:
            record["search_candidates"] = list(self.search_candidates)
        if self.direct_fit_bounds is not None:
            record["direct_fit_bounds"] = list(self.direct_fit_bounds)
        if self.direct_fit_candidates is not None:
            record["direct_fit_candidates"] = list(self.direct_fit_candidates)
        if self.default is not None:
            record["default"] = self.default
        return record


# These values formalize the existing direct-fit clipping and fit_merge search
# spaces.  They are intentionally collected here so an optimization cannot use
# a wider domain than the one documented in the final artifact.
PARAMETER_CONSTRAINTS: dict[str, ParameterConstraint] = {
    "acc_time_constant": ParameterConstraint(
        "acc_time_constant", "縦加速度の一次遅れを、直接同定で採用済みの安定応答域に制限する。",
        search_bounds=(0.1, 3.0),
        direct_fit_bounds=(0.01, 5.0),
        optimization_stages=frozenset({FIT_LON, FIT_MERGE}),
    ),
    "acc_time_delay": ParameterConstraint(
        "acc_time_delay", "縦加速度のむだ時間を、既存の離散探索で評価済みの遅延候補に制限する。",
        search_candidates=(0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50),
        direct_fit_candidates=tuple(i / 100.0 for i in range(0, 31)),
        optimization_stages=frozenset({FIT_LON, FIT_MERGE}),
    ),
    "debug_acc_scaling_factor": ParameterConstraint(
        "debug_acc_scaling_factor",
        "加速度指令の校正誤差だけを表現し、未モデル化加速度を過大に吸収させない。"
        "下限 0.8 への拡大検証 (2026-07-16, 318 dataset) では最適値が 0.800 と新下限にも"
        "再クランプし (ax プラトー RMSE +4.3% 改善)、校正補正の域を超えて未モデル化加速度を"
        "吸収し続ける構造起因の要求と判断して 0.9 を維持する。",
        search_bounds=(0.9, 1.1),
        optimization_stages=frozenset({FIT_LON, FIT_MERGE}),
    ),
    "steer_time_constant": ParameterConstraint(
        "steer_time_constant", "操舵一次遅れを、直接同定で採用済みの安定応答域に制限する。",
        search_bounds=(0.05, 0.8),
        direct_fit_bounds=(0.01, 2.0),
        optimization_stages=frozenset({FIT_STEER, FIT_MERGE}),
    ),
    "steer_time_delay": ParameterConstraint(
        "steer_time_delay", "操舵むだ時間を、直接同定で採用済みの観測可能な遅延域に制限する。",
        bounds=(0.0, 0.15),
        direct_fit_candidates=tuple(i / 100.0 for i in range(0, 16)),
        optimization_stages=frozenset({FIT_STEER}),
    ),
    "debug_steer_scaling_factor": ParameterConstraint(
        "debug_steer_scaling_factor", "操舵指令の校正誤差だけを表現し、操舵モデル誤差の吸収を抑える。",
        search_bounds=(0.9, 1.1),
        optimization_stages=frozenset({FIT_STEER, FIT_MERGE}),
    ),
    "k_us": ParameterConstraint(
        "k_us", "定常旋回時の速度依存操舵補正を、既存の回帰clip域に制限する。",
        search_bounds=(0.0, 0.05),
        optimization_stages=frozenset({FIT_STEER, FIT_MERGE}),
    ),
    "steer_dead_band": ParameterConstraint(
        "steer_dead_band", "不感帯は非連続要素なので、使わない",
        search_bounds=(0.0, 0.00), default=0.0,
        optimization_stages=frozenset({FIT_MERGE}),
    ),
    "steer_bias": ParameterConstraint(
        "steer_bias", "直進時の操舵オフセットを、既存探索で検証済みのセンサbias域に制限する。",
        search_bounds=(-0.01, 0.01),
        optimization_stages=frozenset({FIT_STEER, FIT_MERGE}),
    ),
    "steer_rate_lim": ParameterConstraint(
        "steer_rate_lim", "操舵速度制限は負値にできず、有限値のみをモデルへ渡す。",
        bounds=(0.0, None), default=5.0,
    ),
    "wheelbase": ParameterConstraint(
        "wheelbase", "ホイールベースは車両幾何として正の有限値でなければならない。",
        bounds=(0.0, None), minimum_inclusive=False, default=4.76012,
    ),
    # 実データ (372 dataset, n=2,479,500) での直接同定値 +0.004278 を含み、
    # 過去の固定値 +0.022 も包含する範囲。符号は一貫して正であることを確認済み。
    "xy_heading_rate_coeff": ParameterConstraint(
        "xy_heading_rate_coeff", "位置積分時の heading 補正係数を、実データで確認済みの正の観測レンジに制限する。",
        bounds=(-0.005, 0.05), default=0.0,
        direct_fit_bounds=(-0.005, 0.05),
        optimization_stages=frozenset({FIT_XY}),
    ),
    # ---- v3 構造項 (2026-07-17 残差分析キャンペーンの証拠に基づく。analysis.html 参照) ----
    # 定常マップで throttle ゲイン ≈1.0 / brake ゲイン ≈0.7 の非対称を確認。単一項スクリーニング
    # (gate screen, dev 248) では brake ゲイン単独変更 (0.7) が N=30 long/pos cvar を +13〜19%
    # 悪化させた — 定常ゲインだけ弱めると応答の形が崩れるため、brake は gain と τ の同時同定が
    # 必須。よって直接値の手動指定ではなく fit_merge の関節探索対象として登録する。
    # 全 default=0.0 (センチネル/無効) で v2 と bit 一致。
    "lon_drag_c0": ParameterConstraint(
        "lon_drag_c0",
        "転がり抵抗相当の定数減速度 [m/s²]。0 で無効 (v2 と bit 一致)。物理オーダ (μ_rr·g) に制限。",
        search_bounds=(0.0, 0.3), default=0.0,
        optimization_stages=frozenset({FIT_MERGE}),
    ),
    "lon_drag_c2": ParameterConstraint(
        "lon_drag_c2",
        "空力抵抗係数 [1/m] (減速度 = c2·v²)。0 で無効。15 m/s で ≤0.45 m/s² となる範囲に制限。",
        search_bounds=(0.0, 0.002), default=0.0,
        optimization_stages=frozenset({FIT_MERGE}),
    ),
    "brake_time_constant": ParameterConstraint(
        "brake_time_constant",
        "減速指令時の一次遅れ時定数 [s]。0 はセンチネル (acc_time_constant を継承 = v2 と同挙動)。"
        "0 近傍の正値は MIN_TIME_CONSTANT へクランプされる (センチネルとの不連続に注意)。",
        search_bounds=(0.0, 1.5), default=0.0,
        optimization_stages=frozenset({FIT_MERGE}),
    ),
    "brake_scaling_factor": ParameterConstraint(
        "brake_scaling_factor",
        "減速指令時の scaling。0 はセンチネル (debug_acc_scaling_factor を継承 = v2 と同挙動)。"
        "定常マップの brake ゲイン実測 ≈0.69–0.72 を含む範囲に制限。",
        search_bounds=(0.0, 1.1), default=0.0,
        optimization_stages=frozenset({FIT_MERGE}),
    ),
}


DIRECT_FIT_STAGES: tuple[str, ...] = (FIT_LON, FIT_STEER, FIT_XY)
ALL_CONSTRAINED_KEYS: frozenset[str] = frozenset(PARAMETER_CONSTRAINTS)
# default を持たない制約キー。ステージ成果物 (phase artifact) はこの集合を必ず含むこと。
# default を持つキーは省略可で、apply_constraint_defaults() が補完する。
REQUIRED_ARTIFACT_KEYS: frozenset[str] = frozenset(
    name for name, constraint in PARAMETER_CONSTRAINTS.items() if constraint.default is None
)


def apply_constraint_defaults(params: dict) -> dict:
    """default を持つ制約キーを setdefault で補完する (in-place、戻り値は同じ dict)。"""
    for name, constraint in PARAMETER_CONSTRAINTS.items():
        if constraint.default is not None:
            params.setdefault(name, constraint.default)
    return params


def stage_targets(stage: str) -> frozenset[str]:
    """Return the parameters enabled for one identification stage."""
    return frozenset(
        name for name, constraint in PARAMETER_CONSTRAINTS.items()
        if stage in constraint.optimization_stages
    )


def stage_input_keys(stage: str) -> frozenset[str]:
    """Keys the stage's input artifact must contain: union of prior stage targets."""
    prior = DIRECT_FIT_STAGES[: DIRECT_FIT_STAGES.index(stage)]
    keys: frozenset[str] = frozenset()
    for prior_stage in prior:
        keys |= stage_targets(prior_stage)
    return keys


def search_constraints(stage: str = FIT_MERGE) -> dict[str, ParameterConstraint]:
    return {
        name: constraint
        for name, constraint in PARAMETER_CONSTRAINTS.items()
        if stage in constraint.optimization_stages
        and (constraint.search_bounds is not None or constraint.search_candidates is not None)
    }


def validate_parameters(
    params: Mapping[str, Any], keys: set[str] | frozenset[str], *, source: str,
) -> None:
    """Reject direct inputs outside their physical range without modifying them."""
    failures: list[str] = []
    for key in sorted(keys):
        constraint = PARAMETER_CONSTRAINTS[key]
        value = params.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            failures.append(f"{key}={value!r} (有限数値が必要; {constraint.reason})")
            continue
        numeric = float(value)
        if not constraint.contains(numeric):
            failures.append(
                f"{key}={numeric:g} (許容域 {constraint.range_text()}; {constraint.reason})"
            )
    if failures:
        raise ValueError(f"{source} に物理範囲外の params があります: " + "; ".join(failures))


def clamp_search_parameters(
    params: Mapping[str, Any], stage: str = FIT_MERGE,
) -> dict[str, Any]:
    """Return a candidate whose searched parameters are inside physical domains."""
    result = dict(params)
    for key in search_constraints(stage):
        if key in result:
            result[key] = PARAMETER_CONSTRAINTS[key].clamp(float(result[key]))
    return result


def build_constraint_audit(params: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    """Produce the stable, YAML-serializable constraint snapshot for final params."""
    return {
        key: constraint.audit_record(params[key])
        for key, constraint in PARAMETER_CONSTRAINTS.items()
        if key in params
    }
