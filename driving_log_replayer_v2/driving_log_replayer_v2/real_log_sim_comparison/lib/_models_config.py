"""Unified models registry — scenario.yaml Conditions.models / cases / sim_runs / overlay.

cases.yaml と sim_runs*.yaml は廃止され、設定はすべて scenario.yaml の
Conditions ブロックに統合された。各モデルは名前付きエントリとして同列に定義され、
`cases:` / `sim_runs:` はその名前のリストで参照する。

schema (Conditions ブロック):

    models:
      <name>:                          # 名前 = 出力 tag (nstep/<name>/ / lite/<name>.lite/)
        vehicle_model_type: <str>      # open-loop VehicleModel クラス名 (cases で必須)
        vehicle_model: <str>           # closed-loop description パッケージ名 (sim_runs で必須)
        params: {<key>: <scalar>}      # open/sim 共有 params (simulator_model 上書き)
        # --- sim 専用 (open-loop では無視) ---
        architecture_type: <str>       # 既定 awf/universe/20250130
        sensor_model: <str>            # 既定 aip_x2_gen2
        initialize_duration: <int>     # 既定 100 [s]
        godot_executable: <str>        # vehicle_model が *_godot のとき必須
        timeout_s: <int>               # 既定 600 [s]
        dp_model_dir: <str>            # DP モデルディレクトリ (任意・ローカル)
        dp_model_release: <str>        # Web.Auto release 名 (任意・自動 pull)
        dp_model_package: <str>        # dp_model_release 指定時の package 名

    cases:    [<name>, ...]            # open-loop VehicleModel 解析。vehicle_model_type 必須
    sim_runs: [<name>, ...]            # closed-loop sim 実行。vehicle_model 必須
    overlay:
      reference_tag: <str>             # 基準モデル (cases 集約の overlay 比較基準)
      plots: [cascade_error]
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import re
import warnings
from typing import Any

import yaml

from ._accel_source import normalize_accel_source


# vehicle_model_type (open-loop クラス名) → sim の vehicle_model_type パラメータ (enum 文字列)
# step3_run_sims が simple_sensor_simulator.vehicle_model_type:=<ENUM> として launch に渡す。
_VEHICLE_MODEL_TYPE_ENUM: dict[str, str] = {
    "delay_steer_acc_geared_wo_fall_guard": "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD",
    "delay_steer_acc_geared_for_diffusion_planner": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
    "ideal_steer_acc": "IDEAL_STEER_ACC",
    "taiga_dyn": "TAIGA_DYN",
    "taiga_x": "TAIGA_X",
}

SUPPORTED_VMT: frozenset[str] = frozenset(_VEHICLE_MODEL_TYPE_ENUM.keys())

# vm_create_* / VehicleModel.__init__ が消費する物理パラメータ名 (typo 検出用)
KNOWN_PARAM_KEYS: frozenset[str] = frozenset({
    "vel_lim", "steer_lim", "vel_rate_lim", "steer_rate_lim",
    "wheelbase", "acc_time_delay", "acc_time_constant",
    "steer_time_delay", "steer_time_constant",
    "steer_dead_band", "steer_bias",
    "debug_acc_scaling_factor", "debug_steer_scaling_factor",
    "k_us", "sub_dt",
    # taiga_dyn / taiga_x 動的自転車モデル
    "mass", "inertia_z", "lf", "lr",
    "cornering_stiffness_front", "cornering_stiffness_rear", "vx_min_dyn",
    # taiga_x (PhysX backend) 専用
    "track_width", "cg_offset_x", "max_accel", "max_brake", "wheel_radius",
    "taiga_x_fixed_dt",
    # learned_arx
    "arx_ax_coeffs", "arx_wz_coeffs",
    # sim params (vehicle_model_type は sim で注入されるため params には書かない)
    "vehicle_model_type",
    "delay_steer_acc_geared_for_diffusion_planner.version",
})

_KNOWN_MODEL_KEYS: frozenset[str] = frozenset({
    "vehicle_model_type", "vehicle_model", "acceleration_source",
    "model_role", "version", "source_param_path", "params",
    "sensor_model", "initialize_duration", "architecture_type",
    "godot_executable", "timeout_s",
    "dp_model_dir", "dp_model_release", "dp_model_package",
})

_DEFAULT_DP_PACKAGE = "diffusion_planner_for_x2_exp"

# tag は出力ディレクトリ名に直接連結されるため安全文字のみ許可
_TAG_PATTERN = re.compile(r"^[A-Za-z0-9_\-.]+$")


def _expand_path(value: Any) -> str | None:
    """環境変数/チルダ展開が必要なパス値を正規化する。"""
    if not value:
        return None
    return os.path.expandvars(os.path.expanduser(str(value)))


def _load_yaml_doc(path: Path) -> dict[str, Any]:
    """scenario.yaml を読み込む。"""
    if not path.exists():
        raise FileNotFoundError(f"scenario.yaml が見つかりません: {path}")

    with path.open(encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: YAML ルートがマッピング (dict) ではありません")
    return doc


def _load_conditions(path: Path) -> dict[str, Any]:
    """scenario.yaml の Evaluation.Conditions を返す。"""
    doc = _load_yaml_doc(path)
    conditions = (doc.get("Evaluation") or {}).get("Conditions") or {}
    if not isinstance(conditions, dict):
        raise ValueError(f"{path}: Evaluation.Conditions がマッピング (dict) ではありません")
    return conditions


def _validate_named_model_list(
    *,
    scenario_path: Path,
    models: dict[str, ModelSpec],
    raw_names: Any,
    list_name: str,
    required_attr: str,
    required_detail: str,
    empty_note: str,
) -> list[str]:
    """models から cases / sim_runs の名前リストを検証する。"""
    if not isinstance(raw_names, list):
        raise ValueError(f"{scenario_path}: Conditions.{list_name} がリストではありません")

    names: list[str] = []
    for name in raw_names:
        name = str(name)
        if name not in models:
            raise ValueError(
                f"{scenario_path}: {list_name} に未定義の model 名 {name!r} が含まれます。"
                f"定義済: {list(models.keys())}"
            )
        if getattr(models[name], required_attr) is None:
            raise ValueError(
                f"{scenario_path}: {list_name} に含まれる {name!r} には {required_detail} が必要です"
            )
        names.append(name)

    if not names:
        warnings.warn(f"{scenario_path}: Conditions.{list_name} が空です ({empty_note})")
    return names


def _load_overlay_spec(conditions: dict[str, Any]) -> OverlaySpec:
    """overlay セクションを OverlaySpec に変換する。"""
    overlay_raw = conditions.get("overlay") or {}
    return OverlaySpec(
        reference_tag=overlay_raw.get("reference_tag"),
        plots=list(overlay_raw.get("plots") or ["cascade_error"]),
    )


@dataclass
class ModelSpec:
    """1 モデルエントリの定義 (Conditions.models[<name>])."""

    name: str
    vehicle_model_type: str | None = None      # open-loop VehicleModel クラス名
    vehicle_model: str | None = None           # closed-loop description パッケージ名
    acceleration_source: str = "accel"          # 評価用実測加速度系列
    model_role: str | None = None
    version: str | int | None = None
    source_param_path: str | None = None
    params: dict[str, Any] = field(default_factory=dict)
    # sim 専用フィールド (open-loop では無視)
    sensor_model: str = "aip_x2_gen2"
    initialize_duration: int = 100
    architecture_type: str = "awf/universe/20250130"
    godot_executable: str | None = None
    timeout_s: int = 600
    dp_model_dir: str | None = None
    dp_model_release: str | None = None
    dp_model_package: str | None = None

    @property
    def tag(self) -> str:
        return self.name


@dataclass
class OverlaySpec:
    """cases 集約の overlay セクション."""

    reference_tag: str | None = None
    plots: list[str] = field(default_factory=lambda: ["cascade_error"])


@dataclass
class ModelsDoc:
    """scenario.yaml から読み取った models / cases / sim_runs / overlay の全体。"""

    models: dict[str, ModelSpec]
    cases_list: list[str]
    sim_runs_list: list[str]
    comparison_models_list: list[str] = field(default_factory=list)
    overlay: OverlaySpec = field(default_factory=OverlaySpec)

    def find_case(self, tag: str) -> ModelSpec:
        return _find_named(self.cases, tag, "case")

    def find_run(self, tag: str) -> ModelSpec:
        return _find_named(self.runs, tag, "run")

    @property
    def tags(self) -> list[str]:
        return self.cases_list

    @property
    def cases(self) -> list[ModelSpec]:
        return [self.models[name] for name in self.cases_list]

    @property
    def runs(self) -> list[ModelSpec]:
        return [self.models[name] for name in self.sim_runs_list]

    @property
    def comparison_models(self) -> list[ModelSpec]:
        names = self.comparison_models_list or self.cases_list
        return [self.models[name] for name in names]


def _find_named(items: list[ModelSpec], tag: str, kind: str) -> ModelSpec:
    for item in items:
        if item.name == tag:
            return item
    raise KeyError(f"tag={tag!r} が Conditions.{kind}s に見つかりません")


def load_models_doc(scenario_path: str | Path) -> ModelsDoc:
    """scenario.yaml の Conditions から ModelsDoc を構築する。

    必須: models が dict。cases / sim_runs は名前リスト。
    バリデーション:
      - モデル名は安全文字のみ。
      - cases に挙げた名前は vehicle_model_type 必須、未定義名は error。
      - sim_runs に挙げた名前は vehicle_model 必須、未定義名は error。
      - dp_model_dir と dp_model_release の排他チェック。
      - params に未知キーがあれば warn (typo 防止)。
    """
    p = Path(scenario_path)
    conditions = _load_conditions(p)

    # ── models ──────────────────────────────────────────────────────────────
    raw_models = conditions.get("models") or {}
    if not isinstance(raw_models, dict):
        raise ValueError(f"{p}: Conditions.models がマッピング (dict) ではありません")

    models: dict[str, ModelSpec] = {}
    for name, entry in raw_models.items():
        name = str(name)
        if not _TAG_PATTERN.match(name):
            raise ValueError(
                f"{p}: models のキー {name!r} は安全文字 [A-Za-z0-9_.-] のみ許可 "
                "(path traversal / 不正ディレクトリ名を防ぐ)"
            )
        if entry is None:
            entry = {}
        if not isinstance(entry, dict):
            raise ValueError(f"{p}: models.{name} が dict ではありません")

        unknown = set(entry.keys()) - _KNOWN_MODEL_KEYS
        if unknown:
            warnings.warn(
                f"{p}: models.{name} に未知キー {sorted(unknown)} が含まれます "
                f"(typo の可能性。既知キー: {sorted(_KNOWN_MODEL_KEYS)})"
            )

        # vehicle_model_type バリデーション
        vmt = entry.get("vehicle_model_type")
        if vmt is not None:
            vmt = str(vmt)
            if vmt not in SUPPORTED_VMT:
                raise ValueError(
                    f"{p}: models.{name}.vehicle_model_type={vmt!r} は未対応。"
                    f"対応: {sorted(SUPPORTED_VMT)}"
                )

        # params バリデーション
        raw_params = entry.get("params") or {}
        if not isinstance(raw_params, dict):
            raise ValueError(f"{p}: models.{name}.params が dict ではありません")
        unknown_params = set(raw_params.keys()) - KNOWN_PARAM_KEYS
        if unknown_params:
            warnings.warn(
                f"{p}: models.{name}.params に未知キー {sorted(unknown_params)} が含まれます "
                f"(typo の可能性。既知キー: {sorted(KNOWN_PARAM_KEYS)})"
            )

        # godot 警告
        vm = entry.get("vehicle_model")
        if isinstance(vm, str) and vm.endswith("_godot") and not entry.get("godot_executable"):
            warnings.warn(
                f"{p}: models.{name} vehicle_model={vm!r} だが "
                "godot_executable が未指定 (scenario_test_runner の既定パスにフォールバック)"
            )

        # パス展開 (ros2 launch は ~ / ${VAR} を展開しないため)
        godot_exe = _expand_path(entry.get("godot_executable"))
        dp_model_dir = _expand_path(entry.get("dp_model_dir"))

        dp_model_release = entry.get("dp_model_release")
        dp_model_package = entry.get("dp_model_package")

        if dp_model_release and dp_model_dir:
            raise ValueError(
                f"{p}: models.{name} dp_model_dir と dp_model_release は同時指定できません "
                "(どちらか一方)"
            )
        if dp_model_package and not dp_model_release:
            raise ValueError(
                f"{p}: models.{name} dp_model_package は dp_model_release 指定時のみ有効です"
            )
        if dp_model_release:
            dp_model_release = str(dp_model_release)
            dp_model_package = str(dp_model_package) if dp_model_package else _DEFAULT_DP_PACKAGE

        models[name] = ModelSpec(
            name=name,
            vehicle_model_type=vmt,
            vehicle_model=str(vm) if vm is not None else None,
            acceleration_source=normalize_accel_source(entry.get("acceleration_source"), default="accel"),
            model_role=str(entry["model_role"]) if entry.get("model_role") is not None else None,
            version=entry.get("version"),
            source_param_path=_expand_path(entry.get("source_param_path")),
            params=dict(raw_params),
            sensor_model=str(entry.get("sensor_model", "aip_x2_gen2")),
            initialize_duration=int(entry.get("initialize_duration", 100)),
            architecture_type=str(entry.get("architecture_type", "awf/universe/20250130")),
            godot_executable=godot_exe,
            timeout_s=int(entry.get("timeout_s", 600)),
            dp_model_dir=dp_model_dir,
            dp_model_release=dp_model_release,
            dp_model_package=dp_model_package,
        )

    # ── cases リスト ─────────────────────────────────────────────────────────
    cases_list = _validate_named_model_list(
        scenario_path=p,
        models=models,
        raw_names=conditions.get("cases") or [],
        list_name="cases",
        required_attr="vehicle_model_type",
        required_detail="vehicle_model_type (open-loop VehicleModel クラス選択)",
        empty_note="open-loop 解析はスキップされます",
    )

    # ── sim_runs リスト ──────────────────────────────────────────────────────
    sim_runs_list = _validate_named_model_list(
        scenario_path=p,
        models=models,
        raw_names=conditions.get("sim_runs") or [],
        list_name="sim_runs",
        required_attr="vehicle_model",
        required_detail="vehicle_model (closed-loop description パッケージ)",
        empty_note="closed-loop sim はスキップされます",
    )

    # ── comparison_models リスト ─────────────────────────────────────────────
    comparison_models_list = _validate_named_model_list(
        scenario_path=p,
        models=models,
        raw_names=conditions.get("comparison_models", cases_list),
        list_name="comparison_models",
        required_attr="vehicle_model_type",
        required_detail="vehicle_model_type (比較評価用 open-loop VehicleModel クラス選択)",
        empty_note="比較評価は Conditions.cases にフォールバックします",
    )

    overlay = _load_overlay_spec(conditions)

    return ModelsDoc(
        models=models,
        cases_list=cases_list,
        sim_runs_list=sim_runs_list,
        comparison_models_list=comparison_models_list,
        overlay=overlay,
    )


def resolve_baseline_model(doc: ModelsDoc) -> tuple[str, dict, str]:
    """Conditions.overlay.reference_tag が指す baseline model の
    (vehicle_model_type, params, case_name) を返す。

    scenario.yaml をこの解決の唯一の SSOT とする。呼び出し側 (multi_dataset_tune.py の
    スコア正規化・physical_validity_report.py のレポート比較) が別々に baseline model を
    ハードコードすると、片方だけ更新されて静かに不整合を起こす (2026-07-07 に実際発生)。
    """
    reference = doc.overlay.reference_tag
    if not reference or reference not in doc.models:
        raise ValueError(
            f"Conditions.overlay.reference_tag に有効な baseline model 名が必要です (現在: {reference!r})"
        )
    baseline = doc.models[reference]
    if baseline.vehicle_model_type is None:
        raise ValueError(f"baseline model {reference!r} には vehicle_model_type が必要です")
    return baseline.vehicle_model_type, dict(baseline.params), reference


def load_run_models(scenario_path: str | Path) -> ModelsDoc:
    doc = load_models_doc(scenario_path)
    for name in doc.sim_runs_list:
        model = doc.models[name]
        for key, value in model.params.items():
            if not isinstance(value, (int, float, bool, str)):
                raise ValueError(
                    f"{scenario_path}: models.{name}.params.{key} はスカラのみ可"
                )
        if model.vehicle_model_type and "vehicle_model_type" not in model.params:
            model.params["vehicle_model_type"] = _VEHICLE_MODEL_TYPE_ENUM[model.vehicle_model_type]
    return doc
