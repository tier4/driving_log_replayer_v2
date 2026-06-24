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


# vehicle_model_type (open-loop クラス名) → sim の vehicle_model_type パラメータ (enum 文字列)
# step3_run_sims が simple_sensor_simulator.vehicle_model_type:=<ENUM> として launch に渡す。
_VEHICLE_MODEL_TYPE_ENUM: dict[str, str] = {
    "delay_steer_acc_geared_wo_fall_guard": "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD",
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
    # 縦横モデル検証ビューア相当の表現力 (delay_steer_acc_geared_wo_fall_guard)
    # brake_time_constant=throttle/brake 分離, lon_drag_c0/c1/c2=走行抵抗 poly(v),
    # lon_lat_coupling=縦横連成 c·(v·ω)²
    "brake_time_constant", "lon_drag_c0", "lon_drag_c1", "lon_drag_c2", "lon_lat_coupling",
    # Euler sub-steps per outer update() call (delay_steer_acc_geared_wo_fall_guard のみ有効)
    "n_substep",
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
})

_KNOWN_MODEL_KEYS: frozenset[str] = frozenset({
    "vehicle_model_type", "vehicle_model", "params",
    "sensor_model", "initialize_duration", "architecture_type",
    "godot_executable", "timeout_s",
    "dp_model_dir", "dp_model_release", "dp_model_package",
})

_DEFAULT_DP_PACKAGE = "diffusion_planner_for_x2_exp"

# tag は出力ディレクトリ名に直接連結されるため安全文字のみ許可
_TAG_PATTERN = re.compile(r"^[A-Za-z0-9_\-.]+$")


@dataclass
class ModelSpec:
    """1 モデルエントリの定義 (Conditions.models[<name>])."""

    name: str
    vehicle_model_type: str | None = None      # open-loop VehicleModel クラス名
    vehicle_model: str | None = None           # closed-loop description パッケージ名
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
    overlay: OverlaySpec = field(default_factory=OverlaySpec)


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
    if not p.exists():
        raise FileNotFoundError(f"scenario.yaml が見つかりません: {p}")

    with p.open(encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}

    conditions: dict = (doc.get("Evaluation") or {}).get("Conditions") or {}

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
        godot_exe = entry.get("godot_executable")
        if godot_exe:
            godot_exe = os.path.expandvars(os.path.expanduser(str(godot_exe)))

        dp_model_dir = entry.get("dp_model_dir")
        if dp_model_dir:
            dp_model_dir = os.path.expandvars(os.path.expanduser(str(dp_model_dir)))

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
    raw_cases = conditions.get("cases") or []
    if not isinstance(raw_cases, list):
        raise ValueError(f"{p}: Conditions.cases がリストではありません")
    cases_list: list[str] = []
    for name in raw_cases:
        name = str(name)
        if name not in models:
            raise ValueError(
                f"{p}: cases に未定義の model 名 {name!r} が含まれます。"
                f"定義済: {list(models.keys())}"
            )
        if models[name].vehicle_model_type is None:
            raise ValueError(
                f"{p}: cases に含まれる {name!r} には vehicle_model_type が必要です "
                "(open-loop VehicleModel クラス選択)"
            )
        cases_list.append(name)

    # ── sim_runs リスト ──────────────────────────────────────────────────────
    raw_sim_runs = conditions.get("sim_runs") or []
    if not isinstance(raw_sim_runs, list):
        raise ValueError(f"{p}: Conditions.sim_runs がリストではありません")
    sim_runs_list: list[str] = []
    for name in raw_sim_runs:
        name = str(name)
        if name not in models:
            raise ValueError(
                f"{p}: sim_runs に未定義の model 名 {name!r} が含まれます。"
                f"定義済: {list(models.keys())}"
            )
        if models[name].vehicle_model is None:
            raise ValueError(
                f"{p}: sim_runs に含まれる {name!r} には vehicle_model が必要です "
                "(closed-loop description パッケージ)"
            )
        sim_runs_list.append(name)

    if not cases_list:
        warnings.warn(f"{p}: Conditions.cases が空です (open-loop 解析はスキップされます)")
    if not sim_runs_list:
        warnings.warn(f"{p}: Conditions.sim_runs が空です (closed-loop sim はスキップされます)")

    # 未使用モデルへの警告
    used = set(cases_list) | set(sim_runs_list)
    for name in models:
        if name not in used:
            warnings.warn(
                f"{p}: models.{name} は cases にも sim_runs にも含まれていません (未使用)"
            )

    # ── overlay ──────────────────────────────────────────────────────────────
    overlay_raw = conditions.get("overlay") or {}
    overlay = OverlaySpec(
        reference_tag=overlay_raw.get("reference_tag"),
        plots=list(overlay_raw.get("plots") or ["cascade_error"]),
    )

    return ModelsDoc(
        models=models,
        cases_list=cases_list,
        sim_runs_list=sim_runs_list,
        overlay=overlay,
    )
