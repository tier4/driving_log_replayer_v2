"""Shared vehicle model registry and ctypes wrapper for open-loop evaluation."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Callable


_BASE_DELAY_ARGS: tuple[str, ...] = (
    "vel_lim",
    "steer_lim",
    "vel_rate_lim",
    "steer_rate_lim",
    "wheelbase",
    "sub_dt",
    "acc_time_delay",
    "acc_time_constant",
    "steer_time_delay",
    "steer_time_constant",
    "steer_dead_band",
    "steer_bias",
    "debug_acc_scaling_factor",
    "debug_steer_scaling_factor",
    "k_us",
)

@dataclass(frozen=True)
class VehicleModelSpec:
    """Vehicle-model metadata needed by config, release, and ctypes rollout."""

    sim_enum: str
    factory_name: str
    factory_arg_count: int
    build_args: Callable[[dict[str, Any], float], tuple[float, ...]]
    param_keys: frozenset[str] = frozenset()
    namespaced_param_root: str | None = None
    version_param_key: str | None = None
    namespaced_param_keys: frozenset[str] = frozenset()


def _arg(p: dict[str, Any], key: str, default: float | None = None) -> float:
    if key == "sub_dt":
        raise KeyError("sub_dt must be passed explicitly")
    if default is None:
        return float(p[key])
    return float(p.get(key, default))


def _build_delay_args(p: dict[str, Any], sub_dt: float) -> tuple[float, ...]:
    values: list[float] = []
    defaults = {
        "debug_acc_scaling_factor": 1.0,
        "debug_steer_scaling_factor": 1.0,
        "k_us": 0.0,
    }
    for key in _BASE_DELAY_ARGS:
        if key == "sub_dt":
            values.append(float(sub_dt))
        else:
            values.append(_arg(p, key, defaults.get(key)))
    return tuple(values)


def _build_dp_args(p: dict[str, Any], sub_dt: float) -> tuple[float, ...]:
    values: list[float] = []
    defaults = {
        "debug_acc_scaling_factor": 1.0,
        "debug_steer_scaling_factor": 1.0,
        "k_us": 0.0,
        "xy_heading_rate_coeff": 0.0,
        # v3 構造項。中立値 0.0 で v2 と bit 一致 (brake_* の 0 は「対称値を継承」のセンチネル、
        # steer_relaxation_length の 0 は無効 = ヨーは STEER 状態を直接使用)。
        "lon_drag_c0": 0.0,
        "lon_drag_c2": 0.0,
        "brake_time_constant": 0.0,
        "brake_scaling_factor": 0.0,
        "steer_relaxation_length": 0.0,
        "use_rk4": 0.0,
    }
    # C ABI (vm_create_delay_steer_acc_geared_for_diffusion_planner_v3) の引数順と
    # 一字一句同順であること (C++ 側ゴールデンテストと test_param_ssot が監視)。
    dp_args = _BASE_DELAY_ARGS + (
        "xy_heading_rate_coeff",
        "lon_drag_c0",
        "lon_drag_c2",
        "brake_time_constant",
        "brake_scaling_factor",
        "steer_relaxation_length",
        "use_rk4",
    )
    for key in dp_args:
        if key == "sub_dt":
            values.append(float(sub_dt))
        else:
            val = _arg(p, key, defaults.get(key))
            if key == "use_rk4":
                # C++ ABI へ渡す bool トグル。フィット対象ではないため
                # PARAMETER_CONSTRAINTS には登録しない (test_param_ssot が監視)。
                val = 1.0 if bool(val) else 0.0
            values.append(val)
    return tuple(values)


_DELAY_PARAM_KEYS = frozenset(_BASE_DELAY_ARGS)
# リリース YAML のグローバル欄に置くキー (バージョン名前空間の外)。
_GLOBAL_ARG_KEYS = frozenset(
    {"vel_lim", "steer_lim", "vel_rate_lim", "steer_rate_lim", "wheelbase", "sub_dt"}
)
_DP_NAMESPACED_PARAM_KEYS = _DELAY_PARAM_KEYS - _GLOBAL_ARG_KEYS
# diffusion_planner 派生の追加キー (v2: xy_heading_rate_coeff/use_rk4、v3: drag/brake/緩和長)。
_DP_EXTRA_KEYS = (
    "xy_heading_rate_coeff",
    "lon_drag_c0",
    "lon_drag_c2",
    "brake_time_constant",
    "brake_scaling_factor",
    "steer_relaxation_length",
    "use_rk4",
)


VEHICLE_MODEL_SPECS: dict[str, VehicleModelSpec] = {
    "delay_steer_acc_geared_wo_fall_guard": VehicleModelSpec(
        sim_enum="DELAY_STEER_ACC_GEARED_WO_FALL_GUARD",
        factory_name="vm_create_delay_steer_acc_geared_wo_fall_guard",
        factory_arg_count=15,
        build_args=_build_delay_args,
        param_keys=_DELAY_PARAM_KEYS,
    ),
    "delay_steer_acc_geared_for_diffusion_planner": VehicleModelSpec(
        sim_enum="DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
        factory_name="vm_create_delay_steer_acc_geared_for_diffusion_planner_v3",
        factory_arg_count=22,
        build_args=_build_dp_args,
        param_keys=_DELAY_PARAM_KEYS | frozenset(_DP_EXTRA_KEYS),
        namespaced_param_root="delay_steer_acc_geared_for_diffusion_planner",
        version_param_key="delay_steer_acc_geared_for_diffusion_planner.version",
        namespaced_param_keys=_DP_NAMESPACED_PARAM_KEYS | frozenset(_DP_EXTRA_KEYS),
    ),
}

SUPPORTED_VMT: frozenset[str] = frozenset(VEHICLE_MODEL_SPECS)


def supported_vehicle_model_types() -> tuple[str, ...]:
    return tuple(sorted(SUPPORTED_VMT))


def get_vehicle_model_spec(model_type: str) -> VehicleModelSpec:
    try:
        return VEHICLE_MODEL_SPECS[model_type]
    except KeyError as exc:
        raise ValueError(
            f"未対応の model_type: {model_type!r}. 対応: {supported_vehicle_model_types()}"
        ) from exc


def _spec_from_model_selector(
    params: dict[str, Any], model_type: str | None = None
) -> VehicleModelSpec | None:
    if model_type:
        return get_vehicle_model_spec(model_type)

    selected = params.get("vehicle_model_type")
    if selected is None:
        return None
    selected = str(selected)
    if selected in VEHICLE_MODEL_SPECS:
        return VEHICLE_MODEL_SPECS[selected]
    for spec in VEHICLE_MODEL_SPECS.values():
        if selected == spec.sim_enum:
            return spec
    return None


def _version_param_to_key(version: Any) -> str:
    if isinstance(version, float) and version.is_integer():
        version = int(version)
    text = str(version).strip()
    if text.startswith("v"):
        return text
    return f"v{text}"


def apply_versioned_model_params(
    params: dict[str, Any], model_type: str | None = None
) -> dict[str, Any]:
    """Expand version-selected nested simulator params into flat model params.

    For models such as delay_steer_acc_geared_for_diffusion_planner, the ROS
    YAML stores versioned values under `<model_root>.vN`. Open-loop code consumes
    flat params, so `version` is treated as a selector and the selected `vN` block
    is merged into the returned dict.
    """

    out = dict(params)
    spec = _spec_from_model_selector(out, model_type)
    if spec is None or not spec.namespaced_param_root or not spec.version_param_key:
        return out

    root = out.get(spec.namespaced_param_root)
    if not isinstance(root, dict):
        return out

    version = out.get(spec.version_param_key, root.get("version"))
    if version is None:
        return out

    version_key = _version_param_to_key(version)
    version_params = root.get(version_key)
    if not isinstance(version_params, dict):
        raise ValueError(
            f"{spec.namespaced_param_root}.{version_key} が simulator_model.param.yaml に見つかりません"
        )

    out.update(version_params)
    out[spec.version_param_key] = version
    return out


def merge_vehicle_model_params(
    base_params: dict[str, Any],
    override_params: dict[str, Any],
    model_type: str | None = None,
) -> dict[str, Any]:
    """Merge base and scenario params while honoring version-selected defaults.

    If `override_params` selects a model version, that version is expanded from
    `base_params` first, then all explicit overrides are applied. This keeps
    `version` as a selector while still allowing per-case params to override a
    value from the selected version block.
    """

    spec = _spec_from_model_selector({**base_params, **override_params}, model_type)
    selected_base = dict(base_params)
    if spec is not None and spec.version_param_key in override_params:
        selected_base[spec.version_param_key] = override_params[spec.version_param_key]

    merged = apply_versioned_model_params(selected_base, model_type)
    merged.update(override_params)
    return merged


def _resolve_so_path() -> Path:
    env = os.environ.get("VEHICLE_MODEL_SO_PATH")
    if env:
        p = Path(env)
        if p.exists():
            return p
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("simple_sensor_simulator"))
    return share / "libvehicle_model_wrapper.so"


def _bind_factory(lib: ctypes.CDLL, spec: VehicleModelSpec) -> None:
    fn = getattr(lib, spec.factory_name)
    fn.restype = ctypes.c_void_p
    fn.argtypes = [ctypes.c_double] * spec.factory_arg_count


def _load_lib() -> ctypes.CDLL:
    so = _resolve_so_path()
    if not so.exists():
        raise FileNotFoundError(
            f"{so} が見つかりません。simple_sensor_simulator を colcon build してください。"
        )
    lib = ctypes.CDLL(str(so))

    c_double = ctypes.c_double
    c_void_p = ctypes.c_void_p
    for spec in VEHICLE_MODEL_SPECS.values():
        _bind_factory(lib, spec)

    lib.vm_reset_state.restype = None
    lib.vm_reset_state.argtypes = [c_void_p] + [c_double] * 8

    if not hasattr(lib, "vm_reset_state_v2"):
        raise RuntimeError(
            "libvehicle_model_wrapper.so に vm_reset_state_v2 が未 export です。"
            " simple_sensor_simulator を再ビルドしてください。"
        )
    lib.vm_reset_state_v2.restype = None
    lib.vm_reset_state_v2.argtypes = [c_void_p] + [c_double] * 9

    lib.vm_set_queues.restype = None
    lib.vm_set_queues.argtypes = [
        c_void_p,
        ctypes.POINTER(c_double),
        ctypes.c_int,
        ctypes.POINTER(c_double),
        ctypes.c_int,
    ]

    _c_dbl_p = ctypes.POINTER(c_double)
    _c_int_p = ctypes.POINTER(ctypes.c_int)
    if not hasattr(lib, "vm_integrate_to_horizons"):
        raise RuntimeError(
            "libvehicle_model_wrapper.so に vm_integrate_to_horizons が未 export です。"
            " simple_sensor_simulator を再ビルドしてください。"
        )
    lib.vm_integrate_to_horizons.restype = None
    lib.vm_integrate_to_horizons.argtypes = [
        c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        _c_dbl_p,
        _c_dbl_p,
        _c_int_p,
        _c_dbl_p,
        c_double,
        ctypes.c_int,
        _c_int_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
    ]

    # v2: steer_des の直後に nullable な slope_accx 配列 (None で SLOPE_ACCX=0、旧と bit 一致)。
    if not hasattr(lib, "vm_integrate_to_horizons_v2"):
        raise RuntimeError(
            "libvehicle_model_wrapper.so に vm_integrate_to_horizons_v2 が未 export です。"
            " simple_sensor_simulator を再ビルドしてください。"
        )
    lib.vm_integrate_to_horizons_v2.restype = None
    lib.vm_integrate_to_horizons_v2.argtypes = [
        c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_int_p,
        _c_dbl_p,
        c_double,
        ctypes.c_int,
        _c_int_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
        _c_dbl_p,
    ]

    lib.vm_destroy.restype = None
    lib.vm_destroy.argtypes = [c_void_p]

    return lib


class VehicleModel:
    """ctypes wrapper around libvehicle_model_wrapper.so vehicle models."""

    _lib: ctypes.CDLL | None = None

    @classmethod
    def _get_lib(cls) -> ctypes.CDLL:
        if cls._lib is None:
            cls._lib = _load_lib()
        return cls._lib

    def __init__(self, params: dict[str, Any], sub_dt: float, model_type: str):
        spec = get_vehicle_model_spec(model_type)
        lib = self._get_lib()
        self._lib = lib
        factory = getattr(lib, spec.factory_name)
        self._ptr = factory(*spec.build_args(params, sub_dt))

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr and hasattr(self, "_lib") and self._lib:
            self._lib.vm_destroy(self._ptr)
            self._ptr = None

    def reset_with_history_ptr(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        steer_actual: float,
        ax: float,
        acc_ptr: ctypes.POINTER(ctypes.c_double),
        n_acc: int,
        steer_ptr: ctypes.POINTER(ctypes.c_double),
        n_steer: int,
        wz: float = 0.0,
        vy: float = 0.0,
        slope_accx0: float = 0.0,
    ) -> None:
        # slope_accx0 は開始点の勾配加速度。PEDAL_ACCX 初期値を ax − slope − drag で逆算する
        # (0.0 なら旧 vm_reset_state と bit 一致)。
        self._lib.vm_reset_state_v2(
            self._ptr, x, y, yaw, vx, steer_actual, ax, wz, vy, slope_accx0,
        )
        self._lib.vm_set_queues(
            self._ptr,
            acc_ptr,
            ctypes.c_int(n_acc),
            steer_ptr,
            ctypes.c_int(n_steer),
        )
