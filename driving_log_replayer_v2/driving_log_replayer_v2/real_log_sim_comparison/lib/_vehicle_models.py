"""Shared vehicle model registry and ctypes wrapper for open-loop evaluation."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
import sys
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

_TAIGA_DYN_DEFAULTS: dict[str, float] = {
    "mass": 6560.0,
    "inertia_z": 25868.2318,
    "cg_offset_x": -0.94323,
    "cornering_stiffness_front": 115830.0,
    "cornering_stiffness_rear": 535860.0,
    "vx_min_dyn": 1.0,
}

_TAIGA_X_DEFAULTS: dict[str, float] = {
    "track_width": 1.754,
    "mass": 6560.0,
    "inertia_z": 25868.2318,
    "cg_offset_x": -0.94323,
    "max_accel": 2.3,
    "max_brake": 5.9,
    "wheel_radius": 0.3725,
    "taiga_x_fixed_dt": 1.0 / 1200.0,
}


@dataclass(frozen=True)
class VehicleModelSpec:
    """Open-loop vehicle model metadata shared by config, sim launch, and ctypes."""

    model_type: str
    sim_enum: str
    factory_name: str
    factory_arg_count: int
    build_args: Callable[[dict[str, Any], float], tuple[float, ...]]
    steer_bias_key: str | None = "steer_bias"
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


def _build_ideal_args(p: dict[str, Any], sub_dt: float) -> tuple[float, ...]:
    return (float(p["wheelbase"]), float(sub_dt))


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


def _build_taiga_dyn_args(p: dict[str, Any], sub_dt: float) -> tuple[float, ...]:
    wb = float(p["wheelbase"])
    base = list(_build_delay_args(p, sub_dt)[:-1])
    return tuple(
        base
        + [
            _arg(p, "mass", _TAIGA_DYN_DEFAULTS["mass"]),
            _arg(p, "inertia_z", _TAIGA_DYN_DEFAULTS["inertia_z"]),
            _arg(p, "lf", wb * 0.5 - _TAIGA_DYN_DEFAULTS["cg_offset_x"]),
            _arg(p, "lr", wb * 0.5 + _TAIGA_DYN_DEFAULTS["cg_offset_x"]),
            _arg(
                p,
                "cornering_stiffness_front",
                _TAIGA_DYN_DEFAULTS["cornering_stiffness_front"],
            ),
            _arg(
                p,
                "cornering_stiffness_rear",
                _TAIGA_DYN_DEFAULTS["cornering_stiffness_rear"],
            ),
            _arg(p, "vx_min_dyn", _TAIGA_DYN_DEFAULTS["vx_min_dyn"]),
        ]
    )


def _build_dp_args(p: dict[str, Any], sub_dt: float) -> tuple[float, ...]:
    values: list[float] = []
    defaults = {
        "debug_acc_scaling_factor": 1.0,
        "debug_steer_scaling_factor": 1.0,
        "k_us": 0.0,
        "xy_heading_rate_coeff": 0.0,
        "use_rk4": 0.0,
    }
    dp_args = _BASE_DELAY_ARGS + (
        "xy_heading_rate_coeff",
        "use_rk4",
    )
    for key in dp_args:
        if key == "sub_dt":
            values.append(float(sub_dt))
        else:
            val = _arg(p, key, defaults.get(key))
            if key == "use_rk4":
                val = 1.0 if bool(val) else 0.0
            values.append(val)
    return tuple(values)


def _build_taiga_x_args(p: dict[str, Any], sub_dt: float) -> tuple[float, ...]:
    return (
        float(p["wheelbase"]),
        _arg(p, "track_width", _TAIGA_X_DEFAULTS["track_width"]),
        _arg(p, "mass", _TAIGA_X_DEFAULTS["mass"]),
        _arg(p, "inertia_z", _TAIGA_X_DEFAULTS["inertia_z"]),
        _arg(p, "cg_offset_x", _TAIGA_X_DEFAULTS["cg_offset_x"]),
        float(p["steer_lim"]),
        _arg(p, "max_accel", _TAIGA_X_DEFAULTS["max_accel"]),
        _arg(p, "max_brake", _TAIGA_X_DEFAULTS["max_brake"]),
        _arg(p, "wheel_radius", _TAIGA_X_DEFAULTS["wheel_radius"]),
        float(sub_dt),
        _arg(p, "taiga_x_fixed_dt", _TAIGA_X_DEFAULTS["taiga_x_fixed_dt"]),
    )


_DELAY_PARAM_KEYS = frozenset(_BASE_DELAY_ARGS)
_TAIGA_DYN_PARAM_KEYS = frozenset(_BASE_DELAY_ARGS[:-1]) | frozenset(
    {
        "mass",
        "inertia_z",
        "lf",
        "lr",
        "cornering_stiffness_front",
        "cornering_stiffness_rear",
        "vx_min_dyn",
    }
)
_TAIGA_X_PARAM_KEYS = frozenset(
    {
        "wheelbase",
        "steer_lim",
        "sub_dt",
        "track_width",
        "mass",
        "inertia_z",
        "cg_offset_x",
        "max_accel",
        "max_brake",
        "wheel_radius",
        "taiga_x_fixed_dt",
    }
)
_DP_NAMESPACED_PARAM_KEYS = frozenset(
    {
        "acc_time_delay",
        "acc_time_constant",
        "steer_time_delay",
        "steer_time_constant",
        "steer_dead_band",
        "steer_bias",
        "debug_acc_scaling_factor",
        "debug_steer_scaling_factor",
        "k_us",
    }
)


VEHICLE_MODEL_SPECS: dict[str, VehicleModelSpec] = {
    "delay_steer_acc_geared_wo_fall_guard": VehicleModelSpec(
        model_type="delay_steer_acc_geared_wo_fall_guard",
        sim_enum="DELAY_STEER_ACC_GEARED_WO_FALL_GUARD",
        factory_name="vm_create_delay_steer_acc_geared_wo_fall_guard",
        factory_arg_count=15,
        build_args=_build_delay_args,
        param_keys=_DELAY_PARAM_KEYS,
    ),
    "delay_steer_acc_geared_for_diffusion_planner": VehicleModelSpec(
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        sim_enum="DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
        factory_name="vm_create_delay_steer_acc_geared_for_diffusion_planner",
        factory_arg_count=17,
        build_args=_build_dp_args,
        param_keys=_DELAY_PARAM_KEYS | frozenset({"xy_heading_rate_coeff", "use_rk4"}),
        namespaced_param_root="delay_steer_acc_geared_for_diffusion_planner",
        version_param_key="delay_steer_acc_geared_for_diffusion_planner.version",
        namespaced_param_keys=_DP_NAMESPACED_PARAM_KEYS | frozenset({"xy_heading_rate_coeff", "use_rk4"}),
    ),
    "ideal_steer_acc": VehicleModelSpec(
        model_type="ideal_steer_acc",
        sim_enum="IDEAL_STEER_ACC",
        factory_name="vm_create_ideal_steer_acc",
        factory_arg_count=2,
        build_args=_build_ideal_args,
        steer_bias_key=None,
        param_keys=frozenset({"wheelbase", "sub_dt"}),
    ),
    "taiga_dyn": VehicleModelSpec(
        model_type="taiga_dyn",
        sim_enum="TAIGA_DYN",
        factory_name="vm_create_taiga_dyn",
        factory_arg_count=21,
        build_args=_build_taiga_dyn_args,
        param_keys=_TAIGA_DYN_PARAM_KEYS,
    ),
    "taiga_x": VehicleModelSpec(
        model_type="taiga_x",
        sim_enum="TAIGA_X",
        factory_name="vm_create_taiga_x",
        factory_arg_count=11,
        build_args=_build_taiga_x_args,
        steer_bias_key=None,
        param_keys=_TAIGA_X_PARAM_KEYS,
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


def vehicle_model_type_to_enum(model_type: str) -> str:
    return get_vehicle_model_spec(model_type).sim_enum


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


def known_vehicle_model_param_keys() -> frozenset[str]:
    keys: set[str] = {"vehicle_model_type"}
    for spec in VEHICLE_MODEL_SPECS.values():
        keys.update(spec.param_keys)
        if spec.version_param_key:
            keys.add(spec.version_param_key)
    # Non-ctypes tuning experiments may still attach these in scenario params.
    keys.update({"arx_ax_coeffs", "arx_wz_coeffs"})
    return frozenset(keys)


_HAS_WZ = False


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

    lib.vm_reset_full.restype = None
    lib.vm_reset_full.argtypes = [c_void_p] + [c_double] * 8

    lib.vm_reset_state.restype = None
    lib.vm_reset_state.argtypes = [c_void_p] + [c_double] * 8

    lib.vm_set_queues.restype = None
    lib.vm_set_queues.argtypes = [
        c_void_p,
        ctypes.POINTER(c_double),
        ctypes.c_int,
        ctypes.POINTER(c_double),
        ctypes.c_int,
    ]

    lib.vm_get_acc_q_size.restype = ctypes.c_int
    lib.vm_get_acc_q_size.argtypes = [c_void_p]
    lib.vm_get_steer_q_size.restype = ctypes.c_int
    lib.vm_get_steer_q_size.argtypes = [c_void_p]

    lib.vm_set_input.restype = None
    lib.vm_set_input.argtypes = [c_void_p, c_double, c_double]

    lib.vm_step.restype = None
    lib.vm_step.argtypes = [c_void_p]

    lib.vm_step_dt.restype = None
    lib.vm_step_dt.argtypes = [c_void_p, c_double]

    for fn in (
        "vm_get_x",
        "vm_get_y",
        "vm_get_yaw",
        "vm_get_vx",
        "vm_get_vy",
        "vm_get_steer",
        "vm_get_ax",
    ):
        getattr(lib, fn).restype = c_double
        getattr(lib, fn).argtypes = [c_void_p]

    global _HAS_WZ  # noqa: PLW0603
    if hasattr(lib, "vm_get_wz"):
        lib.vm_get_wz.restype = c_double
        lib.vm_get_wz.argtypes = [c_void_p]
        _HAS_WZ = True
    else:
        _HAS_WZ = False
        print(
            "[WARN] libvehicle_model_wrapper.so に vm_get_wz が無いため sim_wz は NaN。"
            "simple_sensor_simulator を再ビルドしてください。",
            file=sys.stderr,
        )

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
        self._steer_bias = float(params.get(spec.steer_bias_key, 0.0)) if spec.steer_bias_key else 0.0

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr and hasattr(self, "_lib") and self._lib:
            self._lib.vm_destroy(self._ptr)
            self._ptr = None

    @property
    def acc_q_size(self) -> int:
        return self._lib.vm_get_acc_q_size(self._ptr)

    @property
    def steer_q_size(self) -> int:
        return self._lib.vm_get_steer_q_size(self._ptr)

    def reset_with_history(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        steer_actual: float,
        ax: float,
        acc_history: list[float],
        steer_history: list[float],
        wz: float = 0.0,
        vy: float = 0.0,
    ) -> None:
        self._lib.vm_reset_state(self._ptr, x, y, yaw, vx, steer_actual, ax, wz, vy)

        n_acc = len(acc_history)
        n_steer = len(steer_history)
        arr_acc = (ctypes.c_double * n_acc)(*acc_history)
        arr_steer = (ctypes.c_double * n_steer)(*steer_history)
        self._lib.vm_set_queues(
            self._ptr,
            arr_acc,
            ctypes.c_int(n_acc),
            arr_steer,
            ctypes.c_int(n_steer),
        )

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
    ) -> None:
        self._lib.vm_reset_state(self._ptr, x, y, yaw, vx, steer_actual, ax, wz, vy)
        self._lib.vm_set_queues(
            self._ptr,
            acc_ptr,
            ctypes.c_int(n_acc),
            steer_ptr,
            ctypes.c_int(n_steer),
        )

    def step(self, accel_des: float, steer_des: float) -> None:
        self._lib.vm_set_input(self._ptr, accel_des, steer_des)
        self._lib.vm_step(self._ptr)

    def step_dt(self, accel_des: float, steer_des: float, dt: float) -> None:
        self._lib.vm_set_input(self._ptr, accel_des, steer_des)
        self._lib.vm_step_dt(self._ptr, ctypes.c_double(dt))

    @property
    def x(self) -> float:
        return self._lib.vm_get_x(self._ptr)

    @property
    def y(self) -> float:
        return self._lib.vm_get_y(self._ptr)

    @property
    def yaw(self) -> float:
        return self._lib.vm_get_yaw(self._ptr)

    @property
    def vx(self) -> float:
        return self._lib.vm_get_vx(self._ptr)

    @property
    def ax(self) -> float:
        return self._lib.vm_get_ax(self._ptr)

    @property
    def vy(self) -> float:
        return self._lib.vm_get_vy(self._ptr)

    @property
    def wz(self) -> float:
        if not _HAS_WZ:
            return float("nan")
        return self._lib.vm_get_wz(self._ptr)

    @property
    def steer_state(self) -> float:
        return self._lib.vm_get_steer(self._ptr) - self._steer_bias
