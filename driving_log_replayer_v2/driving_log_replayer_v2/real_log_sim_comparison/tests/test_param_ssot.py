"""パラメータ名の SSOT (PARAMETER_CONSTRAINTS) と外部契約の整合性を監視する。

C++ ABI の引数順・ROS YAML のリネームマップは導出できない外部契約なので、
ここでドリフトを検知する。
"""
from __future__ import annotations

from driving_log_replayer_v2.real_log_sim_comparison.lib._vehicle_models import (
    VEHICLE_MODEL_SPECS,
    get_vehicle_model_spec,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_merge
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.parameter_constraints import (
    ALL_CONSTRAINED_KEYS,
    DIRECT_FIT_STAGES,
    FIT_LON,
    FIT_MERGE,
    FIT_STEER,
    FIT_XY,
    stage_input_keys,
    stage_targets,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.release_params import (
    _GLOBAL_PARAM_KEYS,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import (
    TARGET_MODEL_TYPE,
)

_MINIMAL_PARAMS = {
    "vel_lim": 30.0,
    "steer_lim": 0.7,
    "vel_rate_lim": 5.0,
    "steer_rate_lim": 5.0,
    "wheelbase": 4.7,
    "acc_time_delay": 0.1,
    "acc_time_constant": 0.3,
    "steer_time_delay": 0.05,
    "steer_time_constant": 0.3,
    "steer_dead_band": 0.0,
    "steer_bias": 0.0,
}


def test_build_args_match_factory_arg_count() -> None:
    """C++ factory の引数個数 (ABI) と build_args の出力が一致すること。"""
    for model_type, spec in VEHICLE_MODEL_SPECS.items():
        args = spec.build_args(_MINIMAL_PARAMS, 1.0 / 30.0)
        assert len(args) == spec.factory_arg_count, model_type


def test_constrained_keys_are_target_model_params() -> None:
    """制約登録済みキーはすべてリリース対象モデルの実パラメータであること。"""
    spec = get_vehicle_model_spec(TARGET_MODEL_TYPE)
    assert ALL_CONSTRAINED_KEYS <= spec.param_keys


def test_target_model_passthrough_keys_are_documented() -> None:
    """制約系の外にあるパラメータは既知のパススルー集合だけであること (ドリフト警報)。"""
    spec = get_vehicle_model_spec(TARGET_MODEL_TYPE)
    passthrough = spec.param_keys - ALL_CONSTRAINED_KEYS
    assert passthrough == {"vel_lim", "steer_lim", "vel_rate_lim", "sub_dt", "use_rk4"}


def test_release_global_keys_are_model_params() -> None:
    """リリース YAML のグローバル欄リネームマップのキーはモデルパラメータであること。"""
    spec = get_vehicle_model_spec(TARGET_MODEL_TYPE)
    assert set(_GLOBAL_PARAM_KEYS) <= spec.param_keys


def test_direct_fit_keys_cover_all_constraints() -> None:
    assert fit_merge._DIRECT_FIT_KEYS == ALL_CONSTRAINED_KEYS


def test_stage_targets_are_nonempty_and_chained() -> None:
    """各ステージに最適化対象があり、入力キーが前段までの対象の和であること。"""
    for stage in (FIT_LON, FIT_STEER, FIT_XY, FIT_MERGE):
        assert stage_targets(stage)
    assert stage_input_keys(FIT_LON) == frozenset()
    assert stage_input_keys(FIT_STEER) == stage_targets(FIT_LON)
    assert stage_input_keys(FIT_XY) == stage_targets(FIT_LON) | stage_targets(FIT_STEER)
    assert DIRECT_FIT_STAGES == (FIT_LON, FIT_STEER, FIT_XY)
