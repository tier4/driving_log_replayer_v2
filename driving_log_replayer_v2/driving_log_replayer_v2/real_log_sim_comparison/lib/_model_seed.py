"""縦横モデルビューアのつまみ初期値を組み立てる helper."""

from __future__ import annotations


def _seed_from_params(params: dict) -> dict:
    """モデルレジストリ/spec の params dict からモデル検証つまみのシードを構築する."""

    def f(key: str, default: float) -> float:
        try:
            return float(params.get(key, default))
        except (TypeError, ValueError):
            return float(default)

    acc_tc = f("acc_time_constant", 0.1)
    acc_td = f("acc_time_delay", 0.1)
    brake_tc = params.get("brake_time_constant")
    brake_tc = float(brake_tc) if brake_tc not in (None, "", 0, 0.0) else acc_tc
    # full-RHS 遅延（新モデル DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER）か、指令のみ遅延
    # （旧モデル wo_fall_guard）か。ビューアの遅延モードトグルの既定値を run のモデル種別から決める。
    # 未指定 / 旧モデルは False（＝指令のみ遅延）で既存挙動を維持。
    full_rhs_delay = (
        str(params.get("vehicle_model_type", "")).upper()
        == "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER"
    )
    return {
        "full_rhs_delay": full_rhs_delay,
        "tau_acc_thr": acc_tc,
        "t_acc_thr": acc_td,
        "tau_acc_brk": brake_tc,
        "t_acc_brk": acc_td,
        "tau_acc_slope": 0.0,
        "poly0": f("lon_drag_c0", 0.0),
        "poly1": f("lon_drag_c1", 0.0),
        "poly2": f("lon_drag_c2", 0.0),
        "v_stop": 0.2,
        "c_slope": f("lon_slope_gain", 1.0),
        "tau_steer": f("steer_time_constant", 0.27),
        "t_steer": f("steer_time_delay", 0.24),
        "steer_bias": f("steer_bias", 0.0),
        "k_us": f("k_us", 0.0),
        "k_us_bands": params.get("k_us_bands"),
        "k_us_thresholds": params.get("k_us_thresholds"),
        "steer_scaling": f("debug_steer_scaling_factor", 1.0),
        "acc_scaling": f("debug_acc_scaling_factor", 1.0),
        "steer_dead_band": f("steer_dead_band", 0.0),
    }
