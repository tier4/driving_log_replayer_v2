"""縦横モデルビューアのつまみ初期値を組み立てる helper."""

from __future__ import annotations


def _seed_from_params(params: dict) -> dict:
    """モデルレジストリ/spec の params dict から lon_lat_model のつまみシードを構築する."""

    def f(key: str, default: float) -> float:
        try:
            return float(params.get(key, default))
        except (TypeError, ValueError):
            return float(default)

    acc_tc = f("acc_time_constant", 0.1)
    acc_td = f("acc_time_delay", 0.1)
    brake_tc = params.get("brake_time_constant")
    brake_tc = float(brake_tc) if brake_tc not in (None, "", 0, 0.0) else acc_tc
    return {
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
