from __future__ import annotations

import numpy as np
import pytest

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import rollout


class _FakeLibrary:
    @staticmethod
    def vm_integrate_to_horizons(*args) -> None:
        x_out, y_out, yaw_out, vx_out, ax_out, steer_out = args[-6:]
        x_out[0] = 1.0
        y_out[0] = 0.0
        yaw_out[0] = 0.0
        vx_out[0] = 5.0
        ax_out[0] = 0.0
        steer_out[0] = 0.17


class _FakeVehicleModel:
    resets: list[dict] = []

    def __init__(self, _params: dict, _dt: float, _model_type: str) -> None:
        self._lib = _FakeLibrary()
        self._ptr = None

    def reset_with_history_ptr(self, **kwargs) -> None:
        self.resets.append(kwargs)


def _ground_truth(vx: float = 5.0) -> dict:
    zeros = np.zeros(2, dtype=np.float64)
    return {
        "t_cmd": np.array([0.0, 0.1]),
        "gt_x": np.array([0.0, 1.0]),
        "gt_y": zeros,
        "gt_yaw": zeros,
        "gt_vx": np.full(2, vx),
        "gt_steer": np.full(2, 0.17),
        "gt_steer_kinematic": np.full(2, -0.40),
        "gt_wz": zeros,
        "gt_ax": zeros,
        "gt_vy": zeros,
        "nfull_arr": np.array([1], dtype=np.int32),
        "rem_arr": np.array([0.0]),
        "bad_iv_cumsum": np.array([0, 0]),
        "bad_gear_cumsum": np.array([0, 0, 0]),
        "acc_hist_all": np.zeros((2, 1)),
        "steer_hist_all": np.zeros((2, 1)),
        "cos_y_arr": np.ones(2),
        "sin_y_arr": zeros,
        "accel_des_arr": zeros,
        "steer_des_arr": zeros,
    }


def test_rollout_initializes_from_measured_steering(monkeypatch) -> None:
    _FakeVehicleModel.resets.clear()
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModel)

    rollout.eval_rollout_rmse(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.02},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(),
    )

    assert _FakeVehicleModel.resets[0]["steer_actual"] == pytest.approx(0.19)
    assert _FakeVehicleModel.resets[0]["steer_actual"] != pytest.approx(-0.38)


def test_rollout_marks_horizon_without_valid_samples_as_infinite(monkeypatch) -> None:
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModel)

    result = rollout.eval_rollout_rmse(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.0},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(vx=0.0),
    )

    assert all(np.isinf(value) for value in result[1].values())
