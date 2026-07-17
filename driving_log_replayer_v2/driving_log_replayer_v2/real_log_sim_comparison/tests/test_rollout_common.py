from __future__ import annotations

import numpy as np
import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib._nstep_common import (
    MEAN_METRIC_KEYS,
    METRIC_KEYS,
)
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


class _FakeLibraryUndershoot:
    """sim が GT を下回る (err = GT - sim > 0) 終端状態を返すフェイク。"""

    @staticmethod
    def vm_integrate_to_horizons(*args) -> None:
        x_out, y_out, yaw_out, vx_out, ax_out, steer_out = args[-6:]
        x_out[0] = 1.0
        y_out[0] = 0.0
        yaw_out[0] = 0.0
        vx_out[0] = 4.5
        ax_out[0] = -0.1
        steer_out[0] = 0.15


class _FakeVehicleModelUndershoot(_FakeVehicleModel):
    def __init__(self, _params: dict, _dt: float, _model_type: str) -> None:
        super().__init__(_params, _dt, _model_type)
        self._lib = _FakeLibraryUndershoot()


def test_rollout_default_result_has_no_mean_keys(monkeypatch) -> None:
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModel)

    result = rollout.eval_rollout_rmse(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.0},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(),
    )

    assert set(result[1]) == set(METRIC_KEYS)


def test_rollout_include_mean_returns_signed_errors(monkeypatch) -> None:
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModelUndershoot)

    result = rollout.eval_rollout_rmse(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.0},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(),
        include_mean=True,
    )

    metrics = result[1]
    assert set(metrics) == set(METRIC_KEYS) | set(MEAN_METRIC_KEYS)
    # err = GT - sim。GT steer=0.17, sim=0.15 → 正のアンダーシュート。
    assert metrics["steer_mean"] == pytest.approx(np.degrees(0.02))
    assert metrics["vx_mean"] == pytest.approx(0.5)
    assert metrics["ax_mean"] == pytest.approx(0.1)
    # サンプル 1 点なので RMSE = |mean|。
    assert metrics["steer"] == pytest.approx(abs(metrics["steer_mean"]))


def test_terminal_errors_match_signed_error_convention(monkeypatch) -> None:
    """eval_rollout_terminal_errors は include_mean と同じ署名規約 (err = GT - sim)。"""
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModelUndershoot)

    df = rollout.eval_rollout_terminal_errors(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.0},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(),
    )

    assert list(df.columns) == list(rollout.TERMINAL_ERROR_COLUMNS)
    assert len(df) == 1
    row = df.iloc[0]
    assert row["horizon"] == 1
    assert row["err_steer_deg"] == pytest.approx(np.degrees(0.02))
    assert row["err_vx"] == pytest.approx(0.5)
    assert row["err_ax"] == pytest.approx(0.1)
    # フェイク GT に pitch が無い場合はゼロ埋めで動作する。
    assert row["pitch_mean"] == pytest.approx(0.0)
    assert row["pitch_lf_mean"] == pytest.approx(0.0)


def test_terminal_errors_empty_when_no_valid_start(monkeypatch) -> None:
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModel)

    df = rollout.eval_rollout_terminal_errors(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.0},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(vx=0.0),
    )

    assert df.empty
    assert list(df.columns) == list(rollout.TERMINAL_ERROR_COLUMNS)


def test_rollout_include_mean_marks_invalid_horizon_as_infinite(monkeypatch) -> None:
    monkeypatch.setattr(rollout, "VehicleModel", _FakeVehicleModel)

    result = rollout.eval_rollout_rmse(
        data={},
        t0_ns=0,
        params={"steer_bias": 0.0},
        model_type="delay_steer_acc_geared_for_diffusion_planner",
        horizons=(1,),
        stride=1,
        gt=_ground_truth(vx=0.0),
        include_mean=True,
    )

    assert set(result[1]) == set(METRIC_KEYS) | set(MEAN_METRIC_KEYS)
    assert all(np.isinf(value) for value in result[1].values())
