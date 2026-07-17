from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from driving_log_replayer_v2.real_log_sim_comparison.reidentify.analyze import (
    conditioned,
    gt_quality,
    pitch_sign,
    regime,
    steady_state,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.analyze.split import (
    HOLDOUT_PCT,
    split_dataset_ids,
    split_of,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.rollout import _lowpass_signal
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import RESAMPLE_DT


def test_split_is_deterministic() -> None:
    ds = "0a246ffe-e8a6-5027-94f6-b860ecdda887"
    assert split_of(ds) == split_of(ds)
    assert split_of(ds) in ("dev", "holdout")


def test_split_fraction_is_close_to_holdout_pct() -> None:
    ids = [f"dataset-{i:04d}" for i in range(2000)]
    dev, holdout = split_dataset_ids(ids)
    assert len(dev) + len(holdout) == len(ids)
    ratio = len(holdout) / len(ids)
    assert abs(ratio - HOLDOUT_PCT / 100.0) < 0.03


def _synthetic_resampled(slope_sign: float) -> dict:
    n = 6000  # 60 s @ dt=0.01
    t = np.arange(n) * RESAMPLE_DT
    pitch = 0.02 * np.sin(2 * np.pi * t / 30.0)  # ±1.15 deg の勾配変動
    a_act = slope_sign * pitch_sign.GRAVITY * np.sin(pitch) - 0.1
    return {
        "a_cmd": np.zeros(n, dtype=np.float32),
        "a_act": a_act.astype(np.float32),
        "vx": np.full(n, 10.0, dtype=np.float32),
        "pitch": pitch.astype(np.float32),
        "gear_drive": np.ones(n, dtype=bool),
    }


def test_pitch_sign_regression_recovers_positive_slope() -> None:
    reg = pitch_sign._dataset_regression(_synthetic_resampled(+1.0))
    assert reg is not None
    assert reg["slope"] == pytest.approx(1.0, abs=0.05)
    assert reg["intercept"] == pytest.approx(-0.1, abs=0.02)
    assert reg["r2"] > 0.99


def test_pitch_sign_regression_recovers_negative_slope() -> None:
    reg = pitch_sign._dataset_regression(_synthetic_resampled(-1.0))
    assert reg is not None
    assert reg["slope"] == pytest.approx(-1.0, abs=0.05)


def test_pitch_sign_regression_rejects_flat_pitch() -> None:
    res = _synthetic_resampled(+1.0)
    res["pitch"] = np.zeros_like(res["pitch"])
    res["a_act"] = np.full_like(res["a_act"], -0.1)
    reg = pitch_sign._dataset_regression(res)
    assert reg is not None
    assert np.isnan(reg["slope"])  # 勾配情報なし → 判定保留


def _synthetic_traces(n_datasets: int = 12, rows: int = 400, seed: int = 0) -> pd.DataFrame:
    """err_ax@30 が g·sin(pitch_lf) に比例 (+ノイズ) する合成トレース。"""
    rng = np.random.default_rng(seed)
    frames = []
    for i in range(n_datasets):
        pitch_lf = rng.normal(0.0, 0.01, rows)
        g_sin = conditioned.GRAVITY * np.sin(pitch_lf)
        frames.append(pd.DataFrame({
            "dataset_id": f"ds-{i:03d}",
            "horizon": 30,
            "err_ax": 1.0 * g_sin + rng.normal(0.0, 0.05, rows),
            "err_steer_deg": rng.normal(0.0, 0.1, rows),
            "err_yaw_deg": rng.normal(0.0, 0.1, rows),
            "err_lat_cm": rng.normal(0.0, 5.0, rows),
            "pitch_lf_mean": pitch_lf,
            "vx_mean": rng.uniform(3.0, 15.0, rows),
            "a_cmd_mean": rng.normal(0.0, 0.5, rows),
            "jerk_cmd_abs_mean": np.abs(rng.normal(0.0, 0.3, rows)),
            "steer_rate_mean": rng.normal(0.0, 0.01, rows),
            "ay_mean": rng.normal(0.0, 0.5, rows),
        }))
    return pd.concat(frames, ignore_index=True)


def test_conditioned_detects_planted_pitch_dependency() -> None:
    summary, bins, slopes = conditioned.analyze_conditioned(_synthetic_traces())
    hit = summary[
        (summary["target"] == "err_ax") & (summary["feature"] == "g_sin_pitch_lf")
    ].iloc[0]
    assert bool(hit["significant"])
    assert hit["pooled_slope"] == pytest.approx(1.0, abs=0.1)
    assert hit["sign_agreement"] == 1.0
    # 仕込んでいない特徴は有意にならない。
    miss = summary[
        (summary["target"] == "err_ax") & (summary["feature"] == "ay_mean")
    ].iloc[0]
    assert not bool(miss["significant"])
    assert not bins.empty
    assert not slopes.empty


def test_counterfactual_predicts_slope_feed_reduction() -> None:
    cf = conditioned.counterfactual_corrections(_synthetic_traces())
    beta1 = cf[cf["correction"] == "slope_feed_beta1"].iloc[0]
    # 残差の主部が g·sin(pitch) なので β=1 注入で大きく減る。
    assert beta1["reduction_pct"] > 20.0


def test_bh_adjust_monotone_and_bounded() -> None:
    adjusted = conditioned._bh_adjust([0.01, 0.04, 0.03, float("nan"), 0.5])
    finite = [p for p in adjusted if np.isfinite(p)]
    assert all(0.0 <= p <= 1.0 for p in finite)
    assert np.isnan(adjusted[3])
    raw = [0.01, 0.04, 0.03, 0.5]
    fin = [p for i, p in enumerate(adjusted) if i != 3]
    assert all(a >= r for a, r in zip(fin, raw))


def test_steady_points_and_gain() -> None:
    n = 12000
    t = np.arange(n) * RESAMPLE_DT
    # 前半: 一定スロットル +1.0 (achieved 1.0)、後半: 一定ブレーキ -1.0 (achieved -0.7)。
    a_cmd = np.where(t < t[n // 2], 1.0, -1.0)
    a_act = np.where(t < t[n // 2], 1.0, -0.7)
    res = {
        "a_cmd": a_cmd.astype(np.float32),
        "a_act": a_act.astype(np.float32),
        "d_cmd": np.full(n, 0.1, dtype=np.float32),
        "d_act": np.full(n, 0.105, dtype=np.float32),
        "vx": np.full(n, 8.0, dtype=np.float32),
        "pitch": np.zeros(n, dtype=np.float32),
        "gear_drive": np.ones(n, dtype=bool),
    }
    pts = steady_state._steady_points(res)
    assert len(pts) > 0.8 * n  # 遷移点近傍以外は定常
    gains = steady_state._gain_through_origin(
        pts["a_cmd"].values, pts["a_act"].values, *steady_state.ACC_GAIN_RANGE,
    )
    assert gains["gain_pos"] == pytest.approx(1.0, abs=0.01)
    assert gains["gain_neg"] == pytest.approx(0.7, abs=0.01)


def test_regime_classification_boundaries() -> None:
    labels = regime.classify_regime(np.array([-1.0, -0.3, -0.29, 0.0, 0.29, 0.31]))
    assert list(labels) == ["brake", "coast", "coast", "coast", "coast", "throttle"]


def test_regime_split_detects_brake_bias() -> None:
    rng = np.random.default_rng(1)
    frames = []
    for i in range(8):
        n = 200
        a_cmd = rng.uniform(-1.5, 1.5, n)
        err = np.where(a_cmd < regime.BRAKE_A_CMD, 0.2, 0.0) + rng.normal(0, 0.05, n)
        frames.append(pd.DataFrame({
            "dataset_id": f"ds-{i}",
            "horizon": 30,
            "a_cmd_mean": a_cmd,
            "err_ax": err,
            "err_vx": np.zeros(n),
            "err_long_cm": np.zeros(n),
        }))
    table = regime.analyze_regimes(pd.concat(frames, ignore_index=True))
    ax30 = table[(table["target"] == "err_ax") & (table["horizon"] == 30)].set_index("regime")
    assert ax30.loc["brake", "mean"] == pytest.approx(0.2, abs=0.03)
    assert abs(ax30.loc["throttle", "mean"]) < 0.03
    assert ax30.loc["brake", "mean_ci_lo"] > ax30.loc["throttle", "mean_ci_hi"]


def test_rts_smoother_beats_wide_savgol_on_transients() -> None:
    """RTS は広窓 savgol より過渡忠実 (積分整合) かつノイズを抑えられる。"""
    rng = np.random.default_rng(3)
    dt = 0.01
    n = 6000
    t = np.arange(n) * dt
    # ブレーキ様の鋭い減速パルスを含む真の加速度。
    a_true = 0.4 * np.sin(2 * np.pi * 0.1 * t)
    a_true[2000:2150] -= 1.5
    v_true = 8.0 + np.cumsum(a_true) * dt
    vx_meas = v_true + rng.normal(0.0, 0.03, n)

    a_rts = gt_quality.rts_smooth_accel(vx_meas, dt, q_jerk=0.2)
    a_sg = gt_quality._savgol_deriv_window(vx_meas, dt, 0.4)

    # q=0.2 は現行 savgol 0.4 s を過渡・雑音の両軸で支配する (合成検証 2026-07-17)。
    pulse = slice(1950, 2250)
    calm = slice(3000, 5000)
    rmse_rts = float(np.sqrt(np.mean((a_rts[pulse] - a_true[pulse]) ** 2)))
    rmse_sg = float(np.sqrt(np.mean((a_sg[pulse] - a_true[pulse]) ** 2)))
    noise_rts = float(np.std(a_rts[calm] - a_true[calm]))
    noise_sg = float(np.std(a_sg[calm] - a_true[calm]))
    assert rmse_rts < rmse_sg
    assert noise_rts < noise_sg * 1.05


def test_integral_consistency_penalizes_smoothing_loss() -> None:
    """減衰した GT は Δv 整合性誤差が大きく出る。"""
    dt = 0.01
    n = 4000
    a_true = np.zeros(n)
    a_true[1000:1150] = -2.0
    vx = 8.0 + np.cumsum(a_true) * dt
    a_cmd = a_true.copy()
    gear = np.ones(n, dtype=bool)
    masks = gt_quality._regime_masks(a_cmd, gear, dt, 1.0)
    exact = gt_quality._integral_consistency(a_true, vx, dt, 1.0, masks)
    damped = gt_quality._integral_consistency(0.7 * a_true, vx, dt, 1.0, masks)
    # サンプリング規約の O(dt) 差のみ許容し、減衰 GT との 1 桁以上の分離を確認する。
    assert exact["brake"] < 0.02
    assert damped["brake"] > 0.1
    assert damped["brake"] > 5 * exact["brake"]


def test_lowpass_separates_slow_component() -> None:
    t = np.arange(0, 120.0, 0.02)  # 50 Hz, 120 s
    slow = 0.05 * np.sin(2 * np.pi * 0.02 * t)  # 0.02 Hz (通過域)
    fast = 0.05 * np.sin(2 * np.pi * 1.0 * t)  # 1 Hz (阻止域)
    out = _lowpass_signal(slow + fast, t, cutoff_hz=0.1)
    mid = slice(len(t) // 4, 3 * len(t) // 4)
    residual = out[mid] - slow[mid]
    assert float(np.std(residual)) < 0.2 * float(np.std(fast))


def test_lowpass_short_series_falls_back() -> None:
    t = np.arange(0, 0.1, 0.02)
    values = np.ones_like(t)
    out = _lowpass_signal(values, t, cutoff_hz=0.1)
    assert out.shape == values.shape
    assert np.all(np.isfinite(out))
