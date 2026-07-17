"""GT フレーム (observation_frame) の単体テスト。

localization_consistent フレームは pose–twist 不整合 (pose 実効ラグ +93 ms / twist vx
+0.4% 過大、report 9 章) を観測モデル同定値で補正し、cmd→ax→vx→long の縦系チェーンを
単一系統で閉じる。ここでは合成データで補正の運動学整合と配管 (build_rollout_data /
scenario パース / fit.freeze) を検証する。
"""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest
import yaml

from driving_log_replayer_v2.real_log_sim_comparison.lib._localization_observation import (
    TWIST_VX_LAG_S,
    TWIST_VX_SCALE,
    consistent_kinematic_frame,
    normalize_observation_frame,
    twist_vx_localization_view,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_merge
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.load_data import (
    build_rollout_data,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.model_config import (
    load_model_config,
)


def _synthetic_kin(n: int = 2000, dt: float = 0.01) -> tuple[pd.DataFrame, np.ndarray, np.ndarray]:
    """真値 (v_true, s_true) から localization 観測 (twist 遅延 + 過大) を合成する。

    pose は真値位置 (地図アンカー)。twist vx は真値速度を +TWIST_VX_LAG_S 遅れて
    1/TWIST_VX_SCALE 倍過大に出力する (実測の観測モデル、report 9 章)。
    """
    t = np.arange(n) * dt
    v_true = 10.0 + np.sin(0.5 * t)
    s_true = np.concatenate([[0.0], np.cumsum((v_true[1:] + v_true[:-1]) * 0.5 * dt)])
    vx_twist = np.interp(t - TWIST_VX_LAG_S, t, v_true) / TWIST_VX_SCALE
    df = pd.DataFrame(
        {
            "t_ns": (t * 1e9).astype(np.int64),
            "x": s_true,
            "y": np.zeros(n),
            "yaw": np.zeros(n),
            "vx": vx_twist,
            "wz": np.zeros(n),
            "pitch": np.zeros(n),
        }
    )
    return df, t, v_true


def test_normalize_observation_frame_accepts_known_and_rejects_unknown() -> None:
    assert normalize_observation_frame(None) == "raw"
    assert normalize_observation_frame("raw") == "raw"
    assert normalize_observation_frame("localization_consistent") == "localization_consistent"
    with pytest.raises(ValueError, match="observation_frame"):
        normalize_observation_frame("pose_only")


def test_consistent_frame_restores_pose_twist_kinematic_consistency() -> None:
    df, t, v_true = _synthetic_kin()
    vx_before = df["vx"].to_numpy().copy()
    out = consistent_kinematic_frame(df)

    # twist vx の実効遅延とスケールが除去され真値速度へ戻る (両端の外挿クランプ域は除く)。
    core = slice(10, len(t) - 20)
    np.testing.assert_allclose(out["vx"].to_numpy()[core], v_true[core], atol=1e-4)
    # pose は真値アンカーとして無補正。
    np.testing.assert_allclose(out["x"].to_numpy(), df["x"].to_numpy())

    # 補正後は「pose 変位 = ∫vx dt」が窓によらず成立する (raw では系統的に食い違う)。
    dt = 0.01
    k0, k1 = 300, 1300
    disp_pose = out["x"].iloc[k1] - out["x"].iloc[k0]
    vx = out["vx"].to_numpy()
    disp_twist = np.sum((vx[k0:k1] + vx[k0 + 1 : k1 + 1]) * 0.5 * dt)
    assert disp_pose == pytest.approx(disp_twist, abs=2e-3)

    raw_vx = df["vx"].to_numpy()
    raw_disp_twist = np.sum((raw_vx[k0:k1] + raw_vx[k0 + 1 : k1 + 1]) * 0.5 * dt)
    assert abs(disp_pose - raw_disp_twist) > 0.3  # 元の不整合 (~0.4 m / 100 m)

    # 元の DataFrame は不変。
    np.testing.assert_allclose(df["vx"].to_numpy(), vx_before)


def test_twist_view_is_inverse_of_consistent_frame() -> None:
    """本番 publisher 向け視点変換 (真値→twist) が整合フレーム補正の逆変換になっている。"""
    n, dt = 2000, 0.01
    t = np.arange(n) * dt
    v_true = 10.0 + np.sin(0.5 * t)
    vx_twist = twist_vx_localization_view(v_true, dt)
    df = pd.DataFrame(
        {
            "t_ns": (t * 1e9).astype(np.int64),
            "x": np.zeros(n), "y": np.zeros(n), "yaw": np.zeros(n),
            "vx": vx_twist, "wz": np.zeros(n), "pitch": np.zeros(n),
        }
    )
    out = consistent_kinematic_frame(df)
    core = slice(20, n - 20)
    np.testing.assert_allclose(out["vx"].to_numpy()[core], v_true[core], atol=5e-3)


def _synthetic_dfs() -> dict[str, pd.DataFrame]:
    df_kin, t, _ = _synthetic_kin()
    n = len(t)
    t_ns = df_kin["t_ns"]
    return {
        "kinematic": df_kin,
        "velocity": pd.DataFrame({"t_ns": t_ns, "lon_vel": df_kin["vx"]}),
        "accel": pd.DataFrame({"t_ns": t_ns, "accel": np.zeros(n)}),
        "steering": pd.DataFrame({"t_ns": t_ns, "steer": np.zeros(n)}),
        "cmd": pd.DataFrame({"t_ns": t_ns, "cmd_accel": np.zeros(n), "cmd_steer": np.zeros(n)}),
        "mode": pd.DataFrame({"t_ns": t_ns, "mode": np.full(n, 2)}),
        "gear": pd.DataFrame({"t_ns": t_ns, "gear": np.full(n, 2)}),
    }


def test_build_rollout_data_applies_consistent_frame_to_kin_and_derived_accel() -> None:
    dfs = _synthetic_dfs()
    raw = build_rollout_data(dfs, acceleration_source="kinematic_savgol")
    consistent = build_rollout_data(
        dfs, acceleration_source="kinematic_savgol",
        observation_frame="localization_consistent",
    )

    # raw は従来どおり無補正。
    np.testing.assert_allclose(raw["kin"]["vx"], dfs["kinematic"]["vx"])
    # consistent は twist vx を +lag 前進 + ×scale した kin を返し、kinematic_* 系の
    # 加速度 GT も補正済み vx から導出される (同じ位相前進 + スケール)。
    t = dfs["kinematic"]["t_ns"].to_numpy(float) * 1e-9
    raw_vx = dfs["kinematic"]["vx"].to_numpy(float)
    np.testing.assert_allclose(
        consistent["kin"]["vx"],
        np.interp(t + TWIST_VX_LAG_S, t, raw_vx) * TWIST_VX_SCALE,
        atol=1e-9,
    )
    core = slice(100, len(t) - 100)
    t_acc = raw["acc"]["t_ns"].to_numpy(float) * 1e-9
    expected_ax = np.interp(
        t_acc + TWIST_VX_LAG_S, t_acc, raw["acc"]["ax"].to_numpy(float),
    ) * TWIST_VX_SCALE
    np.testing.assert_allclose(
        consistent["acc"]["ax"].to_numpy()[core], expected_ax[core], atol=1e-4,
    )
    with pytest.raises(ValueError, match="observation_frame"):
        build_rollout_data(dfs, observation_frame="bogus")


def _observation_scenario_document(conditions_extra: dict) -> dict:
    conditions = {
        "comparison_models": ["baseline"],
        "fit": {"stages": []},
        "models": {
            "baseline": {
                "vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                "params": {"wheelbase": 4.7},
            },
        },
    }
    conditions.update(conditions_extra)
    return {"Evaluation": {"Conditions": conditions}}


def test_scenario_parses_observation_frame_and_defaults_to_raw(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_observation_scenario_document({})), encoding="utf-8",
    )
    assert load_model_config(scenario).observation_frame == "raw"

    scenario.write_text(
        yaml.safe_dump(
            _observation_scenario_document({"observation_frame": "localization_consistent"})
        ),
        encoding="utf-8",
    )
    assert load_model_config(scenario).observation_frame == "localization_consistent"

    scenario.write_text(
        yaml.safe_dump(_observation_scenario_document({"observation_frame": "bogus"})),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="observation_frame"):
        load_model_config(scenario)


def test_scenario_parses_fit_freeze_and_rejects_unknown_names(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    freeze = ["lon_drag_c0", "steer_relaxation_length"]
    document = _observation_scenario_document(
        {"fit": {"stages": [], "freeze": freeze}}
    )
    scenario.write_text(yaml.safe_dump(document), encoding="utf-8")
    assert load_model_config(scenario).fit.freeze == tuple(freeze)

    document = _observation_scenario_document(
        {"fit": {"stages": [], "freeze": ["not_a_parameter"]}}
    )
    scenario.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="fit.freeze"):
        load_model_config(scenario)

    document = _observation_scenario_document(
        {"fit": {"stages": [], "freeze": ["lon_drag_c0", "lon_drag_c0"]}}
    )
    scenario.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="重複"):
        load_model_config(scenario)


def test_robust_search_excludes_frozen_parameters_from_search(monkeypatch) -> None:
    seen: list[dict] = []

    def fake_evaluate_candidate(
        _ctxs, params, _model_type, _source, _steer_source="steer", _slope_source="none", **_kwargs,
    ):
        seen.append(dict(params))
        return {
            "by_h": {
                horizon: {
                    f"{key}_{stat}": 1.0
                    for key in ("nyaw", "nlong", "nlat", "nsteer", "nax")
                    for stat in ("mean", "worst", "cvar")
                }
                for horizon in fit_merge.HORIZONS
            }
        }

    monkeypatch.setattr(fit_merge, "_evaluate_candidate", fake_evaluate_candidate)
    context = fit_merge.DatasetCtx(
        dataset_id="dataset-a", dfs={}, t0_ns=0, base={"acc_time_delay": 0.1},
    )
    case = SimpleNamespace(
        params={"acc_time_constant": 0.3, "acc_time_delay": 0.1},
        vehicle_model_type="delay_steer_acc_geared_for_diffusion_planner",
        acceleration_source="accel",
    )
    config = SimpleNamespace(
        find_case=lambda _name: case,
        fit=SimpleNamespace(
            target="current",
            freeze=("acc_time_constant", "steer_relaxation_length"),
        ),
    )

    result = fit_merge.robust_search(
        [context], config, n_trials=3, n_jobs=1,
        direct_fit_params={"acc_time_constant": 0.42, "steer_relaxation_length": 0.0},
    )

    # 凍結キーは探索されず direct-fit 値のまま透過される。
    assert result["acc_time_constant"] == pytest.approx(0.42)
    assert result["steer_relaxation_length"] == pytest.approx(0.0)
    assert all(p["acc_time_constant"] == pytest.approx(0.42) for p in seen[1:])
    # 非凍結キーは通常どおり探索される。
    assert any("steer_time_constant" in p for p in seen[1:])
