from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_core
from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_lon
from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_merge
from driving_log_replayer_v2.real_log_sim_comparison.reidentify import fit_steer
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.parameter_constraints import (
    PARAMETER_CONSTRAINTS,
    FIT_LON,
    FIT_MERGE,
    FIT_STEER,
    FIT_XY,
    ParameterConstraint,
    build_constraint_audit,
    clamp_search_parameters,
    stage_targets,
    validate_parameters,
)
from driving_log_replayer_v2.real_log_sim_comparison.reidentify import release_params
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import HORIZONS
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.model_config import (
    load_model_config,
)


def test_first_order_fit_recovers_delay_tau_and_scale() -> None:
    dt = 0.01
    command = np.zeros(500)
    command[100:] = 1.0
    actual = 1.1 * fit_core._simulate(command, 0.2, 10, dt)

    result = fit_core.fit_first_order_delay(
        command,
        actual,
        np.ones(len(command), dtype=bool),
        dt,
        tau_bounds=(0.05, 0.5),
        delay_candidates=(0.0, 0.05, 0.1, 0.15),
        fit_scale=True,
    )

    assert result is not None
    assert result["tau"] == pytest.approx(0.2, abs=0.01)
    assert result["delay"] == pytest.approx(0.1)
    assert result["scale"] == pytest.approx(1.1, abs=0.01)


def test_robust_search_rejects_zero_trials() -> None:
    with pytest.raises(ValueError, match="at least 1"):
        fit_merge.robust_search([], None, n_trials=0, direct_fit_params={})


def test_zero_rollout_metrics_are_not_valid_candidates() -> None:
    metrics = {
        horizon: {key: 0.0 for key in fit_merge._RMSE_KEYS}
        for horizon in fit_merge.HORIZONS
    }

    assert not fit_merge._rollout_metric_is_valid(metrics)


def test_report_horizons_are_dense_without_changing_score_horizons() -> None:
    assert HORIZONS == (10, 30, 70, 150, 300)
    assert fit_merge.HORIZONS is HORIZONS
    assert fit_merge.REPORT_HORIZONS == tuple(range(1, 301))


def test_report_metrics_use_every_dense_horizon(monkeypatch) -> None:
    context = fit_merge.DatasetCtx(
        dataset_id="dataset-a",
        dfs={},
        t0_ns=0,
        base={},
    )
    calls: list[tuple[str, tuple[int, ...]]] = []

    def fake_eval(
        _ctx, _params, model_type, _source, _steer_source="steer",
        *, horizons=fit_merge.HORIZONS,
    ):
        calls.append((model_type, horizons))
        return {
            horizon: {key: float(horizon) for key in fit_merge._RMSE_KEYS}
            for horizon in horizons
        }

    monkeypatch.setattr(fit_merge, "_eval", fake_eval)
    baselines, tuned = fit_merge._evaluate_report_metrics(
        [context],
        baseline_params={},
        baseline_model_type="baseline-model",
        baseline_acceleration_source="accel",
        tuned_params={},
        tuned_model_type="tuned-model",
        tuned_acceleration_source="kinematic_savgol",
    )

    assert calls == [
        ("baseline-model", fit_merge.REPORT_HORIZONS),
        ("tuned-model", fit_merge.REPORT_HORIZONS),
    ]
    assert tuple(baselines["dataset-a"]) == fit_merge.REPORT_HORIZONS
    assert tuple(tuned["dataset-a"]) == fit_merge.REPORT_HORIZONS


def test_comparison_report_metrics_evaluate_every_declared_model(monkeypatch) -> None:
    context = fit_merge.DatasetCtx(dataset_id="dataset-a", dfs={}, t0_ns=0, base={})
    calls: list[str] = []

    def fake_eval(
        _ctx, _params, model_type, _source, _steer_source="steer",
        *, horizons=fit_merge.HORIZONS,
    ):
        calls.append(model_type)
        return {horizon: {key: float(horizon) for key in fit_merge._RMSE_KEYS} for horizon in horizons}

    monkeypatch.setattr(fit_merge, "_eval", fake_eval)
    result = fit_merge._evaluate_comparison_report_metrics(
        [context],
        [
            ("baseline", {}, "baseline-model", "accel", "steer"),
            ("v1", {}, "v1-model", "accel", "steer"),
            ("v2_rk4", {}, "rk4-model", "accel", "steer"),
            ("tuned", {}, "tuned-model", "accel", "steer_savgol"),
        ],
    )

    assert calls == ["baseline-model", "v1-model", "rk4-model", "tuned-model"]
    assert list(result) == ["baseline", "v1", "v2_rk4", "tuned"]
    assert all(tuple(per_model["dataset-a"]) == fit_merge.REPORT_HORIZONS for per_model in result.values())


def test_robust_search_parallel_evaluates_each_trial_once(monkeypatch) -> None:
    aggregate = {
        "by_h": {
            horizon: {
                "nyaw_mean": 1.0,
                "nyaw_worst": 1.0,
                "nlong_mean": 1.0,
                "nlong_worst": 1.0,
                "nlat_mean": 1.0,
                "nlat_worst": 1.0,
                "nsteer_mean": 1.0,
                "nsteer_worst": 1.0,
                "nax_mean": 1.0,
                "nax_worst": 1.0,
            }
            for horizon in fit_merge.HORIZONS
        }
    }

    def fake_evaluate_candidate(_ctxs, _params, _model_type, _source, _steer_source="steer", **_kwargs):
        return aggregate

    monkeypatch.setattr(fit_merge, "_evaluate_candidate", fake_evaluate_candidate)
    context = fit_merge.DatasetCtx(
        dataset_id="dataset-a",
        dfs={},
        t0_ns=0,
        base={"acc_time_delay": 0.1},
    )
    case = SimpleNamespace(
        params={"acc_time_delay": 0.1},
        vehicle_model_type="delay_steer_acc_geared_for_diffusion_planner",
        acceleration_source="accel",
    )
    config = SimpleNamespace(find_case=lambda _name: case, fit=SimpleNamespace(target="current"))

    result = fit_merge.robust_search(
        [context], config, n_trials=4, n_jobs=2, direct_fit_params={}
    )

    assert isinstance(result, dict)
    assert fit_merge._SEARCH_CTXS == []


def test_robust_search_uses_direct_fit_for_single_trial(monkeypatch) -> None:
    def fake_evaluate_candidate(_ctxs, params, _model_type, _source, _steer_source="steer", **_kwargs):
        value = float(params["acc_time_constant"])
        return {
            "by_h": {
                horizon: {
                    "nyaw_mean": value,
                    "nyaw_worst": value,
                    "nlong_mean": value,
                    "nlong_worst": value,
                    "nlat_mean": value,
                    "nlat_worst": value,
                    "nsteer_mean": value,
                    "nsteer_worst": value,
                    "nax_mean": value,
                    "nax_worst": value,
                }
                for horizon in fit_merge.HORIZONS
            }
        }

    monkeypatch.setattr(fit_merge, "_evaluate_candidate", fake_evaluate_candidate)
    context = fit_merge.DatasetCtx(
        dataset_id="dataset-a",
        dfs={},
        t0_ns=0,
        base={"acc_time_delay": 0.1},
    )
    case = SimpleNamespace(
        params={"acc_time_constant": 0.3, "acc_time_delay": 0.1},
        vehicle_model_type="delay_steer_acc_geared_for_diffusion_planner",
        acceleration_source="accel",
    )
    config = SimpleNamespace(find_case=lambda _name: case, fit=SimpleNamespace(target="current"))

    params = fit_merge.robust_search(
        [context],
        config,
        n_trials=1,
        n_jobs=1,
        direct_fit_params={"acc_time_constant": 0.1, "acc_time_delay": 0.1},
    )

    assert params["acc_time_constant"] == pytest.approx(0.1)


def test_fit_merge_rejects_incomplete_direct_fit_artifact(tmp_path: Path) -> None:
    phase3 = tmp_path / "phase3.yaml"
    phase3.write_text("params: {}\nmetadata:\n  phase: 3\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing="):
        fit_merge.run(
            tmp_path,
            tmp_path / "scenario.yaml",
            tmp_path / "tuned.yaml",
            phase3_params_path=phase3,
            metrics_out=tmp_path / "metrics.csv",
        )


_VALID_DIRECT_FIT_PARAMS = {
    "acc_time_constant": 0.3,
    "acc_time_delay": 0.1,
    "debug_acc_scaling_factor": 1.0,
    "steer_time_constant": 0.3,
    "steer_time_delay": 0.05,
    "debug_steer_scaling_factor": 1.0,
    "steer_dead_band": 0.0,
    "steer_bias": 0.0,
    "steer_rate_lim": 5.0,
    "k_us": 0.01,
    "wheelbase": 4.7,
    "xy_heading_rate_coeff": 0.0,
}


def test_fit_merge_rejects_wrong_phase_metadata(tmp_path: Path) -> None:
    phase3 = tmp_path / "phase3.yaml"
    phase3.write_text(
        yaml.safe_dump({"params": _VALID_DIRECT_FIT_PARAMS, "metadata": {"phase": 2}}),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="metadata.phase"):
        fit_merge.run(
            tmp_path,
            tmp_path / "scenario.yaml",
            tmp_path / "tuned.yaml",
            phase3_params_path=phase3,
            metrics_out=tmp_path / "metrics.csv",
        )


def test_direct_fit_rejects_physical_range_violation() -> None:
    params = {name: 1.0 for name in fit_merge._DIRECT_FIT_KEYS}
    params.update({"wheelbase": 4.7, "steer_bias": 0.02})

    with pytest.raises(ValueError, match="steer_bias=0.02.*許容域.*操舵オフセット"):
        validate_parameters(params, fit_merge._DIRECT_FIT_KEYS, source="直接同定結果")


def test_search_candidates_are_clamped_to_physical_ranges() -> None:
    candidate = clamp_search_parameters({"acc_time_constant": 99.0, "steer_bias": -9.0})

    assert candidate["acc_time_constant"] == pytest.approx(3.0)
    assert candidate["steer_bias"] == pytest.approx(-0.01)


def test_constraints_have_ranges_and_reasons() -> None:
    assert fit_merge._DIRECT_FIT_KEYS <= PARAMETER_CONSTRAINTS.keys()
    assert all(constraint.reason and constraint.range_text() for constraint in PARAMETER_CONSTRAINTS.values())


def test_search_definition_is_the_single_source_of_allowed_range() -> None:
    constraint = ParameterConstraint("test", "reason", search_candidates=(0.1, 0.4))

    assert constraint.allowed_bounds == (0.1, 0.4)
    with pytest.raises(ValueError, match="二重指定"):
        ParameterConstraint("test", "reason", bounds=(0.0, 1.0), search_bounds=(0.0, 1.0))


def test_constraint_audit_is_yaml_serializable_and_includes_reason() -> None:
    audit = build_constraint_audit({"acc_time_constant": 0.3, "wheelbase": 4.7})

    assert audit["acc_time_constant"]["value"] == pytest.approx(0.3)
    assert audit["wheelbase"]["allowed_range"]["minimum_inclusive"] is False
    assert audit["wheelbase"]["default"] == pytest.approx(4.76012)
    assert audit["acc_time_constant"]["reason"]
    assert audit["acc_time_constant"]["direct_fit_bounds"] == [0.01, 5.0]
    assert audit["acc_time_constant"]["optimization_targets"] == [FIT_LON, FIT_MERGE]
    yaml.safe_dump(audit, allow_unicode=True)


def test_optimization_targets_are_configured_per_stage() -> None:
    assert "acc_time_constant" in stage_targets(FIT_LON)
    assert "steer_time_constant" not in stage_targets(FIT_LON)
    assert "steer_time_delay" in stage_targets(FIT_STEER)
    assert "steer_time_delay" not in stage_targets(FIT_MERGE)
    assert "xy_heading_rate_coeff" in stage_targets(FIT_XY)
    assert "xy_heading_rate_coeff" not in stage_targets(FIT_MERGE)


def test_fit_lon_uses_scenario_value_when_target_is_disabled(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    for key in fit_lon._LONG_KEYS:
        monkeypatch.setitem(
            PARAMETER_CONSTRAINTS,
            key,
            replace(PARAMETER_CONSTRAINTS[key], optimization_stages=frozenset()),
        )

    result = fit_lon.fit_lon(
        tmp_path,
        initial_params={
            "acc_time_constant": 0.3,
            "acc_time_delay": 0.1,
            "debug_acc_scaling_factor": 1.0,
        },
    )

    assert result["params"]["acc_time_constant"] == pytest.approx(0.3)
    assert result["metadata"]["optimized_parameters"] == []


def test_fit_merge_skips_optuna_when_all_targets_are_disabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    for key, constraint in list(PARAMETER_CONSTRAINTS.items()):
        monkeypatch.setitem(
            PARAMETER_CONSTRAINTS,
            key,
            replace(
                constraint,
                optimization_stages=constraint.optimization_stages - {FIT_MERGE},
            ),
        )
    aggregate = {
        "by_h": {
            horizon: {
                "nyaw_mean": 1.0, "nyaw_worst": 1.0,
                "nlong_mean": 1.0, "nlong_worst": 1.0,
                "nlat_mean": 1.0, "nlat_worst": 1.0,
                "nsteer_mean": 1.0, "nsteer_worst": 1.0,
                "nax_mean": 1.0, "nax_worst": 1.0,
            }
            for horizon in fit_merge.HORIZONS
        }
    }
    monkeypatch.setattr(fit_merge, "_evaluate_candidate", lambda *_args, **_kwargs: aggregate)
    context = fit_merge.DatasetCtx("dataset-a", {}, 0, {"acc_time_delay": 0.1})
    case = SimpleNamespace(
        params={"acc_time_constant": 0.3, "acc_time_delay": 0.1},
        vehicle_model_type="delay_steer_acc_geared_for_diffusion_planner",
        acceleration_source="accel",
    )

    result = fit_merge.robust_search(
        [context], SimpleNamespace(find_case=lambda _name: case, fit=SimpleNamespace(target="current")), n_trials=1,
        direct_fit_params={"acc_time_constant": 0.2},
    )

    assert result["acc_time_constant"] == pytest.approx(0.2)


def test_fit_steer_requires_scenario_fallback_when_targets_disabled(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    for key in ("steer_time_constant", "steer_time_delay", "steer_bias",
                "debug_steer_scaling_factor", "k_us"):
        monkeypatch.setitem(
            PARAMETER_CONSTRAINTS,
            key,
            replace(PARAMETER_CONSTRAINTS[key], optimization_stages=frozenset()),
        )

    with pytest.raises(ValueError, match="scenario 初期値がありません"):
        fit_steer.fit_steer(tmp_path, phase1_params={}, wheelbase=4.7)


def test_fit_steer_rejects_incomplete_longitudinal_artifact(tmp_path: Path) -> None:
    phase1 = tmp_path / "phase1.yaml"
    phase1.write_text("params: {}\nmetadata:\n  phase: 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="invalid fit_lon result"):
        fit_steer.run(
            tmp_path,
            tmp_path / "phase2.yaml",
            phase1_params_path=phase1,
            scenario=tmp_path / "scenario.yaml",
        )


def test_rollout_gt_cache_is_bounded_to_active_trial(monkeypatch) -> None:
    context = fit_merge.DatasetCtx(
        dataset_id="dataset-a",
        dfs={},
        t0_ns=0,
        base={
            "acc_time_delay": 0.1,
            "steer_time_delay": 0.1,
            "wheelbase": 4.7,
            "sub_dt": 1 / 30,
        },
    )
    context.data_cache[("accel", "steer")] = {}
    monkeypatch.setattr(
        fit_merge.rollout,
        "_prepare_gt",
        lambda _data, _t0, params: {"delay": params["steer_time_delay"]},
    )
    monkeypatch.setattr(
        fit_merge.rollout,
        "eval_rollout_rmse",
        lambda *_args, horizons, **_kwargs: {
            horizon: {"yaw": 1.0, "long": 1.0, "lat": 1.0} for horizon in horizons
        },
    )

    fit_merge._eval(
        context,
        {"steer_time_delay": 0.2},
        "delay_steer_acc_geared_wo_fall_guard",
    )
    fit_merge._eval(
        context,
        {"steer_time_delay": 0.3},
        "delay_steer_acc_geared_wo_fall_guard",
    )

    assert len(context.gt_cache) == 1
    assert next(iter(context.gt_cache.values()))["delay"] == 0.3


def test_minimal_scenario_and_release_artifact(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(
            {
                "Evaluation": {
                    "Conditions": {
                        "comparison_models": ["baseline", "current"],
                        "release": {"model": "tuned", "version": 100},
                        "models": {
                            "baseline": {
                                "vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                                "params": {"wheelbase": 4.7},
                            },
                            "current": {
                                "vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                                "params": {"wheelbase": 4.7},
                            },
                        },
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    config = load_model_config(scenario)
    assert config.find_case("baseline").params["wheelbase"] == 4.7
    assert config.find_case("current").params["wheelbase"] == 4.7

    source_param = tmp_path / "custom-name.yaml"
    source_param.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
                        "delay_steer_acc_geared_for_diffusion_planner": {
                            "version": 1,
                            "v1": {"k_us": 0.0},
                        }
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    tuned = tmp_path / "tuned.yaml"
    tuned.write_text(
        yaml.safe_dump(
            {
                "params": {
                    "acc_time_delay": 0.1,
                    "acc_time_constant": 0.2,
                    "steer_time_delay": 0.05,
                    "steer_time_constant": 0.3,
                    "steer_dead_band": 0.001,
                    "steer_bias": 0.002,
                    "debug_acc_scaling_factor": 1.01,
                    "debug_steer_scaling_factor": 0.99,
                    "k_us": 0.012,
                    "xy_heading_rate_coeff": 0.0,
                    "use_rk4": False,
                    "vel_lim": 50.0,
                    "vel_rate_lim": 7.0,
                    "steer_lim": 1.0,
                    "steer_rate_lim": 0.6,
                    "wheelbase": 4.7,
                },
                "metadata": {
                    "vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner"
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    released = release_params.release(source_param, tuned, tmp_path / "out", scenario=scenario)

    assert released.name == "simulator_model.param.yaml"
    document = yaml.safe_load(released.read_text(encoding="utf-8"))
    model = document["/**"]["ros__parameters"][
        "delay_steer_acc_geared_for_diffusion_planner"
    ]
    assert model["version"] == 100
    assert model["v100"]["k_us"] == 0.012
    assert "steer_rate_lim" not in model["v100"]
    assert document["/**"]["ros__parameters"]["steer_rate_lim"] == 0.6
    assert document["/**"]["ros__parameters"]["wheel_base"] == 4.7


def _release_scenario_document(release: dict | None) -> dict:
    """release 指定つき最小 scenario を組み立てる (v2 = 固定リリース候補ケース)。"""
    conditions = {
        "comparison_models": ["baseline", "v2", "current"],
        "models": {
            "baseline": {
                "vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                "params": {"wheelbase": 4.7},
            },
            "v2": {
                "vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                "params": {
                    "wheelbase": 4.7,
                    "acc_time_constant": 0.3,
                    "debug_acc_scaling_factor": 0.9,
                    "debug_steer_scaling_factor": 1.015827,
                    "steer_rate_lim": 0.6,
                    "k_us": 0.018,
                },
            },
            "current": {
                "vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                "params": {"wheelbase": 4.7},
            },
        },
    }
    if release is not None:
        conditions["release"] = release
    return {"Evaluation": {"Conditions": conditions}}


def test_scenario_parses_release_spec(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document({"model": "v2", "version": 2})),
        encoding="utf-8",
    )
    config = load_model_config(scenario)
    assert config.release is not None
    assert config.release.model == "v2"
    assert config.release.version == 2

    scenario.write_text(
        yaml.safe_dump(_release_scenario_document(None)), encoding="utf-8",
    )
    assert load_model_config(scenario).release is None


@pytest.mark.parametrize(
    ("release", "match"),
    [
        ({"model": "unknown", "version": 2}, "release.model"),
        ({"model": "baseline", "version": 2}, "vehicle_model_type"),
        ({"model": "v2", "version": 0}, "release.version"),
        ({"model": "v2", "version": True}, "release.version"),
    ],
)
def test_scenario_rejects_invalid_release_spec(tmp_path: Path, release, match) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document(release)), encoding="utf-8",
    )
    with pytest.raises(ValueError, match=match):
        load_model_config(scenario)


def test_scenario_accepts_tuned_release_when_fit_enabled(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document({"model": "tuned", "version": 3})),
        encoding="utf-8",
    )
    config = load_model_config(scenario)
    assert config.release is not None
    assert config.release.model == "tuned"


def test_scenario_rejects_tuned_release_when_fit_disabled(tmp_path: Path) -> None:
    document = _release_scenario_document({"model": "tuned", "version": 3})
    document["Evaluation"]["Conditions"]["fit"] = {"stages": []}
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="tuned.* は fit が有効"):
        load_model_config(scenario)


def test_release_writes_designated_case_into_version_slot(tmp_path: Path) -> None:
    """release 指定時は指定ケースが v{N} スロットに入り、tuned の v100 は書かれない。"""
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document({"model": "v2", "version": 2})),
        encoding="utf-8",
    )
    source_param = tmp_path / "input.yaml"
    source_param.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
                        "delay_steer_acc_geared_for_diffusion_planner": {
                            "version": 1,
                            "v1": {"k_us": 0.0},
                        },
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    released = release_params.release(
        source_param, tmp_path / "missing-tuned.yaml", tmp_path / "out",
        scenario=scenario,
    )

    document = yaml.safe_load(released.read_text(encoding="utf-8"))
    ros_params = document["/**"]["ros__parameters"]
    model = ros_params["delay_steer_acc_geared_for_diffusion_planner"]
    assert model["version"] == 2
    assert model["v2"]["debug_acc_scaling_factor"] == 0.9
    assert model["v2"]["debug_steer_scaling_factor"] == 1.015827
    assert model["v2"]["k_us"] == 0.018
    # scenario ケースにないキーは rollout 既定 (ローカル config の選択バージョン) から補完
    assert model["v2"]["use_rk4"] is False
    assert "v100" not in model
    # 既存の確定バージョン v1 は保持され、global 値は指定ケース由来
    assert model["v1"] == {"k_us": 0.0}
    assert ros_params["steer_rate_lim"] == 0.6
    assert ros_params["wheel_base"] == 4.7


def test_release_rejects_overwriting_existing_version_slot(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document({"model": "v2", "version": 1})),
        encoding="utf-8",
    )
    source_param = tmp_path / "input.yaml"
    source_param.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
                        "delay_steer_acc_geared_for_diffusion_planner": {
                            "version": 1,
                            "v1": {"k_us": 0.0},
                        },
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="already contains 'v1'"):
        release_params.release(
            source_param, tmp_path / "missing-tuned.yaml", tmp_path / "out",
            scenario=scenario,
        )


def test_release_is_idempotent_for_identical_version_slot(tmp_path: Path) -> None:
    """リリース適用済み入力での再実行は、同一内容の v{N} を冪等に許可する。"""
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document({"model": "v2", "version": 2})),
        encoding="utf-8",
    )
    source_param = tmp_path / "input.yaml"
    source_param.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
                        "delay_steer_acc_geared_for_diffusion_planner": {
                            "version": 1,
                            "v1": {"k_us": 0.0},
                        },
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    first = release_params.release(
        source_param, tmp_path / "missing-tuned.yaml", tmp_path / "out1",
        scenario=scenario,
    )
    # 1 回目の出力 (v2 適用済み) を入力にした再実行は同一内容なので成功する。
    second = release_params.release(
        first, tmp_path / "missing-tuned.yaml", tmp_path / "out2",
        scenario=scenario,
    )

    assert yaml.safe_load(second.read_text(encoding="utf-8")) == yaml.safe_load(
        first.read_text(encoding="utf-8")
    )


def _tuned_document() -> dict:
    return {
        "params": {
            "acc_time_delay": 0.1, "acc_time_constant": 0.2,
            "steer_time_delay": 0.05, "steer_time_constant": 0.3,
            "steer_dead_band": 0.0, "steer_bias": 0.0,
            "debug_acc_scaling_factor": 1.0, "debug_steer_scaling_factor": 1.0,
            "k_us": 0.012, "xy_heading_rate_coeff": 0.0, "use_rk4": False,
            "vel_lim": 50.0, "vel_rate_lim": 7.0, "steer_lim": 1.0,
            "steer_rate_lim": 0.6, "wheelbase": 4.7,
        },
        "metadata": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner"},
    }


def test_release_tuned_overwrites_existing_slot_without_idempotency_guard(tmp_path: Path) -> None:
    """model: tuned は Optuna 非再現性のため、既存スロットを別内容でも上書き許可する。"""
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(_release_scenario_document({"model": "tuned", "version": 2})),
        encoding="utf-8",
    )
    source_param = tmp_path / "input.yaml"
    source_param.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
                        "delay_steer_acc_geared_for_diffusion_planner": {
                            "version": 2,
                            "v2": {"k_us": 0.999},
                        },
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    tuned = tmp_path / "tuned.yaml"
    tuned.write_text(yaml.safe_dump(_tuned_document()), encoding="utf-8")

    released = release_params.release(source_param, tuned, tmp_path / "out", scenario=scenario)
    model = yaml.safe_load(released.read_text(encoding="utf-8"))["/**"]["ros__parameters"][
        "delay_steer_acc_geared_for_diffusion_planner"
    ]
    assert model["version"] == 2
    assert model["v2"]["k_us"] == 0.012  # 既存の 0.999 を上書き


def test_release_requires_a_release_spec(tmp_path: Path) -> None:
    """release 指定のない scenario / scenario=None は release でエラーにする。"""
    source_param = tmp_path / "input.yaml"
    source_param.write_text("x: 1\n", encoding="utf-8")
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(yaml.safe_dump(_release_scenario_document(None)), encoding="utf-8")

    with pytest.raises(ValueError, match="release がありません"):
        release_params.release(source_param, tmp_path / "t.yaml", tmp_path / "out", scenario=scenario)
    with pytest.raises(ValueError, match="release 指定が必要"):
        release_params.release(source_param, tmp_path / "t.yaml", tmp_path / "out")


def test_scenario_rejects_non_target_current_model(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(
            {
                "Evaluation": {
                    "Conditions": {
                        "comparison_models": ["baseline", "current"],
                        "models": {
                            "baseline": {
                                "vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard"
                            },
                            "current": {
                                "vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard"
                            },
                        }
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="fit.target='current' の vehicle_model_type"):
        load_model_config(scenario)


@pytest.mark.parametrize("wheelbase", [True, float("inf")])
def test_scenario_rejects_non_finite_or_boolean_wheelbase(
    tmp_path: Path, wheelbase: object
) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        yaml.safe_dump(
            {
                "Evaluation": {
                    "Conditions": {
                        "comparison_models": ["baseline", "current"],
                        "models": {
                            "baseline": {
                                "vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                                "params": {"wheelbase": wheelbase},
                            },
                            "current": {
                                "vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                                "params": {"wheelbase": 4.7},
                            },
                        }
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="params.wheelbase"):
        load_model_config(scenario)


def test_scenario_rejects_non_mapping_document(tmp_path: Path) -> None:
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text("- not\n- a\n- mapping\n", encoding="utf-8")

    with pytest.raises(ValueError, match="top-level"):
        load_model_config(scenario)


def _synthetic_metric() -> dict:
    return {h: {k: 1.0 + 0.001 * h for k in fit_merge._RMSE_KEYS} for h in fit_merge.HORIZONS}


def _install_synthetic_evaluation(monkeypatch) -> None:
    def fake_eval(_ctx, _params, _model_type, _accel="accel", _steer="steer",
                  *, horizons=fit_merge.HORIZONS, include_mean=False):
        return {h: {k: 1.0 + 0.001 * h for k in fit_merge._RMSE_KEYS} for h in horizons}

    monkeypatch.setattr(fit_merge, "_eval", fake_eval)


def _synthetic_ctxs() -> list:
    ctxs = []
    for ds_id in ("ds-a", "ds-b"):
        ctx = fit_merge.DatasetCtx(dataset_id=ds_id, dfs={}, t0_ns=0, base={})
        ctx.base_metric = _synthetic_metric()
        ctxs.append(ctx)
    return ctxs


def test_evaluate_only_without_fit_roundtrips_to_report(tmp_path: Path, monkeypatch) -> None:
    """fit 無効 (stages: []) の評価専用パスが metrics.csv + document を生成し report が描画できる。"""
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify import report

    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(yaml.safe_dump({"Evaluation": {"Conditions": {
        "comparison_models": ["baseline", "current"],
        "fit": {"stages": []},
        "models": {
            "baseline": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                          "params": {"wheelbase": 4.7}},
            "current": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                         "params": {"wheelbase": 4.7}},
        },
    }}}), encoding="utf-8")

    _install_synthetic_evaluation(monkeypatch)
    baseline_bundle = ("delay_steer_acc_geared_wo_fall_guard", {"wheelbase": 4.7}, "accel", "steer")
    monkeypatch.setattr(
        fit_merge, "_load_ctxs",
        lambda _c, _cfg, _n: (2, _synthetic_ctxs(), [], baseline_bundle),
    )

    out = tmp_path / "reidentify" / "tuned_params.yaml"
    metrics = tmp_path / "reidentify" / "metrics.csv"
    result = fit_merge.run_evaluate(
        tmp_path, scenario, out, direct_fit_params_path=None, metrics_out=metrics, n_jobs=1,
    )

    assert result["params"] == {}  # fit なしなので tuned なし
    assert set(result["comparison"]) == {"baseline", "current"}
    assert out.is_file() and metrics.is_file()

    monkeypatch.setattr(
        report.physical_validity, "build_sections",
        lambda *_a, **_k: report.physical_validity.PhysicalValiditySections(
            equations="", prepare="", longitudinal="", steering="", yaw="", xy="", timeseries="",
        ),
    )
    report_out = report.run(
        out, metrics, tmp_path / "report.html", failures={},
        collection_dir=tmp_path, scenario=scenario,
    )
    rendered = report_out.read_text(encoding="utf-8")
    assert "8. Released YAML" in rendered
    assert "リリース YAML は生成されていません" in rendered
    assert "No parameter summary available" in rendered


def test_build_comparison_document_includes_tuned_when_fit_ran(tmp_path: Path, monkeypatch) -> None:
    """fit 実行時は tuned が comparison/metrics に含まれ document.params に反映される。"""
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(yaml.safe_dump({"Evaluation": {"Conditions": {
        "comparison_models": ["baseline", "current"],
        "models": {
            "baseline": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                          "params": {"wheelbase": 4.7}},
            "current": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                         "params": {"wheelbase": 4.7}},
        },
    }}}), encoding="utf-8")
    cfg = load_model_config(scenario)
    _install_synthetic_evaluation(monkeypatch)

    metrics = tmp_path / "metrics.csv"
    tuned_params = {"wheelbase": 4.7, "k_us": 0.012}
    result = fit_merge.build_comparison_document(
        tmp_path, scenario, cfg, _synthetic_ctxs(),
        n_datasets=2, fit_skipped=[],
        baseline_bundle=("delay_steer_acc_geared_wo_fall_guard", {"wheelbase": 4.7}, "accel", "steer"),
        tuned_params=tuned_params, metrics_out=metrics,
    )

    assert result["params"] == tuned_params
    assert "tuned" in result["comparison"]
    assert "current" not in result["comparison"]  # fit 対象は tuned として表示
    import pandas as pd  # noqa: PLC0415

    models = set(pd.read_csv(metrics)["model"].unique())
    assert models == {"baseline", "tuned"}


def test_build_comparison_document_omits_tuned_when_target_not_in_comparison(
    tmp_path: Path, monkeypatch,
) -> None:
    """fit 対象を comparison_models に入れなければ tuned は比較に出ない (params は保持)。"""
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(yaml.safe_dump({"Evaluation": {"Conditions": {
        "comparison_models": ["baseline", "v1"],  # 対象 current を含めない
        "models": {
            "baseline": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard",
                          "params": {"wheelbase": 4.7}},
            "v1": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                    "params": {"wheelbase": 4.7}},
            "current": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner",
                         "params": {"wheelbase": 4.7}},
        },
    }}}), encoding="utf-8")
    cfg = load_model_config(scenario)
    _install_synthetic_evaluation(monkeypatch)

    metrics = tmp_path / "metrics.csv"
    tuned_params = {"wheelbase": 4.7, "k_us": 0.012}
    result = fit_merge.build_comparison_document(
        tmp_path, scenario, cfg, _synthetic_ctxs(),
        n_datasets=2, fit_skipped=[],
        baseline_bundle=("delay_steer_acc_geared_wo_fall_guard", {"wheelbase": 4.7}, "accel", "steer"),
        tuned_params=tuned_params, metrics_out=metrics,
    )

    assert "tuned" not in result["comparison"]
    assert set(result["comparison"]) == {"baseline", "v1"}
    assert result["params"] == tuned_params  # fit 結果は Final params として保持
    import pandas as pd  # noqa: PLC0415

    assert set(pd.read_csv(metrics)["model"].unique()) == {"baseline", "v1"}
