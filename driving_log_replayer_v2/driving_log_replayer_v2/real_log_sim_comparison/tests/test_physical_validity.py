from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from driving_log_replayer_v2.real_log_sim_comparison import physical_validity


def _dataset(samples: int = 300) -> dict:
    time = np.arange(samples) * 0.01
    return {
        "a_cmd": np.where(time > 0.5, 1.0, 0.0),
        "a_act": np.where(time > 0.6, 0.8, 0.0),
        "d_cmd": np.where(time > 0.8, 0.1, 0.0),
        "d_act": np.where(time > 0.9, 0.08, 0.0),
        "vx": np.full(samples, 5.0),
        "wz": np.full(samples, 0.01),
        "gear_drive": np.ones(samples, dtype=bool),
        "xy": (5 * time, np.zeros(samples), np.zeros(samples), np.full(samples, 5.0), np.zeros(samples)),
    }


def _params(path: Path) -> Path:
    path.write_text(yaml.safe_dump({"params": {"wheelbase": 4.7}}), encoding="utf-8")
    return path


def _scenario(path: Path, comparison_models: list[str]) -> Path:
    common = {
        "wheelbase": 4.7,
        "acc_time_delay": 0.1,
        "acc_time_constant": 0.2,
        "steer_time_delay": 0.1,
        "steer_time_constant": 0.2,
        "steer_bias": 0.0,
        "k_us": 0.01,
        "xy_heading_rate_coeff": 0.0,
    }
    models = {
        "baseline": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard", "params": common},
        "v1": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard", "params": common},
        "v1_rk4": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard", "params": common},
        "current": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner", "params": common},
    }
    path.write_text(yaml.safe_dump({"Evaluation": {"Conditions": {"comparison_models": comparison_models, "models": models}}}, sort_keys=False), encoding="utf-8")
    return path


def test_steps_keep_results_and_sections_separate(tmp_path: Path, monkeypatch) -> None:
    csv = tmp_path / "cache.csv"
    monkeypatch.setattr(physical_validity, "discover_cached_datasets", lambda _root: [("good", csv), ("bad", csv)])
    calls = {"count": 0}

    class Kinematic:
        empty = False
        def __getitem__(self, name):
            class Column:
                iloc = [1_000_000_000]
                def to_numpy(self, dtype=float):
                    return np.array([1_000_000_000, 4_000_000_000], dtype=dtype)
            return Column()

    def resample(_source, _dt, *, context):
        calls["count"] += 1
        if context.endswith("bad"):
            raise ValueError("invalid cache")
        return _dataset()

    monkeypatch.setattr(physical_validity, "read_dataset_csv", lambda _path: {topic: Kinematic() for topic in ("cmd", "accel", "steering", "velocity", "kinematic")})
    monkeypatch.setattr(physical_validity, "build_resampled", resample)
    context, prepared = physical_validity.prepare_datasets(tmp_path, _params(tmp_path / "params.yaml"))

    assert calls["count"] == 2
    assert prepared.result["n_valid"] == 1
    assert "bad: invalid cache" in prepared.html
    steps = [physical_validity.validate_longitudinal(context), physical_validity.validate_steering(context), physical_validity.validate_yaw(context), physical_validity.validate_xy(context)]
    assert [step.name for step in steps] == ["longitudinal", "steering", "yaw", "xy"]
    assert all(isinstance(step.result, dict) and step.html.startswith("<section") for step in steps)
    assert all("good" in step.result["datasets"] for step in steps)


def test_build_sections_returns_ordered_report_fragments(tmp_path: Path, monkeypatch) -> None:
    csv = tmp_path / "cache.csv"
    monkeypatch.setattr(
        physical_validity,
        "discover_cached_datasets",
        lambda _root: [("good-a", csv), ("good-b", csv)],
    )

    class Kinematic:
        empty = False
        def __getitem__(self, _name):
            class Column:
                iloc = [1_000_000_000]
                def to_numpy(self, dtype=float):
                    return np.array([1_000_000_000, 4_000_000_000], dtype=dtype)
            return Column()

    monkeypatch.setattr(physical_validity, "read_dataset_csv", lambda _path: {topic: Kinematic() for topic in ("cmd", "accel", "steering", "velocity", "kinematic")})
    monkeypatch.setattr(physical_validity, "build_resampled", lambda *_args, **_kwargs: _dataset())
    rendered = physical_validity.build_sections(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        scenario=_scenario(tmp_path / "scenario.yaml", ["baseline", "current"]),
    )

    assert 'id="prepare"' in rendered.prepare
    assert "有効データセット: 2" in rendered.prepare
    assert 'id="longitudinal"' in rendered.longitudinal
    assert "<table" in rendered.longitudinal
    assert 'id="steering"' in rendered.steering
    assert 'id="yaw"' in rendered.yaw
    assert 'id="xy"' in rendered.xy
    combined = rendered.prepare + rendered.longitudinal + rendered.steering + rendered.yaw + rendered.xy
    assert "good-a" not in combined
    assert "good-b" not in combined


def test_no_valid_dataset_still_produces_all_sections(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(physical_validity, "discover_cached_datasets", lambda _root: [("bad", tmp_path / "cache.csv")])
    monkeypatch.setattr(physical_validity, "read_dataset_csv", lambda _path: (_ for _ in ()).throw(ValueError("bad input")))
    rendered = physical_validity.build_sections(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        scenario=_scenario(tmp_path / "scenario.yaml", ["baseline", "current"]),
    )
    assert "bad: bad input" in rendered.prepare
    assert 'id="prepare"' in rendered.prepare
    assert 'id="longitudinal"' in rendered.longitudinal
    assert 'id="steering"' in rendered.steering
    assert 'id="yaw"' in rendered.yaw
    assert 'id="xy"' in rendered.xy


def test_comparison_models_validate_duplicates_unknown_and_required_cases(tmp_path: Path) -> None:
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.model_config import load_model_config

    for models, message in [
        (["baseline", "baseline", "current"], "重複"),
        (["baseline", "missing", "current"], "未定義"),
        (["baseline"], "必須モデル"),
    ]:
        with np.testing.assert_raises_regex(ValueError, message):
            load_model_config(_scenario(tmp_path / f"{message}.yaml", models))


def test_report_omits_detailed_fit_diagnostics_and_keeps_fixed_evaluations(tmp_path: Path, monkeypatch) -> None:
    csv = tmp_path / "cache.csv"
    monkeypatch.setattr(physical_validity, "discover_cached_datasets", lambda _root: [("good", csv)])

    class Kinematic:
        empty = False
        def __getitem__(self, _name):
            class Column:
                iloc = [1_000_000_000]
                def to_numpy(self, dtype=float):
                    return np.array([1_000_000_000, 4_000_000_000], dtype=dtype)
            return Column()

    monkeypatch.setattr(physical_validity, "read_dataset_csv", lambda _path: {topic: Kinematic() for topic in ("cmd", "accel", "steering", "velocity", "kinematic")})
    monkeypatch.setattr(physical_validity, "build_resampled", lambda *_args, **_kwargs: _dataset())
    params = tmp_path / "tuned_params.yaml"
    params.write_text(yaml.safe_dump({"params": {"acc_time_constant": 0.9, "wheelbase": 4.8}}), encoding="utf-8")
    rendered = physical_validity.build_sections(
        tmp_path,
        params,
        scenario=_scenario(tmp_path / "scenario.yaml", ["baseline", "v1", "v1_rk4", "current"]),
    )

    combined = rendered.prepare + rendered.longitudinal + rendered.steering + rendered.yaw + rendered.xy
    assert 'current</th><td class="rmse-cell"' in combined
    assert "acc_time_constant=0.9" in rendered.longitudinal
    assert 'baseline</th><td class="rmse-cell"' in combined
    # 数式ハブと各セクションの参照行が生成されること
    assert 'id="eq-notation"' in rendered.equations
    assert 'id="eq-yaw"' in rendered.equations
    assert 'href="#eq-long"' in rendered.longitudinal
    assert 'class="rmse-fill' in combined
    assert "acc_time_constant=0.2" in rendered.longitudinal
    assert "データ駆動フィット診断" not in combined
    assert "代表時系列" not in combined
