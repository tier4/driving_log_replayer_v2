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


def _scenario(
    path: Path,
    comparison_models: list[str],
    *,
    plot_dataset: object | None = None,
    release: dict | None = None,
    fit: dict | None = None,
) -> Path:
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
        "v2_rk4": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard", "params": common},
        "current": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner", "params": common},
    }
    conditions: dict = {"comparison_models": comparison_models, "models": models}
    if plot_dataset is not None:
        conditions["plot_dataset"] = plot_dataset
    if release is not None:
        conditions["release"] = release
    if fit is not None:
        conditions["fit"] = fit
    path.write_text(yaml.safe_dump({"Evaluation": {"Conditions": conditions}}, sort_keys=False), encoding="utf-8")
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
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.model_config import (
        load_model_config,
    )

    for models, message in [
        (["baseline", "baseline", "current"], "重複"),
        (["baseline", "missing", "current"], "未定義"),
        # baseline のみ必須。baseline を欠く comparison_models はエラー。
        (["current"], "必須モデル"),
    ]:
        with np.testing.assert_raises_regex(ValueError, message):
            load_model_config(_scenario(tmp_path / f"{message}.yaml", models))


def _timeseries_source(samples: int = 400, dt: float = 1.0 / 30.0) -> dict:
    """C++ 駆動 8 章用の合成 7-topic ソース (rollout._prepare_gt が読める形)。"""
    import pandas as pd

    t_ns = (np.arange(samples) * dt * 1e9).astype(np.int64) + 1_000_000_000
    time = np.arange(samples) * dt

    def frame(**cols):
        return pd.DataFrame({"t_ns": t_ns, **cols})

    return {
        "mode": frame(mode=np.zeros(samples, dtype=int)),
        "velocity": frame(lon_vel=np.full(samples, 5.0)),
        "steering": frame(steer=np.full(samples, 0.01)),
        "gear": frame(gear=np.full(samples, 2, dtype=int)),
        "kinematic": frame(
            x=5.0 * time, y=np.zeros(samples), yaw=np.zeros(samples),
            pitch=np.zeros(samples), vx=np.full(samples, 5.0),
            vy=np.zeros(samples), wz=np.zeros(samples),
        ),
        "accel": frame(accel=np.zeros(samples)),
        "cmd": frame(
            cmd_vel=np.full(samples, 5.0), cmd_accel=0.5 * np.sin(0.5 * time),
            cmd_steer=np.full(samples, 0.01),
        ),
    }


_TS_COMMON_PARAMS = {
    "wheelbase": 4.7,
    "acc_time_delay": 0.1,
    "acc_time_constant": 0.2,
    "steer_time_delay": 0.1,
    "steer_time_constant": 0.2,
    "steer_bias": 0.0,
    "k_us": 0.01,
    "xy_heading_rate_coeff": 0.0,
}


def _timeseries_compared_model() -> physical_validity.ComparedModel:
    return physical_validity.ComparedModel(
        "current", "delay_steer_acc_geared_for_diffusion_planner", "accel",
        dict(_TS_COMMON_PARAMS), "steer", "none",
    )


class _TimeseriesFakeLib:
    """vm_integrate_to_horizons の全 horizon へ定常値を書くフェイク。"""

    @staticmethod
    def vm_integrate_to_horizons(*args):
        n_valid = int(args[8].value)
        for out, value in zip(args[-6:], (1.0, 0.0, 0.0, 5.0, 0.0, 0.01)):
            for i in range(n_valid):
                out[i] = value


class _TimeseriesFakeModel:
    def __init__(self, *_args, **_kwargs):
        self._lib = _TimeseriesFakeLib()
        self._ptr = None

    def reset_with_history_ptr(self, **_kwargs):
        pass


def _patch_fake_vehicle_model(monkeypatch) -> None:
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify import rollout

    monkeypatch.setattr(rollout, "VehicleModel", _TimeseriesFakeModel)


def test_timeseries_figures_use_cpp_model_and_render_six_rows(monkeypatch) -> None:
    """モデル側系列は C++ モデル (ここではフェイク) で生成し、6 図を返す。"""
    _patch_fake_vehicle_model(monkeypatch)
    figures, n, reason = physical_validity._timeseries_figures(
        _timeseries_source(), _timeseries_compared_model()
    )

    assert reason is None
    assert n > 0
    titles = [title for title, _figure in figures]
    assert len(figures) == 6
    assert any("C++ 1-step" in title for title in titles)
    assert any("ヨーレート" in title for title in titles)
    assert [len(figure.data) for _title, figure in figures] == [2, 3, 2, 2, 3, 2]


def test_timeseries_figures_report_short_series(monkeypatch) -> None:
    _patch_fake_vehicle_model(monkeypatch)
    figures, _n, reason = physical_validity._timeseries_figures(
        _timeseries_source(samples=8), _timeseries_compared_model()
    )
    assert figures is None
    assert "短すぎ" in (reason or "")


def test_timeseries_figures_decimate_long_traces(monkeypatch) -> None:
    _patch_fake_vehicle_model(monkeypatch)
    samples = physical_validity._TIMESERIES_MAX_POINTS + 2000
    figures, _n, reason = physical_validity._timeseries_figures(
        _timeseries_source(samples=samples), _timeseries_compared_model()
    )
    assert reason is None
    for _title, figure in figures:
        for trace in figure.data:
            assert len(trace.x) <= physical_validity._TIMESERIES_MAX_POINTS


def test_plot_dataset_key_is_parsed_and_validated(tmp_path: Path) -> None:
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.model_config import (
        load_model_config,
    )

    config = load_model_config(_scenario(tmp_path / "s1.yaml", ["baseline", "current"]))
    assert config.plot_datasets == ()
    config = load_model_config(
        _scenario(tmp_path / "s2.yaml", ["baseline", "current"], plot_dataset=["dataset_1.2-a"])
    )
    assert config.plot_datasets == ("dataset_1.2-a",)
    config = load_model_config(
        _scenario(tmp_path / "s3.yaml", ["baseline", "current"], plot_dataset=["ds-a", "ds-b"])
    )
    assert config.plot_datasets == ("ds-a", "ds-b")
    for name, invalid in (
        ("bare-string", "ds-a"),
        ("mapping", {"a": 1}),
        ("empty-entry", [""]),
        ("traversal", ["../x"]),
        ("non-str-entry", [{"a": 1}]),
        ("duplicates", ["ds-a", "ds-a"]),
    ):
        with np.testing.assert_raises_regex(ValueError, "plot_dataset"):
            load_model_config(
                _scenario(tmp_path / f"bad-{name}.yaml", ["baseline", "current"], plot_dataset=invalid)
            )


def test_build_timeseries_section_without_plot_dataset_shows_guidance(tmp_path: Path) -> None:
    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(tmp_path / "scenario.yaml", ["baseline", "current"]),
    )
    assert "plot_dataset" in rendered
    assert 'class="note"' in rendered
    assert "plotly-graph-div" not in rendered


def test_build_timeseries_section_reports_missing_dataset(tmp_path: Path) -> None:
    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(tmp_path / "scenario.yaml", ["baseline", "current"], plot_dataset=["missing"]),
    )
    assert "missing" in rendered
    assert "見つかりません" in rendered


def test_build_timeseries_section_renders_six_figures(tmp_path: Path, monkeypatch) -> None:
    dataset_dir = tmp_path / "datasets" / "good"
    dataset_dir.mkdir(parents=True)
    (dataset_dir / "reidentify_cache.csv").write_text("stub", encoding="utf-8")
    _patch_fake_vehicle_model(monkeypatch)
    monkeypatch.setattr(
        physical_validity, "read_dataset_csv", lambda _path: _timeseries_source()
    )

    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(
            tmp_path / "scenario.yaml", ["baseline", "current"],
            plot_dataset=["good"], release={"model": "current", "version": 2},
        ),
    )

    assert rendered.count("<h3>") == 6
    assert rendered.count("plotly-graph-div") >= 6
    assert rendered.count('class="ts-dataset"') == 1
    assert "release ケース current" in rendered
    assert 'class="stats"' in rendered
    assert "C++" in rendered
    assert "good" in rendered


def test_build_timeseries_section_refuses_without_release_or_fit(tmp_path: Path) -> None:
    """release も fit も無い場合は代替モデルを描かず、明示的に描画拒否する。"""
    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(
            tmp_path / "scenario.yaml", ["baseline", "current"],
            plot_dataset=["good"],
            fit={"target": "current", "stages": []},
        ),
    )
    assert "描画されません" in rendered
    assert "代替モデル" in rendered
    assert "plotly-graph-div" not in rendered
    assert "baseline" not in rendered.split("描画されません")[1][:200]


def test_timeseries_model_resolves_tuned_release(tmp_path: Path) -> None:
    """release.model=='tuned' は fit 対象ケースへ fit 結果を重ねて解決する。"""
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.model_config import (
        load_model_config,
    )

    config = load_model_config(
        _scenario(
            tmp_path / "scenario.yaml", ["baseline", "current"],
            release={"model": "tuned", "version": 2},
        )
    )
    model, label = physical_validity._timeseries_model(config, {"k_us": 0.99})

    assert model.name == "current"  # fit.target の既定
    assert model.params["k_us"] == 0.99  # fit 結果が重ねられる
    assert "tuned" in label


def test_build_timeseries_section_renders_multiple_datasets(tmp_path: Path, monkeypatch) -> None:
    """リスト指定では各データセットを順に描画し、1 件の失敗が他を止めない。"""
    dataset_dir = tmp_path / "datasets" / "good"
    dataset_dir.mkdir(parents=True)
    (dataset_dir / "reidentify_cache.csv").write_text("stub", encoding="utf-8")
    _patch_fake_vehicle_model(monkeypatch)
    monkeypatch.setattr(
        physical_validity, "read_dataset_csv", lambda _path: _timeseries_source()
    )

    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(
            tmp_path / "scenario.yaml", ["baseline", "current"],
            plot_dataset=["good", "missing"],
        ),
    )

    assert rendered.count('class="ts-dataset"') == 2
    assert rendered.count("plotly-graph-div") == 6
    assert "見つかりません" in rendered
    assert rendered.index("dataset: good") < rendered.index("dataset: missing")
    # パラメータタイルはセクション先頭に 1 回だけ
    assert rendered.count('class="stats"') == 1


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
        scenario=_scenario(tmp_path / "scenario.yaml", ["baseline", "v1", "v2_rk4", "current"]),
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
