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
        "v1_rk4": {"vehicle_model_type": "delay_steer_acc_geared_wo_fall_guard", "params": common},
        "current": {"vehicle_model_type": "delay_steer_acc_geared_for_diffusion_planner", "params": common},
    }
    conditions: dict = {"comparison_models": comparison_models, "models": models}
    if plot_dataset is not None:
        conditions["plot_dataset"] = plot_dataset
    if release is not None:
        conditions["release"] = release
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
        (["baseline"], "必須モデル"),
    ]:
        with np.testing.assert_raises_regex(ValueError, message):
            load_model_config(_scenario(tmp_path / f"{message}.yaml", models))


def _timeseries_dataset(samples: int = 2000, dt: float = 0.01) -> tuple[dict, dict]:
    """滑らかな指令に対する厳密な一次遅れ応答から合成データセットを作る。"""
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.fit_core import (
        simulate_first_order,
    )

    params = {
        "acc_time_constant": 0.2,
        "acc_time_delay": 0.1,
        "steer_time_constant": 0.25,
        "steer_time_delay": 0.05,
        "steer_bias": 0.01,
    }
    time = np.arange(samples) * dt
    a_cmd = 0.5 * np.sin(0.5 * time)
    a_act = simulate_first_order(a_cmd, params["acc_time_constant"], params["acc_time_delay"], dt)
    d_cmd = 0.1 * np.sin(0.3 * time)
    d_act = simulate_first_order(d_cmd, params["steer_time_constant"], params["steer_time_delay"], dt) + params["steer_bias"]
    # 明示 Euler の時刻規約: 時刻 t[k] の速度は前サンプルまでの加速度積分。
    vx = np.empty(samples)
    vx[0] = 5.0
    vx[1:] = 5.0 + np.cumsum(a_act[:-1]) * dt
    data = {
        "a_cmd": a_cmd,
        "a_act": a_act,
        "d_cmd": d_cmd,
        "d_act": d_act,
        "v_cmd": vx + 0.5,
        "vx": vx,
        "wz": np.zeros(samples),
        "gear_drive": np.ones(samples, dtype=bool),
    }
    return data, params


def test_timeseries_figures_match_state_equation_rows() -> None:
    dt = 0.01
    data, params = _timeseries_dataset(dt=dt)
    figures, reason = physical_validity._timeseries_figures(data, params, dt)

    assert reason is None
    assert [len(figure.data) for _title, figure in figures] == [2, 3, 3, 2, 3]
    titles = [title for title, _figure in figures]
    assert any("ジャーク" in title for title in titles)
    assert any("ステアレート" in title for title in titles)

    # 状態量グラフ: 右辺の積分 (一次遅れ予測/加速度積分) が合成 GT を再現する
    acc_figure = figures[1][1]
    np.testing.assert_allclose(np.asarray(acc_figure.data[1].y), np.round(data["a_act"], 5), atol=1e-5)
    vel_figure = figures[2][1]
    np.testing.assert_allclose(np.asarray(vel_figure.data[1].y), np.round(data["vx"], 5), atol=1e-5)
    steer_figure = figures[4][1]
    np.testing.assert_allclose(np.asarray(steer_figure.data[1].y), np.round(data["d_act"], 5), atol=1e-5)

    # 微分量グラフ: 滑らかな入力では左辺 (savgol 微分) と右辺 (状態方程式) が一致する
    interior = slice(100, -100)
    for index in (0, 3):
        figure = figures[index][1]
        lhs = np.asarray(figure.data[0].y, dtype=float)[interior]
        rhs = np.asarray(figure.data[1].y, dtype=float)[interior]
        scale = max(float(np.max(np.abs(rhs))), 1e-9)
        assert float(np.sqrt(np.mean((lhs - rhs) ** 2))) < 0.05 * scale


def test_timeseries_figures_report_invalid_params() -> None:
    data, params = _timeseries_dataset(samples=500)
    del params["steer_bias"]
    figures, reason = physical_validity._timeseries_figures(data, params, 0.01)
    assert figures is None
    assert "steer_bias" in reason

    data, params = _timeseries_dataset(samples=500)
    params["acc_time_delay"] = -0.1
    figures, reason = physical_validity._timeseries_figures(data, params, 0.01)
    assert figures is None
    assert "time_delay" in reason


def test_timeseries_figures_decimate_long_traces() -> None:
    samples = 3 * physical_validity._TIMESERIES_MAX_POINTS
    data, params = _timeseries_dataset(samples=samples)
    figures, reason = physical_validity._timeseries_figures(data, params, 0.01)
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


def test_build_timeseries_section_renders_five_figures(tmp_path: Path, monkeypatch) -> None:
    dataset_dir = tmp_path / "datasets" / "good"
    dataset_dir.mkdir(parents=True)
    (dataset_dir / "reidentify_cache.csv").write_text("stub", encoding="utf-8")
    data, _params_unused = _timeseries_dataset(samples=500)
    monkeypatch.setattr(physical_validity, "read_dataset_csv", lambda _path: {"steering": "df"})
    monkeypatch.setattr(
        physical_validity, "steer_dataframe_from_source", lambda _source, *, df_steer: df_steer
    )
    monkeypatch.setattr(physical_validity, "build_resampled", lambda *_args, **_kwargs: data)

    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(
            tmp_path / "scenario.yaml", ["baseline", "current"],
            plot_dataset=["good"], release={"model": "current", "version": 2},
        ),
    )

    assert rendered.count("<h3>") == 5
    assert rendered.count("plotly-graph-div") >= 5
    assert rendered.count('class="ts-dataset"') == 1
    assert "release ケース current" in rendered
    assert 'class="stats"' in rendered
    assert "good" in rendered


def test_build_timeseries_section_renders_multiple_datasets(tmp_path: Path, monkeypatch) -> None:
    """リスト指定では各データセットを順に描画し、1 件の失敗が他を止めない。"""
    dataset_dir = tmp_path / "datasets" / "good"
    dataset_dir.mkdir(parents=True)
    (dataset_dir / "reidentify_cache.csv").write_text("stub", encoding="utf-8")
    data, _params_unused = _timeseries_dataset(samples=500)
    monkeypatch.setattr(physical_validity, "read_dataset_csv", lambda _path: {"steering": "df"})
    monkeypatch.setattr(
        physical_validity, "steer_dataframe_from_source", lambda _source, *, df_steer: df_steer
    )
    monkeypatch.setattr(physical_validity, "build_resampled", lambda *_args, **_kwargs: data)

    rendered = physical_validity.build_timeseries_section(
        tmp_path,
        _params(tmp_path / "params.yaml"),
        _scenario(
            tmp_path / "scenario.yaml", ["baseline", "current"],
            plot_dataset=["good", "missing"],
        ),
    )

    assert rendered.count('class="ts-dataset"') == 2
    assert rendered.count("plotly-graph-div") == 5
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
