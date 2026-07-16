from __future__ import annotations

from pathlib import Path
import sys

import yaml

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import pipeline


def _write_inputs(tmp_path: Path, *, conditions_extra: dict | None = None) -> tuple[Path, Path, Path]:
    root = tmp_path / "collection"
    (root / "datasets").mkdir(parents=True)
    scenario = tmp_path / "scenario.yaml"
    conditions = {
        "comparison_models": ["baseline", "current"],
        "release": {"model": "current", "version": 2},
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
    conditions.update(conditions_extra or {})
    scenario.write_text(
        yaml.safe_dump({"Evaluation": {"Conditions": conditions}}, sort_keys=False),
        encoding="utf-8",
    )
    input_param = tmp_path / "simulator_model.param.yaml"
    input_param.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER",
                        "delay_steer_acc_geared_for_diffusion_planner": {
                            "version": 1,
                            "v1": {},
                        },
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return root, scenario, input_param


def test_main_runs_only_the_fixed_pipeline_in_order(
    tmp_path: Path, monkeypatch
) -> None:
    root, scenario, input_param = _write_inputs(tmp_path)
    calls: list[str] = []

    monkeypatch.setattr(
        pipeline,
        "_step_extract",
        lambda *_args, **_kwargs: calls.append("extract") or {},
    )
    monkeypatch.setattr(
        pipeline,
        "_step_fit_lon",
        lambda *_args, **_kwargs: calls.append("fit_lon") or tmp_path / "phase1.yaml",
    )
    monkeypatch.setattr(
        pipeline,
        "_step_fit_steer",
        lambda *_args, **_kwargs: calls.append("fit_steer") or tmp_path / "phase2.yaml",
    )
    monkeypatch.setattr(
        pipeline,
        "_step_fit_xy",
        lambda *_args, **_kwargs: calls.append("fit_xy") or tmp_path / "phase3.yaml",
    )
    monkeypatch.setattr(
        pipeline,
        "_step_fit_merge",
        lambda *_args, **_kwargs: (
            calls.append("fit_merge") or tmp_path / "tuned.yaml",
            tmp_path / "metrics.csv",
            {"metadata": {"skipped": []}},
        ),
    )
    monkeypatch.setattr(
        pipeline,
        "_step_report",
        lambda *_args, **_kwargs: calls.append("report"),
    )
    monkeypatch.setattr(
        pipeline,
        "_step_release",
        lambda *_args, **_kwargs: calls.append("release") or tmp_path / "released.yaml",
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "pipeline",
            "--root",
            str(root),
            "--scenario",
            str(scenario),
            "--input-param",
            str(input_param),
            "--n-trials",
            "1",
            "--n-jobs",
            "1",
        ],
    )

    pipeline.main()

    assert calls == ["extract", "fit_lon", "fit_steer", "fit_xy", "fit_merge", "release", "report"]


def _install_call_recorders(monkeypatch, calls: list[str], tmp_path: Path) -> None:
    recorders = {
        "_step_extract": lambda *_a, **_k: calls.append("extract") or {},
        "_step_fit_lon": lambda *_a, **_k: calls.append("fit_lon") or tmp_path / "phase1.yaml",
        "_step_fit_steer": lambda *_a, **_k: calls.append("fit_steer") or tmp_path / "phase2.yaml",
        "_step_fit_xy": lambda *_a, **_k: calls.append("fit_xy") or tmp_path / "phase3.yaml",
        "_step_fit_merge": lambda *_a, **_k: (
            calls.append("fit_merge") or tmp_path / "tuned.yaml",
            tmp_path / "metrics.csv",
            {"metadata": {"skipped": []}},
        ),
        "_step_evaluate": lambda *_a, **_k: (
            calls.append("evaluate") or tmp_path / "tuned.yaml",
            tmp_path / "metrics.csv",
            {"metadata": {"skipped": []}},
        ),
        "_step_release": lambda *_a, **_k: calls.append("release") or tmp_path / "released.yaml",
        "_step_report": lambda *_a, **_k: calls.append("report"),
    }
    for name, fn in recorders.items():
        monkeypatch.setattr(pipeline, name, fn)


def test_main_skips_disabled_fit_stages_and_release(tmp_path: Path, monkeypatch) -> None:
    """fit.stages: [] は全 fit をスキップし evaluate を実行、release 未指定なら release も飛ばす。"""
    root, scenario, _input_param = _write_inputs(
        tmp_path, conditions_extra={"fit": {"stages": []}, "release": None},
    )
    # release: None は mapping 検証を通らないので削除する。
    import yaml as _yaml  # noqa: PLC0415

    doc = _yaml.safe_load(scenario.read_text(encoding="utf-8"))
    doc["Evaluation"]["Conditions"].pop("release")
    scenario.write_text(_yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")

    calls: list[str] = []
    _install_call_recorders(monkeypatch, calls, tmp_path)
    monkeypatch.setattr(
        sys, "argv",
        ["pipeline", "--root", str(root), "--scenario", str(scenario),
         "--n-trials", "1", "--n-jobs", "1"],
    )

    pipeline.main()

    assert calls == ["extract", "evaluate", "report"]


def test_main_runs_merge_only_without_direct_fit(tmp_path: Path, monkeypatch) -> None:
    """fit.stages: [merge] は direct-fit を飛ばして fit_merge のみ実行する。"""
    root, scenario, input_param = _write_inputs(
        tmp_path, conditions_extra={"fit": {"stages": ["merge"]}},
    )
    calls: list[str] = []
    _install_call_recorders(monkeypatch, calls, tmp_path)
    monkeypatch.setattr(
        sys, "argv",
        ["pipeline", "--root", str(root), "--scenario", str(scenario),
         "--input-param", str(input_param), "--n-trials", "1", "--n-jobs", "1"],
    )

    pipeline.main()

    assert calls == ["extract", "fit_merge", "release", "report"]


def test_main_report_only_regenerates_report_from_existing_artifacts(
    tmp_path: Path, monkeypatch,
) -> None:
    """--report-only は fit/release を行わず、既存成果物から report だけ作り直す。"""
    root, scenario, _input_param = _write_inputs(tmp_path)
    out_dir = root / "reidentify"
    out_dir.mkdir(parents=True)
    (out_dir / "tuned_params.yaml").write_text("params: {}\n", encoding="utf-8")
    (out_dir / "metrics.csv").write_text("dataset_id,model,horizon\n", encoding="utf-8")

    calls: list[str] = []
    _install_call_recorders(monkeypatch, calls, tmp_path)
    monkeypatch.setattr(
        sys, "argv",
        ["pipeline", "--root", str(root), "--scenario", str(scenario),
         "--n-jobs", "1", "--report-only"],
    )

    pipeline.main()

    assert calls == ["report"]


def test_report_step_generates_only_the_unified_report(
    tmp_path: Path, monkeypatch
) -> None:
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify import report

    out_dir = tmp_path / "collection" / "reidentify"
    out_dir.mkdir(parents=True)
    (out_dir.parent / "datasets").mkdir()
    tuned = out_dir / "tuned_params.yaml"
    tuned.write_text("params: {}\n", encoding="utf-8")
    phase1 = out_dir / "phase1_acc.yaml"
    phase2 = out_dir / "phase2_steer.yaml"
    phase3 = out_dir / "phase3_xy.yaml"
    released = out_dir / "simulator_model.param.yaml"
    calls: list[dict] = []
    monkeypatch.setattr(report, "run", lambda *_args, **kwargs: calls.append(kwargs))

    pipeline._step_report(
        out_dir,
        phase1,
        phase2,
        phase3,
        tuned,
        out_dir / "metrics.csv",
        released,
        {},
        {"metadata": {"skipped": []}},
        tmp_path / "scenario.yaml",
        3,
    )

    assert calls == [{
        "failures": {"skipped": [], "n_skipped": 0},
        "collection_dir": out_dir.parent,
        "scenario": tmp_path / "scenario.yaml",
        "n_jobs": 3,
        "extraction_summary": {},
        "phase1_params": phase1,
        "phase2_params": phase2,
        "phase3_params": phase3,
        "release_params": released,
    }]
