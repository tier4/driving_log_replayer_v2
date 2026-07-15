from __future__ import annotations

from typing import TYPE_CHECKING

import pandas as pd
import pytest
import yaml

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import report

if TYPE_CHECKING:
    from pathlib import Path


def _write_metrics(path: Path, horizons: tuple[float, ...] = (0.5, 1.0)) -> None:
    rows = []
    for dataset_id, scale in (("dataset-a", 1.0), ("dataset-b", 2.0)):
        for horizon in horizons:
            for model, model_scale in (("baseline", 1.0), ("tuned", 0.8)):
                value = scale * horizon * model_scale
                rows.append(
                    {
                        "dataset_id": dataset_id,
                        "model": model,
                        "horizon": horizon,
                        "pos": value,
                        "long": value + 0.1,
                        "lat": value + 0.2,
                        "yaw": value + 0.3,
                        "steer": value + 0.4,
                        "vx": value + 0.5,
                        "ax": value + 0.6,
                    }
                )
    pd.DataFrame(rows).to_csv(path, index=False)


def _stub_physical_validity(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        report.physical_validity,
        "build_sections",
        lambda *_args, **_kwargs: report.physical_validity.PhysicalValiditySections(
            prepare='<section id="prepare">prepared</section>',
            longitudinal="",
            steering="",
            yaw="",
            xy="",
        ),
    )


def test_run_builds_unified_report_from_finalized_artifacts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    _stub_physical_validity(monkeypatch)
    tuned = tmp_path / "tuned_params.yaml"
    tuned.write_text(
        yaml.safe_dump(
            {
                "params": {
                    "acc_time_constant": 0.45,
                    "steer_bias": "<unsafe & escaped>",
                },
                "parameter_constraints": {
                    "acc_time_constant": {
                    "allowed_range": {
                            "minimum": 0.1, "maximum": 3.0,
                            "minimum_inclusive": True, "maximum_inclusive": True,
                        },
                        "reason": "stable response",
                    },
                },
                "comparison": {
                    "baseline": {"score": 2.5},
                    "tuned": {"score": 1.25},
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    metrics = tmp_path / "metrics.csv"
    _write_metrics(metrics)
    output = tmp_path / "nested" / "report.html"

    result = report.run(
        tuned,
        metrics,
        output,
        failures={
            "n_datasets": 3,
            "n_cached": 1,
            "n_extracted": 1,
            "n_skipped": 1,
            "n_failed": 0,
            "skipped": [{"dataset_id": "bad<script>", "reason": "missing & short"}],
            "failed": [],
        },
        collection_dir=tmp_path,
        scenario=tmp_path / "scenario.yaml",
    )

    rendered = output.read_text(encoding="utf-8")
    assert result == output
    assert "<!doctype html>" in rendered
    assert "Parameters" in rendered
    assert "Allowed range" in rendered
    assert "stable response" in rendered
    assert "Aggregate comparison" in rendered
    assert "Horizon comparison" not in rendered
    assert "<th>Horizon</th>" not in rendered
    assert "Error by horizon N" in rendered
    assert rendered.count('class="horizon-chart"') == len(report.REQUIRED_METRICS)
    assert rendered.count('class="series baseline"') == len(report.REQUIRED_METRICS)
    assert rendered.count('class="series tuned"') == len(report.REQUIRED_METRICS)
    assert rendered.count('class="point baseline"') == 2 * len(report.REQUIRED_METRICS)
    assert rendered.count('class="point tuned"') == 2 * len(report.REQUIRED_METRICS)
    assert "N (horizon)" in rendered
    assert "mean RMSE" in rendered
    assert "pos — baseline — N 0.5: mean RMSE 0.75" in rendered
    for metric in report.REQUIRED_METRICS:
        assert f"{metric} RMSE" in rendered
    assert "Dataset distributions" in rendered
    assert "1. Extraction results" in rendered
    assert "2. Longitudinal direct identification" in rendered
    assert "3. Steering direct identification" in rendered
    assert "4. XY heading-rate direct identification" in rendered
    assert "5. Integrated optimization" in rendered
    assert "6. Released YAML" in rendered
    assert "2.5" in rendered
    assert "1.25" in rendered
    assert "tuned_params.yaml: comparison.baseline.score" in rendered
    assert "scenario.yaml: models.baseline.params (fixed)" in rendered
    assert "tuned_params.yaml: params" in rendered
    assert "dataset-a" in rendered
    assert "<svg" in rendered
    assert "&lt;unsafe &amp; escaped&gt;" in rendered
    assert "bad&lt;script&gt;" not in rendered
    assert "missing &amp; short" not in rendered
    assert 'id="prepare"' in rendered


def test_run_plots_every_available_horizon(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    _stub_physical_validity(monkeypatch)
    tuned = tmp_path / "tuned.yaml"
    tuned.write_text("params:\n  k_us: 0.01\n", encoding="utf-8")
    metrics = tmp_path / "metrics.csv"
    horizons = tuple(float(index) for index in range(1, 301))
    _write_metrics(metrics, horizons)

    output = report.run(
        tuned, metrics, tmp_path / "report.html", failures={},
        collection_dir=tmp_path, scenario=tmp_path / "scenario.yaml",
    )

    rendered = output.read_text(encoding="utf-8")
    expected_points = len(horizons) * len(report.REQUIRED_METRICS)
    assert rendered.count('class="point baseline"') == expected_points
    assert rendered.count('class="point tuned"') == expected_points
    assert rendered.count('class="tick x-tick"') <= 11 * len(report.REQUIRED_METRICS)
    assert rendered.count('width="960"') == len(report.REQUIRED_METRICS)


def test_run_plots_all_scenario_comparison_models_in_declared_order(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    _stub_physical_validity(monkeypatch)
    scenario = tmp_path / "scenario.yaml"
    model_type = "delay_steer_acc_geared_for_diffusion_planner"
    scenario.write_text(yaml.safe_dump({"Evaluation": {"Conditions": {
        "comparison_models": ["baseline", "v1", "v1_rk4", "current"],
        "models": {
            name: {"vehicle_model_type": model_type, "params": {"wheelbase": 4.7}}
            for name in ("baseline", "v1", "v1_rk4", "current")
        },
    }}}), encoding="utf-8")
    tuned = tmp_path / "tuned.yaml"
    tuned.write_text("params: {}\n", encoding="utf-8")
    rows = []
    for model, scale in (("baseline", 1.0), ("v1", 0.9), ("v1_rk4", 0.85), ("tuned", 0.8)):
        for horizon in (1, 2):
            rows.append({"dataset_id": "dataset-a", "model": model, "horizon": horizon,
                         **{metric: scale * horizon for metric in report.REQUIRED_METRICS}})
    metrics = tmp_path / "metrics.csv"
    pd.DataFrame(rows).to_csv(metrics, index=False)

    output = report.run(tuned, metrics, tmp_path / "report.html", failures={},
                        collection_dir=tmp_path, scenario=scenario)
    rendered = output.read_text(encoding="utf-8")
    first_chart = rendered[rendered.index('class="horizon-chart"'):]
    assert [first_chart.index(f'class="series {model}"') for model in ("baseline", "v1", "v1_rk4", "tuned")] == sorted(
        first_chart.index(f'class="series {model}"') for model in ("baseline", "v1", "v1_rk4", "tuned")
    )
    for model in ("baseline", "v1", "v1_rk4", "tuned"):
        assert rendered.count(f'class="series {model}"') == len(report.REQUIRED_METRICS)
        assert rendered.count(f'class="point {model}"') == 2 * len(report.REQUIRED_METRICS)
    assert "scenario.yaml: models.v1.params (fixed)" in rendered
    assert "scenario.yaml: models.v1_rk4.params (fixed)" in rendered
    assert "tuned_params.yaml: params" in rendered


def test_chart_uses_numeric_horizon_spacing(tmp_path: Path) -> None:
    metrics = tmp_path / "metrics.csv"
    _write_metrics(metrics, (10.0, 20.0, 100.0))

    rendered = report._line_chart_svg(report._load_metrics(metrics), "pos")

    assert 'points="68.00,' in rendered
    assert "164.44," in rendered
    assert "936.00," in rendered


def test_summary_keeps_configured_score_horizons(tmp_path: Path) -> None:
    metrics = tmp_path / "metrics.csv"
    _write_metrics(metrics, (1.0, 2.0, 3.0))
    frame = report._add_normalized_scores(report._load_metrics(metrics))

    summary = report._summary_frame(
        {"metadata": {"score_horizons": [1, 3]}},
        frame,
    )

    assert set(summary["horizon"]) == {1.0, 3.0}
    assert len(summary) == 2 * 2 * 2


def test_run_rejects_unpaired_baseline_and_tuned_rows(tmp_path: Path) -> None:
    tuned = tmp_path / "tuned.yaml"
    tuned.write_text("params:\n  k_us: 0.01\n", encoding="utf-8")
    metrics = tmp_path / "metrics.csv"
    _write_metrics(metrics)
    frame = pd.read_csv(metrics)
    frame = frame.loc[
        ~(
            (frame["dataset_id"] == "dataset-b")
            & (frame["model"] == "tuned")
            & (frame["horizon"] == 1.0)
        )
    ]
    frame.to_csv(metrics, index=False)

    with pytest.raises(ValueError, match="identical dataset/horizon pairs"):
        report.run(
            tuned, metrics, tmp_path / "report.html", failures={},
            collection_dir=tmp_path, scenario=tmp_path / "scenario.yaml",
        )
