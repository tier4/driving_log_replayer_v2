"""Parallel physical-validity sections for the unified reidentification report."""
from __future__ import annotations

import multiprocessing
from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np
import plotly.graph_objects as go
import yaml

from .lib._parallel import normalize_parallel_jobs, pool_chunksize, set_worker_thread_env_defaults
from .lib._report_format import escape as _escape, format_number as _number
from .reidentify.fit_core import simulate_first_order
from .reidentify.load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .reidentify.model_config import ModelSpec, load_model_config
from .reidentify.residuals import build_xy_columns, rmse, xy_residual, yaw_residual
from .reidentify.settings import (
    BASELINE_MODEL_NAME,
    LONG_DA_THRESH,
    LONG_VX_MIN,
    RESAMPLE_DT,
    STEER_DSTEER_MIN,
    TARGET_MODEL_NAME,
)


@dataclass
class DatasetEvaluation:
    dataset_id: str
    metrics: dict[str, dict[str, dict[str, Any]]] = field(default_factory=dict)
    error: str | None = None


@dataclass
class ComparedModel:
    name: str
    vehicle_model_type: str
    acceleration_source: str
    params: dict[str, Any]


@dataclass
class ValidationContext:
    params: dict[str, Any]
    model_type: str
    wheelbase: float
    models: list[ComparedModel] = field(default_factory=list)
    evaluations: list[DatasetEvaluation] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)


@dataclass
class ValidationStep:
    name: str
    result: dict[str, Any]
    html: str


@dataclass
class PhysicalValiditySections:
    """Physical-validity fragments, one per validation step; the caller decides placement."""

    prepare: str
    longitudinal: str
    steering: str
    yaw: str
    xy: str


def _figure_html(fig: go.Figure, title: str) -> str:
    return f"<h3>{_escape(title)}</h3>{fig.to_html(full_html=False, include_plotlyjs=False)}"


def _histogram_by_model(values: dict[str, list[float]], title: str) -> go.Figure:
    fig = go.Figure()
    for name, samples in values.items():
        if samples:
            fig.add_trace(go.Histogram(x=samples, name=name, opacity=0.65))
    fig.update_layout(title=title, barmode="overlay", xaxis_title="RMSE", yaxis_title="データセット数")
    return fig


def _valid_number(params: dict[str, Any], key: str, *, positive: bool = False) -> tuple[float | None, str | None]:
    value = params.get(key)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None, f"{key} がありません"
    if not np.isfinite(number) or (positive and number <= 0.0):
        return None, f"{key} は{'正の' if positive else '有限の'}数値である必要があります"
    return number, None


def _models_from_inputs(params_path: Path, scenario: Path | None, case: str) -> list[ComparedModel]:
    tuned_document = yaml.safe_load(params_path.read_text(encoding="utf-8")) or {}
    tuned_params = dict(tuned_document.get("params", tuned_document))
    if scenario is None:
        return [ComparedModel(case, str(tuned_params.get("vehicle_model_type", case)), "accel", tuned_params)]
    config = load_model_config(scenario)
    models: list[ComparedModel] = []
    for name in config.comparison_models:
        spec: ModelSpec = config.find_case(name)
        params = dict(spec.params)
        if name == TARGET_MODEL_NAME:
            params.update(tuned_params)
        models.append(ComparedModel(name, spec.vehicle_model_type, spec.acceleration_source, params))
    return models


def _fixed_metric(data: dict[str, Any], params: dict[str, Any], *, steer: bool) -> dict[str, Any]:
    metrics, _prediction, reason = _fixed_response(data, params, steer=steer)
    return metrics or {"rmse": float("nan"), "n": 0, "reason": reason or "評価不能"}


def _cross_metric(data: dict[str, Any], params: dict[str, Any], *, xy: bool) -> dict[str, Any]:
    first, reason = _valid_number(
        params, "xy_heading_rate_coeff" if xy else "wheelbase", positive=not xy
    )
    second, second_reason = (None, None) if xy else _valid_number(params, "k_us")
    if reason or second_reason:
        return {"rmse": float("nan"), "n": 0, "reason": reason or second_reason or "評価不能"}
    if xy:
        residual = np.concatenate(xy_residual(data, float(first)))
    else:
        residual = yaw_residual(data, k_us=float(second), wheelbase=float(first))
    return {
        "rmse": rmse(residual),
        "n": int(len(residual)),
    }


def _evaluate_dataset(
    task: tuple[str, Path, tuple[ComparedModel, ...], bool]
) -> DatasetEvaluation:
    """Read, resample, and evaluate one dataset without returning raw arrays."""
    dataset_id, csv_path, models, use_default_acceleration_source = task
    try:
        source = read_dataset_csv(csv_path)
        sources = tuple(dict.fromkeys(model.acceleration_source for model in models))
        data_by_source: dict[str, dict[str, Any]] = {}
        for acceleration_source in sources:
            if use_default_acceleration_source:
                data = build_resampled(source, RESAMPLE_DT, context=f"physical_validity:{dataset_id}")
            else:
                data = build_resampled(
                    source,
                    RESAMPLE_DT,
                    context=f"physical_validity:{dataset_id}",
                    acceleration_source=acceleration_source,
                )
            if data is None:
                raise ValueError("共通時間範囲が短すぎるか、必須信号が空です")
            build_xy_columns(data, source)
            data_by_source[acceleration_source] = data

        metrics = {name: {} for name in ("longitudinal", "steering", "yaw", "xy")}
        for model in models:
            data = data_by_source[model.acceleration_source]
            metrics["longitudinal"][model.name] = _fixed_metric(data, model.params, steer=False)
            metrics["steering"][model.name] = _fixed_metric(data, model.params, steer=True)
            metrics["yaw"][model.name] = _cross_metric(data, model.params, xy=False)
            metrics["xy"][model.name] = _cross_metric(data, model.params, xy=True)
        return DatasetEvaluation(dataset_id, metrics=metrics)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return DatasetEvaluation(dataset_id, error=str(exc))


def prepare_datasets(
    collection_dir: Path,
    params_path: Path,
    *,
    scenario: Path | None = None,
    case: str = TARGET_MODEL_NAME,
    n_jobs: int = 1,
) -> tuple[ValidationContext, ValidationStep]:
    """Evaluate cached datasets in parallel while retaining only compact results."""
    models = _models_from_inputs(params_path, scenario, case)
    current = next((model for model in models if model.name == TARGET_MODEL_NAME), models[0])
    wheelbase, _ = _valid_number(current.params, "wheelbase", positive=True)
    context = ValidationContext(
        params=current.params, model_type=current.vehicle_model_type,
        wheelbase=wheelbase if wheelbase is not None else float("nan"), models=models,
    )
    tasks = [
        (dataset_id, csv_path, tuple(models), scenario is None)
        for dataset_id, csv_path in discover_cached_datasets(collection_dir)
    ]
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    print(f"[report] physical-validity: {len(tasks)} datasets, {n_workers} worker(s)")
    if n_workers == 1:
        evaluations = [_evaluate_dataset(task) for task in tasks]
    else:
        chunksize = pool_chunksize(len(tasks), n_workers)
        with multiprocessing.get_context("fork").Pool(
            n_workers, initializer=set_worker_thread_env_defaults
        ) as pool:
            evaluations = list(pool.imap(_evaluate_dataset, tasks, chunksize=chunksize))
    for evaluation in evaluations:
        if evaluation.error:
            context.skipped.append(f"{evaluation.dataset_id}: {evaluation.error}")
        else:
            context.evaluations.append(evaluation)
    context.evaluations.sort(key=lambda evaluation: evaluation.dataset_id)
    skipped = "".join(f"<li>{_escape(reason)}</li>" for reason in context.skipped)
    model_rows = "".join(
        f"<li><code>{_escape(model.name)}</code>: {_escape(model.vehicle_model_type)} / accel={_escape(model.acceleration_source)}</li>"
        for model in models
    )
    section = f'''<section id="prepare"><h2>準備</h2><p>有効データセット: {len(context.evaluations)}</p>
<h3>比較モデル</h3><ul>{model_rows}</ul><h3>除外理由</h3>{f'<ul>{skipped}</ul>' if skipped else '<p>なし</p>'}</section>'''
    return context, ValidationStep("prepare", {"n_valid": len(context.evaluations), "skipped": context.skipped, "models": [m.name for m in models]}, section)


def _fixed_response(data: dict[str, Any], params: dict[str, Any], *, steer: bool) -> tuple[dict[str, Any] | None, np.ndarray | None, str | None]:
    prefix = "steer" if steer else "acc"
    tau, reason = _valid_number(params, f"{prefix}_time_constant", positive=True)
    delay, delay_reason = _valid_number(params, f"{prefix}_time_delay")
    if reason or delay_reason or delay is None or delay < 0.0:
        return None, None, reason or ("%s_time_delay は0以上である必要があります" % prefix)
    cmd = data["d_cmd"] if steer else data["a_cmd"]
    actual = data["d_act"] if steer else data["a_act"]
    threshold = STEER_DSTEER_MIN / RESAMPLE_DT if steer else LONG_DA_THRESH
    mask = data["gear_drive"] & (data["vx"] > LONG_VX_MIN) & (np.abs(np.gradient(cmd, RESAMPLE_DT)) > threshold)
    bias = 0.0
    if steer:
        bias, bias_reason = _valid_number(params, "steer_bias")
        if bias_reason:
            return None, None, bias_reason
    prediction = simulate_first_order(cmd, tau, delay, RESAMPLE_DT) + bias
    residual = prediction[mask] - actual[mask]
    return {"rmse": rmse(residual), "n": int(len(residual))}, prediction, None


def _summary_table(models: list[ComparedModel], results: dict[str, dict[str, Any]], param_keys: tuple[str, ...]) -> str:
    baseline = results.get(BASELINE_MODEL_NAME, {}).get("rmse", float("nan"))
    rows = []
    for model in models:
        value = results[model.name]
        rmse = value.get("rmse", float("nan"))
        if model.name == BASELINE_MODEL_NAME and np.isfinite(rmse):
            ratio = 1.0
        else:
            ratio = rmse / baseline if np.isfinite(rmse) and np.isfinite(baseline) and baseline != 0 else float("nan")
        params = ", ".join(f"{key}={_number(model.params.get(key))}" for key in param_keys)
        reason = value.get("reason", "")
        rows.append(f"<tr><th>{_escape(model.name)}</th><td>{_number(rmse)}</td><td>{_number(ratio)}</td><td>{value.get('n', 0)}</td><td><code>{_escape(params)}</code></td><td>{_escape(reason)}</td></tr>")
    return "<table><thead><tr><th>モデル</th><th>固定評価 RMSE</th><th>baseline 比</th><th>サンプル数</th><th>使用パラメータ</th><th>評価不能理由</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table>"


def _response_step(context: ValidationContext, *, steer: bool) -> ValidationStep:
    title, section_id = ("操舵", "steering") if steer else ("縦方向", "longitudinal")
    fixed: dict[str, dict[str, Any]] = {}
    distributions: dict[str, list[float]] = {}
    for model in context.models:
        per_dataset = {item.dataset_id: item.metrics[section_id][model.name] for item in context.evaluations}
        samples = [value["rmse"] for value in per_dataset.values() if np.isfinite(value["rmse"])]
        n = sum(value.get("n", 0) for value in per_dataset.values())
        total_squared_error = sum(value.get("rmse", float("nan")) ** 2 * value.get("n", 0) for value in per_dataset.values() if np.isfinite(value.get("rmse", float("nan"))))
        fixed[model.name] = {
            "datasets": per_dataset,
            "rmse": float(np.sqrt(total_squared_error / n)) if n else float("nan"),
            "n": n,
            "reason": next((value.get("reason") for value in per_dataset.values() if value.get("reason")), ""),
        }
        distributions[model.name] = samples
    keys = ("steer_time_constant", "steer_time_delay", "steer_bias") if steer else ("acc_time_constant", "acc_time_delay")
    section = f'<section id="{section_id}"><h2>{title}</h2>' + _summary_table(context.models, fixed, keys) + _figure_html(_histogram_by_model(distributions, f"{title}: モデル別固定評価 RMSE"), "固定評価 RMSE 分布") + "</section>"
    return ValidationStep(section_id, {"models": fixed, "datasets": fixed.get(TARGET_MODEL_NAME, {}).get("datasets", {})}, section)


def validate_longitudinal(context: ValidationContext) -> ValidationStep:
    return _response_step(context, steer=False)


def validate_steering(context: ValidationContext) -> ValidationStep:
    return _response_step(context, steer=True)


def _cross_step(context: ValidationContext, *, xy: bool) -> ValidationStep:
    title, section_id = ("x/y", "xy") if xy else ("yaw", "yaw")
    fixed: dict[str, dict[str, Any]] = {}
    distributions: dict[str, list[float]] = {}
    for model in context.models:
        per_dataset = {item.dataset_id: item.metrics[section_id][model.name] for item in context.evaluations}
        samples = [value["rmse"] for value in per_dataset.values() if np.isfinite(value["rmse"])]
        n = sum(value["n"] for value in per_dataset.values())
        total_squared_error = sum(value["rmse"] ** 2 * value["n"] for value in per_dataset.values() if np.isfinite(value["rmse"]))
        fixed[model.name] = {"datasets": per_dataset, "rmse": float(np.sqrt(total_squared_error / n)) if n else float("nan"), "n": n, "reason": next((value.get("reason") for value in per_dataset.values() if value.get("reason")), "")}
        distributions[model.name] = samples
    keys = ("xy_heading_rate_coeff",) if xy else ("wheelbase", "k_us")
    section = f'<section id="{section_id}"><h2>{title}</h2>' + _summary_table(context.models, fixed, keys) + _figure_html(_histogram_by_model(distributions, f"{title}: モデル別固定評価 RMSE"), "固定評価 RMSE 分布") + "</section>"
    return ValidationStep(section_id, {"models": fixed, "datasets": fixed.get(TARGET_MODEL_NAME, {}).get("datasets", {})}, section)


def validate_yaw(context: ValidationContext) -> ValidationStep:
    return _cross_step(context, xy=False)


def validate_xy(context: ValidationContext) -> ValidationStep:
    return _cross_step(context, xy=True)


def build_sections(
    collection_dir: Path,
    params_path: Path,
    *,
    scenario: Path,
    n_jobs: int = 1,
) -> PhysicalValiditySections:
    """Build report fragments; the caller places each fragment in its owning section."""
    started = perf_counter()
    context, prepared = prepare_datasets(collection_dir, params_path, scenario=scenario, n_jobs=n_jobs)
    sections = PhysicalValiditySections(
        prepare=prepared.html,
        longitudinal=validate_longitudinal(context).html,
        steering=validate_steering(context).html,
        yaw=validate_yaw(context).html,
        xy=validate_xy(context).html,
    )
    print(f"[report] physical-validity complete: {perf_counter() - started:.1f}s")
    return sections
