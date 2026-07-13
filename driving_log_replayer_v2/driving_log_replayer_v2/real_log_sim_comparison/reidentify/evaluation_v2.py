"""Additive version-2 rollout metrics and report rendering.

The legacy N-step metrics deliberately live elsewhere.  This module consumes
already-computed per-dataset RMSE values and builds a self-contained V2 block,
so adding it to ``tuned_params.yaml`` cannot change ``score`` or
``comparison.*.{score,by_h}``.
"""

from __future__ import annotations

from collections.abc import Iterable
from collections.abc import Mapping
import html
import math
import statistics
from typing import Any

METRICS_SCHEMA_VERSION = 2
HORIZONS_S: tuple[float, ...] = (0.25, 0.5, 1.0, 2.0, 3.0)
METRIC_WEIGHTS: dict[str, float] = {
    "yaw": 0.20,
    "lat": 0.15,
    "long": 0.15,
    "vx": 0.15,
    "wz": 0.15,
    "steer": 0.10,
    "ax": 0.10,
}
METRIC_UNITS: dict[str, str] = {
    "yaw": "deg",
    "lat": "cm",
    "long": "cm",
    "vx": "m/s",
    "wz": "rad/s",
    "steer": "deg",
    "ax": "m/s^2",
}
CVAR_TAIL_FRACTION = 0.05
_NORMALIZATION_EPSILON = 1.0e-12

PerDatasetMetrics = (
    Mapping[str, Mapping[float, Mapping[str, float]]]
    | Iterable[tuple[str, Mapping[float, Mapping[str, float]]]]
)


def horizon_steps_from_timestamps(
    timestamps_s: Iterable[float],
    horizons_s: tuple[float, ...] = HORIZONS_S,
) -> dict[float, int]:
    """Map V2 time horizons to nearest sample intervals for one dataset.

    The mapping is derived independently for every dataset from its median
    command period. Half-sample ties round up (for example, 0.25 s at 10 Hz
    becomes three intervals), avoiding Python's even-number tie rounding.
    """
    timestamps = [float(value) for value in timestamps_s]
    if len(timestamps) < 2:
        raise ValueError("at least two timestamps are required for V2 horizons")
    if not all(math.isfinite(value) for value in timestamps):
        raise ValueError("V2 timestamps must all be finite")
    intervals = [right - left for left, right in zip(timestamps[:-1], timestamps[1:], strict=True)]
    if any(interval <= 0.0 for interval in intervals):
        raise ValueError("V2 timestamps must be strictly increasing")
    sample_period = float(statistics.median(intervals))

    result: dict[float, int] = {}
    used_steps: set[int] = set()
    for horizon in horizons_s:
        canonical_horizon = _canonical_horizon(horizon)
        steps = max(1, int(math.floor(canonical_horizon / sample_period + 0.5 + 1.0e-12)))
        if steps in used_steps:
            raise ValueError(
                "V2 time horizons collapse to the same sample interval; "
                f"sample_period={sample_period}, horizon={canonical_horizon}, steps={steps}"
            )
        used_steps.add(steps)
        result[canonical_horizon] = steps
    return result


def select_time_horizon_metrics(
    step_metrics: Mapping[int, Mapping[str, float]],
    horizon_steps: Mapping[float, int],
) -> dict[float, dict[str, float]]:
    """Relabel evaluated sample-interval metrics with their requested seconds."""
    result: dict[float, dict[str, float]] = {}
    for raw_horizon, raw_steps in horizon_steps.items():
        horizon = _canonical_horizon(raw_horizon)
        steps = int(raw_steps)
        if steps not in step_metrics:
            raise ValueError(f"rollout result lacks the {steps}-interval V2 horizon ({horizon}s)")
        result[horizon] = {key: float(value) for key, value in step_metrics[steps].items()}
    return result


def worst_tail_cvar(values: Iterable[float], tail_fraction: float = CVAR_TAIL_FRACTION) -> float:
    """Return the arithmetic mean of the largest ``tail_fraction`` values.

    At least one sample is selected.  ``ceil`` is intentional: for a dataset
    count that is not divisible by twenty, using ``floor`` would cover less
    than the requested worst five percent.
    """
    if not 0.0 < tail_fraction <= 1.0:
        raise ValueError(f"tail_fraction must be in (0, 1], got {tail_fraction!r}")

    clean = [float(value) for value in values]
    if not clean:
        raise ValueError("CVaR requires at least one value")
    if not all(math.isfinite(value) for value in clean):
        raise ValueError("CVaR values must all be finite")

    count = max(1, math.ceil(len(clean) * tail_fraction))
    return float(statistics.fmean(sorted(clean, reverse=True)[:count]))


def _canonical_horizon(value: Any) -> float:
    try:
        horizon = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid V2 horizon: {value!r}") from exc
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError(f"V2 horizon must be finite and positive, got {value!r}")
    return round(horizon, 9)


def _validate_weights(weights: Mapping[str, float]) -> dict[str, float]:
    if set(weights) != set(METRIC_WEIGHTS):
        missing = sorted(set(METRIC_WEIGHTS) - set(weights))
        extra = sorted(set(weights) - set(METRIC_WEIGHTS))
        raise ValueError(
            f"V2 weights must cover exactly seven metrics; missing={missing}, extra={extra}"
        )

    clean = {metric: float(weights[metric]) for metric in METRIC_WEIGHTS}
    if any(not math.isfinite(weight) or weight < 0.0 for weight in clean.values()):
        raise ValueError("V2 metric weights must be finite and non-negative")
    if not math.isclose(sum(clean.values()), 1.0, rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError(f"V2 metric weights must sum to 1.0, got {sum(clean.values())}")
    return clean


def _baseline_ratio(value: float, baseline: float) -> float:
    """Normalize an RMSE while treating two numerical zeros as equal."""
    denominator = abs(baseline)
    if denominator <= _NORMALIZATION_EPSILON:
        return 1.0 if abs(value) <= _NORMALIZATION_EPSILON else value / _NORMALIZATION_EPSILON
    return value / denominator


def _canonicalize_per_dataset(
    metrics: PerDatasetMetrics,
    horizons_s: tuple[float, ...],
) -> dict[str, dict[float, dict[str, float]]]:
    items = metrics.items() if isinstance(metrics, Mapping) else metrics
    result: dict[str, dict[float, dict[str, float]]] = {}
    required_horizons = {_canonical_horizon(horizon) for horizon in horizons_s}

    for raw_dataset_id, raw_by_h in items:
        dataset_id = str(raw_dataset_id)
        if not dataset_id:
            raise ValueError("V2 dataset_id must not be empty")
        if dataset_id in result:
            raise ValueError(f"duplicate V2 dataset_id: {dataset_id!r}")
        if not isinstance(raw_by_h, Mapping):
            raise ValueError(f"V2 metrics for {dataset_id!r} must be a horizon mapping")

        available = {_canonical_horizon(horizon): values for horizon, values in raw_by_h.items()}
        missing_horizons = sorted(required_horizons - set(available))
        if missing_horizons:
            raise ValueError(f"V2 metrics for {dataset_id!r} lack horizons {missing_horizons}")

        by_h: dict[float, dict[str, float]] = {}
        for horizon in horizons_s:
            canonical_h = _canonical_horizon(horizon)
            raw_values = available[canonical_h]
            if not isinstance(raw_values, Mapping):
                raise ValueError(
                    f"V2 metrics for {dataset_id!r} at {canonical_h}s must be a metric mapping"
                )
            missing_metrics = sorted(set(METRIC_WEIGHTS) - set(raw_values))
            if missing_metrics:
                raise ValueError(
                    f"V2 metrics for {dataset_id!r} at {canonical_h}s lack {missing_metrics}"
                )
            values = {metric: float(raw_values[metric]) for metric in METRIC_WEIGHTS}
            if any(not math.isfinite(value) or value < 0.0 for value in values.values()):
                raise ValueError(
                    f"V2 RMSE values for {dataset_id!r} at {canonical_h}s "
                    "must be finite and non-negative"
                )
            by_h[canonical_h] = values
        result[dataset_id] = by_h

    if not result:
        raise ValueError("V2 evaluation requires at least one dataset")
    return result


def aggregate_model_v2(
    per_dataset_metrics: PerDatasetMetrics,
    baseline_metrics: PerDatasetMetrics,
    *,
    horizons_s: tuple[float, ...] = HORIZONS_S,
    weights: Mapping[str, float] = METRIC_WEIGHTS,
    cvar_tail_fraction: float = CVAR_TAIL_FRACTION,
) -> dict[str, Any]:
    """Aggregate one model with dataset mean and worst-tail CVaR.

    Each RMSE is normalized by the corresponding dataset/baseline RMSE before
    metrics with different physical units are combined.  The scalar score is
    the horizon-average of ``normalized mean + normalized worst-5% CVaR``.
    Raw mean/CVaR values remain in ``by_h`` for diagnosis and reporting.
    """
    if not math.isclose(
        cvar_tail_fraction,
        CVAR_TAIL_FRACTION,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("metrics schema V2 fixes the worst-tail CVaR fraction at 0.05")
    canonical_horizons = tuple(_canonical_horizon(horizon) for horizon in horizons_s)
    clean_weights = _validate_weights(weights)
    candidate = _canonicalize_per_dataset(per_dataset_metrics, canonical_horizons)
    baseline = _canonicalize_per_dataset(baseline_metrics, canonical_horizons)

    candidate_ids = set(candidate)
    baseline_ids = set(baseline)
    if candidate_ids != baseline_ids:
        missing = sorted(baseline_ids - candidate_ids)
        extra = sorted(candidate_ids - baseline_ids)
        raise ValueError(
            "V2 candidate and baseline must use identical datasets; "
            f"missing={missing}, extra={extra}"
        )

    by_h: dict[float, dict[str, Any]] = {}
    horizon_mean_scores: list[float] = []
    horizon_cvar_scores: list[float] = []

    for horizon in canonical_horizons:
        metric_summary: dict[str, dict[str, float | str]] = {}
        score_mean = 0.0
        score_cvar = 0.0
        for metric, weight in clean_weights.items():
            raw_values = [candidate[dataset_id][horizon][metric] for dataset_id in candidate]
            normalized_values = [
                _baseline_ratio(
                    candidate[dataset_id][horizon][metric],
                    baseline[dataset_id][horizon][metric],
                )
                for dataset_id in candidate
            ]
            raw_mean = float(statistics.fmean(raw_values))
            raw_cvar = worst_tail_cvar(raw_values, cvar_tail_fraction)
            normalized_mean = float(statistics.fmean(normalized_values))
            normalized_cvar = worst_tail_cvar(normalized_values, cvar_tail_fraction)
            metric_summary[metric] = {
                "unit": METRIC_UNITS[metric],
                "mean": raw_mean,
                "worst_5pct_cvar": raw_cvar,
                "normalized_mean": normalized_mean,
                "normalized_worst_5pct_cvar": normalized_cvar,
            }
            score_mean += weight * normalized_mean
            score_cvar += weight * normalized_cvar

        horizon_mean_scores.append(score_mean)
        horizon_cvar_scores.append(score_cvar)
        by_h[horizon] = {
            "metrics": metric_summary,
            "score_components": {
                "mean": float(score_mean),
                "worst_5pct_cvar": float(score_cvar),
            },
            "score": float(score_mean + score_cvar),
        }

    mean_component = float(statistics.fmean(horizon_mean_scores))
    cvar_component = float(statistics.fmean(horizon_cvar_scores))
    return {
        "n_datasets": len(candidate),
        "score": float(mean_component + cvar_component),
        "score_components": {
            "mean": mean_component,
            "worst_5pct_cvar": cvar_component,
        },
        "by_h": by_h,
    }


def build_evaluation_v2(
    comparison_metrics: Mapping[str, PerDatasetMetrics],
    *,
    baseline_name: str = "baseline",
    horizons_s: tuple[float, ...] = HORIZONS_S,
    weights: Mapping[str, float] = METRIC_WEIGHTS,
    cvar_tail_fraction: float = CVAR_TAIL_FRACTION,
) -> dict[str, Any]:
    """Build the additive ``evaluation_v2`` YAML value for all models."""
    if baseline_name not in comparison_metrics:
        raise ValueError(f"V2 baseline {baseline_name!r} is absent from comparison metrics")
    clean_weights = _validate_weights(weights)
    canonical_horizons = tuple(_canonical_horizon(horizon) for horizon in horizons_s)
    baseline_metrics = comparison_metrics[baseline_name]

    comparison = {
        str(name): aggregate_model_v2(
            metrics,
            baseline_metrics,
            horizons_s=canonical_horizons,
            weights=clean_weights,
            cvar_tail_fraction=cvar_tail_fraction,
        )
        for name, metrics in comparison_metrics.items()
    }
    return {
        "horizons_s": list(canonical_horizons),
        "weights": clean_weights,
        "aggregation": {
            "dataset_mean": True,
            "tail_statistic": "CVaR",
            "worst_fraction": float(cvar_tail_fraction),
        },
        "normalization": {
            "method": "per_dataset_baseline_ratio",
            "baseline": baseline_name,
            "zero_epsilon": _NORMALIZATION_EPSILON,
        },
        "score_formula": "horizon_mean(weighted_normalized_mean + weighted_normalized_worst_5pct_cvar)",
        "comparison": comparison,
    }


def add_evaluation_v2(
    legacy_document: Mapping[str, Any],
    comparison_metrics: Mapping[str, PerDatasetMetrics],
    *,
    baseline_name: str = "baseline",
) -> dict[str, Any]:
    """Return a copy of a legacy metrics document with only V2 fields added."""
    result = dict(legacy_document)
    result["metrics_schema_version"] = METRICS_SCHEMA_VERSION
    result["evaluation_v2"] = build_evaluation_v2(
        comparison_metrics,
        baseline_name=baseline_name,
    )
    return result


def render_evaluation_v2_html(evaluation_v2: Mapping[str, Any] | None) -> str:
    """Render a compact HTML section that can be placed beside legacy metrics."""
    if not isinstance(evaluation_v2, Mapping):
        return ""
    comparison = evaluation_v2.get("comparison")
    if not isinstance(comparison, Mapping) or not comparison:
        return ""

    weights = evaluation_v2.get("weights") or METRIC_WEIGHTS
    horizons = evaluation_v2.get("horizons_s") or HORIZONS_S
    weight_text = " / ".join(
        f"{html.escape(metric)}={float(weights.get(metric, 0.0)):.2f}" for metric in METRIC_WEIGHTS
    )

    overall_rows: list[str] = []
    detail_rows: list[str] = []
    for raw_name, raw_result in comparison.items():
        if not isinstance(raw_result, Mapping):
            continue
        name = html.escape(str(raw_name))
        components = raw_result.get("score_components") or {}
        overall_rows.append(
            "<tr>"
            f"<td><b>{name}</b></td>"
            f"<td>{float(components.get('mean', float('nan'))):.4f}</td>"
            f"<td>{float(components.get('worst_5pct_cvar', float('nan'))):.4f}</td>"
            f"<td><b>{float(raw_result.get('score', float('nan'))):.4f}</b></td>"
            "</tr>"
        )

        by_h = raw_result.get("by_h") or {}
        for horizon in horizons:
            h_result = by_h.get(horizon, by_h.get(str(horizon), {}))
            if not isinstance(h_result, Mapping):
                continue
            metric_values = h_result.get("metrics") or {}
            cells = []
            for metric in METRIC_WEIGHTS:
                values = metric_values.get(metric, {})
                if not isinstance(values, Mapping):
                    cells.append("<td>N/A</td>")
                    continue
                n_mean = float(values.get("normalized_mean", float("nan")))
                n_cvar = float(values.get("normalized_worst_5pct_cvar", float("nan")))
                raw_mean = float(values.get("mean", float("nan")))
                raw_cvar = float(values.get("worst_5pct_cvar", float("nan")))
                unit = html.escape(str(values.get("unit", METRIC_UNITS[metric])))
                title = html.escape(
                    f"raw mean={raw_mean:.4g} {unit}, raw worst-5% CVaR={raw_cvar:.4g} {unit}",
                    quote=True,
                )
                cells.append(f'<td title="{title}">{n_mean:.3f} / {n_cvar:.3f}</td>')
            score_components = h_result.get("score_components") or {}
            detail_rows.append(
                "<tr>"
                f"<td>{name}</td><td>{float(horizon):g} s</td>"
                + "".join(cells)
                + f"<td>{float(score_components.get('mean', float('nan'))):.3f} / "
                f"{float(score_components.get('worst_5pct_cvar', float('nan'))):.3f}</td>"
                "</tr>"
            )

    if not overall_rows:
        return ""

    metric_headers = "".join(f"<th>{html.escape(metric)}</th>" for metric in METRIC_WEIGHTS)
    return f"""
<section id="evaluation-v2">
<h2>4-1b. Evaluation V2（時間 horizon・mean + worst 5% CVaR）</h2>
<p>
legacy の <code>score</code> / N-step 指標はそのまま残し、V2 を加算表示する。
各成分は同一データセットの baseline RMSE 比で正規化してから重み付けする。
重み: <code>{weight_text}</code>。小さいほど良い。
</p>
<table class="param-table" style="font-size:12px">
  <thead><tr><th>モデル</th><th>mean</th><th>worst 5% CVaR</th><th>V2 score</th></tr></thead>
  <tbody>{"".join(overall_rows)}</tbody>
</table>
<details>
<summary>horizon・成分別（normalized mean / worst 5% CVaR）</summary>
<div style="overflow-x:auto">
<table class="param-table" style="font-size:11px">
  <thead><tr><th>モデル</th><th>horizon</th>{metric_headers}<th>weighted score</th></tr></thead>
  <tbody>{"".join(detail_rows)}</tbody>
</table>
</div>
</details>
</section>
"""


__all__ = [
    "CVAR_TAIL_FRACTION",
    "HORIZONS_S",
    "METRICS_SCHEMA_VERSION",
    "METRIC_WEIGHTS",
    "add_evaluation_v2",
    "aggregate_model_v2",
    "build_evaluation_v2",
    "horizon_steps_from_timestamps",
    "render_evaluation_v2_html",
    "select_time_horizon_metrics",
    "worst_tail_cvar",
]
