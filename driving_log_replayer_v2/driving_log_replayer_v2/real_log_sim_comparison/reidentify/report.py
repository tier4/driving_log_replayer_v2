"""
Render the compact reidentification report from finalized artifacts.

It combines the finalized N-step metrics with physical-validity diagnostics.
The N-step metrics are read from ``fit_merge`` artifacts without rerunning a
rollout; physical-validity diagnostics read the existing CSV cache once.
"""

from __future__ import annotations

from collections.abc import Iterable
from collections.abc import Mapping
import html
import json
import math
from pathlib import Path
from time import perf_counter
from typing import Any

import pandas as pd
import yaml
from plotly.offline import get_plotlyjs

from .. import physical_validity
from .model_config import load_model_config
from .settings import TARGET_MODEL_NAME

REQUIRED_COLUMNS = ("dataset_id", "model", "horizon")
REQUIRED_METRICS = ("pos", "long", "lat", "yaw", "steer", "vx", "ax")
METRIC_UNITS = {
    "pos": "cm",
    "long": "cm",
    "lat": "cm",
    "yaw": "deg",
    "steer": "deg",
    "vx": "m/s",
    "ax": "m/s²",
}
MODEL_ORDER = ("baseline", "tuned")
_ZERO_EPSILON = 1.0e-12
_MODEL_COLORS = ("#667085", "#2563eb", "#16a34a", "#d97706", "#9333ea", "#0891b2", "#e11d48")


def _escape(value: Any) -> str:
    return html.escape(str(value), quote=True)


def _format_number(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(number):
        return "—"
    return f"{number:.6g}"


def _format_param_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return _format_number(value)
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


def _load_tuned_document(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    if not path.is_file():
        message = f"Tuned parameters file not found: {path}"
        raise FileNotFoundError(message)
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, Mapping):
        message = "Tuned parameters YAML must contain a mapping."
        raise TypeError(message)

    document = dict(raw)
    params = document.get("params")
    if not isinstance(params, Mapping):
        message = "Tuned parameters YAML must contain a 'params' mapping."
        raise TypeError(message)
    return document, dict(params)


def _select_metric_columns(frame: pd.DataFrame) -> pd.DataFrame:
    expected = (*REQUIRED_COLUMNS, *REQUIRED_METRICS)
    missing = [column for column in expected if column not in frame]
    if missing:
        message = f"Metrics CSV is missing required columns: {missing}"
        raise ValueError(message)
    if frame.empty:
        message = "Metrics CSV contains no rows."
        raise ValueError(message)
    return frame.loc[:, list(expected)].copy()


def _coerce_metric_values(frame: pd.DataFrame) -> None:
    if frame.loc[:, ["dataset_id", "model"]].isna().any(axis=None):
        message = "Metrics CSV contains an empty dataset_id or model name."
        raise ValueError(message)

    frame["dataset_id"] = frame["dataset_id"].astype(str).str.strip()
    frame["model"] = frame["model"].astype(str).str.strip().str.lower()
    if (frame["dataset_id"] == "").any():
        message = "Metrics CSV contains an empty dataset_id."
        raise ValueError(message)
    if (frame["model"] == "").any():
        message = "Metrics CSV contains an empty model name."
        raise ValueError(message)

    for column in ("horizon", *REQUIRED_METRICS):
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
        values = frame[column].astype(float)
        if not values.map(math.isfinite).all():
            message = f"Metrics CSV column {column!r} contains a non-finite value."
            raise ValueError(message)
    if (frame["horizon"] <= 0.0).any():
        message = "Metrics CSV horizons must be positive."
        raise ValueError(message)
    if (frame.loc[:, REQUIRED_METRICS] < 0.0).any(axis=None):
        message = "Metrics CSV RMSE values must be non-negative."
        raise ValueError(message)


def _validate_metric_coverage(frame: pd.DataFrame, model_order: tuple[str, ...] = MODEL_ORDER) -> None:
    models = set(frame["model"])
    missing_models = [model for model in model_order if model not in models]
    if missing_models:
        message = f"Metrics CSV is missing required models: {missing_models}"
        raise ValueError(message)

    duplicate = frame.duplicated(subset=["dataset_id", "model", "horizon"], keep=False)
    if duplicate.any():
        keys = frame.loc[duplicate, ["dataset_id", "model", "horizon"]].iloc[0].to_dict()
        message = f"Metrics CSV contains duplicate dataset/model/horizon rows: {keys}"
        raise ValueError(message)

    baseline_keys = set(
        frame.loc[frame["model"] == "baseline", ["dataset_id", "horizon"]]
        .itertuples(index=False, name=None)
    )
    for model in model_order:
        model_keys = set(
            frame.loc[frame["model"] == model, ["dataset_id", "horizon"]]
            .itertuples(index=False, name=None)
        )
        if baseline_keys != model_keys:
            missing_model = sorted(baseline_keys - model_keys)[:5]
            missing_baseline = sorted(model_keys - baseline_keys)[:5]
            raise ValueError(
                "Baseline and comparison metrics must cover identical dataset/horizon pairs; "
                f"model={model}, missing_model={missing_model}, missing_baseline={missing_baseline}"
            )


def _load_metrics(path: Path, model_order: tuple[str, ...] = MODEL_ORDER) -> pd.DataFrame:
    if not path.is_file():
        message = f"Metrics CSV not found: {path}"
        raise FileNotFoundError(message)
    frame = _select_metric_columns(pd.read_csv(path))
    _coerce_metric_values(frame)
    _validate_metric_coverage(frame, model_order)

    return frame.sort_values(["horizon", "dataset_id", "model"], kind="stable").reset_index(
        drop=True
    )


def _add_normalized_scores(frame: pd.DataFrame) -> pd.DataFrame:
    baseline = frame.loc[frame["model"] == "baseline"].set_index(["dataset_id", "horizon"])
    result = frame.copy()
    row_scores: list[float] = []
    for row in result.itertuples(index=False):
        key = (row.dataset_id, row.horizon)
        baseline_row = baseline.loc[key]
        ratios = []
        for metric in REQUIRED_METRICS:
            value = float(getattr(row, metric))
            reference = float(baseline_row[metric])
            if abs(reference) <= _ZERO_EPSILON:
                ratio = 1.0 if abs(value) <= _ZERO_EPSILON else value / _ZERO_EPSILON
            else:
                ratio = value / reference
            ratios.append(ratio)
        row_scores.append(float(sum(ratios) / len(ratios)))
    result["normalized_score"] = row_scores
    return result


def _summary_frame(document: Mapping[str, Any], frame: pd.DataFrame, model_order: tuple[str, ...] = MODEL_ORDER) -> pd.DataFrame:
    """Keep aggregate/distribution semantics on the optimization horizons."""
    metadata = document.get("metadata")
    if not isinstance(metadata, Mapping) or "score_horizons" not in metadata:
        return frame
    raw_horizons = metadata["score_horizons"]
    if not isinstance(raw_horizons, (list, tuple)) or not raw_horizons:
        message = "metadata.score_horizons must be a non-empty sequence."
        raise ValueError(message)
    try:
        horizons = {float(value) for value in raw_horizons}
    except (TypeError, ValueError) as error:
        message = "metadata.score_horizons must contain numeric values."
        raise ValueError(message) from error
    if not all(math.isfinite(value) and value > 0.0 for value in horizons):
        message = "metadata.score_horizons must contain finite positive values."
        raise ValueError(message)

    available = set(float(value) for value in frame["horizon"].unique())
    missing = sorted(horizons - available)
    if missing:
        message = f"Metrics CSV is missing configured score horizons: {missing}"
        raise ValueError(message)
    summary = frame.loc[frame["horizon"].isin(horizons)].copy()
    expected_rows = frame["dataset_id"].nunique() * len(model_order) * len(horizons)
    if len(summary) != expected_rows:
        message = "Configured score horizons must exist for every dataset and model."
        raise ValueError(message)
    return summary


def _yaml_score(document: Mapping[str, Any], model: str) -> float | None:
    comparison = document.get("comparison")
    if not isinstance(comparison, Mapping):
        return None
    model_data = comparison.get(model)
    if not isinstance(model_data, Mapping):
        return None
    try:
        score = float(model_data["score"])
    except (KeyError, TypeError, ValueError):
        return None
    return score if math.isfinite(score) else None


def _table(headers: Iterable[str], rows: Iterable[Iterable[Any]], *, css_class: str = "") -> str:
    head = "".join(f"<th>{_escape(header)}</th>" for header in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{_escape(cell)}</td>" for cell in row) + "</tr>"
        for row in rows
    )
    class_attr = f' class="{_escape(css_class)}"' if css_class else ""
    return f'<div class="table-wrap"><table{class_attr}><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def _format_constraint_range(constraint: Mapping[str, Any]) -> str:
    allowed = constraint.get("allowed_range")
    if not isinstance(allowed, Mapping):
        return "-"
    minimum = allowed.get("minimum")
    maximum = allowed.get("maximum")
    left = "[" if allowed.get("minimum_inclusive", True) else "("
    right = "]" if allowed.get("maximum_inclusive", True) else ")"
    lower = "-∞" if minimum is None else _format_param_value(minimum)
    upper = "∞" if maximum is None else _format_param_value(maximum)
    return f"{left}{lower}, {upper}{right}"


def _render_params(document: Mapping[str, Any], params: Mapping[str, Any]) -> str:
    constraints = document.get("parameter_constraints")
    if not isinstance(constraints, Mapping):
        rows = ((name, _format_param_value(params[name])) for name in sorted(params))
        return _table(("Parameter", "Tuned value"), rows, css_class="params")
    rows = []
    for name in sorted(params):
        constraint = constraints.get(name)
        if not isinstance(constraint, Mapping):
            rows.append((name, _format_param_value(params[name]), "-", "-", "-"))
            continue
        search = constraint.get("search_candidates", "continuous")
        rows.append((
            name,
            _format_param_value(params[name]),
            _format_constraint_range(constraint),
            _format_param_value(search),
            constraint.get("reason", "-"),
        ))
    return _table(
        ("Parameter", "Tuned value", "Allowed range", "Search candidates", "Reason"),
        rows,
        css_class="params",
    )


def _render_aggregate(document: Mapping[str, Any], frame: pd.DataFrame, model_order: tuple[str, ...] = MODEL_ORDER) -> str:
    rows = []
    for model in model_order:
        model_rows = frame.loc[frame["model"] == model]
        yaml_score = _yaml_score(document, model)
        derived_score = float(model_rows["normalized_score"].mean())
        score = yaml_score if yaml_score is not None else derived_score
        score_source = (
            f"tuned_params.yaml: comparison.{model}.score"
            if yaml_score is not None
            else "normalized RMSE ratio"
        )
        # ``fit_merge`` replaces only ``current`` (shown as ``tuned``) with the
        # fitted parameters.  Every other comparison model is evaluated with its
        # declared scenario parameters, including released models such as v1.
        parameter_source = (
            "tuned_params.yaml: params"
            if model == "tuned"
            else f"scenario.yaml: models.{model}.params (fixed)"
        )
        rows.append(
            (
                model,
                _format_number(score),
                score_source,
                parameter_source,
                _format_number(derived_score),
                *(_format_number(model_rows[metric].mean()) for metric in REQUIRED_METRICS),
            )
        )
    return _table(
        (
            "Model",
            "Aggregate score",
            "Score source",
            "Parameter source",
            "Mean normalized RMSE",
            *REQUIRED_METRICS,
        ),
        rows,
    )


def _x_ticks(lower: float, upper: float, *, max_intervals: int = 10) -> list[float]:
    """Return readable numeric ticks while preserving every plotted data point."""
    if math.isclose(lower, upper, rel_tol=0.0, abs_tol=_ZERO_EPSILON):
        return [lower]
    raw_step = (upper - lower) / max_intervals
    magnitude = 10.0 ** math.floor(math.log10(raw_step))
    normalized = raw_step / magnitude
    if normalized <= 1.0:
        multiplier = 1.0
    elif normalized <= 2.0:
        multiplier = 2.0
    elif normalized <= 5.0:
        multiplier = 5.0
    else:
        multiplier = 10.0
    step = multiplier * magnitude

    ticks = [lower]
    value = math.ceil((lower - _ZERO_EPSILON) / step) * step
    while value <= upper + _ZERO_EPSILON:
        if not math.isclose(value, ticks[-1], rel_tol=0.0, abs_tol=_ZERO_EPSILON):
            ticks.append(value)
        value += step
    if not math.isclose(ticks[-1], upper, rel_tol=0.0, abs_tol=_ZERO_EPSILON):
        ticks.append(upper)
    return ticks


def _line_chart_svg(frame: pd.DataFrame, metric: str, model_order: tuple[str, ...] = MODEL_ORDER) -> str:
    means = (
        frame.loc[frame["model"].isin(model_order)]
        .groupby(["horizon", "model"], sort=True)[metric]
        .mean()
    )
    horizons = sorted(float(value) for value in means.index.get_level_values("horizon").unique())
    values = [float(means.loc[(horizon, model)]) for horizon in horizons for model in model_order]

    svg_width = 960
    legend_columns = 4
    legend_rows = math.ceil(len(model_order) / legend_columns)
    svg_height = 310 + max(0, legend_rows - 1) * 18
    left, right, top, bottom = 68, 24, 48 + max(0, legend_rows - 1) * 18, 52
    chart_width = svg_width - left - right
    chart_height = svg_height - top - bottom

    lower_horizon = horizons[0]
    upper_horizon = horizons[-1]
    horizon_span = upper_horizon - lower_horizon

    upper = max(values)
    upper = upper * 1.05 if upper > _ZERO_EPSILON else 1.0

    def x_position(horizon: float) -> float:
        if horizon_span <= _ZERO_EPSILON:
            return left + chart_width / 2
        return left + (horizon - lower_horizon) * chart_width / horizon_span

    def y_position(value: float) -> float:
        return top + chart_height * (1.0 - value / upper)

    grid = []
    for index in range(5):
        value = upper * index / 4
        y = y_position(value)
        grid.append(
            f'<line class="grid" x1="{left}" x2="{left + chart_width}" '
            f'y1="{y:.2f}" y2="{y:.2f}" />'
            f'<text class="tick y-tick" x="{left - 9}" y="{y + 4:.2f}">'
            f'{_format_number(value)}</text>'
        )

    x_ticks = []
    for horizon in _x_ticks(lower_horizon, upper_horizon):
        x = x_position(horizon)
        x_ticks.append(
            f'<line class="tick-mark" x1="{x:.2f}" x2="{x:.2f}" '
            f'y1="{top + chart_height}" y2="{top + chart_height + 5}" />'
            f'<text class="tick x-tick" x="{x:.2f}" y="{top + chart_height + 20}">'
            f'{_format_number(horizon)}</text>'
        )

    series = []
    point_radius = 4 if len(horizons) <= 30 else 2.5
    for index, model in enumerate(model_order):
        points = [
            (horizon, float(means.loc[(horizon, model)]))
            for horizon in horizons
        ]
        coordinates = " ".join(
            f"{x_position(horizon):.2f},{y_position(value):.2f}"
            for horizon, value in points
        )
        circles = "".join(
            f'<circle class="point {model}" cx="{x_position(horizon):.2f}" '
            f'cy="{y_position(value):.2f}" r="{point_radius}"><title>'
            f'{_escape(metric)} — {_escape(model)} — N {_format_number(horizon)}: '
            f'mean RMSE {_format_number(value)}</title></circle>'
            for horizon, value in points
        )
        series.append(
            f'<polyline class="series {model}" style="--series-color:{_MODEL_COLORS[index % len(_MODEL_COLORS)]}" points="{coordinates}" />'
            + circles.replace(f'class="point {model}"', f'class="point {model}" style="--series-color:{_MODEL_COLORS[index % len(_MODEL_COLORS)]}"')
        )

    legend_x = left + chart_width - 4 * 110
    legend = "".join(
        f'<g class="legend {model}" style="--series-color:{_MODEL_COLORS[index % len(_MODEL_COLORS)]}"><line x1="{legend_x + (index % legend_columns) * 110}" '
        f'x2="{legend_x + 26 + (index % legend_columns) * 110}" y1="{20 + (index // legend_columns) * 18}" y2="{20 + (index // legend_columns) * 18}" />'
        f'<circle cx="{legend_x + 13 + (index % legend_columns) * 110}" cy="{20 + (index // legend_columns) * 18}" r="4" />'
        f'<text x="{legend_x + 32 + (index % legend_columns) * 110}" y="{24 + (index // legend_columns) * 18}">{_escape(model)}</text></g>'
        for index, model in enumerate(model_order)
    )

    unit = METRIC_UNITS[metric]
    return (
        '<div class="chart-wrap">'
        f'<svg class="horizon-chart" role="img" aria-label="{_escape(metric)} mean RMSE ({_escape(unit)}) by horizon N" '
        f'width="{svg_width}" height="{svg_height}" viewBox="0 0 {svg_width} {svg_height}">'
        f'<title>{_escape(metric)} mean RMSE ({_escape(unit)}) by horizon N</title>'
        + "".join(grid)
        + f'<line class="axis" x1="{left}" x2="{left + chart_width}" '
        f'y1="{top + chart_height}" y2="{top + chart_height}" />'
        + f'<line class="axis" x1="{left}" x2="{left}" y1="{top}" '
        f'y2="{top + chart_height}" />'
        + "".join(x_ticks)
        + "".join(series)
        + legend
        + f'<text class="axis-title" x="{left + chart_width / 2:.2f}" y="{svg_height - 8}">'
        "N (horizon)</text>"
        + f'<text class="axis-title" transform="translate(17 {top + chart_height / 2:.2f}) rotate(-90)">'
        f"mean RMSE ({_escape(unit)})</text></svg></div>"
    )


def _render_horizon_charts(frame: pd.DataFrame, model_order: tuple[str, ...] = MODEL_ORDER) -> str:
    return "".join(
        f'<article class="metric-chart"><h3>{_escape(metric)} RMSE '
        f'({_escape(METRIC_UNITS[metric])})</h3>'
        f'{_line_chart_svg(frame, metric, model_order)}</article>'
        for metric in REQUIRED_METRICS
    )


def _dataset_scores(frame: pd.DataFrame) -> pd.DataFrame:
    return (
        frame.groupby(["dataset_id", "model"], sort=True, as_index=False)["normalized_score"]
        .mean()
        .pivot_table(
            index="dataset_id",
            columns="model",
            values="normalized_score",
            aggfunc="first",
        )
        .sort_index()
    )


def _distribution_summary(dataset_scores: pd.DataFrame, model_order: tuple[str, ...] = MODEL_ORDER) -> str:
    rows = []
    for model in model_order:
        values = dataset_scores[model]
        rows.append(
            (
                model,
                len(values),
                _format_number(values.min()),
                _format_number(values.quantile(0.25)),
                _format_number(values.median()),
                _format_number(values.quantile(0.75)),
                _format_number(values.quantile(0.90)),
                _format_number(values.max()),
            )
        )
    return _table(("Model", "Datasets", "Min", "P25", "Median", "P75", "P90", "Max"), rows)


def _histogram_svg(values: Iterable[float]) -> str:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return ""

    n_bins = min(12, max(4, math.ceil(math.sqrt(len(clean)))))
    lower = min(clean)
    upper = max(clean)
    if math.isclose(lower, upper, rel_tol=0.0, abs_tol=1.0e-12):
        pad = max(abs(lower) * 0.05, 0.05)
        lower -= pad
        upper += pad
    width = (upper - lower) / n_bins
    counts = [0] * n_bins
    for value in clean:
        index = min(n_bins - 1, int((value - lower) / width))
        counts[index] += 1

    svg_width, svg_height = 760, 230
    left, right, top, bottom = 54, 18, 18, 46
    chart_width = svg_width - left - right
    chart_height = svg_height - top - bottom
    max_count = max(counts)
    slot = chart_width / n_bins
    bars = []
    for index, count in enumerate(counts):
        bar_height = chart_height * count / max_count
        x = left + index * slot + 2
        y = top + chart_height - bar_height
        bars.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{max(1.0, slot - 4):.2f}" '
            f'height="{bar_height:.2f}" rx="2"><title>{count} datasets</title></rect>'
        )

    reference = ""
    if lower <= 1.0 <= upper:
        ref_x = left + (1.0 - lower) / (upper - lower) * chart_width
        reference = (
            f'<line class="reference" x1="{ref_x:.2f}" x2="{ref_x:.2f}" '
            f'y1="{top}" y2="{top + chart_height}" />'
            f'<text class="label" x="{ref_x + 4:.2f}" y="{top + 12}">baseline ratio 1.0</text>'
        )

    return (
        f'<svg class="histogram" role="img" aria-label="Tuned normalized score distribution" '
        f'viewBox="0 0 {svg_width} {svg_height}">'
        f'<line class="axis" x1="{left}" x2="{left + chart_width}" y1="{top + chart_height}" '
        f'y2="{top + chart_height}" />'
        + "".join(bars)
        + reference
        + f'<text class="tick" x="{left}" y="{svg_height - 18}">{_format_number(lower)}</text>'
        + f'<text class="tick end" x="{left + chart_width}" y="{svg_height - 18}">{_format_number(upper)}</text>'
        + f'<text class="axis-title" x="{left + chart_width / 2:.2f}" y="{svg_height - 4}">'
        "mean normalized RMSE (lower is better)</text></svg>"
    )


def _render_dataset_distribution(frame: pd.DataFrame, model_order: tuple[str, ...] = MODEL_ORDER) -> str:
    scores = _dataset_scores(frame)
    target = "tuned" if "tuned" in scores else model_order[-1]
    worst = scores.sort_values(target, ascending=False).head(10)
    worst_rows = (
        (dataset_id, _format_number(row[target]))
        for dataset_id, row in worst.iterrows()
    )
    return (
        _distribution_summary(scores, model_order)
        + _histogram_svg(scores[target])
        + f"<h3>Highest {_escape(target)} dataset scores</h3>"
        + _table(("Dataset", f"{target} normalized score"), worst_rows)
    )


def _render_failures(summary: Mapping[str, Any]) -> str:
    count_keys = ("n_datasets", "n_cached", "n_extracted", "n_skipped", "n_failed")
    counts = {key: summary[key] for key in count_keys if key in summary}
    count_labels = {
        "n_datasets": "Discovered",
        "n_cached": "Cached",
        "n_extracted": "Extracted",
        "n_skipped": "Skipped",
        "n_failed": "Failed",
    }
    cards = "".join(
        f'<div class="stat"><span>{_escape(count_labels[key])}</span><strong>{_escape(value)}</strong></div>'
        for key, value in counts.items()
    )
    return f'<div class="stats">{cards}</div>'


def _load_artifact(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        return {}
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return raw if isinstance(raw, Mapping) else {}


def _render_artifact(path: Path, *, parameter_title: str = "Parameters") -> str:
    document = _load_artifact(path)
    params = document.get("params") if isinstance(document.get("params"), Mapping) else {}
    metadata = document.get("metadata") if isinstance(document.get("metadata"), Mapping) else {}
    source = f'<p class="source">Artifact: {_escape(path)}</p>'
    param_table = _render_params(document, params) if params else '<p class="note">No parameter summary available.</p>'
    metadata_rows = ((key, _format_param_value(value)) for key, value in sorted(metadata.items()))
    return f'<h3>{_escape(parameter_title)}</h3>{source}{param_table}'


def _comparison_model_order(scenario: Path) -> tuple[str, ...]:
    if not scenario.is_file():
        return MODEL_ORDER
    config = load_model_config(scenario)
    return tuple(
        ("tuned" if name == TARGET_MODEL_NAME else name).lower()
        for name in config.comparison_models
    )


def _render_document(
    tuned_path: Path,
    metrics_path: Path,
    document: Mapping[str, Any],
    params: Mapping[str, Any],
    frame: pd.DataFrame,
    failures: Mapping[str, Any],
    physical_sections: physical_validity.PhysicalValiditySections,
    model_order: tuple[str, ...],
    extraction_summary: Mapping[str, Any],
    phase1_path: Path,
    phase2_path: Path,
    phase3_path: Path,
    release_path: Path,
) -> str:
    datasets = frame["dataset_id"].nunique()
    horizons = frame["horizon"].nunique()
    summary = _summary_frame(document, frame, model_order)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Reidentification and physical-validity report</title>
<style>
:root {{ color-scheme: light; --ink:#172033; --muted:#667085; --line:#d8dee9; --paper:#fff; --wash:#f5f7fa; --accent:#2563eb; --bad:#b42318; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--wash); color:var(--ink); font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }}
main {{ width:min(1180px,calc(100% - 32px)); margin:32px auto 64px; }}
header,section {{ background:var(--paper); border:1px solid var(--line); border-radius:10px; padding:20px 24px; margin-bottom:16px; }}
h1 {{ margin:0 0 6px; font-size:26px; }} h2 {{ margin:0 0 14px; font-size:19px; }} h3 {{ margin:20px 0 8px; font-size:15px; }}
.lede,.source,.note {{ color:var(--muted); }} .source {{ margin:3px 0; overflow-wrap:anywhere; }}
.stats {{ display:flex; flex-wrap:wrap; gap:10px; margin:10px 0 14px; }}
.stat {{ min-width:120px; padding:10px 12px; background:var(--wash); border-radius:8px; }}
.stat span {{ display:block; color:var(--muted); font-size:12px; }} .stat strong {{ font-size:20px; }}
.table-wrap {{ width:100%; overflow-x:auto; }} table {{ width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }}
th,td {{ border-bottom:1px solid var(--line); padding:8px 10px; text-align:right; white-space:nowrap; }}
th {{ color:var(--muted); background:var(--wash); font-size:12px; }} th:first-child,td:first-child {{ text-align:left; }}
.params td:nth-child(2) {{ text-align:left; white-space:normal; overflow-wrap:anywhere; }}
.histogram {{ display:block; width:min(760px,100%); margin:18px auto 8px; }}
.histogram rect {{ fill:var(--accent); opacity:.82; }} .histogram .axis {{ stroke:#98a2b3; }}
.histogram .reference {{ stroke:var(--bad); stroke-width:1.5; stroke-dasharray:5 4; }}
.histogram text {{ fill:var(--muted); font-size:11px; }} .histogram .end {{ text-anchor:end; }} .histogram .axis-title {{ text-anchor:middle; }}
.metric-chart {{ margin-top:22px; }} .metric-chart:first-of-type {{ margin-top:0; }}
.chart-wrap {{ width:100%; overflow-x:auto; }}
.horizon-chart {{ display:block; margin:0 auto 8px; }}
.horizon-chart .axis,.horizon-chart .tick-mark {{ stroke:#98a2b3; }}
.horizon-chart .grid {{ stroke:#e4e7ec; }}
.horizon-chart text {{ fill:var(--muted); font-size:11px; }}
.horizon-chart .x-tick,.horizon-chart .axis-title {{ text-anchor:middle; }}
.horizon-chart .y-tick {{ text-anchor:end; }}
.horizon-chart .series {{ fill:none; stroke:var(--series-color); stroke-width:2; stroke-linejoin:round; stroke-linecap:round; }}
.horizon-chart .point,.horizon-chart .legend circle {{ fill:var(--series-color); }}
.horizon-chart .legend line {{ stroke:var(--series-color); }}
.horizon-chart .legend line {{ stroke-width:2; }} .horizon-chart .legend text {{ fill:var(--series-color); }}
.ok {{ color:#067647; }}
.plotly-graph-div {{ max-width:100%; }}
</style>
<script>{get_plotlyjs()}</script>
</head>
<body><main>
<header>
  <h1>Reidentification and physical-validity report</h1>
  <p class="lede">N-step metrics are finalized artifacts; physical-validity diagnostics use the existing CSV cache.</p>
  <div class="stats"><div class="stat"><span>Valid datasets</span><strong>{datasets}</strong></div><div class="stat"><span>Horizons</span><strong>{horizons}</strong></div></div>
  <p class="source">Parameters: {_escape(tuned_path)}</p>
  <p class="source">Metrics: {_escape(metrics_path)}</p>
</header>
<section><h2>1. Extraction results</h2>{_render_failures(extraction_summary)}{physical_sections.prepare}</section>
<section><h2>2. Longitudinal direct identification</h2>{_render_artifact(phase1_path, parameter_title="phase1_acc.yaml")}{physical_sections.longitudinal}</section>
<section><h2>3. Steering direct identification</h2>{_render_artifact(phase2_path, parameter_title="phase2_steer.yaml")}{physical_sections.steering}{physical_sections.yaw}</section>
<section><h2>4. XY heading-rate direct identification</h2>{_render_artifact(phase3_path, parameter_title="phase3_xy.yaml")}{physical_sections.xy}</section>
<section><h2>5. Integrated optimization</h2>{_render_artifact(tuned_path, parameter_title="Final parameters")}<h3>Aggregate comparison</h3><p class="note">Raw columns are mean RMSE. Aggregate and distribution values use the configured optimization horizons; graphs use every available N. Lower is better. Score source identifies the stored aggregate score; parameter source identifies the parameters used for rollout evaluation.</p>{_render_aggregate(document, summary, model_order)}<h3>Error by horizon N</h3><p class="note">Each point is the mean RMSE across valid datasets. Every available N is plotted; lower is better.</p>{_render_horizon_charts(frame, model_order)}<h3>Dataset distributions</h3>{_render_dataset_distribution(summary, model_order)}</section>
<section><h2>6. Released YAML</h2><p class="source">Released parameter YAML: {_escape(release_path)}</p></section>
</main></body></html>
"""


def run(
    tuned_params: Path | str,
    metrics_csv: Path | str,
    out: Path | str,
    *,
    failures: Mapping[str, Any],
    collection_dir: Path | str,
    scenario: Path | str,
    n_jobs: int = 1,
    extraction_summary: Mapping[str, Any] | None = None,
    phase1_params: Path | str | None = None,
    phase2_params: Path | str | None = None,
    phase3_params: Path | str | None = None,
    release_params: Path | str | None = None,
) -> Path:
    """Generate the unified ``report.html``."""
    tuned_path = Path(tuned_params)
    metrics_path = Path(metrics_csv)
    out_path = Path(out)
    started = perf_counter()
    document, params = _load_tuned_document(tuned_path)
    scenario_path = Path(scenario)
    model_order = _comparison_model_order(scenario_path)
    frame = _add_normalized_scores(_load_metrics(metrics_path, model_order))
    metrics_elapsed = perf_counter() - started
    physical_started = perf_counter()
    physical_sections = physical_validity.build_sections(
        Path(collection_dir), tuned_path, scenario=scenario_path, n_jobs=n_jobs
    )
    physical_elapsed = perf_counter() - physical_started
    render_started = perf_counter()
    rendered = _render_document(
        tuned_path,
        metrics_path,
        document,
        params,
        frame,
        failures,
        physical_sections,
        model_order,
        extraction_summary if extraction_summary is not None else failures,
        Path(phase1_params) if phase1_params is not None else tuned_path,
        Path(phase2_params) if phase2_params is not None else tuned_path,
        Path(phase3_params) if phase3_params is not None else tuned_path,
        Path(release_params) if release_params is not None else tuned_path,
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(rendered, encoding="utf-8")
    print(
        "[report] complete: "
        f"metrics={metrics_elapsed:.1f}s physical-validity={physical_elapsed:.1f}s "
        f"html={perf_counter() - render_started:.1f}s total={perf_counter() - started:.1f}s"
    )
    return out_path
