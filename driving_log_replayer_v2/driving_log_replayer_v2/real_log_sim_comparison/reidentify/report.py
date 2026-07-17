"""
Render the compact reidentification report from finalized artifacts.

It combines the finalized N-step metrics with physical-validity diagnostics.
The N-step metrics are read from ``fit_merge`` artifacts without rerunning a
rollout; physical-validity diagnostics read the existing CSV cache once.
"""

from __future__ import annotations

from collections.abc import Iterable
from collections.abc import Mapping
import json
import math
from pathlib import Path
from time import perf_counter
from typing import Any

import pandas as pd
from plotly.offline import get_plotlyjs
import yaml

from .model_config import comparison_display_order
from .model_config import load_model_config
from .release_params import GLOBAL_PARAM_KEYS
from .settings import BASELINE_MODEL_NAME
from .settings import RELEASE_MODEL_KEY
from .settings import TUNED_MODEL_DISPLAY_NAME
from .. import physical_validity
from ..lib._nstep_common import METRIC_KEYS
from ..lib._report_format import escape as _escape
from ..lib._report_format import format_number as _format_number
from ..lib._report_format import html_table as _table

REQUIRED_COLUMNS = ("dataset_id", "model", "horizon")
REQUIRED_METRICS = METRIC_KEYS
METRIC_UNITS = {
    "pos": "cm",
    "long": "cm",
    "lat": "cm",
    "yaw": "deg",
    "steer": "deg",
    "vx": "m/s",
    "ax": "m/s²",
}
MODEL_ORDER = (BASELINE_MODEL_NAME, TUNED_MODEL_DISPLAY_NAME)
_ZERO_EPSILON = 1.0e-12
_MODEL_COLORS = ("#667085", "#2563eb", "#16a34a", "#d97706", "#9333ea", "#0891b2", "#e11d48")

# 数式描画は MathJax (tex-svg) を CDN 参照する。plotly.js はオフライン用にインライン埋め込みだが、
# 数式は CDN 配信(閲覧時にインターネット接続が必要)。旧 physical_validity_report と同方式。
_MATHJAX_HEAD = (
    "<script>window.MathJax={tex:{inlineMath:[['\\\\(','\\\\)']],"
    "displayMath:[['\\\\[','\\\\]']]},svg:{fontCache:'global'}};</script>"
    "<script async src='https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js'></script>"
)


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


def _validate_metric_coverage(frame: pd.DataFrame, model_order: tuple[str, ...]) -> None:
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
        frame.loc[frame["model"] == BASELINE_MODEL_NAME, ["dataset_id", "horizon"]]
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


def _load_metrics(path: Path, model_order: tuple[str, ...]) -> pd.DataFrame:
    if not path.is_file():
        message = f"Metrics CSV not found: {path}"
        raise FileNotFoundError(message)
    frame = _select_metric_columns(pd.read_csv(path))
    _coerce_metric_values(frame)
    _validate_metric_coverage(frame, model_order)

    # metrics.csv が model_order の上位集合を含む場合 (scenario で比較集合を減らした後の
    # 再レポートや --report-only の再利用) に備え、描画対象を model_order に限定する。
    frame = frame.loc[frame["model"].isin(model_order)]
    return frame.sort_values(["horizon", "dataset_id", "model"], kind="stable").reset_index(
        drop=True
    )


def _add_normalized_scores(frame: pd.DataFrame) -> pd.DataFrame:
    baseline = frame.loc[frame["model"] == BASELINE_MODEL_NAME].set_index(["dataset_id", "horizon"])
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


def _summary_frame(document: Mapping[str, Any], frame: pd.DataFrame, model_order: tuple[str, ...]) -> pd.DataFrame:
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


def _score_cell(score: float, baseline_score: float | None, *, is_baseline: bool) -> str:
    """Color the aggregate-score cell by baseline ratio (green=better, red=worse)."""
    text = _format_number(score)
    if is_baseline or baseline_score is None or not math.isfinite(score) or not math.isfinite(baseline_score):
        return f"<td>{text}</td>"
    cls = "score-good" if score < baseline_score else "score-bad" if score > baseline_score else ""
    return f'<td><span class="{cls}">{text}</span></td>' if cls else f"<td>{text}</td>"


def _render_aggregate(document: Mapping[str, Any], frame: pd.DataFrame, model_order: tuple[str, ...]) -> str:
    def model_score(model: str) -> float:
        yaml_score = _yaml_score(document, model)
        derived = float(frame.loc[frame["model"] == model, "normalized_score"].mean())
        return yaml_score if yaml_score is not None else derived

    baseline_score = model_score(BASELINE_MODEL_NAME) if BASELINE_MODEL_NAME in model_order else None
    headers = ("Model", "Aggregate score", "Score source", "Parameter source", "Mean normalized RMSE", *REQUIRED_METRICS)
    head = "".join(f"<th>{_escape(h)}</th>" for h in headers)
    body = ""
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
            if model == TUNED_MODEL_DISPLAY_NAME
            else f"scenario.yaml: models.{model}.params (fixed)"
        )
        cells = (
            f"<td>{_escape(model)}</td>"
            + _score_cell(score, baseline_score, is_baseline=model == BASELINE_MODEL_NAME)
            + f"<td>{_escape(score_source)}</td><td>{_escape(parameter_source)}</td>"
            + f"<td>{_escape(_format_number(derived_score))}</td>"
            + "".join(f"<td>{_escape(_format_number(model_rows[metric].mean()))}</td>" for metric in REQUIRED_METRICS)
        )
        body += f"<tr>{cells}</tr>"
    return f'<div class="table-wrap"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


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


def _line_chart_svg(frame: pd.DataFrame, metric: str, model_order: tuple[str, ...]) -> str:
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


def _render_horizon_charts(frame: pd.DataFrame, model_order: tuple[str, ...]) -> str:
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


def _distribution_summary(dataset_scores: pd.DataFrame, model_order: tuple[str, ...]) -> str:
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


def _render_dataset_distribution(frame: pd.DataFrame, model_order: tuple[str, ...]) -> str:
    scores = _dataset_scores(frame)
    target = TUNED_MODEL_DISPLAY_NAME if TUNED_MODEL_DISPLAY_NAME in scores else model_order[-1]
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


def _render_artifact_document(document: Mapping[str, Any], path: Path, *, parameter_title: str = "Parameters") -> str:
    params = document.get("params") if isinstance(document.get("params"), Mapping) else {}
    source = f'<p class="source">Artifact: {_escape(path)}</p>'
    param_table = _render_params(document, params) if params else '<p class="note">No parameter summary available.</p>'
    return f'<h3>{_escape(parameter_title)}</h3>{source}{param_table}'


def _render_artifact(path: Path, *, parameter_title: str = "Parameters") -> str:
    return _render_artifact_document(_load_artifact(path), path, parameter_title=parameter_title)


def _render_stage_plateau(path: Path, *, scale_key: str, unit: str, channel: str) -> str:
    """
    Render the in-stage plateau scaling identification for sections 3/4.

    チャネル固有の方法論ノート (常時描画) と、phase 成果物 metadata に plateau_scale が
    記録されていれば採用値サマリを付す。fit_lon / fit_steer は τ/delay 確定後に scaling を
    プラトー RMSE 最小化で決め直し、metadata に plateau_scale / dynamic_scale /
    before-after RMSE を記録する。プラトー特性・2 段構成の共有理論は 1 章 (#plateau-theory)。
    """
    common = (
        '<h3>スケーリングのプラトー同定 (τ/むだ時間 固定)</h3>'
        f'<p>{"ax" if channel == "ax" else "steer"} の open-loop 誤差は '
        f'N≈{9 if channel == "ax" else 20} で定常値に飽和する'
        '(<a href="#plateau-theory">プラトー特性・1 章</a>)。τ/むだ時間の確定後、'
        f'<code>{_escape(scale_key)}</code> は N-step rollout のプラトー mean RMSE を'
        '最小化して決定する (共通コア fit_scaling_channels)。動的励起マスク上の同時推定値は'
        f'τ/むだ時間の同定精度のために計算するが採用しない。GT は '
        f'{"kinematic_savgol" if channel == "ax" else "steer_savgol"}。</p>'
    )
    if channel == "steer":
        common += (
            '<div class="note"><b>steer_bias は同定対象に含めない</b>: モデル構造上 steer 状態は'
            ' steer_des × scaling に収束し、bias はヨーレート計算にのみ入るため、プラトー目的関数に'
            '対して<b>平坦 (同定不能)</b>。bias の同定は yaw を見る統合最適化 (fit_merge) に委ねる。</div>'
            '<p><b>steer_dead_band の判定</b>: dead_band (現在 0 固定) の探索解禁は「scaling/bias 補正後も'
            ' per-dataset |steer_mean| が残存し、かつ残差が低操舵振幅域に集中する」場合のみとする。'
            '2026-07 分析 (318 datasets) では |steer_mean| 中央値 ≈ 0.06° ≪ プラトー RMSE 0.35–0.41° で'
            '非系統成分支配のため、0 固定を維持。</p>'
        )
    metadata = _load_artifact(path).get("metadata")
    if not isinstance(metadata, Mapping) or "plateau_scale" not in metadata:
        return common
    try:
        plateau_scale = float(metadata["plateau_scale"])
        dynamic_scale = float(metadata["dynamic_scale"])
        rmse_initial = float(metadata["plateau_rmse_initial"])
        rmse_final = float(metadata["plateau_rmse_final"])
        n_valid = int(metadata["plateau_n_valid"])
    except (KeyError, TypeError, ValueError):
        return common
    improvement = (
        f" ({1 - rmse_final / rmse_initial:+.1%})" if rmse_initial > 0.0 else ""
    )
    return common + f"""<div class="stats">
<div class="stat"><span>採用値 (plateau)</span><strong>{plateau_scale:.4f}</strong></div>
<div class="stat"><span>動的フィット値 (不採用)</span><strong>{dynamic_scale:.4f}</strong></div>
<div class="stat"><span>plateau mean RMSE</span><strong>{rmse_initial:.4f} → {rmse_final:.4f} {_escape(unit)}{improvement}</strong></div>
<div class="stat"><span>有効 dataset</span><strong>{n_valid}</strong></div>
</div>"""


def _render_lon_brake_split(
    frame: pd.DataFrame, model_order: tuple[str, ...], analysis_dir: Path,
) -> str:
    """3 章: 加減速分離 (brake split) の構造・同定方法論・効果を描画する。

    解説と方程式は静的、ax の N-step 比較表は metrics.csv (frame) から、レジーム別
    実測表は make analyze の成果物 (analysis/regime_metrics.csv) があれば描画する。
    """
    parts: list[str] = ['<h3 id="brake-split">加減速分離 (brake split)</h3>']
    parts.append(
        '<p>実車の縦系応答は加速と減速で異なる (レジーム分離同定 2026-07-17)。'
        '加速度チャネルの一次遅れは、<b>遅延済み生指令の符号</b>で時定数とゲインを切り替える:</p>'
        '<div class="eq-block">\\[\\dot a_{ped} = -\\frac{a_{ped}(t{-}d_a) - K\\,u_a(t{-}d_a)}{\\tau},'
        '\\qquad (K,\\tau)=\\begin{cases}(K_{acc},\\ \\tau_{acc}) & u_a^{raw}(t{-}d_a)\\ge 0\\\\'
        '(K_{brk},\\ \\tau_{brk}) & u_a^{raw}(t{-}d_a)<0\\end{cases}\\]</div>'
        '<p><code>brake_scaling_factor</code> / <code>brake_time_constant</code> が '
        '\\(K_{brk}, \\tau_{brk}\\)。<b>&le;0 はセンチネル</b>で対称値 '
        '(\\(K_{acc}, \\tau_{acc}\\)) を継承し、v2 と bit 一致に退化する。</p>'
    )
    parts.append(
        '<h3>同定方法論: レジーム分離 (2026-07-17)</h3>'
        '<p>集約目的 (robust_score や全シーン ax RMS) は coast/throttle 窓が支配的で brake 側の'
        '効果が希釈される。そこで per-start 窓を窓平均指令で brake (&lt; &minus;0.3 m/s²) / coast / '
        'throttle (&gt; +0.3 m/s²) に分類し、<b>各レジームの ax N-step RMS を独立に最小化</b>した。</p>'
        '<ul>'
        '<li><b>v2 の実態</b>: ax 誤差は brake レジームに集中 (RMS が coast の約 2 倍、'
        'バイアス +0.11〜0.15 m/s² = sim が過減速)。throttle/coast はバイアス &asymp;0。</li>'
        '<li><b>throttle 側</b>: レジーム内再同定は &minus;6.7% を示すが、グローバルには ax &plusmn;0% で'
        'テールを微劣化させる (SG 平滑 GT への過適合、v2_t と同型) — <b>v2 値を維持</b>。</li>'
        '<li><b>brake 側</b>: 定常マップ実測ゲイン &asymp;0.7 の単独適用は N=30 の long/pos を'
        '+13〜19% 破綻させる (応答の形が崩れる)。<b>ゲインと τ の同時同定が必須</b>で、'
        '位置追従と両立する境界 \\(K_{brk}=0.84,\\ \\tau_{brk}=0.60\\) を採用 '
        '(全 318 dataset で score-horizon 全セル mean/cvar +2% 非劣化 PASS)。</li>'
        '<li><b>GT アーティファクトの示唆</b>: ax GT (kinematic_savgol、窓 0.4 s) はブレーキ過渡の'
        '減速ピークをなますため実測ゲインを過小評価する。アーティファクトフリーな位置 (long) が'
        '許容する範囲でゲインを確定した (savgol GT の言う 0.72 ではなく 0.84)。</li>'
        '</ul>'
    )

    # ax N-step 比較表 (metrics.csv 由来)。
    ref = "v2" if "v2" in model_order else model_order[0]
    table_horizons = [h for h in (5, 10, 30, 70, 150, 300) if h in set(frame["horizon"])]
    if table_horizons:
        ax_mean = (
            frame[frame["horizon"].isin(table_horizons)]
            .groupby(["model", "horizon"])["ax"].mean()
        )
        head = "".join(f"<th>N={h}</th>" for h in table_horizons)
        rows = []
        for model in model_order:
            cells = []
            for h in table_horizons:
                try:
                    value = float(ax_mean.loc[(model, h)])
                    ref_value = float(ax_mean.loc[(ref, h)])
                except KeyError:
                    cells.append("<td>—</td>")
                    continue
                text = f"{value:.4f}"
                if model != ref and ref_value > 0:
                    diff = (value / ref_value - 1) * 100
                    cls = "score-good" if diff < -0.05 else ("score-bad" if diff > 0.05 else "")
                    text += f' <span class="{cls}">({diff:+.1f}%)</span>'
                cells.append(f"<td>{text}</td>")
            rows.append(f"<tr><td>{_escape(model)}</td>{''.join(cells)}</tr>")
        parts.append(
            f'<h3>ax mean RMSE の N-step 比較 [m/s²]</h3>'
            f'<p class="note">括弧は {_escape(ref)} 比。加減速分離の効果は短ホライズン'
            ' (N&le;30、アクチュエータ過渡) に現れる。</p>'
            f'<div class="table-wrap"><table><thead><tr><th>Model</th>{head}</tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table></div>'
        )

    # レジーム別実測表 (make analyze の成果物があれば)。
    regime_csv = analysis_dir / "regime_metrics.csv"
    if regime_csv.is_file():
        try:
            regime = pd.read_csv(regime_csv)
            case = ""
            meta_path = analysis_dir / "residuals_meta.json"
            if meta_path.is_file():
                case = str(json.loads(meta_path.read_text(encoding="utf-8")).get("case", ""))
            sub = regime[regime["target"] == "err_ax"]
            r_horizons = sorted(set(sub["horizon"]))
            head = "".join(f"<th>N={h}</th>" for h in r_horizons)
            rows = []
            for name in ("brake", "coast", "throttle"):
                cells = []
                for h in r_horizons:
                    row = sub[(sub["regime"] == name) & (sub["horizon"] == h)]
                    if row.empty:
                        cells.append("<td>—</td>")
                    else:
                        r = row.iloc[0]
                        cells.append(
                            f"<td>{r['rms']:.3f} <span class='rmse-ratio'>({r['mean']:+.3f})</span></td>"
                        )
                rows.append(f"<tr><td>{name}</td>{''.join(cells)}</tr>")
            parts.append(
                f'<h3>レジーム別 err_ax [m/s²] (make analyze'
                f'{f", case={_escape(case)}" if case else ""})</h3>'
                '<p class="note">値は RMS (括弧は署名付き平均 = バイアス、err = GT &minus; sim で'
                '正はモデルの過減速/アンダーシュート)。窓平均指令による分類。</p>'
                f'<div class="table-wrap"><table><thead><tr><th>Regime</th>{head}</tr></thead>'
                f'<tbody>{"".join(rows)}</tbody></table></div>'
            )
        except Exception:  # noqa: BLE001 (レポートは成果物欠損に寛容)
            pass
    else:
        parts.append(
            '<p class="note">レジーム別実測表は <code>make analyze ANALYZE_STAGES=traces,regime</code>'
            ' の成果物 (analysis/regime_metrics.csv) があるときに表示される。</p>'
        )
    return "".join(parts)


def _render_release_spec(scenario: Path, release_path: Path) -> str:
    """Scenario の release 指定と、実際にリリースされた param 値を 7 章に表示する。"""
    note = ""
    if scenario.is_file():
        release = load_model_config(scenario).release
        if release is None:
            note = '<p class="note">release 指定なし: リリース出力なし。</p>'
        else:
            note = (
                f'<p class="note">release 指定: <b>{_escape(release.model)}</b> を '
                f"<b>v{release.version}</b> スロット (version: {release.version}) "
                "としてリリース。</p>"
            )
    return note + _render_release_params(release_path)


def _render_release_params(release_path: Path) -> str:
    """リリース YAML (simulator_model.param.yaml) の版スロット・global 値をテーブル表示する。"""
    if not release_path.is_file():
        return '<p class="note">リリース YAML は生成されていません (release 未指定)。</p>'
    try:
        raw = yaml.safe_load(release_path.read_text(encoding="utf-8"))
        ros_params = raw["/**"]["ros__parameters"]
        model_params = ros_params[RELEASE_MODEL_KEY]
        version = model_params["version"]
        slot = model_params[f"v{version}"]
    except (KeyError, TypeError, AttributeError):
        return '<p class="note">リリース YAML の形式が想定と異なり、パラメータを表示できません。</p>'
    if not isinstance(slot, Mapping):
        return '<p class="note">リリース YAML の版スロットが mapping ではありません。</p>'
    slot_table = _table(
        ("Parameter", "Released value"),
        ((name, _format_param_value(slot[name])) for name in sorted(slot)),
        css_class="params",
    )
    global_rows = [
        (ros_key, _format_param_value(ros_params[ros_key]))
        for ros_key in sorted(set(GLOBAL_PARAM_KEYS.values()))
        if ros_key in ros_params
    ]
    global_table = (
        _table(("Parameter", "Released value"), global_rows, css_class="params")
        if global_rows
        else '<p class="note">global 制限値がリリース YAML にありません。</p>'
    )
    return (
        f"<h3>Released versioned parameters (v{_escape(version)})</h3>{slot_table}"
        f"<h3>Released global limits</h3>{global_table}"
    )


def _comparison_model_order(scenario: Path) -> tuple[str, ...]:
    if not scenario.is_file():
        return MODEL_ORDER
    config = load_model_config(scenario)
    return tuple(name.lower() for name in comparison_display_order(config))


def _objective_version(document: Mapping[str, Any]) -> int | None:
    """成果物 metadata の objective version (旧成果物や欠落は None)。"""
    metadata = document.get("metadata")
    if not isinstance(metadata, Mapping):
        return None
    objective = metadata.get("objective")
    if not isinstance(objective, Mapping):
        return None
    try:
        return int(objective["version"])
    except (KeyError, TypeError, ValueError):
        return None


def _render_objective_equations(objective_version: int | None) -> str:
    """
    Render the objective equations (N-step rollout, normalization, robust_score).

    統合最適化セクションの目的関数を数式で提示する。式は fit_merge / _multi_agg が実際に
    最適化する robust_score に一致させる。レポートの「Mean normalized RMSE」列・分布は
    別定義(7 指標 RMSE 比の平均)なので明確に区別する。

    以下の数式は objective v3 の定義。report-only 再生成で旧成果物
    (metadata.objective.version != 3) を読んだ場合は、スコアが旧定義で算出されている旨の
    警告を先頭に挿入する。
    """
    warning = ""
    if objective_version != 3:
        found = "記録なし" if objective_version is None else f"v{objective_version}"
        warning = (
            '<p class="note"><b>⚠ この成果物は旧 objective で算出されています'
            f"(metadata.objective.version: {found})。</b>"
            "以下の数式は現行 objective v3 の定義であり、この成果物の Aggregate score とは"
            "一致しません。旧ランとの比較は当時の定義 (report/YAML の記録) に従ってください。</p>"
        )
    return f"""<details id="eq-score"><summary>評価関数の定義(N-step rollout・正規化・robust_score)</summary>
{warning}""" + """
<p>各データセットについて、実機ログの初期状態から制御コマンドを <b>N ステップ前向き積分</b>し、
実機軌跡との<b>終端誤差 RMSE</b> を horizon \\(N\\) 別に評価する(yaw [deg]・long [cm]・lat [cm] 等)。</p>
<p>難易度の異なるデータセットを公平に集約するため、<b>baseline モデルの誤差でフロアクリップ付き正規化</b>する:</p>
\\[ \\text{nyaw}_N = \\frac{\\text{yaw}_{\\mathrm{tuned}}}{\\max(\\text{yaw}_{\\mathrm{baseline}},\\; f_{\\mathrm{yaw}}(N))},
\\quad (\\text{nlong}, \\text{nlat}\\ \\text{も同様}) \\]
<p>フロア \\(f(N)\\) は <b>baseline モデルの per-dataset RMSE 分布の p10</b> を horizon 別に採用した
テーブル(steer/ax フロアと同じ方法論。openloop_j6_16_onwards, 318 datasets, 2026-07-16)で、
ほぼ直進のデータセットで分母がゼロ近くになる暴発を防ぐ。
旧 per-step 線形フロア(yaw 0.006 deg・long 0.1 cm・lat 0.03 cm × \\(N\\))は実データの誤差成長と
乖離していた(yaw は長 horizon で p10 の 2 倍 = 過剰クリップ、long/lat は p10 の 0.2〜0.3 倍で
暴発を許す)ため objective v3 で再校正した。採用値は tuned_params.yaml の
metadata.objective.floors_by_horizon に記録される。</p>
<p>steer・ax も同様に baseline 比で正規化するが、両者は安定な 1 次遅れ系の状態量で
open-loop 誤差が N≈20 (steer) / N≈9 (ax) までに定常値へ飽和する(プラトー特性)ため、
フロアは \\(N\\) に比例させない<b>定数</b>(steer 0.12 deg・ax 0.10 m/s²、
baseline per-dataset 分布の p10)とする:</p>
\\[ \\text{nsteer}_N = \\frac{\\text{steer}_{\\mathrm{tuned}}}{\\max(\\text{steer}_{\\mathrm{baseline}},\\; 0.12)},
\\quad \\text{nax}_N = \\frac{\\text{ax}_{\\mathrm{tuned}}}{\\max(\\text{ax}_{\\mathrm{baseline}},\\; 0.10)} \\]
<p>最終目的関数はホライズン等重みの mean + worst 合算(<b>小さいほど良い</b>)。
アクチュエータ項は同じ理由(プラトーで \\(N\\) 非依存)から全ホライズンでなく、
過渡 \\(N=10\\)(時定数・むだ時間の情報)とプラトー \\(N=30\\)(定常ゲイン・バイアスの情報)の
代表 2 点のみ、重み 0.5 で加算する:</p>
\\[ \\text{score} = \\sum_{N} \\left[
  \\left(\\overline{\\text{nyaw}} + 0.5\\,\\overline{\\text{nlong}} + 0.5\\,\\overline{\\text{nlat}}\\right)
+ 0.5\\left(\\widehat{\\text{nyaw}} + 0.5\\,\\widehat{\\text{nlong}} + 0.5\\,\\widehat{\\text{nlat}}\\right)
\\right]
+ \\sum_{N \\in \\{10,30\\}} 0.5\\left[
  \\left(\\overline{\\text{nsteer}} + \\overline{\\text{nax}}\\right)
+ 0.5\\left(\\widehat{\\text{nsteer}} + \\widehat{\\text{nax}}\\right)
\\right] \\]
<p>ここで \\(\\overline{\\cdot}\\) は全データセットの mean、\\(\\widehat{\\cdot}\\) は
<b>CVaR@90%(正規化比の上位 10% の平均)</b>。yaw : 位置 = 1 : 1、
worst 項の重み 0.5(mean の改善と worst 側テールの頑健性を半々で重視)。
旧 objective の worst=max は dataset 数が多いとき単一の外れ dataset が score を支配した
(318 datasets の実測で worst 項が score の 61%、外れ 1 件除外で −9.3% 変動)ため、
v3 で CVaR@90% に置換した(worst 項比率 46%・外れ 1 件除外の変動 −1.3%)。</p>
<div class="note"><b>表の 2 つのスコアを混同しないこと</b>:
<b>Aggregate score</b> 列は上記 \\(\\text{score}\\)(robust_score、最適化の目的関数)。
一方 <b>Mean normalized RMSE</b> 列とデータセット分布は別定義で、pos/long/lat/yaw/steer/vx/ax の
全 7 指標について各データセット・各 horizon の <b>RMSE を baseline 比にした値の平均</b>。用途が異なる。</div>
</details>"""


def _render_plateau_theory() -> str:
    """
    Render the channel-agnostic plateau methodology (section 1 shared preamble).

    steer/ax の N-step 誤差プラトー特性・2 段構成の定常同定・GT 整備を 1 箇所にまとめ、
    3・4 章の各系統ステージがここを参照する。数式・定数は _multi_agg.py / settings.py /
    fit_plateau.py の実装に一致させる。系統別の採用値と個別注記は 3・4 章に置く。
    """
    return r"""<h3 id="plateau-theory">プラトー特性とスケーリングの定常同定</h3>
<p>steer・ax は安定な <b>1 次遅れ + むだ時間系の状態量</b>である。open-loop rollout では GT で
同期した初期状態の記憶が washout 時間 \(\approx 3\tau + L\) で消え、以後の応答はコマンド履歴だけで
決まるため、終端誤差は<b>定常過程</b> (モデルの定常ミスマッチ + 計測誤差) に飽和する:</p>
\[ \lim_{N\to\infty} \text{RMSE}(N) = \sigma_{\infty}
\quad (\text{washout} \approx 3\tau + L \approx 0.3\text{–}0.6\,\mathrm{s}
\;\Rightarrow\; N \approx 10\text{–}20 \;@\;0.03\,\mathrm{s/step}) \]
<p>一方 yaw/long/lat はこれらの<b>積分</b>なので誤差が蓄積し、N とともに単調増加する。
プラトー値は初期条件に依存しない定常忠実度の指標なので、各系統の τ・むだ時間を固定したまま
scaling factor だけを直接同定できる。</p>
<h3>スケーリングのプラトー同定: 系統別の 2 段構成</h3>
<p>各系統の直接同定は「<b>τ・むだ時間を動的励起データの最小二乗で決定 → scaling factor を
プラトー (定常残差、既定 N=30) の最小化で決定</b>」の 2 段で行う (3・4 章)。モデル構造上
steer 終端状態は steer 系のみ、ax 終端状態は acc 系のみに依存するため、目的関数は独立な
1 次元フィットに分離される:</p>
\[ J_{\text{steer}}(k_s) = \overline{\text{steer RMSE}_{N}},\qquad
   J_{\text{ax}}(k_a) = \overline{\text{ax RMSE}_{N}}
\quad (\text{各チャネルの生単位、探索域は parameter\_constraints.py の SSOT}) \]
<p>同定コアは rollout の正式評価 (fit_merge._eval) を使う単一の実装
(fit_plateau.fit_scaling_channels) で、fit_lon / fit_steer が τ/delay 確定後にこのコアを呼んで
自チャネルの scaling を決め直す。プラトー値は N 非依存なので、統合最適化のアクチュエータ項も
定数フロア・代表 2 horizon (N=10 過渡・N=30 プラトー) で足りる
(<a href="#eq-score">🔗 評価関数の定義</a>)。長 horizon の yaw/位置ドリフトは定常アクチュエータ
誤差の積分なので、プラトーの系統成分削減が長期ドリフト削減に直結する。</p>
<h3>プラトーの下限と GT の整備 (ノイズか、帯域内ミスマッチか)</h3>
<p>残差の累積分散スペクトル (2026-07, 12 datasets) では、steer 残差の約 53%・ax 残差の約 42% が
0.1 Hz 以下に集中し、2 Hz のゼロ位相 LPF を掛けても RMS は 5% 前後しか下がらない。
つまりプラトーは計測ノイズの下限ではなく<b>帯域内・低周波の系統的ミスマッチ</b>
(動作条件依存のゲイン/位相誤差) が支配的である。この知見に基づき GT を整備する:</p>
<ul>
<li><b>GT の平滑化は後段 LPF を足さず savgol の窓に統合</b>:
ax は微分窓 0.2→0.4 s (実効カットオフ 2.76→1.42 Hz、acc 帯域 1/(2πτ)≈0.53 Hz の ~2.7 倍、
lib/_accel_source.py)、steer は平滑化窓 0.4 s (カットオフ 2.55 Hz、steer 帯域 ≈1.06 Hz の
~2.4 倍、lib/_steer_source.py の <code>steering_source: steer_savgol</code>) で、
「生 GT + 2 Hz ゼロ位相 LPF カスケード」相当のフロア低減を窓のみで実現</li>
<li><b>GT ソースの整合</b>: baseline / v1 は raw ソース (accel は
<code>/localization/acceleration</code> +0.080 s 遅延補償、steer は steering_status 生値)。
<b>v2 は定常補正パラメータ + SG 系 GT (steer_savgol + kinematic_savgol) の組</b>で、
GT のみ SG 化した旧 v1_sg はここに統合した (窓 0.4 s では raw との ax 差は +1〜4% 程度)。
v2 ベースの ablation (v2_rk4 / v2_c / v2_t) と同定対象 current も
同じ SG 系 GT に揃えている</li>
</ul>"""


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
    scenario_path: Path,
    analysis_dir: Path,
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
nav {{ margin:8px 0 0; font-size:13px; }} nav a {{ margin-right:14px; white-space:nowrap; }}
.eqref {{ color:var(--muted); font-size:13px; margin:0 0 10px; }}
.eq-block {{ margin:14px 0; padding:2px 14px; border-left:3px solid var(--line); overflow-x:auto; }}
.note {{ background:#fff8e1; border-left:4px solid #ffc107; color:var(--ink); padding:8px 12px; margin:8px 0; font-size:13px; border-radius:4px; }}
details {{ margin:10px 0; }} details > summary {{ cursor:pointer; font-weight:600; color:var(--accent); padding:4px 0; }}
.rmse-cell {{ min-width:190px; text-align:left; }}
.rmse-main {{ display:flex; justify-content:space-between; align-items:baseline; gap:10px; }}
.rmse-value {{ font-weight:600; }} .rmse-ratio {{ color:var(--muted); font-size:11px; white-space:nowrap; }}
.rmse-bar {{ position:relative; height:10px; margin-top:5px; background:#f1f3f5; border-radius:999px; overflow:hidden; }}
.rmse-bar::after {{ content:""; position:absolute; left:50%; top:0; bottom:0; border-left:1px solid rgba(0,0,0,.38); }}
.rmse-fill {{ position:absolute; top:0; bottom:0; left:0; border-radius:999px; background:#adb5bd; }}
.rmse-fill.good {{ background:#28a745; }} .rmse-fill.bad {{ background:#dc3545; }} .rmse-fill.neutral {{ background:#868e96; }}
.rmse-legend {{ font-weight:600; }} .rmse-legend.good {{ color:#28a745; }} .rmse-legend.bad {{ color:#dc3545; }} .rmse-legend.neutral {{ color:#868e96; }}
.score-good {{ color:#28a745; font-weight:600; }} .score-bad {{ color:#dc3545; }}
</style>
{_MATHJAX_HEAD}
<script>{get_plotlyjs()}</script>
</head>
<body><main>
<header>
  <h1>Reidentification and physical-validity report</h1>
  <p class="lede">N-step metrics are finalized artifacts; physical-validity diagnostics use the existing CSV cache.</p>
  <div class="stats"><div class="stat"><span>Valid datasets</span><strong>{datasets}</strong></div><div class="stat"><span>Horizons</span><strong>{horizons}</strong></div></div>
  <p class="source">Parameters: {_escape(tuned_path)}</p>
  <p class="source">Metrics: {_escape(metrics_path)}</p>
  <nav>
    <a href="#eq-notation">1. 記号と運動方程式</a><a href="#sec-extraction">2. Extraction</a>
    <a href="#longitudinal">3. Longitudinal</a><a href="#steering">4. Steering</a>
    <a href="#xy">5. XY</a><a href="#sec-optimization">6. Optimization</a><a href="#sec-released">7. Released</a><a href="#sec-timeseries">8. Timeseries</a>
  </nav>
</header>
<section><h2>1. 記号と運動方程式</h2><p class="lede">以降の各セクションはここで定義した記号・残差式を参照する。</p>{physical_sections.equations}{_render_plateau_theory()}</section>
<section id="sec-extraction"><h2>2. Extraction results</h2>{_render_failures(extraction_summary)}{physical_sections.prepare}</section>
<section><h2>3. Longitudinal direct identification</h2>{_render_artifact(phase1_path, parameter_title="phase1_acc.yaml")}{_render_stage_plateau(phase1_path, scale_key="debug_acc_scaling_factor", unit="m/s²", channel="ax")}{_render_lon_brake_split(frame, model_order, analysis_dir)}{physical_sections.longitudinal}</section>
<section><h2>4. Steering direct identification</h2>{_render_artifact(phase2_path, parameter_title="phase2_steer.yaml")}{_render_stage_plateau(phase2_path, scale_key="debug_steer_scaling_factor", unit="deg", channel="steer")}{physical_sections.steering}{physical_sections.yaw}</section>
<section><h2>5. XY heading-rate direct identification</h2>{_render_artifact(phase3_path, parameter_title="phase3_xy.yaml")}{physical_sections.xy}</section>
<section id="sec-optimization"><h2>6. Integrated optimization</h2>{_render_artifact_document(document, tuned_path, parameter_title="Final parameters")}<h3>Aggregate comparison</h3>{_render_objective_equations(_objective_version(document))}<p class="note">Raw columns (pos/long/…) are mean RMSE. <b>Aggregate score</b> is the optimized robust_score; <b>Mean normalized RMSE</b> is the 7-metric ratio mean — see the objective definition above (<a href="#eq-score">🔗 評価関数の定義</a>). Aggregate/distribution values use the configured optimization horizons; graphs use every available N. Lower is better.</p>{_render_aggregate(document, summary, model_order)}<h3>Error by horizon N</h3><p class="note">Each point is the mean RMSE across valid datasets. Every available N is plotted; lower is better.</p>{_render_horizon_charts(frame, model_order)}<h3>Dataset distributions</h3>{_render_dataset_distribution(summary, model_order)}</section>
<section id="sec-released"><h2>7. Released YAML</h2>{_render_release_spec(scenario_path, release_path)}<p class="source">Released parameter YAML: {_escape(release_path)}</p></section>
<section id="sec-timeseries"><h2>8. 対象データセットの時系列診断</h2><p class="lede">scenario の plot_dataset で指定したデータセットについて、実測系列と <b>C++ 車両モデル (リリース実装) の予測系列</b>を重ね描きする: 微分行(<a href="#eq-long">縦方向</a>/<a href="#eq-steer">操舵</a>)の両辺 (1-step 予測)、および加速度/速度/ステア/ヨーレートの窓リスタート free-run 軌跡と指令値。対象モデルは release 指定 (または fit 結果) のみ — 特定できない場合は描画しない。</p>{physical_sections.timeseries}</section>
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
        Path(release_params)
        if release_params is not None
        else tuned_path.parent / "simulator_model.param.yaml",
        scenario_path,
        tuned_path.parent / "analysis",
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(rendered, encoding="utf-8")
    print(
        "[report] complete: "
        f"metrics={metrics_elapsed:.1f}s physical-validity={physical_elapsed:.1f}s "
        f"html={perf_counter() - render_started:.1f}s total={perf_counter() - started:.1f}s"
    )
    return out_path
