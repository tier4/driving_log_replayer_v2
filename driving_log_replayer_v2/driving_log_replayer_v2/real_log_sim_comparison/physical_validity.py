"""Parallel physical-validity sections for the unified reidentification report."""
from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import math
import multiprocessing
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np
import plotly.graph_objects as go
import yaml

from .lib._accel_source import savgol_derivative
from .lib._parallel import imap_with_watchdog
from .lib._parallel import normalize_parallel_jobs
from .lib._parallel import pool_chunksize
from .lib._parallel import set_worker_thread_env_defaults
from .lib._report_format import escape as _escape
from .lib._report_format import format_number as _number
from .lib._steer_source import steer_dataframe_from_source
from .reidentify.csv_schema import CACHE_NAME
from .reidentify.fit_core import delay_command
from .reidentify.fit_core import simulate_first_order
from .reidentify.load_data import build_resampled
from .reidentify.load_data import build_rollout_data
from .reidentify.load_data import discover_cached_datasets
from .reidentify.load_data import read_dataset_csv
from .reidentify.model_config import load_model_config
from .reidentify.model_config import ModelConfig
from .reidentify.model_config import ModelSpec
from .reidentify.model_config import RELEASE_TUNED_NAME
from .reidentify.residuals import build_xy_columns
from .reidentify.residuals import rmse
from .reidentify.residuals import xy_residual
from .reidentify.residuals import yaw_residual
from .reidentify.settings import BASELINE_MODEL_NAME
from .reidentify.settings import LONG_DA_THRESH
from .reidentify.settings import LONG_VX_MIN
from .reidentify.settings import RESAMPLE_DT
from .reidentify.settings import STEER_DSTEER_MIN
from .reidentify.settings import TARGET_MODEL_NAME


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
    steering_source: str = "steer"
    slope_source: str = "none"


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

    equations: str
    prepare: str
    longitudinal: str
    steering: str
    yaw: str
    xy: str
    timeseries: str


# RMSE 色分けバーの判定しきい値。ratio=RMSE/baseline に対し、改善/悪化/中立を分ける。
_RATIO_GOOD = 0.99
_RATIO_BAD = 1.01
_RATIO_BAR_MAX = 2.0  # バー幅の飽和点 (baseline の 2 倍で 100% 幅)

# 各直接同定セクションが参照する状態方程式行のアンカー ID。式は _build_equations_section が定義する。
_EQ_ANCHORS = {
    "longitudinal": ("eq-long", "状態方程式(縦方向)"),
    "steering": ("eq-steer", "状態方程式(操舵)"),
    "yaw": ("eq-yaw", "状態方程式(ヨー)"),
    "xy": ("eq-xy", "状態方程式(位置 x/y)"),
}


def _eqref(section_id: str) -> str:
    """Return the equation-reference line placed at the top of a section."""
    anchor, label = _EQ_ANCHORS[section_id]
    return (
        f'<p class="eqref">🔗 <a href="#{anchor}">{_escape(label)}</a> の整合性を残差 RMSE で評価する。'
        '記号の意味は <a href="#eq-notation">記号定義</a> を参照。</p>'
    )


_NOTATION_ROWS = (
    (r"\(x, y\)", "地図平面上の車両位置", "m", "状態 / —(実測)"),
    (r"\(\theta\)", "ヨー角(地図 X 軸からの回転)", "rad", "状態 / —(実測)"),
    (r"\(v_x\)", "前進速度", "m/s", "状態 / —(実測)"),
    (r"\(\delta_{\mathrm{act}}\)", "前輪実舵角(操舵アクチュエータ状態)", "rad", "状態 / —(実測)"),
    (r"\(a_{\mathrm{act}}\)", "加速度アクチュエータ状態", "m/s²", "状態 / —(実測)"),
    (r"\(\omega\)", r"ヨーレート(\(\dot\theta\))", "rad/s", "—(実測)"),
    (r"\(a_{\mathrm{cmd}}, \delta_{\mathrm{cmd}}\)", "加速度指令 / 操舵指令", "m/s², rad", "入力"),
    (r"\(a_{\mathrm{slope}}\)", "路面勾配による加速度成分", "m/s²", "入力"),
    (r"\(\tau_a,\ T_a\)", "加速度の一次遅れ時定数 / むだ時間", "s", "<code>acc_time_constant</code>, <code>acc_time_delay</code>"),
    (r"\(\tau_\delta,\ T_\delta\)", "操舵の一次遅れ時定数 / むだ時間", "s", "<code>steer_time_constant</code>, <code>steer_time_delay</code>"),
    (r"\(\beta\)", "ステアバイアス(系統的操舵オフセット)", "rad", "<code>steer_bias</code>"),
    (r"\(L\)", "ホイールベース", "m", "<code>wheelbase</code>"),
    (r"\(k_{\mathrm{us}}\)", "アンダーステア係数", "rad·s²/m", "<code>k_us</code>"),
    (r"\(c\)", "位置式の heading 補正係数", "s²/m", "<code>xy_heading_rate_coeff</code>"),
)


def _build_equations_section() -> str:
    r"""
    Build the equation hub centered on the state-space equations of motion.

    数式ハブの中心は状態方程式 \(\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})\)。各直接同定は、その各行を
    実測状態に当てはめてパラメータを同定し、両辺の残差 RMSE で整合性を評価する(rollout を伴わない
    固定評価)。式は reidentify の実装 (residuals.py / fit_core._simulate) と一致させる。
    """
    rows = "".join(
        f"<tr><td>{symbol}</td><td>{meaning}</td><td>{unit}</td><td>{impl}</td></tr>"
        for symbol, meaning, unit, impl in _NOTATION_ROWS
    )
    return f"""<h3 id="eq-notation">記号の定義</h3>
<p>シミュレータは状態 \\(\\mathbf{{x}} = (x, y, \\theta, v_x, \\delta_{{\\mathrm{{act}}}}, a_{{\\mathrm{{act}}}})\\) を
入力 \\(\\mathbf{{u}} = (a_{{\\mathrm{{cmd}}}}, \\delta_{{\\mathrm{{cmd}}}})\\) で駆動する連続時間の状態方程式
\\(\\dot{{\\mathbf{{x}}}} = f(\\mathbf{{x}}, \\mathbf{{u}})\\) を数値積分する。以降で用いる記号は次の通り。</p>
<div class="table-wrap"><table class="params"><thead>
<tr><th>記号</th><th>意味</th><th>単位</th><th>実装(状態 / 入力 / 設定パラメータ)</th></tr>
</thead><tbody>{rows}</tbody></table></div>

<h3>状態方程式(運動方程式)</h3>
<p>本レポートの中心となる運動方程式は次の状態方程式である。操舵・加速度アクチュエータはむだ時間付きの
一次遅れで表され、観測される操舵角は \\(\\delta = \\delta_{{\\mathrm{{act}}}} + \\beta\\)、位置式の実効方位は
\\(\\theta_{{\\mathrm{{eff}}}} = \\theta - c\\,v_x\\,\\dot\\theta\\) とする。</p>
<div class="eq-block">
\\[ \\dot{{\\mathbf{{x}}}} =
\\begin{{pmatrix}} \\dot x \\\\ \\dot y \\\\ \\dot\\theta \\\\ \\dot v_x \\\\ \\dot\\delta_{{\\mathrm{{act}}}} \\\\ \\dot a_{{\\mathrm{{act}}}} \\end{{pmatrix}}
=
\\begin{{pmatrix}}
v_x \\cos\\theta_{{\\mathrm{{eff}}}} \\\\[2pt]
v_x \\sin\\theta_{{\\mathrm{{eff}}}} \\\\[2pt]
\\dfrac{{v_x \\tan\\delta_{{\\mathrm{{act}}}}}}{{L + k_{{\\mathrm{{us}}}}\\,v_x^{{2}}}} \\\\[6pt]
a_{{\\mathrm{{act}}}} + a_{{\\mathrm{{slope}}}} \\\\[2pt]
\\dfrac{{\\delta_{{\\mathrm{{cmd}}}}(t - T_\\delta) - \\delta_{{\\mathrm{{act}}}}}}{{\\tau_\\delta}} \\\\[6pt]
\\dfrac{{a_{{\\mathrm{{cmd}}}}(t - T_a) - a_{{\\mathrm{{act}}}}}}{{\\tau_a}}
\\end{{pmatrix}} \\]
</div>
<p>各直接同定セクションは、この状態方程式の該当行だけを取り出し、実測状態を代入して右辺(モデル予測)と
左辺(実測)の差 \\(E\\) の RMSE を最小化する形でパラメータを同定する。以下は各行の再掲で、丸括弧内は
その行で同定するパラメータを示す。</p>
<div class="eq-block" id="eq-long">
<p><b>縦方向 — 加速度アクチュエータ \\(\\dot a_{{\\mathrm{{act}}}}\\) の行</b>(同定: \\(\\tau_a, T_a\\))</p>
\\[ \\dot a_{{\\mathrm{{act}}}} = \\frac{{a_{{\\mathrm{{cmd}}}}(t - T_a) - a_{{\\mathrm{{act}}}}}}{{\\tau_a}} \\]
<p class="note">むだ時間 \\(T_a\\) だけ遅れた指令へ時定数 \\(\\tau_a\\) で一次遅れ追従する。固定評価では
この行から予測した \\(a_{{\\mathrm{{act}}}}\\) と実測加速度の残差 RMSE を用い、加減速中(DRIVE・十分な速度・
指令変化があるサンプル)のみで評価する。</p>
</div>
<div class="eq-block" id="eq-steer">
<p><b>操舵 — 操舵アクチュエータ \\(\\dot\\delta_{{\\mathrm{{act}}}}\\) の行</b>(同定: \\(\\tau_\\delta, T_\\delta, \\beta\\))</p>
\\[ \\dot\\delta_{{\\mathrm{{act}}}} = \\frac{{\\delta_{{\\mathrm{{cmd}}}}(t - T_\\delta) - \\delta_{{\\mathrm{{act}}}}}}{{\\tau_\\delta}},
\\qquad \\delta = \\delta_{{\\mathrm{{act}}}} + \\beta \\]
<p class="note">縦方向と同じ一次遅れ構造。観測される操舵角はステアバイアス \\(\\beta\\) を加えた \\(\\delta\\)。</p>
</div>
<div class="eq-block" id="eq-yaw">
<p><b>ヨー — キネマティック単純モデルの \\(\\dot\\theta\\) の行</b>(同定: \\(L, k_{{\\mathrm{{us}}}}\\))</p>
\\[ \\dot\\theta = \\omega = \\frac{{v_x \\tan\\delta_{{\\mathrm{{act}}}}}}{{L + k_{{\\mathrm{{us}}}}\\,v_x^{{2}}}} \\]
<p class="note">外部入力を持つアクチュエータ応答ではなく、観測状態 \\(v_x, \\delta_{{\\mathrm{{act}}}}, \\omega\\) だけで
閉じる関係。固定評価では右辺と実測ヨーレート \\(\\omega\\) の残差 RMSE を用いる。</p>
</div>
<div class="eq-block" id="eq-xy">
<p><b>位置 — heading 補正付きキネマティクスの \\(\\dot x, \\dot y\\) の行</b>(同定: \\(c\\))</p>
\\[ \\dot x = v_x \\cos\\theta_{{\\mathrm{{eff}}}}, \\qquad \\dot y = v_x \\sin\\theta_{{\\mathrm{{eff}}}},
\\qquad \\theta_{{\\mathrm{{eff}}}} = \\theta - c\\,v_x\\,\\dot\\theta \\]
<p class="note">固定評価では右辺と実測軌跡の数値微分(Savitzky-Golay で得た \\(\\dot x, \\dot y\\))の残差 RMSE を
用いる(\\(x, y\\) の残差を結合)。</p>
</div>"""


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
        if config.fit.enabled and name == config.fit.target:
            params.update(tuned_params)
        models.append(
            ComparedModel(
                name, spec.vehicle_model_type, spec.acceleration_source, params,
                spec.steering_source,
            )
        )
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
        # 親プロセス側でも fork 前に呼ぶ (jemalloc bg thread 保持ロックの fork-time
        # デッドロック対策、reidentify/fit_plateau.py 参照)。initializer は子側の保険。
        set_worker_thread_env_defaults()
        with multiprocessing.get_context("fork").Pool(
            n_workers, initializer=set_worker_thread_env_defaults
        ) as pool:
            evaluations = imap_with_watchdog(pool, _evaluate_dataset, tasks, chunksize=chunksize)
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


def _rmse_visual_cell(value: float, baseline: float, *, is_baseline: bool) -> str:
    """
    Render an RMSE value as a two-row cell (number + baseline-ratio bar).

    RMSE を「数値 + baseline 比の色分け横棒バー」の 2 段セルで表示する。中央線 (50% 幅) が
    baseline (ratio=1.0)。緑=改善 (ratio<_RATIO_GOOD)、赤=悪化 (ratio>_RATIO_BAD)、灰=中立。
    バーが短いほど良い。
    """
    if not np.isfinite(value):
        return '<td class="rmse-cell">—</td>'
    ratio = value / baseline if np.isfinite(baseline) and baseline != 0 else float("nan")
    if is_baseline:
        ratio = 1.0
    if np.isfinite(ratio):
        width = min(max(ratio, 0.0), _RATIO_BAR_MAX) * 50.0
        ratio_txt = f"{ratio:.2f}x"
        pct_txt = f"{(1.0 - ratio) * 100.0:+.0f}%"
        cls = "good" if ratio < _RATIO_GOOD else "bad" if ratio > _RATIO_BAD else "neutral"
    else:
        width, ratio_txt, pct_txt, cls = 50.0, "—", "—", "neutral"
    title = f"RMSE={_number(value)}, baseline={_number(baseline)}, ratio={ratio_txt}, improvement={pct_txt}"
    return (
        f'<td class="rmse-cell" title="{_escape(title)}">'
        f'<div class="rmse-main"><span class="rmse-value">{_number(value)}</span>'
        f'<span class="rmse-ratio">{_escape(ratio_txt)} / {_escape(pct_txt)}</span></div>'
        f'<div class="rmse-bar"><span class="rmse-fill {cls}" style="width:{width:.1f}%"></span></div>'
        "</td>"
    )


def _summary_table(models: list[ComparedModel], results: dict[str, dict[str, Any]], param_keys: tuple[str, ...]) -> str:
    baseline = results.get(BASELINE_MODEL_NAME, {}).get("rmse", float("nan"))
    rows = []
    for model in models:
        value = results[model.name]
        rmse = value.get("rmse", float("nan"))
        params = ", ".join(f"{key}={_number(model.params.get(key))}" for key in param_keys)
        reason = value.get("reason", "")
        model_style = ' style="color:#888"' if model.name == BASELINE_MODEL_NAME else ""
        rows.append(
            f"<tr><th{model_style}>{_escape(model.name)}</th>"
            f"{_rmse_visual_cell(rmse, baseline, is_baseline=model.name == BASELINE_MODEL_NAME)}"
            f"<td>{value.get('n', 0)}</td><td><code>{_escape(params)}</code></td>"
            f"<td>{_escape(reason)}</td></tr>"
        )
    note = (
        '<div class="note">固定評価 RMSE は <b>baseline を中央線 (1.0x=50% 幅)</b> とする横棒バーで表示。'
        "<span class='rmse-legend good'>緑</span>=baseline より小さい残差(改善)、"
        "<span class='rmse-legend bad'>赤</span>=大きい残差(悪化)、"
        "<span class='rmse-legend neutral'>灰</span>=同等。バーが短いほど良い。</div>"
    )
    return (
        "<table><thead><tr><th>モデル</th><th>固定評価 RMSE(baseline 比)</th>"
        "<th>サンプル数</th><th>使用パラメータ</th><th>評価不能理由</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
        + note
    )


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
    section = f'<section id="{section_id}"><h2>{title}</h2>' + _eqref(section_id) + _summary_table(context.models, fixed, keys) + _figure_html(_histogram_by_model(distributions, f"{title}: モデル別固定評価 RMSE"), "固定評価 RMSE 分布") + "</section>"
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
    section = f'<section id="{section_id}"><h2>{title}</h2>' + _eqref(section_id) + _summary_table(context.models, fixed, keys) + _figure_html(_histogram_by_model(distributions, f"{title}: モデル別固定評価 RMSE"), "固定評価 RMSE 分布") + "</section>"
    return ValidationStep(section_id, {"models": fixed, "datasets": fixed.get(TARGET_MODEL_NAME, {}).get("datasets", {})}, section)


def validate_yaw(context: ValidationContext) -> ValidationStep:
    return _cross_step(context, xy=False)


def validate_xy(context: ValidationContext) -> ValidationStep:
    return _cross_step(context, xy=True)


# --- 対象データセットの時系列診断 (レポート 8 章) ---

# 1 トレースの最大表示点数。数値計算は RESAMPLE_DT のフルグリッドで行い、表示だけを
# stride 間引きする (savgol 微分・積分を間引きの影響から切り離す)。
_TIMESERIES_MAX_POINTS = 6000
# 非 DRIVE 区間シェーディングの上限。超えたら描かない (vrect の DOM 肥大防止)。
_TIMESERIES_MAX_SPANS = 32

_TS_MEASURED_STYLE = {"color": "#172033", "width": 2}
_TS_MODEL_STYLE = {"color": "#2563eb", "width": 2}
_TS_CMD_STYLE = {"color": "#b42318", "width": 1.5, "dash": "dot"}


def _timeseries_stride(n: int) -> int:
    return max(1, math.ceil(n / _TIMESERIES_MAX_POINTS))


def _non_drive_spans(gear_drive: np.ndarray, dt: float) -> list[tuple[float, float]]:
    """非 DRIVE の連続区間を (開始秒, 終了秒) のリストで返す。"""
    indices = np.flatnonzero(~np.asarray(gear_drive, dtype=bool))
    if len(indices) == 0:
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [(float(group[0]) * dt, float(group[-1] + 1) * dt) for group in groups]


def _timeseries_figure(
    title: str,
    y_title: str,
    t: np.ndarray,
    traces: list[tuple[str, np.ndarray, dict]],
    stride: int,
    spans: list[tuple[float, float]],
    decel_spans: list[tuple[float, float]] | None = None,
) -> go.Figure:
    fig = go.Figure()
    t_shown = np.round(t[::stride], 3)
    for name, values, style in traces:
        fig.add_trace(
            go.Scatter(x=t_shown, y=np.round(values[::stride], 5), name=name, mode="lines", line=style)
        )
    if spans and len(spans) <= _TIMESERIES_MAX_SPANS:
        for x0, x1 in spans:
            fig.add_vrect(x0=x0, x1=x1, fillcolor="gray", opacity=0.15, line_width=0)
    if decel_spans and len(decel_spans) <= _TIMESERIES_MAX_SPANS:
        # 減速区間 (加減速分離が効く場所) をオレンジで示す。
        for x0, x1 in decel_spans:
            fig.add_vrect(x0=x0, x1=x1, fillcolor="#ff7f0e", opacity=0.10, line_width=0)
    fig.update_layout(
        title=title, xaxis_title="時刻 [s]", yaxis_title=y_title,
        hovermode="x unified", legend={"orientation": "h"}, height=340,
        margin={"t": 48, "b": 40, "l": 60, "r": 20},
    )
    return fig


def _mask_spans(t: np.ndarray, mask: np.ndarray) -> list[tuple[float, float]]:
    """真の連続区間を (開始秒, 終了秒) のリストで返す (非一様時刻グリッド対応)。"""
    indices = np.flatnonzero(np.asarray(mask, dtype=bool))
    if len(indices) == 0:
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [(float(t[group[0]]), float(t[min(group[-1] + 1, len(t) - 1)])) for group in groups]


# 窓リスタート幅 (コマンド interval 数)。~10 s @ 30 Hz。窓ごとに GT 状態へリセットして
# free-run し、モデル軌跡を実測に重ねる (発散を防ぎつつ N-step 挙動を見せる)。
_TS_WINDOW_INTERVALS = 300
# 減速ハイライトのしきい値 (窓平均でなく瞬時指令) [m/s²]。analyze.regime と同じ境界。
_TS_DECEL_A_CMD = -0.3


def _timeseries_figures(
    source: dict[str, Any], model: ComparedModel,
) -> tuple[list[tuple[str, go.Figure]] | None, int, str | None]:
    """
    C++ 車両モデル (リリース実装そのもの) で 8 章の時系列 6 図を組み立てる。

    モデル側の系列は Python の簡略式ではなく、ctypes wrapper 経由の C++ モデルを
    2 通りに走らせて得る (簡略式はモデル進化 — brake split / 実効ステア等 — から
    取り残されるため使わない):
      (a) 1-step 予測: 各時刻で GT 状態にリセットして 1 interval 積分 → 微分方程式の
          右辺相当 (ジャーク行・ステアレート行の「両辺」比較)
      (b) 窓リスタート free-run (窓幅 ~10 s): 窓頭で GT 状態にリセットして自由走行 →
          状態量 (加速度/速度/ステア/ヨーレート) の実測 vs モデル軌跡
    返り値: (figures, n_points, error_reason)。
    """
    from .lib._vehicle_models import merge_vehicle_model_params
    from .reidentify import rollout

    data = build_rollout_data(
        source,
        acceleration_source=model.acceleration_source,
        steering_source=model.steering_source,
    )
    t0_ns = rollout.find_autonomous_start(data)
    params = merge_vehicle_model_params(
        rollout.build_params(), dict(model.params), model.vehicle_model_type
    )
    g = rollout._prepare_gt(data, t0_ns, params)
    t = np.asarray(g["t_cmd"], dtype=float)
    n = len(t)
    if n < 10:
        return None, n, "コマンド系列が短すぎます"
    gt_ax = np.asarray(g["gt_ax"], dtype=float)
    gt_vx = np.asarray(g["gt_vx"], dtype=float)
    gt_steer = np.asarray(g["gt_steer"], dtype=float)
    gt_yaw = np.asarray(g["gt_yaw"], dtype=float)
    gt_wz = np.asarray(g["gt_wz"], dtype=float)
    a_cmd = np.asarray(g["accel_des_arr"], dtype=float)
    d_cmd = np.asarray(g["steer_des_arr"], dtype=float)
    iv = np.asarray(g["iv_arr"], dtype=float)

    # (a) 1-step 予測 (毎 interval、GT 状態リセット)。
    k1, sim1 = rollout._rollout_sim_states(
        g, params, model.vehicle_model_type, [1], 1, slope_source=model.slope_source,
    )
    jerk_rhs = np.full(n, np.nan)
    steer_rate_rhs = np.full(n, np.nan)
    jerk_rhs[k1] = (sim1["ax"][:, 0] - gt_ax[k1]) / iv[k1]
    steer_rate_rhs[k1] = (sim1["steer"][:, 0] - gt_steer[k1]) / iv[k1]
    jerk_lhs = np.gradient(gt_ax, t)
    steer_rate_lhs = np.gradient(gt_steer, t)

    # (b) 窓リスタート free-run。
    window = _TS_WINDOW_INTERVALS
    horizons = list(range(1, window + 1))
    k_w, sim_w = rollout._rollout_sim_states(
        g, params, model.vehicle_model_type, horizons, window,
        slope_source=model.slope_source,
    )
    traj = {key: np.full(n, np.nan) for key in ("ax", "vx", "steer", "yaw")}
    for wi, k0 in enumerate(k_w):
        end = min(window, n - 1 - int(k0))
        if end <= 0:
            continue
        sl = slice(int(k0) + 1, int(k0) + 1 + end)
        for key in traj:
            traj[key][sl] = sim_w[key][wi, :end]
        # ヨーレート差分の起点として窓頭 (リセット点 = GT) を埋める。
        if not np.isnan(sim_w["yaw"][wi, 0]):
            traj["yaw"][int(k0)] = gt_yaw[int(k0)]
    wz_model = np.full(n, np.nan)
    wz_model[1:] = np.diff(traj["yaw"]) / np.diff(t)

    stride = _timeseries_stride(n)
    spans = _mask_spans(t, ~np.asarray(g["valid_gear_arr"], dtype=bool))
    decel_spans = _mask_spans(t, a_cmd < _TS_DECEL_A_CMD)

    figures = [
        ("ジャーク: ȧ_act 行の両辺 (C++ 1-step)", _timeseries_figure(
            "ジャーク", "ジャーク [m/s³]", t,
            [
                ("左辺: 実測ジャーク (a_act の勾配差分)", jerk_lhs, _TS_MEASURED_STYLE),
                ("右辺: C++ モデル 1-step 予測 Δa/Δt", jerk_rhs, _TS_MODEL_STYLE),
            ], stride, spans, decel_spans,
        )),
        ("加速度: 実測 vs C++ モデル軌跡 (窓リスタート free-run)", _timeseries_figure(
            "加速度", "加速度 [m/s²]", t,
            [
                ("実測加速度 a_act", gt_ax, _TS_MEASURED_STYLE),
                ("C++ モデル a (窓 free-run)", traj["ax"], _TS_MODEL_STYLE),
                ("指令 a_cmd", a_cmd, _TS_CMD_STYLE),
            ], stride, spans, decel_spans,
        )),
        ("速度: 実測 vs C++ モデル軌跡", _timeseries_figure(
            "速度", "速度 [m/s]", t,
            [
                ("実測速度 v_x", gt_vx, _TS_MEASURED_STYLE),
                ("C++ モデル v_x (窓 free-run)", traj["vx"], _TS_MODEL_STYLE),
            ], stride, spans, decel_spans,
        )),
        ("ステアレート: δ̇_act 行の両辺 (C++ 1-step)", _timeseries_figure(
            "ステアレート", "ステアレート [rad/s]", t,
            [
                ("左辺: 実測ステアレート (δ の勾配差分)", steer_rate_lhs, _TS_MEASURED_STYLE),
                ("右辺: C++ モデル 1-step 予測 Δδ/Δt", steer_rate_rhs, _TS_MODEL_STYLE),
            ], stride, spans,
        )),
        ("ステア: 実測 vs C++ モデル軌跡", _timeseries_figure(
            "ステア", "ステア [rad]", t,
            [
                ("実測ステア δ", gt_steer, _TS_MEASURED_STYLE),
                ("C++ モデル δ (窓 free-run)", traj["steer"], _TS_MODEL_STYLE),
                ("指令 δ_cmd", d_cmd, _TS_CMD_STYLE),
            ], stride, spans,
        )),
        ("ヨーレート: 実測 vs C++ モデル軌跡 (実効ステア/緩和長の見どころ)", _timeseries_figure(
            "ヨーレート", "ヨーレート [rad/s]", t,
            [
                ("実測ヨーレート w_z", gt_wz, _TS_MEASURED_STYLE),
                ("C++ モデル w_z (モデル yaw の差分)", wz_model, _TS_MODEL_STYLE),
            ], stride, spans,
        )),
    ]
    return figures, n, None


def _timeseries_model(
    config: ModelConfig, tuned_params: dict[str, Any],
) -> tuple[ComparedModel | None, str]:
    """時系列診断に使うモデルケースを解決する。

    優先順: (1) release が固定ケース指定 → そのケース、(2) release が "tuned" 指定 or
    release 未指定で fit 有効 → fit 対象ケース + fit 結果。どちらも無い場合は
    **描画しない (None)**: それっぽい別モデルを黙って表示することは何も出さないより
    有害なため、代替モデルへのフォールバックは行わない。
    """
    release = config.release
    if release is not None and release.model != RELEASE_TUNED_NAME:
        spec: ModelSpec = config.find_case(release.model)
        params = dict(spec.params)
        label = f"release ケース {spec.name} (scenario.yaml の release 指定)"
    elif config.fit.enabled:
        spec = config.find_case(config.fit.target)
        params = dict(spec.params)
        params.update(tuned_params)
        label = (
            f"release tuned (fit 対象 {spec.name} + fit 結果)"
            if release is not None
            else f"tuned (release 未指定, fit 対象 {spec.name})"
        )
    else:
        return None, (
            "scenario に release (固定ケース) も有効な fit も指定されていないため、"
            "対象モデルを特定できません。誤解を招くため代替モデルの描画は行いません。"
            "描画するには release: {model: <case>, ...} か fit を scenario に指定してください。"
        )
    return (
        ComparedModel(
            spec.name, spec.vehicle_model_type, spec.acceleration_source, params,
            spec.steering_source, getattr(spec, "slope_source", "none"),
        ),
        label,
    )


def _timeseries_stat(label: str, value: Any) -> str:
    return f'<div class="stat"><span>{_escape(label)}</span><strong>{_escape(str(value))}</strong></div>'


def _render_dataset_timeseries(collection_dir: Path, dataset_id: str, model: ComparedModel) -> str:
    """1 データセット分の時系列診断フラグメントを返す。失敗は .note に変換する。"""
    csv_path = Path(collection_dir) / "datasets" / dataset_id / CACHE_NAME
    if not csv_path.is_file():
        n_cached = len(discover_cached_datasets(Path(collection_dir)))
        return (
            f'<p class="note">plot_dataset={_escape(dataset_id)} のキャッシュ CSV が見つかりません: '
            f"{_escape(str(csv_path))} (キャッシュ済み dataset: {n_cached} 件)</p>"
        )
    try:
        source = read_dataset_csv(csv_path)
        figures, n, reason = _timeseries_figures(source, model)
        if figures is None:
            return (
                f'<p class="note">plot_dataset={_escape(dataset_id)}: 描画できません: '
                f'{_escape(reason or "")}</p>'
            )
        stride = _timeseries_stride(n)
        figures_html = "".join(_figure_html(fig, title) for title, fig in figures)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, FileNotFoundError) as exc:
        # C++ モデル (.so) 未ビルド等も含め、失敗は明示的に報告する (代替描画はしない)。
        return (
            f'<p class="note">plot_dataset={_escape(dataset_id)} の時系列診断の構築に失敗しました: '
            f"{_escape(str(exc))}</p>"
        )
    return f'<p class="source">点数: {n} (表示 1/{stride})</p>{figures_html}'


def build_timeseries_section(collection_dir: Path, params_path: Path, scenario: Path) -> str:
    """
    レポート 8 章 (対象データセットの時系列診断) の HTML 断片を構築する。

    scenario の Evaluation.Conditions.plot_dataset で指定したデータセットのリストを
    順に、状態方程式の左辺・右辺・指令値の時系列で重ね描きする。
    失敗はレポート全体を止めず、データセット単位の .note に変換する。
    """
    try:
        config = load_model_config(scenario)
    except (OSError, ValueError) as exc:
        return f'<p class="note">scenario の読み込みに失敗しました: {_escape(str(exc))}</p>'
    if not config.plot_datasets:
        return (
            '<p class="note">scenario の <code>Evaluation.Conditions.plot_dataset: '
            "[&lt;dataset-id&gt;, ...]</code> を指定すると、そのデータセットについて"
            "状態方程式の左辺・右辺・指令値の時系列診断を表示します。</p>"
        )
    try:
        tuned_document = yaml.safe_load(Path(params_path).read_text(encoding="utf-8")) or {}
        tuned_params = dict(tuned_document.get("params", tuned_document))
        model, label = _timeseries_model(config, tuned_params)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return f'<p class="note">パラメータの解決に失敗しました: {_escape(str(exc))}</p>'
    if model is None:
        return f'<div class="note"><b>時系列診断は描画されません。</b> {_escape(label)}</div>'
    stats = "".join(
        _timeseries_stat(stat_label, value)
        for stat_label, value in (
            ("τ_a [s]", _number(model.params.get("acc_time_constant"))),
            ("T_a [s]", _number(model.params.get("acc_time_delay"))),
            ("K_brk", _number(model.params.get("brake_scaling_factor"))),
            ("τ_brk [s]", _number(model.params.get("brake_time_constant"))),
            ("τ_δ [s]", _number(model.params.get("steer_time_constant"))),
            ("T_δ [s]", _number(model.params.get("steer_time_delay"))),
            ("L_rel [m]", _number(model.params.get("steer_relaxation_length"))),
            ("β [rad]", _number(model.params.get("steer_bias"))),
            ("accel source", model.acceleration_source),
            ("steer source", model.steering_source),
        )
    )
    notes = (
        '<div class="note"><b>モデル側の系列は C++ 車両モデル (リリース実装そのもの) で生成</b>: '
        '簡略な Python 再実装はモデル進化 (加減速分離・実効ステア等) から取り残されるため使わない。'
        '「両辺」図は毎 interval GT 状態にリセットした 1-step 予測、「軌跡」図は窓幅 '
        f'~{_TS_WINDOW_INTERVALS} interval (~10 s) ごとに GT 状態へリセットした free-run。'
        '時間軸はコマンド時刻 (~30 Hz)。'
        f'表示は最大 {_TIMESERIES_MAX_POINTS} 点/トレースへ間引き (数値計算はフル系列)。'
        '<b>オレンジ帯 = 減速区間 (a_cmd &lt; '
        f'{_TS_DECEL_A_CMD:g} m/s²)</b> — 加減速分離 (K_brk/τ_brk) の効果はここで見る。'
        'ヨーレート図は実効ステア (緩和長 L_rel) の効果が旋回過渡の立ち上がりに現れる。'
        '灰色帯 = 非 DRIVE 区間、途切れ = 停止近傍等の評価除外区間 (描画対象外)。'
        '要ビルド済み .so (simple_sensor_simulator)。</div>'
    )
    parts = [
        f"<p>使用パラメータ: <b>{_escape(label)}</b></p>",
        f'<div class="stats">{stats}</div>',
        notes,
    ]
    for dataset_id in config.plot_datasets:
        parts.append(f'<h3 class="ts-dataset">📈 dataset: {_escape(dataset_id)}</h3>')
        parts.append(_render_dataset_timeseries(collection_dir, dataset_id, model))
    return "".join(parts)


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
        equations=_build_equations_section(),
        prepare=prepared.html,
        longitudinal=validate_longitudinal(context).html,
        steering=validate_steering(context).html,
        yaw=validate_yaw(context).html,
        xy=validate_xy(context).html,
        timeseries=build_timeseries_section(collection_dir, params_path, scenario),
    )
    print(f"[report] physical-validity complete: {perf_counter() - started:.1f}s")
    return sections
