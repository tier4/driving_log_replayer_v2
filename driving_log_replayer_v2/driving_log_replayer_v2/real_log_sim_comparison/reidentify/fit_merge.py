#!/usr/bin/env python3
"""直接同定値を初期値にした横断ロバスト最適化。"""
from __future__ import annotations

import csv
from dataclasses import dataclass, field
import datetime
import math
from pathlib import Path
import sys

import optuna
import yaml

from ..lib._accel_source import normalize_accel_source
from ..lib._multi_agg import HORIZONS, aggregate_normalized, format_agg, robust_score
from ..lib._parallel import (
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from ..lib._vehicle_models import get_vehicle_model_spec, merge_vehicle_model_params
from . import rollout
from .load_data import build_rollout_data, discover_cached_datasets, read_dataset_csv
from .model_config import load_model_config, resolve_baseline_model
from .settings import (
    ROLLOUT_STRIDE,
    SEARCH_DELAY_CANDIDATES,
    SEARCH_SPACE_ACC,
    SEARCH_SPACE_STEER,
    TARGET_MODEL_NAME,
)

_GT_KEYS = ("acc_time_delay", "steer_time_delay", "wheelbase", "sub_dt")
_RMSE_KEYS = ("pos", "long", "lat", "yaw", "steer", "vx", "ax")
# Keep the optimization objective on the established sparse horizons, while
# exporting every available integer N up to the already-required N=100 for the
# comparison report.  A dataset accepted by the sparse evaluation has valid
# coverage at N=100, so all smaller horizons are available as well.
REPORT_HORIZONS = tuple(range(1, max(HORIZONS) + 1))
_DIRECT_FIT_KEYS = frozenset(
    {
        "acc_time_constant",
        "acc_time_delay",
        "debug_acc_scaling_factor",
        "steer_time_constant",
        "steer_time_delay",
        "debug_steer_scaling_factor",
        "steer_dead_band",
        "steer_bias",
        "steer_rate_lim",
        "k_us",
        "wheelbase",
    }
)


@dataclass
class DatasetCtx:
    """1 データセットの rollout 実行コンテキスト。"""

    dataset_id: str
    dfs: dict
    t0_ns: int
    base: dict
    data_cache: dict[str, dict] = field(default_factory=dict)
    gt_cache: dict = field(default_factory=dict)
    base_metric: dict = field(default_factory=dict)


def _ctx_data(ctx: DatasetCtx, acceleration_source: str) -> dict:
    source = normalize_accel_source(acceleration_source)
    if source not in ctx.data_cache:
        ctx.data_cache[source] = build_rollout_data(ctx.dfs, acceleration_source=source)
    return ctx.data_cache[source]


def _eval(
    ctx: DatasetCtx, override: dict, model_type: str, acceleration_source: str = "accel",
    *, horizons: tuple[int, ...] = HORIZONS,
) -> dict:
    """1 dataset・1 パラメータ組の horizon 別終端誤差 RMSE {h: {yaw,pos,long,lat,steer}}。"""
    params = merge_vehicle_model_params(ctx.base, override, model_type)
    source = normalize_accel_source(acceleration_source)
    data = _ctx_data(ctx, source)
    key = (source, *tuple(round(float(params[k]), 9) for k in _GT_KEYS))
    gt = ctx.gt_cache.get(key)
    if gt is None:
        gt = rollout._prepare_gt(data, ctx.t0_ns, params)
        # Delay candidates change between trials. Keeping only the active GT avoids
        # retaining one large rollout payload for every Optuna trial.
        ctx.gt_cache = {key: gt}
    rmse = rollout.eval_rollout_rmse(
        data, ctx.t0_ns, params, model_type, horizons=horizons, stride=ROLLOUT_STRIDE, gt=gt,
    )
    return {h: rmse[h] for h in horizons}


def _rollout_metric_is_finite(
    metric: dict, horizons: tuple[int, ...] = HORIZONS,
) -> bool:
    try:
        return all(
            math.isfinite(float(metric[h][key])) and float(metric[h][key]) >= 0.0
            for h in horizons
            for key in _RMSE_KEYS
        )
    except (KeyError, TypeError, ValueError):
        return False


def _rollout_metric_is_valid(
    metric: dict, horizons: tuple[int, ...] = HORIZONS,
) -> bool:
    if not _rollout_metric_is_finite(metric, horizons):
        return False
    try:
        informative = all(
            any(float(metric[h][key]) > 0.0 for key in _RMSE_KEYS) for h in horizons
        )
    except (KeyError, TypeError, ValueError):
        return False
    return informative


def _baseline_metric_is_valid(metric: dict) -> bool:
    return _rollout_metric_is_valid(metric) and all(
        float(metric[h]["yaw"]) > 0
        and (float(metric[h]["long"]) > 0 or float(metric[h]["lat"]) > 0)
        for h in HORIZONS
    )


def _evaluate_report_metrics(
    ctxs: list[DatasetCtx],
    *,
    baseline_params: dict,
    baseline_model_type: str,
    baseline_acceleration_source: str,
    tuned_params: dict,
    tuned_model_type: str,
    tuned_acceleration_source: str,
    horizons: tuple[int, ...] = REPORT_HORIZONS,
) -> tuple[dict[str, dict], dict[str, dict]]:
    """Evaluate dense report-only horizons without changing the search score."""
    baseline_metrics: dict[str, dict] = {}
    tuned_metrics: dict[str, dict] = {}
    for ctx in ctxs:
        baseline_metric = _eval(
            ctx,
            baseline_params,
            baseline_model_type,
            baseline_acceleration_source,
            horizons=horizons,
        )
        tuned_metric = _eval(
            ctx,
            tuned_params,
            tuned_model_type,
            tuned_acceleration_source,
            horizons=horizons,
        )
        if not _rollout_metric_is_finite(baseline_metric, horizons):
            raise RuntimeError(
                f"レポート用 baseline rollout 指標が不正です: {ctx.dataset_id}"
            )
        if not _rollout_metric_is_finite(tuned_metric, horizons):
            raise RuntimeError(
                f"レポート用 tuned rollout 指標が不正です: {ctx.dataset_id}"
            )
        baseline_metrics[ctx.dataset_id] = baseline_metric
        tuned_metrics[ctx.dataset_id] = tuned_metric
    return baseline_metrics, tuned_metrics


def _finite_robust_score(aggregate: dict | None) -> float:
    if aggregate is None:
        return math.inf
    score = robust_score(aggregate)
    return score if math.isfinite(score) and score >= 0.0 else math.inf


def _load_dataset_ctx(
    ds_id: str,
    csv_path: Path,
    *,
    baseline_model_type: str,
    baseline_params: dict,
    baseline_acceleration_source: str,
) -> tuple[DatasetCtx | None, str]:
    """1 dataset を読み込み、baseline を検証した結果と失敗理由を返す。"""
    try:
        dfs = read_dataset_csv(csv_path)
        data = build_rollout_data(dfs, acceleration_source=baseline_acceleration_source)
        t0_ns = rollout.find_autonomous_start(data)
        base = rollout.build_params()
        ctx = DatasetCtx(ds_id, dfs, t0_ns, base)
        ctx.data_cache[normalize_accel_source(baseline_acceleration_source)] = data
        ctx.base_metric = _eval(
            ctx,
            baseline_params,
            baseline_model_type,
            baseline_acceleration_source,
        )
        if not _baseline_metric_is_valid(ctx.base_metric):
            reason = "baseline 誤差が無効 (yaw/縦/横≤0 or NaN)"
            print(f"[SKIP] {ds_id}: {reason}", file=sys.stderr)
            return None, reason
    except Exception as e:  # noqa: BLE001
        reason = f"ロード失敗 ({type(e).__name__}: {e})"
        print(f"[SKIP] {ds_id}: {reason}", file=sys.stderr)
        return None, reason
    return ctx, ""


# fork プールワーカーが参照する globals。fork 前に親 (load_datasets) がセットし、
# 子プロセスは COW で読み取り専用に継承する (multiprocessing.Pool.imap はタスクを
# pickle 転送するため、これらをクロージャで束ねると "can't pickle local object" になる)。
_LOAD_BASELINE_MODEL: str = ""
_LOAD_BASELINE_PARAMS: dict = {}
_LOAD_BASELINE_ACCEL_SOURCE: str = ""


def _load_one(args: tuple[str, Path]) -> tuple[DatasetCtx | None, str]:
    ds_id, csv_path = args
    return _load_dataset_ctx(
        ds_id, csv_path,
        baseline_model_type=_LOAD_BASELINE_MODEL,
        baseline_params=_LOAD_BASELINE_PARAMS,
        baseline_acceleration_source=_LOAD_BASELINE_ACCEL_SOURCE,
    )


def load_datasets(
    tasks: list[tuple[str, Path]],
    n_jobs: int = 1,
    *,
    baseline_model_type: str,
    baseline_params: dict,
    baseline_acceleration_source: str,
) -> tuple[list[DatasetCtx], list[dict[str, str]]]:
    """収集された CSV キャッシュを読み込み DatasetCtx を構築する (baseline 誤差も計算)。"""
    if n_jobs <= 1:
        ctxs: list[DatasetCtx] = []
        failures: list[dict[str, str]] = []
        for ds_id, csv_path in tasks:
            ctx, reason = _load_dataset_ctx(
                ds_id, csv_path,
                baseline_model_type=baseline_model_type,
                baseline_params=baseline_params,
                baseline_acceleration_source=baseline_acceleration_source,
            )
            if ctx is not None:
                ctxs.append(ctx)
            elif reason:
                failures.append({"dataset_id": ds_id, "reason": reason})
        print(f"[INFO] ロード完了: {len(ctxs)}/{len(tasks)} ({len(tasks) - len(ctxs)} SKIP)", file=sys.stderr)
        return ctxs, failures

    import multiprocessing

    global _LOAD_BASELINE_MODEL, _LOAD_BASELINE_PARAMS, _LOAD_BASELINE_ACCEL_SOURCE  # noqa: PLW0603
    _LOAD_BASELINE_MODEL = baseline_model_type
    _LOAD_BASELINE_PARAMS = dict(baseline_params or {})
    _LOAD_BASELINE_ACCEL_SOURCE = normalize_accel_source(baseline_acceleration_source)

    mp_ctx = multiprocessing.get_context("fork")
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    chunksize = pool_chunksize(len(tasks), n_workers)
    with mp_ctx.Pool(n_workers) as pool:
        results = list(pool.imap(_load_one, tasks, chunksize=chunksize))

    ctxs = []
    failures = []
    for (ctx, reason), (ds_id, _csv_path) in zip(results, tasks):
        if ctx is not None:
            ctxs.append(ctx)
        elif reason:
            failures.append({"dataset_id": ds_id, "reason": reason})
    n_skip = len(failures)
    print(f"[INFO] ロード完了: {len(ctxs)}/{len(tasks)} ({n_skip} SKIP)", file=sys.stderr)
    return ctxs, failures


def _evaluate_candidate(
    ctxs: list[DatasetCtx],
    params: dict,
    model_type: str,
    acceleration_source: str,
    *,
    aggregate: bool = True,
) -> dict | list[tuple[str, dict]] | None:
    per_dataset = [
        (ctx.dataset_id, _eval(ctx, params, model_type, acceleration_source))
        for ctx in ctxs
    ]
    if not all(_rollout_metric_is_valid(metric) for _, metric in per_dataset):
        if aggregate:
            return None
        raise RuntimeError("最終パラメータの rollout 指標が不正です")
    if not aggregate:
        return per_dataset
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    return aggregate_normalized(per_dataset, baselines)


_SEARCH_CTXS: list[DatasetCtx] = []
_SEARCH_MODEL = ""
_SEARCH_ACCEL_SOURCE = ""


def _eval_search_candidate(params: dict) -> float:
    """fork worker entrypoint; dataset data is inherited read-only via COW."""
    return _finite_robust_score(
        _evaluate_candidate(
            _SEARCH_CTXS,
            params,
            _SEARCH_MODEL,
            _SEARCH_ACCEL_SOURCE,
        )
    )


def robust_search(
    ctxs: list[DatasetCtx], cfg, *, n_trials: int = 200, n_jobs: int = 1,
    phase2_params: dict,
) -> dict:
    """Optuna TPE でデータセット横断ロバスト最適化する。"""
    global _SEARCH_CTXS, _SEARCH_MODEL, _SEARCH_ACCEL_SOURCE  # noqa: PLW0603

    if n_trials < 1:
        raise ValueError("n_trials must be at least 1")
    cur_case = cfg.find_case(TARGET_MODEL_NAME)
    cur_best = dict(cur_case.params)
    cur_model = cur_case.vehicle_model_type
    cur_accel_source = cur_case.acceleration_source

    delay_candidates = SEARCH_DELAY_CANDIDATES

    continuous_space = {**SEARCH_SPACE_STEER, **SEARCH_SPACE_ACC}
    searched = set(continuous_space) | {"acc_time_delay"}
    passthrough = {
        key: value for key, value in phase2_params.items() if key not in searched
    }
    cur_best.update(passthrough)
    if passthrough:
        print(f"[fit_merge] 直接同定値を透過 (Optuna 非探索): {sorted(passthrough)}")

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    sampler = optuna.samplers.TPESampler(seed=42)
    delay_list = list(delay_candidates)

    def _make_enqueue(params: dict) -> dict:
        eq = {
            key: min(upper, max(lower, float(params[key])))
            for key, (lower, upper) in continuous_space.items()
            if key in params
        }
        value = float(params.get("acc_time_delay", ctxs[0].base["acc_time_delay"]))
        eq["acc_time_delay"] = min(
            delay_list, key=lambda candidate: abs(candidate - value)
        )
        return eq

    init_agg = _evaluate_candidate(ctxs, cur_best, cur_model, cur_accel_source)
    init_score = _finite_robust_score(init_agg)
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=n_trials)
    print(f"\n## Optuna TPE ({TARGET_MODEL_NAME}, {n_trials} trials, {n_workers} jobs)")

    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.enqueue_trial(_make_enqueue(phase2_params))
    best_params = dict(cur_best)
    best_score = init_score

    def _sample(trial: optuna.Trial) -> dict:
        params = dict(cur_best)
        for pname, (lo, hi) in continuous_space.items():
            params[pname] = trial.suggest_float(pname, lo, hi)
        params["acc_time_delay"] = trial.suggest_categorical(
            "acc_time_delay", delay_candidates
        )
        return params

    executor = None
    completed = 0
    try:
        if n_workers > 1:
            import multiprocessing
            from concurrent.futures import ProcessPoolExecutor

            _SEARCH_CTXS = ctxs
            _SEARCH_MODEL = cur_model
            _SEARCH_ACCEL_SOURCE = cur_accel_source
            executor = ProcessPoolExecutor(
                max_workers=n_workers,
                mp_context=multiprocessing.get_context("fork"),
                initializer=set_worker_thread_env_defaults,
            )
        while completed < n_trials:
            batch_size = min(n_workers, n_trials - completed)
            trials = [study.ask() for _ in range(batch_size)]
            candidates = [_sample(trial) for trial in trials]
            if executor is None:
                scores = [
                    _finite_robust_score(
                        _evaluate_candidate(ctxs, params, cur_model, cur_accel_source)
                    )
                    for params in candidates
                ]
            else:
                scores = list(executor.map(_eval_search_candidate, candidates))

            for trial, params, score in zip(trials, candidates, scores):
                study.tell(trial, score)
                if score < best_score:
                    best_params = dict(params)
                    best_score = score
                completed += 1
                print(
                    f"trial {trial.number + 1:3d}/{n_trials}  score={score:.4f}  "
                    f"best={best_score:.4f}  {trial.params}"
                )
    finally:
        if executor is not None:
            executor.shutdown()
        _SEARCH_CTXS = []
        _SEARCH_MODEL = ""
        _SEARCH_ACCEL_SOURCE = ""

    if not math.isfinite(best_score):
        raise RuntimeError("全ての探索候補で有限な rollout 指標を計算できませんでした")
    return best_params


def fit_merge(
    collection_dir: Path,
    scenario: Path,
    *,
    phase2_params: dict,
    n_trials: int = 50,
    n_jobs: int = 1,
    metrics_out: Path,
) -> dict:
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    baseline_accel_source = cfg.models[baseline_case].acceleration_source
    cur_case = cfg.find_case(TARGET_MODEL_NAME)
    cur_model = cur_case.vehicle_model_type
    print(
        f"[INFO] baseline model = {baseline_model_type} "
        f"(scenario.yaml の '{baseline_case}' ケース, accel_source={baseline_accel_source})"
    )
    print(
        f"[INFO] current model  = {cur_model} "
        f"('{TARGET_MODEL_NAME}' ケース, accel_source={cur_case.acceleration_source})"
    )

    set_worker_thread_env_defaults()

    ctxs, fit_skipped = load_datasets(
        tasks,
        n_jobs=n_jobs,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=baseline_accel_source,
    )
    if len(ctxs) < 1:
        raise RuntimeError("有効な dataset が 0 件です")

    tuned_params = robust_search(
        ctxs, cfg, n_trials=n_trials, n_jobs=n_jobs, phase2_params=phase2_params,
    )
    # 1. Evaluate baseline
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    baseline_metrics = [(ctx.dataset_id, ctx.base_metric) for ctx in ctxs]
    baseline_agg = aggregate_normalized(baseline_metrics, baselines)
    baseline_score = _finite_robust_score(baseline_agg)

    # 2. Evaluate tuned (current)
    merged_tuned_params = merge_vehicle_model_params(ctxs[0].base, tuned_params, cur_model)
    model_keys = get_vehicle_model_spec(cur_model).param_keys
    full_tuned_params = {
        key: merged_tuned_params[key]
        for key in sorted(model_keys)
        if key in merged_tuned_params
    }
    tuned_metrics_list = _evaluate_candidate(
        ctxs,
        full_tuned_params,
        cur_model,
        cur_case.acceleration_source,
        aggregate=False,
    )
    tuned_agg = aggregate_normalized(tuned_metrics_list, baselines)
    tuned_score = _finite_robust_score(tuned_agg)
    if not math.isfinite(tuned_score):
        raise RuntimeError("最終パラメータで有限な rollout 指標を計算できませんでした")

    comparison_results = {}

    def clean_agg(agg_dict: dict) -> dict:
        clean = {}
        for h, v in agg_dict["by_h"].items():
            clean[int(h)] = {k: float(val) for k, val in v.items()}
        return clean

    comparison_results["baseline"] = {
        "score": float(baseline_score),
        "by_h": clean_agg(baseline_agg),
        "acceleration_source": baseline_accel_source,
    }

    comparison_results["tuned"] = {
        "score": float(tuned_score),
        "by_h": clean_agg(tuned_agg),
        "acceleration_source": cur_case.acceleration_source,
    }

    # Print N-step rollout comparison summary to console
    print("\n" + "=" * 72)
    print("N-step Rollout Performance Comparison (All Valid Datasets)")
    print("=" * 72)
    for name, data in comparison_results.items():
        dummy_agg = {"by_h": data["by_h"]}
        print(f"  {format_agg(name, dummy_agg)}  score={data['score']:.4f}")
    print("=" * 72 + "\n")

    print(
        f"[INFO] レポート用 rollout を N={REPORT_HORIZONS[0]}.."
        f"{REPORT_HORIZONS[-1]} で評価"
    )
    report_baselines, report_tuned = _evaluate_report_metrics(
        ctxs,
        baseline_params=baseline_params,
        baseline_model_type=baseline_model_type,
        baseline_acceleration_source=baseline_accel_source,
        tuned_params=full_tuned_params,
        tuned_model_type=cur_model,
        tuned_acceleration_source=cur_case.acceleration_source,
    )

    fieldnames = [
        "dataset_id",
        "model",
        "horizon",
        "pos",
        "long",
        "lat",
        "yaw",
        "steer",
        "vx",
        "ax",
    ]
    metrics_out.parent.mkdir(parents=True, exist_ok=True)
    with metrics_out.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for ctx in ctxs:
            for model, per_horizon in (
                ("baseline", report_baselines[ctx.dataset_id]),
                ("tuned", report_tuned[ctx.dataset_id]),
            ):
                for horizon in REPORT_HORIZONS:
                    writer.writerow(
                        {
                            "dataset_id": ctx.dataset_id,
                            "model": model,
                            "horizon": horizon,
                            **{
                                metric: float(per_horizon[horizon][metric])
                                for metric in fieldnames[3:]
                            },
                        }
                    )
    print(f"[INFO] dataset別 metrics 保存: {metrics_out}")

    return {
        "params": full_tuned_params,
        "score": float(tuned_score),
        "comparison": comparison_results,
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": len(ctxs),
            "scenario": str(scenario),
            "vehicle_model_type": cur_model,
            "score_horizons": list(HORIZONS),
            "report_horizons": list(REPORT_HORIZONS),
            "skipped": fit_skipped,
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        },
    }


def run(
    collection_dir: Path, scenario: Path, out: Path, *,
    phase2_params_path: Path, metrics_out: Path,
    n_trials: int = 50, n_jobs: int = 1,
) -> dict:
    if not phase2_params_path.is_file():
        raise FileNotFoundError(f"直接同定結果が見つかりません: {phase2_params_path}")
    with phase2_params_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("params"), dict):
        raise ValueError(f"直接同定結果の params が不正です: {phase2_params_path}")
    phase2_params = dict(data["params"])
    missing = _DIRECT_FIT_KEYS - phase2_params.keys()
    if missing:
        raise ValueError(f"直接同定結果に必須 params がありません: {sorted(missing)}")
    invalid = []
    for key in _DIRECT_FIT_KEYS:
        value = phase2_params[key]
        valid = (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )
        if not valid:
            invalid.append(key)
    if invalid:
        raise ValueError(f"直接同定結果の params が有限数値ではありません: {sorted(invalid)}")
    metadata = data.get("metadata")
    if not isinstance(metadata, dict) or metadata.get("phase") != 2:
        raise ValueError(f"直接同定結果の metadata.phase は 2 である必要があります: {phase2_params_path}")
    print(f"[fit_merge] 直接同定値を warm-start / passthrough に使用: {phase2_params_path}")

    out.parent.mkdir(parents=True, exist_ok=True)
    result = fit_merge(
        collection_dir, scenario, phase2_params=phase2_params,
        n_trials=n_trials, n_jobs=n_jobs, metrics_out=metrics_out,
    )
    with out.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(result, stream, allow_unicode=True, sort_keys=False)
    print(f"[INFO] FINAL params 保存: {out}")
    return result
