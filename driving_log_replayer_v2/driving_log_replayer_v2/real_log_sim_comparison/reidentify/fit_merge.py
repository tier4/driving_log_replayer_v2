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
from ..lib._steer_source import normalize_steer_source
from ..lib._multi_agg import ACT_W, AX_FLOOR_MPS2, STEER_FLOOR_DEG, aggregate_normalized, format_agg, robust_score
from ..lib._nstep_common import METRIC_KEYS
from ..lib._parallel import (
    DEFAULT_IMAP_WATCHDOG_TIMEOUT_S,
    imap_with_watchdog,
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from ..lib._vehicle_models import get_vehicle_model_spec, merge_vehicle_model_params
from . import rollout
from .load_data import build_rollout_data, discover_cached_datasets, read_dataset_csv
from .model_config import comparison_display_order, load_model_config, resolve_baseline_model
from .parameter_constraints import (
    ALL_CONSTRAINED_KEYS,
    PARAMETER_CONSTRAINTS,
    FIT_MERGE,
    build_constraint_audit,
    clamp_search_parameters,
    search_constraints,
    validate_parameters,
)
from .settings import (
    ACT_SCORE_HORIZONS,
    BASELINE_MODEL_NAME,
    HORIZONS,
    ROLLOUT_STRIDE,
    TUNED_MODEL_DISPLAY_NAME,
)
from .stage_common import read_phase_artifact

_GT_KEYS = ("acc_time_delay", "steer_time_delay", "wheelbase", "sub_dt")
_RMSE_KEYS = METRIC_KEYS
# Keep the optimization objective on the established sparse horizons, while
# exporting every available integer N up to the maximum score horizon for the
# comparison report. A dataset accepted by the sparse evaluation has valid
# coverage at that maximum, so all smaller horizons are available as well.
REPORT_HORIZONS = tuple(range(1, max(HORIZONS) + 1))
# fit_merge の入力成果物は制約登録済みの全パラメータを持っていなければならない。
_DIRECT_FIT_KEYS = ALL_CONSTRAINED_KEYS


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


def _ctx_data(
    ctx: DatasetCtx, acceleration_source: str, steering_source: str = "steer",
) -> dict:
    key = (normalize_accel_source(acceleration_source), normalize_steer_source(steering_source))
    if key not in ctx.data_cache:
        ctx.data_cache[key] = build_rollout_data(
            ctx.dfs, acceleration_source=key[0], steering_source=key[1],
        )
    return ctx.data_cache[key]


def _eval(
    ctx: DatasetCtx, override: dict, model_type: str, acceleration_source: str = "accel",
    steering_source: str = "steer",
    *, horizons: tuple[int, ...] = HORIZONS, include_mean: bool = False,
) -> dict:
    """1 dataset・1 パラメータ組の horizon 別終端誤差 RMSE {h: {yaw,pos,long,lat,steer}}。

    include_mean=True のとき署名付き平均誤差 (MEAN_METRIC_KEYS) も含む (診断用)。
    """
    params = merge_vehicle_model_params(ctx.base, override, model_type)
    source = normalize_accel_source(acceleration_source)
    steer_source = normalize_steer_source(steering_source)
    data = _ctx_data(ctx, source, steer_source)
    key = (source, steer_source, *tuple(round(float(params[k]), 9) for k in _GT_KEYS))
    gt = ctx.gt_cache.get(key)
    if gt is None:
        gt = rollout._prepare_gt(data, ctx.t0_ns, params)
        # Delay candidates change between trials. Keeping only the active GT avoids
        # retaining one large rollout payload for every Optuna trial.
        ctx.gt_cache = {key: gt}
    rmse = rollout.eval_rollout_rmse(
        data, ctx.t0_ns, params, model_type, horizons=horizons, stride=ROLLOUT_STRIDE, gt=gt,
        include_mean=include_mean,
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
    baseline_steering_source: str = "steer",
    tuned_params: dict,
    tuned_model_type: str,
    tuned_acceleration_source: str,
    tuned_steering_source: str = "steer",
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
            baseline_steering_source,
            horizons=horizons,
        )
        tuned_metric = _eval(
            ctx,
            tuned_params,
            tuned_model_type,
            tuned_acceleration_source,
            tuned_steering_source,
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


def _evaluate_comparison_report_metrics(
    ctxs: list[DatasetCtx],
    comparison_cases: list[tuple[str, dict, str, str, str]],
    *,
    horizons: tuple[int, ...] = REPORT_HORIZONS,
) -> dict[str, dict[str, dict]]:
    """Evaluate every scenario comparison case at the dense report horizons."""
    results: dict[str, dict[str, dict]] = {}
    for name, params, model_type, acceleration_source, steering_source in comparison_cases:
        per_dataset: dict[str, dict] = {}
        for ctx in ctxs:
            metric = _eval(
                ctx, params, model_type, acceleration_source, steering_source,
                horizons=horizons,
            )
            if not _rollout_metric_is_finite(metric, horizons):
                raise RuntimeError(f"レポート用 {name} rollout 指標が不正です: {ctx.dataset_id}")
            per_dataset[ctx.dataset_id] = metric
        results[name] = per_dataset
    return results


def _finite_robust_score(aggregate: dict | None) -> float:
    """最適化に使う目的関数 (objective v2): yaw/long/lat + steer/ax アクチュエータ項。"""
    if aggregate is None:
        return math.inf
    score = robust_score(aggregate, HORIZONS, act_horizons=ACT_SCORE_HORIZONS)
    return score if math.isfinite(score) and score >= 0.0 else math.inf


def _finite_legacy_score(aggregate: dict | None) -> float:
    """旧目的関数 (objective v1: yaw/long/lat のみ)。過去ランとの比較・非退行監査用。"""
    if aggregate is None:
        return math.inf
    score = robust_score(aggregate, HORIZONS)
    return score if math.isfinite(score) and score >= 0.0 else math.inf


def _load_dataset_ctx(
    ds_id: str,
    csv_path: Path,
    *,
    baseline_model_type: str,
    baseline_params: dict,
    baseline_acceleration_source: str,
    baseline_steering_source: str = "steer",
) -> tuple[DatasetCtx | None, str]:
    """1 dataset を読み込み、baseline を検証した結果と失敗理由を返す。"""
    try:
        dfs = read_dataset_csv(csv_path)
        cache_key = (
            normalize_accel_source(baseline_acceleration_source),
            normalize_steer_source(baseline_steering_source),
        )
        data = build_rollout_data(
            dfs, acceleration_source=cache_key[0], steering_source=cache_key[1],
        )
        t0_ns = rollout.find_autonomous_start(data)
        base = rollout.build_params()
        ctx = DatasetCtx(ds_id, dfs, t0_ns, base)
        ctx.data_cache[cache_key] = data
        ctx.base_metric = _eval(
            ctx,
            baseline_params,
            baseline_model_type,
            baseline_acceleration_source,
            baseline_steering_source,
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
_LOAD_BASELINE_STEER_SOURCE: str = "steer"


def _load_one(args: tuple[str, Path]) -> tuple[DatasetCtx | None, str]:
    ds_id, csv_path = args
    return _load_dataset_ctx(
        ds_id, csv_path,
        baseline_model_type=_LOAD_BASELINE_MODEL,
        baseline_params=_LOAD_BASELINE_PARAMS,
        baseline_acceleration_source=_LOAD_BASELINE_ACCEL_SOURCE,
        baseline_steering_source=_LOAD_BASELINE_STEER_SOURCE,
    )


def load_datasets(
    tasks: list[tuple[str, Path]],
    n_jobs: int = 1,
    *,
    baseline_model_type: str,
    baseline_params: dict,
    baseline_acceleration_source: str,
    baseline_steering_source: str = "steer",
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
                baseline_steering_source=baseline_steering_source,
            )
            if ctx is not None:
                ctxs.append(ctx)
            elif reason:
                failures.append({"dataset_id": ds_id, "reason": reason})
        print(f"[INFO] ロード完了: {len(ctxs)}/{len(tasks)} ({len(tasks) - len(ctxs)} SKIP)", file=sys.stderr)
        return ctxs, failures

    import multiprocessing

    global _LOAD_BASELINE_MODEL, _LOAD_BASELINE_PARAMS, _LOAD_BASELINE_ACCEL_SOURCE, _LOAD_BASELINE_STEER_SOURCE  # noqa: PLW0603
    _LOAD_BASELINE_MODEL = baseline_model_type
    _LOAD_BASELINE_PARAMS = dict(baseline_params or {})
    _LOAD_BASELINE_ACCEL_SOURCE = normalize_accel_source(baseline_acceleration_source)
    _LOAD_BASELINE_STEER_SOURCE = normalize_steer_source(baseline_steering_source)

    # 呼び出し元が fork 前に set_worker_thread_env_defaults() 済みであること
    # (jemalloc bg thread 保持ロックの fork-time デッドロック対策、fit_plateau.py 参照)。
    mp_ctx = multiprocessing.get_context("fork")
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    chunksize = pool_chunksize(len(tasks), n_workers)
    with mp_ctx.Pool(n_workers) as pool:
        results = imap_with_watchdog(pool, _load_one, tasks, chunksize=chunksize)

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
    steering_source: str = "steer",
    *,
    aggregate: bool = True,
) -> dict | list[tuple[str, dict]] | None:
    per_dataset = [
        (ctx.dataset_id, _eval(ctx, params, model_type, acceleration_source, steering_source))
        for ctx in ctxs
    ]
    if not all(_rollout_metric_is_valid(metric) for _, metric in per_dataset):
        if aggregate:
            return None
        raise RuntimeError("最終パラメータの rollout 指標が不正です")
    if not aggregate:
        return per_dataset
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    return aggregate_normalized(per_dataset, baselines, HORIZONS)


_SEARCH_CTXS: list[DatasetCtx] = []
_SEARCH_MODEL = ""
_SEARCH_ACCEL_SOURCE = ""
_SEARCH_STEER_SOURCE = "steer"


def _eval_search_candidate(params: dict) -> float:
    """fork worker entrypoint; dataset data is inherited read-only via COW."""
    return _finite_robust_score(
        _evaluate_candidate(
            _SEARCH_CTXS,
            params,
            _SEARCH_MODEL,
            _SEARCH_ACCEL_SOURCE,
            _SEARCH_STEER_SOURCE,
        )
    )


def robust_search(
    ctxs: list[DatasetCtx], cfg, *, n_trials: int = 200, n_jobs: int = 1,
    direct_fit_params: dict,
) -> dict:
    """Optuna TPE でデータセット横断ロバスト最適化する。"""
    global _SEARCH_CTXS, _SEARCH_MODEL, _SEARCH_ACCEL_SOURCE, _SEARCH_STEER_SOURCE  # noqa: PLW0603

    if n_trials < 1:
        raise ValueError("n_trials must be at least 1")
    cur_case = cfg.find_case(cfg.fit.target)
    cur_best = dict(cur_case.params)
    cur_model = cur_case.vehicle_model_type
    cur_accel_source = cur_case.acceleration_source
    cur_steer_source = getattr(cur_case, "steering_source", "steer")

    constraints = search_constraints(FIT_MERGE)
    continuous_space = {
        key: constraint.search_bounds
        for key, constraint in constraints.items()
        if constraint.search_bounds is not None
    }
    delay_constraint = constraints.get("acc_time_delay")
    delay_candidates = delay_constraint.search_candidates if delay_constraint else ()
    searched = set(constraints)
    validate_parameters(
        direct_fit_params,
        set(PARAMETER_CONSTRAINTS) & set(direct_fit_params),
        source="直接同定結果",
    )
    passthrough = {
        key: value for key, value in direct_fit_params.items() if key not in searched
    }
    cur_best.update(passthrough)
    validate_parameters(
        cur_best,
        set(PARAMETER_CONSTRAINTS) & set(cur_best),
        source="scenario/current の初期値",
    )
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
        if delay_constraint is not None:
            value = float(params.get("acc_time_delay", ctxs[0].base["acc_time_delay"]))
            eq["acc_time_delay"] = min(
                delay_list, key=lambda candidate: abs(candidate - value)
            )
        return clamp_search_parameters(eq, FIT_MERGE)

    init_agg = _evaluate_candidate(ctxs, cur_best, cur_model, cur_accel_source, cur_steer_source)
    init_score = _finite_robust_score(init_agg)
    if not searched:
        print("[fit_merge] 最適化対象がないため、直接同定値 / scenario 初期値を使用")
        return cur_best
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=n_trials)
    print(f"\n## Optuna TPE ({cfg.fit.target}, {n_trials} trials, {n_workers} jobs)")

    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.enqueue_trial(_make_enqueue(direct_fit_params))
    best_params = dict(cur_best)
    best_score = init_score

    def _sample(trial: optuna.Trial) -> dict:
        params = dict(cur_best)
        for pname, (lo, hi) in continuous_space.items():
            params[pname] = trial.suggest_float(pname, lo, hi)
        if delay_constraint is not None:
            params["acc_time_delay"] = trial.suggest_categorical(
                "acc_time_delay", delay_candidates
            )
        return clamp_search_parameters(params, FIT_MERGE)

    executor = None
    completed = 0
    try:
        if n_workers > 1:
            import multiprocessing
            from concurrent.futures import ProcessPoolExecutor

            # fork worker が COW で ctxs を読み取り継承する (再ロード/再pickle を避ける)。
            # pandas 連鎖 import の pyarrow が jemalloc bg thread を持つため、fork 直前に
            # set_worker_thread_env_defaults() 済みであること (呼び出し元 identify() 参照)。
            _SEARCH_CTXS = ctxs
            _SEARCH_MODEL = cur_model
            _SEARCH_ACCEL_SOURCE = cur_accel_source
            _SEARCH_STEER_SOURCE = cur_steer_source
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
                        _evaluate_candidate(ctxs, params, cur_model, cur_accel_source, cur_steer_source)
                    )
                    for params in candidates
                ]
            else:
                try:
                    # timeout: fork 直後に worker がデッドロックした場合 (jemalloc 背景
                    # スレッド等が fork 時にロックを保持していた場合など) に、応答のない
                    # 1 worker のせいで永久にハングしないようにする。TimeoutError も
                    # 下の except で捕捉し、逐次評価にフォールバックする。
                    scores = list(
                        executor.map(
                            _eval_search_candidate, candidates,
                            timeout=DEFAULT_IMAP_WATCHDOG_TIMEOUT_S,
                        )
                    )
                except Exception as exc:  # noqa: BLE001 (worker may die outside Python, or watchdog timeout)
                    # A native dependency or external signal can terminate a
                    # fork worker without returning its original exception.
                    # Preserve the already completed Optuna trials and retry
                    # this batch in the parent, which also prevents every
                    # subsequent batch from failing with BrokenProcessPool.
                    print(
                        "[WARN] 並列 rollout worker が終了しました "
                        f"({type(exc).__name__}: {exc}); このバッチ以降は逐次評価します",
                        file=sys.stderr,
                    )
                    executor.shutdown(wait=False, cancel_futures=True)
                    executor = None
                    scores = [
                        _finite_robust_score(
                            _evaluate_candidate(ctxs, params, cur_model, cur_accel_source, cur_steer_source)
                        )
                        for params in candidates
                    ]

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
    validate_parameters(best_params, set(PARAMETER_CONSTRAINTS) & set(best_params), source="最終探索候補")
    return best_params


def _clean_agg(agg_dict: dict) -> dict:
    return {
        int(h): {k: float(val) for k, val in v.items()}
        for h, v in agg_dict["by_h"].items()
    }


def _full_model_params(base: dict, params: dict, model_type: str) -> dict:
    """探索/直接同定の param を model の全キーへマージし、リリース可能な完全集合にする。"""
    merged = merge_vehicle_model_params(base, params, model_type)
    model_keys = get_vehicle_model_spec(model_type).param_keys
    return {key: merged[key] for key in sorted(model_keys) if key in merged}


def _load_ctxs(
    collection_dir: Path, cfg, n_jobs: int,
) -> tuple[int, list[DatasetCtx], list[dict[str, str]], tuple[str, dict, str, str]]:
    """CSV キャッシュを読み込み、baseline 情報付きの DatasetCtx 群を返す。"""
    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    baseline_accel_source = cfg.models[baseline_case].acceleration_source
    baseline_steer_source = cfg.models[baseline_case].steering_source
    print(
        f"[INFO] baseline model = {baseline_model_type} "
        f"(scenario.yaml の '{baseline_case}' ケース, accel_source={baseline_accel_source})"
    )

    set_worker_thread_env_defaults()
    ctxs, fit_skipped = load_datasets(
        tasks,
        n_jobs=n_jobs,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=baseline_accel_source,
        baseline_steering_source=baseline_steer_source,
    )
    if len(ctxs) < 1:
        raise RuntimeError("有効な dataset が 0 件です")
    baseline_bundle = (
        baseline_model_type, baseline_params, baseline_accel_source, baseline_steer_source,
    )
    return len(tasks), ctxs, fit_skipped, baseline_bundle


def build_comparison_document(
    collection_dir: Path,
    scenario: Path,
    cfg,
    ctxs: list[DatasetCtx],
    *,
    n_datasets: int,
    fit_skipped: list[dict[str, str]],
    baseline_bundle: tuple[str, dict, str, str],
    tuned_params: dict | None,
    metrics_out: Path,
) -> dict:
    """baseline + 固定比較ケース (+ 任意で tuned) を評価し metrics.csv と成果物 document を作る。

    ``tuned_params`` が与えられれば fit 出力を "tuned" として比較・出力に含める。None なら
    fit 非実行 (固定比較のみ) を表し、document の ``params`` は空にする。
    """
    baseline_model_type, baseline_params, baseline_accel, baseline_steer = baseline_bundle
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    baseline_metrics = [(ctx.dataset_id, ctx.base_metric) for ctx in ctxs]

    tuned_case = cfg.find_case(cfg.fit.target) if tuned_params is not None else None
    tuned_metrics_list = None
    if tuned_params is not None:
        tuned_metrics_list = _evaluate_candidate(
            ctxs, tuned_params, tuned_case.vehicle_model_type,
            tuned_case.acceleration_source, tuned_case.steering_source, aggregate=False,
        )

    display_order = comparison_display_order(cfg)
    cases: list[tuple[str, dict, str, str, str]] = []
    sparse: dict[str, list[tuple[str, dict]]] = {}
    for display_name in display_order:
        if display_name == BASELINE_MODEL_NAME:
            params, model_type = baseline_params, baseline_model_type
            accel, steer = baseline_accel, baseline_steer
            sparse[display_name] = baseline_metrics
        elif display_name == TUNED_MODEL_DISPLAY_NAME:
            params, model_type = tuned_params, tuned_case.vehicle_model_type
            accel, steer = tuned_case.acceleration_source, tuned_case.steering_source
            sparse[display_name] = tuned_metrics_list
        else:
            case = cfg.find_case(display_name)
            params, model_type = dict(case.params), case.vehicle_model_type
            accel, steer = case.acceleration_source, case.steering_source
            sparse[display_name] = _evaluate_candidate(
                ctxs, params, model_type, accel, steer, aggregate=False,
            )
        cases.append((display_name, params, model_type, accel, steer))

    comparison_results: dict[str, dict] = {}
    for display_name, _params, _model_type, accel, steer in cases:
        aggregate = aggregate_normalized(sparse[display_name], baselines, HORIZONS)
        comparison_results[display_name] = {
            "score": float(_finite_robust_score(aggregate)),
            "score_legacy": float(_finite_legacy_score(aggregate)),
            "by_h": _clean_agg(aggregate),
            "acceleration_source": accel,
            "steering_source": steer,
        }
    if tuned_params is not None and not math.isfinite(
        comparison_results[TUNED_MODEL_DISPLAY_NAME]["score"]
    ):
        raise RuntimeError("最終パラメータで有限な rollout 指標を計算できませんでした")

    # Print N-step rollout comparison summary to console
    print("\n" + "=" * 72)
    print("N-step Rollout Performance Comparison (All Valid Datasets)")
    print("=" * 72)
    for name, data in comparison_results.items():
        dummy_agg = {"by_h": data["by_h"]}
        print(
            f"  {format_agg(name, dummy_agg, HORIZONS)}  score={data['score']:.4f}"
            f" (legacy={data['score_legacy']:.4f})"
        )
    print("=" * 72 + "\n")

    print(
        f"[INFO] レポート用 rollout を N={REPORT_HORIZONS[0]}.."
        f"{REPORT_HORIZONS[-1]} で評価"
    )
    report_metrics = _evaluate_comparison_report_metrics(ctxs, cases)

    fieldnames = ["dataset_id", "model", "horizon", *METRIC_KEYS]
    metrics_out.parent.mkdir(parents=True, exist_ok=True)
    with metrics_out.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for ctx in ctxs:
            for model, _params, _model_type, _source, _steer_source in cases:
                per_horizon = report_metrics[model][ctx.dataset_id]
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

    metadata = {
        "collection_dir": str(collection_dir),
        "n_datasets": n_datasets,
        "n_valid": len(ctxs),
        "scenario": str(scenario),
        "fit_stages": list(cfg.fit.stages),
        "fit_target": cfg.fit.target if cfg.fit.enabled else None,
        "score_horizons": list(HORIZONS),
        # objective v2: yaw/long/lat に加え steer/ax のプラトー項を最適化する。
        # v1 (score_legacy) は yaw/long/lat のみ。過去ランとの score 比較時はここを確認する。
        "objective": {
            "version": 2,
            "act_horizons": list(ACT_SCORE_HORIZONS),
            "act_w": ACT_W,
            "steer_floor_deg": STEER_FLOOR_DEG,
            "ax_floor_mps2": AX_FLOOR_MPS2,
        },
        "report_horizons": list(REPORT_HORIZONS),
        "skipped": fit_skipped,
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    result: dict = {"comparison": comparison_results, "metadata": metadata}
    if tuned_params is not None:
        metadata["vehicle_model_type"] = tuned_case.vehicle_model_type
        result["params"] = tuned_params
        result["score"] = comparison_results[TUNED_MODEL_DISPLAY_NAME]["score"]
        result["parameter_constraints"] = build_constraint_audit(tuned_params)
    else:
        # fit 非実行: report._load_tuned_document が params mapping を要求するため空で置く。
        result["params"] = {}
    return result


def fit_merge(
    collection_dir: Path,
    scenario: Path,
    *,
    direct_fit_params: dict,
    n_trials: int = 50,
    n_jobs: int = 1,
    metrics_out: Path,
) -> dict:
    cfg = load_model_config(scenario)
    cur_case = cfg.find_case(cfg.fit.target)
    print(
        f"[INFO] fit target = {cur_case.vehicle_model_type} "
        f"('{cfg.fit.target}' ケース, accel_source={cur_case.acceleration_source})"
    )

    n_datasets, ctxs, fit_skipped, baseline_bundle = _load_ctxs(collection_dir, cfg, n_jobs)

    tuned_params = robust_search(
        ctxs, cfg, n_trials=n_trials, n_jobs=n_jobs, direct_fit_params=direct_fit_params,
    )
    full_tuned_params = _full_model_params(
        ctxs[0].base, tuned_params, cur_case.vehicle_model_type,
    )
    return build_comparison_document(
        collection_dir, scenario, cfg, ctxs,
        n_datasets=n_datasets, fit_skipped=fit_skipped, baseline_bundle=baseline_bundle,
        tuned_params=full_tuned_params, metrics_out=metrics_out,
    )


def evaluate_only(
    collection_dir: Path,
    scenario: Path,
    *,
    direct_fit_params_path: Path | None,
    n_jobs: int = 1,
    metrics_out: Path,
) -> dict:
    """merge を実行しない評価専用パス (fit 完全スキップ or direct-fit のみ)。

    ``direct_fit_params_path`` が与えられれば、その direct-fit 成果物を "tuned" として
    評価に含める。None なら固定比較ケースのみ評価する。
    """
    cfg = load_model_config(scenario)
    n_datasets, ctxs, fit_skipped, baseline_bundle = _load_ctxs(collection_dir, cfg, n_jobs)

    tuned_params = None
    if direct_fit_params_path is not None:
        direct_fit_params = read_phase_artifact(
            direct_fit_params_path,
            expected_phase=_LAST_DIRECT_PHASE[cfg.fit.direct_stages[-1]],
            required_keys=frozenset(),
            producer="direct fit",
        )
        target_case = cfg.find_case(cfg.fit.target)
        tuned_params = _full_model_params(
            ctxs[0].base, direct_fit_params, target_case.vehicle_model_type,
        )
        print(f"[evaluate] direct-fit 成果物を tuned として評価: {direct_fit_params_path}")
    else:
        print("[evaluate] fit 非実行: 固定比較ケースのみ評価します")

    return build_comparison_document(
        collection_dir, scenario, cfg, ctxs,
        n_datasets=n_datasets, fit_skipped=fit_skipped, baseline_bundle=baseline_bundle,
        tuned_params=tuned_params, metrics_out=metrics_out,
    )


# direct-fit ステージ名 -> その成果物 metadata.phase (evaluate_only の read 検証用)。
_LAST_DIRECT_PHASE = {"lon": 1, "steer": 2, "xy": 3}


def run(
    collection_dir: Path, scenario: Path, out: Path, *,
    phase3_params_path: Path, metrics_out: Path,
    n_trials: int = 50, n_jobs: int = 1,
) -> dict:
    direct_fit_params = read_phase_artifact(
        phase3_params_path, expected_phase=3, required_keys=_DIRECT_FIT_KEYS, producer="fit_xy",
    )
    validate_parameters(direct_fit_params, _DIRECT_FIT_KEYS, source="直接同定結果")
    print(f"[fit_merge] 直接同定値を warm-start / passthrough に使用: {phase3_params_path}")

    out.parent.mkdir(parents=True, exist_ok=True)
    result = fit_merge(
        collection_dir, scenario, direct_fit_params=direct_fit_params,
        n_trials=n_trials, n_jobs=n_jobs, metrics_out=metrics_out,
    )
    with out.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(result, stream, allow_unicode=True, sort_keys=False)
    print(f"[INFO] FINAL params 保存: {out}")
    return result


def run_evaluate(
    collection_dir: Path, scenario: Path, out: Path, *,
    direct_fit_params_path: Path | None, metrics_out: Path, n_jobs: int = 1,
) -> dict:
    """merge を実行しない評価専用ステージ。成果物 document を out へ書き出す。"""
    out.parent.mkdir(parents=True, exist_ok=True)
    result = evaluate_only(
        collection_dir, scenario,
        direct_fit_params_path=direct_fit_params_path, n_jobs=n_jobs, metrics_out=metrics_out,
    )
    with out.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(result, stream, allow_unicode=True, sort_keys=False)
    print(f"[INFO] 評価成果物 保存: {out}")
    return result
