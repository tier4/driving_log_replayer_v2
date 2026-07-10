#!/usr/bin/env python3
"""[Step4a] Optuna warm-start 統合チューニング → tuned_params.yaml。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import datetime
import os
from pathlib import Path
import sys
import traceback

import optuna
import yaml

from ..lib._collection import discover_collection
from ..lib._accel_source import normalize_accel_source
from ..lib._models_config import load_models_doc, resolve_baseline_model
from ..lib._multi_agg import HORIZONS, WORST_W, acc_score, aggregate_normalized, format_agg, robust_score, steer_score
from ..lib._parallel import (
    default_parallel_jobs,
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from ..lib._vehicle_models import merge_vehicle_model_params
from . import rollout
from .csv_schema import CACHE_NAME
from .load_data import build_rollout_data, read_dataset_csv
from .settings import ROLLOUT_STRIDE, SEARCH_DELAY_CANDIDATES, SEARCH_SPACE_ACC, SEARCH_SPACE_STEER

_GT_KEYS = ("acc_time_delay", "steer_time_delay", "wheelbase", "sub_dt")


@dataclass
class DatasetCtx:
    """1 データセットの rollout 実行コンテキスト。"""

    dataset_id: str
    dfs: dict
    data: dict
    t0_ns: int
    base: dict
    data_cache: dict[str, dict] = field(default_factory=dict)
    gt_cache: dict = field(default_factory=dict)
    base_metric: dict = field(default_factory=dict)
    n_cmd_samples: int = 0
    n_drive_samples: int = 0


def _ctx_data(ctx: DatasetCtx, acceleration_source: str) -> dict:
    source = normalize_accel_source(acceleration_source)
    if source not in ctx.data_cache:
        ctx.data_cache[source] = build_rollout_data(ctx.dfs, acceleration_source=source)
    return ctx.data_cache[source]


def _eval(
    ctx: DatasetCtx, override: dict, model_type: str, acceleration_source: str = "accel",
) -> dict:
    """1 dataset・1 パラメータ組の horizon 別終端誤差 RMSE {h: {yaw,pos,long,lat,steer}}。"""
    params = merge_vehicle_model_params(ctx.base, override, model_type)
    source = normalize_accel_source(acceleration_source)
    data = _ctx_data(ctx, source)
    key = (source, *tuple(round(float(params[k]), 9) for k in _GT_KEYS))
    gt = ctx.gt_cache.get(key)
    if gt is None:
        gt = rollout._prepare_gt(data, ctx.t0_ns, params)
        ctx.gt_cache[key] = gt
    rmse = rollout.eval_rollout_rmse(
        data, ctx.t0_ns, params, model_type, horizons=HORIZONS, stride=ROLLOUT_STRIDE, gt=gt,
    )
    return {h: rmse[h] for h in HORIZONS}


def _baseline_metric_is_valid(metric: dict) -> bool:
    return all(metric[h]["yaw"] > 0 and (metric[h]["long"] > 0 or metric[h]["lat"] > 0) for h in HORIZONS)


def _baseline_metric_summary(metric: dict) -> str:
    return "  ".join(
        f"baseline@N{h} yaw={metric[h]['yaw']:.4f} 縦={metric[h]['long']:.3f} 横={metric[h]['lat']:.3f}"
        for h in HORIZONS
    )


def _gear_metric_summary(ctx: DatasetCtx) -> str:
    if ctx.n_cmd_samples <= 0:
        return "gear=0/0"
    pct = 100.0 * ctx.n_drive_samples / ctx.n_cmd_samples
    return f"gear DRIVE={ctx.n_drive_samples}/{ctx.n_cmd_samples} ({pct:.1f}%)"


def _load_dataset_ctx(
    ds_id: str,
    csv_path: Path,
    *,
    verbose: bool = False,
    log: bool = True,
    baseline_model_type: str,
    baseline_params: dict | None = None,
    baseline_acceleration_source: str = "accel",
) -> DatasetCtx | None:
    """1 dataset を読み込み、baseline を検証した DatasetCtx を返す。失敗時は None。"""
    try:
        dfs = read_dataset_csv(csv_path)
        data = build_rollout_data(dfs, acceleration_source=baseline_acceleration_source)
        t0_ns = rollout.find_autonomous_start(data)
        base = rollout.build_params()
        ctx = DatasetCtx(ds_id, dfs, data, t0_ns, base)
        ctx.data_cache[normalize_accel_source(baseline_acceleration_source)] = data
        ctx.base_metric = _eval(
            ctx,
            baseline_params or {},
            baseline_model_type,
            baseline_acceleration_source,
        )
        if ctx.gt_cache:
            gt0 = next(iter(ctx.gt_cache.values()))
            valid_gear = gt0.get("valid_gear_arr", [])
            ctx.n_cmd_samples = int(len(valid_gear))
            ctx.n_drive_samples = int(sum(bool(v) for v in valid_gear))
        if not _baseline_metric_is_valid(ctx.base_metric):
            print(f"[SKIP] {ds_id}: baseline 誤差が無効 (yaw/縦/横≤0 or NaN)", file=sys.stderr)
            return None
    except Exception as e:  # noqa: BLE001
        msg = f"[SKIP] {ds_id}: ロード失敗 ({type(e).__name__}: {e})"
        if verbose:
            tb_lines = traceback.format_exc().strip().splitlines()
            msg += "\n  " + "\n  ".join(tb_lines[-3:])
        print(msg, file=sys.stderr)
        return None

    if log:
        print(f"[load] {ds_id}: {_gear_metric_summary(ctx)}  {_baseline_metric_summary(ctx.base_metric)}")
    return ctx


# fork プールワーカーが参照する globals。fork 前に親 (load_datasets) がセットし、
# 子プロセスは COW で読み取り専用に継承する (multiprocessing.Pool.imap はタスクを
# pickle 転送するため、これらをクロージャで束ねると "can't pickle local object" になる)。
_LOAD_VERBOSE: bool = False
_LOAD_BASELINE_MODEL: str = ""
_LOAD_BASELINE_PARAMS: dict = {}
_LOAD_BASELINE_ACCEL_SOURCE: str = "accel"


def _load_one(args: tuple[str, Path]) -> DatasetCtx | None:
    ds_id, csv_path = args
    return _load_dataset_ctx(
        ds_id, csv_path, verbose=_LOAD_VERBOSE, log=False,
        baseline_model_type=_LOAD_BASELINE_MODEL,
        baseline_params=_LOAD_BASELINE_PARAMS,
        baseline_acceleration_source=_LOAD_BASELINE_ACCEL_SOURCE,
    )


def load_datasets(
    tasks: list[tuple[str, Path]],
    n_jobs: int = 1,
    verbose: bool = False,
    *,
    baseline_model_type: str,
    baseline_params: dict | None = None,
    baseline_acceleration_source: str = "accel",
) -> list[DatasetCtx]:
    """収集された CSV キャッシュを読み込み DatasetCtx を構築する (baseline 誤差も計算)。"""
    if n_jobs <= 1:
        ctxs: list[DatasetCtx] = []
        for ds_id, csv_path in tasks:
            ctx = _load_dataset_ctx(
                ds_id, csv_path, verbose=verbose, log=verbose,
                baseline_model_type=baseline_model_type,
                baseline_params=baseline_params,
                baseline_acceleration_source=baseline_acceleration_source,
            )
            if ctx is not None:
                ctxs.append(ctx)
        print(f"[INFO] ロード完了: {len(ctxs)}/{len(tasks)} ({len(tasks) - len(ctxs)} SKIP)", file=sys.stderr)
        return ctxs

    import multiprocessing

    global _LOAD_VERBOSE, _LOAD_BASELINE_MODEL, _LOAD_BASELINE_PARAMS, _LOAD_BASELINE_ACCEL_SOURCE  # noqa: PLW0603
    _LOAD_VERBOSE = verbose
    _LOAD_BASELINE_MODEL = baseline_model_type
    _LOAD_BASELINE_PARAMS = dict(baseline_params or {})
    _LOAD_BASELINE_ACCEL_SOURCE = normalize_accel_source(baseline_acceleration_source)

    mp_ctx = multiprocessing.get_context("fork")
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(tasks))
    chunksize = pool_chunksize(len(tasks), n_workers)
    with mp_ctx.Pool(n_workers) as pool:
        results = list(pool.imap(_load_one, tasks, chunksize=chunksize))

    ctxs = []
    for ctx in results:
        if ctx is not None:
            ctxs.append(ctx)
            if verbose:
                print(f"[load] {ctx.dataset_id}: {_gear_metric_summary(ctx)}  {_baseline_metric_summary(ctx.base_metric)}")
    n_skip = sum(1 for r in results if r is None)
    print(f"[INFO] ロード完了: {len(ctxs)}/{len(tasks)} ({n_skip} SKIP)", file=sys.stderr)
    return ctxs


# 並列評価サポート (fork プールによる per-(trial × dataset) 並列化)

_CTXS: list[DatasetCtx] = []


def _worker_eval_one(args: tuple[int, int, dict, str, str]) -> tuple[int, int, str, dict]:
    trial_idx, ctx_idx, override, model_type, acceleration_source = args
    ctx = _CTXS[ctx_idx]
    metrics = _eval(ctx, override, model_type, acceleration_source)
    return trial_idx, ctx_idx, ctx.dataset_id, metrics


def _eval_grid(
    pool, ctxs: list[DatasetCtx], trials: list[dict], model_type: str, n_jobs: int = 1,
    agg_fn=None, acceleration_source: str = "accel",
) -> list[dict]:
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    _agg = agg_fn if agg_fn is not None else (lambda pm, bs: aggregate_normalized(pm, bs))

    if pool is None:
        result = []
        for t in trials:
            per_ds = [
                (ctx.dataset_id, _eval(ctx, t, model_type, acceleration_source))
                for ctx in ctxs
            ]
            result.append(_agg(per_ds, baselines))
        return result

    n_trials = len(trials)
    n_ctxs = len(ctxs)
    units = [
        (ti, ci, trials[ti], model_type, acceleration_source)
        for ti in range(n_trials)
        for ci in range(n_ctxs)
    ]
    chunksize = pool_chunksize(len(units), n_jobs)
    raw = pool.map(_worker_eval_one, units, chunksize=chunksize)

    per_trial: list[list[tuple[str, dict]]] = [[] for _ in range(n_trials)]
    for ti, _ci, ds_id, metrics in raw:
        per_trial[ti].append((ds_id, metrics))
    return [_agg(pm, baselines) for pm in per_trial]


def _run_worker(
    worker_id: int, db_url: str, n_trials_w: int, n_trials: int, cur_best: dict,
    continuous_space: dict, explore_delay: bool, delay_candidates: tuple[float, ...],
    ctxs_search: list[DatasetCtx], cur_model: str, cur_accel_source: str, score_fn, worst_w: float, out_path,
) -> None:
    """fork プールワーカー: SQLite 経由で Optuna study.optimize を並列実行。"""
    set_worker_thread_env_defaults()
    study = optuna.load_study(study_name="robust_search", storage=db_url)

    def _checkpoint(params: dict, score: float) -> None:
        if out_path is None:
            return
        tmp = out_path.with_suffix(".tmp")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(
            yaml.safe_dump({"params": params, "score": score}, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        tmp.rename(out_path)

    def worker_objective(trial: optuna.Trial) -> float:
        params = dict(cur_best)
        for pname, (lo, hi) in continuous_space.items():
            params[pname] = trial.suggest_float(pname, lo, hi)
        if explore_delay:
            params["acc_time_delay"] = trial.suggest_categorical("acc_time_delay", delay_candidates)

        agg = _eval_grid(
            None, ctxs_search, [params], cur_model, 1, agg_fn=None,
            acceleration_source=cur_accel_source,
        )[0]
        score = score_fn(agg, worst_w=worst_w)
        try:
            current_best = study.best_value
        except ValueError:
            current_best = float("inf")
        if score < current_best:
            _checkpoint(params, score)
        return score

    study.optimize(worker_objective, n_trials=n_trials_w)


def robust_search(
    ctxs: list[DatasetCtx], cfg, *, case_name: str = "current", n_trials: int = 200, n_jobs: int = 1,
    search_subsample: int | None = None, out_path: Path | None = None, extra_enqueue: list[dict] | None = None,
    worst_w: float = WORST_W, phase: int = 0, phase_fixed_params: dict | None = None,
    base_override: dict | None = None,
) -> dict:
    """Optuna TPE でデータセット横断ロバスト最適化 (連続値 + 離散 delay)。

    phase: 0=全パラメータ同時最適化(既定), 1=acc のみ/long スコア, 2=steer のみ/yaw+lat スコア。
    """
    def _checkpoint(params: dict, score: float) -> None:
        if out_path is None:
            return
        tmp = out_path.with_suffix(".tmp")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(
            yaml.safe_dump({"params": params, "score": score}, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        tmp.rename(out_path)

    ctxs_search = ctxs[:search_subsample] if search_subsample else ctxs
    cur_case = cfg.find_case(case_name)
    cur_best = dict(cur_case.params)
    cur_model = cur_case.vehicle_model_type
    cur_accel_source = cur_case.acceleration_source

    if search_subsample:
        print(f"[INFO] search_subsample={search_subsample}: 探索は ctxs[:{min(search_subsample, len(ctxs))}] を使用 (全件={len(ctxs)})。")

    delay_candidates = SEARCH_DELAY_CANDIDATES

    if phase == 1:
        continuous_space: dict[str, tuple[float, float]] = dict(SEARCH_SPACE_ACC)
        score_fn = acc_score
        explore_delay = True
        print("[Phase 1] acc パラメータ最適化 (long スコア)。steer 系は cur_best から固定。")
    elif phase == 2:
        continuous_space = dict(SEARCH_SPACE_STEER)
        score_fn = steer_score
        explore_delay = False
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 2] steer パラメータ最適化 (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
    else:
        continuous_space = {**SEARCH_SPACE_STEER, **SEARCH_SPACE_ACC}
        score_fn = robust_score
        explore_delay = True

    if base_override:
        searched = set(continuous_space) | {"acc_time_delay"}
        passthrough = {k: v for k, v in base_override.items() if k not in searched}
        if passthrough:
            cur_best.update(passthrough)
            print(f"[Phase {phase}] 直接同定値を透過 (Optuna 非探索): {sorted(passthrough)}")

    acc_delay_set: set[float] = set(delay_candidates) if explore_delay else set()
    if "acc_time_delay" in cur_best:
        acc_delay_set.add(float(cur_best["acc_time_delay"]))
    else:
        acc_delay_set.add(float(ctxs[0].base["acc_time_delay"]))
    for ctx in ctxs:
        for ad in acc_delay_set:
            merged = dict(ctx.base)
            merged.update(cur_best)
            merged["acc_time_delay"] = ad
            key = tuple(round(float(merged[k]), 9) for k in _GT_KEYS)
            if key not in ctx.gt_cache:
                ctx.gt_cache[key] = rollout._prepare_gt(ctx.data, ctx.t0_ns, merged)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    sampler = optuna.samplers.TPESampler(multivariate=True, seed=42)
    delay_list = list(delay_candidates)

    def _make_enqueue(params: dict) -> dict:
        eq: dict = {k: float(params[k]) for k in continuous_space if k in params}
        if explore_delay:
            if "acc_time_delay" in params:
                v = float(params["acc_time_delay"])
                eq["acc_time_delay"] = min(delay_list, key=lambda x: abs(x - v))
            elif ctxs and "acc_time_delay" in ctxs[0].base:
                spec_v = float(ctxs[0].base["acc_time_delay"])
                eq["acc_time_delay"] = min(delay_list, key=lambda x: abs(x - spec_v))
            else:
                eq["acc_time_delay"] = delay_list[0]
        return eq

    if n_jobs <= 1:
        init_agg = _eval_grid(
            None, ctxs_search, [cur_best], cur_model, 1, agg_fn=None,
            acceleration_source=cur_accel_source,
        )[0]
        init_score = score_fn(init_agg, worst_w=worst_w)
        print(f"\n## Optuna TPE ({case_name}, {n_trials} trials, cross-dataset normalized, worst_w={worst_w}, phase={phase})")
        best_result: dict = {"params": dict(cur_best), "score": init_score, "agg": init_agg}
        _checkpoint(cur_best, init_score)

        def objective(trial: optuna.Trial) -> float:
            params = dict(cur_best)
            for pname, (lo, hi) in continuous_space.items():
                params[pname] = trial.suggest_float(pname, lo, hi)
            if explore_delay:
                params["acc_time_delay"] = trial.suggest_categorical("acc_time_delay", delay_candidates)
            agg = _eval_grid(
                None, ctxs_search, [params], cur_model, 1, agg_fn=None,
                acceleration_source=cur_accel_source,
            )[0]
            score = score_fn(agg, worst_w=worst_w)
            if score < best_result["score"]:
                best_result.update({"params": dict(params), "score": score, "agg": agg})
                _checkpoint(params, score)
            return score

        def _log_cb(study: optuna.Study, trial: optuna.trial.FrozenTrial) -> None:
            if trial.state == optuna.trial.TrialState.COMPLETE:
                print(f"trial {trial.number + 1:3d}/{n_trials}  score={trial.value:.4f}  best={study.best_value:.4f}  {trial.params}")

        study = optuna.create_study(direction="minimize", sampler=sampler)
        study.enqueue_trial(_make_enqueue(cur_best))
        for ep in (extra_enqueue or []):
            study.enqueue_trial(_make_enqueue(ep))
        study.optimize(objective, n_trials=n_trials, callbacks=[_log_cb])

        state = best_result["params"]
        best_s = best_result["score"]
        _checkpoint(state, best_s)

        if search_subsample and len(ctxs_search) < len(ctxs):
            print(f"[INFO] 最終 score を全 {len(ctxs)} データセットで評価中...")
            full_agg = _eval_grid(
                None, ctxs, [state], cur_model, 1, agg_fn=None,
                acceleration_source=cur_accel_source,
            )[0]
            best_s = score_fn(full_agg, worst_w=worst_w)
            print(format_agg("FINAL(全件)", full_agg) + f"  score={best_s:.4f}")
            best_result.update({"score": best_s, "agg": full_agg})

        return {"params": state, "agg": best_result["agg"], "score": best_s}

    import tempfile
    from multiprocessing import Process

    db_fd, db_path = tempfile.mkstemp(suffix=".db")
    os.close(db_fd)
    db_url = f"sqlite:///{db_path}"

    try:
        init_agg = _eval_grid(
            None, ctxs_search, [cur_best], cur_model, 1, agg_fn=None,
            acceleration_source=cur_accel_source,
        )[0]
        init_score = score_fn(init_agg, worst_w=worst_w)
        print(f"\n## Optuna TPE ({case_name}, {n_trials} trials, phase={phase}) [SQLite Process Parallel: {n_jobs} jobs]")
        _checkpoint(cur_best, init_score)

        study = optuna.create_study(
            study_name="robust_search", storage=db_url, direction="minimize", sampler=sampler, load_if_exists=True,
        )
        study.enqueue_trial(_make_enqueue(cur_best))
        for ep in (extra_enqueue or []):
            study.enqueue_trial(_make_enqueue(ep))

        trials_per_worker = [n_trials // n_jobs] * n_jobs
        for i in range(n_trials % n_jobs):
            trials_per_worker[i] += 1

        processes = []
        for worker_id, n_trials_w in enumerate(trials_per_worker):
            if n_trials_w == 0:
                continue
            p = Process(
                target=_run_worker,
                args=(worker_id, db_url, n_trials_w, n_trials, cur_best, continuous_space, explore_delay,
                      delay_candidates, ctxs_search, cur_model, cur_accel_source, score_fn, worst_w, out_path),
            )
            p.start()
            processes.append(p)
        for p in processes:
            p.join()

        study = optuna.load_study(study_name="robust_search", storage=db_url)
        best_trial = study.best_trial
        best_params = best_trial.params
        best_s = best_trial.value

        state = dict(cur_best)
        state.update(best_params)

        print("[INFO] Optuna optimization complete. Re-evaluating best params...")
        final_agg = _eval_grid(
            None, ctxs_search, [state], cur_model, 1, agg_fn=None,
            acceleration_source=cur_accel_source,
        )[0]
        _checkpoint(state, best_s)

        if search_subsample and len(ctxs_search) < len(ctxs):
            print(f"[INFO] 最終 score を全 {len(ctxs)} データセットで評価中...")
            full_agg = _eval_grid(
                None, ctxs, [state], cur_model, 1, agg_fn=None,
                acceleration_source=cur_accel_source,
            )[0]
            best_s = score_fn(full_agg, worst_w=worst_w)
            print(format_agg("FINAL(全件)", full_agg) + f"  score={best_s:.4f}")
            final_agg = full_agg

        return {"params": state, "agg": final_agg, "score": best_s}
    finally:
        try:
            if os.path.exists(db_path):
                os.remove(db_path)
        except Exception as e:
            print(f"[WARN] Failed to delete temp database {db_path}: {e}")


def _discover(collection_dir: Path) -> list[tuple[str, Path]]:
    """収集ディレクトリ配下の CSV キャッシュ (reidentify_cache.csv) を列挙する。"""
    result = []
    for e in discover_collection(collection_dir):
        if e.dir is None:
            continue
        csv_path = e.dir / CACHE_NAME
        if csv_path.exists():
            result.append((e.dataset_id, csv_path))
    return result


def fit_merge(
    collection_dir: Path,
    scenario: Path,
    *,
    case: str = "current",
    phase: int = 0,
    phase2_params: dict | None = None,
    n_trials: int = 50,
    n_jobs: int = 1,
    search_subsample: int | None = None,
    worst_w: float = WORST_W,
    verbose: bool = False,
    out_path: Path | None = None,
) -> dict:
    tasks = _discover(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir} (先に extract.py を実行してください)")

    cfg = load_models_doc(scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    baseline_accel_source = cfg.models[baseline_case].acceleration_source
    cur_case = cfg.find_case(case)
    cur_model = cur_case.vehicle_model_type
    print(
        f"[INFO] baseline model = {baseline_model_type} "
        f"(scenario.yaml の '{baseline_case}' ケース, accel_source={baseline_accel_source})"
    )
    print(
        f"[INFO] current model  = {cur_model} "
        f"('{case}' ケース, accel_source={cur_case.acceleration_source})"
    )

    set_worker_thread_env_defaults()

    ctxs = load_datasets(
        tasks,
        n_jobs=n_jobs,
        verbose=verbose,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=baseline_accel_source,
    )
    if len(ctxs) < 1:
        raise RuntimeError("有効な dataset が 0 件です")

    global _CTXS  # noqa: PLW0603
    _CTXS = ctxs

    phase_fixed_params: dict | None = None
    base_override: dict | None = None
    extra_enqueue: list[dict] | None = None
    if phase2_params:
        base_override = dict(phase2_params)
        acc_keys = {"acc_time_constant", "acc_time_delay"}
        phase_fixed_params = {k: v for k, v in base_override.items() if k in acc_keys} if phase == 2 else {}
        extra_enqueue = [dict(phase2_params)]

    result = robust_search(
        ctxs, cfg, case_name=case, n_trials=n_trials, n_jobs=n_jobs, search_subsample=search_subsample,
        out_path=out_path, extra_enqueue=extra_enqueue, worst_w=worst_w, phase=phase,
        phase_fixed_params=phase_fixed_params, base_override=base_override,
    )
    # 1. Evaluate baseline
    from ..lib._multi_agg import robust_score as score_fn
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    baseline_metrics = [(ctx.dataset_id, ctx.base_metric) for ctx in ctxs]
    baseline_agg = aggregate_normalized(baseline_metrics, baselines)
    baseline_score = score_fn(baseline_agg, worst_w=worst_w)

    # 2. Evaluate tuned (current)
    tuned_params = result["params"]
    full_tuned_params = dict(cur_case.params)
    full_tuned_params.update(tuned_params)
    tuned_agg = result["agg"]
    tuned_score = result["score"]

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

    for spec in cfg.comparison_models:
        name = spec.name
        if name == baseline_case or name == case:
            continue
        try:
            h_model_type = spec.vehicle_model_type
            if h_model_type is None:
                continue
            full_h_params = dict(spec.params)

            # Evaluate using _eval_grid
            h_agg = _eval_grid(
                None, ctxs, [full_h_params], h_model_type, 1, agg_fn=None,
                acceleration_source=spec.acceleration_source,
            )[0]
            h_score = score_fn(h_agg, worst_w=worst_w)

            comparison_results[name] = {
                "score": float(h_score),
                "by_h": clean_agg(h_agg),
                "acceleration_source": spec.acceleration_source,
            }
        except Exception as e:
            print(f"[WARN] Failed to evaluate historical model {name}: {e}", file=sys.stderr)

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

    return {
        "params": full_tuned_params,
        "score": result["score"],
        "comparison": comparison_results,
        "metadata": {
            "collection_dir": str(collection_dir),
            "n_datasets": len(tasks),
            "n_valid": len(ctxs),
            "scenario": str(scenario),
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        },
    }


def run(
    collection_dir: Path, scenario: Path, out: Path, *, case: str = "current", phase: int = 0,
    phase2_params_path: Path | None = None, n_trials: int = 50, n_jobs: int = 1,
    search_subsample: int | None = None, worst_w: float = WORST_W, verbose: bool = False,
) -> dict:
    phase2_params: dict = {}
    if phase2_params_path is not None and phase2_params_path.exists():
        with phase2_params_path.open("r") as f:
            data = yaml.safe_load(f)
        phase2_params = dict(data.get("params", data))
        print(f"[fit_merge] 直接同定値を warm-start / passthrough に使用: {phase2_params_path}")

    out.parent.mkdir(parents=True, exist_ok=True)
    result = fit_merge(
        collection_dir, scenario, case=case, phase=phase, phase2_params=phase2_params,
        n_trials=n_trials, n_jobs=n_jobs, search_subsample=search_subsample, worst_w=worst_w,
        verbose=verbose, out_path=out,
    )
    with out.open("w") as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f"[INFO] FINAL params 保存: {out}")
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description="Optuna warm-start 統合チューニング (Step4a)")
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--case", default="current")
    ap.add_argument("--phase", type=int, default=0, choices=[0, 1, 2])
    ap.add_argument("--phase-params", type=Path, default=None, help="Step3 (fit_steer) の出力 YAML (warm-start兼passthrough)")
    ap.add_argument("--n-trials", type=int, default=50)
    ap.add_argument("--jobs", type=int, default=default_parallel_jobs())
    ap.add_argument("--search-subsample", type=int, default=None)
    ap.add_argument("--worst-weight", type=float, default=WORST_W)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    run(
        args.collection_dir, args.scenario, args.out, case=args.case, phase=args.phase,
        phase2_params_path=args.phase_params, n_trials=args.n_trials, n_jobs=normalize_parallel_jobs(args.jobs),
        search_subsample=args.search_subsample, worst_w=args.worst_weight, verbose=args.verbose,
    )


if __name__ == "__main__":
    main()
