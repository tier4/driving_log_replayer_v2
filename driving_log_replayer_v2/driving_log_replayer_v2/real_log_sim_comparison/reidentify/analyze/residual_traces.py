"""per-start 署名付き N-step 終端誤差の抽出 (残差分析キャンペーンの共通基盤)。

fit_merge と同一の dataset 採否集団 (baseline 検証込みロード) に対して、指定 case の
パラメータで rollout.eval_rollout_terminal_errors を実行し、(dataset_id, k0, horizon)
ごとの署名付き終端誤差 + 条件付け特徴量を 1 本の DataFrame に集める。
並列化は fit_plateau と同じ fork+COW パターン (_CTXS グローバル継承)。
"""
from __future__ import annotations

import datetime
import multiprocessing
from pathlib import Path
import sys

import pandas as pd

from ..fit_merge import DatasetCtx, _ctx_data, load_datasets
from ..load_data import discover_cached_datasets
from ..model_config import load_model_config, resolve_baseline_model
from ..rollout import eval_rollout_terminal_errors
from ..settings import HORIZONS, ROLLOUT_STRIDE
from ...lib._parallel import (
    imap_with_watchdog,
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from ...lib._vehicle_models import merge_vehicle_model_params
from .split import split_of

# 抽出する horizon: N=1 (過渡診断) + 最適化 horizon (プラトー N=30、yaw N=70、lat N=150 を含む)。
TRACE_HORIZONS: tuple[int, ...] = (1, *HORIZONS)

# fork worker が COW で読み取り継承する globals (fit_plateau._CTXS と同じパターン)。
_CTXS: list[DatasetCtx] = []
_MODEL_TYPE = ""
_ACCEL_SOURCE = ""
_STEER_SOURCE = "steer"
_SLOPE_SOURCE = "none"
_PARAMS: dict = {}
_HORIZONS: tuple[int, ...] = TRACE_HORIZONS
_STRIDE: int = ROLLOUT_STRIDE


def _trace_one(idx: int) -> pd.DataFrame | None:
    ctx = _CTXS[idx]
    try:
        params = merge_vehicle_model_params(ctx.base, _PARAMS, _MODEL_TYPE)
        data = _ctx_data(ctx, _ACCEL_SOURCE, _STEER_SOURCE)
        df = eval_rollout_terminal_errors(
            data, ctx.t0_ns, params, _MODEL_TYPE,
            horizons=_HORIZONS, stride=_STRIDE, slope_source=_SLOPE_SOURCE,
        )
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] residual trace 失敗 ({ctx.dataset_id}): {e}", file=sys.stderr)
        return None
    if df.empty:
        return None
    df.insert(0, "dataset_id", ctx.dataset_id)
    return df


def extract_residual_traces(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str,
    horizons: tuple[int, ...] = TRACE_HORIZONS,
    stride: int = ROLLOUT_STRIDE,
    n_jobs: int = 1,
    splits: tuple[str, ...] = ("dev",),
) -> tuple[pd.DataFrame, dict]:
    """指定 case の per-start 終端誤差 DataFrame と実行メタデータを返す。"""
    global _CTXS, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE, _SLOPE_SOURCE, _PARAMS, _HORIZONS, _STRIDE  # noqa: PLW0603

    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    case = cfg.find_case(case_name)

    set_worker_thread_env_defaults()

    ctxs, skipped = load_datasets(
        tasks,
        n_jobs=n_jobs,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=cfg.models[baseline_case].acceleration_source,
        baseline_steering_source=cfg.models[baseline_case].steering_source,
    )
    if not ctxs:
        raise RuntimeError("有効な dataset が 0 件です")

    n_loaded = len(ctxs)
    ctxs = [ctx for ctx in ctxs if split_of(ctx.dataset_id) in splits]
    print(
        f"[analyze] split フィルタ {sorted(splits)}: {len(ctxs)}/{n_loaded} datasets",
        file=sys.stderr,
    )
    if not ctxs:
        raise RuntimeError(f"split {splits} に該当する dataset が 0 件です")

    _CTXS = ctxs
    _MODEL_TYPE = case.vehicle_model_type
    _ACCEL_SOURCE = case.acceleration_source
    _STEER_SOURCE = case.steering_source
    _SLOPE_SOURCE = getattr(case, "slope_source", "none")
    _PARAMS = dict(case.params)
    _HORIZONS = tuple(sorted(horizons))
    _STRIDE = int(stride)

    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(_CTXS))
    indices = list(range(len(_CTXS)))
    if n_workers > 1:
        pool = multiprocessing.get_context("fork").Pool(n_workers)
        try:
            results = imap_with_watchdog(
                pool, _trace_one, indices,
                chunksize=pool_chunksize(len(indices), n_workers),
            )
        finally:
            pool.close()
            pool.join()
    else:
        results = [_trace_one(idx) for idx in indices]

    frames = []
    for ctx, df in zip(_CTXS, results):
        if df is None:
            skipped.append({"dataset_id": ctx.dataset_id, "reason": f"{case_name} residual trace が空/失敗"})
        else:
            frames.append(df)
    if not frames:
        raise RuntimeError(f"{case_name} で residual trace が 1 件も得られませんでした")

    traces = pd.concat(frames, ignore_index=True)
    metadata = {
        "case": case_name,
        "model_type": _MODEL_TYPE,
        "acceleration_source": _ACCEL_SOURCE,
        "steering_source": _STEER_SOURCE,
        "horizons": list(_HORIZONS),
        "stride": _STRIDE,
        "splits": sorted(splits),
        "n_datasets": len(tasks),
        "n_valid": traces["dataset_id"].nunique(),
        "n_rows": int(len(traces)),
        "skipped": skipped,
        "scenario": str(scenario),
        "collection_dir": str(collection_dir),
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    return traces, metadata
