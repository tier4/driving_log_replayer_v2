"""A2: Oracle 境界 / 異質性解析 と CVaR テール特性。

(1) per-dataset scaling oracle — fit_plateau の 1 次元 scaling フィットを dataset 単位に
    分解し、各 dataset が単独で到達できるプラトー RMSE の下限と最適 scaling を求める。
    scaling 分布が条件 (平均勾配・速度帯・ブレーキ頻度) と相関すれば「構造欠落」、
    分布が狭いのに RMSE が残れば「scaling では説明不可 (構造確定)」、
    分布が広く条件と無相関なら「個体差/日次差 (単一グローバルモデルでは吸収不能)」。
    探索域はリリース探索 (0.9–1.1) より広い分析専用域を使い、クランプの実像を見る。

(2) CVaR テール特性 — 既存 metrics.csv から対象モデルの per-dataset 正規化スコア寄与を
    計算し、上位 (worst) dataset の走行条件を traces 由来の条件サマリと突き合わせる。
    追加 rollout なし。
"""
from __future__ import annotations

import datetime
import multiprocessing
from pathlib import Path
import sys

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

from ..fit_merge import DatasetCtx, _eval, load_datasets
from ..load_data import discover_cached_datasets
from ..model_config import load_model_config, resolve_baseline_model
from ..settings import ACT_SCORE_HORIZONS, HORIZONS
from ...lib._multi_agg import POS_W, ACT_W, normalize_components
from ...lib._parallel import (
    imap_with_watchdog,
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from .split import split_of

# oracle の対象チャネル (fit_plateau.CHANNELS と同じ組)。
ORACLE_CHANNELS: tuple[tuple[str, str], ...] = (
    ("steer", "debug_steer_scaling_factor"),
    ("ax", "debug_acc_scaling_factor"),
)
ORACLE_HORIZON = 30
# 分析専用の広い探索域 (リリース探索の 0.9–1.1 では 0.9 クランプの実像が見えない)。
ORACLE_BOUNDS = (0.5, 1.5)
_PENALTY = 1.0e9

# テール特性: 正規化スコア寄与の上位分位。
TAIL_QUANTILE = 0.90
# 条件サマリの brake/accel 判定しきい値 (窓平均指令 [m/s^2])。
BRAKE_A_CMD = -0.3
ACCEL_A_CMD = 0.3

# fork worker が COW で読み取り継承する globals。
_CTXS: list[DatasetCtx] = []
_MODEL_TYPE = ""
_ACCEL_SOURCE = ""
_STEER_SOURCE = "steer"
_SLOPE_SOURCE = "none"
_BASE_PARAMS: dict = {}


def _oracle_one(idx: int) -> dict | None:
    """1 dataset の系統別 1 次元 scaling oracle。"""
    ctx = _CTXS[idx]
    out: dict = {"dataset_id": ctx.dataset_id}
    try:
        initial = _eval(
            ctx, _BASE_PARAMS, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE, _SLOPE_SOURCE,
            horizons=(ORACLE_HORIZON,),
        )
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] oracle 初期評価失敗 ({ctx.dataset_id}): {e}", file=sys.stderr)
        return None
    for metric_key, param_key in ORACLE_CHANNELS:
        init_val = float(initial[ORACLE_HORIZON][metric_key])
        if not np.isfinite(init_val):
            return None

        def objective(scale: float, param_key: str = param_key, metric_key: str = metric_key) -> float:
            params = dict(_BASE_PARAMS)
            params[param_key] = float(scale)
            try:
                m = _eval(
                    ctx, params, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE, _SLOPE_SOURCE,
                    horizons=(ORACLE_HORIZON,),
                )
            except Exception:  # noqa: BLE001
                return _PENALTY
            value = float(m[ORACLE_HORIZON][metric_key])
            return value if np.isfinite(value) else _PENALTY

        result = minimize_scalar(
            objective, bounds=ORACLE_BOUNDS, method="bounded", options={"xatol": 1e-3},
        )
        out[f"{metric_key}_scale_init"] = float(_BASE_PARAMS[param_key])
        out[f"{metric_key}_scale_opt"] = float(result.x)
        out[f"{metric_key}_rmse_init"] = init_val
        out[f"{metric_key}_rmse_opt"] = float(result.fun)
    return out


def fit_per_dataset_oracle(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str,
    n_jobs: int = 1,
    splits: tuple[str, ...] = ("dev",),
) -> tuple[pd.DataFrame, dict]:
    """per-dataset scaling oracle の表と実行メタデータを返す。"""
    global _CTXS, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE, _SLOPE_SOURCE, _BASE_PARAMS  # noqa: PLW0603

    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    case = cfg.find_case(case_name)

    set_worker_thread_env_defaults()
    ctxs, _skipped = load_datasets(
        tasks,
        n_jobs=n_jobs,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=cfg.models[baseline_case].acceleration_source,
        baseline_steering_source=cfg.models[baseline_case].steering_source,
    )
    ctxs = [ctx for ctx in ctxs if split_of(ctx.dataset_id) in splits]
    if not ctxs:
        raise RuntimeError(f"split {splits} に該当する dataset が 0 件です")

    _CTXS = ctxs
    _MODEL_TYPE = case.vehicle_model_type
    _ACCEL_SOURCE = case.acceleration_source
    _STEER_SOURCE = case.steering_source
    _SLOPE_SOURCE = getattr(case, "slope_source", "none")
    _BASE_PARAMS = dict(case.params)

    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(_CTXS))
    indices = list(range(len(_CTXS)))
    if n_workers > 1:
        pool = multiprocessing.get_context("fork").Pool(n_workers)
        try:
            results = imap_with_watchdog(
                pool, _oracle_one, indices,
                chunksize=pool_chunksize(len(indices), n_workers),
            )
        finally:
            pool.close()
            pool.join()
    else:
        results = [_oracle_one(idx) for idx in indices]

    rows = [r for r in results if r is not None]
    if not rows:
        raise RuntimeError("oracle 結果が 0 件です")
    df = pd.DataFrame(rows)
    metadata = {
        "case": case_name,
        "horizon": ORACLE_HORIZON,
        "bounds": list(ORACLE_BOUNDS),
        "channels": [list(c) for c in ORACLE_CHANNELS],
        "splits": sorted(splits),
        "n_valid": int(len(df)),
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    return df, metadata


def dataset_conditions(traces: pd.DataFrame) -> pd.DataFrame:
    """traces (horizon=30 行) から per-dataset の走行条件サマリを作る。"""
    df = traces[traces["horizon"] == 30]
    grouped = df.groupby("dataset_id")
    out = pd.DataFrame({
        "pitch_lf_abs_mean_deg": np.degrees(grouped["pitch_lf_mean"].apply(lambda s: float(np.mean(np.abs(s))))),
        "vx_p50": grouped["vx_mean"].median(),
        "vx_p90": grouped["vx_mean"].quantile(0.9),
        "brake_frac": grouped["a_cmd_mean"].apply(lambda s: float((s < BRAKE_A_CMD).mean())),
        "accel_frac": grouped["a_cmd_mean"].apply(lambda s: float((s > ACCEL_A_CMD).mean())),
        "steer_rate_abs_p90": grouped["steer_rate_abs_mean"].quantile(0.9),
        "ay_abs_p90": grouped["ay_mean"].apply(lambda s: float(np.quantile(np.abs(s), 0.9))),
        "n_rows": grouped.size(),
    })
    return out.reset_index()


def _per_dataset_score_contribution(metrics: pd.DataFrame, model: str) -> pd.DataFrame:
    """metrics.csv から robust_score の mean 項と同型の per-dataset 寄与を計算する。"""
    required_h = set(HORIZONS)
    piv: dict[str, dict[int, dict]] = {}
    for m in ("baseline", model):
        sub = metrics[(metrics["model"] == m) & (metrics["horizon"].isin(required_h))]
        for ds_id, ds_sub in sub.groupby("dataset_id"):
            by_h = {int(r["horizon"]): r for _, r in ds_sub.iterrows()}
            piv.setdefault(ds_id, {})[m] = by_h

    rows = []
    for ds_id, models in piv.items():
        if "baseline" not in models or model not in models:
            continue
        base_h, model_h = models["baseline"], models[model]
        if not required_h <= set(base_h) or not required_h <= set(model_h):
            continue
        score = 0.0
        comp: dict[str, float] = {}
        valid = True
        for h in HORIZONS:
            norm = normalize_components(
                {k: float(model_h[h][k]) for k in ("yaw", "long", "lat", "steer", "ax")},
                {k: float(base_h[h][k]) for k in ("yaw", "long", "lat", "steer", "ax")},
                h,
            )
            if not all(np.isfinite(v) for v in norm.values()):
                valid = False
                break
            score += norm["nyaw"] + POS_W * (norm["nlong"] + norm["nlat"])
            if h in ACT_SCORE_HORIZONS:
                score += ACT_W * (norm["nsteer"] + norm["nax"])
            for key in ("nyaw", "nlong", "nlat", "nsteer", "nax"):
                comp[f"{key}_h{h}"] = norm[key]
        if not valid:
            continue
        rows.append({"dataset_id": ds_id, "score_contribution": score, **comp})
    return pd.DataFrame(rows)


def characterize_tail(
    metrics_csv: Path,
    traces: pd.DataFrame,
    *,
    model: str,
    splits: tuple[str, ...] = ("dev",),
) -> pd.DataFrame:
    """per-dataset 正規化スコア寄与 + 条件サマリ + テールフラグの表を返す。"""
    metrics = pd.read_csv(metrics_csv)
    if model not in set(metrics["model"]):
        raise ValueError(f"metrics.csv に model={model!r} がありません: {sorted(set(metrics['model']))}")
    contrib = _per_dataset_score_contribution(metrics, model)
    contrib = contrib[contrib["dataset_id"].map(split_of).isin(splits)]
    if contrib.empty:
        raise RuntimeError("スコア寄与を計算できる dataset が 0 件です")

    conditions = dataset_conditions(traces)
    out = contrib.merge(conditions, on="dataset_id", how="left")
    threshold = out["score_contribution"].quantile(TAIL_QUANTILE)
    out["is_tail"] = out["score_contribution"] >= threshold
    return out.sort_values("score_contribution", ascending=False).reset_index(drop=True)
