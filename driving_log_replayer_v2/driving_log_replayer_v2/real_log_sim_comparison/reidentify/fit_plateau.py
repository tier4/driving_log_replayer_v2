#!/usr/bin/env python3
"""プラトー領域の steer/ax RMSE から定常アクチュエータパラメータを直接同定する独立解析。

steer/ax の open-loop N-step 誤差は、初期状態の記憶が消える N≈20 (steer) / N≈9 (ax)
以降、コマンド履歴だけで決まる定常誤差に飽和する (プラトー特性)。プラトー値は
初期条件に依存しないアクチュエータ定常忠実度の指標なので、時定数・むだ時間を固定した
まま定常パラメータ (steer_bias / steer・acc スケーリング) だけを軽量に直接同定できる。

7 ステージパイプラインには組み込まない単発解析。結果は scenario.yaml の比較モデル
(例: v1_p) へ手動転記する。実行例:

    make fit_plateau ROOT=<collection> SCENARIO=<scenario.yaml>
"""
from __future__ import annotations

import argparse
import csv
import datetime
import math
import multiprocessing
from pathlib import Path
import statistics as stats
import sys

import numpy as np
from scipy.optimize import minimize
import yaml

from ..lib._parallel import (
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from . import fit_merge
from .load_data import discover_cached_datasets
from .model_config import load_model_config, resolve_baseline_model
from .parameter_constraints import PARAMETER_CONSTRAINTS
from .settings import DEFAULT_OUTPUT_DIR_NAME, HORIZONS

# 同定対象の定常パラメータ。時定数・むだ時間 (過渡特性) は対象外で case の値を固定する。
# steer_bias は含めない: モデル構造上 steer 状態は steer_des·scaling に収束し、bias は
# ヨーレート計算にのみ入る (sim_model_delay_steer_acc_geared_for_diffusion_planner.cpp)。
# そのため steer/ax プラトー目的関数に対して平坦 (同定不能) で、探索すると任意の値に漂流する。
# bias の同定は yaw を見る fit_merge に委ねる。
THETA_KEYS = ("debug_steer_scaling_factor", "debug_acc_scaling_factor")
DEFAULT_CASE = "v1"
# steer (N≈20)・ax (N≈9) の両方が飽和済みで、かつ HORIZONS ⊆ (base_metric を流用できる)。
DEFAULT_HORIZON = 30
# 無効 rollout 用の有限ペナルティ (Powell は inf より有限大値の方が安定)。
_PENALTY = 1.0e9

# fork worker が COW で読み取り継承する globals (fit_merge の _SEARCH_* と同じパターン)。
_CTXS: list[fit_merge.DatasetCtx] = []
_MODEL_TYPE = ""
_ACCEL_SOURCE = ""
_STEER_SOURCE = "steer"


def _eval_one(args: tuple[int, dict, tuple[int, ...], bool]) -> dict | None:
    idx, params, horizons, include_mean = args
    try:
        return fit_merge._eval(
            _CTXS[idx], params, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE,
            horizons=horizons, include_mean=include_mean,
        )
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] rollout 失敗 ({_CTXS[idx].dataset_id}): {e}", file=sys.stderr)
        return None


def _eval_all(
    pool, params: dict, horizons: tuple[int, ...], *,
    include_mean: bool = False, n_workers: int = 1,
) -> list[dict | None]:
    tasks = [(idx, params, horizons, include_mean) for idx in range(len(_CTXS))]
    if pool is None:
        return [_eval_one(task) for task in tasks]
    return list(pool.imap(_eval_one, tasks, chunksize=pool_chunksize(len(tasks), n_workers)))


def _plateau_score(
    steer_vals: list[float], ax_vals: list[float], scale_steer: float, scale_ax: float,
) -> float:
    """プラトー目的関数: steer/ax の全 dataset 平均 RMSE を baseline スケールで等寄与に合算。"""
    return stats.mean(steer_vals) / scale_steer + stats.mean(ax_vals) / scale_ax


def _plateau_terms(metrics: list[dict | None], horizon: int) -> tuple[list[float], list[float]] | None:
    """全 dataset の steer/ax プラトー RMSE。ひとつでも無効なら None (ペナルティ)。"""
    steer_vals: list[float] = []
    ax_vals: list[float] = []
    for metric in metrics:
        if metric is None:
            return None
        steer = float(metric[horizon]["steer"])
        ax = float(metric[horizon]["ax"])
        if not (math.isfinite(steer) and math.isfinite(ax)):
            return None
        steer_vals.append(steer)
        ax_vals.append(ax)
    return steer_vals, ax_vals


def fit_plateau(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str = DEFAULT_CASE,
    horizon: int = DEFAULT_HORIZON,
    n_jobs: int = 1,
    acceleration_source: str | None = None,
    steering_source: str | None = None,
) -> dict:
    """case のパラメータを初期値に、プラトー RMSE を最小化する定常パラメータを同定する。

    acceleration_source / steering_source を指定すると case の GT ソースを上書きする
    (例: v1 を初期値に SG 系 GT で同定して v1_p に転記する場合)。
    """
    global _CTXS, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE  # noqa: PLW0603

    if horizon not in HORIZONS:
        raise ValueError(f"horizon は最適化ホライズン {HORIZONS} から選んでください: {horizon}")

    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    baseline_accel_source = cfg.models[baseline_case].acceleration_source
    baseline_steer_source = cfg.models[baseline_case].steering_source
    case = cfg.find_case(case_name)
    base_params = dict(case.params)
    for key in THETA_KEYS:
        if key not in base_params:
            raise ValueError(f"scenario の '{case_name}' に {key} がありません")

    set_worker_thread_env_defaults()

    # fit_merge と同一の dataset 採否集団を保証する (baseline 検証込みロード)。
    ctxs, skipped = fit_merge.load_datasets(
        tasks,
        n_jobs=n_jobs,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=baseline_accel_source,
        baseline_steering_source=baseline_steer_source,
    )
    if not ctxs:
        raise RuntimeError("有効な dataset が 0 件です")

    _MODEL_TYPE = case.vehicle_model_type
    _ACCEL_SOURCE = acceleration_source or case.acceleration_source
    _STEER_SOURCE = steering_source or case.steering_source

    # 初期パラメータで 1 回だけ親プロセスで評価し、GT キャッシュを温める
    # (theta は GT キーに影響しないため、fork 後の worker は準備済み GT を COW 継承する)。
    # 同時に無効 dataset の除外と "before" 指標の取得を済ませる。
    print(f"[fit_plateau] 初期評価 (case={case_name}, N={horizon}) と GT 準備中...", file=sys.stderr)
    _CTXS = ctxs
    initial_metrics = _eval_all(None, base_params, (horizon,))
    kept: list[fit_merge.DatasetCtx] = []
    kept_metrics: list[dict] = []
    for ctx, metric in zip(ctxs, initial_metrics):
        if metric is None or not fit_merge._rollout_metric_is_finite(metric, (horizon,)):
            skipped.append({"dataset_id": ctx.dataset_id, "reason": f"{case_name} rollout 指標が不正"})
            continue
        kept.append(ctx)
        kept_metrics.append(metric)
    _CTXS = kept
    if not _CTXS:
        raise RuntimeError(f"{case_name} で有効な dataset が 0 件です")
    print(f"[fit_plateau] 有効 dataset: {len(_CTXS)}/{len(tasks)}", file=sys.stderr)

    # steer/ax を等寄与にするスケール = baseline のプラトー平均 (load 時の base_metric を流用)。
    scale_steer = stats.mean(float(ctx.base_metric[horizon]["steer"]) for ctx in _CTXS)
    scale_ax = stats.mean(float(ctx.base_metric[horizon]["ax"]) for ctx in _CTXS)
    if not (scale_steer > 0.0 and scale_ax > 0.0):
        raise RuntimeError(f"baseline プラトーが退化しています (steer={scale_steer}, ax={scale_ax})")

    bounds = []
    for key in THETA_KEYS:
        search_bounds = PARAMETER_CONSTRAINTS[key].search_bounds
        if search_bounds is None:
            raise RuntimeError(f"{key} に search_bounds がありません (SSOT を確認)")
        bounds.append(search_bounds)
    x0 = np.array([
        min(hi, max(lo, float(base_params[key])))
        for key, (lo, hi) in zip(THETA_KEYS, bounds)
    ])

    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(_CTXS))
    pool = (
        multiprocessing.get_context("fork").Pool(n_workers) if n_workers > 1 else None
    )
    n_evals = 0

    def objective(theta: np.ndarray) -> float:
        nonlocal n_evals
        n_evals += 1
        params = dict(base_params)
        params.update({key: float(value) for key, value in zip(THETA_KEYS, theta)})
        terms = _plateau_terms(
            _eval_all(pool, params, (horizon,), n_workers=n_workers), horizon,
        )
        if terms is None:
            return _PENALTY
        steer_vals, ax_vals = terms
        score = _plateau_score(steer_vals, ax_vals, scale_steer, scale_ax)
        if n_evals % 10 == 1:
            print(
                f"[fit_plateau] eval {n_evals}: score={score:.6f} "
                f"theta={dict(zip(THETA_KEYS, [round(float(v), 6) for v in theta]))}",
                file=sys.stderr,
            )
        return score

    try:
        initial_terms = _plateau_terms(kept_metrics, horizon)
        assert initial_terms is not None
        objective_initial = _plateau_score(*initial_terms, scale_steer, scale_ax)
        result = minimize(
            objective, x0, method="Powell", bounds=bounds,
            options={"xtol": 1e-4, "ftol": 1e-6},
        )
        fitted = {
            key: PARAMETER_CONSTRAINTS[key].clamp(float(value))
            for key, value in zip(THETA_KEYS, result.x)
        }
        fitted_params = dict(base_params)
        fitted_params.update(fitted)

        # 診断: 過渡 (N=1, 10) とプラトー (N=horizon) で before/after の RMSE と署名付き平均。
        diag_horizons = tuple(sorted({1, 10, horizon}))
        diag = {
            case_name: _eval_all(
                pool, base_params, diag_horizons, include_mean=True, n_workers=n_workers,
            ),
            f"{case_name}_p": _eval_all(
                pool, fitted_params, diag_horizons, include_mean=True, n_workers=n_workers,
            ),
        }
    finally:
        if pool is not None:
            pool.close()
            pool.join()

    return {
        "case_name": case_name,
        "horizon": horizon,
        "diag_horizons": diag_horizons,
        "params": fitted_params,
        "fitted": fitted,
        "objective_initial": float(objective_initial),
        "objective_final": float(result.fun),
        "scale_steer_deg": float(scale_steer),
        "scale_ax_mps2": float(scale_ax),
        "n_evals": n_evals,
        "dataset_ids": [ctx.dataset_id for ctx in _CTXS],
        "diagnostics": diag,
        "metadata": {
            "tuning_type": "plateau_direct_fit",
            "case": case_name,
            "horizon": horizon,
            "acceleration_source": _ACCEL_SOURCE,
            "steering_source": _STEER_SOURCE,
            "theta_keys": list(THETA_KEYS),
            "n_datasets": len(tasks),
            "n_valid": len(_CTXS),
            "skipped": skipped,
            "scenario": str(scenario),
            "collection_dir": str(collection_dir),
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        },
    }


def _write_diagnostics_csv(out: Path, result: dict) -> None:
    fieldnames = [
        "dataset_id", "model", "horizon",
        "steer", "ax", "steer_mean", "ax_mean", "yaw", "long", "lat", "vx", "pos",
    ]
    with out.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for model, metrics in result["diagnostics"].items():
            for ds_id, metric in zip(result["dataset_ids"], metrics):
                if metric is None:
                    continue
                for horizon in result["diag_horizons"]:
                    row = metric[horizon]
                    writer.writerow({
                        "dataset_id": ds_id,
                        "model": model,
                        "horizon": horizon,
                        **{key: float(row[key]) for key in fieldnames[3:]},
                    })


def _print_summary(result: dict) -> None:
    horizon = result["horizon"]
    case_name = result["case_name"]
    print(f"\n[fit_plateau] 同定結果 (N={horizon}, evals={result['n_evals']}):")
    for key, value in result["fitted"].items():
        print(f"  {key} = {value:.6f}")
    improvement = (
        f" ({(1 - result['objective_final'] / result['objective_initial']):+.1%} 改善)"
        if result["objective_initial"] > 0.0 else ""
    )
    print(
        f"  objective: {result['objective_initial']:.6f} -> {result['objective_final']:.6f}"
        + improvement
    )
    for model in (case_name, f"{case_name}_p"):
        metrics = [m for m in result["diagnostics"][model] if m is not None]
        if not metrics:
            print(f"  {model:8s} 有効な診断 rollout がありません")
            continue
        steer_vals = [float(m[horizon]["steer"]) for m in metrics]
        abs_means = [abs(float(m[horizon]["steer_mean"])) for m in metrics]
        ax_vals = [float(m[horizon]["ax"]) for m in metrics]
        print(
            f"  {model:8s} steer@N{horizon}: mean={stats.mean(steer_vals):.4f}"
            f" med={stats.median(steer_vals):.4f} deg"
            f"  |steer_mean| med={stats.median(abs_means):.4f} deg"
            f"  ax: mean={stats.mean(ax_vals):.4f} m/s^2"
        )


def run(
    collection_dir: Path,
    scenario: Path,
    out_dir: Path,
    *,
    case_name: str = DEFAULT_CASE,
    horizon: int = DEFAULT_HORIZON,
    n_jobs: int = 1,
    acceleration_source: str | None = None,
    steering_source: str | None = None,
) -> dict:
    result = fit_plateau(
        collection_dir, scenario, case_name=case_name, horizon=horizon, n_jobs=n_jobs,
        acceleration_source=acceleration_source, steering_source=steering_source,
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    params_path = out_dir / "plateau_params.yaml"
    with params_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {
                "params": result["params"],
                "metadata": {
                    **result["metadata"],
                    "objective_initial": result["objective_initial"],
                    "objective_final": result["objective_final"],
                    "scale_steer_deg": result["scale_steer_deg"],
                    "scale_ax_mps2": result["scale_ax_mps2"],
                },
            },
            stream, sort_keys=True, allow_unicode=True,
        )
    diag_path = out_dir / "plateau_diagnostics.csv"
    _write_diagnostics_csv(diag_path, result)

    _print_summary(result)
    print(f"\n[fit_plateau] パラメータ保存: {params_path}")
    print(f"[fit_plateau] 診断保存: {diag_path}")
    print(f"[fit_plateau] scenario.yaml へ手動転記して comparison_models に登録してください")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="datasets/ を持つ collection root")
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--case", default=DEFAULT_CASE, help=f"初期値にする scenario ケース名 (default: {DEFAULT_CASE})")
    parser.add_argument("--horizon", type=int, default=DEFAULT_HORIZON, help=f"プラトー horizon (default: {DEFAULT_HORIZON})")
    parser.add_argument("--n-jobs", type=int, default=1)
    parser.add_argument("--out-dir", type=Path, default=None, help="成果物出力先 (default: <root>/reidentify)")
    parser.add_argument(
        "--acceleration-source", default=None,
        help="GT の ax ソースを case 設定から上書き (accel | kinematic_savgol など)",
    )
    parser.add_argument(
        "--steering-source", default=None,
        help="GT の steer ソースを case 設定から上書き (steer | steer_savgol)",
    )
    args = parser.parse_args()

    out_dir = args.out_dir if args.out_dir is not None else args.root / DEFAULT_OUTPUT_DIR_NAME
    run(
        args.root, args.scenario, out_dir,
        case_name=args.case, horizon=args.horizon, n_jobs=args.n_jobs,
        acceleration_source=args.acceleration_source,
        steering_source=args.steering_source,
    )


if __name__ == "__main__":
    main()
