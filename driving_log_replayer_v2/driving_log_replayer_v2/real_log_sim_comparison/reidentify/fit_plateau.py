#!/usr/bin/env python3
"""プラトー領域の steer/ax RMSE から定常スケーリングを直接同定する独立解析。

steer/ax の open-loop N-step 誤差は、初期状態の記憶が消える N≈20 (steer) / N≈9 (ax)
以降、コマンド履歴だけで決まる定常誤差に飽和する (プラトー特性)。プラトー値は
初期条件に依存しないアクチュエータ定常忠実度の指標なので、各系統の時定数・むだ時間を
固定したまま scaling factor だけを軽量に直接同定できる。

モデル構造上 steer 終端状態は steer 系のみ、ax 終端状態は acc 系のみに依存するため、
目的関数は系統別に完全に分離できる。「τ/delay 決定後の scaling 決定」は steer / ax の
独立な 1 次元フィットとして fit_scaling_channels に実装されており、パイプラインの
fit_lon / fit_steer も同じコアを τ/delay 確定後に呼ぶ (実装は rollout 正式評価の 1 本のみ)。

この CLI は同じコアを任意ケース (既定 v1) を初期値に単発実行するもので、結果は
scenario.yaml の比較モデル (例: v1_p) へ手動転記する。実行例:

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

from scipy.optimize import minimize_scalar
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

# 系統別の (プラトー指標, 同定する scaling キー)。時定数・むだ時間 (過渡特性) は対象外で
# case の値を固定する。steer 終端状態は steer 系のみ、ax 終端状態は acc 系のみに依存する
# ため、各チャネルは独立の 1 次元フィットとして分離できる。
# steer_bias は含めない: モデル構造上 steer 状態は steer_des·scaling に収束し、bias は
# ヨーレート計算にのみ入る (sim_model_delay_steer_acc_geared_for_diffusion_planner.cpp)。
# そのため steer/ax プラトー目的関数に対して平坦 (同定不能) で、探索すると任意の値に漂流する。
# bias の同定は yaw を見る fit_merge に委ねる。
CHANNELS: tuple[tuple[str, str], ...] = (
    ("steer", "debug_steer_scaling_factor"),
    ("ax", "debug_acc_scaling_factor"),
)
THETA_KEYS = tuple(key for _metric, key in CHANNELS)
DEFAULT_CASE = "v1"
# steer (N≈20)・ax (N≈9) の両方が飽和済みで、かつ HORIZONS ⊆ (base_metric を流用できる)。
DEFAULT_HORIZON = 30
# 無効 rollout 用の有限ペナルティ (有界スカラー探索は inf より有限大値の方が安定)。
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


def _plateau_channel_values(
    metrics: list[dict | None], horizon: int, metric_key: str,
) -> list[float] | None:
    """全 dataset の単一チャネルのプラトー RMSE。ひとつでも無効なら None (ペナルティ)。"""
    values: list[float] = []
    for metric in metrics:
        if metric is None:
            return None
        value = float(metric[horizon][metric_key])
        if not math.isfinite(value):
            return None
        values.append(value)
    return values


def fit_scaling_channels(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str = DEFAULT_CASE,
    override_params: dict | None = None,
    channels: tuple[tuple[str, str], ...] = CHANNELS,
    horizon: int = DEFAULT_HORIZON,
    n_jobs: int = 1,
    acceleration_source: str | None = None,
    steering_source: str | None = None,
    with_diagnostics: bool = False,
) -> dict:
    """case のパラメータを初期値に、プラトー RMSE を最小化する scaling を系統別に同定する。

    rollout の正式実装 (fit_merge._eval) を使う唯一のプラトー同定コアで、
    単発解析 (fit_plateau CLI) と各ステージ (fit_lon / fit_steer) の両方から呼ばれる。
    override_params は case パラメータへの上書き (ステージが確定させた τ/delay 等)。
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
    base_params.update(override_params or {})
    for _metric, key in channels:
        if key not in base_params:
            raise ValueError(f"scenario の '{case_name}' + override に {key} がありません")

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

    bounds: dict[str, tuple[float, float]] = {}
    for _metric, key in channels:
        search_bounds = PARAMETER_CONSTRAINTS[key].search_bounds
        if search_bounds is None:
            raise RuntimeError(f"{key} に search_bounds がありません (SSOT を確認)")
        bounds[key] = search_bounds

    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(_CTXS))
    pool = (
        multiprocessing.get_context("fork").Pool(n_workers) if n_workers > 1 else None
    )

    try:
        # 系統別に独立な 1 次元フィット: 各チャネルの目的関数は自チャネルの
        # scaling にしか依存しないため、順序に依らず分離して解ける。
        fitted: dict[str, float] = {}
        objectives: dict[str, dict[str, float]] = {}
        for metric_key, param_key in channels:
            initial_values = _plateau_channel_values(kept_metrics, horizon, metric_key)
            assert initial_values is not None
            n_evals = 0

            def objective(scale: float) -> float:
                nonlocal n_evals
                n_evals += 1
                params = dict(base_params)
                params[param_key] = float(scale)
                values = _plateau_channel_values(
                    _eval_all(pool, params, (horizon,), n_workers=n_workers),
                    horizon, metric_key,
                )
                if values is None:
                    return _PENALTY
                score = stats.mean(values)
                if n_evals % 5 == 1:
                    print(
                        f"[fit_plateau] {metric_key} eval {n_evals}: "
                        f"{param_key}={float(scale):.6f} mean RMSE={score:.6f}",
                        file=sys.stderr,
                    )
                return score

            result = minimize_scalar(
                objective, bounds=bounds[param_key], method="bounded",
                options={"xatol": 1e-4},
            )
            fitted[param_key] = PARAMETER_CONSTRAINTS[param_key].clamp(float(result.x))
            objectives[metric_key] = {
                "initial": float(stats.mean(initial_values)),
                "final": float(result.fun),
                "n_evals": n_evals,
            }
        fitted_params = dict(base_params)
        fitted_params.update(fitted)

        # 診断 (任意): 過渡 (N=1, 10) とプラトー (N=horizon) で before/after の
        # RMSE と署名付き平均。
        diag_horizons = tuple(sorted({1, 10, horizon}))
        diag = None
        if with_diagnostics:
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
        # チャネル別の平均プラトー RMSE (steer: deg / ax: m/s^2 の生単位)。
        "objectives": objectives,
        "dataset_ids": [ctx.dataset_id for ctx in _CTXS],
        "diagnostics": diag,
        "metadata": {
            "tuning_type": "plateau_direct_fit",
            "case": case_name,
            "override_keys": sorted(override_params or {}),
            "horizon": horizon,
            "acceleration_source": _ACCEL_SOURCE,
            "steering_source": _STEER_SOURCE,
            "channels": [list(channel) for channel in channels],
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
    print(f"\n[fit_plateau] 同定結果 (N={horizon}, 系統別 1 次元フィット):")
    units = {"steer": "deg", "ax": "m/s^2"}
    for metric_key, param_key in CHANNELS:
        obj = result["objectives"][metric_key]
        improvement = (
            f" ({(1 - obj['final'] / obj['initial']):+.1%} 改善)"
            if obj["initial"] > 0.0 else ""
        )
        print(
            f"  {metric_key:5s} {param_key} = {result['fitted'][param_key]:.6f}"
            f"  mean RMSE {obj['initial']:.6f} -> {obj['final']:.6f} {units[metric_key]}"
            f"{improvement} (evals={obj['n_evals']})"
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
    result = fit_scaling_channels(
        collection_dir, scenario, case_name=case_name, horizon=horizon, n_jobs=n_jobs,
        acceleration_source=acceleration_source, steering_source=steering_source,
        with_diagnostics=True,
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    params_path = out_dir / "plateau_params.yaml"
    with params_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {
                "params": result["params"],
                "metadata": {
                    **result["metadata"],
                    # チャネル別の平均プラトー RMSE (steer: deg / ax: m/s^2)。
                    "objectives": result["objectives"],
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
