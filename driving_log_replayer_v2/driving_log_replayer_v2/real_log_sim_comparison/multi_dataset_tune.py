#!/usr/bin/env python3
"""
マルチデータセット横断でのロバスト best_normal 同定 (robust_search 専用ツール)。

クラウドは 1 評価ジョブ = 1 データセットのため、各ジョブが出力する real.lite を
`collect_datasets.py` で収集ディレクトリ (<collection>/<dataset_id>/real.lite) に集約した上で、
本モジュールが dataset 横断で誤差を集計しロバストなパラメータを同定する。
ケース横断のレポート用集約 (旧 evaluate_cases / multi_cases_summary.md) は
step13_cross_dataset.py に一本化した (per-dataset の cases_metrics.json を再集計するため
rollout 再実行が不要)。本モジュールは rollout を伴うパラメータ探索のみを担う。

設計の要点:
- closed-loop はローカルで退化するため評価は open-loop N-step rollout (step5.run_rollout) のみ。
- 最大 horizon (N=20) の終端誤差 RMSE で評価 (step7 sweep と同じ指標。小 N は seed バイアス)。
- **per-dataset 正規化** (lib._multi_agg): 各 dataset の baseline 誤差で割って正規化してから
  dataset 横断の mean / worst(max) を取る。baseline は cases.yaml の overlay.reference_tag
  ケース (無補正 delay モデル) と同定義 — step13 の正規化と一致する。

使い方:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
        --collection-dir sample/multi --scenario sample/scenario.yaml
"""

from __future__ import annotations

import argparse
import datetime
from dataclasses import dataclass, field
import multiprocessing
import os
from pathlib import Path
import sys
import traceback

import optuna
import yaml

from . import step_ol1_analyze_nstep as s5
from .lib._collection import discover_collection
from .lib._models_config import load_models_doc, resolve_baseline_model
from .lib._accel_source import normalize_accel_source, rollout_data_with_accel_source
from .lib._lite_resolver import resolve_lite_bag
from .lib._multi_agg import (
    HORIZONS, WORST_W,
    acc_score, aggregate_normalized,
    format_agg, robust_score, steer_score,
)
from .lib._parallel import (
    default_parallel_jobs,
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from .lib._physical_validity import WHEELBASE
from .lib._validation import MissingRequiredDataError

STRIDE = 5
# フォールバック既定値のみ。実運用では main() が scenario.yaml の
# Conditions.overlay.reference_tag (resolve_baseline_model) から解決した値を
# 明示的に渡す。ここを直接書き換えても scenario.yaml と分離したままなので反映されない。
_BASELINE_MODEL = "delay_steer_acc_geared_wo_fall_guard"
# _prepare_gt は params の delay/wheelbase/sub_dt にのみ依存 (run_rollout docstring)
_GT_KEYS = ("acc_time_delay", "steer_time_delay", "wheelbase", "sub_dt")
# fork 前に load_datasets が設定し、fork 後の worker は COW で継承する (P-5/P-7)
_VERBOSE: bool = False
_LOAD_BASELINE_MODEL: str = _BASELINE_MODEL
_LOAD_BASELINE_PARAMS: dict = {}
_LOAD_BASELINE_ACCEL_SOURCE: str = "accel"


@dataclass
class DatasetCtx:
    """1 データセットの rollout 実行コンテキスト (data/t0/base params を保持)。"""

    dataset_id: str
    data: dict
    t0_ns: int
    base: dict
    gt_cache: dict  # gt-key -> gt
    base_metric: dict  # baseline (無補正) の {h: {yaw, pos, long, lat, steer}} (h ∈ HORIZONS)
    data_cache: dict[str, dict] = field(default_factory=dict)
    n_cmd_samples: int = 0
    n_drive_samples: int = 0


def _ctx_data(ctx: DatasetCtx, acceleration_source: str) -> dict:
    source = normalize_accel_source(acceleration_source)
    if source not in ctx.data_cache:
        ctx.data_cache[source] = rollout_data_with_accel_source(ctx.data, source)
    return ctx.data_cache[source]


def _eval(
    ctx: DatasetCtx, override: dict, model_type: str, acceleration_source: str = "accel"
) -> dict:
    """1 dataset・1 パラメータ組の horizon 別終端誤差 RMSE {h: {yaw,pos,long,lat,steer}}。"""
    params = dict(ctx.base)
    params.update(override)
    source = normalize_accel_source(acceleration_source)
    data = _ctx_data(ctx, source)
    key = (source, *tuple(round(float(params[k]), 9) for k in _GT_KEYS))
    gt = ctx.gt_cache.get(key)
    if gt is None:
        gt = s5._prepare_gt(data, ctx.t0_ns, params)
        ctx.gt_cache[key] = gt
    rmse = s5.eval_rollout_rmse(
        data, ctx.t0_ns, params, model_type, horizons=HORIZONS, stride=STRIDE, gt=gt
    )
    return {h: rmse[h] for h in HORIZONS}


def _baseline_metric_is_valid(metric: dict) -> bool:
    """正規化の分母として使える baseline 誤差かを判定する。"""
    return all(
        metric[h]["yaw"] > 0
        and (metric[h]["long"] > 0 or metric[h]["lat"] > 0)
        for h in HORIZONS
    )


def _baseline_metric_summary(metric: dict) -> str:
    """baseline 誤差をログ向けに整形する。"""
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
    lite_dir: Path,
    *,
    verbose: bool = False,
    log: bool = True,
    baseline_model_type: str = _BASELINE_MODEL,
    baseline_params: dict | None = None,
    baseline_acceleration_source: str = "accel",
) -> DatasetCtx | None:
    """1 dataset を読み込み、baseline を検証した DatasetCtx を返す。失敗時は None."""
    s5.LITE_DIR = lite_dir
    real = resolve_lite_bag(lite_dir, "real")
    if real is None:
        print(f"[WARN] real.lite が見つかりません: {lite_dir}", file=sys.stderr)
        return None
    try:
        data = s5.load_real_bag(real)
        t0_ns = s5.find_autonomous_start(data)
        base = s5._build_params()
        base["wheelbase"] = WHEELBASE
        s5.SUB_DT = base["sub_dt"]
        ctx = DatasetCtx(ds_id, data, t0_ns, base, {}, {})
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
    except MissingRequiredDataError:
        raise
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


def _load_one(args: tuple[str, Path]) -> DatasetCtx | None:
    """fork プールワーカー: 1 dataset を読み込み DatasetCtx を返す (失敗時 None)。

    fork による COW 継承で呼ばれるため、s5.LITE_DIR / s5.SUB_DT への書き込みは
    このワーカープロセス内のみに留まり、親プロセスには影響しない。
    """
    ds_id, lite_dir = args
    return _load_dataset_ctx(
        ds_id,
        lite_dir,
        verbose=_VERBOSE,
        log=False,
        baseline_model_type=_LOAD_BASELINE_MODEL,
        baseline_params=_LOAD_BASELINE_PARAMS,
        baseline_acceleration_source=_LOAD_BASELINE_ACCEL_SOURCE,
    )


def load_datasets(
    lite_dirs: list[tuple[str, Path]],
    n_jobs: int = 1,
    verbose: bool = False,
    *,
    baseline_model_type: str = _BASELINE_MODEL,
    baseline_params: dict | None = None,
    baseline_acceleration_source: str = "accel",
) -> list[DatasetCtx]:
    """収集された real.lite を読み込み DatasetCtx を構築する (baseline 誤差も計算)。

    n_jobs > 1 のとき fork プールで D 本を並列ロードする。入力順を保持するため
    pool.imap の返り値順 (None 除外) をそのまま使い、tie-break 再現を保証する。
    verbose=False (既定) のとき per-dataset の [load] 行を抑制し集計サマリのみ出力する。
    """
    global _LOAD_BASELINE_MODEL, _LOAD_BASELINE_PARAMS, _LOAD_BASELINE_ACCEL_SOURCE, _VERBOSE  # noqa: PLW0603
    _VERBOSE = verbose
    _LOAD_BASELINE_MODEL = baseline_model_type
    _LOAD_BASELINE_PARAMS = dict(baseline_params or {})
    _LOAD_BASELINE_ACCEL_SOURCE = normalize_accel_source(baseline_acceleration_source)

    if n_jobs <= 1:
        # --jobs 1 の逐次パス (再現性の基準)
        ctxs: list[DatasetCtx] = []
        for ds_id, lite_dir in lite_dirs:
            # 多数の異種データセットを横断するため、ロード失敗 (AUTONOMOUS 窓なし・トピック欠落・
            # baseline 誤差が NaN/0 等) は致命にせず skip する。
            ctx = _load_dataset_ctx(
                ds_id,
                lite_dir,
                verbose=verbose,
                log=verbose,
                baseline_model_type=baseline_model_type,
                baseline_params=baseline_params,
                baseline_acceleration_source=baseline_acceleration_source,
            )
            if ctx is None:
                continue
            ctxs.append(ctx)
        n_skip = len(lite_dirs) - len(ctxs)
        print(f"[INFO] ロード完了: {len(ctxs)}/{len(lite_dirs)} ({n_skip} SKIP)", file=sys.stderr)
        return ctxs

    # 並列パス: fork プールで D 本を並列ロード (pool.imap で順序保証・ピーク転送を平準化)
    mp_ctx = multiprocessing.get_context("fork")
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(lite_dirs))
    chunksize = pool_chunksize(len(lite_dirs), n_workers)
    with mp_ctx.Pool(n_workers) as pool:
        results = list(pool.imap(_load_one, lite_dirs, chunksize=chunksize))

    ctxs = []
    for ctx in results:
        if ctx is not None:
            ctxs.append(ctx)
            if verbose:
                print(f"[load] {ctx.dataset_id}: {_gear_metric_summary(ctx)}  {_baseline_metric_summary(ctx.base_metric)}")

    n_skip = sum(1 for r in results if r is None)
    print(f"[INFO] ロード完了: {len(ctxs)}/{len(lite_dirs)} ({n_skip} SKIP)", file=sys.stderr)

    # fork ワーカーは s5.SUB_DT を自身のコピーに書いているため、親の値は変わっていない。
    # 後続の run_rollout が正しい sub_dt を使えるよう、最初の有効 ctx の値で親を更新する。
    if ctxs:
        s5.SUB_DT = ctxs[0].base["sub_dt"]
    return ctxs


def aggregate(
    ctxs: list[DatasetCtx], override: dict, model_type: str, acceleration_source: str = "accel"
) -> dict:
    """各 ctx を rollout 評価し lib._multi_agg.aggregate_normalized で横断集約する薄ラッパ。

    返り値スキーマは aggregate_normalized と同一
    ({per_ds: [{dataset_id, by_h}], by_h: {h: {nyaw_mean,...,nlat_worst}}})。
    """
    per_ds_metrics = [
        (ctx.dataset_id, _eval(ctx, override, model_type, acceleration_source))
        for ctx in ctxs
    ]
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    return aggregate_normalized(per_ds_metrics, baselines)


# ---------------------------------------------------------------------------
# 並列評価サポート (fork プールによる per-(trial × dataset) 並列化)
# ---------------------------------------------------------------------------

# worker が参照するグローバル ctxs。fork 前に親がセットし、子は COW で読み取り専用で継承する。
# これにより ctxs の pickle 転送ゼロ・メモリ共有を実現する。
_CTXS: list[DatasetCtx] = []


def _worker_eval_one(args: tuple[int, int, dict, str, str]) -> tuple[int, int, str, dict]:
    """プールワーカー: _CTXS[ctx_idx] で 1 dataset の誤差を評価し返す (fork COW 継承)。

    返り値: (trial_idx, ctx_idx, dataset_id, metrics)
    親が trial 単位に再集約するため、pickle 転送量は per-trial-aggregate より大幅に小さい。
    """
    trial_idx, ctx_idx, override, model_type, acceleration_source = args
    ctx = _CTXS[ctx_idx]
    metrics = _eval(ctx, override, model_type, acceleration_source)
    return trial_idx, ctx_idx, ctx.dataset_id, metrics


def _eval_grid(
    pool,
    ctxs: list[DatasetCtx],
    trials: list[dict],
    model_type: str,
    n_jobs: int = 1,
    agg_fn=None,
    acceleration_source: str = "accel",
) -> list[dict]:
    """trials を並列 (pool 非 None) または逐次で aggregate 評価し元順の agg リストを返す。

    pool が None のとき逐次実行して ctxs を直接使う (--jobs 1 での完全逐次互換)。
    pool が非 None のとき (trial_idx, ctx_idx) を平坦化して pool.map し、親で trial 単位に
    再集約する (per-(trial×dataset) 並列化)。
    いずれも返り値は trials と同順の agg リスト (tie-break の決定論的再現を保証)。
    agg_fn: None → aggregate_normalized を使用。
            callable(per_ds_metrics, baselines) → 任意の集約関数。
    """
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
    # (trial_idx, ctx_idx) の直積を生成し一括 map
    units = [
        (ti, ci, trials[ti], model_type, acceleration_source)
        for ti in range(n_trials)
        for ci in range(n_ctxs)
    ]
    chunksize = pool_chunksize(len(units), n_jobs)
    raw = pool.map(_worker_eval_one, units, chunksize=chunksize)

    # trial 単位に per_ds_metrics を再構成し集約
    per_trial: list[list[tuple[str, dict]]] = [[] for _ in range(n_trials)]
    for ti, _ci, ds_id, metrics in raw:
        per_trial[ti].append((ds_id, metrics))
    return [_agg(pm, baselines) for pm in per_trial]


def _run_worker(
    worker_id: int,
    db_url: str,
    n_trials_w: int,
    n_trials: int,
    cur_best: dict,
    CONTINUOUS_SPACE: dict,
    explore_delay: bool,
    DELAY_CANDIDATES: tuple[float, ...],
    ctxs_search: list[DatasetCtx],
    cur_model: str,
    cur_accel_source: str,
    score_fn,
    worst_w: float,
    out_path,
) -> None:
    """fork プールワーカー: SQLite 経由で Optuna study.optimize を並列実行。"""
    import os
    import yaml
    import optuna

    set_worker_thread_env_defaults()

    # Reload study in worker
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
        for pname, (lo, hi) in CONTINUOUS_SPACE.items():
            params[pname] = trial.suggest_float(pname, lo, hi)
        if explore_delay:
            params["acc_time_delay"] = trial.suggest_categorical(
                "acc_time_delay", DELAY_CANDIDATES
            )

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

    def worker_log(study: optuna.Study, trial: optuna.trial.FrozenTrial) -> None:
        if trial.state == optuna.trial.TrialState.COMPLETE:
            try:
                n_completed = len(study.trials)
                best_val = study.best_value
            except Exception:
                n_completed = 0
                best_val = trial.value

    study.optimize(worker_objective, n_trials=n_trials_w, callbacks=[worker_log])



def robust_search(
    ctxs: list[DatasetCtx],
    cfg,
    *,
    case_name: str = "current",
    n_trials: int = 200,
    n_jobs: int = 1,
    search_subsample: int | None = None,
    out_path: Path | None = None,
    extra_enqueue: list[dict] | None = None,
    worst_w: float = WORST_W,
    phase: int = 0,
    phase_fixed_params: dict | None = None,
    base_override: dict | None = None,
) -> dict:
    """Optuna TPE でデータセット横断ロバスト最適化 (連続値 + 離散 delay)。

    探索空間:
      k_us / steer_time_constant / debug_steer_scaling_factor / acc_time_constant:
          連続値 (suggest_float)。TPE が実数空間を滑らかに探索する。
      acc_time_delay: 離散 (suggest_categorical)。値を離散に保つことで gt_cache を
          全 trial で完全ヒットさせ COW 共有の効率を維持する。
    その他のパラメータは case_name の scenario.yaml 定義値を固定継承する。
    warm start: trial 0 に scenario.yaml の case_name 定義値を、extra_enqueue 指定時は
        続く trial に追加 params を投入する。

    phase: 0=全パラメータ同時最適化(既定), 1=acc のみ/long スコア, 2=steer のみ/yaw+lat スコア。
    phase_fixed_params: phase=2 で acc 系を固定する値 (Phase 1 の --out YAML から読み込む)。
    base_override: 直接同定 (--phase-params) の全 params。CONTINUOUS_SPACE / acc_time_delay を
        除くキー (= steer_time_delay/steer_rate_lim 等) を cur_best に透過し、Optuna が探索しない
        同定値を tuned_params.yaml → apply → closed_loop まで届ける。
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
        print(
            f"[INFO] search_subsample={search_subsample}: "
            f"探索フェーズは ctxs[:{min(search_subsample, len(ctxs))}] を使用 "
            f"(全件={len(ctxs)})。最終 score 評価は全件。"
        )

    # Phase に応じて探索空間・スコア関数・delay 探索フラグを決定
    # acc_time_delay は旧 sweeps と同じ離散候補 (gt_cache COW 共有のため離散に保つ)
    DELAY_CANDIDATES: tuple[float, ...] = (0.10, 0.15, 0.20, 0.30, 0.40, 0.50)

    if phase == 1:
        # Phase 1: acc パラメータのみ最適化 (long スコア)。steer 系は cur_best に固定。
        CONTINUOUS_SPACE: dict[str, tuple[float, float]] = {
            "acc_time_constant":        (0.20, 0.30),  # 直接同定値(phase1: ~0.25)近傍に限定し、yaw/lat とのトレードオフで非物理的な値に逃げるのを防ぐ
            "debug_acc_scaling_factor": (0.80, 1.20),
        }
        score_fn = acc_score
        explore_delay = True
        print("[Phase 1] acc パラメータ最適化 (long スコア)。steer 系は cur_best から固定。")
    elif phase == 2:
        # Phase 2: steer パラメータのみ最適化 (yaw+lat スコア)。acc 系は固定。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.80),
            "debug_steer_scaling_factor": (0.75, 1.20),  # (0.80, 1.05) → (0.75, 1.20) に拡張
            "k_us":                       (0.0,  0.05),
            "steer_dead_band":            (0.0,  0.02),   # アクチュエータ不感帯 [rad]
            "steer_bias":                 (-0.01, 0.01),  # 系統的ステアオフセット [rad]
        }
        score_fn = steer_score
        explore_delay = False
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 2] steer パラメータ最適化 (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
        else:
            print("[Phase 2] steer パラメータ最適化 (yaw+lat スコア)。acc 系は cur_best から固定。")
    else:
        # Phase 0: 全パラメータ同時最適化 (従来の robust_score)
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.80),
            "debug_steer_scaling_factor": (0.75, 1.20),  # (0.80, 1.05) → (0.75, 1.20) に拡張
            "acc_time_constant":          (0.20, 0.30),  # 直接同定値(phase1: ~0.25)近傍に限定し、yaw/lat とのトレードオフで非物理的な値に逃げるのを防ぐ
            "debug_acc_scaling_factor":   (0.80, 1.20),
            "k_us":                       (0.0,  0.05),

            "steer_dead_band":            (0.0,  0.02),   # アクチュエータ不感帯 [rad]
            "steer_bias":                 (-0.01, 0.01),  # 系統的ステアオフセット [rad]
        }
        score_fn = robust_score
        explore_delay = True

    # Step 1.5: 直接同定 (--phase-params) の値のうち Optuna が探索しないキーを cur_best に透過。
    # これで fit_lon/fit_steer が出した param が既定で tuned_params.yaml → apply → closed_loop に届く。
    # 除外: CONTINUOUS_SPACE (Optuna が上書き探索) / acc_time_delay (categorical 探索)。
    if base_override:
        searched = set(CONTINUOUS_SPACE) | {"acc_time_delay"}
        passthrough = {k: v for k, v in base_override.items() if k not in searched}
        if passthrough:
            cur_best.update(passthrough)
            print(f"[Phase {phase}] 直接同定値を透過 (Optuna 非探索): {sorted(passthrough)}")

    # gt 事前計算 (fork 前に親で完了し COW 共有)
    acc_delay_set: set[float] = set(DELAY_CANDIDATES) if explore_delay else set()
    if "acc_time_delay" in cur_best:
        acc_delay_set.add(float(cur_best["acc_time_delay"]))
    for ctx in ctxs:
        for ad in (acc_delay_set or {float(cur_best.get("acc_time_delay", 0.1))}):
            merged = dict(ctx.base)
            merged.update(cur_best)
            merged["acc_time_delay"] = ad
            key = (cur_accel_source, *tuple(round(float(merged[k]), 9) for k in _GT_KEYS))
            if key not in ctx.gt_cache:
                data = _ctx_data(ctx, cur_accel_source)
                ctx.gt_cache[key] = s5._prepare_gt(data, ctx.t0_ns, merged)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    sampler = optuna.samplers.TPESampler(multivariate=True, seed=42)

    # warm start: trial 0+ に既知良点を投入
    delay_list = list(DELAY_CANDIDATES)

    def _make_enqueue(params: dict) -> dict:
        eq: dict = {k: float(params[k]) for k in CONTINUOUS_SPACE if k in params}
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
        # Sequential path: in-memory Optuna (fully deterministic & low overhead)
        try:
            init_agg = _eval_grid(
                None, ctxs_search, [cur_best], cur_model, 1, agg_fn=None,
                acceleration_source=cur_accel_source,
            )[0]
            init_score = score_fn(init_agg, worst_w=worst_w)
            phase_label = f"phase={phase}" if phase else "phase=0(all)"
            print(f"\n## Optuna TPE ({case_name}, {n_trials} trials, cross-dataset normalized, worst_w={worst_w}, {phase_label})")

            best_result: dict = {"params": dict(cur_best), "score": init_score, "agg": init_agg}
            _checkpoint(cur_best, init_score)

            def objective(trial: optuna.Trial) -> float:
                params = dict(cur_best)
                for pname, (lo, hi) in CONTINUOUS_SPACE.items():
                    params[pname] = trial.suggest_float(pname, lo, hi)
                if explore_delay:
                    params["acc_time_delay"] = trial.suggest_categorical("acc_time_delay", DELAY_CANDIDATES)

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
                    print(
                        f"trial {trial.number + 1:3d}/{n_trials}"
                        f"  score={trial.value:.4f}"
                        f"  best={study.best_value:.4f}"
                        f"  {trial.params}"
                    )

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
        finally:
            pass

    # Parallel path: SQLite + Process-based concurrency (zero-copy for ctxs via fork COW)
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
        phase_label = f"phase={phase}" if phase else "phase=0(all)"
        print(f"\n## Optuna TPE ({case_name}, {n_trials} trials, cross-dataset normalized, worst_w={worst_w}, {phase_label}) [SQLite Process Parallel: {n_jobs} jobs]")
        _checkpoint(cur_best, init_score)

        study = optuna.create_study(
            study_name="robust_search",
            storage=db_url,
            direction="minimize",
            sampler=sampler,
            load_if_exists=True,
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
                args=(
                    worker_id,
                    db_url,
                    n_trials_w,
                    n_trials,
                    cur_best,
                    CONTINUOUS_SPACE,
                    explore_delay,
                    DELAY_CANDIDATES,
                    ctxs_search,
                    cur_model,
                    cur_accel_source,
                    score_fn,
                    worst_w,
                    out_path,
                ),
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
    """収集ディレクトリ配下の <dataset_id>/real.lite を列挙する。"""
    return [
        (e.dataset_id, e.dir)
        for e in discover_collection(collection_dir)
        if e.dir is not None and e.real_lite is not None
    ]


def _ds_recording_date(ds_dir: Path) -> datetime.date | None:
    """DS の録画日 (UTC) を mcap サマリから取得する。取得失敗時は None。"""
    from mcap.reader import make_reader  # noqa: PLC0415

    bag = resolve_lite_bag(ds_dir, "real")
    if bag is None:
        return None
    if bag.is_dir():
        mcaps = sorted(bag.glob("*.mcap"))
        if not mcaps:
            return None
        bag = mcaps[0]
    try:
        with bag.open("rb") as f:
            stats = make_reader(f).get_summary().statistics
        if stats is None or stats.message_start_time == 0:
            return None
        return datetime.datetime.fromtimestamp(
            stats.message_start_time / 1e9, tz=datetime.timezone.utc
        ).date()
    except Exception:
        return None


def _filter_by_date(
    lite_dirs: list[tuple[str, Path]],
    before: datetime.date | None,
    after: datetime.date | None,
) -> list[tuple[str, Path]]:
    """録画日 (UTC) で DS をフィルタする。
    before: この日より前 (exclusive)。after: この日以降 (inclusive)。
    """
    if before is None and after is None:
        return lite_dirs
    filtered = []
    skipped = 0
    for ds_id, ds_dir in lite_dirs:
        rec = _ds_recording_date(ds_dir)
        if rec is None:
            print(f"[WARN] {ds_id}: 録画日取得失敗 → スキップ", file=sys.stderr)
            skipped += 1
            continue
        if before is not None and rec >= before:
            continue
        if after is not None and rec < after:
            continue
        filtered.append((ds_id, ds_dir))
    print(
        f"[date filter] {len(lite_dirs)} → {len(filtered)} datasets "
        f"(before={before or '—'}, after={after or '—'}, skip={skipped})"
    )
    return filtered


def main() -> None:
    ap = argparse.ArgumentParser(
        description="マルチデータセット横断のロバスト current 同定 (robust_search)"
    )
    ap.add_argument("--collection-dir", default=str(Path(__file__).parent / "sample" / "multi"))
    ap.add_argument(
        "--case",
        default="current",
        metavar="CASE_NAME",
        help="チューニング対象の scenario.yaml models エントリ名 (既定: current)",
    )
    ap.add_argument("--scenario", default=str(Path(__file__).parent / "sample" / "scenario.yaml"),
                    help="scenario.yaml のパス (Conditions.models / cases を含む)")
    ap.add_argument(
        "--lite-dir",
        action="append",
        default=[],
        metavar="DATASET_ID=LITE_DIR",
        help="収集を使わず直接指定 (複数可)",
    )
    ap.add_argument(
        "--out",
        default="",
        help="FINAL params を保存する YAML ファイルパス (省略時は保存しない)。"
        "scenario.yaml の models へ反映する際の受け渡しファイルとして使う",
    )
    ap.add_argument(
        "--jobs",
        type=int,
        default=default_parallel_jobs(),
        metavar="N",
        help="並列ワーカー数 (既定: REAL_LOG_SIM_COMPARISON_JOBS または CPU コア数)。1 で逐次実行",
    )
    ap.add_argument(
        "--n-trials",
        type=int,
        default=200,
        metavar="N",
        help="Optuna TPE のトライアル数 (既定: 200)",
    )
    ap.add_argument(
        "--search-subsample",
        type=int,
        default=None,
        metavar="N",
        help=(
            "探索フェーズで使用するデータセット数の上限 (既定 None=全件)。"
            "指定時は探索中のみ ctxs[:N] を使い、最終 score 評価は全件で行う。"
            "compute のみ削減でロードは全件行われるため RAM は削減されない。"
            "結果が変わりうる (既定オフ)。"
        ),
    )
    ap.add_argument(
        "--enqueue-params",
        action="append",
        default=[],
        metavar="YAML_PATH",
        help=(
            "warm start として追加 enqueue する params YAML (複数可)。"
            "tuned_params_*.yaml を指定し既知良点から探索を始める。"
            "acc_time_delay がない場合は最近接の離散候補に snap する"
        ),
    )
    ap.add_argument(
        "--worst-weight",
        type=float,
        default=WORST_W,
        metavar="W",
        help=(
            f"robust_score の worst-case 項の重み (既定: {WORST_W})。"
            "増やすと worst >1.0 を優先的に改善する。score のスケールが変わるため "
            "異なる --worst-weight 間での score 比較は無効 (worst サブメトリクスで比較すること)。"
        ),
    )
    ap.add_argument(
        "--verbose",
        action="store_true",
        help="per-dataset の [load] 行を出力する (既定: 集計サマリのみ)。--verbose 指定時は SKIP の traceback も表示",
    )
    ap.add_argument(
        "--ds-before",
        default="",
        metavar="YYYY-MM-DD",
        help="この日付より前 (exclusive, UTC) に録画されたデータセットのみ使用する。understeer_compensation 切替前データの抽出に使う",
    )
    ap.add_argument(
        "--ds-after",
        default="",
        metavar="YYYY-MM-DD",
        help="この日付以降 (inclusive, UTC) に録画されたデータセットのみ使用する。understeer_compensation 切替後データの抽出に使う",
    )
    ap.add_argument(
        "--phase",
        type=int,
        default=0,
        choices=[0, 1, 2],
        help=(
            "チューニングフェーズ (既定: 0=全パラメータ同時最適化)。"
            "1=acc のみ探索・long スコア (steer 系は scenario.yaml の定義値に固定)。"
            "2=steer のみ探索・yaw+lat スコア (acc 系は scenario.yaml 定義値または --phase-params に固定)。"
        ),
    )
    ap.add_argument(
        "--phase-params",
        default="",
        metavar="YAML_PATH",
        help=(
            "--phase 2 で acc 系 (acc_time_constant / acc_time_delay) を固定する YAML。"
            "Phase 1 の --out で生成したファイルを指定する。省略時は scenario.yaml の定義値を使用。"
        ),
    )
    ap.add_argument(
        "--report",
        default="",
        metavar="HTML_PATH",
        help=(
            "最適化完了後に HTML レポートを生成するパス。"
            "省略時はレポートを生成しない。"
            "--out が指定されている場合、今回の結果が自動で比較対象に含まれる。"
        ),
    )
    ap.add_argument(
        "--report-compare",
        action="append",
        default=[],
        metavar="[LABEL=]YAML_PATH",
        help=(
            "レポートで比較する既存の params YAML（複数可）。"
            "'label=path' 形式でラベルを指定、省略時はファイル名 stem をラベルに使う。"
        ),
    )
    ap.add_argument(
        "--report-worst-weight",
        type=float,
        default=1.0,
        metavar="W",
        help="レポートのスコア表示に使う worst 重み（既定: 1.0）",
    )
    args = ap.parse_args()

    if args.lite_dir:
        lite_dirs = []
        for spec in args.lite_dir:
            ds_id, raw = spec.split("=", 1)
            lite_dirs.append((ds_id.strip(), Path(raw.strip())))
    else:
        lite_dirs = _discover(Path(args.collection_dir))

    ds_before = datetime.date.fromisoformat(args.ds_before) if args.ds_before else None
    ds_after  = datetime.date.fromisoformat(args.ds_after)  if args.ds_after  else None
    lite_dirs = _filter_by_date(lite_dirs, ds_before, ds_after)

    if not lite_dirs:
        print(f"ERROR: real.lite が見つかりません: {args.collection_dir}", file=sys.stderr)
        sys.exit(1)

    n_jobs = normalize_parallel_jobs(args.jobs)

    # baseline model は scenario.yaml (Conditions.overlay.reference_tag) を唯一の SSOT として
    # 解決する。load_datasets より前に確定させ、baseline 側の rollout 評価に明示的に渡す
    # (_BASELINE_MODEL 定数への暗黙依存は事故の元 — 2026-07-07 に実際発生)。
    cfg = load_models_doc(args.scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    baseline_accel_source = cfg.models[baseline_case].acceleration_source
    cur_case = cfg.find_case(args.case)
    cur_model = cur_case.vehicle_model_type
    print(
        f"[INFO] baseline model = {baseline_model_type} "
        f"(scenario.yaml の '{baseline_case}' ケース, accel_source={baseline_accel_source})"
    )
    print(
        f"[INFO] current model  = {cur_model} "
        f"('{args.case}' ケース, accel_source={cur_case.acceleration_source})"
    )

    # OMP スレッド数を抑制してから全 fork (load_datasets / robust_search 両方をカバー):
    # worker 内の BLAS/OpenMP が親コア数分のスレッドを再起動するのを防ぎ、
    # コア間の CPU リソース競合を回避する。load_datasets の前に設定しないと
    # 並列ロード中の baseline rollout が oversubscribe する。
    set_worker_thread_env_defaults()

    ctxs = load_datasets(
        lite_dirs, n_jobs=n_jobs, verbose=args.verbose,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=baseline_accel_source,
    )
    if len(ctxs) < 1:
        print("ERROR: 有効な dataset が 0 件", file=sys.stderr)
        sys.exit(1)

    # グローバル _CTXS に ctxs をセット (fork COW 継承パターン)。
    # pool.map は pickle 転送せず fork した子プロセスがグローバルを直接参照する。
    global _CTXS  # noqa: PLW0603
    _CTXS = ctxs

    out_path = Path(args.out) if args.out else None

    extra_enqueue: list[dict] | None = None
    if args.enqueue_params:
        extra_enqueue = []
        for path_str in args.enqueue_params:
            p = Path(path_str)
            with p.open("r") as f:
                data = yaml.safe_load(f)
            extra_enqueue.append(data["params"])
        print(f"[INFO] extra enqueue: {len(extra_enqueue)} params loaded")

    phase_fixed_params: dict | None = None
    base_override: dict | None = None
    if args.phase_params:
        p = Path(args.phase_params)
        with p.open("r") as f:
            phase_data = yaml.safe_load(f)
        all_params = phase_data.get("params", phase_data)
        base_override = all_params
        acc_keys = {"acc_time_constant", "acc_time_delay"}
        if args.phase == 2:
            fixed_keys = acc_keys
        else:
            fixed_keys = set()
        phase_fixed_params = {k: v for k, v in all_params.items() if k in fixed_keys}
        print(f"[INFO] Phase {args.phase} 固定 params (from {args.phase_params}): {list(phase_fixed_params.keys())}")

    result = robust_search(
        ctxs,
        cfg,
        case_name=args.case,
        n_trials=args.n_trials,
        n_jobs=n_jobs,
        search_subsample=args.search_subsample,
        out_path=out_path,
        extra_enqueue=extra_enqueue,
        worst_w=args.worst_weight,
        phase=args.phase,
        phase_fixed_params=phase_fixed_params,
        base_override=base_override,
    )

    if out_path is not None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            yaml.safe_dump(
                {
                    "params": result["params"],
                    "score": result["score"],
                    "metadata": {
                        "collection_dir": args.collection_dir,
                        "n_datasets": len(lite_dirs),
                        "n_valid": len(ctxs),
                        "scenario": args.scenario,
                        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
                    },
                },
                allow_unicode=True,
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        print(f"[INFO] FINAL params 保存: {out_path}")

    if args.report:
        from .lib._tune_report import generate_report  # noqa: PLC0415

        # 今回の結果を最初のエントリとして配置
        report_configs: dict[str, dict] = {}
        result_label = out_path.stem if out_path else "new_result"
        report_configs[result_label] = result["params"]

        # 比較対象 YAML をパース（label=path 形式 or path のみ）
        for spec in args.report_compare:
            if "=" in spec:
                label, path_str = spec.split("=", 1)
            else:
                label = Path(spec).stem
                path_str = spec
            with open(path_str) as f:
                compare_data = yaml.safe_load(f)
            report_configs[label] = compare_data["params"]

        generate_report(
            ctxs=ctxs,
            configs=report_configs,
            out_html=Path(args.report),
            model_type=cur_model,
            eval_fn=_eval,
            stride=STRIDE,
            worst_w=args.report_worst_weight,
        )


if __name__ == "__main__":
    main()
