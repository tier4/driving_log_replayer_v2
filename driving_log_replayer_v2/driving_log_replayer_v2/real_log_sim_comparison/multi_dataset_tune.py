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
from dataclasses import dataclass
import multiprocessing
import os
from pathlib import Path
import sys
import traceback

import optuna
import yaml

from . import step5_analyze_nstep as s5
from .lib._cases_config import load_cases_config
from .lib._io import resolve_lite_bag
from .lib._multi_agg import HORIZONS, WORST_W, acc_score, aggregate_normalized, format_agg, robust_score, steer_score

STRIDE = 5
WHEELBASE = 4.76012
_BASELINE_MODEL = "delay_steer_acc_geared_wo_fall_guard"
# _prepare_gt は params の delay/wheelbase/sub_dt にのみ依存 (run_rollout docstring)
_GT_KEYS = ("acc_time_delay", "steer_time_delay", "wheelbase", "sub_dt")
# fork 前に load_datasets が設定し、fork 後の worker は COW で継承する (P-5/P-7)
_VERBOSE: bool = False


@dataclass
class DatasetCtx:
    """1 データセットの rollout 実行コンテキスト (data/t0/base params を保持)。"""

    dataset_id: str
    data: dict
    t0_ns: int
    base: dict
    gt_cache: dict  # gt-key -> gt
    base_metric: dict  # baseline (無補正) の {h: {yaw, pos, long, lat, steer}} (h ∈ HORIZONS)


def _eval(ctx: DatasetCtx, override: dict, model_type: str) -> dict:
    """1 dataset・1 パラメータ組の horizon 別終端誤差 RMSE {h: {yaw,pos,long,lat,steer}}。"""
    params = dict(ctx.base)
    params.update(override)
    key = tuple(round(float(params[k]), 9) for k in _GT_KEYS)
    gt = ctx.gt_cache.get(key)
    if gt is None:
        gt = s5._prepare_gt(ctx.data, ctx.t0_ns, params)
        ctx.gt_cache[key] = gt
    rmse = s5.eval_rollout_rmse(
        ctx.data, ctx.t0_ns, params, model_type, horizons=HORIZONS, stride=STRIDE, gt=gt
    )
    return {h: rmse[h] for h in HORIZONS}


def _load_one(args: tuple[str, Path]) -> DatasetCtx | None:
    """fork プールワーカー: 1 dataset を読み込み DatasetCtx を返す (失敗時 None)。

    fork による COW 継承で呼ばれるため、s5.LITE_DIR / s5.SUB_DT への書き込みは
    このワーカープロセス内のみに留まり、親プロセスには影響しない。
    """
    ds_id, lite_dir = args
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
        ctx.base_metric = _eval(ctx, {}, _BASELINE_MODEL)
        if not all(
            ctx.base_metric[h]["yaw"] > 0
            and (ctx.base_metric[h]["long"] > 0 or ctx.base_metric[h]["lat"] > 0)
            for h in HORIZONS
        ):
            print(f"[SKIP] {ds_id}: baseline 誤差が無効 (yaw/縦/横≤0 or NaN)", file=sys.stderr)
            return None
    except Exception as e:  # noqa: BLE001
        msg = f"[SKIP] {ds_id}: ロード失敗 ({type(e).__name__}: {e})"
        if _VERBOSE:
            tb_lines = traceback.format_exc().strip().splitlines()
            msg += "\n  " + "\n  ".join(tb_lines[-3:])
        print(msg, file=sys.stderr)
        return None
    return ctx


def load_datasets(
    lite_dirs: list[tuple[str, Path]], n_jobs: int = 1, verbose: bool = False
) -> list[DatasetCtx]:
    """収集された real.lite を読み込み DatasetCtx を構築する (baseline 誤差も計算)。

    n_jobs > 1 のとき fork プールで D 本を並列ロードする。入力順を保持するため
    pool.imap の返り値順 (None 除外) をそのまま使い、tie-break 再現を保証する。
    verbose=False (既定) のとき per-dataset の [load] 行を抑制し集計サマリのみ出力する。
    """
    global _VERBOSE  # noqa: PLW0603
    _VERBOSE = verbose

    if n_jobs <= 1:
        # --jobs 1 の逐次パス (再現性の基準)
        ctxs: list[DatasetCtx] = []
        for ds_id, lite_dir in lite_dirs:
            s5.LITE_DIR = lite_dir
            real = resolve_lite_bag(lite_dir, "real")
            if real is None:
                print(f"[WARN] real.lite が見つかりません: {lite_dir}", file=sys.stderr)
                continue
            # 多数の異種データセットを横断するため、ロード失敗 (AUTONOMOUS 窓なし・トピック欠落・
            # baseline 誤差が NaN/0 等) は致命にせず skip する。
            try:
                data = s5.load_real_bag(real)
                t0_ns = s5.find_autonomous_start(data)
                base = s5._build_params()
                base["wheelbase"] = WHEELBASE
                s5.SUB_DT = base["sub_dt"]
                ctx = DatasetCtx(ds_id, data, t0_ns, base, {}, {})
                ctx.base_metric = _eval(ctx, {}, _BASELINE_MODEL)  # 正規化基準 (horizon 別)
                # 全 horizon で yaw>0 かつ縦横いずれかが正でなければ正規化分母が立たない
                if not all(
                    ctx.base_metric[h]["yaw"] > 0
                    and (ctx.base_metric[h]["long"] > 0 or ctx.base_metric[h]["lat"] > 0)
                    for h in HORIZONS
                ):
                    print(f"[SKIP] {ds_id}: baseline 誤差が無効 (yaw/縦/横≤0 or NaN)", file=sys.stderr)
                    continue
            except Exception as e:  # noqa: BLE001
                msg = f"[SKIP] {ds_id}: ロード失敗 ({type(e).__name__}: {e})"
                if verbose:
                    tb_lines = traceback.format_exc().strip().splitlines()
                    msg += "\n  " + "\n  ".join(tb_lines[-3:])
                print(msg, file=sys.stderr)
                continue
            ctxs.append(ctx)
            if verbose:
                print(
                    f"[load] {ds_id}: "
                    + "  ".join(
                        f"baseline@N{h} yaw={ctx.base_metric[h]['yaw']:.4f} "
                        f"縦={ctx.base_metric[h]['long']:.3f} 横={ctx.base_metric[h]['lat']:.3f}"
                        for h in HORIZONS
                    )
                )
        n_skip = len(lite_dirs) - len(ctxs)
        print(f"[INFO] ロード完了: {len(ctxs)}/{len(lite_dirs)} ({n_skip} SKIP)", file=sys.stderr)
        return ctxs

    # 並列パス: fork プールで D 本を並列ロード (pool.imap で順序保証・ピーク転送を平準化)
    mp_ctx = multiprocessing.get_context("fork")
    chunksize = max(1, len(lite_dirs) // (n_jobs * 4))
    with mp_ctx.Pool(n_jobs) as pool:
        results = list(pool.imap(_load_one, lite_dirs, chunksize=chunksize))

    ctxs = []
    for ctx in results:
        if ctx is not None:
            ctxs.append(ctx)
            if verbose:
                print(
                    f"[load] {ctx.dataset_id}: "
                    + "  ".join(
                        f"baseline@N{h} yaw={ctx.base_metric[h]['yaw']:.4f} "
                        f"縦={ctx.base_metric[h]['long']:.3f} 横={ctx.base_metric[h]['lat']:.3f}"
                        for h in HORIZONS
                    )
                )

    n_skip = sum(1 for r in results if r is None)
    print(f"[INFO] ロード完了: {len(ctxs)}/{len(lite_dirs)} ({n_skip} SKIP)", file=sys.stderr)

    # fork ワーカーは s5.SUB_DT を自身のコピーに書いているため、親の値は変わっていない。
    # 後続の run_rollout が正しい sub_dt を使えるよう、最初の有効 ctx の値で親を更新する。
    if ctxs:
        s5.SUB_DT = ctxs[0].base["sub_dt"]
    return ctxs


def aggregate(ctxs: list[DatasetCtx], override: dict, model_type: str) -> dict:
    """各 ctx を rollout 評価し lib._multi_agg.aggregate_normalized で横断集約する薄ラッパ。

    返り値スキーマは aggregate_normalized と同一
    ({per_ds: [{dataset_id, by_h}], by_h: {h: {nyaw_mean,...,nlat_worst}}})。
    """
    per_ds_metrics = [(ctx.dataset_id, _eval(ctx, override, model_type)) for ctx in ctxs]
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    return aggregate_normalized(per_ds_metrics, baselines)


# ---------------------------------------------------------------------------
# 並列評価サポート (fork プールによる per-(trial × dataset) 並列化)
# ---------------------------------------------------------------------------

# worker が参照するグローバル ctxs。fork 前に親がセットし、子は COW で読み取り専用で継承する。
# これにより ctxs の pickle 転送ゼロ・メモリ共有を実現する。
_CTXS: list[DatasetCtx] = []


def _worker_eval_one(args: tuple[int, int, dict, str]) -> tuple[int, int, str, dict]:
    """プールワーカー: _CTXS[ctx_idx] で 1 dataset の誤差を評価し返す (fork COW 継承)。

    返り値: (trial_idx, ctx_idx, dataset_id, metrics)
    親が trial 単位に再集約するため、pickle 転送量は per-trial-aggregate より大幅に小さい。
    """
    trial_idx, ctx_idx, override, model_type = args
    ctx = _CTXS[ctx_idx]
    metrics = _eval(ctx, override, model_type)
    return trial_idx, ctx_idx, ctx.dataset_id, metrics


def _eval_grid(
    pool,
    ctxs: list[DatasetCtx],
    trials: list[dict],
    model_type: str,
    n_jobs: int = 1,
) -> list[dict]:
    """trials を並列 (pool 非 None) または逐次で aggregate 評価し元順の agg リストを返す。

    pool が None のとき逐次実行して ctxs を直接使う (--jobs 1 での完全逐次互換)。
    pool が非 None のとき (trial_idx, ctx_idx) を平坦化して pool.map し、親で trial 単位に
    再集約する (per-(trial×dataset) 並列化)。
    いずれも返り値は trials と同順の agg リスト (tie-break の決定論的再現を保証)。
    """
    if pool is None:
        return [aggregate(ctxs, t, model_type) for t in trials]

    n_trials = len(trials)
    n_ctxs = len(ctxs)
    # (trial_idx, ctx_idx) の直積を生成し一括 map
    units = [
        (ti, ci, trials[ti], model_type)
        for ti in range(n_trials)
        for ci in range(n_ctxs)
    ]
    chunksize = max(1, len(units) // (n_jobs * 4))
    raw = pool.map(_worker_eval_one, units, chunksize=chunksize)

    # trial 単位に per_ds_metrics を再構成し親で aggregate_normalized を呼ぶ
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    per_trial: list[list[tuple[str, dict]]] = [[] for _ in range(n_trials)]
    for ti, _ci, ds_id, metrics in raw:
        per_trial[ti].append((ds_id, metrics))
    return [aggregate_normalized(pm, baselines) for pm in per_trial]


def robust_search(
    ctxs: list[DatasetCtx],
    cfg,
    *,
    case_name: str = "best_normal",
    n_trials: int = 200,
    n_jobs: int = 1,
    search_subsample: int | None = None,
    out_path: Path | None = None,
    extra_enqueue: list[dict] | None = None,
    worst_w: float = WORST_W,
    phase: int = 0,
    phase_fixed_params: dict | None = None,
) -> dict:
    """Optuna TPE でデータセット横断ロバスト最適化 (連続値 + 離散 delay)。

    探索空間:
      k_us / steer_time_constant / debug_steer_scaling_factor / acc_time_constant:
          連続値 (suggest_float)。TPE が実数空間を滑らかに探索する。
      acc_time_delay: 離散 (suggest_categorical)。値を離散に保つことで gt_cache を
          全 trial で完全ヒットさせ COW 共有の効率を維持する。
    n_substep 等その他のパラメータは case_name の scenario.yaml 定義値を固定継承する。
    warm start: trial 0 に scenario.yaml の case_name 定義値を、extra_enqueue 指定時は
        続く trial に追加 params を投入する。

    phase: 0=全パラメータ同時最適化(既定), 1=acc のみ/long スコア, 2=steer のみ/yaw+lat スコア。
    phase_fixed_params: phase=2 で acc 系を固定する値 (Phase 1 の --out YAML から読み込む)。
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

    if search_subsample:
        print(
            f"[INFO] search_subsample={search_subsample}: "
            f"探索フェーズは ctxs[:{min(search_subsample, len(ctxs))}] を使用 "
            f"(全件={len(ctxs)})。最終 score 評価は全件。"
        )

    # Phase に応じて探索空間・スコア関数・delay 探索フラグを決定
    # acc_time_delay は旧 sweeps と同じ離散候補 (gt_cache COW 共有のため離散に保つ)
    DELAY_CANDIDATES: tuple[float, ...] = (0.10, 0.15, 0.20, 0.30, 0.40, 0.50)
    # steer_time_delay の離散候補 (SUB_DT=1/30s の整数倍に近い値)
    # 0 step: 0.0315 (デフォルト ≈ 1step), 2step: 0.066, 3step: 0.099, 4step: 0.132
    STEER_DELAY_CANDIDATES: tuple[float, ...] = (0.0315, 0.066, 0.099, 0.132)

    if phase == 1:
        # Phase 1: acc パラメータのみ最適化 (long スコア)。steer 系は cur_best に固定。
        CONTINUOUS_SPACE: dict[str, tuple[float, float]] = {
            "acc_time_constant": (0.05, 1.20),  # 下限を 0.10→0.05 に拡張
        }
        score_fn = acc_score
        explore_delay = True
        print(f"[Phase 1] acc パラメータ最適化 (long スコア)。steer 系は cur_best から固定。")
    elif phase == 2:
        # Phase 2: steer パラメータのみ最適化 (yaw+lat スコア)。acc 系は固定。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.80),
            "debug_steer_scaling_factor": (0.75, 1.20),  # (0.80, 1.05) → (0.75, 1.20) に拡張
            "k_us":                       (0.0,  0.05),
            # 速度依存 k_us: vx < vx_lo で k_us_eff=0、vx > vx_hi で k_us_eff=k_us
            # 両方 0.0 (デフォルト) → ランプなし (全速度で k_us そのまま、後方互換)
            "k_us_vx_lo":                 (0.5,  6.0),
            "k_us_vx_hi":                 (1.0, 12.0),
            "steer_dead_band":            (0.0,  0.02),   # アクチュエータ不感帯 [rad]
            "steer_bias":                 (-0.01, 0.01),  # 系統的ステアオフセット [rad]
        }
        score_fn = steer_score
        explore_delay = False  # acc_time_delay は cur_best (Phase 1 best) に固定
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 2] steer パラメータ最適化 (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
        else:
            print(f"[Phase 2] steer パラメータ最適化 (yaw+lat スコア)。acc 系は cur_best から固定。")
    elif phase == 3:
        # Phase 3: no-ramp k_us 実験。k_us を全速度域で定数として扱い、ランプを無効化。
        # データ分析から物理的 k_us ≈ 0.011-0.016 なので (0.0, 0.025) 範囲でカバー。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.80),
            "debug_steer_scaling_factor": (0.75, 1.20),
            "k_us":                       (0.0,  0.025),  # 物理推定値 (0.011-0.016) をカバー
            "steer_dead_band":            (0.0,  0.02),
            "steer_bias":                 (-0.01, 0.01),
        }
        score_fn = steer_score
        explore_delay = False
        # ランプを無効化 (k_us_vx_lo=k_us_vx_hi=0.0 → 全速度で定数 k_us)
        cur_best["k_us_vx_lo"] = 0.0
        cur_best["k_us_vx_hi"] = 0.0
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 3] no-ramp k_us 実験 (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
        else:
            print(f"[Phase 3] no-ramp k_us 実験 (yaw+lat スコア)。acc 系は cur_best から固定。")
    elif phase == 4:
        # Phase 4: TC 絞り込み再チューニング。
        # コホートテストで TC=0.08 が全速度域で TC=0.142 を上回ることが判明したため、
        # TC 範囲を [0.05, 0.25] に絞って Phase 2 と同じ steer パラメータを再探索する。
        # k_us ランプも同時に再最適化し TC の最適値に合わせた k_us を探す。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.25),  # Phase 2 (0.05, 0.80) から絞り込み
            "debug_steer_scaling_factor": (0.75, 1.20),
            "k_us":                       (0.0,  0.05),
            "k_us_vx_lo":                 (0.5,  6.0),
            "k_us_vx_hi":                 (1.0, 12.0),
            "steer_dead_band":            (0.0,  0.02),
            "steer_bias":                 (-0.01, 0.01),
        }
        score_fn = steer_score
        explore_delay = False
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 4] TC 絞り込み再チューニング (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
        else:
            print(f"[Phase 4] TC 絞り込み再チューニング (yaw+lat スコア)。acc 系は cur_best から固定。")
    elif phase == 5:
        # Phase 5: TC=0.12 中心で再チューニング。
        # TC スイープ分析で TC=0.12 が steer_score 最良 (3.7567) と判明したため、
        # TC 範囲を [0.09, 0.15] に絞って他 steer パラメータを再最適化する。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.09, 0.15),  # Phase 4 [0.05, 0.25] から TC=0.12 中心に絞り込み
            "debug_steer_scaling_factor": (0.75, 1.20),
            "k_us":                       (0.0,  0.05),
            "k_us_vx_lo":                 (0.5,  6.0),
            "k_us_vx_hi":                 (1.0, 12.0),
            "steer_dead_band":            (0.0,  0.02),
            "steer_bias":                 (-0.01, 0.01),
        }
        score_fn = steer_score
        explore_delay = False
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 5] TC=0.12 中心再チューニング (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
        else:
            print(f"[Phase 5] TC=0.12 中心再チューニング (yaw+lat スコア)。acc 系は cur_best から固定。")
    elif phase == 10:
        # Phase 10: steer_time_delay を離散候補に追加した Phase 9 相当の再チューニング。
        # probe_steer_delay 結果: delay=0.099s で -0.155 (Phase9 比) の改善を確認。
        # steer_time_delay は _GT_KEYS に含まれるため離散候補で gt_cache を効率化。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.09, 0.16),
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.003, 0.010),
            "k_us_vx_lo":                 (2.0,  6.0),
            "k_us_vx_hi":                 (6.0, 13.0),
            "steer_dead_band":            (0.001, 0.005),
            "steer_bias":                 (-0.002, 0.003),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = True
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 10] steer_time_delay 付き再チューニング (yaw+lat スコア)。固定 acc params: {phase_fixed_params}")
        else:
            print(f"[Phase 10] steer_time_delay 付き再チューニング (yaw+lat スコア)。acc 系は cur_best から固定。")
    elif phase == 11:
        # Phase 11: Phase 10 best から探索空間の境界を拡張した再チューニング。
        # Phase 10 観察: steer_time_constant best=0.157 (上限 0.16 付近)、
        #                k_us_vx_lo best=2.127 (下限 2.0 付近) → 境界が制約。
        # steer_time_delay=0.066 は Phase 10 top 10 全試行で一致 → 固定。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.14, 0.30),   # 上限 0.16 → 0.30 に大幅拡張
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.003, 0.012),  # 上限 0.010 → 0.012 に微拡張
            "k_us_vx_lo":                 (1.0,  6.0),    # 下限 2.0 → 1.0 に拡張
            "k_us_vx_hi":                 (6.0, 13.0),
            "steer_dead_band":            (0.001, 0.005),
            "steer_bias":                 (-0.002, 0.003),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = False   # steer_time_delay=0.066 に固定 (phase_params から継承)
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 11] 境界拡張再チューニング (yaw+lat スコア)。固定 params: {phase_fixed_params} | steer_time_delay={cur_best.get('steer_time_delay', '未設定')}")
        else:
            print(f"[Phase 11] 境界拡張再チューニング (yaw+lat スコア)。steer_time_delay は cur_best から固定。")
    elif phase == 12:
        # Phase 12: acc_time_constant を変数化した再チューニング。
        # probe_p10_boundary_v2 結果 (2026-06-25):
        #   - acc_time_constant=0.107 (固定) → 0.18 で -0.048 改善余地あり
        #   - steer_time_constant 最適値は 0.165〜0.186 付近
        #   - Phase 11 best: stc=0.178, score=7.0763
        # Phase 10/11 では acc_time_constant を acc_time_delay と同じく固定していたが、
        # 実際には acc_time_constant が最大の改善ポテンシャルを持つ。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.14, 0.25),   # Phase 11 観察から最適域 0.17-0.19 付近
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.003, 0.012),
            "k_us_vx_lo":                 (1.0,  6.0),
            "k_us_vx_hi":                 (6.0, 13.0),
            "steer_dead_band":            (0.001, 0.005),
            "steer_bias":                 (-0.002, 0.003),
            "acc_time_constant":          (0.10, 0.25),   # Phase 10/11 の固定 0.107 を変数化
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = False   # steer_time_delay=0.066 に固定 (phase_params から継承)
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 12] acc_time_constant 変数化チューニング (yaw+lat スコア)。固定 params: {phase_fixed_params} | steer_time_delay={cur_best.get('steer_time_delay', '未設定')}")
        else:
            print(f"[Phase 12] acc_time_constant 変数化チューニング (yaw+lat スコア)。steer_time_delay は cur_best から固定。")
    elif phase == 13:
        # Phase 13: steer_rate_lim を追加した Phase 12 再チューニング。
        # steer_rate_lim 1D スキャン (2026-06-26): srl=0.40 で -0.5% 改善 (654 datasets)。
        # discriminating test (新規 3055): srl=0.40 で同方向改善 → 物理的に有効と判定。
        # tight bound (0.35-0.55 rad/s): 実機 p99=0.376 rad/s に基づく物理アンカー。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.14, 0.25),
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.003, 0.012),
            "k_us_vx_lo":                 (1.0,  6.0),
            "k_us_vx_hi":                 (6.0, 13.0),
            "steer_dead_band":            (0.001, 0.005),
            "steer_bias":                 (-0.002, 0.003),
            "acc_time_constant":          (0.10, 0.25),
            "steer_rate_lim":             (0.35, 0.55),   # 物理アンカー tight bound
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = False
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 13] steer_rate_lim joint チューニング (yaw+lat スコア)。固定 params: {phase_fixed_params} | steer_time_delay={cur_best.get('steer_time_delay', '未設定')}")
        else:
            print(f"[Phase 13] steer_rate_lim joint チューニング (yaw+lat スコア)。steer_time_delay は cur_best から固定。")
    elif phase == 14:
        # Phase 14: 速度帯依存 k_us ランプ (k_us_lo/k_us_vx_thresh) 追加チューニング。
        # NOTE: 元々「steer 角依存 k_us ランプ (k_us_steer_lo/hi)」として設計されたが、
        # C++ 実装の変更により k_us_lo (低速帯 k_us 定数) と k_us_vx_thresh (閾値速度) に変更。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.12, 0.22),
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.003, 0.020),
            "k_us_vx_lo":                 (1.0,  6.0),
            "k_us_vx_hi":                 (6.0, 13.0),
            "k_us_lo":                    (0.005, 0.025),   # 低速帯 (vx < thresh) の k_us 定数
            "k_us_vx_thresh":             (2.0,  6.0),      # 低速/高速切り替え閾値 [m/s]
            "steer_dead_band":            (0.001, 0.005),
            "steer_bias":                 (-0.002, 0.003),
            "acc_time_constant":          (0.10, 0.25),
            "steer_rate_lim":             (0.35, 0.55),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = False
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 14] 速度帯依存 k_us チューニング (yaw+lat スコア)。固定 params: {phase_fixed_params} | steer_time_delay={cur_best.get('steer_time_delay', '未設定')}")
        else:
            print(f"[Phase 14] 速度帯依存 k_us チューニング (yaw+lat スコア)。steer_time_delay は cur_best から固定。")
    elif phase == 43:
        # Phase 43: 速度帯別定数 k_us (k_us_lo/k_us_vx_thresh) の最適化。
        # Phase 42 best_params (k_us=0.0035, ランプなし) を起点に、低速帯 (vx < thresh) に
        # 別の k_us_lo を設定して steer_score を改善する。
        # スイープ結果 (2026-06-28): 低速帯最良 k_us=0.010, 中速/高速=0.0035。
        # per-band oracle: サブセット -1.01% 改善 (Phase 42 比 → 全体 -10% 到達推定)。
        # k_us_vx_lo/hi=0 に固定してランプを無効化し、定数切り替えのみ探索する。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.15, 0.25),
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.002, 0.007),  # 高速帯 (vx >= thresh) の k_us
            "k_us_lo":                    (0.005, 0.020),  # 低速帯 (vx < thresh) の k_us
            "k_us_vx_thresh":             (2.0,   6.0),   # 低速/高速切り替え閾値 [m/s]
            "steer_dead_band":            (0.001, 0.007),
            "steer_bias":                 (-0.003, 0.003),
            "acc_time_constant":          (0.12, 0.22),
            "steer_rate_lim":             (0.40, 0.65),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = False
        cur_best["k_us_vx_lo"] = 0.0   # ランプを無効化
        cur_best["k_us_vx_hi"] = 0.0
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 43] 速度帯別定数 k_us 最適化 (yaw+lat スコア)。固定 params: {phase_fixed_params} | steer_time_delay={cur_best.get('steer_time_delay', '未設定')}")
        else:
            print(f"[Phase 43] 速度帯別定数 k_us 最適化 (yaw+lat スコア)。steer_time_delay は cur_best から固定。")
    elif phase == 44:
        # Phase 44: 3分割速度帯定数 k_us (k_us_lo/k_us_mid/k_us + thresh1/thresh2) の最適化。
        # Phase 43 best_params を起点に、中速帯 (thresh1 <= vx < thresh2) を追加する。
        # k_us_lo/k_us_vx_thresh/k_us_mid/k_us_vx_thresh2 → step5 で k_us_bands/k_us_thresholds に変換。
        # k_us_vx_lo/hi=0 に固定してランプを無効化する。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.15, 0.25),
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.001, 0.007),  # 高速帯 (vx >= thresh2) の k_us
            "k_us_lo":                    (0.005, 0.025),  # 低速帯 (vx < thresh1) の k_us
            "k_us_mid":                   (0.002, 0.010),  # 中速帯 (thresh1 <= vx < thresh2) の k_us
            "k_us_vx_thresh":             (2.0,   5.0),   # 低速/中速切り替え閾値 [m/s]
            "k_us_vx_thresh2":            (5.0,  10.0),   # 中速/高速切り替え閾値 [m/s]
            "steer_dead_band":            (0.001, 0.007),
            "steer_bias":                 (-0.003, 0.003),
            "acc_time_constant":          (0.12, 0.22),
            "steer_rate_lim":             (0.40, 0.65),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = False
        cur_best["k_us_vx_lo"] = 0.0   # ランプを無効化
        cur_best["k_us_vx_hi"] = 0.0
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 44] 3分割速度帯 k_us 最適化 (yaw+lat スコア)。固定 params: {phase_fixed_params} | steer_time_delay={cur_best.get('steer_time_delay', '未設定')}")
        else:
            print(f"[Phase 44] 3分割速度帯 k_us 最適化 (yaw+lat スコア)。steer_time_delay は cur_best から固定。")
    elif phase == 45:
        # Phase 45: steer_dead_band=0 固定 + k_us 3分割 + steer_time_constant/delay 広域探索。
        # 背景: Phase 44 で k_us が OLS 推定値 (~0.015) より大幅に低くなる原因として
        # steer_dead_band との交絡を特定。dead_band=0 で固定し、k_us が本来の物理値に
        # 近づくかを検証する。
        # steer_time_constant と steer_time_delay は identify_steer_dynamics.py による
        # 直接同定値 (τ_med, T_med) を参考に探索範囲を設定する。
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.50),   # 同定結果に応じて範囲設定
            "debug_steer_scaling_factor": (0.92, 1.02),
            "k_us":                       (0.001, 0.025),  # 高速帯 (vx >= thresh2)
            "k_us_lo":                    (0.005, 0.030),  # 低速帯 (vx < thresh1)
            "k_us_mid":                   (0.002, 0.020),  # 中速帯 (thresh1 <= vx < thresh2)
            "k_us_vx_thresh":             (2.0,   5.0),
            "k_us_vx_thresh2":            (5.0,  10.0),
            "steer_bias":                 (-0.003, 0.003),
            "acc_time_constant":          (0.12, 0.22),
            "steer_rate_lim":             (0.40, 0.65),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = True   # steer_time_delay も探索
        cur_best["k_us_vx_lo"] = 0.0
        cur_best["k_us_vx_hi"] = 0.0
        cur_best["steer_dead_band"] = 0.0   # dead_band=0 固定
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
            print(f"[Phase 45] dead_band=0 固定 + k_us 3分割 + steer delay 探索。固定 params: {phase_fixed_params}")
        else:
            print("[Phase 45] steer_dead_band=0 固定 + k_us 3分割 + steer_time_constant/delay 広域探索。")
    elif phase == 46:
        # Phase 46: k_us を最小二乗法実測値から固定し、ステア動力学パラメータのみ最適化。
        # 背景: 診断により open-loop rollout スコアは δ_cmd バイアスで k_us を低く引き寄せる
        # 構造的問題が確認された。最小二乗法（δ_actual 使用）で得られた物理値を固定し、
        # steer 動力学系（time_constant / delay / rate_lim 等）のみを探索する。
        #
        # k_us プロファイル（最小二乗法 重み付き平均から設定）:
        #   k_us_lo  = 0.000 （< 2.71 m/s: 低速は物理的に understeer なし）
        #   k_us_mid = 0.020 （2.71-5.91 m/s: 最小二乗法 重み付き平均）
        #   k_us     = 0.016 （≥ 5.91 m/s: 最小二乗法 高速ビン重み付き平均）
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.50),
            "debug_steer_scaling_factor": (0.90, 1.05),
            "steer_bias":                 (-0.005, 0.005),
            "acc_time_constant":          (0.10, 0.30),
            "steer_rate_lim":             (0.30, 0.80),
            "steer_dead_band":            (0.000, 0.010),
        }
        score_fn = steer_score
        explore_delay = False
        explore_steer_delay = True
        # k_us を OLS 実測値で上書き（phase_fixed_params より後に適用して確実に固定）
        _KUS_OLS_OVERRIDES = {
            "k_us_lo":         0.000,
            "k_us_mid":        0.020,
            "k_us":            0.016,
            "k_us_vx_thresh":  2.71,
            "k_us_vx_thresh2": 5.91,
            "k_us_vx_lo":      0.0,
            "k_us_vx_hi":      0.0,
        }
        if phase_fixed_params:
            cur_best.update(phase_fixed_params)
        cur_best.update(_KUS_OLS_OVERRIDES)  # 最小二乗法の値で上書き（固定）
        print(
            f"[Phase 46] k_us を最小二乗法で固定 (lo={_KUS_OLS_OVERRIDES['k_us_lo']:.3f}/"
            f"mid={_KUS_OLS_OVERRIDES['k_us_mid']:.3f}/hi={_KUS_OLS_OVERRIDES['k_us']:.3f})"
            f" + steer 動力学最適化。"
        )
    else:
        # Phase 0: 全パラメータ同時最適化 (従来の robust_score)
        CONTINUOUS_SPACE = {
            "steer_time_constant":        (0.05, 0.80),
            "debug_steer_scaling_factor": (0.75, 1.20),  # (0.80, 1.05) → (0.75, 1.20) に拡張
            "acc_time_constant":          (0.05, 1.20),  # 下限を 0.10→0.05 に拡張
            "k_us":                       (0.0,  0.05),
            "k_us_vx_lo":                 (0.5,  6.0),
            "k_us_vx_hi":                 (1.0, 12.0),
            "steer_dead_band":            (0.0,  0.02),   # アクチュエータ不感帯 [rad]
            "steer_bias":                 (-0.01, 0.01),  # 系統的ステアオフセット [rad]
        }
        score_fn = robust_score
        explore_delay = True
        explore_steer_delay = False

    # explore_steer_delay は Phase 10 のみ True (他 phase では未設定の場合を考慮)
    if "explore_steer_delay" not in locals():
        explore_steer_delay = False

    # gt 事前計算 (fork 前に親で完了し COW 共有)
    acc_delay_set: set[float] = set(DELAY_CANDIDATES) if explore_delay else set()
    if "acc_time_delay" in cur_best:
        acc_delay_set.add(float(cur_best["acc_time_delay"]))
    steer_delay_set: set[float] = set(STEER_DELAY_CANDIDATES) if explore_steer_delay else set()
    if "steer_time_delay" in cur_best:
        steer_delay_set.add(float(cur_best["steer_time_delay"]))
    for ctx in ctxs:
        for ad in (acc_delay_set or {float(cur_best.get("acc_time_delay", 0.1))}):
            for sd in (steer_delay_set or {float(cur_best.get("steer_time_delay", 0.0315))}):
                merged = dict(ctx.base)
                merged.update(cur_best)
                merged["acc_time_delay"] = ad
                merged["steer_time_delay"] = sd
                key = tuple(round(float(merged[k]), 9) for k in _GT_KEYS)
                if key not in ctx.gt_cache:
                    ctx.gt_cache[key] = s5._prepare_gt(ctx.data, ctx.t0_ns, merged)

    # プール生成 (gt 事前計算完了後にフォーク → COW 共有確定)
    pool = None
    if n_jobs > 1:
        mp_ctx = multiprocessing.get_context("fork")
        pool = mp_ctx.Pool(n_jobs)
        print(f"[INFO] 並列実行: {n_jobs} workers (fork)")

    try:
        # 初期スコア表示
        init_agg = _eval_grid(pool, ctxs_search, [cur_best], cur_model, n_jobs)[0]
        init_score = score_fn(init_agg, worst_w=worst_w)
        phase_label = f"phase={phase}" if phase else "phase=0(all)"
        print(f"\n## Optuna TPE ({case_name}, {n_trials} trials, cross-dataset normalized, worst_w={worst_w}, {phase_label})")
        print(format_agg("init", init_agg) + f"  score={init_score:.4f}  {cur_best}")

        best_result: dict = {"params": dict(cur_best), "score": init_score, "agg": init_agg}
        _checkpoint(cur_best, init_score)

        def objective(trial: optuna.Trial) -> float:
            params = dict(cur_best)  # 固定パラメータ (n_substep 等) を継承
            for pname, (lo, hi) in CONTINUOUS_SPACE.items():
                params[pname] = trial.suggest_float(pname, lo, hi)
            if explore_delay:
                params["acc_time_delay"] = trial.suggest_categorical(
                    "acc_time_delay", DELAY_CANDIDATES
                )
            if explore_steer_delay:
                params["steer_time_delay"] = trial.suggest_categorical(
                    "steer_time_delay", STEER_DELAY_CANDIDATES
                )

            agg = _eval_grid(pool, ctxs_search, [params], cur_model, n_jobs)[0]
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

        optuna.logging.set_verbosity(optuna.logging.WARNING)
        sampler = optuna.samplers.TPESampler(multivariate=True, seed=42)
        study = optuna.create_study(direction="minimize", sampler=sampler)

        # warm start: trial 0+ に既知良点を投入
        delay_list = list(DELAY_CANDIDATES)
        steer_delay_list = list(STEER_DELAY_CANDIDATES)

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
            if explore_steer_delay:
                if "steer_time_delay" in params:
                    v = float(params["steer_time_delay"])
                    eq["steer_time_delay"] = min(steer_delay_list, key=lambda x: abs(x - v))
                else:
                    eq["steer_time_delay"] = steer_delay_list[0]
            return eq

        study.enqueue_trial(_make_enqueue(cur_best))
        for ep in (extra_enqueue or []):
            study.enqueue_trial(_make_enqueue(ep))

        study.optimize(objective, n_trials=n_trials, callbacks=[_log_cb])

        state = best_result["params"]
        best_s = best_result["score"]
        print(format_agg("FINAL", best_result["agg"]) + f"  score={best_s:.4f}")
        print(f"FINAL params: {state}")
        _checkpoint(state, best_s)

        # search_subsample 指定時: 最終 params の score を全件で再評価
        if search_subsample and len(ctxs_search) < len(ctxs):
            print(f"[INFO] 最終 score を全 {len(ctxs)} データセットで評価中...")
            full_agg = _eval_grid(pool, ctxs, [state], cur_model, n_jobs)[0]
            best_s = score_fn(full_agg, worst_w=worst_w)
            best_result.update({"score": best_s, "agg": full_agg})
            print(format_agg("FINAL(全件)", full_agg) + f"  score={best_s:.4f}")

        return {"params": state, "agg": best_result["agg"], "score": best_s}
    finally:
        if pool is not None:
            pool.close()
            pool.join()


def _discover(collection_dir: Path) -> list[tuple[str, Path]]:
    """収集ディレクトリ配下の <dataset_id>/real.lite を列挙 (datasets/ サブディレクトリ対応)。"""
    from .lib._collection import datasets_root  # noqa: PLC0415

    out = []
    for sub in sorted(datasets_root(collection_dir).iterdir()):
        if sub.is_dir() and resolve_lite_bag(sub, "real") is not None:
            out.append((sub.name, sub))
    return out


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
        description="マルチデータセット横断のロバスト best_normal 同定 (robust_search)"
    )
    ap.add_argument("--collection-dir", default=str(Path(__file__).parent / "sample" / "multi"))
    ap.add_argument(
        "--case",
        default="best_normal",
        metavar="CASE_NAME",
        help="チューニング対象の scenario.yaml models エントリ名 (既定: best_normal)",
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
        default=os.cpu_count(),
        metavar="N",
        help="並列ワーカー数 (既定: CPU コア数)。1 で逐次実行 (デバッグ・再現性検証用)",
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
        choices=[0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 43, 44, 45, 46],
        help=(
            "チューニングフェーズ (既定: 0=全パラメータ同時最適化)。"
            "1=acc のみ探索・long スコア (steer 系は scenario.yaml の定義値に固定)。"
            "2=steer のみ探索・yaw+lat スコア (acc 系は scenario.yaml 定義値または --phase-params に固定)。"
            "3=no-ramp k_us 実験・yaw+lat スコア (k_us をランプなし定数として探索、acc 系は固定)。"
            "4=TC 絞り込み再チューニング・yaw+lat スコア (TC を [0.05, 0.25] に絞り steer パラメータを再探索)。"
            "5=TC=0.12 中心再チューニング・yaw+lat スコア (TC を [0.09, 0.15] に絞り steer パラメータを再探索)。"
            "10=steer_time_delay を離散候補に追加した Phase 9 相当の再チューニング (yaw+lat スコア)。"
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
    print(f"datasets: {[d for d, _ in lite_dirs]}")

    n_jobs = max(1, args.jobs or 1)

    # OMP スレッド数を抑制してから全 fork (load_datasets / robust_search 両方をカバー):
    # worker 内の BLAS/OpenMP が親コア数分のスレッドを再起動するのを防ぎ、
    # コア間の CPU リソース競合を回避する。load_datasets の前に設定しないと
    # 並列ロード中の baseline rollout が oversubscribe する。
    os.environ.setdefault("OMP_NUM_THREADS", "1")

    ctxs = load_datasets(lite_dirs, n_jobs=n_jobs, verbose=args.verbose)
    if len(ctxs) < 1:
        print("ERROR: 有効な dataset が 0 件", file=sys.stderr)
        sys.exit(1)

    cfg = load_cases_config(args.scenario)

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
    if args.phase_params:
        p = Path(args.phase_params)
        with p.open("r") as f:
            phase_data = yaml.safe_load(f)
        all_params = phase_data.get("params", phase_data)
        acc_keys = {"acc_time_constant", "acc_time_delay"}
        # Phase 43/44/45: 前フェーズの全 params を cur_best として継承（探索空間外の値を保持）
        if args.phase in (43, 44, 45, 46):
            fixed_keys = set(all_params.keys())
        # Phase 11/12/13: steer_time_delay も固定値として引き継ぐ
        # Phase 12/13: acc_time_constant は変数化するため固定しない (acc_time_delay のみ固定)
        elif args.phase in (12, 13):
            fixed_keys = {"acc_time_delay", "steer_time_delay"}
        elif args.phase == 11:
            fixed_keys = acc_keys | {"steer_time_delay"}
        elif args.phase in (2, 3, 4, 5, 10):
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
            model_type=_BASELINE_MODEL,
            eval_fn=_eval,
            stride=STRIDE,
            worst_w=args.report_worst_weight,
        )


if __name__ == "__main__":
    main()
