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
from dataclasses import dataclass
import itertools
import multiprocessing
import os
from pathlib import Path
import sys

from . import step5_analyze_nstep as s5
from .lib._cases_config import load_cases_config
from .lib._io import resolve_lite_bag
from .lib._multi_agg import HORIZONS, aggregate_normalized, format_agg, robust_score
from .lib._nstep_common import rmse_by_horizon

STRIDE = 5
WHEELBASE = 4.76012
_BASELINE_MODEL = "delay_steer_acc_geared_wo_fall_guard"
# _prepare_gt は params の delay/wheelbase/sub_dt にのみ依存 (run_rollout docstring)
_GT_KEYS = ("acc_time_delay", "steer_time_delay", "wheelbase", "sub_dt")


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
    df = s5.run_rollout(
        ctx.data, ctx.t0_ns, params, model_type, horizons=HORIZONS, stride=STRIDE, gt=gt
    )
    rmse = rmse_by_horizon(df)
    return {h: rmse[h] for h in HORIZONS}


def load_datasets(lite_dirs: list[tuple[str, Path]]) -> list[DatasetCtx]:
    """収集された real.lite を読み込み DatasetCtx を構築する (baseline 誤差も計算)。"""
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
            print(f"[SKIP] {ds_id}: ロード失敗 ({type(e).__name__}: {e})", file=sys.stderr)
            continue
        ctxs.append(ctx)
        print(
            f"[load] {ds_id}: "
            + "  ".join(
                f"baseline@N{h} yaw={ctx.base_metric[h]['yaw']:.4f} "
                f"縦={ctx.base_metric[h]['long']:.3f} 横={ctx.base_metric[h]['lat']:.3f}"
                for h in HORIZONS
            )
        )
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
# 並列評価サポート (fork プールによる trial 単位並列化)
# ---------------------------------------------------------------------------

# worker が参照するグローバル ctxs。fork 前に親がセットし、子は COW で読み取り専用で継承する。
# これにより ctxs の pickle 転送ゼロ・メモリ共有を実現する。
_CTXS: list[DatasetCtx] = []


def _worker_aggregate(args: tuple[dict, str]) -> dict:
    """プールワーカー: グローバル _CTXS を使って aggregate を呼ぶ (fork COW 継承)。"""
    override, model_type = args
    return aggregate(_CTXS, override, model_type)


def _eval_grid(
    pool,
    ctxs: list[DatasetCtx],
    trials: list[dict],
    model_type: str,
) -> list[dict]:
    """trials を並列 (pool 非 None) または逐次で aggregate 評価し元順の agg リストを返す。

    pool が None のとき逐次実行して ctxs を直接使う (--jobs 1 での完全逐次互換)。
    pool が非 None のとき fork ワーカーがグローバル _CTXS を使って評価する。
    いずれも返り値は trials と同順の agg リスト (tie-break の決定論的再現を保証)。
    """
    if pool is None:
        return [aggregate(ctxs, t, model_type) for t in trials]
    return pool.map(_worker_aggregate, [(t, model_type) for t in trials])


_KUS0020 = {"k_us": 0.020}


def robust_search(ctxs: list[DatasetCtx], cfg, *, n_jobs: int = 1) -> dict:
    """
    best_normal 集合 (代理含む) を coordinate descent で dataset 横断ロバスト最適化。

    目的: 全 horizon の正規化 mean + worst (yaw+縦+横) を最小化
    (lib._multi_agg.robust_score; worst を重み付きで含む)。
    正規化 baseline は無補正 delay モデル (_BASELINE_MODEL) の rollout = Conditions.overlay.reference_tag
    ケースと同定義で、step13_cross_dataset の正規化と一致する。
    参照点 (現 best_normal) は scenario.yaml の best_normal ケースから取得する
    (ハードコードしない。yaml を更新後に再探索しても整合する)。

    n_jobs: 並列ワーカー数 (1 で逐次実行)。プールは gt 事前計算が完了した後にフォークする
    ことで、COW 継承により worker が gt_cache を再計算なしに利用できる。
    """
    cur_case = cfg.find_case("best_normal")
    cur_best = dict(cur_case.params)
    cur_model = cur_case.vehicle_model_type
    sweeps = {
        "k_us": [0.0, 0.005, 0.010, 0.012, 0.015, 0.018, 0.020, 0.025, 0.030],
        "steer_time_constant": [0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.70],
        "debug_steer_scaling_factor": [0.85, 0.88, 0.90, 0.93, 0.96, 0.98, 1.0],
        "acc_time_constant": [0.30, 0.35, 0.45, 0.50, 0.60, 0.70, 0.80, 1.0],
        "acc_time_delay": [0.10, 0.15, 0.20, 0.30, 0.40, 0.50],
    }

    # --- gt 事前計算 (fork 前に親で完了し COW 共有) ---
    # _GT_KEYS のうち探索中に変化するのは acc_time_delay のみ (steer_time_delay・wheelbase・
    # sub_dt は全 run で定数)。cur_best 由来の値と sweeps["acc_time_delay"] を網羅することで
    # worker での _prepare_gt 再計算をゼロにし、gt_cache の COW ページが shared のまま維持される。
    acc_delay_candidates: set = set(sweeps["acc_time_delay"])
    base_delay = cur_best.get("acc_time_delay")
    if base_delay is not None:
        acc_delay_candidates.add(float(base_delay))
    for ctx in ctxs:
        for v in acc_delay_candidates:
            merged = dict(ctx.base)
            merged.update(cur_best)
            merged["acc_time_delay"] = v
            key = tuple(round(float(merged[k]), 9) for k in _GT_KEYS)
            if key not in ctx.gt_cache:
                ctx.gt_cache[key] = s5._prepare_gt(ctx.data, ctx.t0_ns, merged)

    # --- プール生成 (gt 事前計算の完了後にフォークして COW 共有を確定) ---
    pool = None
    if n_jobs > 1:
        mp_ctx = multiprocessing.get_context("fork")
        pool = mp_ctx.Pool(n_jobs)
        print(f"[INFO] 並列実行: {n_jobs} workers (fork)")

    try:
        cur_agg = aggregate(ctxs, cur_best, cur_model)
        print("\n## robust coordinate descent (best_normal family, cross-dataset normalized)")
        print(format_agg("cur_best", cur_agg) + f"  score={robust_score(cur_agg):.4f}  {cur_best}")
        print(format_agg("kus0020", aggregate(ctxs, _KUS0020, cur_model)))

        # worst は robust_score に組み込み済み (ハード guard なし) なので mean↔worst の trade を許容する。
        state = dict(cur_best)
        best_agg = cur_agg
        best_s = robust_score(cur_agg)
        for _pass in range(3):
            improved = False
            for pname, grid in sweeps.items():
                # trials を一括並列評価し、元の反復順でリダクションを replay する。
                # grid 内では pname 以外の state フィールドは変化しないため並列化が安全:
                # 仮に前の v で state[pname] が更新されても、次の v では state[pname]=v で上書き
                # されるため他フィールドの値は同一 (= 元コードの trial = {**state, pname: v} と等価)。
                trials = [dict(state) for _ in grid]
                for t, v in zip(trials, grid):
                    t[pname] = v
                aggs = _eval_grid(pool, ctxs, trials, cur_model)
                for v, agg in zip(grid, aggs):
                    s = robust_score(agg)
                    if s < best_s - 1e-6:
                        new_state = dict(state)
                        new_state[pname] = v
                        best_s, best_agg, state, improved = s, agg, new_state, True
            print(format_agg(f"pass{_pass}", best_agg) + f"  score={best_s:.4f}  {state}")
            if not improved:
                break
        # --- 直積グリッド精密化 (coordinate descent の経路依存を排除) ---
        # descent 最適の近傍を主要 4 パラメータで直積評価し真の結合最小を確定する。
        # acc/steer の遅延を spec 固定にすることで gt がデータセット毎 1 回キャッシュされ高速。
        print("\n## product-grid refinement around descent optimum")
        refine = {
            "k_us": sorted({0.015, 0.018, 0.020, 0.025}),
            "steer_time_constant": sorted({0.12, 0.15, 0.20, 0.30}),
            "debug_steer_scaling_factor": sorted({0.93, 0.96, 0.98, 1.0}),
            "acc_time_constant": sorted({0.10, 0.20, 0.30, 0.45, 0.60}),
        }
        keys = list(refine)
        combos = list(itertools.product(*(refine[k] for k in keys)))
        refine_trials = []
        for combo in combos:
            t = dict(cur_best)  # descent 外の spec パラメータを保持
            t.update({k: v for k, v in zip(keys, combo)})
            refine_trials.append(t)
        aggs = _eval_grid(pool, ctxs, refine_trials, cur_model)
        for i, (combo, agg) in enumerate(zip(combos, aggs)):  # noqa: B007
            s = robust_score(agg)
            if s < best_s - 1e-6:
                best_s, best_agg, state = s, agg, refine_trials[i]
        print(format_agg("FINAL", best_agg) + f"  score={best_s:.4f}")
        print(f"FINAL params: {state}")
        return {"params": state, "agg": best_agg, "score": best_s}
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


def main() -> None:
    ap = argparse.ArgumentParser(
        description="マルチデータセット横断のロバスト best_normal 同定 (robust_search)"
    )
    ap.add_argument("--collection-dir", default=str(Path(__file__).parent / "sample" / "multi"))
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
    args = ap.parse_args()

    if args.lite_dir:
        lite_dirs = []
        for spec in args.lite_dir:
            ds_id, raw = spec.split("=", 1)
            lite_dirs.append((ds_id.strip(), Path(raw.strip())))
    else:
        lite_dirs = _discover(Path(args.collection_dir))
    if not lite_dirs:
        print(f"ERROR: real.lite が見つかりません: {args.collection_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"datasets: {[d for d, _ in lite_dirs]}")

    ctxs = load_datasets(lite_dirs)
    if len(ctxs) < 1:
        print("ERROR: 有効な dataset が 0 件", file=sys.stderr)
        sys.exit(1)

    cfg = load_cases_config(args.scenario)

    # OMP スレッド数を抑制してからフォーク:
    # worker 内の BLAS/OpenMP が親コア数分のスレッドを再起動するのを防ぎ、
    # コア間の CPU リソース競合を回避する。
    os.environ.setdefault("OMP_NUM_THREADS", "1")

    # グローバル _CTXS に ctxs をセット (fork COW 継承パターン)。
    # pool.map は pickle 転送せず fork した子プロセスがグローバルを直接参照する。
    global _CTXS  # noqa: PLW0603
    _CTXS = ctxs

    n_jobs = max(1, args.jobs or 1)
    result = robust_search(ctxs, cfg, n_jobs=n_jobs)

    if args.out:
        import yaml as _yaml  # noqa: PLC0415
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            _yaml.safe_dump(
                {"params": result["params"], "score": result["score"]},
                allow_unicode=True,
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        print(f"[INFO] FINAL params 保存: {out_path}")


if __name__ == "__main__":
    main()
