"""採用ゲートの成文化: セル別非劣化判定 (v2 採用時の運用を自動化)。

ゲート定義 (v2 の k_us スイープで用いた運用の一般化):
  全セル (統計量 mean / cvar@90 × horizon × 指標 pos/long/lat/yaw/steer/vx/ax) について
  candidate の per-dataset RMSE 集約値が reference 比 +threshold% 以内なら PASS。
  max (単一最悪 dataset) は参考表示のみ (外れ 1 件に支配されるため判定には使わない)。

2 つのモード:
  metrics — 既存 metrics.csv から判定 (リリース級、dense horizon も可、追加 rollout なし)
  screen  — scenario ケース + パラメータ上書きを直接 rollout 評価して判定
            (score horizon のみの速報スクリーニング。C++ 変更前の候補選別用)
"""
from __future__ import annotations

import argparse
import multiprocessing
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from ..lib._multi_agg import cvar_worst, robust_score, aggregate_normalized
from ..lib._nstep_common import METRIC_KEYS
from ..lib._parallel import (
    imap_with_watchdog,
    normalize_parallel_jobs,
    pool_chunksize,
    set_worker_thread_env_defaults,
)
from . import fit_merge
from .load_data import discover_cached_datasets
from .model_config import load_model_config, resolve_baseline_model
from .settings import ACT_SCORE_HORIZONS, HORIZONS

DEFAULT_THRESHOLD_PCT = 2.0
# 判定に使う統計量。max は表示のみ。
GATE_STATS: tuple[str, ...] = ("mean", "cvar")


def build_cells(
    per_dataset: dict[str, dict[int, dict[str, float]]],
    horizons: tuple[int, ...],
) -> pd.DataFrame:
    """per-dataset RMSE {ds: {h: {metric: value}}} をセル表 (horizon×metric×統計量) に集約する。"""
    rows = []
    for h in horizons:
        for metric in METRIC_KEYS:
            values = [per_dataset[ds][h][metric] for ds in per_dataset]
            rows.append({
                "horizon": h,
                "metric": metric,
                "mean": float(np.mean(values)),
                "cvar": float(cvar_worst(list(map(float, values)))),
                "max": float(np.max(values)),
            })
    return pd.DataFrame(rows)


def judge_cells(
    candidate_cells: pd.DataFrame,
    reference_cells: pd.DataFrame,
    *,
    threshold_pct: float = DEFAULT_THRESHOLD_PCT,
) -> tuple[pd.DataFrame, bool]:
    """セル別 %差分表と PASS/FAIL を返す。判定は GATE_STATS のみ、max は参考。"""
    merged = candidate_cells.merge(
        reference_cells, on=("horizon", "metric"), suffixes=("_cand", "_ref"),
    )
    for stat in (*GATE_STATS, "max"):
        merged[f"{stat}_diff_pct"] = (
            (merged[f"{stat}_cand"] - merged[f"{stat}_ref"]) / merged[f"{stat}_ref"] * 100.0
        )
    merged["gate_ok"] = np.all(
        [merged[f"{stat}_diff_pct"] <= threshold_pct for stat in GATE_STATS], axis=0,
    )
    return merged, bool(merged["gate_ok"].all())


def _print_judgement(cells: pd.DataFrame, ok: bool, *, threshold_pct: float, label: str) -> None:
    n_fail = int((~cells["gate_ok"]).sum())
    print(f"[gate] {label}: {'PASS' if ok else 'FAIL'} "
          f"(セル {len(cells)} 件中 {n_fail} 件が +{threshold_pct}% 超過)")
    offenders = cells[~cells["gate_ok"]].copy()
    if not offenders.empty:
        offenders["worst_stat_diff"] = offenders[[f"{s}_diff_pct" for s in GATE_STATS]].max(axis=1)
        offenders = offenders.sort_values("worst_stat_diff", ascending=False)
        for _, row in offenders.head(15).iterrows():
            print(
                f"[gate]   N={int(row['horizon'])} {row['metric']}: "
                + " ".join(f"{s}={row[f'{s}_diff_pct']:+.1f}%" for s in GATE_STATS)
                + f" max={row['max_diff_pct']:+.1f}%"
            )
    improved = cells[[f"{s}_diff_pct" for s in GATE_STATS]].mean().mean()
    print(f"[gate] 全セル平均変化: {improved:+.2f}%")


def _split_filter(dataset_ids: list[str], splits: tuple[str, ...] | None) -> set[str]:
    if not splits:
        return set(dataset_ids)
    from .analyze.split import split_of

    return {ds for ds in dataset_ids if split_of(ds) in splits}


# ---- metrics モード ------------------------------------------------------


def judge_from_metrics(
    metrics_csv: Path,
    *,
    candidate: str,
    reference: str,
    horizons: tuple[int, ...] | None = None,
    threshold_pct: float = DEFAULT_THRESHOLD_PCT,
    splits: tuple[str, ...] | None = None,
) -> tuple[pd.DataFrame, bool]:
    """metrics.csv からセル判定する。horizons=None で csv にある全 horizon (dense)。"""
    metrics = pd.read_csv(metrics_csv)
    models = set(metrics["model"])
    for m in (candidate, reference):
        if m not in models:
            raise ValueError(f"metrics.csv に model={m!r} がありません: {sorted(models)}")

    keep = _split_filter(sorted(set(metrics["dataset_id"])), splits)
    metrics = metrics[metrics["dataset_id"].isin(keep)]

    if horizons is None:
        horizons = tuple(sorted(set(metrics["horizon"])))

    def _per_dataset(model: str) -> dict:
        sub = metrics[(metrics["model"] == model) & (metrics["horizon"].isin(horizons))]
        out: dict[str, dict[int, dict[str, float]]] = {}
        for (ds, h), row in sub.set_index(["dataset_id", "horizon"]).iterrows():
            out.setdefault(ds, {})[int(h)] = {k: float(row[k]) for k in METRIC_KEYS}
        return out

    cand, ref = _per_dataset(candidate), _per_dataset(reference)
    common = sorted(set(cand) & set(ref))
    complete = [
        ds for ds in common
        if set(cand[ds]) >= set(horizons) and set(ref[ds]) >= set(horizons)
    ]
    if not complete:
        raise RuntimeError("両モデルの全 horizon が揃う dataset がありません")
    cand = {ds: cand[ds] for ds in complete}
    ref = {ds: ref[ds] for ds in complete}
    print(f"[gate] datasets={len(complete)} horizons={len(horizons)} "
          f"candidate={candidate} reference={reference}")

    cells, ok = judge_cells(
        build_cells(cand, tuple(horizons)),
        build_cells(ref, tuple(horizons)),
        threshold_pct=threshold_pct,
    )
    _print_judgement(cells, ok, threshold_pct=threshold_pct, label=f"{candidate} vs {reference}")
    return cells, ok


# ---- screen モード -------------------------------------------------------

_CTXS: list = []
_MODEL_TYPE = ""
_ACCEL_SOURCE = ""
_STEER_SOURCE = "steer"
_SLOPE_SOURCE = "none"
_PARAMS: dict = {}


def _screen_eval_one(idx: int) -> tuple[str, dict] | None:
    ctx = _CTXS[idx]
    try:
        metric = fit_merge._eval(
            ctx, _PARAMS, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE, _SLOPE_SOURCE,
            horizons=HORIZONS,
        )
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] screen 評価失敗 ({ctx.dataset_id}): {e}", file=sys.stderr)
        return None
    if not fit_merge._rollout_metric_is_finite(metric, HORIZONS):
        return None
    return ctx.dataset_id, metric


def _screen_eval_all(params: dict, n_jobs: int, slope_source: str) -> dict[str, dict]:
    global _PARAMS, _SLOPE_SOURCE  # noqa: PLW0603
    _PARAMS = dict(params)
    _SLOPE_SOURCE = slope_source
    indices = list(range(len(_CTXS)))
    n_workers = normalize_parallel_jobs(n_jobs, n_tasks=len(indices))
    if n_workers > 1:
        pool = multiprocessing.get_context("fork").Pool(n_workers)
        try:
            results = imap_with_watchdog(
                pool, _screen_eval_one, indices,
                chunksize=pool_chunksize(len(indices), n_workers),
            )
        finally:
            pool.close()
            pool.join()
    else:
        results = [_screen_eval_one(i) for i in indices]
    return {ds: metric for r in results if r is not None for ds, metric in [r]}


def screen_params(
    collection_dir: Path,
    scenario: Path,
    *,
    case_name: str,
    overrides: dict[str, float],
    slope_source: str | None = None,
    n_jobs: int = 1,
    threshold_pct: float = DEFAULT_THRESHOLD_PCT,
    splits: tuple[str, ...] | None = None,
) -> tuple[pd.DataFrame, bool]:
    """case パラメータ + overrides を直接評価し、case 自身 (override なし) と比較判定する。

    slope_source を与えると candidate 側のみその給電で評価する (reference は case の設定)。
    """
    global _CTXS, _MODEL_TYPE, _ACCEL_SOURCE, _STEER_SOURCE  # noqa: PLW0603

    tasks = discover_cached_datasets(collection_dir)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {collection_dir}")

    cfg = load_model_config(scenario)
    baseline_model_type, baseline_params, baseline_case = resolve_baseline_model(cfg)
    case = cfg.find_case(case_name)
    unknown = sorted(set(overrides) - set(case.params))
    if unknown:
        print(f"[gate] 注意: case '{case_name}' に無いキーを上書き: {unknown}", file=sys.stderr)

    set_worker_thread_env_defaults()
    ctxs, _skipped = fit_merge.load_datasets(
        tasks,
        n_jobs=n_jobs,
        baseline_model_type=baseline_model_type,
        baseline_params=baseline_params,
        baseline_acceleration_source=cfg.models[baseline_case].acceleration_source,
        baseline_steering_source=cfg.models[baseline_case].steering_source,
    )
    if splits:
        keep = _split_filter([ctx.dataset_id for ctx in ctxs], splits)
        ctxs = [ctx for ctx in ctxs if ctx.dataset_id in keep]
    if not ctxs:
        raise RuntimeError("有効な dataset が 0 件です")

    _CTXS = ctxs
    _MODEL_TYPE = case.vehicle_model_type
    _ACCEL_SOURCE = case.acceleration_source
    _STEER_SOURCE = case.steering_source

    ref_params = dict(case.params)
    cand_params = {**ref_params, **overrides}
    case_slope = getattr(case, "slope_source", "none")
    cand_slope = case_slope if slope_source is None else slope_source
    print(
        f"[gate] screen: case={case_name} overrides={overrides} "
        f"slope_source={cand_slope} (ref={case_slope}) datasets={len(ctxs)}"
    )

    ref_pd = _screen_eval_all(ref_params, n_jobs, case_slope)
    cand_pd = _screen_eval_all(cand_params, n_jobs, cand_slope)
    common = sorted(set(ref_pd) & set(cand_pd))
    if not common:
        raise RuntimeError("両候補で有効な dataset がありません")
    ref_pd = {ds: ref_pd[ds] for ds in common}
    cand_pd = {ds: cand_pd[ds] for ds in common}

    cells, ok = judge_cells(
        build_cells(cand_pd, HORIZONS),
        build_cells(ref_pd, HORIZONS),
        threshold_pct=threshold_pct,
    )
    label = f"{case_name}+{overrides}"
    if cand_slope != case_slope:
        label += f"+slope:{cand_slope}"
    label += f" vs {case_name}"
    _print_judgement(cells, ok, threshold_pct=threshold_pct, label=label)

    # objective v3 の robust_score 比較 (baseline 正規化はロード時の base_metric)。
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs if ctx.dataset_id in set(common)}
    def _score(per_dataset: dict[str, dict]) -> float:
        agg = aggregate_normalized(
            [(ds, per_dataset[ds]) for ds in common], baselines, HORIZONS,
        )
        return robust_score(agg, HORIZONS, act_horizons=ACT_SCORE_HORIZONS, worst_stat="cvar")

    score_ref, score_cand = _score(ref_pd), _score(cand_pd)
    print(f"[gate] robust_score(v3): {score_ref:.4f} -> {score_cand:.4f} "
          f"({(score_cand / score_ref - 1) * 100:+.2f}%)")
    return cells, ok


# ---- CLI -----------------------------------------------------------------


def _parse_overrides(items: list[str]) -> dict[str, float]:
    out: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise argparse.ArgumentTypeError(f"--set は key=value 形式: {item!r}")
        key, value = item.split("=", 1)
        out[key.strip()] = float(value)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="v3 採用ゲート (セル別非劣化判定)")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_m = sub.add_parser("metrics", help="metrics.csv から判定 (リリース級)")
    p_m.add_argument("--metrics", required=True, type=Path)
    p_m.add_argument("--candidate", required=True)
    p_m.add_argument("--reference", default="v2")
    p_m.add_argument("--score-horizons", action="store_true",
                     help="dense でなく score horizon (10/30/70/150/300) のみで判定")
    p_m.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD_PCT)
    p_m.add_argument("--splits", default=None, help="dev/holdout のカンマ区切り (省略時 全件)")
    p_m.add_argument("--out", type=Path, default=None, help="セル表 CSV の出力先")

    p_s = sub.add_parser("screen", help="パラメータ上書き候補の速報スクリーニング")
    p_s.add_argument("--root", required=True, type=Path)
    p_s.add_argument("--scenario", required=True, type=Path)
    p_s.add_argument("--case", default="v2")
    p_s.add_argument("--set", dest="overrides", action="append", default=[],
                     metavar="KEY=VALUE", help="上書きパラメータ (複数可)")
    p_s.add_argument("--slope-source", dest="slope_source", default=None,
                     choices=("none", "pitch"),
                     help="candidate 側の SLOPE_ACCX 給電ソース (省略時 case の設定)")
    p_s.add_argument("--n-jobs", type=int, default=1)
    p_s.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD_PCT)
    p_s.add_argument("--splits", default=None)
    p_s.add_argument("--out", type=Path, default=None)

    args = parser.parse_args()
    splits = tuple(s.strip() for s in args.splits.split(",")) if args.splits else None

    if args.mode == "metrics":
        horizons = HORIZONS if args.score_horizons else None
        cells, ok = judge_from_metrics(
            args.metrics, candidate=args.candidate, reference=args.reference,
            horizons=horizons, threshold_pct=args.threshold, splits=splits,
        )
    else:
        overrides = _parse_overrides(args.overrides)
        if not overrides and args.slope_source is None:
            parser.error("screen には --set KEY=VALUE か --slope-source が最低 1 つ必要です")
        cells, ok = screen_params(
            args.root, args.scenario, case_name=args.case, overrides=overrides,
            slope_source=args.slope_source,
            n_jobs=args.n_jobs, threshold_pct=args.threshold, splits=splits,
        )
    if args.out:
        cells.to_csv(args.out, index=False)
        print(f"[gate] セル表: {args.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
