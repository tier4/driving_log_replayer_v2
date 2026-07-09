"""collection 横断の再集計・物理妥当性チェックで使う共通関数.

step13_cross_dataset.py と、per-dataset 側の cases 集約で使う JSON 化処理を
ここに寄せる。純関数ベースにしておくことで、rollout を伴わない再集計ロジックを
他スクリプトから再利用しやすくする。
"""

from __future__ import annotations

from collections import Counter
import math
from pathlib import Path
import sys

import numpy as np

from ._collection import DatasetEntry
from ._models_config import load_models_doc
from ._multi_agg import HORIZONS, POS_W, WORST_W, aggregate_normalized, robust_score
from ._physical_validity import (
    compute_cross_long_rows,
    compute_cross_steer_rows,
    fit_long_cross_dataset_bounded,
)

_MIN_DS_LOO = 2
_MIN_DS_OUTLIER = 3
_OUTLIER_Z = 2.0


def build_closed_loop_matrix(metrics: dict[str, dict]) -> dict:
    """ds × sim run の closed-loop 行列 (s2r 平均 / 完走率)。欠損は NaN。"""
    ds_ids = list(metrics)
    run_tags = sorted({tag for m in metrics.values() for tag in m["closed"]["runs"]})
    shape = (len(ds_ids), len(run_tags))
    s2r = np.full(shape, np.nan)
    comp = np.full(shape, np.nan)
    for i, ds in enumerate(ds_ids):
        runs = metrics[ds]["closed"]["runs"]
        for j, tag in enumerate(run_tags):
            rec = runs.get(tag)
            if rec is None:
                continue
            if rec.get("s2r_mean_m") is not None:
                s2r[i, j] = rec["s2r_mean_m"]
            if rec.get("completion_pct") is not None:
                comp[i, j] = rec["completion_pct"]
    return {"ds_ids": ds_ids, "run_tags": run_tags, "s2r_mean_m": s2r, "completion_pct": comp}


def build_open_loop_matrix(metrics: dict[str, dict]) -> dict:
    """ds × case の open-loop 終端誤差 (horizon 別 long/lat/yaw)。欠損は NaN。"""
    ds_ids = list(metrics)
    case_tags = sorted({tag for m in metrics.values() for tag in m["cases"]["cases"]})
    horizons = [
        h for h in HORIZONS
        if all(
            str(h) in (c.get("by_h") or {})
            for m in metrics.values() for c in m["cases"]["cases"].values()
        )
    ]
    mats = {
        h: {k: np.full((len(ds_ids), len(case_tags)), np.nan) for k in ("long", "lat", "yaw")}
        for h in horizons
    }
    for i, ds in enumerate(ds_ids):
        cases = metrics[ds]["cases"]["cases"]
        for j, tag in enumerate(case_tags):
            rec = cases.get(tag)
            if rec is None:
                continue
            for h in horizons:
                by = rec["by_h"].get(str(h)) or {}
                for k in ("long", "lat", "yaw"):
                    if by.get(k) is not None:
                        mats[h][k][i, j] = by[k]
    return {"ds_ids": ds_ids, "case_tags": case_tags, "horizons": horizons, "mats": mats}


def resolve_reference_tag(metrics: dict[str, dict], *, verbose: bool = False) -> str:
    """DS 間で reference_tag の一致を確認し、最頻値を返す。"""
    tags = [m["cases"].get("reference_tag") or "" for m in metrics.values()]
    counts = Counter(t for t in tags if t)
    if not counts:
        return ""
    ref, _ = counts.most_common(1)[0]
    if len(counts) > 1 and verbose:
        print(f"[WARN] reference_tag が DS 間で不一致: {dict(counts)} → '{ref}' を採用",
              file=sys.stderr)
    return ref


def _case_by_h(metrics: dict, ds: str, tag: str, horizons: list[int]) -> dict[int, dict] | None:
    """1 DS・1 case の {h: {yaw,long,lat}} (いずれかが欠損なら None)。"""
    rec = metrics[ds]["cases"]["cases"].get(tag)
    if rec is None:
        return None
    out = {}
    for h in horizons:
        by = rec["by_h"].get(str(h)) or {}
        if any(by.get(k) is None for k in ("yaw", "long", "lat")):
            return None
        out[h] = {k: by[k] for k in ("yaw", "long", "lat")}
    return out


def cross_normalized(metrics: dict[str, dict], open_m: dict, reference_tag: str,
                     *, verbose: bool = False) -> dict:
    """case ごとの dataset 横断正規化集約とロバストスコア。"""
    horizons = open_m["horizons"]
    if not horizons or not reference_tag:
        return {"horizons": horizons, "baseline_ds": [], "excluded": list(metrics),
                "aggs": {}, "scores": {}, "ranking": []}

    baselines: dict[str, dict[int, dict]] = {}
    excluded: list[str] = []
    for ds in metrics:
        base = _case_by_h(metrics, ds, reference_tag, horizons)
        if base is None or any(
            base[h]["yaw"] <= 0 or (base[h]["long"] <= 0 and base[h]["lat"] <= 0)
            for h in horizons
        ):
            excluded.append(ds)
            continue
        baselines[ds] = base
    if excluded and verbose:
        print(
            f"[WARN] reference_tag={reference_tag} の有効な baseline が無い DS を"
            f"正規化集約から除外: {excluded}",
            file=sys.stderr,
        )

    aggs: dict[str, dict] = {}
    scores: dict[str, float] = {}
    for tag in open_m["case_tags"]:
        per_ds = []
        for ds in baselines:
            m = _case_by_h(metrics, ds, tag, horizons)
            if m is not None:
                per_ds.append((ds, m))
        if not per_ds:
            continue
        agg = aggregate_normalized(per_ds, baselines, tuple(horizons))
        aggs[tag] = agg
        scores[tag] = robust_score(agg, tuple(horizons))
    ranking = sorted(scores, key=scores.get)
    return {"horizons": horizons, "baseline_ds": list(baselines), "excluded": excluded,
            "aggs": aggs, "scores": scores, "ranking": ranking}


def leave_one_out(norm: dict) -> dict | None:
    """DS を 1 つずつ除外した正規化再集計でランキング安定性を測る。"""
    aggs, scores = norm["aggs"], norm["scores"]
    horizons = norm["horizons"]
    ds_ids = norm["baseline_ds"]
    if len(ds_ids) < _MIN_DS_LOO or not scores:
        return None
    case_tags = list(aggs)
    full_best = min(scores, key=scores.get)

    vals: dict[str, dict[str, dict]] = {
        tag: {d["dataset_id"]: d["by_h"] for d in aggs[tag]["per_ds"]} for tag in case_tags
    }

    def _score_without(tag: str, excluded: str) -> float | None:
        rows = [by_h for ds, by_h in vals[tag].items() if ds != excluded]
        if not rows:
            return None
        s = 0.0
        for h in horizons:
            ny = [r[h]["nyaw"] for r in rows]
            nlo = [r[h]["nlong"] for r in rows]
            nla = [r[h]["nlat"] for r in rows]
            s += float(np.mean(ny)) + POS_W * (float(np.mean(nlo)) + float(np.mean(nla)))
            s += WORST_W * (max(ny) + POS_W * (max(nlo) + max(nla)))
        return s

    score_delta = np.full((len(ds_ids), len(case_tags)), np.nan)
    by_excluded: dict[str, dict] = {}
    for i, ds in enumerate(ds_ids):
        loo_scores: dict[str, float] = {}
        for j, tag in enumerate(case_tags):
            s = _score_without(tag, ds)
            if s is None:
                continue
            loo_scores[tag] = s
            score_delta[i, j] = s - scores[tag]
        ranking = sorted(loo_scores, key=loo_scores.get)
        best = ranking[0] if ranking else None
        by_excluded[ds] = {
            "ranking": ranking,
            "best": best,
            "best_changed": best is not None and best != full_best,
        }
    return {
        "full_best": full_best,
        "ds_ids": ds_ids,
        "case_tags": case_tags,
        "score_delta": score_delta,
        "by_excluded": by_excluded,
    }


def detect_outliers(norm: dict, reference_tag: str) -> list[dict] | None:
    """正規化誤差プロファイルの robust z-score (median/MAD) で外れ DS を検出する。"""
    ds_ids = norm["baseline_ds"]
    if len(ds_ids) < _MIN_DS_OUTLIER:
        return None
    horizons = norm["horizons"]
    profile: dict[str, list[float]] = {ds: [] for ds in ds_ids}
    for tag, agg in norm["aggs"].items():
        if tag == reference_tag:
            continue
        for d in agg["per_ds"]:
            vals = [
                (d["by_h"][h]["nyaw"] + d["by_h"][h]["nlong"] + d["by_h"][h]["nlat"]) / 3.0
                for h in horizons
            ]
            profile[d["dataset_id"]].extend(vals)
    means = {ds: float(np.mean(v)) for ds, v in profile.items() if v}
    if len(means) < _MIN_DS_OUTLIER:
        return None
    arr = np.array(list(means.values()))
    med = float(np.median(arr))
    mad = float(np.median(np.abs(arr - med)))
    scale = mad * 1.4826 if mad > 1e-12 else float(np.std(arr)) or 1e-12
    out = []
    for ds, m in means.items():
        z = (m - med) / scale
        out.append({"dataset_id": ds, "profile_mean": m, "z": z,
                    "outlier": abs(z) > _OUTLIER_Z})
    return sorted(out, key=lambda r: -abs(r["z"]))


def _discover_models_doc(collection_dir: Path, scenario_path: Path | None = None,
                         *, verbose: bool = False):
    """scenario.yaml から Conditions.models/cases/overlay を含む ModelsDoc を読む。"""
    if scenario_path is not None and scenario_path.is_file():
        try:
            return load_models_doc(scenario_path)
        except Exception as exc:
            if verbose:
                print(f"[WARN] --scenario の読み込みに失敗: {scenario_path} ({exc})",
                      file=sys.stderr)

    for p in sorted((collection_dir / "scenarios").glob("*.yaml")):
        try:
            doc = load_models_doc(p)
        except Exception:
            continue
        if doc.models:
            return doc
    return None


def collect_physical_validity(metrics: dict[str, dict]) -> dict[str, dict | None]:
    """各 DS の cases_metrics.json["physical_validity"] を取り出す (欠損は None)。"""
    return {ds: (m["cases"].get("physical_validity")) for ds, m in metrics.items()}


def cross_physical_validity_analysis(
    entries: list[DatasetEntry],
    metrics: dict[str, dict],
    collection_dir: Path,
    scenario_path: Path | None = None,
    *,
    verbose: bool = False,
) -> dict | None:
    """per-dataset の物理妥当性同定 (縦/操舵) を dataset 横断で集約する。"""
    pv_by_ds = collect_physical_validity(metrics)
    per_ds_long = {ds: pv["long"] for ds, pv in pv_by_ds.items() if pv and pv.get("long")}
    per_ds_steer = {ds: pv["steer"] for ds, pv in pv_by_ds.items() if pv and pv.get("steer")}

    if not per_ds_long and not per_ds_steer:
        return None

    models_doc = _discover_models_doc(collection_dir, scenario_path, verbose=verbose)
    models = None
    if models_doc is not None:
        models = {
            name: spec for name, spec in models_doc.models.items() if name in models_doc.cases_list
        }

    cross_fit_long = None
    rows_long: list[dict] = []
    if per_ds_long:
        cross_fit_long = fit_long_cross_dataset_bounded(entries, per_ds_long)
        rows_long = compute_cross_long_rows(entries, per_ds_long, cross_fit_long, models)

    rows_steer: list[dict] = []
    if per_ds_steer:
        rows_steer = compute_cross_steer_rows(entries, per_ds_steer, models)

    return {
        "cross_fit_long": cross_fit_long,
        "rows_long": rows_long,
        "rows_steer": rows_steer,
        "models": models,
        "n_long": len(per_ds_long),
        "n_steer": len(per_ds_steer),
    }


def write_cross_metrics_json(
    out_path: Path,
    *,
    metrics: dict[str, dict],
    missing: list[DatasetEntry],
    closed_m: dict,
    open_m: dict,
    norm: dict,
    loo: dict | None,
    outliers: list[dict] | None,
    pv: dict | None = None,
) -> None:
    """step13 マルチ DS レポートとの契約 SSOT (cross_metrics.json) を書き出す。"""

    def _agg_jsonable(agg: dict) -> dict:
        return {
            "per_ds": agg["per_ds"],
            "by_h": {str(h): v for h, v in agg["by_h"].items()},
        }

    def _finite(x) -> float | None:
        xf = float(x)
        return xf if math.isfinite(xf) else None

    def _pv_jsonable(cross_pv: dict | None) -> dict | None:
        if cross_pv is None:
            return None
        cf = cross_pv.get("cross_fit_long")
        cf_json = None
        if cf is not None:
            cf_json = {k: (_finite(v) if isinstance(v, float) else v) for k, v in cf.items()}
        return {
            "long_cross_fit": cf_json,
            "n_datasets": {
                "long": cross_pv.get("n_long", 0),
                "steer": cross_pv.get("n_steer", 0),
            },
        }

    payload = {
        "schema_version": 1,
        "n_datasets": len(metrics),
        "dataset_ids": list(metrics),
        "missing": [{"dataset_id": e.dataset_id, "status": e.status} for e in missing],
        "closed_loop": {
            "run_tags": closed_m["run_tags"],
            "s2r_mean_m": [[None if not np.isfinite(v) else float(v) for v in row]
                             for row in closed_m["s2r_mean_m"]],
            "completion_pct": [[None if not np.isfinite(v) else float(v) for v in row]
                                 for row in closed_m["completion_pct"]],
        },
        "open_loop": {
            "case_tags": open_m["case_tags"],
            "horizons": open_m["horizons"],
            "reference_tag": norm.get("reference_tag", ""),
            "baseline_ds": norm["baseline_ds"],
            "normalization_excluded": norm["excluded"],
            "aggs": {tag: _agg_jsonable(a) for tag, a in norm["aggs"].items()},
            "scores": norm["scores"],
            "ranking": norm["ranking"],
        },
        "coverage": {
            ds: (m["closed"].get("real") or {}).get("coverage") for ds, m in metrics.items()
        },
        "loo": None
        if loo is None
        else {
            "full_best": loo["full_best"],
            "ds_ids": loo["ds_ids"],
            "case_tags": loo["case_tags"],
            "score_delta": [[None if not np.isfinite(v) else float(v) for v in row]
                             for row in loo["score_delta"]],
            "by_excluded": loo["by_excluded"],
        },
        "outliers": outliers,
        "physical_validity": _pv_jsonable(pv),
    }
    out_path.write_text(
        __import__("json").dumps(payload, ensure_ascii=False, allow_nan=False, indent=1),
        encoding="utf-8",
    )
