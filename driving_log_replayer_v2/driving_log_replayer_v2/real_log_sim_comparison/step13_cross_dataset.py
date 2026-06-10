#!/usr/bin/env python3
"""Stage 13: collection 横断のモデル×データセット分析 (rollout 再実行なし・JSON 再集計のみ).

collect_datasets.py が収集した per-dataset の機械可読メトリクス
(comparison/metrics_closed_loop.json + comparison/cases/cases_metrics.json) を再集計し、
「特定データセットに偏った分析」を脱するための横断ビューを生成する:

1. **モデル×DS 行列**: closed-loop 軌跡乖離・完走率 (dataset × sim run) と
   open-loop N-step 終端誤差 (dataset × case) のヒートマップ。
2. **正規化 mean/worst 集約** (lib._multi_agg): 各 dataset の reference_tag (baseline) 誤差で
   正規化してから横断の mean/worst を取り、ロバスト性で case をランキングする。
3. **走行特性カバレッジ** (lib._coverage の per-DS 出力): 速度域・加減速・曲率域・走行距離の
   偏りを俯瞰する。
4. **leave-one-out 安定性・外れ DS 検出**: DS を 1 つ除いた再集計でモデル比較の結論
   (best case) が入れ替わらないか、誤差プロファイルが他と乖離した DS が無いかを定量化する。

すべて per-dataset JSON の numpy 再集計のみ (open-loop rollout は step5 実行済みの値を使う)
ため数秒で完了する。LOO も O(D²·C) の再集計で済む。

出力 (<collection>/cross_dataset/):
    cross_closed_loop_heatmap.fig.json
    cross_nstep_heatmap_n<H>.fig.json
    cross_normalized_bars.fig.json
    coverage_overview.fig.json
    loo_stability.fig.json
    cross_metrics.json    (step11 マルチ DS レポートとの契約 SSOT)
    cross_summary.md

縮退規則 (単一 DS でも同一コードパス):
    N=0 → exit 1。N=1 → 行列 1 行・mean==worst、LOO/外れ検出は理由を summary に明記して
    スキップ。N=2 → LOO は実行するが参考扱いと注記。外れ検出 (z-score) は N≥3 のみ。
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

from .lib._collection import (
    CROSS_DIR_NAME,
    DatasetEntry,
    discover_collection,
    load_dataset_metrics,
)
from .lib._fig_io import write_fig_json
from .lib._figures import (
    build_fig_coverage_overview,
    build_fig_cross_closed_loop_heatmap,
    build_fig_cross_normalized_bars,
    build_fig_cross_nstep_heatmap,
    build_fig_loo_stability,
)
from .lib._multi_agg import (
    HORIZONS,
    aggregate_normalized,
    robust_score,
    score_formula_md,
)
from .lib._nstep_common import metrics_description_md

_MIN_DS_LOO = 2       # LOO を実行する最小 DS 数 (2 は参考扱い)
_MIN_DS_OUTLIER = 3   # 外れ検出 (robust z-score) の最小 DS 数
_OUTLIER_Z = 2.0


def _short_labels(ds_ids: list[str]) -> dict[str, str]:
    """dataset_id → 短縮表示ラベル (先頭 8 文字、衝突時はフル ID)。"""
    short = {ds: ds[:8] for ds in ds_ids}
    counts = Counter(short.values())
    return {ds: (s if counts[s] == 1 else ds) for ds, s in short.items()}


# ---------------------------------------------------------------------------
# 行列化
# ---------------------------------------------------------------------------


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
    """ds × case の open-loop 終端誤差 (horizon 別 long/lat/yaw)。欠損は NaN。

    horizon は _multi_agg.HORIZONS (正規化フロアが定義された N) のうち
    全 DS の cases_metrics に存在するものに限定する。
    """
    ds_ids = list(metrics)
    case_tags = sorted({tag for m in metrics.values() for tag in m["cases"]["cases"]})
    horizons = [
        h for h in HORIZONS
        if all(str(h) in (c.get("by_h") or {})
               for m in metrics.values() for c in m["cases"]["cases"].values())
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


def _resolve_reference_tag(metrics: dict[str, dict]) -> str:
    """DS 間で reference_tag の一致を確認 (不一致は WARN し最頻値を採用)。"""
    tags = [m["cases"].get("reference_tag") or "" for m in metrics.values()]
    counts = Counter(t for t in tags if t)
    if not counts:
        return ""
    ref, _ = counts.most_common(1)[0]
    if len(counts) > 1:
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


def cross_normalized(
    metrics: dict[str, dict], open_m: dict, reference_tag: str
) -> dict:
    """case ごとの dataset 横断正規化集約とロバストスコア。

    baseline (正規化分母) は各 DS の reference_tag ケース。reference を持たない DS は
    正規化から除外する (除外 DS は excluded に列挙)。
    返り値: {"horizons", "baseline_ds", "excluded", "aggs": {case: agg}, "scores": {case: float},
             "ranking": [case...]}
    """
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
    if excluded:
        print(f"[WARN] reference_tag={reference_tag} の有効な baseline が無い DS を"
              f"正規化集約から除外: {excluded}", file=sys.stderr)

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


# ---------------------------------------------------------------------------
# leave-one-out / 外れ検出 (集約済み per_ds 値の再集計のみ)
# ---------------------------------------------------------------------------


def leave_one_out(norm: dict) -> dict | None:
    """DS を 1 つずつ除外した正規化再集計でランキング安定性を測る。

    norm["aggs"][case]["per_ds"] (正規化済み per-DS 値) から numpy 再集計するだけなので
    rollout は不要。返り値:
      {"full_best": case, "ds_ids": [...], "case_tags": [...],
       "score_delta": 2D list (DS × case), "by_excluded": {ds: {"ranking", "best", "best_changed"}}}
    """
    aggs, scores = norm["aggs"], norm["scores"]
    horizons = norm["horizons"]
    ds_ids = norm["baseline_ds"]
    if len(ds_ids) < _MIN_DS_LOO or not scores:
        return None
    case_tags = list(aggs)
    full_best = min(scores, key=scores.get)

    # per-DS の正規化値をテーブル化: vals[case][ds] = {h: {nyaw,nlong,nlat}}
    vals: dict[str, dict[str, dict]] = {
        tag: {d["dataset_id"]: d["by_h"] for d in aggs[tag]["per_ds"]} for tag in case_tags
    }

    def _score_without(tag: str, excluded: str) -> float | None:
        rows = [by_h for ds, by_h in vals[tag].items() if ds != excluded]
        if not rows:
            return None
        s = 0.0
        from .lib._multi_agg import POS_W, WORST_W  # noqa: PLC0415

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
    """正規化誤差プロファイルの robust z-score (median/MAD) で外れ DS を検出する。

    プロファイル = DS ごとの「全 case (reference 除く)・全 horizon の正規化誤差平均」。
    他 DS と大きく傾向が違う DS (|z| > _OUTLIER_Z) は、評価結論を歪める可能性として報告する。
    """
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


# ---------------------------------------------------------------------------
# 出力
# ---------------------------------------------------------------------------


def _nan_to_none(arr: np.ndarray) -> list:
    return [[None if not np.isfinite(v) else float(v) for v in row] for row in arr]


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
) -> None:
    """step11 マルチ DS レポートとの契約 SSOT (cross_metrics.json) を書き出す。"""

    def _agg_jsonable(agg: dict) -> dict:
        return {
            "per_ds": agg["per_ds"],
            "by_h": {str(h): v for h, v in agg["by_h"].items()},
        }

    payload = {
        "schema_version": 1,
        "n_datasets": len(metrics),
        "dataset_ids": list(metrics),
        "missing": [{"dataset_id": e.dataset_id, "status": e.status} for e in missing],
        "closed_loop": {
            "run_tags": closed_m["run_tags"],
            "s2r_mean_m": _nan_to_none(closed_m["s2r_mean_m"]),
            "completion_pct": _nan_to_none(closed_m["completion_pct"]),
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
            "score_delta": _nan_to_none(loo["score_delta"]),
            "by_excluded": loo["by_excluded"],
        },
        "outliers": outliers,
    }
    out_path.write_text(
        json.dumps(payload, ensure_ascii=False, allow_nan=False, indent=1), encoding="utf-8"
    )
    print(f"  保存: {out_path}")


def write_cross_summary_md(
    out_path: Path,
    *,
    metrics: dict[str, dict],
    missing: list[DatasetEntry],
    labels: dict[str, str],
    open_m: dict,
    norm: dict,
    loo: dict | None,
    outliers: list[dict] | None,
    reference_tag: str,
) -> None:
    """横断分析の Markdown サマリ (旧 multi_dataset_tune.evaluate_cases の表を継承)。"""
    n = len(metrics)
    horizons = open_m["horizons"]
    lines: list[str] = [
        f"# データセット横断サマリ ({n} datasets)",
        "",
        "per-dataset の解析成果物 (metrics JSON) を再集計した横断分析。"
        "open-loop は N-step rollout の終端誤差 RMSE、closed-loop は軌跡乖離・完走率に基づく。",
        "",
    ]

    # --- データセット一覧 ---
    lines += ["## データセット一覧", "",
              "| dataset | 走行距離 [m] | 走行時間 [s] | カーブ数 | 最高速 [m/s] |", "|---|---|---|---|---|"]
    for ds, m in metrics.items():
        cov = (m["closed"].get("real") or {}).get("coverage") or {}
        curv = cov.get("curvature") or {}
        lines.append(
            f"| {labels[ds]} | {cov.get('dist_m', 0):.0f} | {cov.get('duration_s', 0):.0f} "
            f"| {curv.get('curve_count', '—')} | {cov.get('v_max_mps', 0):.1f} |"
        )
    lines.append("")
    if missing:
        lines += ["### 解析未完了 / 失敗データセット", ""]
        for e in missing:
            lines.append(f"- `{e.dataset_id}` — status: {e.status}"
                         " (metrics JSON 欠損のため横断集約から除外)")
        lines.append("")

    # --- 正規化集約表 (旧 evaluate_cases 継承) ---
    if norm["aggs"]:
        lines += [
            "## case 横断評価 (open-loop 終端誤差・正規化集約)", "",
            metrics_description_md(), "",
            f"各 dataset の baseline (`{reference_tag}`) 誤差で成分別・horizon 別に正規化した値で"
            "集約 (n* = baseline 比、小さいほど良い)。per-dataset セルは生値 `縦/横/yaw` [cm/cm/deg]。",
            "",
            f"> **スコア** {score_formula_md(tuple(horizons))}。",
            "",
        ]
        if norm["excluded"]:
            lines += [f"> 正規化から除外した DS (baseline 無効): "
                      f"{', '.join(labels.get(d, d) for d in norm['excluded'])}", ""]
        ds_cols = norm["baseline_ds"]
        for h in horizons:
            lines.append(f"### N={h} 終端誤差 (per-dataset 縦/横/yaw + 正規化集約)")
            lines.append("")
            header = (
                "| case | "
                + " | ".join(f"{labels[ds]} 縦/横/yaw" for ds in ds_cols)
                + " | nyaw_mean | nyaw_worst | nlong_mean | nlong_worst | nlat_mean | nlat_worst |"
            )
            lines += [header, "|" + "---|" * (1 + len(ds_cols) + 6)]
            for tag in norm["ranking"]:
                agg = norm["aggs"][tag]
                per_ds = {d["dataset_id"]: d["by_h"] for d in agg["per_ds"]}
                cells = " | ".join(
                    (
                        f"{per_ds[ds][h]['long']:.2f}/{per_ds[ds][h]['lat']:.2f}"
                        f"/{per_ds[ds][h]['yaw']:.3f}"
                        if ds in per_ds
                        else "—"
                    )
                    for ds in ds_cols
                )
                b = agg["by_h"][h]
                lines.append(
                    f"| {tag} | {cells} "
                    f"| {b['nyaw_mean']:.3f} | {b['nyaw_worst']:.3f} "
                    f"| {b['nlong_mean']:.3f} | {b['nlong_worst']:.3f} "
                    f"| {b['nlat_mean']:.3f} | {b['nlat_worst']:.3f} |"
                )
            lines.append("")
        lines += ["### スコアランキング (小さいほど良い)", ""]
        for i, tag in enumerate(norm["ranking"], start=1):
            lines.append(f"{i}. `{tag}` — score {norm['scores'][tag]:.4f}")
        lines.append("")

    # --- LOO / 外れ DS ---
    lines += ["## ランキング安定性 (leave-one-out)", ""]
    if loo is None:
        lines += [f"データセット数 {n} < {_MIN_DS_LOO + 1} のため leave-one-out 分析は省略"
                  " (DS を増やすと有効になる)。", ""]
    else:
        if len(loo["ds_ids"]) == _MIN_DS_LOO:
            lines += ["> DS 数 2 のため LOO は「残り 1 DS のみの順位」となる。参考扱い。", ""]
        changed = [ds for ds, r in loo["by_excluded"].items() if r["best_changed"]]
        lines.append(f"全 DS での best case: `{loo['full_best']}`")
        lines.append("")
        if changed:
            lines.append("**除外すると best case が入れ替わる DS (結論を左右する):**")
            for ds in changed:
                r = loo["by_excluded"][ds]
                lines.append(f"- `{labels.get(ds, ds)}` を除外 → best が `{r['best']}` に変化")
        else:
            lines.append("どの DS を除外しても best case は不変 (ランキング頑健)。")
        lines.append("")

    lines += ["## 外れデータセット検出 (正規化誤差プロファイルの robust z-score)", ""]
    if outliers is None:
        lines += [f"データセット数 {n} < {_MIN_DS_OUTLIER} のため外れ検出は省略。", ""]
    else:
        flagged = [r for r in outliers if r["outlier"]]
        lines += ["| dataset | プロファイル平均 (正規化誤差) | z | 判定 |", "|---|---|---|---|"]
        for r in outliers:
            mark = "**外れ候補**" if r["outlier"] else ""
            lines.append(
                f"| {labels.get(r['dataset_id'], r['dataset_id'])} "
                f"| {r['profile_mean']:.3f} | {r['z']:+.2f} | {mark} |"
            )
        lines.append("")
        if flagged:
            lines.append(
                f"> |z| > {_OUTLIER_Z} の DS は他と誤差傾向が大きく異なる。データ品質"
                " (localization 異常・特異な走行条件) を確認し、必要なら除外して再集計すること。"
            )
            lines.append("")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  保存: {out_path}")


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------


def run_cross_analysis(collection_dir: Path, out_dir: Path | None = None) -> Path:
    """横断分析を実行し cross_dataset/ 出力ディレクトリを返す (run_batch から直接呼べる)。"""
    out_dir = out_dir or collection_dir / CROSS_DIR_NAME
    entries = discover_collection(collection_dir)
    if not entries:
        raise FileNotFoundError(f"collection にデータセットが見つかりません: {collection_dir}")

    metrics: dict[str, dict] = {}
    missing: list[DatasetEntry] = []
    for e in entries:
        m = load_dataset_metrics(e)
        if m is None:
            missing.append(e)
            print(f"[WARN] {e.dataset_id}: metrics JSON 欠損 (status={e.status})"
                  " — 横断集約から除外", file=sys.stderr)
        else:
            metrics[e.dataset_id] = m
    if not metrics:
        raise FileNotFoundError(
            "有効な metrics JSON を持つデータセットが 0 件です。"
            "per-dataset 解析 (step4/step6) を済ませてから収集してください"
        )
    n = len(metrics)
    labels = _short_labels(list(metrics))
    print(f"datasets: {[labels[ds] for ds in metrics]} (有効 {n} / 欠損 {len(missing)})")

    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. 行列
    closed_m = build_closed_loop_matrix(metrics)
    open_m = build_open_loop_matrix(metrics)
    ds_labels = [labels[ds] for ds in closed_m["ds_ids"]]
    fig = build_fig_cross_closed_loop_heatmap(
        ds_labels, closed_m["run_tags"], closed_m["s2r_mean_m"], closed_m["completion_pct"]
    )
    if fig is not None:
        write_fig_json(fig, out_dir / "cross_closed_loop_heatmap")
    for h in open_m["horizons"]:
        fig = build_fig_cross_nstep_heatmap(
            [labels[ds] for ds in open_m["ds_ids"]], open_m["case_tags"],
            open_m["mats"][h], h,
        )
        if fig is not None:
            write_fig_json(fig, out_dir / f"cross_nstep_heatmap_n{h}")

    # 2. 正規化集約
    reference_tag = _resolve_reference_tag(metrics)
    norm = cross_normalized(metrics, open_m, reference_tag)
    norm["reference_tag"] = reference_tag
    if norm["aggs"]:
        fig = build_fig_cross_normalized_bars(norm["aggs"], norm["horizons"], norm["scores"])
        if fig is not None:
            write_fig_json(fig, out_dir / "cross_normalized_bars")

    # 3. カバレッジ
    cov_rows = [
        {"label": labels[ds], "coverage": (m["closed"].get("real") or {}).get("coverage")}
        for ds, m in metrics.items()
    ]
    fig = build_fig_coverage_overview(cov_rows)
    if fig is not None:
        write_fig_json(fig, out_dir / "coverage_overview")

    # 4. LOO / 外れ検出
    loo = leave_one_out(norm)
    if loo is not None:
        fig = build_fig_loo_stability(
            [labels[ds] for ds in loo["ds_ids"]],
            loo["case_tags"],
            loo["score_delta"],
            [loo["by_excluded"][ds]["best_changed"] for ds in loo["ds_ids"]],
        )
        if fig is not None:
            write_fig_json(fig, out_dir / "loo_stability")
    outliers = detect_outliers(norm, reference_tag)

    # 5. 機械可読 SSOT + Markdown サマリ
    write_cross_metrics_json(
        out_dir / "cross_metrics.json",
        metrics=metrics, missing=missing, closed_m=closed_m, open_m=open_m,
        norm=norm, loo=loo, outliers=outliers,
    )
    write_cross_summary_md(
        out_dir / "cross_summary.md",
        metrics=metrics, missing=missing, labels=labels, open_m=open_m,
        norm=norm, loo=loo, outliers=outliers, reference_tag=reference_tag,
    )
    return out_dir


def main() -> None:
    ap = argparse.ArgumentParser(
        description="collection 横断のモデル×データセット分析 (Stage 13)"
    )
    ap.add_argument("--collection-dir", required=True,
                    help="collect_datasets.py の収集先 (collection root)")
    ap.add_argument("--out-dir", default="",
                    help=f"出力先 (既定: <collection>/{CROSS_DIR_NAME})")
    args = ap.parse_args()

    collection_dir = Path(args.collection_dir)
    out_dir = Path(args.out_dir) if args.out_dir else None
    try:
        result = run_cross_analysis(collection_dir, out_dir)
    except FileNotFoundError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"\n完了。出力先: {result}")


if __name__ == "__main__":
    main()
