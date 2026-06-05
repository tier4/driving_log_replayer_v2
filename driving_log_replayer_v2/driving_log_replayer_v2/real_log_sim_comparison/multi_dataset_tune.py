#!/usr/bin/env python3
"""
マルチデータセット横断での open-loop N-step rollout 集約とロバスト best_normal 同定。

クラウドは 1 評価ジョブ = 1 データセットのため、各ジョブが出力する real.lite を
`collect_real_lite.py` で収集ディレクトリ (<collection>/<dataset_id>/real.lite) に集約した上で、
本モジュールが dataset 横断で誤差を集計しロバストなパラメータを同定する。

設計の要点:
- closed-loop はローカルで退化するため評価は open-loop N-step rollout (step5.run_rollout) のみ。
- 最大 horizon (N=20) の終端誤差 RMSE で評価 (step7 sweep と同じ指標。小 N は seed バイアス)。
- **per-dataset 正規化**: 各 dataset の baseline (パラメータ無補正) 誤差で割って正規化してから
  dataset 横断の mean / worst(max) を取る。生の deg/cm は baseline 誤差の大きい dataset が
  支配し「全 dataset で良い」を達成できないため (= ロバスト化の核心)。

使い方:
    # cases.yaml の全ケースを dataset 横断評価 (multi_cases_summary.md 出力)
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
        --collection-dir sample/multi --cases-config sample/cases.yaml

    # ロバスト best_normal の結合探索も実行
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
        --collection-dir sample/multi --cases-config sample/cases.yaml --search
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import itertools
from pathlib import Path
import statistics as stats
import sys

from . import step5_analyze_nstep as s5
from .lib._cases_config import load_cases_config
from .lib._io import resolve_lite_bag
from .lib._nstep_common import metrics_description_md, rmse_by_horizon

# 評価する horizon 群。step7 sweep と同じ最大 horizon (N=20) に加え、より長い N=40 で
# dynamics 累積差を観測する。同定スコア (_score) は両 horizon を等重みで集約する。
HORIZONS = (20, 40)
STRIDE = 5
WHEELBASE = 4.76012
# per-dataset 正規化の分母フロア (horizon 別・成分別)。ほぼ直進・低ダイナミクス走行は baseline
# 誤差が極小で、相対誤差 (err/baseline) が暴発し worst-case を支配する (絶対値は微小なのに)。
# pos を縦/横に分けると縦・横でスケールが大きく異なる。20 dataset の baseline 分布
# (load_datasets の print) では【縦の方が横より大きい】(縦 1.3〜7.4cm vs 横 0.1〜3.4cm @N20;
# baseline モデルは縦方向の遅延/時定数が支配的)。フロアが大きすぎると baseline ですら正規化値が
# 1 未満になりその成分の寄与が一律縮小される (= 信号を殺す) ため、各成分は分布下位の低ダイナ走行
# のみをクリップする水準に校正する (横は小さいのでフロアも低め)。
YAW_FLOOR = {20: 0.12, 40: 0.24}   # deg (分布下位 ~20% の低ダイナ yaw をクリップ)
LONG_FLOOR = {20: 2.0, 40: 4.5}    # cm  (縦は誤差が大きい → フロアも大きめ)
LAT_FLOOR = {20: 0.6, 40: 1.2}     # cm  (横は誤差が小さい → フロアも小さめ)
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
    """
    Dataset 横断で per-dataset 正規化した yaw/縦/横の mean と worst(max) を horizon 別に返す。

    pos(2D) を縦(long)/横(lat) に分解し、各成分・各 horizon を baseline 比で正規化する。
    返り値:
      per_ds: [{dataset_id, by_h: {h: {yaw,long,lat, nyaw,nlong,nlat}}}]
      by_h:   {h: {nyaw_mean,nyaw_worst, nlong_mean,nlong_worst, nlat_mean,nlat_worst}}
    """
    per_ds = []
    for ctx in ctxs:
        m = _eval(ctx, override, model_type)
        by_h = {}
        for h in HORIZONS:
            b = ctx.base_metric[h]
            # 分母を成分別・horizon 別フロアで下限クリップ (低ダイナミクス走行の相対誤差暴発を防ぐ)
            by_h[h] = {
                "yaw": m[h]["yaw"],
                "long": m[h]["long"],
                "lat": m[h]["lat"],
                "nyaw": m[h]["yaw"] / max(b["yaw"], YAW_FLOOR[h]),
                "nlong": m[h]["long"] / max(b["long"], LONG_FLOOR[h]),
                "nlat": m[h]["lat"] / max(b["lat"], LAT_FLOOR[h]),
            }
        per_ds.append({"dataset_id": ctx.dataset_id, "by_h": by_h})

    by_h_agg = {}
    for h in HORIZONS:
        nyaws = [d["by_h"][h]["nyaw"] for d in per_ds]
        nlongs = [d["by_h"][h]["nlong"] for d in per_ds]
        nlats = [d["by_h"][h]["nlat"] for d in per_ds]
        by_h_agg[h] = {
            "nyaw_mean": stats.mean(nyaws),
            "nyaw_worst": max(nyaws),
            "nlong_mean": stats.mean(nlongs),
            "nlong_worst": max(nlongs),
            "nlat_mean": stats.mean(nlats),
            "nlat_worst": max(nlats),
        }
    return {"per_ds": per_ds, "by_h": by_h_agg}


_WORST_W = 0.5  # worst-case 項の重み (ユーザー方針: mean+worst 両方を balance)


_POS_W = 0.5  # 縦・横 各成分の重み。pos を縦横に分けても yaw:位置 = 1:1 を維持する
#            (位置 = 0.5·縦 + 0.5·横)。旧 nyaw+npos のバランスと整合させるための補正。


def _score(agg: dict) -> float:
    """ロバスト目的関数: 全 horizon の正規化 mean + worst (yaw + 0.5·縦 + 0.5·横)。小さいほど良い。

    N=20/N=40 を等重みで集約する。縦・横は各 0.5 倍で合算し yaw:位置 = 1:1 に保つ
    (pos を縦横へ分割しても yaw の相対重みが半減しないようにする)。mean だけだと縦/横の mean を
    稼ぐ proxy が特定エリアの worst を悪化させても採用されてしまうため、worst を重み付きで加えて
    mean と worst を両立させる。
    """
    s = 0.0
    for h in HORIZONS:
        b = agg["by_h"][h]
        s += b["nyaw_mean"] + _POS_W * (b["nlong_mean"] + b["nlat_mean"])
        s += _WORST_W * (b["nyaw_worst"] + _POS_W * (b["nlong_worst"] + b["nlat_worst"]))
    return s


def _fmt_agg(tag: str, agg: dict) -> str:
    seg = []
    for h in HORIZONS:
        b = agg["by_h"][h]
        seg.append(
            f"N{h}[ny_m={b['nyaw_mean']:.3f}/w={b['nyaw_worst']:.3f} "
            f"nlo_m={b['nlong_mean']:.3f}/w={b['nlong_worst']:.3f} "
            f"nla_m={b['nlat_mean']:.3f}/w={b['nlat_worst']:.3f}]"
        )
    return f"{tag:14s} " + " ".join(seg)


_KUS0020 = {"k_us": 0.020}


def evaluate_cases(ctxs: list[DatasetCtx], cfg) -> str:
    """cases.yaml の全ケースを dataset 横断評価し Markdown 表を返す (horizon 別)。"""
    lines = [
        "# multi-dataset cases summary (open-loop N=%s 終端誤差)" % ",".join(map(str, HORIZONS)),
        "",
        metrics_description_md(),
        "",
        "各 dataset の baseline 誤差で成分別・horizon 別に正規化した値で集約 "
        "(n* = baseline 比、小さいほど良い)。per-dataset セルは生値 `縦/横/yaw` [cm/cm/deg]。",
        "",
        "> **同定スコア** `_score = Σ_h (nyaw_mean + "
        f"{_POS_W}·(nlong_mean + nlat_mean)) + {_WORST_W}·(nyaw_worst + "
        f"{_POS_W}·(nlong_worst + nlat_worst))`  "
        f"(h ∈ {list(HORIZONS)}、縦横各 {_POS_W} で yaw:位置=1:1、小さいほど良い)。",
        "",
    ]
    # ケースごとに 1 回だけ集約し horizon 別表で再利用
    case_aggs = {
        tag: aggregate(ctxs, cfg.find_case(tag).params, cfg.find_case(tag).vehicle_model)
        for tag in cfg.tags
    }
    for h in HORIZONS:
        lines.append(f"## N={h} 終端誤差 (per-dataset 縦/横/yaw + 正規化集約)")
        lines.append("")
        header = (
            "| case | model | "
            + " | ".join(f"{c.dataset_id[:8]} 縦/横/yaw" for c in ctxs)
            + " | nyaw_mean | nyaw_worst | nlong_mean | nlong_worst | nlat_mean | nlat_worst |"
        )
        sep = "|" + "---|" * (2 + len(ctxs) + 6)
        lines += [header, sep]
        for tag in cfg.tags:
            case = cfg.find_case(tag)
            agg = case_aggs[tag]
            b = agg["by_h"][h]
            cells = " | ".join(
                f"{d['by_h'][h]['long']:.2f}/{d['by_h'][h]['lat']:.2f}/{d['by_h'][h]['yaw']:.3f}"
                for d in agg["per_ds"]
            )
            model_short = case.vehicle_model.replace(
                "delay_steer_acc_geared_wo_fall_guard", "delay"
            )
            lines.append(
                f"| {tag} | {model_short} | {cells} "
                f"| {b['nyaw_mean']:.3f} | {b['nyaw_worst']:.3f} "
                f"| {b['nlong_mean']:.3f} | {b['nlong_worst']:.3f} "
                f"| {b['nlat_mean']:.3f} | {b['nlat_worst']:.3f} |"
            )
        lines.append("")
    return "\n".join(lines)


def robust_search(ctxs: list[DatasetCtx], cfg) -> dict:
    """
    best_normal 集合 (代理含む) を coordinate descent で dataset 横断ロバスト最適化。

    目的: 全 horizon の正規化 mean + worst (yaw+縦+横) を最小化 (_score; worst を重み付きで含む)。
    参照点 (現 best_normal) は cases.yaml の best_normal ケースから取得する
    (ハードコードしない。yaml を更新後に再探索しても整合する)。
    """
    cur_case = cfg.find_case("best_normal")
    cur_best = dict(cur_case.params)
    cur_model = cur_case.vehicle_model
    sweeps = {
        "k_us": [0.0, 0.005, 0.010, 0.012, 0.015, 0.018, 0.020, 0.025, 0.030],
        "steer_time_constant": [0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.70],
        "debug_steer_scaling_factor": [0.85, 0.88, 0.90, 0.93, 0.96, 0.98, 1.0],
        "acc_time_constant": [0.30, 0.35, 0.45, 0.50, 0.60, 0.70, 0.80, 1.0],
        "acc_time_delay": [0.10, 0.15, 0.20, 0.30, 0.40, 0.50],
    }
    cur_agg = aggregate(ctxs, cur_best, cur_model)
    print("\n## robust coordinate descent (best_normal family, cross-dataset normalized)")
    print(_fmt_agg("cur_best", cur_agg) + f"  score={_score(cur_agg):.4f}  {cur_best}")
    print(_fmt_agg("kus0020", aggregate(ctxs, _KUS0020, cur_model)))

    # worst は _score に組み込み済み (ハード guard なし) なので mean↔worst の trade を許容する。
    state = dict(cur_best)
    best_agg = cur_agg
    best_s = _score(cur_agg)
    for _pass in range(3):
        improved = False
        for pname, grid in sweeps.items():
            for v in grid:
                trial = dict(state)
                trial[pname] = v
                agg = aggregate(ctxs, trial, cur_model)
                s = _score(agg)
                if s < best_s - 1e-6:
                    best_s, best_agg, state, improved = s, agg, trial, True
        print(_fmt_agg(f"pass{_pass}", best_agg) + f"  score={best_s:.4f}  {state}")
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
    for combo in itertools.product(*(refine[k] for k in keys)):
        trial = dict(cur_best)  # descent 外の spec パラメータを保持
        trial.update({k: v for k, v in zip(keys, combo)})
        agg = aggregate(ctxs, trial, cur_model)
        s = _score(agg)
        if s < best_s - 1e-6:
            best_s, best_agg, state = s, agg, trial
    print(_fmt_agg("FINAL", best_agg) + f"  score={best_s:.4f}")
    print(f"FINAL params: {state}")
    return {"params": state, "agg": best_agg, "score": best_s}


def _discover(collection_dir: Path) -> list[tuple[str, Path]]:
    """収集ディレクトリ配下の <dataset_id>/real.lite を列挙。"""
    out = []
    for sub in sorted(collection_dir.iterdir()):
        if sub.is_dir() and resolve_lite_bag(sub, "real") is not None:
            out.append((sub.name, sub))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="マルチデータセット横断 open-loop 集約・ロバスト同定")
    ap.add_argument("--collection-dir", default=str(Path(__file__).parent / "sample" / "multi"))
    ap.add_argument("--cases-config", default=str(Path(__file__).parent / "sample" / "cases.yaml"))
    ap.add_argument(
        "--lite-dir",
        action="append",
        default=[],
        metavar="DATASET_ID=LITE_DIR",
        help="収集を使わず直接指定 (複数可)",
    )
    ap.add_argument("--search", action="store_true", help="ロバスト結合探索を実行")
    ap.add_argument(
        "--out", default="", help="multi_cases_summary.md 出力先 (既定: collection-dir 直下)"
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

    cfg = load_cases_config(args.cases_config)
    md = evaluate_cases(ctxs, cfg)
    out_path = Path(args.out) if args.out else Path(args.collection_dir) / "multi_cases_summary.md"
    out_path.write_text(md + "\n", encoding="utf-8")
    print(f"\n書き出し: {out_path}\n")
    print(md)

    if args.search:
        robust_search(ctxs, cfg)


if __name__ == "__main__":
    main()
