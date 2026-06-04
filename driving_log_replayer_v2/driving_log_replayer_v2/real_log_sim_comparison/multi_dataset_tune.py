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
from .lib._nstep_common import rmse_by_horizon

H = 20  # 評価する最大 horizon (step7 sweep と同じ最大 horizon)
STRIDE = 5
WHEELBASE = 4.76012
# per-dataset 正規化の分母フロア。ほぼ直進・低ダイナミクス走行は baseline 誤差が極小で、
# 相対誤差 (err/baseline) が暴発し worst-case を支配する (絶対値は微小なのに)。分母をフロアで
# 下限クリップし、ステア/縦パラメータを弁別できる十分な信号を持つ走行のみを実質的に重み付けする。
YAW_FLOOR = 0.12   # deg @N20 (これ未満の baseline yaw は弁別力が乏しい)
POS_FLOOR = 3.0    # cm  @N20
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
    base_metric: dict  # baseline (無補正) の {yaw, pos, long, lat} @N20


def _eval(ctx: DatasetCtx, override: dict, model_type: str) -> dict:
    """1 dataset・1 パラメータ組の N=20 終端誤差 RMSE {yaw, pos, long, lat, steer}。"""
    params = dict(ctx.base)
    params.update(override)
    key = tuple(round(float(params[k]), 9) for k in _GT_KEYS)
    gt = ctx.gt_cache.get(key)
    if gt is None:
        gt = s5._prepare_gt(ctx.data, ctx.t0_ns, params)
        ctx.gt_cache[key] = gt
    df = s5.run_rollout(
        ctx.data, ctx.t0_ns, params, model_type, horizons=(H,), stride=STRIDE, gt=gt
    )
    return rmse_by_horizon(df)[H]


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
            ctx.base_metric = _eval(ctx, {}, _BASELINE_MODEL)  # 正規化基準
            if not (ctx.base_metric["yaw"] > 0 and ctx.base_metric["pos"] > 0):
                print(f"[SKIP] {ds_id}: baseline 誤差が無効 (yaw/pos≤0 or NaN)", file=sys.stderr)
                continue
        except Exception as e:  # noqa: BLE001
            print(f"[SKIP] {ds_id}: ロード失敗 ({type(e).__name__}: {e})", file=sys.stderr)
            continue
        ctxs.append(ctx)
        print(
            f"[load] {ds_id}: baseline yaw@N{H}={ctx.base_metric['yaw']:.4f} "
            f"pos@N{H}={ctx.base_metric['pos']:.3f}"
        )
    return ctxs


def aggregate(ctxs: list[DatasetCtx], override: dict, model_type: str) -> dict:
    """
    Dataset 横断で per-dataset 正規化した yaw/pos の mean と worst(max) を返す。

    per_ds: [{dataset_id, yaw, pos, nyaw, npos}], nyaw/npos = baseline 比。
    """
    per_ds = []
    for ctx in ctxs:
        m = _eval(ctx, override, model_type)
        # 分母をフロアで下限クリップ (低ダイナミクス走行の相対誤差暴発を防ぐ)
        nyaw = m["yaw"] / max(ctx.base_metric["yaw"], YAW_FLOOR)
        npos = m["pos"] / max(ctx.base_metric["pos"], POS_FLOOR)
        per_ds.append(
            {
                "dataset_id": ctx.dataset_id,
                "yaw": m["yaw"],
                "pos": m["pos"],
                "long": m["long"],
                "lat": m["lat"],
                "nyaw": nyaw,
                "npos": npos,
            }
        )
    nyaws = [d["nyaw"] for d in per_ds]
    nposs = [d["npos"] for d in per_ds]
    return {
        "per_ds": per_ds,
        "nyaw_mean": stats.mean(nyaws),
        "nyaw_worst": max(nyaws),
        "npos_mean": stats.mean(nposs),
        "npos_worst": max(nposs),
    }


_WORST_W = 0.5  # worst-case 項の重み (ユーザー方針: mean+worst 両方を balance)


def _score(agg: dict) -> float:
    """ロバスト目的関数: 正規化 mean + worst (yaw+pos)。小さいほど良い。

    mean だけだと pos-mean を稼ぐ acc_time_constant 等の proxy が特定エリアの worst を
    悪化させても採用されてしまう。worst を重み付きで加えて mean と worst を両立させる。
    """
    return (agg["nyaw_mean"] + agg["npos_mean"]
            + _WORST_W * (agg["nyaw_worst"] + agg["npos_worst"]))


def _fmt_agg(tag: str, agg: dict) -> str:
    parts = " ".join(
        f"{d['dataset_id'][:8]}(y{d['yaw']:.3f}/p{d['pos']:.2f})" for d in agg["per_ds"]
    )
    return (
        f"{tag:14s} nyaw_mean={agg['nyaw_mean']:.3f} nyaw_worst={agg['nyaw_worst']:.3f} "
        f"npos_mean={agg['npos_mean']:.3f} npos_worst={agg['npos_worst']:.3f} | {parts}"
    )


_KUS0020 = {"k_us": 0.020}


def evaluate_cases(ctxs: list[DatasetCtx], cfg) -> str:
    """cases.yaml の全ケースを dataset 横断評価し Markdown 表を返す。"""
    lines = ["# multi-dataset cases summary (open-loop N=%d 終端誤差)" % H, ""]
    lines.append(
        "各 dataset の baseline 誤差で正規化した値で集約 (n* = baseline 比、小さいほど良い)。"
    )
    lines.append("")
    header = (
        "| case | model | "
        + " | ".join(f"{c.dataset_id[:8]} yaw/pos" for c in ctxs)
        + " | nyaw_mean | nyaw_worst | npos_mean | npos_worst |"
    )
    sep = "|" + "---|" * (2 + len(ctxs) + 4)
    lines += [header, sep]
    for tag in cfg.tags:
        case = cfg.find_case(tag)
        agg = aggregate(ctxs, case.params, case.vehicle_model)
        cells = " | ".join(f"{d['yaw']:.3f}/{d['pos']:.2f}" for d in agg["per_ds"])
        lines.append(
            f"| {tag} | {case.vehicle_model.replace('delay_steer_acc_geared_wo_fall_guard', 'delay')} "
            f"| {cells} | {agg['nyaw_mean']:.3f} | {agg['nyaw_worst']:.3f} "
            f"| {agg['npos_mean']:.3f} | {agg['npos_worst']:.3f} |"
        )
    lines.append("")
    return "\n".join(lines)


def robust_search(ctxs: list[DatasetCtx], cfg) -> dict:
    """
    best_normal 集合 (代理含む) を coordinate descent で dataset 横断ロバスト最適化。

    目的: 正規化 mean + worst (yaw+pos) を最小化 (_score; worst を重み付きで含む)。
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
