#!/usr/bin/env python3
"""best_normal (物理) と learned_arx (構造化非線形 ARX) を Optuna で競争的に最適化する。

両モデルとも multi_dataset_tune._score (全 dataset 横断の正規化 mean+worst, N=20/40) を直接
最小化する (同一目的関数 = 対等比較)。さらに「元 best_normal の各誤差スコア (12 指標) の 25%
以下」という凍結ターゲットに対する達成率を毎回報告する。

凍結ターゲット (整合性ガード): 元 best_normal の 12 指標 (N∈{20,40} × {nyaw,nlong,nlat}_{mean,worst})
を一度だけ計算して JSON に固定し、以降の再最適化でも分母を動かさない。target_i = 0.25 × frozen_i。
r = max_i(achieved_i / target_i) が 1 以下なら全指標が 25% 以下を満たす。

learned_arx の構造拡張 (6 係数/ch):
  ax=[a0,a1,a2_thr,a2_brk,a3,a4]  ax[k+1]=a0+a1·ax+a2_thr·max(u,0)+a2_brk·min(u,0)+a3·vx+a4·vx²
  wz=[b0,b1,b2,b3,b4,k_us]        wz[k+1]=b0+b1·wz+(b2·s+b3·vx·s)/(1+k_us·vx²)+b4·vx
探索は well-conditioned 化: tau→AR 係数, g_*→DC ゲイン分離, k_us は understeer。

使い方:
    python3 -m ...arx_optuna --model learned_arx --n-trials 200 --storage sqlite:///... --out ...
    python3 -m ...arx_optuna --model best_normal --n-trials 120 --out ...
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import optuna

from . import multi_dataset_tune as mdt
from .lib._cases_config import load_cases_config

_ARX = "learned_arx"
_PHYS = "delay_steer_acc_geared_wo_fall_guard"
_METRICS = ("nyaw_mean", "nyaw_worst", "nlong_mean", "nlong_worst", "nlat_mean", "nlat_worst")


# --------------------------------------------------------------------------
# learned_arx: trial -> 6+6 係数
# --------------------------------------------------------------------------
def _arx_coeffs(t: optuna.Trial, sub_dt: float) -> tuple[list[float], list[float]]:
    tau_ax = t.suggest_float("tau_ax", 0.03, 2.0, log=True)
    g_thr = t.suggest_float("g_acc_thr", 0.3, 1.5)  # throttle (u>0) DC ゲイン
    g_brk = t.suggest_float("g_acc_brk", 0.3, 1.5)  # brake (u<0) DC ゲイン
    a0 = t.suggest_float("a0", -0.08, 0.08)
    a3 = t.suggest_float("a3", -8e-3, 8e-3)
    a4 = t.suggest_float("a4", -3e-4, 3e-4)
    a1 = math.exp(-sub_dt / tau_ax)
    ax = [a0, a1, g_thr * (1.0 - a1), g_brk * (1.0 - a1), a3, a4]

    tau_wz = t.suggest_float("tau_wz", 0.03, 1.5, log=True)
    g_bike = t.suggest_float("g_bike", 0.05, 0.6)
    k_us = t.suggest_float("k_us_arx", 0.0, 0.05)  # understeer (≥0)
    b2 = t.suggest_float("b2", -0.03, 0.03)
    b0 = t.suggest_float("b0", -2e-3, 2e-3)
    b4 = t.suggest_float("b4", -2e-3, 2e-3)
    b1 = math.exp(-sub_dt / tau_wz)
    wz = [b0, b1, b2, g_bike * (1.0 - b1), b4, k_us]
    return ax, wz


def _arx_seed(ax: list[float], wz: list[float], sub_dt: float) -> dict:
    """cases.yaml の 6 係数 learned_arx を探索空間パラメータへ逆変換 (enqueue 用)。"""
    a0, a1, a2t, a2b, a3, a4 = ax
    b0, b1, b2, b3, b4, k_us = wz
    a1 = min(max(a1, 1e-6), 0.999999)
    b1 = min(max(b1, 1e-6), 0.999999)
    return {
        "tau_ax": min(max(-sub_dt / math.log(a1), 0.03), 2.0),
        "g_acc_thr": min(max(a2t / (1 - a1), 0.3), 1.5),
        "g_acc_brk": min(max(a2b / (1 - a1), 0.3), 1.5),
        "a0": min(max(a0, -0.08), 0.08),
        "a3": min(max(a3, -8e-3), 8e-3),
        "a4": min(max(a4, -3e-4), 3e-4),
        "tau_wz": min(max(-sub_dt / math.log(b1), 0.03), 1.5),
        "g_bike": min(max(b3 / (1 - b1), 0.05), 0.6),
        "k_us_arx": min(max(k_us, 0.0), 0.05),
        "b2": min(max(b2, -0.03), 0.03),
        "b0": min(max(b0, -2e-3), 2e-3),
        "b4": min(max(b4, -2e-3), 2e-3),
    }


def _arx_override(ax: list[float], wz: list[float]) -> dict:
    return {"wheelbase": mdt.WHEELBASE, "arx_ax_coeffs": ax, "arx_wz_coeffs": wz}


# --------------------------------------------------------------------------
# best_normal (物理): trial -> physics params
# --------------------------------------------------------------------------
_PHYS_SPACE = {
    "k_us": (0.0, 0.04),
    "acc_time_constant": (0.05, 1.0),
    "steer_time_constant": (0.05, 0.7),
    "debug_steer_scaling_factor": (0.8, 1.2),
    "debug_acc_scaling_factor": (0.8, 1.2),
    "acc_time_delay": (0.0, 0.5),
    "steer_time_delay": (0.0, 0.4),
}


def _phys_override(t: optuna.Trial) -> dict:
    o = {"wheelbase": mdt.WHEELBASE}
    for name, (lo, hi) in _PHYS_SPACE.items():
        o[name] = t.suggest_float(name, lo, hi)
    return o


def _phys_seed(params: dict) -> dict:
    seed = {}
    for name, (lo, hi) in _PHYS_SPACE.items():
        v = float(params.get(name, (lo + hi) / 2))
        seed[name] = min(max(v, lo), hi)
    return seed


# --------------------------------------------------------------------------
# dynamic_steer_acc_geared (動的自転車派生): trial -> 横パラメータ (縦は best_normal 固定)
# --------------------------------------------------------------------------
_DYN = "dynamic_steer_acc_geared"
# dynamic に最大限のチャンスを与える拡張探索: 横 (剛性域を大幅拡大、高剛性=運動学極限まで) +
# 縦/操舵も開放 (best_normal 固定を解く)。「良いパラメータ組み合わせが存在しないか」の検証用。
_DYN_SPACE = {
    "cf_over_m": (2.0, 2000.0),
    "cr_over_m": (2.0, 2000.0),
    "iz_over_m": (0.1, 20.0),
    "lf": (1.0, 3.7),
    "acc_time_constant": (0.05, 1.0),
    "steer_time_constant": (0.05, 0.7),
    # best_normal との blend: α=1 + k_us で best_normal を厳密再現 (≤best_normal を保証)
    "kinematic_blend": (0.0, 1.0),
    "k_us": (0.0, 0.04),
}
_DYN_LOG = {"cf_over_m", "cr_over_m", "iz_over_m"}
_DYN_FIXED = {"wheelbase": 4.76012, "vx_min_dyn": 2.0}


def _dyn_override(t: optuna.Trial) -> dict:
    o = dict(_DYN_FIXED)
    for name, (lo, hi) in _DYN_SPACE.items():
        o[name] = t.suggest_float(name, lo, hi, log=(name in _DYN_LOG))
    return o


def _dyn_seed(params: dict) -> dict:
    seed = {}
    for name, (lo, hi) in _DYN_SPACE.items():
        v = float(params.get(name, (lo + hi) / 2))
        seed[name] = min(max(v, lo), hi)
    return seed


# --------------------------------------------------------------------------
# 凍結ターゲット
# --------------------------------------------------------------------------
def _frozen_targets(ctxs, cfg, path: Path) -> dict:
    """元 best_normal の 12 指標を一度だけ計算し JSON 固定。target = 0.25×。"""
    if path.exists():
        frozen = json.loads(path.read_text())
    else:
        bn = cfg.find_case("best_normal")
        agg = mdt.aggregate(ctxs, bn.params, bn.vehicle_model_type)
        frozen = {str(h): {m: agg["by_h"][h][m] for m in _METRICS} for h in mdt.HORIZONS}
        path.write_text(json.dumps(frozen, indent=2))
        print(f"[frozen] 元 best_normal 指標を固定: {path}")
    return frozen


def _report_vs_target(agg: dict, frozen: dict) -> tuple[float, str]:
    """達成率 r=max(achieved/target) と内訳文字列。target=0.25×frozen。"""
    lines = ["  metric           frozen  target(25%)  achieved   ratio"]
    worst_r = 0.0
    for h in mdt.HORIZONS:
        for m in _METRICS:
            fr = frozen[str(h)][m]
            tgt = 0.25 * fr
            ach = agg["by_h"][h][m]
            r = ach / tgt if tgt > 0 else float("inf")
            worst_r = max(worst_r, r)
            flag = "OK" if r <= 1.0 else ""
            lines.append(
                f"  N{h:<2d} {m:<12s} {fr:6.3f}   {tgt:6.3f}     {ach:6.3f}   {r:5.2f} {flag}"
            )
    lines.append(f"  => r = max(achieved/target) = {worst_r:.3f}  (全指標 25% 以下なら r<=1)")
    return worst_r, "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description="best_normal / learned_arx 競争的 Optuna 最適化")
    ap.add_argument(
        "--model",
        choices=["learned_arx", "best_normal", "dynamic_steer_acc_geared"],
        default="learned_arx",
    )
    ap.add_argument("--collection-dir", default=str(Path(__file__).parent / "sample" / "multi"))
    ap.add_argument("--scenario", default=str(Path(__file__).parent / "sample" / "scenario.yaml"),
                    help="scenario.yaml のパス (Conditions.models / cases を含む)")
    ap.add_argument("--lite-dir", action="append", default=[], metavar="DATASET_ID=LITE_DIR")
    ap.add_argument("--n-trials", type=int, default=200)
    ap.add_argument("--timeout", type=float, default=0.0)
    ap.add_argument("--max-datasets", type=int, default=0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--storage", default="")
    ap.add_argument("--study-name", default="")
    ap.add_argument(
        "--frozen",
        default=str(Path(__file__).parent / "sample" / "arx_frozen_targets.json"),
        help="元 best_normal の凍結ターゲット JSON (無ければ生成)",
    )
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    if args.lite_dir:
        lite_dirs = [(s.split("=", 1)[0].strip(), Path(s.split("=", 1)[1].strip())) for s in args.lite_dir]
    else:
        lite_dirs = mdt._discover(Path(args.collection_dir))
    if args.max_datasets > 0:
        lite_dirs = lite_dirs[: args.max_datasets]
    if not lite_dirs:
        print(f"ERROR: real.lite が見つかりません: {args.collection_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"datasets ({len(lite_dirs)}): {[d for d, _ in lite_dirs]}")

    ctxs = mdt.load_datasets(lite_dirs)
    if not ctxs:
        print("ERROR: 有効な dataset が 0 件", file=sys.stderr)
        sys.exit(1)
    sub_dt = ctxs[0].base["sub_dt"]

    cfg = load_cases_config(args.scenario)
    frozen = _frozen_targets(ctxs, cfg, Path(args.frozen))

    model = args.model
    if model == _ARX:
        seed_case = cfg.find_case(_ARX)
        seed_ax = [float(v) for v in seed_case.params["arx_ax_coeffs"]]
        seed_wz = [float(v) for v in seed_case.params["arx_wz_coeffs"]]
        seed_override = _arx_override(seed_ax, seed_wz)
        eval_model = _ARX

        def objective(t: optuna.Trial) -> float:
            ax, wz = _arx_coeffs(t, sub_dt)
            return mdt._score(mdt.aggregate(ctxs, _arx_override(ax, wz), _ARX))

        seed_params = _arx_seed(seed_ax, seed_wz, sub_dt)
    elif model == _DYN:
        seed_case = cfg.find_case("dynamic")
        seed_override = dict(seed_case.params)
        eval_model = _DYN

        def objective(t: optuna.Trial) -> float:
            return mdt._score(mdt.aggregate(ctxs, _dyn_override(t), _DYN))

        seed_params = _dyn_seed(seed_case.params)
    else:
        seed_case = cfg.find_case("best_normal")
        seed_override = dict(seed_case.params)
        eval_model = _PHYS

        def objective(t: optuna.Trial) -> float:
            return mdt._score(mdt.aggregate(ctxs, _phys_override(t), _PHYS))

        seed_params = _phys_seed(seed_case.params)

    seed_score = mdt._score(mdt.aggregate(ctxs, seed_override, eval_model))
    print(f"\n[ref] {model} seed _score = {seed_score:.4f}")

    sampler = optuna.samplers.TPESampler(seed=args.seed, multivariate=True)
    study = optuna.create_study(
        direction="minimize",
        sampler=sampler,
        storage=args.storage or None,
        study_name=args.study_name or f"opt_{model}",
        load_if_exists=bool(args.storage),
    )
    if not study.get_trials(deepcopy=False):
        study.enqueue_trial(seed_params)
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    def _cb(st: optuna.Study, tr: optuna.trial.FrozenTrial) -> None:
        if tr.number % 10 == 0 or (tr.value is not None and tr.value <= st.best_value + 1e-9):
            print(f"  trial {tr.number:3d}: score={tr.value:.4f}  best={st.best_value:.4f}")

    study.optimize(
        objective,
        n_trials=args.n_trials,
        timeout=args.timeout if args.timeout > 0 else None,
        callbacks=[_cb],
    )

    best = study.best_trial
    if model == _ARX:
        ax, wz = _arx_coeffs(_Fixed(best.params), sub_dt)
        best_override = _arx_override(ax, wz)
        out_yaml = (
            "arx_ax_coeffs: [" + ", ".join(f"{c:.8g}" for c in ax) + "]\n"
            "arx_wz_coeffs: [" + ", ".join(f"{c:.8g}" for c in wz) + "]\n"
        )
        coeff_desc = f"  ax: {ax}\n  wz: {wz}"
    elif model == _DYN:
        best_override = dict(_DYN_FIXED)
        for k in _DYN_SPACE:
            best_override[k] = best.params[k]
        out_yaml = "".join(f"{k}: {best.params[k]:.6g}\n" for k in _DYN_SPACE)
        coeff_desc = "  " + "  ".join(f"{k}={best.params[k]:.5g}" for k in _DYN_SPACE)
    else:
        best_override = {"wheelbase": mdt.WHEELBASE, **{k: best.params[k] for k in _PHYS_SPACE}}
        out_yaml = "".join(f"{k}: {best.params[k]:.6g}\n" for k in _PHYS_SPACE)
        coeff_desc = "  " + "  ".join(f"{k}={best.params[k]:.5g}" for k in _PHYS_SPACE)

    best_agg = mdt.aggregate(ctxs, best_override, eval_model)
    worst_r, report = _report_vs_target(best_agg, frozen)

    print(f"\n## {model} Optuna 最良 (N-step _score 最小)")
    print(f"  best _score = {study.best_value:.4f}  (seed {seed_score:.4f})")
    print(mdt._fmt_agg(f"best_{model[:8]}", best_agg))
    print(coeff_desc)
    print("\n## 元 best_normal の 25% ターゲットに対する達成状況")
    print(report)
    print("\n## 貼り付け用")
    print(out_yaml)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(out_yaml, encoding="utf-8")
        print(f"書き出し: {args.out}")
    if worst_r <= 1.0:
        print("\n[GOAL] 全 12 指標が元 best_normal の 25% 以下を満たしました。")
    else:
        print(f"\n[STATUS] 未達: r={worst_r:.2f} (>1)。最も遠い指標が target の {worst_r:.1f}x。")


class _Fixed:
    """best.params から係数を再構成する suggest_float 互換シム。"""

    def __init__(self, params: dict):
        self._p = params

    def suggest_float(self, name: str, *_a, **_k) -> float:
        return self._p[name]


if __name__ == "__main__":
    main()
