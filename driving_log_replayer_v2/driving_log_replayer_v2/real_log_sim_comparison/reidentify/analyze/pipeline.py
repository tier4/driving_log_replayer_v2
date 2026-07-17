"""残差分析キャンペーンのエントリポイント (make analyze)。

`make reidentify` とは独立に、CSV キャッシュ済みの collection に対して分析ステージを
実行し、成果物を `<ROOT>/reidentify/analysis/` に出力する。

ステージ:
  split       — dataset の dev/holdout 決定的分割を holdout_split.csv に固定出力
  pitch-sign  — コースト区間回帰で pitch 符号規約を検証 (slope フィード実装の前提)
  traces      — per-start 署名付き N-step 終端誤差の抽出 (residuals_nstep.parquet)
  conditioned — A1: 特徴量条件付き残差 + 反実仮想チェック (traces 成果物が前提)
  regime      — 縦レジーム分割評価 (brake/coast/throttle 別の残差指標、traces が前提)
  steady      — A4: 定常 Hammerstein マップ
  oracle      — A2: per-dataset scaling oracle (重い: dataset ごとに ~60 rollout 評価)
  tail        — A2: CVaR テール特性 (metrics.csv + traces から、追加 rollout なし)
  etfe        — A3: cmd→achieved の周波数応答 (1 次+むだ時間の適合性チェック)
  gt          — 加速度 GT の品質診断 (積分整合性×レジーム + ノイズフロア、RTS 候補込み)
  report      — analysis.html の生成 (存在する成果物のみから描画)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import pandas as pd

from ..load_data import discover_cached_datasets
from .conditioned import analyze_conditioned, counterfactual_corrections
from .etfe import analyze_etfe
from .gt_quality import analyze_gt_quality
from .oracle import characterize_tail, fit_per_dataset_oracle
from .pitch_sign import verify_pitch_sign
from .regime import analyze_regimes
from .report_analysis import write_analysis_report
from .residual_traces import extract_residual_traces
from .split import write_split_csv
from .steady_state import analyze_steady_state

ALL_STAGES: tuple[str, ...] = (
    "split", "pitch-sign", "traces", "conditioned", "regime", "steady",
    "oracle", "tail", "etfe", "gt", "report",
)
DEFAULT_CASE = "v2"


def _load_traces(out_dir: Path) -> pd.DataFrame:
    parquet_path = out_dir / "residuals_nstep.parquet"
    csv_path = out_dir / "residuals_nstep.csv.gz"
    if parquet_path.exists():
        return pd.read_parquet(parquet_path)
    if csv_path.exists():
        return pd.read_csv(csv_path)
    raise RuntimeError(
        f"residual traces がありません: {parquet_path} (先に traces ステージを実行)"
    )


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _step_split(out_dir: Path, root: Path) -> None:
    tasks = discover_cached_datasets(root)
    if not tasks:
        raise RuntimeError(f"CSV キャッシュが見つかりません: {root} (先に make reidentify で extract を実行)")
    df = write_split_csv(out_dir / "holdout_split.csv", [ds_id for ds_id, _ in tasks])
    counts = df["split"].value_counts().to_dict()
    print(f"[analyze:split] {len(df)} datasets -> {counts} ({out_dir / 'holdout_split.csv'})")


def _step_pitch_sign(out_dir: Path, root: Path, scenario: Path, case: str, splits: tuple[str, ...]) -> None:
    df, summary = verify_pitch_sign(root, scenario, case_name=case, splits=splits)
    df.to_csv(out_dir / "pitch_sign_check.csv", index=False)
    _write_json(out_dir / "pitch_sign_summary.json", summary)
    print(f"[analyze:pitch-sign] datasets={summary['n_datasets_regressed']}/{summary['n_datasets_with_coast']}")
    if "pooled_slope" in summary:
        print(
            f"[analyze:pitch-sign] pooled: slope={summary['pooled_slope']:+.3f} "
            f"intercept={summary['pooled_intercept']:+.3f} v2_coeff={summary['pooled_v2_coeff']:+.2e} "
            f"r2={summary['pooled_r2']:.3f} (n={summary['pooled_n']})"
        )
    print(
        f"[analyze:pitch-sign] median slope={summary['median_slope']:+.3f} "
        f"positive rate={summary['positive_sign_rate']:.0%}"
    )
    print(f"[analyze:pitch-sign] 判定: {summary['verdict']}")


def _step_traces(
    out_dir: Path, root: Path, scenario: Path, case: str, splits: tuple[str, ...], n_jobs: int,
) -> None:
    traces, metadata = extract_residual_traces(
        root, scenario, case_name=case, n_jobs=n_jobs, splits=splits,
    )
    parquet_path = out_dir / "residuals_nstep.parquet"
    try:
        traces.to_parquet(parquet_path, index=False)
        metadata["output"] = str(parquet_path)
    except (ImportError, ValueError) as e:
        csv_path = out_dir / "residuals_nstep.csv.gz"
        print(f"[WARN] parquet 書き出し失敗 ({e}) — {csv_path} にフォールバック", file=sys.stderr)
        traces.to_csv(csv_path, index=False)
        metadata["output"] = str(csv_path)
    _write_json(out_dir / "residuals_meta.json", metadata)
    print(
        f"[analyze:traces] {metadata['n_rows']} rows / {metadata['n_valid']} datasets "
        f"(horizons={metadata['horizons']}, stride={metadata['stride']}) -> {metadata['output']}"
    )


def _step_conditioned(out_dir: Path) -> None:
    traces = _load_traces(out_dir)
    summary, bins, slopes = analyze_conditioned(traces)
    summary.to_csv(out_dir / "conditioned_summary.csv", index=False)
    bins.to_csv(out_dir / "conditioned_bins.csv", index=False)
    slopes.to_csv(out_dir / "conditioned_slopes.csv", index=False)
    cf = counterfactual_corrections(traces)
    cf.to_csv(out_dir / "counterfactual.csv", index=False)
    n_sig = int(summary["significant"].sum()) if not summary.empty else 0
    print(f"[analyze:conditioned] {len(summary)} (target,feature) 組 / 有意 {n_sig} 件")
    if n_sig:
        for _, row in summary[summary["significant"]].iterrows():
            print(
                f"[analyze:conditioned]   {row['target']}@N={int(row['horizon'])} vs {row['feature']}: "
                f"slope={row['pooled_slope']:+.4g} agree={row['sign_agreement']:.0%} "
                f"effect={row['effect_ratio']:.0%}"
            )
    for _, row in cf.iterrows():
        print(
            f"[analyze:conditioned]   反実仮想 {row['correction']}: {row['target']}@N={int(row['horizon'])} "
            f"RMS {row['rms_before']:.4f} -> {row['rms_after']:.4f} ({row['reduction_pct']:+.1f}%)"
        )


def _step_regime(out_dir: Path) -> None:
    traces = _load_traces(out_dir)
    table = analyze_regimes(traces)
    table.to_csv(out_dir / "regime_metrics.csv", index=False)
    sub = table[(table["target"] == "err_ax") & (table["horizon"] == 30)]
    for _, row in sub.iterrows():
        print(
            f"[analyze:regime] err_ax@N=30 {row['regime']}: RMS={row['rms']:.4f} "
            f"mean={row['mean']:+.4f} [{row['mean_ci_lo']:+.4f}, {row['mean_ci_hi']:+.4f}] "
            f"(n={int(row['n_datasets'])} ds / {int(row['n_rows'])} rows)"
        )


def _step_steady(out_dir: Path, root: Path, scenario: Path, case: str, splits: tuple[str, ...]) -> None:
    curves, summary = analyze_steady_state(root, scenario, case_name=case, splits=splits)
    curves.to_csv(out_dir / "steady_maps.csv", index=False)
    _write_json(out_dir / "steady_summary.json", summary)
    acc = summary["acc"]
    acc_flat = summary["acc_flat_pitch"]
    steer = summary["steer"]
    print(
        f"[analyze:steady] n={summary['n_points']} (flat pitch {summary['n_points_flat_pitch']})\n"
        f"[analyze:steady]   acc gain pos/neg = {acc['gain_pos']:.4f}/{acc['gain_neg']:.4f} "
        f"(flat: {acc_flat['gain_pos']:.4f}/{acc_flat['gain_neg']:.4f})\n"
        f"[analyze:steady]   steer gain pos/neg = {steer['gain_pos']:.4f}/{steer['gain_neg']:.4f}"
    )


def _step_oracle(
    out_dir: Path, root: Path, scenario: Path, case: str, splits: tuple[str, ...], n_jobs: int,
) -> None:
    df, metadata = fit_per_dataset_oracle(
        root, scenario, case_name=case, n_jobs=n_jobs, splits=splits,
    )
    df.to_csv(out_dir / "oracle_per_dataset.csv", index=False)
    _write_json(out_dir / "oracle_meta.json", metadata)
    for metric_key, _param_key in (("steer", ""), ("ax", "")):
        scale = df[f"{metric_key}_scale_opt"]
        gain = 1.0 - df[f"{metric_key}_rmse_opt"].mean() / df[f"{metric_key}_rmse_init"].mean()
        print(
            f"[analyze:oracle] {metric_key}: scale p10/p50/p90 = "
            f"{scale.quantile(0.1):.4f}/{scale.median():.4f}/{scale.quantile(0.9):.4f} "
            f"RMSE 平均改善 {gain:+.1%} (n={len(df)})"
        )


def _step_tail(out_dir: Path, root: Path, case: str, splits: tuple[str, ...]) -> None:
    metrics_csv = root / "reidentify" / "metrics.csv"
    if not metrics_csv.exists():
        raise RuntimeError(f"metrics.csv がありません: {metrics_csv} (先に make reidentify)")
    traces = _load_traces(out_dir)
    tail = characterize_tail(metrics_csv, traces, model=case, splits=splits)
    tail.to_csv(out_dir / "tail_characterization.csv", index=False)
    top = tail[tail["is_tail"]]
    rest = tail[~tail["is_tail"]]
    print(f"[analyze:tail] tail={len(top)}/{len(tail)} datasets (score 寄与上位 10%)")
    for col in ("pitch_lf_abs_mean_deg", "vx_p90", "brake_frac", "steer_rate_abs_p90", "ay_abs_p90"):
        print(
            f"[analyze:tail]   {col}: tail median={top[col].median():.4g} "
            f"vs rest={rest[col].median():.4g}"
        )


def _step_etfe(out_dir: Path, root: Path, scenario: Path, case: str, splits: tuple[str, ...]) -> None:
    df, metadata = analyze_etfe(root, scenario, case_name=case, splits=splits)
    df.to_csv(out_dir / "etfe.csv", index=False)
    _write_json(out_dir / "etfe_meta.json", metadata)
    for channel in ("acc", "steer"):
        sub = df[(df["channel"] == channel) & (df["amplitude_group"] == "all")]
        if sub.empty:
            continue
        usable = sub[sub["coherence2"] >= 0.5]
        f_hi = usable["freq_hz"].max() if not usable.empty else float("nan")
        print(
            f"[analyze:etfe] {channel}: n={int(sub['n_datasets'].iloc[0])} datasets, "
            f"coherence²≥0.5 は f≤{f_hi:.2f} Hz"
        )


def _step_gt(out_dir: Path, root: Path, scenario: Path, splits: tuple[str, ...]) -> None:
    pooled, extra = analyze_gt_quality(root, scenario, splits=splits)
    pooled.to_csv(out_dir / "gt_quality.csv", index=False)
    extra["per_dataset"].to_csv(out_dir / "gt_quality_per_dataset.csv", index=False)
    _write_json(out_dir / "gt_quality_meta.json", extra["metadata"])
    print(f"[analyze:gt] {extra['metadata']['n_datasets']} datasets")
    cols = ["int_err_all_T1", "int_err_brake_T1", "int_err_brake_T0.5", "noise_floor"]
    print(f"[analyze:gt] {'candidate':22s} " + " ".join(f"{c:>18s}" for c in cols))
    for _, row in pooled.sort_values("int_err_brake_T1").iterrows():
        print(
            f"[analyze:gt] {row['candidate']:22s} "
            + " ".join(f"{row[c]:18.4f}" for c in cols)
        )


def _step_report(out_dir: Path) -> None:
    path = write_analysis_report(out_dir)
    print(f"[analyze:report] {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="残差分析キャンペーン (v3 構造仮説の証拠収集)")
    parser.add_argument("--root", required=True, type=Path, help="collection ディレクトリ")
    parser.add_argument("--scenario", required=True, type=Path, help="scenario.yaml")
    parser.add_argument("--case", default=DEFAULT_CASE, help=f"評価する scenario ケース (既定 {DEFAULT_CASE})")
    parser.add_argument("--n-jobs", type=int, default=1)
    parser.add_argument(
        "--stages", default=",".join(ALL_STAGES),
        help=f"実行ステージのカンマ区切り (既定: {','.join(ALL_STAGES)})",
    )
    parser.add_argument(
        "--splits", default="dev",
        help="分析対象 split のカンマ区切り (dev/holdout、既定 dev。split ステージ自体は常に全件)",
    )
    args = parser.parse_args()

    stages = tuple(s.strip() for s in args.stages.split(",") if s.strip())
    unknown = [s for s in stages if s not in ALL_STAGES]
    if unknown:
        parser.error(f"未知のステージ: {unknown} (対応: {ALL_STAGES})")
    splits = tuple(s.strip() for s in args.splits.split(",") if s.strip())
    if any(s not in ("dev", "holdout") for s in splits):
        parser.error(f"--splits は dev/holdout のみ: {splits}")

    out_dir = args.root / "reidentify" / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    if "split" in stages:
        _step_split(out_dir, args.root)
    if "pitch-sign" in stages:
        _step_pitch_sign(out_dir, args.root, args.scenario, args.case, splits)
    if "traces" in stages:
        _step_traces(out_dir, args.root, args.scenario, args.case, splits, args.n_jobs)
    if "conditioned" in stages:
        _step_conditioned(out_dir)
    if "regime" in stages:
        _step_regime(out_dir)
    if "steady" in stages:
        _step_steady(out_dir, args.root, args.scenario, args.case, splits)
    if "oracle" in stages:
        _step_oracle(out_dir, args.root, args.scenario, args.case, splits, args.n_jobs)
    if "tail" in stages:
        _step_tail(out_dir, args.root, args.case, splits)
    if "etfe" in stages:
        _step_etfe(out_dir, args.root, args.scenario, args.case, splits)
    if "gt" in stages:
        _step_gt(out_dir, args.root, args.scenario, splits)
    if "report" in stages:
        _step_report(out_dir)
    print(f"[analyze] 完了: {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
