"""analysis.html — 残差分析キャンペーンの成果物を 1 枚の HTML にまとめる。

判定しきい値 (事前登録) を冒頭に固定表示し、探索的発見と検証済み発見を区別する。
report.html (reidentify 本体) とは独立で、analysis ディレクトリの成果物のみから描画する。
"""
from __future__ import annotations

import datetime
import html
import json
from pathlib import Path

import numpy as np
import pandas as pd

from .conditioned import (
    BH_ALPHA,
    EFFECT_RATIO_THRESHOLD,
    SIGN_AGREEMENT_THRESHOLD,
)

_CSS = """
body { font-family: sans-serif; margin: 2em; max-width: 1200px; }
h1 { border-bottom: 2px solid #333; padding-bottom: 0.2em; }
h2 { border-bottom: 1px solid #999; padding-bottom: 0.15em; margin-top: 2em; }
table { border-collapse: collapse; margin: 1em 0; font-size: 0.9em; }
th, td { border: 1px solid #ccc; padding: 0.3em 0.6em; text-align: right; }
th { background: #f0f0f0; }
td.text, th.text { text-align: left; }
tr.sig { background: #e6f4e6; }
.note { color: #555; font-size: 0.9em; }
.verdict { font-weight: bold; }
svg { background: #fff; border: 1px solid #ddd; margin: 0.5em 1em 0.5em 0; }
"""


def _fmt(value: float, digits: int = 3) -> str:
    if value is None or (isinstance(value, float) and not np.isfinite(value)):
        return "—"
    return f"{value:.{digits}g}"


def _table(df: pd.DataFrame, *, digits: int = 3, sig_col: str | None = None) -> str:
    if df.empty:
        return "<p class='note'>データなし</p>"
    head = "".join(
        f"<th class='text'>{html.escape(str(c))}</th>" if df[c].dtype == object else f"<th>{html.escape(str(c))}</th>"
        for c in df.columns
    )
    rows = []
    for _, row in df.iterrows():
        cls = " class='sig'" if sig_col and bool(row.get(sig_col)) else ""
        cells = []
        for c in df.columns:
            v = row[c]
            if isinstance(v, (float, np.floating)):
                cells.append(f"<td>{_fmt(float(v), digits)}</td>")
            elif isinstance(v, (bool, np.bool_)):
                cells.append(f"<td>{'✔' if v else ''}</td>")
            elif isinstance(v, (int, np.integer)):
                cells.append(f"<td>{int(v)}</td>")
            else:
                cells.append(f"<td class='text'>{html.escape(str(v))}</td>")
        rows.append(f"<tr{cls}>{''.join(cells)}</tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(rows)}</tbody></table>"


def _line_svg(
    series: list[tuple[str, np.ndarray, np.ndarray]],
    *,
    title: str,
    width: int = 420,
    height: int = 260,
    band: tuple[np.ndarray, np.ndarray] | None = None,
) -> str:
    """複数 series の簡易折れ線 SVG (最初の series に band=(lo,hi) を重ねられる)。"""
    pad = 45
    xs = np.concatenate([s[1] for s in series])
    ys_all = [s[2] for s in series]
    if band is not None:
        ys_all += [band[0], band[1]]
    ys = np.concatenate(ys_all)
    if len(xs) == 0:
        return ""
    x_lo, x_hi = float(np.min(xs)), float(np.max(xs))
    y_lo, y_hi = float(np.min(ys)), float(np.max(ys))
    y_lo = min(y_lo, 0.0)
    y_hi = max(y_hi, 0.0)
    if x_hi == x_lo:
        x_hi = x_lo + 1.0
    if y_hi == y_lo:
        y_hi = y_lo + 1.0

    def sx(v: np.ndarray) -> np.ndarray:
        return pad + (np.asarray(v) - x_lo) / (x_hi - x_lo) * (width - 2 * pad)

    def sy(v: np.ndarray) -> np.ndarray:
        return height - pad - (np.asarray(v) - y_lo) / (y_hi - y_lo) * (height - 2 * pad)

    colors = ("#1f77b4", "#d62728", "#2ca02c")
    parts = [f"<svg width='{width}' height='{height}' xmlns='http://www.w3.org/2000/svg'>"]
    parts.append(f"<text x='{width / 2}' y='16' text-anchor='middle' font-size='12'>{html.escape(title)}</text>")
    zero_y = float(sy(np.array([0.0]))[0])
    parts.append(f"<line x1='{pad}' y1='{zero_y}' x2='{width - pad}' y2='{zero_y}' stroke='#999' stroke-dasharray='3,3'/>")
    if band is not None:
        x0, lo, hi = series[0][1], band[0], band[1]
        px = np.concatenate([sx(x0), sx(x0)[::-1]])
        py = np.concatenate([sy(lo), sy(hi)[::-1]])
        pts = " ".join(f"{a:.1f},{b:.1f}" for a, b in zip(px, py))
        parts.append(f"<polygon points='{pts}' fill='#1f77b4' opacity='0.15'/>")
    for (label, x, y), color in zip(series, colors):
        pts = " ".join(f"{a:.1f},{b:.1f}" for a, b in zip(sx(x), sy(y)))
        parts.append(f"<polyline points='{pts}' fill='none' stroke='{color}' stroke-width='1.5'/>")
        for a, b in zip(sx(x), sy(y)):
            parts.append(f"<circle cx='{a:.1f}' cy='{b:.1f}' r='2' fill='{color}'/>")
    parts.append(
        f"<text x='{pad}' y='{height - 8}' font-size='10'>{_fmt(x_lo)}</text>"
        f"<text x='{width - pad}' y='{height - 8}' text-anchor='end' font-size='10'>{_fmt(x_hi)}</text>"
        f"<text x='4' y='{height - pad}' font-size='10'>{_fmt(y_lo)}</text>"
        f"<text x='4' y='{pad}' font-size='10'>{_fmt(y_hi)}</text>"
    )
    if len(series) > 1:
        for i, ((label, _x, _y), color) in enumerate(zip(series, colors)):
            parts.append(
                f"<rect x='{pad + 5}' y='{22 + i * 14}' width='10' height='3' fill='{color}'/>"
                f"<text x='{pad + 20}' y='{28 + i * 14}' font-size='10'>{html.escape(label)}</text>"
            )
    parts.append("</svg>")
    return "".join(parts)


def _load_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _load_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame()
    return pd.read_csv(path)


def _candidate_list(
    summary: pd.DataFrame,
    counterfactual: pd.DataFrame,
    steady_summary: dict | None,
    oracle: pd.DataFrame,
    regime: pd.DataFrame,
    tail: pd.DataFrame,
) -> pd.DataFrame:
    """成果物から v3 構造候補の証拠サマリ表を組み立てる (順位は証拠の強さの推奨順)。"""
    def _cond(target: str, feature: str, col: str) -> float:
        sel = summary[(summary["target"] == target) & (summary["feature"] == feature)]
        return float(sel.iloc[0][col]) if not sel.empty else float("nan")

    def _cf(prefix: str) -> float:
        sel = counterfactual[counterfactual["correction"].str.startswith(prefix)]
        return float(sel.iloc[0]["reduction_pct"]) if not sel.empty else float("nan")

    rows = []
    ev_brake = []
    if steady_summary:
        acc = steady_summary.get("acc_flat_pitch", steady_summary.get("acc", {}))
        ev_brake.append(f"定常ゲイン pos/neg={_fmt(acc.get('gain_pos'))}/{_fmt(acc.get('gain_neg'))}")
    if not regime.empty:
        b = regime[(regime["target"] == "err_ax") & (regime["horizon"] == 30) & (regime["regime"] == "brake")]
        c = regime[(regime["target"] == "err_ax") & (regime["horizon"] == 30) & (regime["regime"] == "coast")]
        if not b.empty and not c.empty:
            ev_brake.append(
                f"brake RMS {_fmt(b.iloc[0]['rms'])} (coast {_fmt(c.iloc[0]['rms'])}) "
                f"bias {_fmt(b.iloc[0]['mean'])}"
            )
    rows.append({
        "rank": 1,
        "candidate": "brake/throttle 分割 (brake_scaling_factor, brake_time_constant)",
        "channel": "ax",
        "evidence": "; ".join(ev_brake) or "steady/regime 未実行",
        "counterfactual_pct": _cf("brake_throttle_asym"),
        "implementation": "C++ 構造項 (B-M2)",
    })
    rows.append({
        "rank": 2,
        "candidate": "steer 応答再同定 (τ/delay を N-step 目的で、v2_t の単一チャネル版)",
        "channel": "steer/yaw",
        "evidence": (
            f"err_steer vs steer_rate: slope={_fmt(_cond('err_steer_deg', 'steer_rate_mean', 'pooled_slope'))} "
            f"effect={_fmt(_cond('err_steer_deg', 'steer_rate_mean', 'effect_ratio'))} "
            "(実効ラグ ~80ms 分モデルが遅い)"
        ),
        "counterfactual_pct": float("nan"),
        "implementation": "パラメータのみ (C++ 不要、単一チャネルスイープ + ゲート)",
    })
    rows.append({
        "rank": 3,
        "candidate": "SLOPE_ACCX フィード (pitch 由来勾配の給電)",
        "channel": "ax",
        "evidence": (
            f"err_ax vs g·sin(pitch_lf): slope={_fmt(_cond('err_ax', 'g_sin_pitch_lf', 'pooled_slope'))} "
            f"agree={_fmt(_cond('err_ax', 'g_sin_pitch_lf', 'sign_agreement'))}"
        ),
        "counterfactual_pct": _cf("slope_feed_beta_fit"),
        "implementation": "wrapper 入力経路 (B-M1、モデル方程式は既対応)",
    })
    rows.append({
        "rank": 4,
        "candidate": "走行抵抗 c0 + c2·v²",
        "channel": "ax",
        "evidence": f"err_ax vs vx: effect={_fmt(_cond('err_ax', 'vx_mean', 'effect_ratio'))}",
        "counterfactual_pct": _cf("drag_c0_c2"),
        "implementation": "C++ 構造項 (B-M2)",
    })
    ev_lat = [
        f"err_lat vs vx: effect={_fmt(_cond('err_lat_cm', 'vx_mean', 'effect_ratio'))}",
    ]
    if not tail.empty and "ay_abs_p90" in tail.columns:
        t = tail[tail["is_tail"]]["ay_abs_p90"].median()
        r = tail[~tail["is_tail"]]["ay_abs_p90"].median()
        ev_lat.append(f"CVaR テールは高 ay (p90 中央値 {_fmt(t)} vs {_fmt(r)})")
    rows.append({
        "rank": 5,
        "candidate": "横系構造 (速度依存 steer ゲイン / k_us 再検討)",
        "channel": "lat/yaw",
        "evidence": "; ".join(ev_lat),
        "counterfactual_pct": float("nan"),
        "implementation": "要追加分析 (v4 候補含む)",
    })
    if not oracle.empty:
        rows.append({
            "rank": 6,
            "candidate": "(参考) per-dataset 異質性の限界",
            "channel": "ax/steer",
            "evidence": (
                f"scaling oracle: ax p10/p50/p90={_fmt(oracle['ax_scale_opt'].quantile(0.1))}/"
                f"{_fmt(oracle['ax_scale_opt'].median())}/{_fmt(oracle['ax_scale_opt'].quantile(0.9))}, "
                f"oracle 後も ax RMSE {_fmt(oracle['ax_rmse_opt'].mean())} 残存 (非 scaling 構造)"
            ),
            "counterfactual_pct": float("nan"),
            "implementation": "—",
        })
    return pd.DataFrame(rows)


def render_analysis_report(analysis_dir: Path) -> str:
    """analysis ディレクトリの成果物から analysis.html の中身を組み立てる。"""
    pitch = _load_json(analysis_dir / "pitch_sign_summary.json")
    residuals_meta = _load_json(analysis_dir / "residuals_meta.json")
    summary = _load_csv(analysis_dir / "conditioned_summary.csv")
    bins = _load_csv(analysis_dir / "conditioned_bins.csv")
    counterfactual = _load_csv(analysis_dir / "counterfactual.csv")
    steady_curves = _load_csv(analysis_dir / "steady_maps.csv")
    steady_summary = _load_json(analysis_dir / "steady_summary.json")
    split_df = _load_csv(analysis_dir / "holdout_split.csv")
    regime = _load_csv(analysis_dir / "regime_metrics.csv")
    oracle = _load_csv(analysis_dir / "oracle_per_dataset.csv")
    tail = _load_csv(analysis_dir / "tail_characterization.csv")
    etfe = _load_csv(analysis_dir / "etfe.csv")
    etfe_meta = _load_json(analysis_dir / "etfe_meta.json")

    parts = [f"<style>{_CSS}</style>"]
    parts.append("<h1>残差分析キャンペーン (v3 構造仮説)</h1>")
    parts.append(
        f"<p class='note'>生成: {datetime.datetime.now().isoformat(timespec='seconds')}"
        + (f" / traces: case={residuals_meta.get('case')}, {residuals_meta.get('n_valid')} datasets, "
           f"{residuals_meta.get('n_rows')} rows, splits={residuals_meta.get('splits')}" if residuals_meta else "")
        + "</p>"
    )

    # 事前登録しきい値。
    parts.append("<h2>1. 判定基準 (事前登録)</h2>")
    parts.append(
        "<ul>"
        f"<li>符号一致率 ≥ {SIGN_AGREEMENT_THRESHOLD:.0%} かつ二項検定 BH 補正 p &lt; {BH_ALPHA}</li>"
        f"<li>効果量: 説明 std / 残差 RMS ≥ {EFFECT_RATIO_THRESHOLD:.0%}</li>"
        "<li>集計は dataset をクラスタ単位とする 2 段集計 (bin 内 per-dataset 平均 → 横断 mean + bootstrap 95%CI)</li>"
        "<li>候補の最終採否は N-step robust_score の非劣化ゲート (本レポートは候補化まで)</li>"
        "</ul>"
    )

    if split_df is not None and not split_df.empty:
        counts = split_df["split"].value_counts().to_dict()
        parts.append(f"<p class='note'>dev/holdout 分割: {counts} (分析は dev のみ)</p>")

    # pitch 符号検証。
    parts.append("<h2>2. pitch 符号規約の検証 (コースト回帰)</h2>")
    if pitch:
        parts.append(f"<p class='verdict'>判定: {html.escape(str(pitch.get('verdict')))}</p>")
        keys = [
            "n_datasets_with_coast", "n_datasets_regressed", "median_slope",
            "positive_sign_rate", "pooled_slope", "pooled_intercept", "pooled_v2_coeff",
            "pooled_r2", "pooled_n",
        ]
        rows = [{"key": k, "value": pitch[k]} for k in keys if k in pitch]
        parts.append(_table(pd.DataFrame(rows), digits=4))
        parts.append(
            "<p class='note'>注: コースト中は低レベル縦制御が勾配を部分補償しうるため、"
            "係数 &lt; 1 は「補償あり」か「pitch ノイズによる減衰」のいずれか。"
            "slope フィードの最終判定は §4 の N-step 反実仮想による。</p>"
        )
    else:
        parts.append("<p class='note'>pitch_sign_summary.json なし (make analyze ANALYZE_STAGES=pitch-sign)</p>")

    # A1 条件付き残差。
    parts.append("<h2>3. A1: 特徴量条件付き残差 (2 段集計)</h2>")
    if not summary.empty:
        parts.append(_table(summary, digits=3, sig_col="significant"))
        sig = summary[summary["significant"] == True]  # noqa: E712
        if not bins.empty and not sig.empty:
            parts.append("<h3>有意ペアの bin 平均 (帯 = dataset 横断 bootstrap 95%CI)</h3>")
            for _, row in sig.iterrows():
                sel = bins[
                    (bins["target"] == row["target"])
                    & (bins["feature"] == row["feature"])
                    & (bins["horizon"] == row["horizon"])
                ].sort_values("x_center")
                if sel.empty:
                    continue
                parts.append(_line_svg(
                    [(f"{row['target']}@N={int(row['horizon'])}",
                      sel["x_center"].to_numpy(), sel["mean"].to_numpy())],
                    title=f"{row['target']}@N={int(row['horizon'])} vs {row['feature']}",
                    band=(sel["ci_lo"].to_numpy(), sel["ci_hi"].to_numpy()),
                ))
    else:
        parts.append("<p class='note'>conditioned_summary.csv なし (make analyze ANALYZE_STAGES=conditioned)</p>")

    # 反実仮想。
    parts.append("<h2>4. 反実仮想チェック (補正注入による N-step RMS 低下予測)</h2>")
    if not counterfactual.empty:
        parts.append(_table(counterfactual, digits=4))
        parts.append(
            "<p class='note'>drag / brake_throttle_asym は slope 補正後の残差に対する追加低下 (累積)。"
            "v2_t の教訓により、これは N-step 署名付き残差への注入であり 1-step 直接同定ではない。</p>"
        )
    else:
        parts.append("<p class='note'>counterfactual.csv なし</p>")

    # A4 定常マップ。
    parts.append("<h2>5. A4: 定常 Hammerstein マップ</h2>")
    if not steady_curves.empty:
        acc_all = steady_curves[steady_curves["channel"] == "acc_all"].sort_values("x_center")
        acc_flat = steady_curves[steady_curves["channel"] == "acc_flat_pitch"].sort_values("x_center")
        steer_all = steady_curves[steady_curves["channel"] == "steer_all"].sort_values("x_center")
        if not acc_all.empty:
            series = [("all", acc_all["x_center"].to_numpy(),
                       (acc_all["y_median"] - acc_all["x_center"]).to_numpy())]
            if not acc_flat.empty:
                series.append(("|pitch|<0.5deg", acc_flat["x_center"].to_numpy(),
                               (acc_flat["y_median"] - acc_flat["x_center"]).to_numpy()))
            parts.append(_line_svg(series, title="acc: median(a_act) - a_cmd vs a_cmd"))
        if not steer_all.empty:
            parts.append(_line_svg(
                [("all", steer_all["x_center"].to_numpy(),
                  (steer_all["y_median"] - steer_all["x_center"]).to_numpy())],
                title="steer: median(d_act) - d_cmd vs d_cmd",
            ))
        if steady_summary:
            rows = []
            for ch in ("acc", "acc_flat_pitch", "steer"):
                g = steady_summary.get(ch, {})
                rows.append({
                    "channel": ch,
                    "gain_pos": g.get("gain_pos"), "n_pos": g.get("n_pos"),
                    "gain_neg": g.get("gain_neg"), "n_neg": g.get("n_neg"),
                })
            parts.append(_table(pd.DataFrame(rows), digits=4))
            parts.append(
                "<p class='note'>gain_pos/neg = 指令領域の原点通過ゲイン中央値。"
                "acc と acc_flat_pitch の差が小さければ scaling 0.9 クランプは勾配交絡由来ではない。"
                "pos/neg の差はブレーキ/スロットル非対称の直接証拠。</p>"
            )
    else:
        parts.append("<p class='note'>steady_maps.csv なし (make analyze ANALYZE_STAGES=steady)</p>")

    # レジーム分割評価。
    parts.append("<h2>6. 縦レジーム分割評価 (brake / coast / throttle)</h2>")
    if not regime.empty:
        parts.append(_table(regime, digits=4))
        parts.append(
            "<p class='note'>減速領域は別ダイナミクスの可能性 (定常ゲイン非対称) の第一歩として"
            "評価のみをレジーム分割したもの。brake の RMS/バイアスが coast/throttle より系統的に"
            "大きければ、モデル分割 (符号別ゲイン・τ) の直接の根拠になる。</p>"
        )
    else:
        parts.append("<p class='note'>regime_metrics.csv なし (make analyze ANALYZE_STAGES=regime)</p>")

    # oracle。
    parts.append("<h2>7. A2: per-dataset scaling oracle</h2>")
    if not oracle.empty:
        rows = []
        for ch in ("steer", "ax"):
            scale = oracle[f"{ch}_scale_opt"]
            rows.append({
                "channel": ch,
                "scale_p10": scale.quantile(0.1),
                "scale_p50": scale.median(),
                "scale_p90": scale.quantile(0.9),
                "rmse_init_mean": oracle[f"{ch}_rmse_init"].mean(),
                "rmse_opt_mean": oracle[f"{ch}_rmse_opt"].mean(),
                "n": len(oracle),
            })
        parts.append(_table(pd.DataFrame(rows), digits=4))
        parts.append(
            "<p class='note'>探索域は分析専用 (0.5–1.5)。per-dataset 最適化後も残る RMSE が"
            "「scaling では説明できない構造/ノイズ下限」。scale 分布と条件 (brake_frac・平均勾配)"
            " の相関は candidate list の根拠欄を参照。</p>"
        )
    else:
        parts.append("<p class='note'>oracle_per_dataset.csv なし (make analyze ANALYZE_STAGES=oracle)</p>")

    # テール特性。
    parts.append("<h2>8. A2: CVaR テール特性 (worst 10% dataset)</h2>")
    if not tail.empty:
        cond_cols = [
            "dataset_id", "score_contribution", "pitch_lf_abs_mean_deg", "vx_p50", "vx_p90",
            "brake_frac", "accel_frac", "steer_rate_abs_p90", "ay_abs_p90",
        ]
        cols = [c for c in cond_cols if c in tail.columns]
        parts.append("<h3>テール vs 非テールの条件中央値</h3>")
        med_rows = []
        for label, sub in (("tail", tail[tail["is_tail"]]), ("rest", tail[~tail["is_tail"]])):
            med_rows.append({"group": label, "n": len(sub),
                             **{c: sub[c].median() for c in cols if c not in ("dataset_id",)}})
        parts.append(_table(pd.DataFrame(med_rows), digits=4))
        parts.append("<h3>テール上位 10 dataset</h3>")
        parts.append(_table(tail[cols].head(10), digits=4))
    else:
        parts.append("<p class='note'>tail_characterization.csv なし (make analyze ANALYZE_STAGES=tail)</p>")

    # ETFE。
    parts.append("<h2>9. A3: ETFE 周波数応答 (適合性チェック限定)</h2>")
    if not etfe.empty and etfe_meta:
        from .etfe import model_response

        for ch in ("acc", "steer"):
            sub = etfe[(etfe["channel"] == ch) & (etfe["amplitude_group"] == "all")].sort_values("freq_hz")
            if sub.empty:
                continue
            usable = sub[sub["coherence2"] >= 0.5]
            mp = etfe_meta["model_response"][ch]
            gain_m, phase_m = model_response(sub["freq_hz"].to_numpy(), mp["k"], mp["tau"], mp["delay"])
            parts.append(_line_svg(
                [("ETFE", sub["freq_hz"].to_numpy(), sub["gain"].to_numpy()),
                 ("v2 model", sub["freq_hz"].to_numpy(), gain_m)],
                title=f"{ch}: |G(f)| (coherence²≥0.5 は f≤{usable['freq_hz'].max() if not usable.empty else float('nan'):.2f} Hz)",
            ))
            parts.append(_line_svg(
                [("ETFE", sub["freq_hz"].to_numpy(), sub["phase_deg"].to_numpy()),
                 ("v2 model", sub["freq_hz"].to_numpy(), phase_m)],
                title=f"{ch}: phase(f) [deg]",
            ))
            parts.append(_line_svg(
                [("coherence²", sub["freq_hz"].to_numpy(), sub["coherence2"].to_numpy())],
                title=f"{ch}: coherence²",
            ))
        parts.append(
            "<p class='note'>閉ループ運転のため低域はバイアスあり — 同定には使わない (事前登録)。"
            "coherence² が低い帯域の ETFE は解釈しない。acc は coherence 不足のため参考値。</p>"
        )
    else:
        parts.append("<p class='note'>etfe.csv なし (make analyze ANALYZE_STAGES=etfe)</p>")

    # 候補リスト。
    parts.append("<h2>10. v3 構造候補リスト (証拠付き・推奨順)</h2>")
    candidates = _candidate_list(summary, counterfactual, steady_summary, oracle, regime, tail)
    parts.append(_table(candidates, digits=3))
    parts.append(
        "<p class='note'>counterfactual_pct は N-step 残差への補正注入で予測した RMS 低下"
        " (drag/brake は slope 補正後の累積)。最終採否は全 318 dataset + holdout の"
        " robust_score 非劣化ゲート (Phase C) で決める。</p>"
    )
    return "".join(parts)


def write_analysis_report(analysis_dir: Path) -> Path:
    out = analysis_dir / "analysis.html"
    body = render_analysis_report(analysis_dir)
    out.write_text(
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>residual analysis</title></head><body>"
        + body + "</body></html>",
        encoding="utf-8",
    )
    return out
