#!/usr/bin/env python3
"""
物理的妥当性レポート生成スクリプト

速度依存アンダーステア係数ランプと操舵不感帯の物理的妥当性を、
実機ログからの独立同定と理論式の両面から検証する HTML レポートを生成する。

使用法:
  python physical_validity_report.py \\
    --params <collection>/tuned_params.yaml \\
    --collection-dir <collection> \\
    --out <collection>/physical_validity_report.html
"""
from __future__ import annotations

import argparse
import html as _html_stdlib
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import yaml

# ---------------------------------------------------------------------------


from driving_log_replayer_v2.real_log_sim_comparison.lib._collection import DatasetEntry  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._coverage import _curvature_coverage  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._io import load_cmd, resolve_lite_bag  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._map import load_map_ways, resolve_map_osm  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.lib._multi_agg import (  # noqa: E402
    HORIZONS as _HORIZONS,
    acc_score as _acc_score,
    aggregate_normalized as _agg_normalized,
    robust_score as _robust_score,
    steer_score as _steer_score,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._tune_report import _build_viewer_html  # noqa: E402
from driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune import (  # noqa: E402
    _BASELINE_MODEL,
    _discover,
    _eval as _tune_eval,
    _filter_by_date,
    load_datasets,
)

# ---------------------------------------------------------------------------
# 定数 (物理定数・フィット定数の SSOT は lib._physical_validity。本ファイル固有の
#       レポート表示用定数のみここで定義する)
# ---------------------------------------------------------------------------
from driving_log_replayer_v2.real_log_sim_comparison.lib._figures import (  # noqa: E402
    build_fig_cross_long,
    build_fig_cross_long_tau_pointwise,
    build_fig_cross_steer,
    build_fig_kus_single,
    build_fig_long_perf_box,
    build_fig_long_perf_growth,
    build_fig_long_perf_map,
    build_fig_long_tau_pointwise_hist,
    build_fig_perfect_tracking_box,
    build_fig_perfect_tracking_traj,
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._physical_validity import (  # noqa: E402
    DWZ_MAX,
    VX_MIN_CURVE,
    WHEELBASE,
    WZ_MIN,
    _DELAY_CANDIDATES_LONG,
    _FIT_DT,
    _N_CROSS_FIT_DATASET,
    _PERF_HORIZONS,
    _PERF_STRIDE,
    _TAU_BOUNDS_LONG,
    _extract_kus_arrays,
    compute_cross_long_rows,
    compute_cross_steer_rows,
    compute_kus_bins,
    compute_long_perf_data,
    compute_perfect_tracking_data,
    fit_long_cross_dataset_bounded,
    fit_long_single,
    fit_steer_single,
    merged_model_params,
)

_H_SPAN = {10: "≈0.33 s", 20: "≈0.67 s", 30: "≈1.0 s", 40: "≈1.33 s"}

# 1-2/1-5. 理想追従評価に使うデータセット数（レポート固有。ホライズン・stride の SSOT は
# lib._physical_validity の _PERF_HORIZONS / _PERF_STRIDE を import して使う）
_PERF_N_DATASET = 10                                   # 1-5 横方向 box plot 用 データセット数
_LONG_PERF_N_DATASET = 20                                   # 1-2 縦方向理想追従 box plot 用 データセット数


def _to_entries(ds_list: list) -> list[DatasetEntry]:
    """`_discover` の (uuid, lite_dir) タプルリストを lib 共有関数用の DatasetEntry へ変換する。

    real_lite の解決は lib._io.resolve_lite_bag（`real.lite.mcap` 単一ファイル →
    `real.lite` rosbag2 dir の順）に委ねる。comparison/scenarios は本レポートでは不要。
    """
    entries: list[DatasetEntry] = []
    for uuid, lite_dir in ds_list:
        lite_dir = Path(lite_dir)
        entries.append(DatasetEntry(
            dataset_id=uuid,
            dir=lite_dir,
            real_lite=resolve_lite_bag(lite_dir, "real"),
            comparison_dir=None,
            scenarios_dir=None,
            status="success",
        ))
    return entries


def _fit_single_worker(args: tuple) -> tuple[str, dict | None, dict | None]:
    """プロセスワーカー: 1 データセットの縦方向 / 操舵 一次遅れモデル同定。"""
    uuid, bag_str = args
    bag = Path(bag_str)
    return uuid, fit_long_single(bag), fit_steer_single(bag)


def fit_per_dataset(
    entries: list[DatasetEntry], n_jobs: int = 8,
) -> tuple[dict[str, dict], dict[str, dict]]:
    """縦方向 / 操舵の per-dataset 実行時フィットを並列実行する。

    対象は real_lite を持つ全 entries。best/worst 図は「全件中の最良・最悪」を示すのが
    目的のため、旧 identify_{long,steer}_dynamics.py（事前 CSV 生成）や正典
    step_cross_dataset.py と同じく全件を母集団とする。
    Returns: (per_ds_long, per_ds_steer)  ({dataset_id: fit dict})
    """
    targets = [e for e in entries if e.real_lite is not None]
    per_ds_long: dict[str, dict] = {}
    per_ds_steer: dict[str, dict] = {}
    with ProcessPoolExecutor(max_workers=min(n_jobs, max(len(targets), 1))) as pool:
        futs = [
            pool.submit(_fit_single_worker, (e.dataset_id, str(e.real_lite)))
            for e in targets
        ]
        for i, fut in enumerate(as_completed(futs), 1):
            uuid, long_fit, steer_fit = fut.result()
            if long_fit is not None:
                per_ds_long[uuid] = long_fit
            if steer_fit is not None:
                per_ds_steer[uuid] = steer_fit
            if i % 100 == 0:
                print(f"  {i}/{len(targets)} フィット済み", flush=True)
    return per_ds_long, per_ds_steer


_MATHJAX_HEAD = (
    "<script>"
    r"window.MathJax={tex:{inlineMath:[['\\(','\\)']],displayMath:[['\\[','\\]']]},"
    "svg:{fontCache:'global'}};"
    "</script>"
    "<script async src='https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js'></script>"
)
_PLOTLY_CDN = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'

_STYLE = """
body { font-family: sans-serif; max-width: 1300px; margin: 0 auto; padding: 20px; color: #333; }
h1 { color: #222; }
h2 { color: #444; border-bottom: 2px solid #bbb; padding-bottom: 4px; margin-top: 36px; }
h3 { color: #555; margin-top: 20px; }
p { line-height: 1.6; }
code { background: #f0f0f0; padding: 2px 4px; border-radius: 3px; font-size: 12px; }
.param-table { border-collapse: collapse; margin: 12px 0; font-size: 13px; }
.param-table td, .param-table th { border: 1px solid #ddd; padding: 6px 12px; }
.param-table th { background: #f5f5f5; }
.meta { color: #888; font-size: 12px; margin-bottom: 16px; }
.note { background: #fff8e1; border-left: 4px solid #ffc107; padding: 8px 12px;
        margin: 8px 0; font-size: 13px; }
nav a { margin-right: 12px; }
details { margin: 8px 0; }
details > summary {
  cursor: pointer; font-weight: bold; color: #555;
  padding: 4px 0; list-style: none; display: flex; align-items: center; gap: 6px;
}
details > summary::before { content: "▶"; font-size: 10px; color: #888; transition: transform 0.15s; }
details[open] > summary::before { transform: rotate(90deg); }
details > summary::-webkit-details-marker { display: none; }
"""


# ---------------------------------------------------------------------------
# Phase 1: 並列 MCAP 読み込みワーカー（k_us 分析 + カーブカバレッジ）
# ---------------------------------------------------------------------------
def _load_mcap_worker(args: tuple) -> dict | None:
    """プロセスワーカー: 1 MCAP から k_us 分析用データとカーブカバレッジを抽出。"""
    # 引数は (uuid: str, lite_dir: str) のタプル（pickle 対応のため str）
    uuid, lite_dir_str = args
    lite_dir = Path(lite_dir_str)
    mcap = lite_dir / "real.lite" / "real.lite_0.mcap"
    if not mcap.exists():
        return None

    # k_us 分析用配列 (vx/wz/steer_eff/dwz) + 付随情報 (t/yaw) は lib と共通化
    rec = _extract_kus_arrays(mcap)
    if rec is None:
        return None
    t_k = rec["t"]
    vx = rec["vx"]

    try:
        df_cmd = load_cmd(mcap, "/control/command/control_cmd")
    except Exception:
        df_cmd = pd.DataFrame()

    # カーブカバレッジ（_coverage._curvature_coverage は t 列を要求）
    t_rel = t_k - t_k[0]
    kin_cv = pd.DataFrame({"t": t_rel, "yaw": rec["yaw"]})
    vel_cv = pd.DataFrame({"t": t_rel, "lon_vel": vx})
    try:
        cov = _curvature_coverage(kin_cv, vel_cv)
    except Exception:
        cov = {"curve_count": 0, "kappa_max_abs": 0.0}

    # cmd_steer（stride=10 で間引き: 操舵信号の時系列表示用）
    _STRIDE = 10
    t_sub = t_k[::_STRIDE]
    if not df_cmd.empty:
        t_c = df_cmd["t_ns"].values * 1e-9
        cmd_steer_arr = np.interp(t_sub, t_c, df_cmd["cmd_steer"].values).tolist()
    else:
        cmd_steer_arr = []

    return {
        "uuid": uuid,
        "lite_dir": lite_dir_str,
        "vx": vx.tolist(),
        "wz": rec["wz"].tolist(),
        "steer_eff": rec["steer_eff"].tolist(),
        "dwz": rec["dwz"].tolist(),
        "curve_count": cov["curve_count"],
        "kappa_max_abs": cov["kappa_max_abs"],
        "cmd_steer": cmd_steer_arr,
    }


def load_all_mcap(ds_list: list, n_jobs: int = 8) -> list[dict]:
    """全データセットを並列 MCAP 読み込み。"""
    args_list = [(uuid, str(lite_dir)) for uuid, lite_dir in ds_list]
    results: list[dict] = []
    with ProcessPoolExecutor(max_workers=n_jobs) as pool:
        futs = {pool.submit(_load_mcap_worker, a): a for a in args_list}
        for i, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            if r is not None:
                results.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(args_list)} 読み込み済み", flush=True)
    return results


# ---------------------------------------------------------------------------
# HTML セクション組み立て
# ---------------------------------------------------------------------------
def _build_sec_metrics(baseline_score: float | None, phase14_score: float, label: str = "current") -> str:
    """各種メトリクスの直感的・物理的解説セクション。"""
    if baseline_score and baseline_score > 0:
        improvement_pct = (baseline_score - phase14_score) / baseline_score * 100
        nyaw_ratio = 100 - improvement_pct
        score_bullet = (
            f"  <li>{label} の score = <b>{phase14_score:.3f}</b>"
            f"（baseline = <b>{baseline_score:.3f}</b> → 約 <b>{improvement_pct:.1f}%</b> 改善）</li>"
        )
        note_text = (
            f"{label}（{phase14_score:.3f}）が baseline（\\(k_{{\\mathrm{{us}}}}=0\\) / "
            f"<code>steer_dead_band</code>=0, score={baseline_score:.3f}）より"
            f" {improvement_pct:.1f}% 低いことは、yaw/lat 誤差が baseline の約 {nyaw_ratio:.0f}% まで縮小したことを意味する。"
        )
    else:
        score_bullet = f"  <li>{label} の score = <b>{phase14_score:.3f}</b></li>"
        note_text = f"baseline score が未計算のため比較なし（--metrics-cache を指定すると比較が有効になります）。"
    # LaTeX を含む静的部分は通常文字列。プレースホルダを後置換で動的値に差し替える
    tmpl = """
<section id="metrics">
<h2>0. 評価メトリクスの物理的意味</h2>

<details>
<summary>0-1. N-step 前向き積分誤差</summary>
<p>
車両モデルを実機ログの初期状態から N 個の制御コマンド区間だけ前向きに積分し、
実機の自己位置推定軌跡との終端誤差を評価する。
制御コマンドは 30 Hz（\\(\\Delta t = 1/30\\) 秒 \\(\\approx 33\\) ms）で記録されており、
ホライズン N=10（≈ 0.33 秒先）〜 N=40（≈ 1.33 秒先）の 4 点を等重みで集約する。
</p>
<table class="param-table">
  <tr><th>ホライズン</th><th>時間スパン（30 Hz 基準）</th><th>主に捉える現象</th></tr>
  <tr><td>N=10</td><td>≈ 0.33 秒先</td><td>アクチュエータ遅れ・一次遅れ時定数（即応性）</td></tr>
  <tr><td>N=20</td><td>≈ 0.67 秒先</td><td>中期の操舵追従・加速度変動</td></tr>
  <tr><td>N=30</td><td>≈ 1.0 秒先</td><td>ホイールベース・ステアバイアスの累積効果</td></tr>
  <tr><td>N=40</td><td>≈ 1.33 秒先</td><td>アンダーステア・カーブ全体の軌跡ドリフト</td></tr>
</table>
<div class="note">
<b>直感</b>: N=10 でいい成績でも N=40 が悪い場合、モデルは「短期の応答」は捉えているが
「カーブを曲がり続ける能力」に欠陥がある（= アンダーステア補正や累積バイアスの問題）。
</div>
</details>

<details>
<summary>0-2. 誤差の 3 成分（yaw・long・lat）</summary>
<table class="param-table">
  <tr><th>成分</th><th>単位</th><th>物理的意味</th><th>主な感度</th></tr>
  <tr>
    <td><b>yaw 誤差</b></td><td>deg</td>
    <td>N ステップ後の車両姿勢角（ヨー角）誤差。旋回量の過不足を示す。</td>
    <td>操舵一次遅れ時定数 \\(\\tau_\\delta\\)、\\(k_{{\\mathrm{{us}}}}\\)、ホイールベース \\(L\\)</td>
  </tr>
  <tr>
    <td><b>long 誤差</b></td><td>cm</td>
    <td>進行方向の位置誤差（前後方向）。加速度の積算ズレを示す。</td>
    <td>加速度一次遅れ時定数 \\(\\tau_a\\)、純粋遅延 \\(T_a\\)</td>
  </tr>
  <tr>
    <td><b>lat 誤差</b></td><td>cm</td>
    <td>横方向の位置誤差。操舵追従精度とアンダーステアの積算効果を示す。</td>
    <td>\\(k_{{\\mathrm{{us}}}}\\)、<code>steer_dead_band</code>、\\(\\tau_\\delta\\)、<code>steer_bias</code></td>
  </tr>
</table>
<div class="note">
<b>long ⊥ steer の直交性</b>: 低速・定常走行では long 誤差は加速度パラメータのみに感度を持ち、
steer 系パラメータへの感度はほぼゼロ（逆もしかり）。これを利用して2フェーズ独立チューニングを実現。
</div>
</details>

<details>
<summary>0-3. 正規化スコア（nyaw, nlong, nlat）</summary>
<p>
各データセットの誤差を <b>baseline モデル</b>（補正なし遅延モデル、\\(k_{{\\mathrm{{us}}}}=0\\)・<code>steer_dead_band</code>=0 相当）の
誤差で正規化する:
\\[
\\text{nyaw} = \\frac{\\text{yaw}_{\\mathrm{tuned}}}{\\max(\\text{yaw}_{\\mathrm{baseline}},\\; \\text{floor}_{\\mathrm{yaw}})}
\\]
</p>
<ul>
  <li>\\(\\text{nyaw} < 1\\): baseline より良い（チューニング済みが有効）</li>
  <li>\\(\\text{nyaw} = 1\\): baseline と同等</li>
  <li>\\(\\text{nyaw} > 1\\): baseline より悪い（過補正・副作用）</li>
</ul>
<p><b>なぜ正規化するか</b>:
絶対誤差のまま集約すると、大カーブ・高速など「難しいシナリオ」（baseline 誤差が大きいデータセット）が
スコアを支配してしまい、全 650 データセットで均等に改善できているかを測れない。
正規化により「baseline と比べてどれだけ改善したか」を全データセットで統一スケールで評価できる。
</p>
<table class="param-table">
  <tr><th>フロア定数</th><th>N=10</th><th>N=40</th><th>目的</th></tr>
  <tr><td>YAW_FLOOR</td><td>0.06 deg</td><td>0.24 deg</td>
    <td>低ダイナミクス（ほぼ直進）のデータセットで分母がゼロ近くになる暴発を防ぐ</td></tr>
  <tr><td>LONG_FLOOR</td><td>1.0 cm</td><td>4.5 cm</td><td>縦方向の同上</td></tr>
  <tr><td>LAT_FLOOR</td><td>0.3 cm</td><td>1.2 cm</td><td>横方向の同上</td></tr>
</table>
</details>

<details>
<summary>0-4. mean と worst</summary>
<p>
650 データセットの正規化スコアに対して 2 種類の集約を行う:
</p>
<ul>
  <li><b>mean</b>: 全データセットの平均。「全体的に良い設定」を測る。</li>
  <li><b>worst</b>: 全データセットの最大値（最悪ケース）。「どのシナリオでも崩れない頑健性」を測る。</li>
</ul>
<p>
mean だけを最小化すると、一部のデータセットに特化したパラメータが選ばれ worst が悪化することがある。
worst だけだと過度に保守的になる。両者を組み合わせることでロバストな設定を探索する。
</p>
</details>

<details>
<summary>0-5. ロバストスコア（robust_score）</summary>
<p>
最終目的関数:
\\[
\\text{score} = \\sum_{h \\in \\{10,20,30,40\\}} \\left[
  (\\overline{\\text{nyaw}} + 0.5 \\overline{\\text{nlong}} + 0.5 \\overline{\\text{nlat}})
+ 0.5 (\\hat{\\text{nyaw}} + 0.5 \\hat{\\text{nlong}} + 0.5 \\hat{\\text{nlat}})
\\right]
\\]
ここで \\(\\overline{\\cdot}\\) は mean、\\(\\hat{\\cdot}\\) は worst（全データセットの max）。
<b>スコアは小さいほど良い。</b>
</p>
<ul>
  <li>yaw の重み = 1（位置の重み 0.5 + 0.5 = 1 と均等）</li>
  <li>long と lat は各 0.5 倍（yaw : 位置 = 1 : 1 を維持）</li>
  <li>worst 項の重み 0.5 = 「mean の改善と worst の頑健性を半々で重視」</li>
__SCORE_BULLET__
</ul>
<div class="note">
<b>直感的なスケール</b>: score が 1 下がると「全 650 データセット・全 4 ホライズンで平均的に
nyaw が 1/8 改善した」相当（sum over 4 horizons × 2 terms (mean+0.5worst) でほぼ 8 で割る）。
__NOTE_TEXT__
</div>
</details>
</section>
"""
    return tmpl.replace("__SCORE_BULLET__", score_bullet).replace("__NOTE_TEXT__", note_text)


def _kus_band_table_rows(params: dict) -> str:
    """k_us 速度帯パラメータの HTML テーブル行を生成。"""
    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")



    if bands is not None and thresholds is not None:
        rows = []
        for i, b in enumerate(bands):
            if i == 0:
                speed_range = f"\\(v_x\\) &lt; {thresholds[0]:.2f} m/s"
            elif i < len(thresholds):
                speed_range = f"{thresholds[i-1]:.2f} ≤ \\(v_x\\) &lt; {thresholds[i]:.2f} m/s"
            else:
                speed_range = f"\\(v_x\\) ≥ {thresholds[-1]:.2f} m/s"
            rows.append(
                f"  <tr><td><code>k_us_band[{i}]</code></td>"
                f"<td>{b:.6f} rad·s²/m</td>"
                f"<td>{speed_range} の \\(k_{{\\mathrm{{us,eff}}}}\\)</td></tr>"
            )
        for i, thr in enumerate(thresholds):
            rows.append(
                f"  <tr><td><code>k_us_threshold[{i}]</code></td>"
                f"<td>{thr:.3f} m/s</td>"
                f"<td>速度帯 {i} → {i+1} の切替閾値</td></tr>"
            )
        return "\n".join(rows)

    k_us = params.get("k_us", 0.0)
    return f"  <tr><td><code>k_us</code></td><td>{k_us:.6f} rad·s²/m</td><td>アンダーステア係数（速度依存なし）</td></tr>"


def _build_sec1(
    params: dict,
    long_fig: go.Figure,
    steer_fig: go.Figure,
    kus_fig: go.Figure,
    n_dataset: int,
    long_perf_figs: tuple[go.Figure, go.Figure, go.Figure] | None = None,
    long_tau_fig: go.Figure | None = None,
    long_tau_hist_fig: go.Figure | None = None,
) -> str:
    """1-0. 座標系と主要な記号の定義 + モデルパラメータの定義を含む sec1 全体 HTML を返す。"""
    kus_rows = _kus_band_table_rows(params)

    # ---------- モデルパラメータの定義（旧 _build_sec_model_params）----------
    def _fmt_p(v) -> str:
        if isinstance(v, float):
            return f"{v:.6g}"
        return str(v)

    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")
    if bands is not None and thresholds is not None:
        _kus_rows_p = []
        for i, b in enumerate(bands):
            if i == 0:
                speed_range = f"\\(v_x\\) &lt; {thresholds[0]:.2f} m/s"
            elif i < len(thresholds):
                speed_range = f"{thresholds[i-1]:.2f} ≤ \\(v_x\\) &lt; {thresholds[i]:.2f} m/s"
            else:
                speed_range = f"\\(v_x\\) ≥ {thresholds[-1]:.2f} m/s"
            _kus_rows_p.append(
                f"  <tr><td><code>k_us_band[{i}]</code></td>"
                f"<td>{_fmt_p(b)} rad·s²/m</td>"
                f"<td>{speed_range} のアンダーステア係数</td></tr>"
            )
        for i, thr in enumerate(thresholds):
            _kus_rows_p.append(
                f"  <tr><td><code>k_us_threshold[{i}]</code></td>"
                f"<td>{_fmt_p(thr)} m/s</td><td>速度帯 {i} → {i+1} の切替閾値</td></tr>"
            )
        kus_profile_rows = "\n".join(_kus_rows_p)
    else:
        kus_profile_rows = (
            f"  <tr><td><code>k_us</code></td>"
            f"<td>{_fmt_p(params.get('k_us', 0.0))} rad·s²/m</td>"
            f"<td>全速度域で一定のアンダーステア係数</td></tr>"
        )


    def _fmt(v) -> str:  # noqa: E306 (ローカル _fmt。上の _fmt_p とスコープが異なる)
        if isinstance(v, float):
            return f"{v:.6g}"
        return str(v)

    tau_a = _fmt(params.get("acc_time_constant", "N/A"))
    T_a   = _fmt(params.get("acc_time_delay", "N/A"))
    tau_d = _fmt(params.get("steer_time_constant", "N/A"))
    T_d   = _fmt(params.get("steer_time_delay", "N/A"))
    DSF   = _fmt(params.get("debug_steer_scaling_factor", "N/A"))
    beta  = _fmt(params.get("steer_bias", "N/A"))
    db    = _fmt(params.get("steer_dead_band", "N/A"))
    rlim  = _fmt(params.get("steer_rate_lim", "N/A"))

    _long_html_inner  = long_fig.to_html(full_html=False, include_plotlyjs=False)
    _steer_html_inner = steer_fig.to_html(full_html=False, include_plotlyjs=False)
    long_html  = f"<details><summary>時系列グラフを表示（クリックで展開）</summary>{_long_html_inner}</details>"
    steer_html = f"<details><summary>時系列グラフを表示（クリックで展開）</summary>{_steer_html_inner}</details>"
    kus_html   = kus_fig.to_html(full_html=False, include_plotlyjs=False)
    long_tau_html = long_tau_fig.to_html(full_html=False, include_plotlyjs=False) if long_tau_fig is not None else ""
    long_tau_hist_html = (
        long_tau_hist_fig.to_html(full_html=False, include_plotlyjs=False) if long_tau_hist_fig is not None else ""
    )
    _vx_min = VX_MIN_CURVE
    _stride = _PERF_STRIDE
    _dt_long  = _FIT_DT
    _delay_lo = _DELAY_CANDIDATES_LONG[0]
    _delay_hi = _DELAY_CANDIDATES_LONG[-1]
    _delay_m  = len(_DELAY_CANDIDATES_LONG) - 1
    _tau_lo, _tau_hi = _TAU_BOUNDS_LONG
    if long_perf_figs is not None:
        fig_box, fig_growth, fig_map = long_perf_figs
        box_html    = fig_box.to_html(full_html=False, include_plotlyjs=False)
        growth_html = fig_growth.to_html(full_html=False, include_plotlyjs=False)
        map_html    = fig_map.to_html(full_html=False, include_plotlyjs=False)
        long_perf_subsection = (
            "<h3>モデル構造限界評価（acc 理想追従）</h3>"
            "<p>シミュレータの加速度応答が実機と完全一致（\\(a_{\\mathrm{act,sim}} = a_{\\mathrm{act,real}}\\)）した場合に"
            "残る縦方向変位誤差を評価する。<br>"
            "各開始点で \\(v_{x,\\mathrm{sim}}(t_0) = v_{x,\\mathrm{real}}(t_0)\\) に初期化し、"
            "実測加速度 \\(a_{\\mathrm{act}}\\) を直接積分して変位を計算、実車変位と比較する。</p>"
            "<p>1-5 横方向評価との対称性: "
            "1-5 では <i>gt_steer</i> を bicycle model に直接入力して steer 完全追従時の横方向残差を評価する。"
            "本評価では <i>gt_a_act</i> を積分器に直接入力して acc 完全追従時の縦方向残差を評価する。</p>"
            "<div class=\"note\">"
            "&#9888;&#65039; <b>残差の解釈</b>: \\(\\dot{v}_x = a_{\\mathrm{act}}\\) という運動方程式に含まれない"
            "要素（路面勾配・空気抵抗・タイヤ縦力・加速度センサーバイアス等）が残差として現れる。"
            "</div>"
            f"<p>走行区間（\\(v_x > {_vx_min}\\) m/s）を stride={_stride} ステップで走査し、"
            "N-step ロールアウト終端の縦方向誤差絶対値を集計する。"
            "ホライズン: N=10（0.10s）, N=20（0.20s）, N=50（0.50s）, N=100（1.00s）。</p>"
            + box_html
            + "<p><b>図②: ドリフト成長カーブ（符号付き）</b> — "
            "ゼロ線から片側に膨らむ傾向が系統的な過大／過小推定を示す。"
            "帯（四分位範囲 25–75%）はばらつき、膨らむ速さは蓄積の速度を表す。"
            "局面別点線（青=減速・灰=巡航・赤=加速）でどの走行シーンでズレが生じるかを確認できる。"
            "x 軸 of 0.10/0.20/0.50/1.00s は上の box plot のホライズンと一致する。</p>"
            + growth_html
            + "<p><b>図③: 地図上の変位誤差分布</b> — "
            "各点は rollout 開始位置（\\(v_x > v_{\\mathrm{min}}\\) を満たす 1.0s 窓の開始点）。"
            "色は 1.0s 窓終端 of 変位誤差（赤 = シミュレーションが実車より進む過大推定、青 = 過小推定）。"
            "路線・カーブ・区間ごとに誤差パターンを地理的に把握できる。</p>"
            + map_html
        )
    else:
        long_perf_subsection = ""

    return f"""
<section id="sec-coords">
<h2>1-0. 座標系と主要な記号の定義</h2>
<p>
本レポートの運動方程式は、以下の車体基準（body frame）および進行方向基準の座標系に基づく。
「実装ラベル」列は、シミュレータ内部の状態ベクトル（6 状態、詳細は
<a href="#sec-state-space">1-1 節</a>）の IDX か、対応する YAML 設定名を示す（該当しない記号は入力・
導出量などであり、実装ラベル欄には <code>—</code> あるいは補助注記を記す）。「ROS トピック / フィールド」列は、
実機ログ（<code>real.lite</code>）と scenario_simulator_v2 の双方で共通に使われるトピック名と
フィールドを <code>topic.field</code> 形式でまとめる。
</p>
<table class="param-table">
  <tr><th>記号</th><th>意味</th><th>単位</th><th>実装ラベル</th>
      <th>ROS トピック / フィールド</th>
  </tr>
  <tr><td>\\(x, y\\)</td><td>地図平面上の車両位置</td><td>m</td>
      <td><code>IDX::X</code>, <code>IDX::Y</code></td>
      <td><code>/localization/kinematic_state.pose.pose.position.{{x, y}}</code></td>
  </tr>
  <tr><td>\\(\\theta\\)</td><td>ヨー角（車体の向き。地図 X 軸からの反時計回り回転）</td><td>rad</td>
      <td><code>IDX::YAW</code></td>
      <td><code>/localization/kinematic_state.pose.pose.orientation</code></td>
  </tr>
  <tr><td>\\(v_x\\)</td><td>前進速度（進行方向＝車体 X 軸成分。<code>lon_vel</code>）</td><td>m/s</td>
      <td><code>IDX::VX</code></td>
      <td><code>/vehicle/status/velocity_status.longitudinal_velocity</code></td>
  </tr>
  <tr><td>\\(\\delta_{{\\mathrm{{act}}}}\\)</td><td>前輪実ステア角</td><td>rad</td>
      <td><code>IDX::STEER</code></td>
      <td><code>/vehicle/status/steering_status.steering_tire_angle</code></td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{act}}}}\\)</td><td>アクチュエータ出力（実加速度。勾配重力加算前の値）</td><td>m/s²</td>
      <td><code>IDX::PEDAL_ACCX</code></td>
      <td>—（直接の実測トピックなし。次行 \\(a_{{\\mathrm{{report}}}}\\) を実測相当として使用）</td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{report}}}}\\)</td>
      <td>レポート・可視化用の加速度。<code>calcModel</code> の微分には現れず、
      Euler 更新後に \\((v_{{x,\\mathrm{{new}}}} - v_{{x,\\mathrm{{prev}}}}) / \\Delta t\\) として事後計算される
      別枠の出力量（<b>ODE 状態の \\(a_{{\\mathrm{{act}}}}\\) とは別物</b>。実測 \\(a_{{\\mathrm{{act}}}}\\) 相当として使用）</td>
      <td>m/s²</td><td><code>IDX::ACCX</code></td>
      <td><code>/localization/acceleration.accel.accel.linear.x</code></td>
  </tr>
  <tr><td>\\(v_y\\)</td><td>横速度</td><td>m/s</td>
      <td>—（モデルの <code>getVy()</code> は常に 0 固定）</td>
      <td><code>/localization/kinematic_state.twist.twist.linear.y</code></td>
  </tr>
  <tr><td>\\(\\omega\\)</td><td>ヨーレート（\\(\\dot\\theta\\)）</td><td>rad/s</td>
      <td>—（導出量。<code>calc_yaw_rate</code> で算出）</td>
      <td><code>/localization/kinematic_state.twist.twist.angular.z</code></td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{slope}}}}\\)</td><td>路面勾配による重力加速度成分</td><td>m/s²</td>
      <td>—（入力 <code>SLOPE_ACCX</code>）</td>
      <td>専用トピックなし（pitch から算出）</td>
  </tr>
  <tr><td>\\(a_{{\\mathrm{{cmd,des}}}}\\)</td><td>加速度指令</td><td>m/s²</td>
      <td>—（入力）</td>
      <td><code>/control/command/control_cmd.longitudinal.acceleration</code></td>
  </tr>
  <tr><td>\\(\\delta_{{\\mathrm{{cmd,des}}}}\\)</td><td>操舵指令</td><td>rad</td>
      <td>—（入力）</td>
      <td><code>/control/command/control_cmd.lateral.steering_tire_angle</code></td>
  </tr>
  <tr><td><code>gear</code></td><td>ギア状態。運動方程式の連続入力ではなく、DRIVE 系
      （<code>GearReport.report</code> の enum 値 2..19）だけを同定・評価に使うための離散マスク</td><td>—</td>
      <td>—（評価マスク）</td>
      <td><code>/vehicle/status/gear_status.report</code>（直前値で時刻対応）</td>
  </tr>
  <tr><td>\\(\\beta\\)</td><td>ステアバイアス（系統的操舵オフセット）</td><td>rad</td>
      <td><code>steer_bias</code></td>
      <td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(L\\)</td><td>ホイールベース</td><td>m</td>
      <td><code>wheelbase</code></td>
      <td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(k_{{\\mathrm{{us}}}}\\)</td><td>アンダーステア係数（速度依存ランプ）</td><td>rad·s²/m</td>
      <td><code>k_us_bands / k_us</code></td>
      <td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(\\tau_\\delta\\)</td><td>操舵 1 次遅れ時定数</td><td>s</td>
      <td><code>steer_time_constant</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(T_\\delta\\)</td><td>操舵純粋遅延</td><td>s</td>
      <td><code>steer_time_delay</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(\\tau_a\\)</td><td>加速度 1 次遅れ時定数</td><td>s</td>
      <td><code>acc_time_constant</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(T_a\\)</td><td>加速度純粋遅延</td><td>s</td>
      <td><code>acc_time_delay</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(d\\)</td><td>操舵不感帯幅（±d 以内の指令は無視）</td><td>rad</td>
      <td><code>steer_dead_band</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(K_{{\\mathrm{{steer\_scale}}}}\\)</td><td>操舵指令スケーリング倍率（1.0 = 補正なし）</td><td>—</td>
      <td><code>debug_steer_scaling_factor</code></td><td>—（設定パラメータ）</td>
  </tr>
  <tr><td>\\(\\dot{{\\delta}}_{{\\mathrm{{lim}}}}\\)</td><td>操舵レート制限（飽和速度）</td><td>rad/s</td>
      <td><code>steer_rate_lim</code></td><td>—（設定パラメータ）</td>
  </tr>
</table>
</section>

<section id="sec-state-space">
<h2>1-1. 状態空間モデルと数値積分（Euler 法によるアップデート）</h2>
<p>
1-2（縦方向）・1-3（操舵）・1-4（ヨー・横方向）で個別に同定する運動方程式を、実装では一つの状態ベクトルに
まとめた \\(\\dot{{x}} = f(x, u)\\) として扱い、共通の Euler 積分ループで毎ステップ時間発展させている。
各記号の意味・単位・ROS トピックは <a href="#sec-coords">1-0 の記号表</a>を参照。
</p>

<h3>連続時間の状態方程式（ベクトル形式）</h3>
<p class="meta">
<span style="color:#1565c0">青字</span>が状態ベクトル x（6状態）、<span style="color:#e65100">橙字</span>が入力ベクトル u。
</p>
\\[
{{\\color{{#1565c0}} x}} =
\\begin{{pmatrix}}
{{\\color{{#1565c0}} x}} \\\\
{{\\color{{#1565c0}} y}} \\\\
{{\\color{{#1565c0}} \\theta}} \\\\
{{\\color{{#1565c0}} v_x}} \\\\
{{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}}
\\end{{pmatrix}}
, \\qquad
{{\\color{{#e65100}} u}} =
\\begin{{pmatrix}}
{{\\color{{#e65100}} a_{{\\mathrm{{cmd,des}}}}}} \\\\
{{\\color{{#e65100}} \\delta_{{\\mathrm{{cmd,des}}}}}} \\\\
{{\\color{{#e65100}} a_{{\\mathrm{{slope}}}}}}
\\end{{pmatrix}}
\\]
<p class="meta">
\\(a_{{\\mathrm{{cmd,des}}}}, \\delta_{{\\mathrm{{cmd,des}}}}\\) は純粋遅延 \\(T_a, T_\\delta\\) を経て、
下の状態方程式では \\(a_{{\\mathrm{{target}}}}, \\delta_{{\\mathrm{{des}}}}\\)（橙字のまま）として現れる。
<code>gear</code> はこの入力ベクトルには含めず、<code>/vehicle/status/gear_status</code> の直前値から
DRIVE 系（enum 値 2..19）の時刻だけを残す評価マスクとして適用する。非 DRIVE 系
（NEUTRAL/REVERSE/PARK/LOW 等）は同定・評価窓から除外される。
</p>
\\[
\\dot{{x}} =
\\begin{{pmatrix}}
{{\\color{{#1565c0}} \\dot x}} \\\\
{{\\color{{#1565c0}} \\dot y}} \\\\
{{\\color{{#1565c0}} \\dot\\theta}} \\\\
{{\\color{{#1565c0}} \\dot v_x}} \\\\
{{\\color{{#1565c0}} \\dot\\delta_{{\\mathrm{{act}}}}}} \\\\
{{\\color{{#1565c0}} \\dot a_{{\\mathrm{{act}}}}}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} v_x}} \\cos{{\\color{{#1565c0}} \\theta}} \\\\
{{\\color{{#1565c0}} v_x}} \\sin{{\\color{{#1565c0}} \\theta}} \\\\
\\dfrac{{{{\\color{{#1565c0}} v_x}}\\,\\tan({{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}+\\beta)}}{{L + k_{{\\mathrm{{us,eff}}}}({{\\color{{#1565c0}} v_x}})\\,{{\\color{{#1565c0}} v_x}}^2}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}} + {{\\color{{#e65100}} a_{{\\mathrm{{slope}}}}}} \\\\
\\dfrac{{{{\\color{{#e65100}} \\delta_{{\\mathrm{{des}}}}}} - {{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}}}{{\\tau_\\delta}} \\\\
\\dfrac{{{{\\color{{#e65100}} a_{{\\mathrm{{target}}}}}} - {{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}}}}{{\\tau_a}}
\\end{{pmatrix}}
\\]
<p class="meta">
&#128279; \\(\\tau_\\delta, \\beta\\) は <a href="#sec-steer">1-3</a>、\\(\\tau_a\\) は <a href="#sec-long">1-2</a>、
\\(L, k_{{\\mathrm{{us,eff}}}}\\) は <a href="#sec-yaw">1-4</a> で定義。
</p>

<h3>離散化：Euler 法によるアップデート</h3>
<p>
制御周期 \\(\\Delta t\\) ごとの離散時刻を \\(k = 0, 1, 2, \\dots\\) とすると、
\\[
{{\\color{{#1565c0}} x_{{k+1}}}} = {{\\color{{#1565c0}} x_k}} + f({{\\color{{#1565c0}} x_k}}, {{\\color{{#e65100}} u_k}}) \\cdot \\Delta t
\\]
で状態を更新する（前進・陽的 Euler 法、実装は
<code>SimModelInterface::updateEuler(dt, input)</code>）。\\(f\\) を展開すると、各状態は次のように更新される。
</p>
\\[
\\begin{{pmatrix}}
{{\\color{{#1565c0}} x_{{k+1}}}} \\\\
{{\\color{{#1565c0}} y_{{k+1}}}} \\\\
{{\\color{{#1565c0}} \\theta_{{k+1}}}} \\\\
{{\\color{{#1565c0}} v_{{x,k+1}}}} \\\\
{{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k+1}}}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}},k+1}}}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} x_k}} \\\\
{{\\color{{#1565c0}} y_k}} \\\\
{{\\color{{#1565c0}} \\theta_k}} \\\\
{{\\color{{#1565c0}} v_{{x,k}}}} \\\\
{{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k}}}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}},k}}}}
\\end{{pmatrix}}
+
\\begin{{pmatrix}}
{{\\color{{#1565c0}} v_{{x,k}}}} \\cos{{\\color{{#1565c0}} \\theta_k}} \\\\
{{\\color{{#1565c0}} v_{{x,k}}}} \\sin{{\\color{{#1565c0}} \\theta_k}} \\\\
\\dfrac{{{{\\color{{#1565c0}} v_{{x,k}}}}\\,\\tan({{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k}}}}+\\beta)}}{{L + k_{{\\mathrm{{us,eff}}}}({{\\color{{#1565c0}} v_{{x,k}}}})\\,{{\\color{{#1565c0}} v_{{x,k}}}}^2}} \\\\
{{\\color{{#1565c0}} a_{{\\mathrm{{act}},k}}}} + {{\\color{{#e65100}} a_{{\\mathrm{{slope}},k}}}} \\\\
\\dfrac{{{{\\color{{#e65100}} \\delta_{{\\mathrm{{des}},k}}}} - {{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}},k}}}}}}{{\\tau_\\delta}} \\\\
\\dfrac{{{{\\color{{#e65100}} a_{{\\mathrm{{target}},k}}}} - {{\\color{{#1565c0}} a_{{\\mathrm{{act}},k}}}}}}{{\\tau_a}}
\\end{{pmatrix}}
\\cdot \\Delta t
\\]
</section>

<section id="sec-long">
<h2>1-2. 縦方向（加速度アクチュエータ）の同定</h2>
<p class="meta">
&#128279; <a href="#sec-state-space">1-1 状態方程式</a> の \\(\\dot v_x, \\dot a_{{\\mathrm{{act}}}}\\) 行に対応。
</p>
<p>
加速度アクチュエータの一次遅れ式は、入力 \\(a_{{\\mathrm{{cmd}}}}\\) と状態
\\(a_{{\\mathrm{{act}}}}\\) だけで閉じており、操舵角・ヨー角・横位置を参照しない。
さらに \\(a_{{\\mathrm{{act}}}}\\) は実機ログから観測できるため、路面勾配成分を差し引いた
\\(a_{{\\mathrm{{act,corr}}}}\\) とモデル出力 \\(a_{{\\mathrm{{sim}}}}\\) の残差を目的関数として、
\\(\\tau_a, T_a\\) を操舵・横方向パラメータとは独立に同定できる。
</p>

<p><b>運動方程式（1-1 と同じ状態・入力表記）:</b></p>
\\[
\\begin{{pmatrix}}
{{\\color{{#1565c0}} \\dot v_x}} \\\\
{{\\color{{#1565c0}} \\dot a_{{\\mathrm{{act}}}}}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}} + {{\\color{{#e65100}} a_{{\\mathrm{{slope}}}}}} \\\\
\\dfrac{{{{\\color{{#e65100}} a_{{\\mathrm{{target}}}}}} - {{\\color{{#1565c0}} a_{{\\mathrm{{act}}}}}}}}{{\\tau_a}}
\\end{{pmatrix}},
\\qquad
{{\\color{{#e65100}} a_{{\\mathrm{{target}}}}}}(t) = {{\\color{{#e65100}} a_{{\\mathrm{{cmd}}}}}}(t - T_a)
\\]
<p><b>式中の定数・補足:</b></p>
<ul>
  <li>\\(T_a\\)（<code>acc_time_delay</code>）: 加速度指令 \\(a_{{\\mathrm{{cmd}}}}\\) がアクチュエータに届くまでの純粋遅延。</li>
  <li>\\(\\tau_a\\)（<code>acc_time_constant</code>）: 目標加速度 \\(a_{{\\mathrm{{target}}}}\\) に対するアクチュエータ出力 \\(a_{{\\mathrm{{act}}}}\\) の一次遅れ時定数。</li>
  <li>\\(a_{{\\mathrm{{slope}}}}\\): 応答遅れを持たない勾配重力加速度成分。\\(\\dot v_x\\) への外部入力。</li>
  <li>走行抵抗: 一般形では \\(a_{{\\mathrm{{target}}}}\\) に \\(c_0 + c_1 v_x + c_2 v_x^2\\)（<code>lon_drag_c0</code>, <code>lon_drag_c1</code>, <code>lon_drag_c2</code>）を加算。現在はすべて 0 のため上式では省略。</li>
  <li>制限値: 目標加速度とアクチュエータ出力加速度は <code>vel_rate_lim</code>、車速は <code>vel_lim</code>。</li>
</ul>

<h3>実機ログからの独立同定（代表データセットモデルフィット）</h3>
<p>
\\(a_{{\\mathrm{{cmd}}}}\\)（指令加速度）を入力として遅延グリッドサーチ + output-error 非線形最小二乗法で
データセット横断同定した \\((\\tau_a, T_a)\\) のモデル出力（青実線）と
実測 \\(a_{{\\mathrm{{act}}}}\\)（黒実線）を比較する。最小二乗の目的関数では
\\(a_{{\\mathrm{{sim}}}} - a_{{\\mathrm{{act,corr}}}}\\) を使い、表示では路面勾配成分をモデル出力へ戻している。
チューニング値でのシミュレーション結果（点線）も重ね描きする。per-dataset 同定誤差（RMSE）の
最良・最悪データセットを選択し、低速・停車区間は同定対象から除外したうえで図上ではグレー表示する。
</p>

<details>
<summary>推定手法の詳細</summary>
<p>
純粋遅延 \\(T_a\\) は実装上、制御周期 \\(\\Delta t\\)（={_dt_long:.2f} s）の整数倍
（サンプル数換算 \\(n_{{\\mathrm{{delay}}}} = \\mathrm{{round}}(T_a / \\Delta t)\\)）でしか区別できない。
そこで \\(T_a\\) を
\\[
T_a \\in \\{{0,\\ \\Delta t,\\ 2\\Delta t,\\ \\dots,\\ M\\Delta t\\}},
\\qquad M = {_delay_m}\\ \\bigl(T_a: {_delay_lo:.2f}\\text{{–}}{_delay_hi:.2f}\\ \\mathrm{{s}}\\bigr)
\\]
の有限個の候補に固定した外側グリッドサーチとし、各候補ごとに連続値の時定数 \\(\\tau_a\\) を
内側の非線形最小二乗最適化で求める二段探索（grid search × 1 次元非線形最小二乗）を行う。
</p>

<p><b>① 固定 \\(T_a\\) に対するシミュレーション出力</b>: 純粋遅延ぶんシフト済みの指令
\\(u[k] = a_{{\\mathrm{{cmd}}}}[k - n_{{\\mathrm{{delay}}}}]\\) を入力として、1-1 節と同じ 1 次遅れ構造を持つ
離散フィルタ（指数移動平均 / 1 次 IIR）
\\[
a_{{\\mathrm{{sim}}}}[0] = a_{{\\mathrm{{act,corr}}}}[0], \\qquad
a_{{\\mathrm{{sim}}}}[k] = (1-\\alpha)\\, a_{{\\mathrm{{sim}}}}[k-1] + \\alpha\\, u[k]\quad(k\\ge 1),
\\qquad \\alpha := \\frac{{\\Delta t}}{{\\tau_a}} \\in (0, 1]
\\]
でシミュレーション軌道を生成する。初期値はログ切り出し開始時の実測補正加速度に合わせることで、
初期条件誤差を \\(\\tau_a, T_a\\) の同定誤差に混入させない。この漸化式を展開すると
\\[
a_{{\\mathrm{{sim}}}}[k] =
(1-\\alpha)^k a_{{\\mathrm{{act,corr}}}}[0]
+ \\alpha \\sum_{{i=1}}^{{k}} (1-\\alpha)^{{k-i}}\\, u[i]
\\]
となり、\\(a_{{\\mathrm{{sim}}}}\\) は \\(\\alpha\\)（≡ \\(\\tau_a\\)）のべき乗を通じて非線形に依存する。
このため output-error 規準（シミュレーション軌道と実測値の残差二乗和）は \\(\\tau_a\\) に関する線形回帰・
正規方程式には帰着できず、数値的な非線形最小二乗として解く必要がある。
</p>

<p><b>② 隣接サンプルからの直接逆算（点ごとの \\(\\tau_a\\) 推定）とその限界</b>: ①の漸化式
\\(a_{{\\mathrm{{sim}}}}[k] = (1-\\alpha)\\, a_{{\\mathrm{{sim}}}}[k-1] + \\alpha\\, u[k]\\) に実測値
\\(a_{{\\mathrm{{sim}}}}[k] \\approx a_{{\\mathrm{{act,corr}}}}[k]\\) を代入して \\(\\alpha\\)（＝\\(\\Delta t/\\tau_a\\)）について
直接解くと、
\\[
\\alpha = \\frac{{a_{{\\mathrm{{act,corr}}}}[k] - a_{{\\mathrm{{act,corr}}}}[k-1]}}{{u[k] - a_{{\\mathrm{{act,corr}}}}[k-1]}}
\\quad\\Longrightarrow\\quad
\\tau_a[k] = \\frac{{\\Delta t}}{{\\alpha}}
= \\Delta t\\, \\frac{{u[k] - a_{{\\mathrm{{act,corr}}}}[k-1]}}{{a_{{\\mathrm{{act,corr}}}}[k] - a_{{\\mathrm{{act,corr}}}}[k-1]}}
\\]
という「\\(\\tau_a = \\cdots\\)」の閉形式が各サンプル \\(k\\) ごとに得られる。これを代表データセット
（横断フィットの最良・最悪データセット）の全時刻についてプロットしたものが下図
（横軸: 時刻、縦軸: 点ごとの \\(\\tau_a[k]\\) 推定値、赤破線: 横断最小二乗法フィット値）である。
</p>
{long_tau_html}
<p>
分母 \\(a_{{\\mathrm{{act,corr}}}}[k] - a_{{\\mathrm{{act,corr}}}}[k-1]\\) が小さい区間（緩加速・定常走行）では
測定ノイズが増幅されて \\(\\tau_a[k]\\) が大きく散らばり、非物理的な値（負値・極端な発散）も生じる
（グラフでは負値・τ範囲外は除外して表示）。この不安定さが、単純な点ごとの逆算ではなく、
全時刻の残差二乗和を最小化する非線形最小二乗（次項③）が採用されている理由である。<br>
下図は最良・最悪の代表 2〜4 件だけでなく、横断フィットに使用した全データセット
（n_dyn 上位、最大 {_N_CROSS_FIT_DATASET} 件）をプールした瞬時 \\(\\tau_a[k]\\) 推定値のヒストグラムである。
分布の広がり・歪み・裾の長さが、代表データセットの散布図（上図）で見えていた散らばりが
局所的な現象ではなくデータセット横断で一貫した傾向であることを示す。
</p>
{long_tau_hist_html}

<p><b>③ 目的関数と対数変換による 1 次元探索</b>: 動的区間マスク \\(\\mathcal{{K}}\\)
（DRIVE ギア・走行中・\\(|\\dot a_{{\\mathrm{{cmd}}}}|\\) 閾値超過）上での残差二乗平均
\\[
J(\\tau_a;\\, T_a) = \\frac{{1}}{{|\\mathcal{{K}}|}} \\sum_{{k \\in \\mathcal{{K}}}}
\\bigl(a_{{\\mathrm{{sim}}}}[k;\\, \\tau_a, T_a] - a_{{\\mathrm{{act,corr}}}}[k]\\bigr)^2
\\]
を最小化する。\\(\\tau_a\\) の探索範囲（{_tau_lo:.2f}〜{_tau_hi:.3g} s）が 2 桁以上に及ぶため、
\\(\\theta = \\ln \\tau_a\\) と置換して
\\[
\\theta^*(T_a) = \\operatorname*{{arg\\,min}}_{{\\theta \\in [\\ln \\tau_{{\\min}},\\, \\ln \\tau_{{\\max}}]}}
J\\bigl(e^{{\\theta}};\\, T_a\\bigr)
\\]
を有界 1 次元非線形最小化（Brent 法, <code>scipy.optimize.minimize_scalar(method="bounded")</code>）で解き、
\\(\\tau_a^*(T_a) = e^{{\\theta^*(T_a)}}\\) を得る。対数空間にすることで正値制約を自動的に満たしつつ、
広いダイナミックレンジに対して均等な探索分解能を確保できる。
</p>

<p><b>④ 純粋遅延グリッドサーチ（外側ループ）</b>: ③の内側最小化を候補
\\(T_a = m\\Delta t\\)（\\(m = 0, \\dots, M\\)）ごとに実行し、
\\[
(\\tau_a^{{*}},\\, T_a^{{*}}) = \\operatorname*{{arg\\,min}}_{{T_a \\in \\{{0,\\Delta t,\\dots,M\\Delta t\\}}}}
J\\bigl(\\tau_a^*(T_a);\\, T_a\\bigr)
\\]
により大域最適な組を選ぶ。複数データセットを横断して同定する場合は、残差二乗和とサンプル数を
データセット間でプールした
\\[
J_{{\\mathrm{{pool}}}}(\\tau_a; T_a) =
\\frac{{\\sum_d \\sum_{{k \\in \\mathcal{{K}}_d}} \\bigl(a_{{\\mathrm{{sim}},d}}[k] - a_{{\\mathrm{{act,corr}},d}}[k]\\bigr)^2}}
{{\\sum_d |\\mathcal{{K}}_d|}}
\\]
を同じ手順で最小化し、単一の共有パラメータ \\((\\tau_a^*, T_a^*)\\) を得る。
</p>
</details>
{long_html}

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量（RMSE）</th></tr>
  <tr><td><code>acc_time_constant</code> (\\(\\tau_a\\))</td><td>{tau_a} s</td>
      <td>一次遅れ時定数：小さいほど加速応答が速い</td><td rowspan="2">\\(a_{{\\mathrm{{sim}}}} - a_{{\\mathrm{{act,corr}}}}\\)</td></tr>
  <tr><td><code>acc_time_delay</code> (\\(T_a\\))</td><td>{T_a} s</td>
      <td>純粋遅延：指令が実際に入力されるまでの遅延時間</td></tr>
</table>

{long_perf_subsection}
</section>

<section id="sec-steer">
<h2>1-3. 操舵アクチュエータ（追従ループ）の同定</h2>
<p class="meta">
&#128279; <a href="#sec-state-space">1-1 状態方程式</a> の \\(\\dot\\delta_{{\\mathrm{{act}}}}\\) 行に対応。
</p>
<p>
操舵追従ループは実車位置・ヨーのフィードバックを持たないオープンループなので、
\\(\\text{{err}}_{{\\mathrm{{steer}}}}\\) を目的関数として位置・ヨー誤差とは構造的に独立して同定できる。
</p>

<p><b>運動方程式（1-1 と同じ状態・入力表記）:</b></p>
\\[
{{\\color{{#1565c0}} \\dot\\delta_{{\\mathrm{{act}}}}}}(t)
= \\dfrac{{{{\\color{{#e65100}} \\delta_{{\\mathrm{{des}}}}}}(t) - {{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}(t)}}{{\\tau_\\delta}},
\\qquad
{{\\color{{#e65100}} \\delta_{{\\mathrm{{des}}}}}}(t)
= K_{{\\mathrm{{steer\\_scale}}}} \\cdot {{\\color{{#e65100}} \\delta_{{\\mathrm{{cmd}}}}}}(t - T_\\delta)
\\]
<p><b>式中の定数・補足:</b></p>
<ul>
  <li>\\(T_\\delta\\)（<code>steer_time_delay</code>）: 操舵指令 \\(\\delta_{{\\mathrm{{cmd}}}}\\) がアクチュエータに届くまでの純粋遅延。</li>
  <li>\\(\\tau_\\delta\\)（<code>steer_time_constant</code>）: 目標操舵角 \\(\\delta_{{\\mathrm{{des}}}}\\) に対する実舵角 \\(\\delta_{{\\mathrm{{act}}}}\\) の一次遅れ時定数。</li>
  <li>\\(K_{{\\mathrm{{steer\\_scale}}}}\\)（<code>debug_steer_scaling_factor</code>）: 遅延後の操舵指令に乗算する定数ゲイン。</li>
  <li>制限値・不感帯: 目標操舵角は <code>steer_lim</code>、操舵速度は <code>steer_rate_lim</code>、不感帯は <code>steer_dead_band</code>。</li>
  <li>\\(\\beta\\)（<code>steer_bias</code>）: アクチュエータ追従式の外側で扱うバイアス。報告操舵角と 1-4 のヨー式への加算値。</li>
</ul>
<div class="note">
⚠️ <b>結合点</b>: 操舵ゲイン補正倍率（debug_steer_scaling_factor）は操舵指令に定数ゲインをかける形で、直進時の系統的な横力成分（v²δ 由来の
アンダーステア成分）を部分的に吸収できる。
したがって操舵ゲイン補正倍率の最適値は \\(k_{{\\mathrm{{us}}}}\\) の同定後に再検証することが望ましい。
</div>

<h3>実機ログからの独立同定（代表データセットモデルフィット）</h3>
<p>
\\(\\delta_{{\\mathrm{{cmd}}}}\\) を入力として遅延グリッドサーチ + output-error 非線形最小二乗法で
データセット別に同定した \\((\\tau_\\delta, T_\\delta)\\) のモデル出力（青点線）と実測 \\(\\delta_{{\\mathrm{{act}}}}\\)（黒実線）を比較する。
チューニング値でのシミュレーション結果（破線）も重ね描きする。per-dataset 同定誤差（RMSE）の
最良・最悪データセットを選択し、低速・停車区間は除外して表示。
</p>
{steer_html}
<div class="note">
⚠️ <b>注記</b>: <code>cmd_steer</code> は understeer converter 適用前（コントローラ出力）、
<code>delta_act</code> は converter 適用後の実舵角である。定常ゲインは速度依存の補正を含むため、
時定数 \\(\\tau_\\delta\\) と遅延 \\(T_\\delta\\) の分布は quasi-static で有効だが、ゲイン推定は低速帯（v &lt; 3 m/s）に限定して評価することが望ましい。
</div>

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量（RMSE）</th></tr>
  <tr><td><code>steer_time_constant</code> (\\(\\tau_\\delta\\))</td><td>{tau_d} s</td>
      <td>一次遅れ時定数：小さいほど操舵応答が速い</td><td rowspan="4">\\(\\text{{err}}_{{\\mathrm{{steer}}}}\\)</td></tr>
  <tr><td><code>steer_time_delay</code> (\\(T_\\delta\\))</td><td>{T_d} s</td>
      <td>純粋遅延：操舵指令が実際に入力されるまでの遅延時間</td></tr>
  <tr><td><code>debug_steer_scaling_factor</code> (操舵ゲイン補正倍率)</td><td>{DSF}</td>
      <td>指令スケーリング（1.0 = 補正なし）；遅延後に乗算</td></tr>
  <tr><td><code>steer_bias</code> (\\(\\beta\\))</td><td>{beta} rad</td>
      <td>報告操舵角への加算値（\\(\\delta_{{\\mathrm{{act}}}} + \\beta\\)）；1-4 のヨー式と共有</td></tr>
  <tr><td><code>steer_dead_band</code></td><td>{db} rad</td>
      <td>不感帯幅（固定値・同定対象外）</td><td>—</td></tr>
  <tr><td><code>steer_rate_lim</code></td><td>{rlim} rad/s</td>
      <td>操舵レート飽和（固定値・同定対象外）</td><td>—</td></tr>
</table>
</section>
<section id="sec-yaw">
<h2>1-4. ヨー・横方向（運動学的自転車モデル）— 速度ビン別 最小二乗法同定</h2>
<p class="meta">
&#128279; <a href="#sec-state-space">1-1 状態方程式</a> の \\(\\dot\\theta, \\dot x, \\dot y\\) 行に対応。
</p>
<p>
1-2（縦方向）・1-3（操舵追従）が先行して確定した後、
\\(\\text{{err}}_{{wz}}\\) を目的関数として <b>高曲率サブセット</b> で \\(k_{{\\mathrm{{us}}}}\\) を同定する。
直進（\\(\\delta \\approx 0\\)）では感度がゼロなので、全データセット集約スコアは \\(k_{{\\mathrm{{us}}}}\\) に対して構造的不可同定。
</p>

<p><b>運動方程式（1-1 と同じ状態表記）:</b></p>
\\[
\\begin{{pmatrix}}
{{\\color{{#1565c0}} \\dot x}} \\\\
{{\\color{{#1565c0}} \\dot y}} \\\\
{{\\color{{#1565c0}} \\dot\\theta}}
\\end{{pmatrix}}
=
\\begin{{pmatrix}}
{{\\color{{#1565c0}} v_x}} \\cos{{\\color{{#1565c0}} \\theta}} \\\\
{{\\color{{#1565c0}} v_x}} \\sin{{\\color{{#1565c0}} \\theta}} \\\\
\\dfrac{{{{\\color{{#1565c0}} v_x}}\\,\\tan({{\\color{{#1565c0}} \\delta_{{\\mathrm{{act}}}}}}+\\beta)}}{{L + k_{{\\mathrm{{us,eff}}}}({{\\color{{#1565c0}} v_x}})\\,{{\\color{{#1565c0}} v_x}}^2}}
\\end{{pmatrix}}
\\]
<p><b>式中の定数・補足:</b></p>
<ul>
  <li>\\(L\\)（<code>wheelbase</code>）: 自転車モデルのホイールベース。</li>
  <li>\\(k_{{\\mathrm{{us,eff}}}}\\)（<code>k_us_bands / k_us</code>）: 速度帯ごとのアンダーステア係数。</li>
  <li>\\(\\beta\\)（<code>steer_bias</code>）: ヨー式の \\(\\tan(\\cdot)\\) 引数に入るバイアス。系統的なヨーオフセット成分。</li>
  <li>\\(\\delta_{{\\mathrm{{act}}}}\\): 1-3 の操舵追従結果。操舵ゲイン補正倍率による \\(k_{{\\mathrm{{us}}}}\\) の \\(v_x^2\\delta\\) 成分の部分吸収に注意。</li>
</ul>

<h3>実機ログからの独立同定（全 {n_dataset} データセット、速度ビン別 最小二乗法）</h3>

<details>
<summary>推定手法の詳細</summary>
<p>
定常旋回フィルタ（\\(|\\omega| > {WZ_MIN}\\) rad/s、\\(|\\dot{{\\omega}}| < {DWZ_MAX}\\) rad/s²、
\\(v_x > {VX_MIN_CURVE}\\) m/s）を通過した各タイムステップを速度ビンに割り当て、
ビン内で以下の2種類の推定を行う。
</p>
<p><b>① 最小二乗法推定（青丸・実線）</b>: 原点回帰 \\(\\tan(\\delta_{{\\mathrm{{eff}}}}) = C \\cdot \\omega\\) の最小二乗解
\\[
C_{{\\mathrm{{OLS}}}} = \\frac{{\\sum \\omega_i \\, \\tan(\\delta_i)}}{{\\sum \\omega_i^2}},
\\qquad
\\hat{{k}}_{{\\mathrm{{us}}}} = \\frac{{C_{{\\mathrm{{OLS}}}} - L / \\bar{{v}}_x}}{{\\bar{{v}}_x}}
\\]
ここで \\(\\bar{{v}}_x\\) はビン内の速度中央値、\\(L = {WHEELBASE}\\) m はホイールベース。
</p>
<p><b>② 個別サンプル（IQR バンド）</b>: 実機運動学ログの各タイムステップで瞬時 \\(k_{{\\mathrm{{us}}}}\\) を推定し、
ビン内の 25〜75 パーセンタイルをバンドとして表示:
\\[
\\tilde{{k}}_{{\\mathrm{{us}}}}[i] = \\frac{{\\tan(\\delta_i) / \\omega_i - L / v_{{x,i}}}}{{v_{{x,i}}}}
\\]
</p>
</details>
{kus_html}
<div class="note">
<b>解釈</b>: 最小二乗法推定値が低速ビンでほぼ 0、高速ビンで正の値に推移していれば、
ランプ形状は物理的実態と整合している。ただし J6 の多くのデータセットが低速（\\(v_x\\) mean ≈ 1.9 m/s）
のため、高速ビンのサンプル数は少なく推定誤差が大きい点に注意（右パネルのサンプル数を参照）。
</div>

<table class="param-table">
  <tr><th>パラメータ</th><th>値</th><th>式中の役割</th><th>同定誤差量</th></tr>
{kus_rows}
  <tr><td><code>steer_bias</code> (\\(\\beta\\)) <i>[1-3 と共有]</i></td><td>{beta} rad</td>
      <td>\\(\\tan(\\delta_{{\\mathrm{{act}}}} + \\beta)\\) の引数：ヨーオフセット成分</td><td>\\(\\text{{err}}_{{wz}}\\)（間接）</td></tr>
</table>
</section>
"""


def _build_sec14(
    fig_box: go.Figure,
    fig_traj: go.Figure,
    params: dict,
    n_dataset: int,
) -> str:
    """1-5. モデル構造限界（理想追従評価）セクション HTML。"""
    box_html  = fig_box.to_html(full_html=False, include_plotlyjs=False)
    traj_html = fig_traj.to_html(full_html=False, include_plotlyjs=False)
    tau_a = f"{params.get('acc_time_constant', float('nan')):.3g}"
    T_a   = f"{params.get('acc_time_delay', float('nan')):.3g}"
    tau_d = f"{params.get('steer_time_constant', float('nan')):.3g}"
    T_d   = f"{params.get('steer_time_delay', float('nan')):.3g}"
    h_str = ", ".join(f"N={h}（{h * _FIT_DT:.2f}s）" for h in _PERF_HORIZONS)
    return f"""
<section id="sec-perf-tracking">
<h2>1-5. モデル構造限界（理想追従評価）</h2>
<p>
アクチュエータ追従が完璧だった場合（実測 \\(v_x\\) と実測 \\(\\delta_{{\\mathrm{{act}}}}\\) を
自転車モデルの直接入力として使用）に残る位置ずれを評価する。
これにより <b>アクチュエータ遅れの寄与</b> と <b>モデル構造外の寄与</b>（タイヤスリップ、路面バンク、
横速度 \\(v_y\\)、\\(k_{{\\mathrm{{us}}}}\\) キャリブレーション誤差）を分離できる。
</p>
<p>
現行スコアとの差分 ≈ アクチュエータ応答が占める誤差分
（\\(\\tau_a\\)={tau_a} s, \\(T_a\\)={T_a} s, \\(\\tau_\\delta\\)={tau_d} s, \\(T_\\delta\\)={T_d} s の合算効果）。
</p>
<div class="note">
⚠️ <b>設計上の帰結</b>:
縦方向誤差は \\(v_x\\) を実車ログから直接取得しているため積分上ほぼゼロになる。
<b>横方向誤差のみが真のモデル構造限界を表す。</b><br>
残差は「現行チューン値 \\(k_{{\\mathrm{{us}}}}\\) および \\(\\beta\\) での理想追従誤差」であるため、
パラメータのキャリブレーション誤差も一部含む（現行パラメータ前提での下限値）。
</div>

<h3>横方向誤差分布（ホライズン別、上位 {n_dataset} データセット）</h3>
<p>
カーブ走行区間（\\(v_x > {VX_MIN_CURVE}\\) m/s）を stride={_PERF_STRIDE} ステップで走査し、
N-step ロールアウト終端の横方向誤差絶対値を集計する。ホライズン: {h_str}。
</p>
{box_html}

<h3>代表データセット の軌跡比較（実車 vs 自転車モデル）</h3>
<p>
初期状態を実車ログに合わせ、実測 \\(v_x\\) と \\(\\delta_{{\\mathrm{{act}}}}\\) を入力として積分した
自転車モデル軌跡（青破線）を実車の軌跡（黒実線）と比較する。
リセットなしの連続積分であるため、後半の乖離はモデル構造誤差の累積を示す。
座標は初期位置を原点 (0, 0) に正規化している。
</p>
{traj_html}
</section>
"""


def _build_sec2(kus_fig: go.Figure, n_dataset: int) -> str:
    kus_html = kus_fig.to_html(full_html=False, include_plotlyjs=False)
    return f"""
<section id="identification">
<h2>2. 実機ログからの独立同定</h2>

<details open>
<summary>2-1. アンダーステア係数 \\(k_{{\\mathrm{{us}}}}\\) の速度依存性（全 {n_dataset} データセット）</summary>
<details>
<summary>推定手法の詳細</summary>
<p>
定常旋回フィルタ（\\(|\\omega| > {WZ_MIN}\\) rad/s、\\(|\\dot{{\\omega}}| < {DWZ_MAX}\\) rad/s²、
\\(v_x > {VX_MIN_CURVE}\\) m/s）を通過した各タイムステップを速度ビンに割り当て、
ビン内で以下の2種類の推定を行う。
</p>
<p><b>① 最小二乗法推定（青丸・実線）</b>: 原点回帰 \\(\\tan(\\delta_{{\\mathrm{{eff}}}}) = C \\cdot \\omega\\) の最小二乗解
\\[
C_{{\\mathrm{{OLS}}}} = \\frac{{\\sum \\omega_i \\, \\tan(\\delta_i)}}{{\\sum \\omega_i^2}},
\\qquad
\\hat{{k}}_{{\\mathrm{{us}}}} = \\frac{{C_{{\\mathrm{{OLS}}}} - L / \\bar{{v}}_x}}{{\\bar{{v}}_x}}
\\]
ここで \\(\\bar{{v}}_x\\) はビン内の速度中央値、\\(L = {WHEELBASE}\\) m はホイールベース。
</p>
<p><b>② 個別サンプル（IQR バンド）</b>: 実機運動学ログの各タイムステップで瞬時 \\(k_{{\\mathrm{{us}}}}\\) を推定し、
ビン内の 25〜75 パーセンタイルをバンドとして表示:
\\[
\\tilde{{k}}_{{\\mathrm{{us}}}}[i] = \\frac{{\\tan(\\delta_i) / \\omega_i - L / v_{{x,i}}}}{{v_{{x,i}}}}
\\]
チューニング済みランプ曲線（橙色破線）と重ね描きして形状の妥当性を確認する。
</p>
</details>
{kus_html}
<div class="note">
<b>解釈</b>: 最小二乗法推定値が低速ビンでほぼ 0、高速ビンで正の値に推移していれば、
ランプ形状は物理的実態と整合している。ただし J6 の多くのデータセットが低速（\\(v_x\\) mean ≈ 1.9 m/s）
のため、高速ビンのサンプル数は少なく推定誤差が大きい点に注意（右パネルのサンプル数を参照）。
</div>
</details>
</section>
"""


def _build_sec3(viewer_sections: list[str], label: str = "phase14") -> str:
    body = "\n".join(viewer_sections) if viewer_sections else "<p>ビューア生成対象 データセット なし</p>"
    return f"""
<section id="curve-viewer">
<h2>3. カーブ部での実機 vs モデル軌跡（インタラクティブビューア）</h2>
<p>
旋回イベント数 <code>curve_count</code>（\\(|\\kappa| > 0.02\\) m⁻¹、連続弧長 ≥ 10 m）が多い
代表データセットについて縦横モデル検証ビューアを埋め込む。
ドロップダウンで <b>{label}</b>（\\(k_{{\\mathrm{{us}}}}\\) ランプ＋<code>steer_dead_band</code> 有効）と
<b>baseline</b>（\\(k_{{\\mathrm{{us}}}}=0\\), <code>steer_dead_band</code>=0）を切り替えて実機軌跡への一致を比較できる。
</p>
{body}
</section>
"""


def _build_sec_deviation(
    df: pd.DataFrame,
    n_dataset: int,
    recomputed_score: float | None = None,
    expected_score: float | None = None,
    score_name: str = "robust_score",
    label: str = "phase14",
) -> str:
    """N-step 終端誤差の最大乖離テーブルセクション（tuned vs baseline）。"""
    horizons = sorted(df["h"].unique().tolist())

    stats_list = []
    for h in horizons:
        sub = df[df["h"] == h]
        stats_list.append({
            "h": h,
            "p14_yaw_mean": sub["p14_yaw"].mean(), "p14_yaw_p95": sub["p14_yaw"].quantile(0.95), "p14_yaw_p99": sub["p14_yaw"].quantile(0.99), "p14_yaw_max": sub["p14_yaw"].max(),
            "p14_lat_mean": sub["p14_lat"].mean(), "p14_lat_p95": sub["p14_lat"].quantile(0.95), "p14_lat_p99": sub["p14_lat"].quantile(0.99), "p14_lat_max": sub["p14_lat"].max(),
            "p14_long_mean": sub["p14_long"].mean(), "p14_long_p95": sub["p14_long"].quantile(0.95), "p14_long_p99": sub["p14_long"].quantile(0.99), "p14_long_max": sub["p14_long"].max(),
            "p14_vx_mean": sub["p14_vx"].mean(), "p14_vx_p95": sub["p14_vx"].quantile(0.95), "p14_vx_p99": sub["p14_vx"].quantile(0.99), "p14_vx_max": sub["p14_vx"].max(),
            "bl_yaw_mean": sub["bl_yaw"].mean(), "bl_yaw_p95": sub["bl_yaw"].quantile(0.95), "bl_yaw_p99": sub["bl_yaw"].quantile(0.99), "bl_yaw_max": sub["bl_yaw"].max(),
            "bl_lat_mean": sub["bl_lat"].mean(), "bl_lat_p95": sub["bl_lat"].quantile(0.95), "bl_lat_p99": sub["bl_lat"].quantile(0.99), "bl_lat_max": sub["bl_lat"].max(),
            "bl_long_mean": sub["bl_long"].mean(), "bl_long_p95": sub["bl_long"].quantile(0.95), "bl_long_p99": sub["bl_long"].quantile(0.99), "bl_long_max": sub["bl_long"].max(),
            "bl_vx_mean": sub["bl_vx"].mean(), "bl_vx_p95": sub["bl_vx"].quantile(0.95), "bl_vx_p99": sub["bl_vx"].quantile(0.99), "bl_vx_max": sub["bl_vx"].max(),
        })

    score_html = ""
    if recomputed_score is not None and expected_score is not None:
        diff_pct = abs(recomputed_score - expected_score) / expected_score * 100 if expected_score else 0.0
        ok = diff_pct < 2.0
        color = "#28a745" if ok else "#dc3545"
        score_html = (
            f'<div class="note" style="border-color:{color}">'
            f"スコア再現検証 (<code>{score_name}</code>): 再計算 = <b>{recomputed_score:.4f}</b>、"
            f"YAML 期待値 = <b>{expected_score:.4f}</b>（差 {diff_pct:.2f}%）"
            + (" — ✓ 整合" if ok else " — ⚠ 不整合（override/model/SUB_DT を確認）")
            + "</div>"
        )

    def _cell(p14_val: float, bl_val: float, fmt: str = ".3f") -> str:
        ratio = p14_val / bl_val if bl_val > 0 else 1.0
        if ratio < 0.99:
            style = ' style="color:#28a745;font-weight:bold"'
        elif ratio > 1.01:
            style = ' style="color:#dc3545"'
        else:
            style = ""
        return f"<td{style}>{p14_val:{fmt}}</td>"

    tbody_rows = []
    for s in stats_list:
        h = s["h"]
        span = _H_SPAN.get(h, "")
        tbody_rows.append(
            f'<tr>\n'
            f'  <td rowspan="2" style="text-align:center"><b>N={h}</b><br>'
            f'<small style="color:#888">{span}</small></td>\n'
            f'  <td><b>{label}</b></td>\n'
            f'  {_cell(s["p14_yaw_mean"], s["bl_yaw_mean"])}'
            f'{_cell(s["p14_yaw_p95"], s["bl_yaw_p95"])}'
            f'{_cell(s["p14_yaw_p99"], s["bl_yaw_p99"])}'
            f'{_cell(s["p14_yaw_max"], s["bl_yaw_max"])}\n'
            f'  {_cell(s["p14_lat_mean"], s["bl_lat_mean"])}'
            f'{_cell(s["p14_lat_p95"], s["bl_lat_p95"])}'
            f'{_cell(s["p14_lat_p99"], s["bl_lat_p99"])}'
            f'{_cell(s["p14_lat_max"], s["bl_lat_max"])}\n'
            f'  {_cell(s["p14_long_mean"], s["bl_long_mean"])}'
            f'{_cell(s["p14_long_p95"], s["bl_long_p95"])}'
            f'{_cell(s["p14_long_p99"], s["bl_long_p99"])}'
            f'{_cell(s["p14_long_max"], s["bl_long_max"])}\n'
            f'  {_cell(s["p14_vx_mean"], s["bl_vx_mean"], ".4f")}'
            f'{_cell(s["p14_vx_p95"], s["bl_vx_p95"], ".4f")}'
            f'{_cell(s["p14_vx_p99"], s["bl_vx_p99"], ".4f")}'
            f'{_cell(s["p14_vx_max"], s["bl_vx_max"], ".4f")}\n'
            f'</tr>\n'
            f'<tr>\n'
            f'  <td style="color:#888">baseline</td>\n'
            f'  <td>{s["bl_yaw_mean"]:.3f}</td><td>{s["bl_yaw_p95"]:.3f}</td><td>{s["bl_yaw_p99"]:.3f}</td><td>{s["bl_yaw_max"]:.3f}</td>\n'
            f'  <td>{s["bl_lat_mean"]:.3f}</td><td>{s["bl_lat_p95"]:.3f}</td><td>{s["bl_lat_p99"]:.3f}</td><td>{s["bl_lat_max"]:.3f}</td>\n'
            f'  <td>{s["bl_long_mean"]:.3f}</td><td>{s["bl_long_p95"]:.3f}</td><td>{s["bl_long_p99"]:.3f}</td><td>{s["bl_long_max"]:.3f}</td>\n'
            f'  <td>{s["bl_vx_mean"]:.4f}</td><td>{s["bl_vx_p95"]:.4f}</td><td>{s["bl_vx_p99"]:.4f}</td><td>{s["bl_vx_max"]:.4f}</td>\n'
            f'</tr>'
        )

    tbody = "\n".join(tbody_rows)

    return f"""
<section id="deviation">
<h2>N-step 終端誤差（{label} vs baseline）</h2>
<p>
全データセットに対し {label} パラメータと baseline（補正なし）で N-step ロールアウトを実施し、
終端誤差 RMSE の データセット横断 <b>平均</b>（mean）、<b>95パーセンタイル</b>（95%点）、<b>99パーセンタイル</b>（99%点）、<b>最大</b>（worst-case データセット）を N ごとに集計する。
「最大」は「最も誤差が大きかった データセットの RMSE」を指す。
</p>
<p>
最大乖離（Worst値）は突発的なノイズ等で大きくなる場合がありますが、<b>95%点や99%点</b>の指標を見ることで、大部分の走行区間でモデルが極めて高い精度（OKの範囲内）で適合していることを確認できます。
</p>
{score_html}
<table class="param-table" style="font-size:12px">
  <thead>
    <tr>
      <th rowspan="2">N（時間）</th>
      <th rowspan="2">モデル</th>
      <th colspan="4">yaw 誤差 [deg]</th>
      <th colspan="4">lat 誤差 [cm]</th>
      <th colspan="4">long 誤差 [cm]</th>
      <th colspan="4">速度誤差 \\(v_x\\) [m/s]</th>
    </tr>
    <tr>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
      <th>平均</th><th>95%点</th><th>99%点</th><th>最大</th>
    </tr>
  </thead>
  <tbody>
{tbody}
  </tbody>
</table>
<div class="note">
{label} の値が baseline より小さい場合は <b style="color:#28a745">緑（改善）</b>、
大きい場合は <span style="color:#dc3545">赤（悪化）</span> で表示。
RMSE は各データセットの全 k0 ステップ（stride=5）の終端誤差（N ステップ先）の二乗平均平方根。
キャッシュは <code>--metrics-cache</code> で指定した CSV ファイルに保存される。
</div>
</section>
"""


def _build_sec_closed_loop_comparison(collection_dir: Path, uuids_str: str) -> str:
    if not uuids_str:
        return ""
    
    uuids = [u.strip() for u in uuids_str.split(",") if u.strip()]
    sections = []
    
    for uuid in uuids:
        target_dirs = [
            collection_dir / uuid,
            collection_dir / "datasets" / uuid,
        ]
        
        found_dir = None
        for d in target_dirs:
            if d.exists():
                found_dir = d
                break
        
        if not found_dir:
            print(f"  [WARN] クローズドループ結果ディレクトリが見つかりません (UUID: {uuid})")
            continue
            
        metrics_json = found_dir / "metrics_closed_loop.json"
        playback_html = found_dir / "figures" / "viewer.html"
        
        if not playback_html.exists():
            playback_html_list = list(found_dir.glob("**/viewer.html"))
            if playback_html_list:
                playback_html = playback_html_list[0]
            
        metrics_tbl = ""
        if metrics_json.exists():
            try:
                import json
                m = json.loads(metrics_json.read_text(encoding="utf-8"))
                rows = []
                for run_name, run_data in m.get("runs", {}).items():
                    rows.append(f"""
                      <tr>
                        <td><b>{run_name}</b></td>
                        <td>{run_data.get('steer_rmse_deg', 'N/A')}</td>
                        <td>{run_data.get('vel_rmse_mps', 'N/A')}</td>
                      </tr>
                    """)
                if rows:
                    metrics_tbl = f"""
                    <table class="param-table" style="font-size:12px; margin-bottom:10px;">
                      <thead>
                        <tr>
                          <th>シミュレーションモデル</th>
                          <th>操舵 RMSE [deg]</th>
                          <th>速度 RMSE [m/s]</th>
                        </tr>
                      </thead>
                      <tbody>
                        {"".join(rows)}
                      </tbody>
                    </table>
                    """
            except Exception as e:
                print(f"  [WARN] metrics_closed_loop.json 読み込み失敗: {e}")
                
        iframe_html = ""
        if playback_html and playback_html.exists():
            try:
                content = playback_html.read_text(encoding="utf-8")
                srcdoc = _html_stdlib.escape(content, quote=True)
                iframe_html = f"""
                <iframe srcdoc="{srcdoc}"
                  width="100%" height="1000"
                  style="border:1px solid #ccc;border-radius:4px"
                  loading="lazy"></iframe>
                """
            except Exception as e:
                print(f"  [WARN] viewer.html 読み込み失敗: {e}")
                
        if iframe_html or metrics_tbl:
            sections.append(f"""
            <h3>Dataset: <code>{uuid}</code></h3>
            {metrics_tbl}
            {iframe_html}
            """)
            
    if not sections:
        return ""
        
    body = "\n".join(sections)
    return f"""
<section id="sec-closed-loop">
<h2>4. クローズドループシミュレーション比較</h2>
<p>
指定されたデータセットについて、実機走行ログ vs クローズドループシミュレーションによる走行結果の比較（軌跡再生ビューア）を提示します。
</p>
{body}
</section>
"""


def build_html(
    params: dict,
    long_fig: go.Figure,
    steer_fig: go.Figure,
    kus_fig: go.Figure,
    viewer_sections: list[str],
    n_dataset: int,
    baseline_score: float | None = None,
    deviation_html: str = "",
    label: str = "current",
    params_filename: str = "",
    perf_html: str = "",
    long_perf_figs: tuple[go.Figure, go.Figure, go.Figure] | None = None,
    closed_loop_html: str = "",
    long_tau_fig: go.Figure | None = None,
    long_tau_hist_fig: go.Figure | None = None,
) -> str:
    score = params.get("_score", "N/A")
    phase14_score = float(score) if isinstance(score, (int, float, str)) and str(score) != "N/A" else 0.0
    sec_metrics = _build_sec_metrics(baseline_score=baseline_score, phase14_score=phase14_score, label=label)
    
    sec_tuning = f"""
<section id="sec-tuning">
<h2>2. 統合最適化（パラメータ最適化）</h2>
<p>
各モデルの独立最適化パラメータをベースにした、全データセット横断での統合最適化の結果を評価します。
</p>
{sec_metrics}
{deviation_html}
</section>
"""

    sec1 = _build_sec1(
        params, long_fig, steer_fig, kus_fig, n_dataset,
        long_perf_figs=long_perf_figs, long_tau_fig=long_tau_fig, long_tau_hist_fig=long_tau_hist_fig,
    )
    sec3 = _build_sec3(viewer_sections, label=label)

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>物理的妥当性レポート — {label}</title>
  {_MATHJAX_HEAD}
  {_PLOTLY_CDN}
  <style>{_STYLE}</style>
</head>
<body>
<h1>車両モデル物理的妥当性検証レポート — {label}</h1>
<p class="meta">
  生成元: <code>{params_filename or label}</code> &nbsp;|&nbsp;
  score: {score} &nbsp;|&nbsp;
  有効データセット数: {n_dataset}
</p>
<nav>
  <a href="#sec-coords">1-0. 座標系定義</a>
  <a href="#sec-state-space">1-1. 状態空間モデルと数値積分</a>
  <a href="#sec-long">1-2. 縦方向</a>
  <a href="#sec-steer">1-3. 操舵</a>
  <a href="#sec-yaw">1-4. ヨー・横方向</a>
  <a href="#sec-tuning">2. 統合最適化</a>
  {f'<a href="#sec-perf-tracking">1-5. モデル構造限界</a>' if perf_html else ""}
  <a href="#curve-viewer">3. カーブビューア</a>
  {f'<a href="#sec-closed-loop">4. クローズドループ比較</a>' if closed_loop_html else ""}
</nav>
{sec1}
{sec_tuning}
{perf_html}
{sec3}
{closed_loop_html}
</body>
</html>
"""


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def _find_first_curve_t(ctx, pre_roll_s: float = 5.0) -> float:
    """DatasetCtx の運動学データから最初のカーブ開始時刻を検出し、pre_roll_s 秒前を返す。"""
    kin = ctx.data["kin"]
    if kin.empty:
        return 0.0
    t_ns = kin["t_ns"].values
    t_rel = (t_ns - ctx.t0_ns) * 1e-9
    vx = kin["vx"].values
    wz = kin["wz"].values
    with np.errstate(divide="ignore", invalid="ignore"):
        kappa = np.where(vx > 0.5, np.abs(wz / vx), 0.0)
    in_curve = kappa > 0.02
    # 5 フレーム以上連続でカーブ条件を満たす最初の点
    for i in range(len(t_rel) - 5):
        if in_curve[i : i + 5].all():
            return float(max(0.0, t_rel[i] - pre_roll_s))
    return 0.0


def main() -> None:
    ap = argparse.ArgumentParser(description="物理的妥当性レポート生成")
    ap.add_argument(
        "--params", type=Path, required=True,
        help="チューニング済みパラメータ YAML (tuned_params.yaml)",
    )
    ap.add_argument(
        "--collection-dir", type=Path, required=True,
        help="real.lite 群を収集した collection ディレクトリ",
    )
    ap.add_argument(
        "--out", type=Path, required=True,
        help="出力 HTML パス",
    )
    ap.add_argument("--n-curve-ds", type=int, default=3, help="ビューア埋め込みカーブ データセット数")
    ap.add_argument("--n-jobs", type=int, default=8)
    ap.add_argument(
        "--viewer-uuids", type=str, default=None,
        help="ビューアに使う データセット UUID をカンマ区切りで指定（省略時は curve_count 上位を自動選択）",
    )
    ap.add_argument(
        "--pinned-uuids", type=str, default="",
        help="ビューアに必ず含める データセット UUID をカンマ区切りで指定（前方一致）。--viewer-uuids や自動選択より優先して先頭に配置",
    )
    ap.add_argument(
        "--metrics-cache", type=Path, default=None,
        help=(
            "rollout メトリクス CSV キャッシュパス（指定時のみ偏差テーブルを生成）。"
            "ファイルが存在すれば読み込み、なければ全データセット rollout を実行して保存する。"
        ),
    )
    ap.add_argument(
        "--extra-ds", type=Path, nargs="*", default=[],
        help="collection-dir 外から MCAP 解析・ビューアに追加する データセット ディレクトリ（複数指定可）",
    )
    ap.add_argument(
        "--label", type=str, default="current",
        help="レポート内のモデル名ラベル（デフォルト: current）",
    )
    ap.add_argument(
        "--ds-after", type=str, default=None,
        help="この日付以降のデータセットのみ使用（YYYY-MM-DD形式、例: 2026-06-16）",
    )
    ap.add_argument(
        "--ds-before", type=str, default=None,
        help="この日付より前のデータセットのみ使用（YYYY-MM-DD形式）",
    )
    ap.add_argument(
        "--closed-loop-uuids", type=str, default="",
        help="クローズドループ比較をレポートに含めるデータセット UUID をカンマ区切りで指定",
    )
    args = ap.parse_args()

    phase_label = args.label

    with open(args.params) as f:
        yaml_data = yaml.safe_load(f)
    params: dict = yaml_data.get("params", yaml_data)
    params["_score"] = yaml_data.get("score", "N/A")
    print(f"パラメータ: {args.params.name}  (label={phase_label})")
    # k_us 速度帯表示（新形式 / 後方互換形式 両対応）
    bands = params.get("k_us_bands")
    thresholds = params.get("k_us_thresholds")
    if bands is not None and thresholds is not None:
        band_str = " | ".join(
            f"band[{i}]={b:.5f}" for i, b in enumerate(bands)
        )
        thr_str = " | ".join(f"thr[{i}]={t:.2f}" for i, t in enumerate(thresholds))
        print(f"  k_us 速度帯: {band_str}")
        print(f"  閾値: {thr_str} m/s")
    else:
        print(f"  k_us={params.get('k_us', 0):.5f} (速度依存なし)")
    print(f"  steer_dead_band={params.get('steer_dead_band',0):.5f} rad")

    # データセット列挙
    import datetime as _dt
    ds_list = _discover(args.collection_dir)
    for extra in (args.extra_ds or []):
        uuid = extra.name
        if not any(u == uuid for u, _ in ds_list):
            ds_list.append((uuid, extra))
            print(f"  [extra-ds] {uuid} を追加")
    ds_after_date  = _dt.date.fromisoformat(args.ds_after)  if args.ds_after  else None
    ds_before_date = _dt.date.fromisoformat(args.ds_before) if args.ds_before else None
    if ds_after_date or ds_before_date:
        ds_list = _filter_by_date(ds_list, ds_before_date, ds_after_date)
    print(f"\nデータセット: {len(ds_list)} 件")

    # Phase 1: 並列 MCAP 読み込み
    print("\n[Phase 1] MCAP 並列読み込み ...")
    records = load_all_mcap(ds_list, n_jobs=args.n_jobs)
    print(f"  有効: {len(records)} 件")

    # Phase 2: k_us 速度ビン別 最小二乗法
    print("\n[Phase 2] k_us 速度ビン別 最小二乗法 ...")
    bins = compute_kus_bins(records)
    n_valid = int(np.isfinite(bins["kus_ols"]).sum())
    print(f"  有効速度ビン: {n_valid}/{len(bins['kus_ols'])}")

    # Phase 2b: 縦方向 / 操舵 per-dataset 実行時フィット（旧 identify_*_dynamics.py の
    # 事前 CSV 生成を置き換え。best/worst 選定の母集団として全件を並列同定する）
    entries = _to_entries(ds_list)
    n_fit_target = len([e for e in entries if e.real_lite is not None])
    print(f"\n[Phase 2b] 縦方向・操舵 per-dataset フィット (全 {n_fit_target} データセット並列) ...")
    per_ds_long, per_ds_steer = fit_per_dataset(entries, n_jobs=args.n_jobs)
    print(f"  縦方向: {len(per_ds_long)}/{n_fit_target} 件、操舵: {len(per_ds_steer)}/{n_fit_target} 件 同定成功")

    # Phase 3: カーブ多データセット選定
    print("\n[Phase 3] カーブ データセット 選定 ...")
    record_by_uuid = {r["uuid"]: r for r in records}

    def _resolve_uuids(prefix_list: list[str]) -> list[dict]:
        result = []
        seen = set()
        for u in prefix_list:
            matched = [v for k, v in record_by_uuid.items() if k.startswith(u)]
            if not matched:
                print(f"  ⚠ UUID '{u}' が records に見つかりません（スキップ）")
            for m in matched:
                if m["uuid"] not in seen:
                    seen.add(m["uuid"])
                    result.append(m)
        return result

    # まず pinned UUID を先頭に確保
    pinned_prefixes = [u.strip() for u in args.pinned_uuids.split(",") if u.strip()] if args.pinned_uuids else []
    pinned_records = _resolve_uuids(pinned_prefixes)
    pinned_uuids_set = {r["uuid"] for r in pinned_records}

    if args.viewer_uuids:
        requested = [u.strip() for u in args.viewer_uuids.split(",") if u.strip()]
        extra = [r for r in _resolve_uuids(requested) if r["uuid"] not in pinned_uuids_set]
        candidate_curve = pinned_records + extra
        print(f"  --viewer-uuids 指定順モード: pinned={len(pinned_records)} + extra={len(extra)}")
    else:
        records_sorted = sorted(records, key=lambda r: r["curve_count"], reverse=True)
        auto = [r for r in records_sorted if r["uuid"] not in pinned_uuids_set]
        candidate_curve = pinned_records + auto

    top_curve = candidate_curve[: args.n_curve_ds]
    for r in top_curve:
        pinned_mark = " [pinned]" if r["uuid"] in pinned_uuids_set else ""
        print(f"  {r['uuid'][:12]}  curve_count={r['curve_count']}  kappa_max={r['kappa_max_abs']:.4f}{pinned_mark}")

    # Phase 3b / 3c: DatasetCtx 構築 & rollout メトリクス
    top_items = [(r["uuid"], Path(r["lite_dir"])) for r in top_curve]
    deviation_html = ""
    if args.metrics_cache and args.metrics_cache.exists():
        # キャッシュあり → viewer データセット のみ load、メトリクスは CSV から読む
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} データセット) ...")
        ctxs = load_datasets(top_items, n_jobs=min(args.n_jobs, len(top_items)))
        print(f"\n[Phase 3c] rollout メトリクスキャッシュ読み込み ...")
        df_rollout = pd.read_csv(args.metrics_cache)
        n_dataset_cache = df_rollout["uuid"].nunique()
        n_h_cache = df_rollout["h"].nunique()
        print(f"  {len(df_rollout)} 行（{n_dataset_cache} データセット × {n_h_cache} horizons）")
        # score 再現検証（キャッシュロード時も実施）
        per_ds_arg = []
        bl_arg: dict = {}
        for uuid_key, grp in df_rollout.groupby("uuid"):
            gd = grp.set_index("h")[
                ["p14_yaw", "p14_long", "p14_lat", "bl_yaw", "bl_long", "bl_lat"]
            ].to_dict("index")
            per_ds_arg.append((
                uuid_key,
                {int(h): {"yaw": v["p14_yaw"], "long": v["p14_long"], "lat": v["p14_lat"]}
                 for h, v in gd.items()},
            ))
            bl_arg[uuid_key] = {
                int(h): {"yaw": v["bl_yaw"], "long": v["bl_long"], "lat": v["bl_lat"]}
                for h, v in gd.items()
            }
        agg = _agg_normalized(per_ds_arg, bl_arg)
        expected = float(yaml_data.get("score") or 0.0)
        candidates = [
            ("robust_score", _robust_score(agg)),
            ("steer_score",  _steer_score(agg)),
            ("acc_score",    _acc_score(agg)),
        ]
        if expected:
            best_name, recomputed = min(candidates, key=lambda kv: abs(kv[1] - expected))
        else:
            best_name, recomputed = next(kv for kv in candidates if kv[0] == "steer_score")
        diff_str = f"{abs(recomputed - expected) / expected * 100:.2f}%" if expected else "N/A"
        print(f"  再現スコア: {recomputed:.4f} ({best_name})  期待値: {expected:.4f}  差: {diff_str}")
        # baseline (k_us=0) の steer_score を計算
        baseline_steer_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline steer_score: {baseline_steer_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    elif args.metrics_cache:
        # キャッシュなし → 全データセット load（ついでに viewer データセット も取り出す）
        all_items = [(r["uuid"], Path(r["lite_dir"])) for r in records]
        print(f"\n[Phase 3b+3c] 全データセット DatasetCtx 構築 ({len(all_items)} データセット) + rollout メトリクス計算 ...")
        all_ctxs = load_datasets(all_items, n_jobs=args.n_jobs)
        # viewer データセットを all_ctxs から抽出（重複 load 回避）
        ctxs_by_id = {c.dataset_id: c for c in all_ctxs}
        ctxs = [ctxs_by_id[r["uuid"]] for r in top_curve if r["uuid"] in ctxs_by_id]
        # phase14 override: YAML の全 params から _* メタキーを除外（hand-pick より安全）
        override = {k: v for k, v in params.items() if not k.startswith("_")}
        rows = []
        for i, ctx in enumerate(all_ctxs, 1):
            try:
                p14 = _tune_eval(ctx, override, _BASELINE_MODEL)
            except Exception as e:
                print(f"  [WARN] {ctx.dataset_id[:12]}: eval 失敗 ({e})")
                continue
            bl = ctx.base_metric
            for h in _HORIZONS:
                rows.append({
                    "uuid": ctx.dataset_id, "h": h,
                    "p14_yaw": p14[h]["yaw"], "p14_long": p14[h]["long"],
                    "p14_lat": p14[h]["lat"], "p14_vx": p14[h]["vx"],
                    "bl_yaw": bl[h]["yaw"], "bl_long": bl[h]["long"],
                    "bl_lat": bl[h]["lat"], "bl_vx": bl[h]["vx"],
                })
            if i % 100 == 0:
                print(f"  {i}/{len(all_ctxs)} 完了", flush=True)
        df_rollout = pd.DataFrame(rows)
        args.metrics_cache.parent.mkdir(parents=True, exist_ok=True)
        df_rollout.to_csv(args.metrics_cache, index=False)
        print(f"  キャッシュ保存: {args.metrics_cache}")
        # score 再現検証
        per_ds_arg = []
        bl_arg: dict = {}
        for uuid_key, grp in df_rollout.groupby("uuid"):
            grp_dict = grp.set_index("h")[
                ["p14_yaw", "p14_long", "p14_lat", "bl_yaw", "bl_long", "bl_lat"]
            ].to_dict("index")
            per_ds_arg.append((
                uuid_key,
                {int(h): {"yaw": v["p14_yaw"], "long": v["p14_long"], "lat": v["p14_lat"]}
                 for h, v in grp_dict.items()},
            ))
            bl_arg[uuid_key] = {
                int(h): {"yaw": v["bl_yaw"], "long": v["bl_long"], "lat": v["bl_lat"]}
                for h, v in grp_dict.items()
            }
        agg = _agg_normalized(per_ds_arg, bl_arg)
        expected = float(yaml_data.get("score") or 0.0)
        # YAML の score は tuning --phase に応じて steer/acc/robust のいずれかなので最接近を選択
        candidates = [
            ("robust_score", _robust_score(agg)),
            ("steer_score",  _steer_score(agg)),
            ("acc_score",    _acc_score(agg)),
        ]
        if expected:
            best_name, recomputed = min(candidates, key=lambda kv: abs(kv[1] - expected))
        else:
            best_name, recomputed = next(kv for kv in candidates if kv[0] == "steer_score")
        diff_str = f"{abs(recomputed - expected) / expected * 100:.2f}%" if expected else "N/A"
        print(f"  再現スコア: {recomputed:.4f} ({best_name})  期待値: {expected:.4f}  差: {diff_str}")
        # baseline (k_us=0) の steer_score を計算
        baseline_steer_score = _steer_score(_agg_normalized(list(bl_arg.items()), bl_arg))
        print(f"  baseline steer_score: {baseline_steer_score:.4f}")
        deviation_html = _build_sec_deviation(df_rollout, len(records), recomputed, expected, score_name=best_name, label=phase_label)
    else:
        baseline_steer_score = None
        # --metrics-cache 未指定 → 通常の viewer データセット のみ load
        print(f"\n[Phase 3b] DatasetCtx 構築 ({len(top_items)} データセット) ...")
        ctxs = load_datasets(top_items, n_jobs=min(args.n_jobs, len(top_items)))

    curve_count_map = {r["uuid"]: r["curve_count"] for r in top_curve}

    # tuned 設定と baseline 設定（configs = {ラベル: override_params}）
    tuned_keys = [
        "k_us",
        "k_us_bands", "k_us_thresholds",
        "steer_dead_band", "steer_bias",
        "steer_time_constant", "steer_time_delay",
        "acc_time_constant", "acc_time_delay",
        "debug_steer_scaling_factor", "steer_rate_lim",
    ]
    configs: dict[str, dict] = {
        f"{phase_label}（アンダーステア係数ランプ＋steer_dead_band）": {
            k: params[k] for k in tuned_keys if k in params
        },
        "baseline（アンダーステア係数=0 / steer_dead_band=0）": {
            "k_us": 0.0, "steer_dead_band": 0.0,
        },
    }

    # 地図ロード（デフォルトパス自動解決）
    map_osm_path = resolve_map_osm(None)
    map_ways = load_map_ways(map_osm_path) if map_osm_path else None
    if map_ways:
        print(f"  地図ロード完了: {map_osm_path} ({len(map_ways)} ways)")
    else:
        print("  地図なし（ビューアは軌跡のみ表示）")

    viewer_sections: list[str] = []
    for ctx in ctxs:
        # 最初のカーブ開始時刻を検出してプリシーク位置を決定
        initial_t = _find_first_curve_t(ctx, pre_roll_s=5.0)
        vh = _build_viewer_html(ctx, configs, ctx.base, map_ways=map_ways, initial_t=initial_t)
        if vh is None:
            print(f"  {ctx.dataset_id[:12]}: ビューア生成スキップ（データ不足）")
            continue
        srcdoc = _html_stdlib.escape(vh, quote=True)
        cc = curve_count_map.get(ctx.dataset_id, "?")
        viewer_sections.append(f"""
<h3>Dataset: <code>{ctx.dataset_id}</code>  &nbsp;（curve_count = {cc}）</h3>
<p style="font-size:11px;color:#888">
  ドロップダウンで config を切り替え、つまみでパラメータを手動調整できます。
  「最適化」ボタンで最小二乗フィットも実行できます。
</p>
<iframe srcdoc="{srcdoc}"
  width="100%" height="1300"
  style="border:1px solid #ccc;border-radius:4px"
  loading="lazy"></iframe>
""")

    # Phase 4: HTML 組み立て
    print("\n[Phase 4] plotly 図生成 & HTML 組み立て ...")
    kus_fig   = build_fig_kus_single(
        bins, {"チューニング済み速度帯": params},
        thresholds=params.get("k_us_thresholds"),
        title="実機ログからのアンダーステア係数独立同定（速度ビン別 最小二乗法回帰）",
    )
    # 縦方向 / 操舵: 共有ライブラリの実行時フィット (per-dataset フィット結果 + 横断フィット)
    # から best/worst 時系列図を生成する（旧: 事前生成 CSV 依存の build_long_figure /
    # build_steer_id_figure）。凡例にチューン値 τ/T を残すためモデル名に値を埋め込む。
    tuned_clean = {k: v for k, v in params.items() if not k.startswith("_")}
    merged_tuned = merged_model_params(tuned_clean)

    def _model_name(tau_key: str, delay_key: str) -> str:
        tau = merged_tuned.get(tau_key)
        delay = merged_tuned.get(delay_key)
        if tau is None or delay is None:
            return phase_label
        return f"{phase_label} τ={float(tau):.3f}s T={float(delay):.3f}s"

    models_long = {_model_name("acc_time_constant", "acc_time_delay"): SimpleNamespace(params=tuned_clean)}
    models_steer = {_model_name("steer_time_constant", "steer_time_delay"): SimpleNamespace(params=tuned_clean)}

    cross_fit_long = fit_long_cross_dataset_bounded(entries, per_ds_long)
    if np.isfinite(cross_fit_long.get("tau", float("nan"))):
        print(
            f"  縦方向 横断同定: τ={cross_fit_long['tau']:.3f}s"
            f" 遅延={cross_fit_long['delay']:.3f}s"
            f" RMSE={cross_fit_long['rmse_mps2']:.3f} m/s²"
            f" ({cross_fit_long.get('n_datasets', 0)} データセット)"
        )
    else:
        print("  縦方向 横断同定: 失敗（有効データセットなし）")
    rows_long = compute_cross_long_rows(entries, per_ds_long, cross_fit_long, models_long)
    long_fig = build_fig_cross_long(rows_long, cross_fit_long)
    long_tau_fig = build_fig_cross_long_tau_pointwise(rows_long, cross_fit_long)
    long_tau_hist_fig = build_fig_long_tau_pointwise_hist(cross_fit_long)

    rows_steer = compute_cross_steer_rows(entries, per_ds_steer, models_steer)
    # 旧実装の表示要素を維持: 各 subplot タイトルに per-dataset 同定値 τ/T を付記
    for r in rows_steer:
        for ds_id, fit in per_ds_steer.items():
            if ds_id[:8] in r["label"]:
                r["label"] += f"  τ={fit['tau']:.3f}s T={fit['delay']:.3f}s"
                break
    steer_fig = build_fig_cross_steer(rows_steer)

    # 1-2 縦方向理想追従評価（全 records の先頭 _LONG_PERF_N_DATASET データセットを使用。
    # 計算・描画は lib 共有関数。タイトル文言のみレポート従来表記を labels/title で維持）
    long_perf_records = records[:_LONG_PERF_N_DATASET]
    print(f"  [1-2] 縦方向理想追従評価図生成 ({len(long_perf_records)} データセット) ...")
    long_perf_entries = _to_entries([(r["uuid"], Path(r["lite_dir"])) for r in long_perf_records])
    long_perf_data = compute_long_perf_data(long_perf_entries)
    _n_lp = long_perf_data.get("n_dataset", 0)
    long_perf_figs = (
        build_fig_long_perf_box(
            long_perf_data,
            title=f"縦方向 モデル構造限界評価（a_act 直接入力 vs 実車変位、上位 {_n_lp} データセット）",
        ),
        build_fig_long_perf_growth(
            long_perf_data,
            labels={
                "title": (
                    f"縦方向ドリフト成長カーブ"
                    f"（符号付き・reset-stride ロールアウト、上位 {_n_lp} データセット）"
                ),
                "subplot_titles": ["速度誤差  v_x,sim − 実車 v_x  [m/s]",
                                   "変位誤差  s_sim − 実車 s  [cm]"],
                "y_titles": [
                    "速度誤差 [m/s]<br><sup>正 = シミュレーションが実車より速い</sup>",
                    "変位誤差 [cm]<br><sup>正 = シミュレーションが実車より進んでいる</sup>",
                ],
            },
        ),
        build_fig_long_perf_map(
            long_perf_data, map_ways,
            title="縦方向変位誤差の地図分布（1.0s 窓終端・rollout 開始点、正 = シミュレーションが実車より進む）",
        ),
    )

    # 1-5 横方向理想追従評価には curve 上位 _PERF_N_DATASET データセットを使用（viewer 用 top_curve とは独立して選択）
    perf_records = candidate_curve[:_PERF_N_DATASET]
    print(f"  [1-5] 横方向理想追従評価図生成 ({len(perf_records)} データセット) ...")
    perf_entries = _to_entries([(r["uuid"], Path(r["lite_dir"])) for r in perf_records])
    perf_data = compute_perfect_tracking_data(perf_entries, params)
    perf_fig_box = build_fig_perfect_tracking_box(perf_data)
    perf_fig_traj = build_fig_perfect_tracking_traj(
        perf_data,
        labels={
            "title": "実車 vs 自転車モデル軌跡（実測 v_x + 実舵角入力、初期状態を実車ログに合わせたリセットなし積分）",
            "gt_name": "実車 軌跡",
            "model_name": "自転車モデル（理想追従）",
            "x_title": "Δx [m]",
            "y_title": "Δy [m]",
        },
        height=480,
    )
    perf_html = _build_sec14(perf_fig_box, perf_fig_traj, params, len(perf_records))

    closed_loop_html = _build_sec_closed_loop_comparison(args.collection_dir, args.closed_loop_uuids)

    html = build_html(
        params, long_fig, steer_fig, kus_fig, viewer_sections, len(records),
        baseline_score=baseline_steer_score,
        deviation_html=deviation_html,
        label=phase_label,
        params_filename=args.params.name,
        perf_html=perf_html,
        long_perf_figs=long_perf_figs,
        closed_loop_html=closed_loop_html,
        long_tau_fig=long_tau_fig,
        long_tau_hist_fig=long_tau_hist_fig,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding="utf-8")
    size_kb = args.out.stat().st_size // 1024
    print(f"\n✓ 完了: {args.out}  ({size_kb} KB)")


if __name__ == "__main__":
    main()
