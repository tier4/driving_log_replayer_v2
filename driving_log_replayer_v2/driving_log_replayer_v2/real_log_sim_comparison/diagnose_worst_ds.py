#!/usr/bin/env python3
"""worst-case DS の時系列ロールアウト分析スクリプト.

各カテゴリ（yaw+lat / high-lat / long-only）の代表 DS について
N=40 ロールアウトの詳細時系列をプロットし HTML で出力する。
"""
from __future__ import annotations

import datetime
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import yaml
from plotly.subplots import make_subplots

_INSTALL = Path(
    "/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/"
    "driving_log_replayer_v2/local/lib/python3.10/dist-packages"
)
if _INSTALL.exists() and str(_INSTALL) not in sys.path:
    sys.path.insert(0, str(_INSTALL))

from driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune import (
    _BASELINE_MODEL, _discover, _filter_by_date, load_datasets, unify_step_bands,
)
import driving_log_replayer_v2.real_log_sim_comparison.step_ol1_analyze_nstep as s5
import pandas as pd

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
COLLECTION_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june")
PARAMS_YAML    = Path("/home/kotaroyoshimoto/data/openloop_j6_15/tuned_params_phase48.yaml")
OUT_HTML       = Path("/home/kotaroyoshimoto/data/openloop_j6_15/worst_ds_timeseries.html")
DS_AFTER       = datetime.date(2026, 6, 16)

# 分析対象 DS（UUID prefix 8 文字で前方一致）
TARGET_DS = {
    "df57e2ac": "yaw+lat worst (yaw=1.08°, lat=5.21cm @N=40)",
    "a9460b07": "lat+long worst (lat=5.40cm, long=25.30cm @N=40)",
    "882e7b0f": "lat worst #2 (lat=5.22cm @N=40)",
    "701a2ae8": "lat-only (yaw=0.24°, lat=3.54cm @N=40)",
    "7691fae7": "long-only (long=20.55cm @N=40)",
    "a3ba275d": "long+vx worst (long=21.1cm, vx=0.42m/s @N=40)",
}

HORIZONS = (10, 20, 30, 40)
STRIDE   = 2
WHEELBASE = 4.76012

# ---------------------------------------------------------------------------
# パラメータ読み込み
# ---------------------------------------------------------------------------
with open(PARAMS_YAML) as f:
    yd = yaml.safe_load(f)
PARAMS = yd.get("params", yd)


def build_override(base: dict) -> dict:
    ov = unify_step_bands(dict(base))
    ov.update(PARAMS)
    ov["k_us_bands"]      = []
    ov["k_us_thresholds"] = []
    return ov


# ---------------------------------------------------------------------------
# DS 読み込み
# ---------------------------------------------------------------------------
print("[1] DS 検索中 ...")
all_ds = _discover(COLLECTION_DIR)
all_ds = _filter_by_date(all_ds, None, DS_AFTER)

target_items = []
for uuid, lite_dir in all_ds:
    for prefix in TARGET_DS:
        if uuid.startswith(prefix):
            target_items.append((uuid, lite_dir))
            break

print(f"  対象 DS: {len(target_items)}")
for uuid, _ in target_items:
    for pfx, label in TARGET_DS.items():
        if uuid.startswith(pfx):
            print(f"    {uuid[:8]}  [{label}]")

print("[2] DatasetCtx 構築 ...")
ctxs = load_datasets(target_items)
print(f"  ロード成功: {len(ctxs)} DS")

# ---------------------------------------------------------------------------
# rollout 実行
# ---------------------------------------------------------------------------
print("[3] rollout 実行中 ...")
results: dict[str, tuple[pd.DataFrame, dict]] = {}  # ds_id -> (df_rollout, gt)

for ctx in ctxs:
    print(f"  {ctx.dataset_id[:8]}...", flush=True)
    try:
        override = build_override(ctx.base)
        gt = s5._prepare_gt(ctx.data, ctx.t0_ns, override)
        df = s5.run_rollout(ctx.data, ctx.t0_ns, override, _BASELINE_MODEL, HORIZONS, STRIDE, gt=gt)
        results[ctx.dataset_id] = (df, gt)
    except Exception as e:
        import traceback
        print(f"    ERROR: {e}")
        traceback.print_exc()

# ---------------------------------------------------------------------------
# HTML プロット生成
# ---------------------------------------------------------------------------
_MATHJAX = (
    "<script>window.MathJax={tex:{inlineMath:[['\\\\(','\\\\)']]},"
    "svg:{fontCache:'global'}};</script>"
    "<script async src='https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js'></script>"
)
_PLOTLY = "<script src='https://cdn.plot.ly/plotly-2.35.2.min.js'></script>"
_STYLE = """
body{font-family:sans-serif;max-width:1400px;margin:0 auto;padding:20px;color:#333}
h1{color:#222}h2{color:#444;border-bottom:2px solid #bbb;padding-bottom:4px;margin-top:32px}
h3{color:#555;margin-top:16px}
.note{background:#fff8e1;border-left:4px solid #ffc107;padding:8px 12px;margin:8px 0;font-size:13px}
.tbl{border-collapse:collapse;font-size:13px;margin:8px 0}
.tbl td,.tbl th{border:1px solid #ddd;padding:5px 10px}.tbl th{background:#f5f5f5}
.red{color:#dc3545;font-weight:bold}.grn{color:#28a745;font-weight:bold}
"""


def plot_ds_section(ds_id: str, label: str, df: pd.DataFrame, gt: dict) -> str:
    """1 DS のプロット HTML を生成."""

    # ---- N=40 の rollout ----
    d40 = df[df["horizon"] == 40].sort_values("tr")
    d10 = df[df["horizon"] == 10].sort_values("tr")
    d20 = df[df["horizon"] == 20].sort_values("tr")

    # ---- (A) 誤差時系列（各 N） ----
    fig_err = make_subplots(
        rows=2, cols=2,
        subplot_titles=[
            "yaw誤差 [deg]  (|実機-sim|)",
            "横(lat)誤差 [m]  (実機-sim)",
            "縦(long)誤差 [m]  (実機-sim)",
            "速度誤差 [m/s]  (実機-sim)",
        ],
        shared_xaxes=True,
    )
    colors = {10: "royalblue", 20: "orange", 40: "red"}
    for h, dh, c in [(10, d10, colors[10]), (20, d20, colors[20]), (40, d40, colors[40])]:
        if dh.empty:
            continue
        fig_err.add_trace(go.Scatter(x=dh["tr"], y=dh["yaw_err_deg"].abs(),
            name=f"N={h}", line=dict(color=c), legendgroup=f"N={h}"), row=1, col=1)
        fig_err.add_trace(go.Scatter(x=dh["tr"], y=dh["err_ds_lat"],
            name=f"N={h}", line=dict(color=c), legendgroup=f"N={h}", showlegend=False), row=1, col=2)
        fig_err.add_trace(go.Scatter(x=dh["tr"], y=dh["err_ds_long"],
            name=f"N={h}", line=dict(color=c), legendgroup=f"N={h}", showlegend=False), row=2, col=1)
        fig_err.add_trace(go.Scatter(x=dh["tr"], y=dh["err_vx"],
            name=f"N={h}", line=dict(color=c), legendgroup=f"N={h}", showlegend=False), row=2, col=2)

    # 0 ライン
    for row, col in [(1,1),(1,2),(2,1),(2,2)]:
        fig_err.add_hline(y=0, line_dash="dot", line_color="gray", row=row, col=col)

    fig_err.update_layout(height=500, title_text=f"{ds_id[:8]} — N-step 終端誤差時系列",
                          legend=dict(orientation="h", y=1.02))

    # ---- (B) GT vs Sim 時系列（絶対値） ----
    t_gt    = gt["t_cmd"]
    gt_wz   = gt["gt_wz"]
    gt_steer= gt["gt_steer"]
    gt_vx   = gt["gt_vx"]
    gt_ax   = gt["gt_ax"]

    # sim の絶対値は gt - err (N=1 の近似)
    d1 = df[df["horizon"] == 10].sort_values("tr")
    if not d1.empty:
        t1    = d1["tr"].values
        sim_wz_est   = d1["real_wz"].values - d1["err_wz"].values
        sim_steer_est= d1["sim_steer_kend"].values
        sim_vx_est   = d1["sim_vx"].values
    else:
        t1 = np.array([])
        sim_wz_est = sim_steer_est = sim_vx_est = np.array([])

    fig_ts = make_subplots(
        rows=4, cols=1,
        subplot_titles=["ヨーレート wz [rad/s]", "操舵角 steer [rad]",
                        "車速 vx [m/s]", "加速度 ax [m/s²]"],
        shared_xaxes=True,
    )
    t_off = t_gt[0] if len(t_gt) else 0

    fig_ts.add_trace(go.Scatter(x=t_gt - t_off, y=gt_wz,
        name="GT", line=dict(color="blue")), row=1, col=1)
    if len(t1):
        fig_ts.add_trace(go.Scatter(x=t1, y=sim_wz_est,
            name="sim(N=10端)", line=dict(color="red", dash="dot")), row=1, col=1)

    fig_ts.add_trace(go.Scatter(x=t_gt - t_off, y=gt_steer,
        name="GT", line=dict(color="blue"), showlegend=False), row=2, col=1)
    if len(t1):
        fig_ts.add_trace(go.Scatter(x=t1, y=sim_steer_est,
            name="sim(N=10端)", line=dict(color="red", dash="dot"), showlegend=False), row=2, col=1)

    fig_ts.add_trace(go.Scatter(x=t_gt - t_off, y=gt_vx,
        name="GT", line=dict(color="blue"), showlegend=False), row=3, col=1)
    if len(t1):
        fig_ts.add_trace(go.Scatter(x=t1, y=sim_vx_est,
            name="sim(N=10端)", line=dict(color="red", dash="dot"), showlegend=False), row=3, col=1)

    fig_ts.add_trace(go.Scatter(x=t_gt - t_off, y=gt_ax,
        name="GT", line=dict(color="blue"), showlegend=False), row=4, col=1)

    fig_ts.update_layout(height=700, title_text=f"{ds_id[:8]} — GT vs sim 絶対値時系列")

    # ---- まとめテーブル ----
    tbl_rows = ""
    for h in HORIZONS:
        dh = df[df["horizon"] == h]
        if dh.empty:
            continue
        yaw_max  = dh["yaw_err_deg"].abs().max()
        lat_max  = dh["err_ds_lat"].abs().max() * 100
        long_max = dh["err_ds_long"].abs().max() * 100
        vx_max   = dh["err_vx"].abs().max()
        yaw_mean  = dh["yaw_err_deg"].abs().mean()
        lat_mean  = dh["err_ds_lat"].abs().mean() * 100
        long_mean = dh["err_ds_long"].abs().mean() * 100
        vx_mean   = dh["err_vx"].abs().mean()
        tbl_rows += (
            f"<tr><td>N={h}</td>"
            f"<td>{yaw_mean:.3f} / <b>{yaw_max:.3f}</b></td>"
            f"<td>{lat_mean:.2f} / <b>{lat_max:.2f}</b></td>"
            f"<td>{long_mean:.2f} / <b>{long_max:.2f}</b></td>"
            f"<td>{vx_mean:.3f} / <b>{vx_max:.3f}</b></td></tr>"
        )

    # ---- ヨーレート誤差 vs 車速・ステア（構造診断） ----
    # err_wz と real_vx の散布図（k_us 感度確認）
    d_all = df[df["horizon"] == 40]
    fig_scatter = make_subplots(rows=1, cols=2,
        subplot_titles=["err_wz vs real_vx² × |steer|  (k_us 感度)", "err_vx vs real_vx (速度誤差形状)"])
    if not d_all.empty:
        v2_steer = d_all["real_vx"].values**2 * np.abs(d_all["real_steer_k0"].values)
        fig_scatter.add_trace(go.Scatter(x=v2_steer, y=d_all["err_wz"].values,
            mode="markers", marker=dict(size=4, color="royalblue", opacity=0.5),
            name="N=40"), row=1, col=1)
        fig_scatter.add_trace(go.Scatter(x=d_all["real_vx_kend"].values, y=d_all["err_vx"].values,
            mode="markers", marker=dict(size=4, color="orange", opacity=0.5),
            name="N=40"), row=1, col=2)
        for r, c in [(1,1),(1,2)]:
            fig_scatter.add_hline(y=0, line_dash="dot", line_color="gray", row=r, col=c)
    fig_scatter.update_layout(height=350, title_text=f"{ds_id[:8]} — 誤差相関散布図", showlegend=False)

    return f"""
<h2 id="{ds_id[:8]}">{ds_id[:8]}… — {label}</h2>
<table class="tbl">
<tr><th>N</th><th>yaw mean/max [deg]</th><th>lat mean/max [cm]</th>
    <th>long mean/max [cm]</th><th>vx mean/max [m/s]</th></tr>
{tbl_rows}
</table>
{fig_err.to_html(full_html=False, include_plotlyjs=False)}
{fig_ts.to_html(full_html=False, include_plotlyjs=False)}
{fig_scatter.to_html(full_html=False, include_plotlyjs=False)}
"""


print("[4] HTML 生成中 ...")

# worst DS サマリ (cache から)
df_cache = pd.read_csv("/home/kotaroyoshimoto/data/openloop_j6_15/metrics_cache_phase48.csv")
h40_cache = df_cache[df_cache["h"] == 40].sort_values("p14_yaw", ascending=False)

tbl_summary = ""
for _, row in h40_cache.head(15).iterrows():
    cat = ""
    if row["p14_yaw"] > 0.7:
        cat = '<span class="red">yaw+lat</span>'
    elif row["p14_yaw"] < 0.3 and row["p14_lat"] > 2.0:
        cat = "lat-only"
    elif row["p14_yaw"] < 0.3 and row["p14_long"] > 12.0:
        cat = "long-only"
    else:
        cat = "normal"
    tbl_summary += (
        f"<tr><td>{row['uuid'][:8]}</td><td>{cat}</td>"
        f"<td>{row['p14_yaw']:.3f}</td><td>{row['p14_lat']:.2f}</td>"
        f"<td>{row['p14_long']:.2f}</td><td>{row['p14_vx']:.3f}</td>"
        f"<td>{row['bl_yaw']:.3f}</td><td>{row['bl_lat']:.2f}</td>"
        f"<td>{row['p14_yaw']/max(row['bl_yaw'],1e-4):.2f}</td></tr>"
    )

nav_links = " | ".join(
    f'<a href="#{ds_id[:8]}">{ds_id[:8]}</a>'
    for ds_id, _ in target_items
)

sections = []
for ctx in ctxs:
    ds_id = ctx.dataset_id
    for pfx, label in TARGET_DS.items():
        if ds_id.startswith(pfx):
            r = results.get(ds_id)
            if r is None:
                sections.append(f"<h2>{ds_id[:8]}</h2><p>rollout 失敗</p>")
            else:
                df_r, gt_r = r
                sections.append(plot_ds_section(ds_id, label, df_r, gt_r))
            break

full_html = f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<title>worst DS 時系列分析 — Phase 48 k_us=0</title>
{_PLOTLY}
<style>{_STYLE}</style>
</head>
<body>
<h1>worst-case DS 時系列分析 — Phase 48 (k_us=0)</h1>
<p class="note">
  パラメータ: <code>{PARAMS_YAML.name}</code> &nbsp;|&nbsp;
  DS after: 2026-06-16 &nbsp;|&nbsp; 対象: {len(results)} DS
</p>
<p>{nav_links}</p>

<h2>0. worst DS サマリ (N=40 yaw 上位 15)</h2>
<table class="tbl">
<tr><th>UUID</th><th>cat</th><th>yaw[deg]</th><th>lat[cm]</th><th>long[cm]</th><th>vx[m/s]</th>
    <th>bl_yaw</th><th>bl_lat</th><th>ratio_yaw</th></tr>
{tbl_summary}
</table>

<div class="note">
<b>カテゴリ定義 (N=40):</b><br>
• <span class="red">yaw+lat</span>: yaw&gt;0.7° — カーブ中のモデルズレ<br>
• <b>lat-only</b>: yaw&lt;0.3°, lat&gt;2cm — 直進中の横ドリフト<br>
• <b>long-only</b>: yaw&lt;0.3°, long&gt;12cm — 加速誤差<br>
• ratio_yaw≈1.0 = Phase 48 が baseline（k_us=0 無補正）からほぼ改善なし<br>
</div>

{''.join(sections)}
</body>
</html>"""

OUT_HTML.write_text(full_html, encoding="utf-8")
print(f"\n✓ 完了: {OUT_HTML}  ({OUT_HTML.stat().st_size // 1024} KB)")
