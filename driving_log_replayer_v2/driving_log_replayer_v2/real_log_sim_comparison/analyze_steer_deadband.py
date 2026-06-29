"""
rosbag から per-DS の steer 制御不感帯を推定し CSV + plotly 図を出力する。

指標:
  - steer_diff_p50   : |steer - steer_des| の中央値 [rad]
  - steer_diff_p95   : |steer - steer_des| の p95 [rad]
  - steer_diff_bias  : steer - steer_des の平均（正バイアス > 0 で steer が des を超える）
  - frac_below_db    : |steer_diff| < dead_band_opt のサンプル割合
  - steer_rate_p50   : |d_steer/dt| の中央値 [rad/s]（操舵の活発さ）
  - still_frac       : steer_rate <= 0.01 rad/s のサンプル割合（静止割合）

出力:
  /home/kotaroyoshimoto/data/openloop_j6_15/steer_deadband_report.csv
  <COLLECTION_DIR>/cross_dataset/steer_diff_overview.fig.json
"""
import sys, numpy as np, pandas as pd
from pathlib import Path

sys.path.insert(0, '/home/kotaroyoshimoto/workspace/x2_e2e_curve/install/'
                'driving_log_replayer_v2/local/lib/python3.10/dist-packages')

from driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune import (
    _discover, load_datasets
)
from driving_log_replayer_v2.real_log_sim_comparison.lib._fig_io import write_fig_json

COLLECTION_DIR = Path("/home/kotaroyoshimoto/data/openloop_j6_15_june")
OUT_CSV        = Path("/home/kotaroyoshimoto/data/openloop_j6_15/steer_deadband_report.csv")
DEAD_BAND_OPT  = 0.002438735319718077

def analyze_ds(ctx) -> dict | None:
    data  = ctx.data
    t0_ns = ctx.t0_ns

    st = data.get("steer")
    cm = data.get("cmd")
    if st is None or cm is None:
        return None

    st = st[st["t_ns"] >= t0_ns].copy()
    cm = cm[cm["t_ns"] >= t0_ns].copy()
    if len(st) < 20 or len(cm) < 20:
        return None

    # steer を cmd 時間軸に補間
    steer_interp = np.interp(cm["t_ns"].values, st["t_ns"].values, st["steer"].values)
    diff_signed  = steer_interp - cm["steer_des"].values
    diff_abs     = np.abs(diff_signed)

    # steer_rate
    dt_ns   = np.diff(st["t_ns"].values)
    d_steer = np.diff(st["steer"].values)
    valid   = dt_ns > 0
    rate    = np.abs(d_steer[valid] / (dt_ns[valid] * 1e-9))
    t_rate  = 0.5 * (st["t_ns"].values[:-1][valid] + st["t_ns"].values[1:][valid])
    rate_interp = np.interp(cm["t_ns"].values, t_rate, rate)

    # vx（速度）
    vel_df = data.get("vel")
    kin_df = data.get("kin")
    vx_vals = None
    if kin_df is not None and "vx" in kin_df.columns:
        kin_sub = kin_df[kin_df["t_ns"] >= t0_ns]
        if len(kin_sub) > 0:
            vx_vals = kin_sub["vx"].values
    if vx_vals is None and vel_df is not None:
        vel_sub = vel_df[vel_df["t_ns"] >= t0_ns] if "t_ns" in vel_df.columns else vel_df
        if len(vel_sub) > 0:
            col = [c for c in vel_sub.columns if "vel" in c.lower() or "vx" in c.lower()]
            if col:
                vx_vals = vel_sub[col[0]].values

    vx_mean = float(np.nanmean(vx_vals)) if vx_vals is not None and len(vx_vals) > 0 else float("nan")
    vx_max  = float(np.nanmax(vx_vals))  if vx_vals is not None and len(vx_vals) > 0 else float("nan")

    return {
        "dataset_id":     ctx.dataset_id,
        "n_samples":      len(diff_abs),
        "vx_mean":        vx_mean,
        "vx_max":         vx_max,
        "steer_diff_p50": float(np.percentile(diff_abs, 50)),
        "steer_diff_p95": float(np.percentile(diff_abs, 95)),
        "steer_diff_bias": float(np.mean(diff_signed)),
        "frac_below_db":  float(np.mean(diff_abs < DEAD_BAND_OPT)),
        "steer_rate_p50": float(np.percentile(rate_interp, 50)),
        "still_frac":     float(np.mean(rate_interp <= 0.01)),
    }


def main():
    print("データ読み込み中 ...")
    ctxs = load_datasets(_discover(COLLECTION_DIR), n_jobs=8)
    print(f"  {len(ctxs)} datasets loaded")

    rows = []
    for ctx in ctxs:
        r = analyze_ds(ctx)
        if r is not None:
            rows.append(r)

    df = pd.DataFrame(rows)
    df = df.sort_values("steer_diff_p50", ascending=False)

    df.to_csv(OUT_CSV, index=False, float_format="%.6f")
    print(f"\nCSV 保存: {OUT_CSV}  ({len(df)} rows)")

    # 全体サマリ
    print("\n=== 全体サマリ ===")
    print(f"  steer_diff_p50  mean={df['steer_diff_p50'].mean():.5f} median={df['steer_diff_p50'].median():.5f} rad")
    print(f"  steer_diff_p95  mean={df['steer_diff_p95'].mean():.5f} median={df['steer_diff_p95'].median():.5f} rad")
    print(f"  steer_diff_bias mean={df['steer_diff_bias'].mean():.5f} rad")
    print(f"  frac_below_db   mean={df['frac_below_db'].mean():.3f}")
    print(f"  still_frac      mean={df['still_frac'].mean():.3f}")

    # 速度層別
    low  = df[df["vx_mean"] < 2.5]
    high = df[df["vx_mean"] >= 2.5]
    print(f"\n=== 低速 DS (vx_mean < 2.5 m/s, n={len(low)}) ===")
    print(f"  steer_diff_p50 中央値: {low['steer_diff_p50'].median():.5f} rad")
    print(f"  frac_below_db  中央値: {low['frac_below_db'].median():.3f}")
    print(f"  still_frac     中央値: {low['still_frac'].median():.3f}")
    print(f"\n=== 高速 DS (vx_mean >= 2.5 m/s, n={len(high)}) ===")
    print(f"  steer_diff_p50 中央値: {high['steer_diff_p50'].median():.5f} rad")
    print(f"  frac_below_db  中央値: {high['frac_below_db'].median():.3f}")
    print(f"  still_frac     中央値: {high['still_frac'].median():.3f}")

    # 上位 10 DS（steer_diff_p50 が大きい）
    print("\n=== steer_diff_p50 が大きい上位 10 DS ===")
    cols = ["dataset_id", "vx_mean", "steer_diff_p50", "steer_diff_p95", "steer_diff_bias", "frac_below_db"]
    print(df[cols].head(10).to_string(index=False))

    # dead_band = 0.00244 との対応
    db = DEAD_BAND_OPT
    frac_ds_db_above = (df["steer_diff_p50"] > db).mean()
    print(f"\n=== dead_band={db:.5f} rad との対応 ===")
    print(f"  DS の {frac_ds_db_above*100:.1f}% で p50 > dead_band（大半のデータで dead_band が中央値付近）")
    print(f"  dead_band は steer_diff 全体分布の中央値 ≈ {df['steer_diff_p50'].median():.5f} rad に対応")

    # plotly 図を生成して cross_dataset/ に出力
    fig = _build_steer_diff_fig(df, dead_band=DEAD_BAND_OPT)
    cross_dir = COLLECTION_DIR / "cross_dataset"
    cross_dir.mkdir(exist_ok=True)
    out_fig = cross_dir / "steer_diff_overview.fig.json"
    write_fig_json(fig, out_fig)
    print(f"\nplotly 図保存: {out_fig}")


def _build_steer_diff_fig(df: pd.DataFrame, dead_band: float):
    """steer_diff 統計の plotly 概観図 (2パネル)。

    左: steer_diff_p50 vs vx_mean の散布図（色=frac_below_db、dead_band 参照線付き）
    右: 速度層別 steer_diff_p50 ヒストグラム（dead_band 垂直参照線付き）
    """
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    low  = df[df["vx_mean"] < 2.5]
    high = df[df["vx_mean"] >= 2.5]
    db_deg = dead_band * 180 / np.pi

    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=[
            "steer_diff_p50 vs 平均速度（色: dead_band 以下の割合）",
            "速度層別 steer_diff_p50 分布",
        ],
        horizontal_spacing=0.12,
    )

    # --- 左パネル: 散布図 ---
    for grp_df, name, color in [
        (low,  "低速 (<2.5 m/s)", "#1f77b4"),
        (high, "高速 (≥2.5 m/s)", "#ff7f0e"),
    ]:
        fig.add_trace(go.Scatter(
            x=grp_df["vx_mean"],
            y=grp_df["steer_diff_p50"],
            mode="markers",
            name=name,
            marker=dict(
                size=5,
                color=grp_df["frac_below_db"],
                colorscale="RdYlGn",
                cmin=0.0, cmax=1.0,
                showscale=(name == "高速 (≥2.5 m/s)"),
                colorbar=dict(
                    title="dead_band<br>以下の割合",
                    x=0.46, len=0.85,
                ),
            ),
            customdata=np.stack([
                grp_df["steer_diff_p95"],
                grp_df["frac_below_db"],
                grp_df["dataset_id"].str[:8],
            ], axis=1),
            hovertemplate=(
                "ds=%{customdata[2]}<br>"
                "vx_mean=%{x:.2f} m/s<br>"
                "p50=%{y:.5f} rad<br>"
                "p95=%{customdata[0]:.5f} rad<br>"
                "frac_below_db=%{customdata[1]:.3f}<extra></extra>"
            ),
        ), row=1, col=1)

    # dead_band 参照線（左パネル）
    fig.add_hline(
        y=dead_band, line_dash="dash", line_color="red", line_width=1.5,
        annotation_text=f"dead_band={dead_band:.5f} rad ({db_deg:.3f}°)",
        annotation_position="top right",
        row=1, col=1,
    )

    # --- 右パネル: ヒストグラム ---
    bins = dict(start=0, end=0.05, size=0.001)
    for grp_df, name, color in [
        (low,  "低速 (<2.5 m/s)", "#1f77b4"),
        (high, "高速 (≥2.5 m/s)", "#ff7f0e"),
    ]:
        fig.add_trace(go.Histogram(
            x=grp_df["steer_diff_p50"],
            xbins=bins,
            name=name,
            opacity=0.6,
            marker_color=color,
            showlegend=False,
        ), row=1, col=2)

    # dead_band 参照線（右パネル）
    fig.add_vline(
        x=dead_band, line_dash="dash", line_color="red", line_width=1.5,
        annotation_text=f"{dead_band:.5f}",
        annotation_position="top right",
        row=1, col=2,
    )

    fig.update_layout(
        title_text=(
            f"steer 制御不感帯分析: |steer − steer_des| の per-DS 統計<br>"
            f"<sup>dead_band_opt={dead_band:.5f} rad ({db_deg:.3f}°)  "
            f"全体中央値 p50={df['steer_diff_p50'].median():.5f} rad  n={len(df)} DS</sup>"
        ),
        height=480,
        barmode="overlay",
        legend=dict(x=0.02, y=0.98),
    )
    fig.update_xaxes(title_text="平均速度 vx_mean [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="steer_diff_p50 [rad]",   row=1, col=1)
    fig.update_xaxes(title_text="steer_diff_p50 [rad]",   row=1, col=2)
    fig.update_yaxes(title_text="DS 数",                  row=1, col=2)

    return fig


if __name__ == "__main__":
    main()
