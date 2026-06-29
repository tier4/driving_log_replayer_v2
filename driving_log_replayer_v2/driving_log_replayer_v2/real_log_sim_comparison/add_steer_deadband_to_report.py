#!/usr/bin/env python3
"""
steer_deadband_report.csv から図を生成し、既存の dataset_analysis_report/report.html に追記する。

ROS 環境不要: pandas + matplotlib のみ使用。
analyze_steer_deadband.py を先に実行して CSV を生成しておくこと。
"""

import base64
import io
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

STEER_DEADBAND_CSV = Path("/home/kotaroyoshimoto/data/openloop_j6_15/steer_deadband_report.csv")
REPORT_HTML = Path("/home/kotaroyoshimoto/data/openloop_j6_15/dataset_analysis_report/report.html")
DEAD_BAND_OPT = 0.002438735319718077


def fig_to_b64(fig: plt.Figure, dpi: int = 120) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode()


def plot_steer_deadband(df_db: pd.DataFrame) -> plt.Figure:
    db = DEAD_BAND_OPT
    db_deg = np.degrees(db)
    low  = df_db[df_db["vx_mean"] < 2.5]
    high = df_db[df_db["vx_mean"] >= 2.5]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(
        f"steer 制御不感帯分析: |steer − steer_des| per-DS 統計\n"
        f"dead_band_opt={db:.5f} rad ({db_deg:.3f}°)  N={len(df_db)} DS",
        fontsize=13,
    )

    ax = axes[0]
    sc = ax.scatter(
        df_db["vx_mean"], df_db["steer_diff_p50"],
        c=df_db["frac_below_db"], cmap="RdYlGn", vmin=0, vmax=1,
        s=12, alpha=0.6,
    )
    plt.colorbar(sc, ax=ax, label="frac_below_db (dead_band 以下の割合)")
    ax.axhline(db, ls="--", color="red", lw=1.5,
               label=f"dead_band={db:.5f} rad")
    ax.axhline(df_db["steer_diff_p50"].median(), ls=":", color="black", lw=1.2,
               label=f"全体中央値={df_db['steer_diff_p50'].median():.5f} rad")
    ax.set_xlabel("平均速度 vx_mean [m/s]")
    ax.set_ylabel("steer_diff_p50 [rad]")
    ax.set_title("steer 追従誤差の中央値 vs 平均速度\n（色: dead_band 以下の割合）")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax2 = axes[1]
    bins_hist = np.arange(0, 0.051, 0.001)
    ax2.hist(low["steer_diff_p50"],  bins=bins_hist, alpha=0.6, color="#1f77b4",
             label=f"低速 (<2.5 m/s, n={len(low)})")
    ax2.hist(high["steer_diff_p50"], bins=bins_hist, alpha=0.6, color="#ff7f0e",
             label=f"高速 (≥2.5 m/s, n={len(high)})")
    ax2.axvline(db, ls="--", color="red", lw=1.5, label=f"dead_band={db:.5f}")
    ax2.axvline(df_db["steer_diff_p50"].median(), ls=":", color="black", lw=1.2,
                label=f"全体中央値={df_db['steer_diff_p50'].median():.5f}")
    ax2.set_xlabel("steer_diff_p50 [rad]")
    ax2.set_ylabel("DS 数")
    ax2.set_title("速度層別 steer_diff_p50 分布")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    return fig


def main():
    if not STEER_DEADBAND_CSV.exists():
        print(f"[ERROR] CSV が存在しません: {STEER_DEADBAND_CSV}")
        print("先に analyze_steer_deadband.py を実行してください。")
        return 1

    if not REPORT_HTML.exists():
        print(f"[ERROR] レポートが存在しません: {REPORT_HTML}")
        return 1

    df_db = pd.read_csv(STEER_DEADBAND_CSV)
    print(f"CSV 読み込み: {len(df_db)} rows")

    db = DEAD_BAND_OPT
    low_db  = df_db[df_db["vx_mean"] < 2.5]
    high_db = df_db[df_db["vx_mean"] >= 2.5]

    print("図生成中...")
    fig = plot_steer_deadband(df_db)
    img_b64 = fig_to_b64(fig)

    body_text = (
        f"最適化パラメータ <em>steer_dead_band={db:.5f} rad ({np.degrees(db):.3f}°)</em> の物理的根拠を実機ログから確認。"
        f"per-DS の |steer − steer_des| 中央値 (steer_diff_p50) の全体中央値は "
        f"{df_db['steer_diff_p50'].median():.5f} rad で dead_band_opt にほぼ一致する。"
        f"高速 DS (≥2.5 m/s, n={len(high_db)}) では steer_diff_p50 &lt; dead_band が多く"
        f"（中央値 {high_db['steer_diff_p50'].median():.5f} rad）、"
        f"低速 DS (&lt;2.5 m/s, n={len(low_db)}) では逆転する"
        f"（中央値 {low_db['steer_diff_p50'].median():.5f} rad）。"
        " EPS 追従コントローラが steer_diff &lt; ~0.002 rad で追従を打ち切る特性を"
        " dead_band が近似しており、系統的な未モデル化現象の補正として機能している。"
        f" steer_diff の正バイアス（全体平均 +{df_db['steer_diff_bias'].mean():.5f} rad）は"
        " EPS が指令をわずかに超えた位置で止まる傾向を示す。"
    )

    section_html = (
        f'\n<!-- steer_deadband section (added by add_steer_deadband_to_report.py) -->\n'
        f'<h2>6. steer 制御不感帯分析 (|steer − steer_des| 分布)</h2>\n'
        f'<p>{body_text}</p>\n'
        f'<img src="data:image/png;base64,{img_b64}" style="max-width:100%;margin:12px 0;">\n'
    )

    html = REPORT_HTML.read_text(encoding="utf-8")

    import re
    if "steer_deadband section" in html:
        print("既存の steer_deadband セクションを上書きします")
        html = re.sub(
            r'<!-- steer_deadband section.*?(?=</body>)',
            section_html.lstrip("\n"),
            html, flags=re.DOTALL,
        )
    else:
        html = html.replace("</body>", section_html + "</body>")

    REPORT_HTML.write_text(html, encoding="utf-8")
    print(f"レポート更新: {REPORT_HTML} ({REPORT_HTML.stat().st_size // 1024} KB)")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
