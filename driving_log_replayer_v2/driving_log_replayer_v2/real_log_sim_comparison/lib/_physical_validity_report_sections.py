"""HTML section builders for physical validity reports.

Keep report-only HTML assembly separate from numerical kernels and figure builders.
"""

from __future__ import annotations

from ._figures import build_fig_residual_candidates_hist


def build_equation_residual_optimization_report_html(
    results: list[dict],
    *,
    channel_label: str,
    unit_label: str,
    delay_symbol: str,
    tau_symbol: str,
    tau_inv_symbol: str,
    extra_note: str = "",
) -> str:
    """方程式残差パラメータ最適化の候補表 + residual histogram HTML を共通生成する。"""
    if not results:
        return ""

    fig = build_fig_residual_candidates_hist(
        results,
        channel_label=channel_label,
        unit_label=unit_label,
    )
    rows = ""
    for r in results:
        selected_style = "style='background-color: #e8f5e9; font-weight: bold;'" if r["selected"] else ""
        selected_text = "<b>★ 選択</b>" if r["selected"] else "—"
        rows += f"""<tr {selected_style}>
            <td>{r['delay']:.3f} s</td>
            <td>{r['tau']:.3f} s</td>
            <td>{r['rmse']:.4f} {unit_label}</td>
            <td>{r['mean']:.4e} {unit_label}</td>
            <td>{r['std']:.4f} {unit_label}</td>
            <td>{selected_text}</td>
        </tr>"""

    fig_html = fig.to_html(full_html=False, include_plotlyjs=False)
    extra = f" {extra_note}" if extra_note else ""
    return f"""
    <details>
      <summary><b>遅延固定時の時定数最適化と方程式残差の評価結果（クリックで展開）</b></summary>
      <p>
        遅延時間 \\({delay_symbol}\\) を \\(\\Delta t\\) の整数倍に固定した各候補について、
        方程式残差の二乗和（MSE）を最小化するように \\({tau_inv_symbol}\\) を同時最小二乗で最適化した結果です。
        判定が「★ 選択」された行が、候補横断で residual RMSE 最小の組です。
        全データセットから抽出した動的サンプルをプールして評価しています。{extra}
      </p>
      <table class="param-table">
        <tr><th>固定遅延 \\({delay_symbol}\\)</th><th>最適時定数 \\({tau_symbol}\\)</th><th>方程式残差 RMSE</th><th>残差平均</th><th>残差標準偏差</th><th>判定</th></tr>
        {rows}
      </table>
      {fig_html}
    </details>
    """
