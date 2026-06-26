"""multi_dataset_tune 結果の HTML レポート生成。

最適化完了後に robust_search が返した ctxs を再利用してレポートを生成する。
データセットの再ロードは不要（同一プロセス内で ctxs を共有するため）。

generate_report() が公開 API。multi_dataset_tune.main() から呼ばれる。
"""

from __future__ import annotations

import html as _html_stdlib
import json
import math
import sys
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ._multi_agg import (
    HORIZONS,
    aggregate_normalized,
    normalize_components,
    robust_score,
)

# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------
H_FOCUS = 40          # worst DS 選定・可視化の基準 horizon
TOP_N_WORST = 5       # 各 config の worst-case DS top-N (和集合を取る)
REPORT_MAX_DS = 10    # per-DS プロットに表示する DS 数の上限
VIEWER_MAX_DS = 5     # 縦横モデル検証ビューアを埋め込む worst DS 上限（HTML サイズ抑制）

_COLORS = [
    "#4c72b0",  # blue
    "#e15759",  # red
    "#59a14f",  # green
    "#f28e2b",  # orange
    "#b07aa1",  # purple
    "#76b7b2",  # teal
]

_PLOTLY_CDN = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


# ---------------------------------------------------------------------------
# 内部ユーティリティ
# ---------------------------------------------------------------------------

def _eval_config(ctxs, params: dict, model_type: str, eval_fn) -> tuple[dict, list]:
    """全 DS を評価して (agg, per_ds_metrics) を返す。

    eval_fn: (ctx, override, model_type) -> {h: {yaw,long,lat,...}}
    """
    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    per_ds_metrics = [(ctx.dataset_id, eval_fn(ctx, params, model_type)) for ctx in ctxs]
    agg = aggregate_normalized(per_ds_metrics, baselines)
    return agg, per_ds_metrics


def _per_ds_score(per_ds_entry: dict) -> float:
    """N=H_FOCUS での複合正規化スコア (nyaw + nlong + nlat)。worst DS 選定用。"""
    b = per_ds_entry["by_h"].get(H_FOCUS)
    if b is None:
        return 0.0
    return b["nyaw"] + b["nlong"] + b["nlat"]


def _worst_ds_union(agg_results: dict[str, dict]) -> list[tuple[str, float]]:
    """各 config の worst-top-N の和集合を max スコア降順で返す。

    Returns list of (dataset_id, max_normalized_score_across_configs).
    """
    ds_max_score: dict[str, float] = {}
    for agg in agg_results.values():
        for entry in agg["per_ds"]:
            ds_id = entry["dataset_id"]
            s = _per_ds_score(entry)
            ds_max_score[ds_id] = max(ds_max_score.get(ds_id, 0.0), s)

    worst_ids: set[str] = set()
    for agg in agg_results.values():
        ranked = sorted(agg["per_ds"], key=_per_ds_score, reverse=True)
        for entry in ranked[:TOP_N_WORST]:
            worst_ids.add(entry["dataset_id"])

    result = sorted(worst_ids, key=lambda d: -ds_max_score[d])
    return [(ds_id, ds_max_score[ds_id]) for ds_id in result[:REPORT_MAX_DS]]


def _run_rollout_silent(ctx, params: dict, model_type: str, horizons, stride: int, s5) -> pd.DataFrame:
    """run_rollout を stdout 抑制で実行する。"""
    merged = dict(ctx.base)
    merged.update(params)
    gt = s5._prepare_gt(ctx.data, ctx.t0_ns, merged)

    old_stdout = sys.stdout
    sys.stdout = open("/dev/null", "w")  # noqa: WPS515
    try:
        df = s5.run_rollout(ctx.data, ctx.t0_ns, merged, model_type, horizons, stride, gt=gt)
    finally:
        sys.stdout.close()
        sys.stdout = old_stdout
    return df


def _ctx_to_viewer_data(ctx) -> dict | None:
    """DatasetCtx.data (load_real_bag 形式) を build_playback_payload 入力形式に変換する。

    t_ns → t [秒] 変換基準は ctx.t0_ns（自律走行開始時刻 = t=0）。
    t_launch=0.0 として自律走行開始から表示する。
    """
    from ._playback_viewer import BASELINE_LABEL  # noqa: PLC0415

    raw = ctx.data
    if raw["kin"].empty:
        return None

    t0 = ctx.t0_ns

    def to_t(df: pd.DataFrame) -> pd.DataFrame:
        d = df.copy()
        d["t"] = (d["t_ns"] - t0) / 1e9
        return d.drop(columns=["t_ns"])

    kin = to_t(raw["kin"])                                                  # t, x, y, yaw, vx, vy, wz
    vel = to_t(raw["vel"]).rename(columns={"vx": "lon_vel"})               # t, lon_vel
    acc = to_t(raw["acc"]).rename(columns={"ax": "accel"}) if not raw["acc"].empty else pd.DataFrame(columns=["t", "accel"])  # t, accel
    steer = to_t(raw["steer"])                                              # t, steer [rad]
    cmd = to_t(raw["cmd"]).rename(columns={"accel_des": "cmd_accel", "steer_des": "cmd_steer"}) if not raw["cmd"].empty else pd.DataFrame(columns=["t", "cmd_accel", "cmd_steer"])  # t, cmd_accel, cmd_steer

    return {
        BASELINE_LABEL: {
            "kinematic": kin,
            "velocity": vel,
            "accel": acc,
            "steering": steer,
            "cmd": cmd,
            "color": "#2196f3",
            "t_launch": 0.0,
            "dp_traj": [],
        }
    }


def _build_viewer_html(
    ctx,
    configs: dict[str, dict],
    base_params: dict,
    map_ways: list | None = None,
    initial_t: float | None = None,
) -> str | None:
    """worst-case DS の縦横モデル検証ビューア HTML 文字列を生成する。

    _model_viewer._HTML_TEMPLATE に payload を埋め込んで返す（ファイル書き出しなし）。
    configs の最初のエントリをつまみ初期値とし、全エントリを model_registry に登録する。

    注意: debug_steer_scaling_factor はビューアの JS モデルに非対応のため、
    ステア挙動はつまみで完全再現できない（探索ツールとして利用すること）。
    """
    from ._model_viewer import MODEL_RATE_HZ, _HTML_TEMPLATE, _seed_from_params  # noqa: PLC0415
    from ._playback_viewer import PLAYBACK_WHEELBASE_M, build_playback_payload  # noqa: PLC0415

    viewer_data = _ctx_to_viewer_data(ctx)
    if viewer_data is None:
        return None

    payload = build_playback_payload(viewer_data, map_ways=map_ways, rate_hz=MODEL_RATE_HZ, title=ctx.dataset_id)
    if payload is None:
        return None

    first_params = {**base_params, **next(iter(configs.values()))}
    if initial_t is not None and initial_t > 0:
        payload["initial_t"] = float(initial_t)
    payload["model_seed"] = _seed_from_params(first_params)

    registry: dict[str, dict] = {}
    for label, params in configs.items():
        registry[label] = _seed_from_params({**base_params, **params})
    payload["model_registry"] = registry
    payload["wheelbase"] = float(PLAYBACK_WHEELBASE_M)

    payload_json = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    payload_json = payload_json.replace("</", "<\\/")
    return _HTML_TEMPLATE.replace("__PAYLOAD_JSON__", payload_json)


# ---------------------------------------------------------------------------
# HTML 構築
# ---------------------------------------------------------------------------

def _score_table_html(agg_results: dict[str, dict], worst_w: float) -> str:
    header_cells = "<th>設定</th>"
    for h in HORIZONS:
        for m in ["nyaw_mean", "nyaw_worst", "nlong_mean", "nlong_worst", "nlat_mean", "nlat_worst"]:
            label = m.replace("nyaw", "Nyaw").replace("nlong", "Nlong").replace("nlat", "Nlat").replace("_", " ")
            header_cells += f"<th>N{h}<br>{label}</th>"
    header_cells += f"<th>robust<br>(w={worst_w})</th>"

    rows = []
    best_rs = min(robust_score(agg, worst_w=worst_w) for agg in agg_results.values())
    for cfg_name, agg in agg_results.items():
        rs = robust_score(agg, worst_w=worst_w)
        bold = rs == best_rs
        row = f"<tr><td><b>{cfg_name}</b></td>"
        for h in HORIZONS:
            b = agg["by_h"][h]
            for m in ["nyaw_mean", "nyaw_worst", "nlong_mean", "nlong_worst", "nlat_mean", "nlat_worst"]:
                val = b[m]
                row += f"<td>{val:.3f}</td>"
        score_td = f"<b>{rs:.4f}</b>" if bold else f"{rs:.4f}"
        row += f"<td style='{'background:#cfe;' if bold else ''}'>{score_td}</td></tr>"
        rows.append(row)

    return f"""<table border="1" cellpadding="5" cellspacing="0"
  style="border-collapse:collapse;font-size:12px;margin:8px 0">
  <thead style="background:#dde"><tr>{header_cells}</tr></thead>
  <tbody>{"".join(rows)}</tbody>
</table>"""


def _worst_ds_table_html(
    worst_ds_list: list[tuple[str, float]],
    agg_results: dict[str, dict],
    baselines: dict,
) -> str:
    header = "<th>DS ID (prefix)</th>"
    for cfg_name in agg_results:
        header += f"<th>{cfg_name}<br>N{H_FOCUS} nyaw/nlong/nlat<br>yaw[°] long[cm] lat[cm]</th>"
    header += f"<th>Baseline N{H_FOCUS}<br>yaw[°] long[cm] lat[cm]</th>"

    per_ds_idx: dict[str, dict[str, dict]] = {}
    for cfg_name, agg in agg_results.items():
        per_ds_idx[cfg_name] = {e["dataset_id"]: e for e in agg["per_ds"]}

    rows = []
    for ds_id, max_score in worst_ds_list:
        row = f"<tr><td><code>{ds_id[:12]}</code></td>"
        for cfg_name in agg_results:
            entry = per_ds_idx[cfg_name].get(ds_id)
            if entry is None:
                row += "<td>-</td>"
                continue
            b = entry["by_h"].get(H_FOCUS, {})
            s = b.get("nyaw", 0) + b.get("nlong", 0) + b.get("nlat", 0)
            row += (
                f"<td>"
                f"{b.get('nyaw',0):.3f}/{b.get('nlong',0):.3f}/{b.get('nlat',0):.3f}<br>"
                f"<small>{b.get('yaw',0):.3f}° {b.get('long',0):.2f}cm {b.get('lat',0):.2f}cm</small><br>"
                f"<b>score={s:.3f}</b>"
                f"</td>"
            )
        bl = baselines.get(ds_id, {}).get(H_FOCUS, {})
        row += (
            f"<td>{bl.get('yaw',0):.3f}° / {bl.get('long',0):.2f}cm / {bl.get('lat',0):.2f}cm</td>"
        )
        row += "</tr>"
        rows.append(row)

    return f"""<table border="1" cellpadding="5" cellspacing="0"
  style="border-collapse:collapse;font-size:11px;margin:8px 0">
  <thead style="background:#dde"><tr>{header}</tr></thead>
  <tbody>{"".join(rows)}</tbody>
</table>"""


def _ds_figure(
    ds_id: str,
    rollout_by_cfg: dict[str, pd.DataFrame],
    cfg_colors: dict[str, str],
) -> go.Figure:
    """1 DS の 2×2 サブプロット: マップ + yaw/long/lat 誤差時系列。"""
    fig = make_subplots(
        rows=2, cols=2,
        subplot_titles=[
            f"N{H_FOCUS} 開始位置マップ (色=|yaw誤差|[deg])",
            f"N{H_FOCUS} yaw誤差 [deg]",
            f"N{H_FOCUS} long誤差 [cm]",
            f"N{H_FOCUS} lat誤差 [cm]",
        ],
        column_widths=[0.5, 0.5],
        row_heights=[0.5, 0.5],
        horizontal_spacing=0.09,
        vertical_spacing=0.14,
    )

    # GT trajectory (最初の config から取得)
    ref_df = next(
        (df[df["horizon"] == H_FOCUS].sort_values("k0") for df in rollout_by_cfg.values() if df is not None),
        None,
    )
    if ref_df is not None:
        fig.add_trace(go.Scatter(
            x=ref_df["pos_x"], y=ref_df["pos_y"],
            mode="lines", line=dict(color="lightgray", width=1),
            name="GT trajectory", showlegend=True,
        ), row=1, col=1)

    first_map = True
    for cfg_name, df_cfg in rollout_by_cfg.items():
        if df_cfg is None:
            continue
        color = cfg_colors.get(cfg_name, "#999")
        df_h = df_cfg[df_cfg["horizon"] == H_FOCUS].sort_values("k0")

        # マップ: 開始位置を |yaw誤差| で色付け
        fig.add_trace(go.Scatter(
            x=df_h["pos_x"], y=df_h["pos_y"],
            mode="markers",
            marker=dict(
                color=df_h["yaw_err_deg"].abs(),
                colorscale="Oranges",
                size=5,
                showscale=first_map,
                colorbar=dict(
                    title="|yaw|[°]", thickness=10,
                    len=0.45, y=0.78,
                ) if first_map else None,
            ),
            name=f"{cfg_name} map", showlegend=False,
        ), row=1, col=1)
        first_map = False

        # yaw誤差
        fig.add_trace(go.Scatter(
            x=df_h["tr"], y=df_h["yaw_err_deg"],
            mode="lines", line=dict(color=color, width=1.3),
            name=cfg_name, showlegend=True,
        ), row=1, col=2)

        # long誤差 [cm]
        fig.add_trace(go.Scatter(
            x=df_h["tr"], y=df_h["err_ds_long"] * 100,
            mode="lines", line=dict(color=color, width=1.3),
            name=cfg_name, showlegend=False,
        ), row=2, col=1)

        # lat誤差 [cm]
        fig.add_trace(go.Scatter(
            x=df_h["tr"], y=df_h["err_ds_lat"] * 100,
            mode="lines", line=dict(color=color, width=1.3),
            name=cfg_name, showlegend=False,
        ), row=2, col=2)

    fig.update_xaxes(title_text="pos_x [m]", row=1, col=1)
    fig.update_yaxes(title_text="pos_y [m]", row=1, col=1)
    fig.update_xaxes(title_text="時間 [s]", row=1, col=2)
    fig.update_yaxes(title_text="yaw誤差 [deg]", zeroline=True, row=1, col=2)
    fig.update_xaxes(title_text="時間 [s]", row=2, col=1)
    fig.update_yaxes(title_text="long誤差 [cm]", zeroline=True, row=2, col=1)
    fig.update_xaxes(title_text="時間 [s]", row=2, col=2)
    fig.update_yaxes(title_text="lat誤差 [cm]", zeroline=True, row=2, col=2)

    fig.update_layout(
        title=f"Dataset: {ds_id}",
        height=650,
        margin=dict(l=50, r=50, t=80, b=40),
    )
    return fig


# ---------------------------------------------------------------------------
# 公開 API
# ---------------------------------------------------------------------------

def generate_report(
    ctxs,
    configs: dict[str, dict],
    out_html: Path,
    model_type: str,
    eval_fn,
    *,
    stride: int,
    worst_w: float = 1.0,
    plot_configs: list[str] | None = None,
) -> None:
    """最適化結果の HTML レポートを生成する。

    Parameters
    ----------
    ctxs : list[DatasetCtx]
        load_datasets が返した DatasetCtx リスト（再ロード不要）。
    configs : dict[str, dict]
        {ラベル: params dict}。最初のエントリが「今回の結果」。
    out_html : Path
        出力 HTML ファイルパス。
    model_type : str
        rollout に使う車両モデル種別。
    eval_fn : callable
        (ctx, override, model_type) -> {h: {yaw,long,lat,...}} の評価関数。
    stride : int
        run_rollout の stride。
    worst_w : float
        robust_score の worst 重み（スコア表示用）。
    plot_configs : list[str] | None
        per-DS プロットに使う config ラベルのリスト（None なら最初の 2 つ）。
    """
    # step5 モジュールは上位 package から参照する（循環回避）
    from .. import step5_analyze_nstep as s5  # noqa: PLC0415

    print(f"[report] レポート生成開始: {len(ctxs)} DS × {len(configs)} configs")

    baselines = {ctx.dataset_id: ctx.base_metric for ctx in ctxs}
    ctx_map = {ctx.dataset_id: ctx for ctx in ctxs}

    # 全 config を評価
    agg_results: dict[str, dict] = {}
    for cfg_name, params in configs.items():
        print(f"[report] {cfg_name} 評価中...", end=" ", flush=True)
        agg, _ = _eval_config(ctxs, params, model_type, eval_fn)
        agg_results[cfg_name] = agg
        rs = robust_score(agg, worst_w=worst_w)
        print(f"robust_score(w={worst_w})={rs:.4f}")

    # worst-case DS 特定
    worst_ds_list = _worst_ds_union(agg_results)
    print(f"[report] worst-case DS: {len(worst_ds_list)} 件")

    # per-DS プロット用 config を決定
    all_labels = list(configs.keys())
    if plot_configs is None:
        plot_cfgs = all_labels[:2]
    else:
        plot_cfgs = [c for c in plot_configs if c in configs]
    cfg_colors = {label: _COLORS[i % len(_COLORS)] for i, label in enumerate(all_labels)}

    # run_rollout（plot 対象 config のみ）
    rollout_data: dict[tuple[str, str], pd.DataFrame] = {}
    for ds_id, _ in worst_ds_list:
        ctx = ctx_map[ds_id]
        for cfg_name in plot_cfgs:
            print(f"[report] rollout: {ds_id[:12]}/{cfg_name}", flush=True)
            rollout_data[(ds_id, cfg_name)] = _run_rollout_silent(
                ctx, configs[cfg_name], model_type, HORIZONS, stride, s5
            )

    # 整合性チェック（最初の worst DS だけ）
    if worst_ds_list and plot_cfgs:
        ds_id0, _ = worst_ds_list[0]
        cfg0 = plot_cfgs[0]
        df = rollout_data.get((ds_id0, cfg0))
        if df is not None:
            df40 = df[df["horizon"] == H_FOCUS]
            rollout_yaw = math.sqrt((df40["yaw_err_deg"] ** 2).mean())
            agg_per_ds = {e["dataset_id"]: e for e in agg_results[cfg0]["per_ds"]}
            eval_yaw = agg_per_ds[ds_id0]["by_h"][H_FOCUS]["yaw"]
            match = "✓" if abs(rollout_yaw - eval_yaw) < 0.02 else "✗ MISMATCH"
            print(f"[report] 整合性: {ds_id0[:12]}/{cfg0} rollout={rollout_yaw:.3f}° eval={eval_yaw:.3f}° {match}")

    # 縦横モデル検証ビューア（top VIEWER_MAX_DS DSのみ生成・HTML重量対策）
    base_params = ctxs[0].base if ctxs else {}
    viewer_htmls: dict[str, str] = {}
    for ds_id, _ in worst_ds_list[:VIEWER_MAX_DS]:
        ctx = ctx_map[ds_id]
        print(f"[report] viewer: {ds_id[:12]}", flush=True)
        vh = _build_viewer_html(ctx, configs, base_params)
        if vh is not None:
            viewer_htmls[ds_id] = vh

    # HTML 構築
    score_tbl = _score_table_html(agg_results, worst_w)
    worst_tbl = _worst_ds_table_html(worst_ds_list, agg_results, baselines)

    scaling_note = ""
    if any("debug_steer_scaling_factor" in p for p in configs.values()):
        scaling_note = (
            "<p style='color:#b55;font-size:12px'>"
            "⚠️ <b>注意</b>: <code>debug_steer_scaling_factor</code> はビューアの JS モデルに非対応。"
            "ステア挙動はつまみで完全再現できません（探索用近似モデルとして利用してください）。"
            "</p>"
        )

    ds_sections: list[str] = []
    for ds_id, max_score in worst_ds_list:
        rollout_by_cfg = {
            cfg: rollout_data.get((ds_id, cfg))
            for cfg in plot_cfgs
        }
        if not any(v is not None for v in rollout_by_cfg.values()):
            continue
        fig = _ds_figure(ds_id, rollout_by_cfg, cfg_colors)
        plotly_html = fig.to_html(full_html=False, include_plotlyjs=False)

        viewer_section = ""
        if ds_id in viewer_htmls:
            srcdoc = _html_stdlib.escape(viewer_htmls[ds_id], quote=True)
            viewer_section = f"""
<h4>縦横モデル検証ビューア（インタラクティブ）</h4>
{scaling_note}
<p style="font-size:11px;color:#888">
  ドロップダウンで config を切り替え、つまみでパラメータを調整できます。
  「最適化」ボタンで最小二乗フィットも実行できます。
  地図（lanelet）は非表示・<code>pitch</code> 非対応のため勾配パネルは常時 0 です。
</p>
<iframe srcdoc="{srcdoc}"
  width="100%" height="680" style="border:1px solid #ccc;border-radius:4px"
  loading="lazy"></iframe>
"""

        ds_sections.append(f"""
<h3>Dataset: <code>{ds_id}</code>　worst複合スコア={max_score:.3f}</h3>
{plotly_html}
{viewer_section}
""")

    compare_labels = " / ".join(plot_cfgs)
    report_html = f"""<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <title>multi_dataset_tune 結果レポート</title>
  {_PLOTLY_CDN}
  <style>
    body {{ font-family: sans-serif; max-width: 1400px; margin: 0 auto; padding: 20px; }}
    h1 {{ color: #333; }}
    h2 {{ color: #555; border-bottom: 2px solid #aaa; padding-bottom: 4px; margin-top: 32px; }}
    h3 {{ color: #666; }}
    h4 {{ color: #777; margin-top: 20px; }}
    table {{ margin: 8px 0; }}
    td, th {{ padding: 4px 8px; }}
    code {{ background: #f0f0f0; padding: 2px 4px; border-radius: 3px; font-size: 11px; }}
    .meta {{ color: #888; font-size: 11px; }}
  </style>
</head>
<body>
<h1>multi_dataset_tune 結果レポート</h1>
<p class="meta">
  DSs: {len(ctxs)}件 ／
  robust_score: worst_w={worst_w}、h∈{list(HORIZONS)} ／
  per-DS比較: <b>{compare_labels}</b> ／
  worst DS基準: N{H_FOCUS} (nyaw+nlong+nlat)
</p>

<h2>1. スコア比較</h2>
<p>正規化スコア（baseline比）。<b>太字・緑背景</b>が最良。</p>
{score_tbl}

<h2>2. Worst-case Dataset 一覧</h2>
<p>N{H_FOCUS}正規化スコア上位 {len(worst_ds_list)}件（各config top-{TOP_N_WORST}の和集合）。</p>
{worst_tbl}

<h2>3. Worst-case Dataset 可視化</h2>
<p>比較: <b>{compare_labels}</b>。
マップは N{H_FOCUS} 開始位置を |yaw誤差| で色付け。誤差は「実機−モデル」。<br>
上位 {min(VIEWER_MAX_DS, len(worst_ds_list))} DSには縦横モデル検証ビューア（インタラクティブ）を埋め込んでいます。</p>
{"".join(ds_sections)}

</body>
</html>
"""
    out_html.parent.mkdir(parents=True, exist_ok=True)
    out_html.write_text(report_html, encoding="utf-8")
    print(f"[report] 完了: {out_html}")
