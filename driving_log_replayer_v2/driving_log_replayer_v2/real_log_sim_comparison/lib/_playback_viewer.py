"""
軌跡再生ビューア (trajectory playback) の生成.

実機 + 各 sim run の軌跡を地図上に重ね描きし、シークバーで進行させながら比較する
自己完結 HTML (canvas + インライン JS) を出力する。plotly 非依存・オフライン動作
（外部 JS/CSS 参照なし。step11 はこれを通常の *.html 図として iframe 埋め込みする）。

モード:
  - 時刻同期: 全 run 共通の t グリッド上で、同一経過時刻における各 run の位置を表示。
  - 位置同期 (同一走行距離): ベースライン (実機) の累積走行距離 s をシークし、
    各 sim run は「自軌跡の累積弧長が同じ s になる地点」に表示する (弧長一致方式)。
    同じ距離を走った時点の横ずれ比較用。経路長差も含むため厳密な横偏差ではない。

速度は現在位置から進行方向への矢印 (長さ = v × PREVIEW_SEC 秒の到達距離 [m]、
world 座標で定義しズームに追従) で示す。位置同期モードでは run ごとの経過時刻 t も
凡例に出すため、どの run が時間的に先行/遅延しているかも読み取れる。

DP 計画軌跡 (diffusion_planner_node/output/trajectory) があれば、各 run の
現在時刻における最新フレームを薄い破線で重ね描きする (チェックボックスで切替可)。

データは Python 側で共通 t グリッド (rate_hz) にリサンプリングして JSON 埋め込みし、
JS 側は線形補間と二分探索のみを行う。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

from ._map import map_ways_in_bbox

# 再生データのリサンプリングレート [Hz]。上げると滑らかになるが HTML サイズが増える。
PLAYBACK_RATE_HZ = 10.0
# 地図クリップ・全体表示のマージン [m]（plot_trajectory と同じ値）。
MAP_MARGIN_M = 30.0
# ベースライン (位置同期モードの基準) とする run ラベル。
BASELINE_LABEL = "実機"
# DP 計画軌跡の間引き設定 (HTML サイズ抑制)。フレームは最小間隔 [s]、点は stride 間引き。
DP_FRAME_MIN_DT = 0.5
DP_POINT_STRIDE = 3


def _round_list(arr: np.ndarray, ndigits: int) -> list[float]:
    """JSON サイズ削減のための一括丸め。"""
    return [round(float(v), ndigits) for v in arr]


def _resample_run(kin: pd.DataFrame, vel: pd.DataFrame, t_grid: np.ndarray) -> dict:
    """
    1 run の kinematic / velocity を共有 t グリッドへリサンプリングする。

    - yaw は ±π wrap で補間が暴れるため `np.unwrap` 後に補間する。
    - run の最終時刻より後のグリッド点は `np.interp` が端値ホールドする。
      `n_valid` (最終有効グリッド点数) を併記し、それ以降は v=0 に強制する
      （停止車両に速度矢印が出ないように。JS 側は ended 表示にも使う）。
    - 弧長 s はグリッド化後の座標から累積。端値ホールド区間は ds=0 で plateau し、
      単調非減少が保たれる（JS の二分探索の前提）。
    """
    t = kin["t"].to_numpy(dtype=float)
    x = np.interp(t_grid, t, kin["x"].to_numpy(dtype=float))
    y = np.interp(t_grid, t, kin["y"].to_numpy(dtype=float))
    yaw = np.interp(t_grid, t, np.unwrap(kin["yaw"].to_numpy(dtype=float)))
    if vel is not None and not vel.empty:
        v = np.interp(t_grid, vel["t"].to_numpy(dtype=float), vel["lon_vel"].to_numpy(dtype=float))
    else:
        v = np.zeros_like(t_grid)
    n_valid = int(np.searchsorted(t_grid, t[-1], side="right"))
    n_valid = max(n_valid, 1)
    v[n_valid:] = 0.0
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return {
        "x": _round_list(x, 2),
        "y": _round_list(y, 2),
        "yaw": _round_list(yaw, 3),
        "v": _round_list(v, 2),
        "s": _round_list(s, 2),
        "n_valid": n_valid,
        "s_total": round(float(s[n_valid - 1]), 2),
    }


def _compact_dp_frames(frames: list[dict]) -> dict:
    """DP 計画軌跡フレームを間引き・丸めして JSON 埋め込み用に圧縮する。

    frames: step4 `_load_dp_trajectories` の出力 ({"t", "x", "y"} のリスト、t 昇順)。
    戻り値: {"t": [...], "x": [[...]...], "y": [[...]...]} (保持フレームのみ)。
    点列は DP_POINT_STRIDE 間引き + 終端点保持 (計画長が見た目で縮まないように)。
    """
    out_t: list[float] = []
    out_x: list[list[float]] = []
    out_y: list[list[float]] = []
    last_t = -1e9
    for f in frames:
        t = float(f["t"])
        if t < 0 or t - last_t < DP_FRAME_MIN_DT:
            continue
        last_t = t
        xs = list(f["x"][::DP_POINT_STRIDE])
        ys = list(f["y"][::DP_POINT_STRIDE])
        if (len(f["x"]) - 1) % DP_POINT_STRIDE != 0:
            xs.append(f["x"][-1])
            ys.append(f["y"][-1])
        out_t.append(round(t, 2))
        out_x.append([round(float(v), 2) for v in xs])
        out_y.append([round(float(v), 2) for v in ys])
    return {"t": out_t, "x": out_x, "y": out_y}


def build_playback_payload(
    data: dict,
    map_ways: list | None,
    rate_hz: float = PLAYBACK_RATE_HZ,
    map_margin_m: float = MAP_MARGIN_M,
    title: str = "",
) -> dict | None:
    """
    step4 の `loaded` dict から埋め込み用 payload を構築する。

    Args:
        data: step4 main() の loaded。label -> {"kinematic": df(t,x,y,yaw),
              "velocity": df(t,lon_vel), "color": CSS色, ...}
        map_ways: `load_map_ways()` の戻り値 (None なら地図なし)。
        rate_hz: リサンプリングレート。
        map_margin_m: bbox マージン。
        title: ビューア上部に表示するシナリオ名。

    Returns:
        JSON 化可能な dict。kinematic が全 run 空なら None。

    """
    kins = {lbl: d["kinematic"] for lbl, d in data.items() if not d["kinematic"].empty}
    if not kins:
        return None

    t_max = max(float(k["t"].iloc[-1]) for k in kins.values())
    n = int(np.floor(t_max * rate_hz)) + 1
    t_grid = np.arange(n) / rate_hz

    runs = []
    for label, d in data.items():
        kin = d["kinematic"]
        if kin.empty:
            continue
        run = _resample_run(kin, d.get("velocity"), t_grid)
        run["label"] = label
        run["color"] = str(d["color"])
        run["is_baseline"] = label == BASELINE_LABEL
        run["dp"] = _compact_dp_frames(d.get("dp_traj") or [])
        runs.append(run)
    if not any(r["is_baseline"] for r in runs):
        runs[0]["is_baseline"] = True

    all_xy = np.concatenate([k[["x", "y"]].to_numpy(dtype=float) for k in kins.values()])
    x_min, y_min = all_xy.min(axis=0) - map_margin_m
    x_max, y_max = all_xy.max(axis=0) + map_margin_m
    bbox = [
        round(float(x_min), 1),
        round(float(y_min), 1),
        round(float(x_max), 1),
        round(float(y_max), 1),
    ]

    lanelets: list[list[list[float]]] = []
    if map_ways:
        for pts in map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max)):
            lanelets.append([[round(float(px), 2), round(float(py), 2)] for px, py in pts])

    return {
        "title": title,
        "rate_hz": rate_hz,
        "n": n,
        "bbox": bbox,
        "lanelets": lanelets,
        "runs": runs,
    }


def write_playback_html(payload: dict, out_path: Path) -> None:
    """Payload を埋め込んだ自己完結ビューア HTML を書き出す。"""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # "</" を "<\/" にエスケープして </script> によるパーサ早期終了を防ぐ。
    payload_json = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    payload_json = payload_json.replace("</", "<\\/")
    html = _HTML_TEMPLATE.replace("__PAYLOAD_JSON__", payload_json)
    out_path.write_text(html, encoding="utf-8")
    print(f"  保存: {out_path.name}")


def plot_trajectory_playback(
    data: dict,
    map_ways: list | None,
    figs_dir: Path,
    title: str = "",
) -> None:
    """step4 から呼ぶエントリポイント。`figs_dir/trajectory_playback.html` を生成。"""
    payload = build_playback_payload(data, map_ways, title=title)
    if payload is None:
        import warnings  # noqa: PLC0415

        warnings.warn("kinematic データなし。軌跡再生ビューアをスキップ", stacklevel=2)
        return
    write_playback_html(payload, Path(figs_dir) / "trajectory_playback.html")


# ---------------------------------------------------------------------------
# HTML テンプレート（__PAYLOAD_JSON__ を JSON で置換する）
# ---------------------------------------------------------------------------

_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<title>軌跡再生ビューア</title>
<style>
  html, body { margin: 0; height: 100%; font-family: "Hiragino Sans", "Noto Sans CJK JP", sans-serif; font-size: 13px; background: #fff; color: #333; }
  #app { display: flex; flex-direction: column; height: 100%; }
  #controls { display: flex; flex-wrap: wrap; gap: 4px 16px; align-items: center; padding: 6px 10px; border-bottom: 1px solid #ddd; background: #fafafa; }
  #controls .grp { display: flex; gap: 6px; align-items: center; white-space: nowrap; }
  #seekrow { display: flex; gap: 10px; align-items: center; padding: 4px 10px; border-bottom: 1px solid #ddd; background: #fafafa; }
  #seek { flex: 1; }
  #readout { min-width: 190px; font-family: monospace; font-size: 12px; text-align: right; }
  #hint { font-size: 11px; color: #777; padding: 3px 10px 6px; background: #fafafa; border-bottom: 1px solid #ddd; }
  #main { flex: 1; display: flex; min-height: 0; }
  #canvaswrap { flex: 1; position: relative; min-width: 0; background: #fdfdfd; }
  canvas { position: absolute; inset: 0; width: 100%; height: 100%; }
  #legend { width: 240px; overflow-y: auto; border-left: 1px solid #ddd; padding: 6px 8px; background: #fafafa; box-sizing: border-box; }
  .runrow { padding: 3px 0; border-bottom: 1px dotted #e5e5e5; }
  .runhead { display: flex; align-items: center; gap: 6px; font-size: 12px; }
  .sw { display: inline-block; width: 13px; height: 13px; border-radius: 3px; border: 1px solid rgba(0,0,0,.25); flex: none; }
  .runstat { font-family: monospace; font-size: 11px; color: #555; padding-left: 24px; white-space: pre; }
  .runrow.ended .runstat { color: #b05050; }
  button, select { font-size: 12px; }
  label { user-select: none; cursor: pointer; }
  #playbtn { min-width: 64px; }
</style>
</head>
<body>
<div id="app">
  <div id="controls">
    <span class="grp" id="titlebox" style="font-weight:bold"></span>
    <span class="grp">
      <label><input type="radio" name="mode" value="time" checked> 時刻同期</label>
      <label><input type="radio" name="mode" value="pos"> 位置同期（同一走行距離）</label>
    </span>
    <span class="grp">
      <button id="playbtn">▶ 再生</button>
      <select id="speed">
        <option value="0.5">0.5×</option>
        <option value="1" selected>1×</option>
        <option value="2">2×</option>
        <option value="4">4×</option>
        <option value="8">8×</option>
      </select>
    </span>
    <span class="grp">
      <label><input type="checkbox" id="follow" checked> 追従ズーム</label>
      視野幅 <input type="range" id="vieww" min="30" max="400" value="100" style="width:90px">
      <span id="viewwval" style="font-family:monospace">100m</span>
    </span>
    <span class="grp" id="dpgrp">
      <label><input type="checkbox" id="showdp" checked> DP計画軌跡（破線）</label>
    </span>
  </div>
  <div id="seekrow">
    <input type="range" id="seek" min="0" max="10000" value="0">
    <span id="readout"></span>
  </div>
  <div id="hint"></div>
  <div id="main">
    <div id="canvaswrap"><canvas id="cv"></canvas></div>
    <div id="legend"></div>
  </div>
</div>
<script>
"use strict";
const DATA = __PAYLOAD_JSON__;
(() => {
  const RATE = DATA.rate_hz;
  const N = DATA.n;
  const T_MAX = (N - 1) / RATE;
  const BBOX = DATA.bbox; // [x_min, y_min, x_max, y_max]
  const runs = DATA.runs;
  const baseline = runs.find((r) => r.is_baseline) || runs[0];
  const PREVIEW_SEC = 1.5;   // 速度矢印 = PREVIEW_SEC 秒後の到達距離 [m]
  const CRAWL_V = 0.5;       // 位置同期再生でベースライン停止中も進める最低速度 [m/s]
  runs.forEach((r) => { r.visible = true; });
  const hasDp = runs.some((r) => r.dp && r.dp.t.length > 0);

  // ------------------------------------------------------------------ state
  let mode = "time";   // "time" | "pos"
  let playing = false;
  let speedMul = 1;
  let showDp = hasDp;  // DP 計画軌跡 (破線) の表示
  let curT = 0;        // 時刻同期モードの現在時刻 [s]
  let curS = 0;        // 位置同期モードのベースライン走行距離 [m]
  const cam = { cx: 0, cy: 0, viewW: 100, targetViewW: 100, follow: true, init: false };

  // -------------------------------------------------------------------- DOM
  const $ = (id) => document.getElementById(id);
  const cv = $("cv");
  const ctx = cv.getContext("2d");
  const seekEl = $("seek");
  const readoutEl = $("readout");
  const playBtn = $("playbtn");
  const hintEl = $("hint");
  if (DATA.title) $("titlebox").textContent = DATA.title;

  // ------------------------------------------------------------ 補間ヘルパー
  // 時刻同期: 共有 t グリッドのインデックス線形補間
  function sampleAtT(run, t) {
    const fi = Math.min(Math.max(t * RATE, 0), N - 1);
    const i = Math.floor(fi);
    const j = Math.min(i + 1, N - 1);
    const f = fi - i;
    const L = (a) => a[i] + (a[j] - a[i]) * f;
    const tEnd = (run.n_valid - 1) / RATE;
    const ended = t > tEnd + 1e-9;
    return { x: L(run.x), y: L(run.y), yaw: L(run.yaw), s: L(run.s),
             v: ended ? 0 : L(run.v), t: Math.min(t, tEnd), ended };
  }

  // first index in [0, hi) with arr[idx] > v
  function upperBound(arr, v, hi) {
    let lo = 0;
    while (lo < hi) {
      const m = (lo + hi) >> 1;
      if (arr[m] > v) hi = m; else lo = m + 1;
    }
    return lo;
  }

  // 位置同期: 自軌跡の累積弧長が s になる地点（弧長一致）。run.s は単調非減少。
  function sampleAtS(run, s) {
    const nv = run.n_valid;
    const ended = s > run.s_total - 1e-6;
    if (nv < 2) {
      return { x: run.x[0], y: run.y[0], yaw: run.yaw[0], s: 0, v: 0, t: 0, ended };
    }
    const sc = Math.min(Math.max(s, 0), run.s_total);
    let j = upperBound(run.s, sc, nv);
    j = Math.min(Math.max(j, 1), nv - 1);
    const i = j - 1;
    const ds = run.s[j] - run.s[i];
    const f = ds > 1e-9 ? (sc - run.s[i]) / ds : 0;
    const L = (a) => a[i] + (a[j] - a[i]) * f;
    return { x: L(run.x), y: L(run.y), yaw: L(run.yaw), s: sc,
             v: ended ? 0 : L(run.v), t: (i + f) / RATE, ended };
  }

  const sample = (run) => (mode === "time" ? sampleAtT(run, curT) : sampleAtS(run, curS));

  // ------------------------------------------------------------------- 凡例
  const legendEl = $("legend");
  const statEls = [];
  runs.forEach((run, idx) => {
    const row = document.createElement("div");
    row.className = "runrow";
    const head = document.createElement("label");
    head.className = "runhead";
    const cb = document.createElement("input");
    cb.type = "checkbox";
    cb.checked = true;
    cb.addEventListener("change", () => { run.visible = cb.checked; markDirty(); });
    const sw = document.createElement("span");
    sw.className = "sw";
    sw.style.background = run.color;
    const name = document.createElement("span");
    name.textContent = run.label + (run.is_baseline ? "（基準）" : "");
    head.append(cb, sw, name);
    const stat = document.createElement("div");
    stat.className = "runstat";
    row.append(head, stat);
    legendEl.append(row);
    statEls[idx] = { row, stat };
  });

  function updateLegend(states) {
    runs.forEach((run, idx) => {
      const st = states[idx];
      const { row, stat } = statEls[idx];
      if (!st) { stat.textContent = "(非表示)"; row.classList.remove("ended"); return; }
      row.classList.toggle("ended", st.ended);
      const endNote = st.ended ? (mode === "pos" ? " [到達済/距離不足]" : " [終了]") : "";
      stat.textContent =
        "v " + st.v.toFixed(1).padStart(5) + " m/s" + endNote + "\n" +
        "t " + st.t.toFixed(1).padStart(6) + " s   s " + st.s.toFixed(1).padStart(7) + " m";
    });
  }

  // ------------------------------------------------------------------- 操作
  const HINTS = {
    time: "時刻同期: 各ログの AUTONOMOUS/発進を t=0 とした同一経過時刻の位置を重ね表示（pacing 差で同一 t ≠ 同一地点）。矢印 = 速度 × " + PREVIEW_SEC + "s の到達距離。",
    pos: "位置同期: 実機の走行距離 s を基準に、各 run を自軌跡で同じ距離を走った地点に表示（同一走行距離比較・経路長差を含むため厳密な横偏差ではない）。矢印 = その地点の速度、凡例の t でどの run が時間的に先行/遅延かが分かる。",
  };
  const DP_HINT = " 破線 = その時点で最後に発行された DP 計画軌跡。";
  const setHint = () => { hintEl.textContent = HINTS[mode] + (hasDp ? DP_HINT : ""); };
  function setMode(m) {
    if (m === mode) return;
    // モード間でシーク位置の連続性を保つ（ベースライン基準で変換）
    if (m === "pos") curS = sampleAtT(baseline, curT).s;
    else curT = sampleAtS(baseline, curS).t;
    mode = m;
    setHint();
    syncSeek();
    markDirty();
  }
  document.querySelectorAll("input[name=mode]").forEach((el) => {
    el.addEventListener("change", () => setMode(el.value));
  });
  setHint();
  // DP 計画軌跡トグル（データが無ければコントロールごと隠す）
  if (!hasDp) $("dpgrp").style.display = "none";
  $("showdp").addEventListener("change", (e) => { showDp = e.target.checked; markDirty(); });

  function setPlaying(p) {
    playing = p;
    playBtn.textContent = playing ? "⏸ 停止" : "▶ 再生";
  }
  playBtn.addEventListener("click", () => {
    // 末尾で再生を押したら先頭から
    if (!playing) {
      if (mode === "time" && curT >= T_MAX - 1e-9) curT = 0;
      if (mode === "pos" && curS >= baseline.s_total - 1e-6) curS = 0;
    }
    setPlaying(!playing);
  });
  $("speed").addEventListener("change", (e) => { speedMul = parseFloat(e.target.value); });

  const SEEK_MAX = 10000;
  function syncSeek() {
    const frac = mode === "time" ? (T_MAX > 0 ? curT / T_MAX : 0)
                                 : (baseline.s_total > 0 ? curS / baseline.s_total : 0);
    seekEl.value = Math.round(frac * SEEK_MAX);
    readoutEl.textContent = mode === "time"
      ? "t = " + curT.toFixed(1) + " / " + T_MAX.toFixed(1) + " s"
      : "s = " + curS.toFixed(1) + " / " + baseline.s_total.toFixed(1) + " m";
  }
  seekEl.addEventListener("input", () => {
    const frac = seekEl.valueAsNumber / SEEK_MAX;
    if (mode === "time") curT = frac * T_MAX;
    else curS = frac * baseline.s_total;
    syncSeek();
    markDirty();
  });

  $("follow").addEventListener("change", (e) => { cam.follow = e.target.checked; markDirty(); });
  $("vieww").addEventListener("input", (e) => {
    cam.targetViewW = e.target.valueAsNumber;
    $("viewwval").textContent = e.target.valueAsNumber + "m";
    markDirty();
  });

  // ------------------------------------------------------------------- 描画
  function drawArrowhead(x, y, ang, size) {
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x - size * Math.cos(ang - 0.4), y - size * Math.sin(ang - 0.4));
    ctx.lineTo(x - size * Math.cos(ang + 0.4), y - size * Math.sin(ang + 0.4));
    ctx.closePath();
    ctx.fill();
  }

  function drawScaleBar(w, h, scale) {
    const targetPx = 100;
    const meters = targetPx / scale;
    const pow = Math.pow(10, Math.floor(Math.log10(meters)));
    const cand = [1, 2, 5, 10].map((c) => c * pow);
    const m = cand.reduce((a, b) => (Math.abs(b - meters) < Math.abs(a - meters) ? b : a));
    const px = m * scale;
    const x0 = 12, y0 = h - 14;
    ctx.strokeStyle = "#333";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x0 + px, y0);
    ctx.moveTo(x0, y0 - 4); ctx.lineTo(x0, y0 + 4);
    ctx.moveTo(x0 + px, y0 - 4); ctx.lineTo(x0 + px, y0 + 4);
    ctx.stroke();
    ctx.fillStyle = "#333";
    ctx.font = "11px monospace";
    ctx.fillText(m >= 1 ? m + " m" : m.toFixed(1) + " m", x0 + 4, y0 - 6);
  }

  // 1 フレーム描画する。カメラがまだ目標へ移動中なら true を返す（=継続描画が必要）。
  function draw() {
    const wrap = cv.parentElement;
    const w = wrap.clientWidth;
    const h = wrap.clientHeight;
    if (w === 0 || h === 0) return false; // 非表示タブ内 (display:none) では描画スキップ
    const dpr = window.devicePixelRatio || 1;
    const pw = Math.round(w * dpr);
    const ph = Math.round(h * dpr);
    if (cv.width !== pw || cv.height !== ph) { cv.width = pw; cv.height = ph; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const states = runs.map((r) => (r.visible ? sample(r) : null));

    // --- カメラ
    // 追従 = 表示中 run の現在位置群をフィット（視野幅 slider は最小ズーム）。
    //   走行終了 (ended) の run は終端に静止し続けフィットを巻き込むため除外する
    //   （全 run 終了時はベースライン中心）。
    // 全体 = bbox フィット。
    let tx, ty, tvw;
    if (cam.follow) {
      const pts = states.filter((st) => st && !st.ended);
      if (pts.length === 0) {
        const bs = states[runs.indexOf(baseline)] || sample(baseline);
        tx = bs.x; ty = bs.y; tvw = cam.targetViewW;
      } else {
        const xs = pts.map((p) => p.x);
        const ys = pts.map((p) => p.y);
        const xMin = Math.min(...xs), xMax = Math.max(...xs);
        const yMin = Math.min(...ys), yMax = Math.max(...ys);
        tx = (xMin + xMax) / 2;
        ty = (yMin + yMax) / 2;
        // 全マーカー + 余白が収まる視野幅（slider 値を下回らない）
        tvw = Math.max(
          cam.targetViewW,
          (xMax - xMin) * 1.4 + 20,
          ((yMax - yMin) * 1.4 + 20) * (w / h),
        );
      }
    } else {
      tx = (BBOX[0] + BBOX[2]) / 2;
      ty = (BBOX[1] + BBOX[3]) / 2;
      // 縦横どちらも収まる視野幅
      tvw = Math.max(BBOX[2] - BBOX[0], (BBOX[3] - BBOX[1]) * (w / h)) * 1.05;
    }
    if (!cam.init) { cam.cx = tx; cam.cy = ty; cam.viewW = tvw; cam.init = true; }
    // シーク大ジャンプ等でターゲットが視野外に飛んだら lerp せず即スナップ
    // （lerp 追従だとマーカーが長時間画面外に残る）
    if (Math.hypot(tx - cam.cx, ty - cam.cy) > cam.viewW) {
      cam.cx = tx; cam.cy = ty;
    } else {
      cam.cx += (tx - cam.cx) * 0.15;
      cam.cy += (ty - cam.cy) * 0.15;
    }
    if (tvw > cam.viewW * 2 || tvw < cam.viewW / 2) cam.viewW = tvw;
    else cam.viewW += (tvw - cam.viewW) * 0.15;
    const scale = w / cam.viewW;
    const SX = (wx) => w / 2 + (wx - cam.cx) * scale;
    const SY = (wy) => h / 2 - (wy - cam.cy) * scale;

    // --- lanelet（薄灰）
    ctx.strokeStyle = "#c8c8c8";
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    for (const way of DATA.lanelets) {
      ctx.moveTo(SX(way[0][0]), SY(way[0][1]));
      for (let k = 1; k < way.length; k++) ctx.lineTo(SX(way[k][0]), SY(way[k][1]));
    }
    ctx.stroke();

    // --- 全軌跡（薄く）
    for (const run of runs) {
      if (!run.visible) continue;
      ctx.strokeStyle = run.color;
      ctx.globalAlpha = 0.3;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(SX(run.x[0]), SY(run.y[0]));
      for (let k = 1; k < run.n_valid; k++) ctx.lineTo(SX(run.x[k]), SY(run.y[k]));
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // --- DP 計画軌跡（現在時刻における最新フレームを薄い破線で。run 色・走行終了後は非表示）
    if (showDp) {
      ctx.save();
      ctx.setLineDash([6, 5]);
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.35;
      runs.forEach((run, idx) => {
        const st = states[idx];
        const dp = run.dp;
        if (!st || st.ended || !dp || dp.t.length === 0) return;
        // 現在時刻 st.t 以前に発行された最後のフレーム
        const k = upperBound(dp.t, st.t, dp.t.length) - 1;
        if (k < 0) return;
        const xs = dp.x[k];
        const ys = dp.y[k];
        ctx.strokeStyle = run.color;
        ctx.beginPath();
        ctx.moveTo(SX(xs[0]), SY(ys[0]));
        for (let m = 1; m < xs.length; m++) ctx.lineTo(SX(xs[m]), SY(ys[m]));
        ctx.stroke();
      });
      ctx.restore();
      ctx.globalAlpha = 1;
    }

    // --- 現在位置マーカー + 速度矢印
    runs.forEach((run, idx) => {
      const st = states[idx];
      if (!st) return;
      const px = SX(st.x);
      const py = SY(st.y);
      ctx.globalAlpha = st.ended ? 0.35 : 1;

      // 速度矢印（world 長 = v × PREVIEW_SEC [m]。停止/終了時は出さない）
      const lenM = st.v * PREVIEW_SEC;
      if (!st.ended && lenM * scale > 3) {
        const ex = SX(st.x + lenM * Math.cos(st.yaw));
        const ey = SY(st.y + lenM * Math.sin(st.yaw));
        ctx.strokeStyle = run.color;
        ctx.fillStyle = run.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(ex, ey);
        ctx.stroke();
        drawArrowhead(ex, ey, Math.atan2(ey - py, ex - px), 7);
      }

      // マーカー（基準 run は一回り大きく）
      ctx.beginPath();
      ctx.arc(px, py, run.is_baseline ? 7 : 5.5, 0, Math.PI * 2);
      ctx.fillStyle = run.color;
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = "#fff";
      ctx.stroke();
      ctx.globalAlpha = 1;
    });

    drawScaleBar(w, h, scale);
    updateLegend(states);

    // カメラの lerp 収束判定（未収束なら次フレームも描画）
    return (
      Math.hypot(tx - cam.cx, ty - cam.cy) > cam.viewW * 1e-3 ||
      Math.abs(tvw - cam.viewW) > cam.viewW * 2e-3
    );
  }

  // --------------------------------------------------------------- 再生ループ
  // dirty フラグ: 再生中・UI 操作・リサイズ時のみ再描画し、アイドル時の
  // 60fps 常時再描画を避ける（レポートを開きっぱなしでも CPU を食わない）。
  let dirty = true;
  const markDirty = () => { dirty = true; };
  // iframe/タブ表示切替やウィンドウリサイズでサイズが変わったら再描画
  // （非表示 display:none → 表示 で 0×0 から復帰するケースもここで拾う）。
  new ResizeObserver(markDirty).observe(cv.parentElement);

  let lastTs = null;
  function tick(ts) {
    if (lastTs !== null && playing) {
      const dt = Math.min((ts - lastTs) / 1000, 0.2) * speedMul;
      if (mode === "time") {
        curT += dt;
        if (curT >= T_MAX) { curT = T_MAX; setPlaying(false); }
      } else {
        const bs = sampleAtS(baseline, curS);
        curS += Math.max(bs.v, CRAWL_V) * dt; // 停止区間でも最低 CRAWL_V で進める
        if (curS >= baseline.s_total) { curS = baseline.s_total; setPlaying(false); }
      }
      syncSeek();
      dirty = true;
    }
    lastTs = ts;
    if (dirty) dirty = draw();
    requestAnimationFrame(tick);
  }

  syncSeek();
  requestAnimationFrame(tick);
})();
</script>
</body>
</html>
"""
