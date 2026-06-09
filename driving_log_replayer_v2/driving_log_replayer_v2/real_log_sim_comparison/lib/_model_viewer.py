"""
縦横独立モデル検証ビューア (longitudinal/lateral model replay) の生成.

実機 rosbag のみを対象に、車両の縦・横独立モデルが十分かを対話的に検証する自己完結 HTML
(canvas + インライン JS、plotly 非依存・オフライン動作) を出力する。

検証モデル (いずれも 1 次遅れ + 無駄時間 T):
  縦   : acc_lon'(t)  = -(acc_lon(t)  - acc_lon_cmd(t-T)) / tau
  ステア: delta'(t)   = -(delta(t)    - delta_cmd(t-T))   / tau

指令値からこのモデルで前方積算したシミュレーション値を観測値・指令値と重ね描きし、
単一時定数モデルで応答が再現できるかを目で確認する。オフラインの N-step rollout (step5) が
RMSE で行う検証の対話版で、無駄時間 T・時定数 tau をつまみで変えながら当てはまりを見る。
地図ビューを併設し、走行文脈 (カーブ旋回など) と応答を対応づけられる。

仕様:
  - シミュレーション起点 = プロット時間窓の左端 (シーク時刻から窓いっぱい先まで前方ロールアウト)。
  - 対象は実機 (BASELINE_LABEL) のみ。"sim は介在しない" 実機解析セクションの趣旨に沿う。
  - つまみ初期値は simulator_model.param.yaml の設定値をシード (load_sim_params のフォールバック付)。

ペイロード構築は _playback_viewer.build_playback_payload を再利用する (地図 lanelet・実機軌跡・
加速度/ステアの指令&観測チャンネル `ch` を生成済み)。
"""

from __future__ import annotations

import json
import warnings
from pathlib import Path

from ._playback_viewer import BASELINE_LABEL, build_playback_payload

# 指令ステップのエッジを残し無駄時間応答を見やすくするためのリサンプリングレート [Hz]。
# 単一 run・少数チャンネルのみ埋め込むため、軌跡ビューア (10Hz) より高くても HTML は小さい。
MODEL_RATE_HZ = 50.0


def plot_model_viewer(
    data: dict,
    map_ways: list | None,
    figs_dir: Path,
    sim_params: dict,
    title: str = "",
) -> None:
    """step4 から呼ぶエントリポイント。`figs_dir/lon_lat_model.html` を生成。

    Args:
        data: step4 main() の loaded (label -> {"kinematic", "velocity", "accel",
              "cmd", "steering", "color", "t_launch", ...})。
        map_ways: `load_map_ways()` の戻り値 (None なら地図なし)。
        figs_dir: 出力先 (comparison/figures/)。
        sim_params: `load_sim_params()` の戻り値。つまみ初期値のシードに使う。
        title: ビューア上部に表示するシナリオ名。
    """
    # 実機のみ抽出 (無ければ先頭 run をフォールバック)。
    if BASELINE_LABEL in data:
        real = {BASELINE_LABEL: data[BASELINE_LABEL]}
    else:
        real = dict(list(data.items())[:1])

    payload = build_playback_payload(real, map_ways, rate_hz=MODEL_RATE_HZ, title=title)
    if payload is None:
        warnings.warn("kinematic データなし。縦横モデルビューアをスキップ", stacklevel=2)
        return

    # つまみ初期値 [s]。load_sim_params はフォールバック付きで常に dict を返す。
    payload["model_seed"] = {
        "tau_acc": float(sim_params["acc_time_constant"]),
        "t_acc": float(sim_params["acc_time_delay"]),
        "tau_steer": float(sim_params["steer_time_constant"]),
        "t_steer": float(sim_params["steer_time_delay"]),
    }

    out_path = Path(figs_dir) / "lon_lat_model.html"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    payload_json = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    # "</" を "<\/" にエスケープして </script> によるパーサ早期終了を防ぐ。
    payload_json = payload_json.replace("</", "<\\/")
    html = _HTML_TEMPLATE.replace("__PAYLOAD_JSON__", payload_json)
    out_path.write_text(html, encoding="utf-8")
    print(f"  保存: {out_path.name}")


# ---------------------------------------------------------------------------
# HTML テンプレート（__PAYLOAD_JSON__ を JSON で置換する）
# 地図描画・カメラ lerp は _playback_viewer の draw() を単一 run・時刻同期に簡略化して移植。
# ---------------------------------------------------------------------------

_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<title>縦横独立モデル検証ビューア</title>
<style>
  html, body { margin: 0; height: 100%; font-family: "Hiragino Sans", "Noto Sans CJK JP", sans-serif; font-size: 13px; background: #fff; color: #333; }
  #app { display: flex; flex-direction: column; height: 100%; }
  #controls { display: flex; flex-wrap: wrap; gap: 4px 16px; align-items: center; padding: 6px 10px; border-bottom: 1px solid #ddd; background: #fafafa; }
  #controls .grp { display: flex; gap: 6px; align-items: center; white-space: nowrap; }
  #knobs { display: flex; flex-wrap: wrap; gap: 4px 18px; align-items: center; padding: 5px 10px; border-bottom: 1px solid #ddd; background: #f4f6fa; }
  #knobs .kg { display: flex; gap: 5px; align-items: center; white-space: nowrap; }
  #knobs .kg input[type=range] { width: 110px; }
  #knobs .kval { font-family: monospace; font-size: 12px; min-width: 52px; text-align: right; }
  #knobs .ktitle { font-weight: 600; color: #2b4a8b; }
  #seekrow { display: flex; gap: 10px; align-items: center; padding: 4px 10px; border-bottom: 1px solid #ddd; background: #fafafa; }
  #seek { flex: 1; }
  #readout { min-width: 150px; font-family: monospace; font-size: 12px; text-align: right; }
  #hint { font-size: 11px; color: #777; padding: 3px 10px 6px; background: #fafafa; border-bottom: 1px solid #ddd; }
  #main { flex: 1; display: flex; min-height: 0; }
  #canvaswrap { flex: 1; position: relative; min-width: 0; background: #fdfdfd; }
  #cv { position: absolute; inset: 0; width: 100%; height: 100%; }
  #plots { flex: none; height: 480px; position: relative; border-top: 1px solid #ddd; background: #fff; }
  #plots canvas { position: absolute; inset: 0; width: 100%; height: 100%; }
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
      <button id="playbtn">▶ 再生</button>
      <select id="speed">
        <option value="0.5">0.5×</option>
        <option value="1" selected>1×</option>
        <option value="2">2×</option>
        <option value="4">4×</option>
      </select>
    </span>
    <span class="grp">
      <label><input type="checkbox" id="follow" checked> 追従ズーム</label>
      視野幅 <input type="range" id="vieww" min="30" max="400" value="100" style="width:90px">
      <span id="viewwval" style="font-family:monospace">100m</span>
    </span>
    <span class="grp">
      予測窓 <input type="range" id="plotwin" min="2" max="30" value="8" style="width:90px">
      <span id="plotwinval" style="font-family:monospace">8s</span>
    </span>
  </div>
  <div id="knobs">
    <span class="kg"><span class="ktitle">縦(加速度)</span></span>
    <span class="kg">無駄時間 T <input type="range" id="k_t_acc" min="0" max="0.5" step="0.005" value="0.1"><span class="kval" id="v_t_acc">0.100s</span></span>
    <span class="kg">時定数 τ <input type="range" id="k_tau_acc" min="0.02" max="1.5" step="0.005" value="0.26"><span class="kval" id="v_tau_acc">0.260s</span></span>
    <span class="kg" style="margin-left:8px"><span class="ktitle">横(ステア)</span></span>
    <span class="kg">無駄時間 T <input type="range" id="k_t_steer" min="0" max="0.5" step="0.005" value="0.03"><span class="kval" id="v_t_steer">0.030s</span></span>
    <span class="kg">時定数 τ <input type="range" id="k_tau_steer" min="0.02" max="1.5" step="0.005" value="0.50"><span class="kval" id="v_tau_steer">0.500s</span></span>
  </div>
  <div id="seekrow">
    <input type="range" id="seek" min="0" max="10000" value="0">
    <span id="readout"></span>
  </div>
  <div id="hint"></div>
  <div id="main">
    <div id="canvaswrap"><canvas id="cv"></canvas></div>
  </div>
  <div id="plots">
    <canvas id="plotcv"></canvas>
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
  const run = DATA.runs[0];
  const SIM_COLOR = "#d62728";   // シミュレーション線 (赤)
  const CMD_COLOR = "#888";      // 指令線 (灰・破線)
  const PREVIEW_SEC = 1.5;       // 速度矢印 = PREVIEW_SEC 秒後の到達距離 [m]
  const SUBSTEP = 10;            // Euler sub-step 数/グリッドセル (tau が小さくても発振させない)

  // 検証パネル定義 (上=縦加速度, 下=ステア角)。meas/cmd は run.ch のキー、tauKey/tKey は model のキー。
  const PANELS = [
    { label: "縦加速度", unit: "m/s²", meas: "accel", cmd: "cmd_accel", tauKey: "tau_acc", tKey: "t_acc" },
    { label: "ステア角", unit: "deg", meas: "steer", cmd: "cmd_steer", tauKey: "tau_steer", tKey: "t_steer" },
  ];

  // ----------------------------------------------------------------- state
  let playing = false;
  let speedMul = 1;
  let plotWindowS = 8;   // 予測窓幅 [s]。窓 = [curT, curT + plotWindowS] (左端=起点)。
  let curT = 0;          // 現在時刻 (= シミュレーション起点) [s]
  const cam = { cx: 0, cy: 0, viewW: 100, targetViewW: 100, follow: true, init: false };
  const model = {
    tau_acc: DATA.model_seed.tau_acc, t_acc: DATA.model_seed.t_acc,
    tau_steer: DATA.model_seed.tau_steer, t_steer: DATA.model_seed.t_steer,
  };

  // ------------------------------------------------------------------ DOM
  const $ = (id) => document.getElementById(id);
  const cv = $("cv");
  const ctx = cv.getContext("2d");
  const seekEl = $("seek");
  const readoutEl = $("readout");
  const playBtn = $("playbtn");
  const plotsEl = $("plots");
  const plotCv = $("plotcv");
  const plotCtx = plotCv.getContext("2d");
  if (DATA.title) $("titlebox").textContent = DATA.title + " ｜ 縦横モデル検証";

  // --------------------------------------------------------- 補間ヘルパー
  // 時刻同期: 共有 t グリッドのインデックス線形補間 (x,y,yaw,v)。
  function sampleAtT(t) {
    const fi = Math.min(Math.max(t * RATE, 0), N - 1);
    const i = Math.floor(fi);
    const j = Math.min(i + 1, N - 1);
    const f = fi - i;
    const L = (a) => a[i] + (a[j] - a[i]) * f;
    const tEnd = (run.n_valid - 1) / RATE;
    const ended = t > tEnd + 1e-9;
    return { x: L(run.x), y: L(run.y), yaw: L(run.yaw), v: ended ? 0 : L(run.v), ended };
  }

  // チャンネル値を時刻 t [s] で全域線形補間。範囲外/null は null (= 線の break)。
  // 無駄時間で過去を参照するため窓スライスでなく全配列から引く。
  function chanAt(key, t) {
    const arr = run.ch && run.ch[key];
    if (!arr) return null;
    const fi = t * RATE;
    if (fi < -1e-9 || fi > N - 1 + 1e-9) return null;
    const i = Math.max(0, Math.floor(fi));
    const j = Math.min(i + 1, N - 1);
    const a = arr[i], b = arr[j];
    if (a == null || b == null) return null;
    return a + (b - a) * (fi - i);
  }

  // 1 次遅れ + 無駄時間モデルを起点 t0 の観測値から窓右端 t1 まで前方積算する。
  //   x'(t) = -(x(t) - u(t - T)) / tau ,  u = 指令 (cmdKey), 初期値 = 観測(measKey) at t0
  // sub-step Euler で安定化。指令が範囲外の区間は最後の有効指令をホールド。
  // 戻り値: [{t, v}, ...] (出力は RATE 解像度。観測初期値が無ければ空配列)。
  function simulate(panel, t0, t1) {
    let x = chanAt(panel.meas, t0);
    if (x == null) return [];
    const tau = Math.max(model[panel.tauKey], 1e-3);
    const T = model[panel.tKey];
    const outDt = 1 / RATE;
    const h = outDt / SUBSTEP;
    const pts = [{ t: t0, v: x }];
    let lastU = chanAt(panel.cmd, t0 - T);
    for (let t = t0; t < t1 - 1e-9; t += outDt) {
      for (let s = 0; s < SUBSTEP; s++) {
        const tt = t + s * h;
        const u = chanAt(panel.cmd, tt - T);
        if (u != null) lastU = u;
        const drive = (lastU != null) ? lastU : x;
        x += h * (-(x - drive) / tau);
      }
      pts.push({ t: Math.min(t + outDt, t1), v: x });
    }
    return pts;
  }

  // ------------------------------------------------------------------ 操作
  function setHint() {
    $("hint").textContent =
      "シーク時刻を起点に、指令を無駄時間 T だけ遅延させ時定数 τ の 1 次遅れで前方積算した" +
      "シミュレーション(赤)を、観測(実線)・指令(破線・灰)と重ねる。窓は起点から右いっぱい(予測窓 [s])。" +
      "つまみで T・τ を変えると赤線が即追従。地図は走行文脈(カーブ旋回など)の確認用。";
  }
  setHint();

  function setPlaying(p) {
    playing = p;
    playBtn.textContent = playing ? "⏸ 停止" : "▶ 再生";
  }
  playBtn.addEventListener("click", () => {
    if (!playing && curT >= T_MAX - 1e-9) curT = 0; // 末尾なら先頭から
    setPlaying(!playing);
  });
  $("speed").addEventListener("change", (e) => { speedMul = parseFloat(e.target.value); });

  const SEEK_MAX = 10000;
  function syncSeek() {
    const frac = T_MAX > 0 ? curT / T_MAX : 0;
    seekEl.value = Math.round(frac * SEEK_MAX);
    readoutEl.textContent = "t = " + curT.toFixed(2) + " / " + T_MAX.toFixed(1) + " s";
  }
  seekEl.addEventListener("input", () => {
    curT = (seekEl.valueAsNumber / SEEK_MAX) * T_MAX;
    syncSeek();
    markDirty();
  });

  $("follow").addEventListener("change", (e) => { cam.follow = e.target.checked; markDirty(); });
  $("vieww").addEventListener("input", (e) => {
    cam.targetViewW = e.target.valueAsNumber;
    $("viewwval").textContent = e.target.valueAsNumber + "m";
    markDirty();
  });
  $("plotwin").addEventListener("input", (e) => {
    plotWindowS = e.target.valueAsNumber;
    $("plotwinval").textContent = e.target.valueAsNumber + "s";
    markPlotStatic(); markDirty();
  });

  // つまみ (T, tau) のセットアップ。初期値は model_seed をスライダ範囲にクランプして反映。
  function clamp(v, lo, hi) { return Math.min(Math.max(v, lo), hi); }
  function setupKnob(sliderId, valId, modelKey) {
    const sl = $(sliderId), vEl = $(valId);
    const lo = parseFloat(sl.min), hi = parseFloat(sl.max);
    sl.value = clamp(model[modelKey], lo, hi);
    const show = () => { vEl.textContent = parseFloat(sl.value).toFixed(3) + "s"; };
    model[modelKey] = parseFloat(sl.value); show();
    sl.addEventListener("input", () => {
      model[modelKey] = sl.valueAsNumber;
      show();
      markDirty(); // y レンジは観測/指令依存で不変。sim のみ再計算 → geom 再計算不要。
    });
  }
  setupKnob("k_t_acc", "v_t_acc", "t_acc");
  setupKnob("k_tau_acc", "v_tau_acc", "tau_acc");
  setupKnob("k_t_steer", "v_t_steer", "t_steer");
  setupKnob("k_tau_steer", "v_tau_steer", "tau_steer");

  // ------------------------------------------------------------------ 地図描画
  function drawArrowhead(x, y, ang, size) {
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x - size * Math.cos(ang - 0.4), y - size * Math.sin(ang - 0.4));
    ctx.lineTo(x - size * Math.cos(ang + 0.4), y - size * Math.sin(ang + 0.4));
    ctx.closePath();
    ctx.fill();
  }

  function drawScaleBar(w, h, scale) {
    const meters = 100 / scale;
    const pow = Math.pow(10, Math.floor(Math.log10(meters)));
    const cand = [1, 2, 5, 10].map((c) => c * pow);
    const m = cand.reduce((a, b) => (Math.abs(b - meters) < Math.abs(a - meters) ? b : a));
    const px = m * scale, x0 = 12, y0 = h - 14;
    ctx.strokeStyle = "#333"; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x0, y0); ctx.lineTo(x0 + px, y0);
    ctx.moveTo(x0, y0 - 4); ctx.lineTo(x0, y0 + 4);
    ctx.moveTo(x0 + px, y0 - 4); ctx.lineTo(x0 + px, y0 + 4);
    ctx.stroke();
    ctx.fillStyle = "#333"; ctx.font = "11px monospace";
    ctx.fillText(m >= 1 ? m + " m" : m.toFixed(1) + " m", x0 + 4, y0 - 6);
  }

  // 地図 1 フレーム描画。カメラがまだ移動中なら true (継続描画が必要)。
  function drawMap() {
    const wrap = cv.parentElement;
    const w = wrap.clientWidth, h = wrap.clientHeight;
    if (w === 0 || h === 0) return false;
    const dpr = window.devicePixelRatio || 1;
    const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (cv.width !== pw || cv.height !== ph) { cv.width = pw; cv.height = ph; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const st = sampleAtT(curT);

    // カメラ: 追従 = 現在位置中心 (視野幅 slider が最小ズーム)、全体 = bbox フィット。
    let tx, ty, tvw;
    if (cam.follow) {
      tx = st.x; ty = st.y; tvw = cam.targetViewW;
    } else {
      tx = (BBOX[0] + BBOX[2]) / 2;
      ty = (BBOX[1] + BBOX[3]) / 2;
      tvw = Math.max(BBOX[2] - BBOX[0], (BBOX[3] - BBOX[1]) * (w / h)) * 1.05;
    }
    if (!cam.init) { cam.cx = tx; cam.cy = ty; cam.viewW = tvw; cam.init = true; }
    if (Math.hypot(tx - cam.cx, ty - cam.cy) > cam.viewW) { cam.cx = tx; cam.cy = ty; }
    else { cam.cx += (tx - cam.cx) * 0.15; cam.cy += (ty - cam.cy) * 0.15; }
    if (tvw > cam.viewW * 2 || tvw < cam.viewW / 2) cam.viewW = tvw;
    else cam.viewW += (tvw - cam.viewW) * 0.15;
    const scale = w / cam.viewW;
    const SX = (wx) => w / 2 + (wx - cam.cx) * scale;
    const SY = (wy) => h / 2 - (wy - cam.cy) * scale;

    // lanelet (薄灰)
    ctx.strokeStyle = "#c8c8c8"; ctx.lineWidth = 0.8;
    ctx.beginPath();
    for (const way of DATA.lanelets) {
      ctx.moveTo(SX(way[0][0]), SY(way[0][1]));
      for (let k = 1; k < way.length; k++) ctx.lineTo(SX(way[k][0]), SY(way[k][1]));
    }
    ctx.stroke();

    // 全軌跡 (薄く実機色)
    ctx.strokeStyle = run.color; ctx.globalAlpha = 0.3; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(SX(run.x[0]), SY(run.y[0]));
    for (let k = 1; k < run.n_valid; k++) ctx.lineTo(SX(run.x[k]), SY(run.y[k]));
    ctx.stroke();
    ctx.globalAlpha = 1;

    // 現在位置マーカー + 速度(進行方位)矢印
    const px = SX(st.x), py = SY(st.y);
    const lenM = Math.max(st.v * PREVIEW_SEC, st.ended ? 0 : 3);
    if (lenM * scale > 3) {
      const ex = SX(st.x + lenM * Math.cos(st.yaw)), ey = SY(st.y + lenM * Math.sin(st.yaw));
      ctx.strokeStyle = run.color; ctx.fillStyle = run.color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(ex, ey); ctx.stroke();
      drawArrowhead(ex, ey, Math.atan2(ey - py, ex - px), 7);
    }
    ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2);
    ctx.fillStyle = run.color; ctx.fill();
    ctx.lineWidth = 1.5; ctx.strokeStyle = "#fff"; ctx.stroke();

    drawScaleBar(w, h, scale);
    return (
      Math.hypot(tx - cam.cx, ty - cam.cy) > cam.viewW * 1e-3 ||
      Math.abs(tvw - cam.viewW) > cam.viewW * 2e-3
    );
  }

  // -------------------------------------------------------- 検証パネル描画
  let plotGeom = []; // [{ox,oy,cellW,cellH,px0,py0,plotW,plotH,panel,ymin,ymax,Y}]

  // 窓 [t0,t1] (左端=起点 curT)。右端は T_MAX を超えない。
  function plotWindow() {
    let t0 = curT;
    let t1 = Math.min(curT + plotWindowS, T_MAX);
    if (t1 - t0 < 1e-6) { t0 = Math.max(0, T_MAX - plotWindowS); t1 = T_MAX; }
    if (t1 - t0 < 1e-6) t1 = t0 + 1e-6;
    return [t0, t1];
  }

  // パネル矩形 + y オートスケール (観測+指令の全域。sim 発散時のはみ出しは clip)。構造変化時のみ。
  function computePlotGeom() {
    plotGeom = [];
    const w = plotsEl.clientWidth, h = plotsEl.clientHeight;
    if (w === 0 || h === 0) return;
    const dpr = window.devicePixelRatio || 1;
    const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (plotCv.width !== pw || plotCv.height !== ph) { plotCv.width = pw; plotCv.height = ph; }
    const outpad = 6, gapY = 10;
    const padL = 46, padR = 12, padT = 18, padB = 18;
    const cellW = w - outpad * 2;
    const cellH = (h - outpad * 2 - gapY * (PANELS.length - 1)) / PANELS.length;
    PANELS.forEach((panel, pi) => {
      const ox = outpad, oy = outpad + pi * (cellH + gapY);
      const px0 = ox + padL, py0 = oy + padT;
      const plotW = cellW - padL - padR, plotH = cellH - padT - padB;
      let ymin = Infinity, ymax = -Infinity;
      for (const key of [panel.meas, panel.cmd]) {
        const arr = run.ch && run.ch[key];
        if (!arr) continue;
        for (let i = 0; i < arr.length; i++) {
          const v = arr[i];
          if (v != null) { if (v < ymin) ymin = v; if (v > ymax) ymax = v; }
        }
      }
      if (!isFinite(ymin) || !isFinite(ymax)) { ymin = -1; ymax = 1; }
      if (ymax - ymin < 1e-6) { const c = (ymin + ymax) / 2; ymin = c - 1; ymax = c + 1; }
      const m = (ymax - ymin) * 0.08; ymin -= m; ymax += m;
      const Y = (v) => py0 + (1 - (v - ymin) / (ymax - ymin)) * plotH;
      plotGeom.push({ ox, oy, cellW, cellH, px0, py0, plotW, plotH, panel, ymin, ymax, Y });
    });
  }

  function drawPlots() {
    const w = plotsEl.clientWidth, h = plotsEl.clientHeight;
    if (w === 0 || h === 0 || !plotGeom.length) return;
    const dpr = window.devicePixelRatio || 1;
    const c = plotCtx;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);

    const [t0, t1] = plotWindow();
    const iLo = Math.max(0, Math.floor(t0 * RATE) - 1);
    const iHi = Math.min(N - 1, Math.ceil(t1 * RATE) + 1);

    for (const g of plotGeom) {
      const X = (t) => g.px0 + (t - t0) / (t1 - t0) * g.plotW;

      // 枠・ゼロ線
      c.strokeStyle = "#ddd"; c.lineWidth = 1;
      c.strokeRect(g.px0, g.py0, g.plotW, g.plotH);
      if (g.ymin < 0 && g.ymax > 0) {
        c.strokeStyle = "#eee"; c.beginPath();
        c.moveTo(g.px0, g.Y(0)); c.lineTo(g.px0 + g.plotW, g.Y(0)); c.stroke();
      }
      // y 目盛
      c.fillStyle = "#999"; c.font = "9px monospace";
      c.textAlign = "right"; c.textBaseline = "middle";
      const yt = (g.ymin < 0 && g.ymax > 0) ? [g.ymin, 0, g.ymax]
                                            : [g.ymin, (g.ymin + g.ymax) / 2, g.ymax];
      for (const v of yt) c.fillText(v.toFixed(Math.abs(v) >= 10 ? 0 : 2), g.px0 - 3, g.Y(v));
      // x 目盛 (起点 t0 / 中央 / t1)
      c.textAlign = "center"; c.textBaseline = "top";
      for (let k = 0; k <= 2; k++) {
        const tt = t0 + (t1 - t0) * k / 2;
        c.fillText(tt.toFixed(1) + (k === 2 ? "s" : ""), X(tt), g.py0 + g.plotH + 2);
      }
      // 見出し
      c.textAlign = "left"; c.textBaseline = "alphabetic";
      c.fillStyle = "#333"; c.font = "11px sans-serif";
      const T = model[g.panel.tKey], tau = model[g.panel.tauKey];
      c.fillText(g.panel.label + " [" + g.panel.unit + "]   T=" + T.toFixed(3) +
                 "s  τ=" + tau.toFixed(3) + "s", g.ox + 2, g.oy + 13);

      // パネル矩形でクリップ
      c.save();
      c.beginPath(); c.rect(g.px0, g.py0, g.plotW, g.plotH); c.clip();

      // 観測 (実線・実機色) / 指令 (破線・灰)。窓内 [iLo,iHi] のみ。null で break。
      const lines = [
        { key: g.panel.meas, color: run.color, dash: false, alpha: 1.0, lw: 1.6 },
        { key: g.panel.cmd, color: CMD_COLOR, dash: true, alpha: 0.9, lw: 1.2 },
      ];
      for (const sp of lines) {
        const arr = run.ch && run.ch[sp.key];
        if (!arr) continue;
        c.setLineDash(sp.dash ? [4, 3] : []);
        c.strokeStyle = sp.color; c.globalAlpha = sp.alpha; c.lineWidth = sp.lw;
        c.beginPath();
        let pen = false;
        for (let i = iLo; i <= iHi; i++) {
          const v = arr[i];
          if (v == null) { pen = false; continue; }
          const xx = X(i / RATE), yy = g.Y(v);
          if (!pen) { c.moveTo(xx, yy); pen = true; } else c.lineTo(xx, yy);
        }
        c.stroke();
      }
      c.setLineDash([]); c.globalAlpha = 1;

      // シミュレーション (赤・太め): 起点 t0 から前方積算。
      const sim = simulate(g.panel, t0, t1);
      if (sim.length > 1) {
        c.strokeStyle = SIM_COLOR; c.lineWidth = 2.0; c.globalAlpha = 1;
        c.beginPath();
        c.moveTo(X(sim[0].t), g.Y(sim[0].v));
        for (let i = 1; i < sim.length; i++) c.lineTo(X(sim[i].t), g.Y(sim[i].v));
        c.stroke();
      }

      // 起点(左端)カーソル + 観測初期値ドット
      c.strokeStyle = "#999"; c.lineWidth = 1; c.setLineDash([3, 3]);
      c.beginPath(); c.moveTo(X(t0), g.py0); c.lineTo(X(t0), g.py0 + g.plotH); c.stroke();
      c.setLineDash([]);
      const v0 = chanAt(g.panel.meas, t0);
      if (v0 != null) {
        c.fillStyle = SIM_COLOR; c.beginPath();
        c.arc(X(t0), g.Y(v0), 4, 0, Math.PI * 2); c.fill();
        c.strokeStyle = "#fff"; c.lineWidth = 1; c.stroke();
      }

      c.restore();
    }
  }

  // --------------------------------------------------------------- 再生ループ
  let dirty = true;
  let plotStaticDirty = true;
  const markDirty = () => { dirty = true; };
  const markPlotStatic = () => { plotStaticDirty = true; dirty = true; };
  const _ro = new ResizeObserver(markPlotStatic);
  _ro.observe(cv.parentElement);
  _ro.observe(plotsEl);

  let lastTs = null;
  function tick(ts) {
    if (lastTs !== null && playing) {
      const dt = Math.min((ts - lastTs) / 1000, 0.2) * speedMul;
      curT += dt;
      if (curT >= T_MAX) { curT = T_MAX; setPlaying(false); }
      syncSeek();
      dirty = true;
    }
    lastTs = ts;
    if (dirty) {
      const cont = drawMap();
      if (plotStaticDirty) { computePlotGeom(); plotStaticDirty = false; }
      drawPlots();
      dirty = cont;
    }
    requestAnimationFrame(tick);
  }

  syncSeek();
  requestAnimationFrame(tick);
})();
</script>
</body>
</html>
"""
