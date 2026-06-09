"""
縦横モデル検証ビューア (longitudinal/lateral equations-of-motion replay) の生成.

実機 rosbag のみを対象に、車両の運動方程式（アクチュエータ1次遅れ + 運動学的自転車モデル）が
十分かを対話的に検証する自己完結 HTML (canvas + インライン JS、plotly 非依存・オフライン動作)。

検証する運動方程式（起点 t0 = シーク時刻から前方積算）:
  縦  : a' = -(a - a_target) / tau(v)  ,  a_target = a_cmd(t-T) + poly(v)  ,  v' = a
        加減速で特性が異なるため throttle(a_cmd>=0) と brake(a_cmd<0) で T・tau を分離。
        速度帯依存も tau(v) = tau0 + slope*v (v は観測 lon_vel) で表現。
        定常オフセット (転がり抵抗・勾配・空気抵抗) を多項式 poly(v)=p0+p1*v+p2*v^2 で補正
        (各次 ON/OFF 切替・有効項のみ最小二乗最適化対象)。
  横  : delta' = -(delta - delta_cmd(t-T_d)) / tau_d        (delta は rad で積分)
        omega = v*tan(delta + beta) / (L + k_us*v^2) ,  theta' = omega
        x' = v*cos(theta) , y' = v*sin(theta) ,  a_y = v*omega

指令値からこのモデルで積算したシミュレーションを観測値・指令値と重ね描きし、単一時定数 +
キネマティック自転車モデルで応答・軌跡が再現できるかを目で確認する。つまみで無駄時間 T・
時定数 tau・アンダーステア係数 k_us・ステアバイアス beta を変えながら当てはまりを見る。
地図に観測軌跡とシミュレーション軌跡を重ね、カーブ旋回など走行文脈と応答を対応づけられる。

横チェーンの速度 v は観測 lon_vel を使う（縦の誤差を横テストに混入させないアイソレーション。
手書きメモ「速度は rosbag から」に対応）。位置・ω は既定でシミュレーション δ から積算するが、
「ステア源」トグルで観測 δ に切り替えられる（メモの「位置のみ simulation」モード）。

仕様:
  - シミュレーション起点 = プロット時間窓の左端 (シーク時刻から窓いっぱい先まで前方ロールアウト)。
  - 対象は実機 (BASELINE_LABEL) のみ。"sim は介在しない" 実機解析セクションの趣旨に沿う。
  - つまみ初期値は simulator_model.param.yaml の設定値をシード (load_sim_params のフォールバック付)。

ペイロード構築は _playback_viewer.build_playback_payload を再利用する (地図 lanelet・実機軌跡・
加速度/速度/ステア/ヨーレートの指令&観測チャンネル `ch` を生成済み)。
"""

from __future__ import annotations

import json
import warnings
from pathlib import Path

from ._playback_viewer import (
    BASELINE_LABEL,
    PLAYBACK_WHEELBASE_M,
    build_playback_payload,
)

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

    # つまみ初期値。tau/T [s], slope [s/(m/s)], steer_bias [rad], k_us [s^2/m]。
    # load_sim_params はフォールバック付きで常に dict を返す。縦は加減速で別特性のため
    # throttle=acc_*, brake=brake_* をシード（実機モデルも acc/brake で別 time_constant・delay）。
    # tau_acc_slope は速度依存の初期 0（つまみで調整）。k_us は yaml 未定義のことが多く
    # その場合 0.0 (純キネマティック) をシード（best_normal の 0.018 等へつまみで調整可能）。
    payload["model_seed"] = {
        "tau_acc_thr": float(sim_params["acc_time_constant"]),
        "t_acc_thr": float(sim_params["acc_time_delay"]),
        "tau_acc_brk": float(sim_params.get("brake_time_constant", sim_params["acc_time_constant"])),
        "t_acc_brk": float(sim_params.get("brake_delay", sim_params["acc_time_delay"])),
        "tau_acc_slope": 0.0,
        "poly0": 0.0,  # 縦 定常補正 多項式係数 (0/1/2 次, 既定 OFF・0)。a_target に加算。
        "poly1": 0.0,
        "poly2": 0.0,
        "tau_steer": float(sim_params["steer_time_constant"]),
        "t_steer": float(sim_params["steer_time_delay"]),
        "steer_bias": float(sim_params.get("steer_bias", 0.0)),
        "k_us": float(sim_params.get("k_us", 0.0)),
    }
    # 自転車モデルの wheelbase L [m]（JS でハードコード二重定義しないようペイロードで渡す）。
    payload["wheelbase"] = float(PLAYBACK_WHEELBASE_M)

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
<title>縦横モデル検証ビューア</title>
<style>
  html, body { margin: 0; height: 100%; font-family: "Hiragino Sans", "Noto Sans CJK JP", sans-serif; font-size: 13px; background: #fff; color: #333; }
  #app { display: flex; flex-direction: column; height: 100%; }
  #controls { display: flex; flex-wrap: wrap; gap: 4px 16px; align-items: center; padding: 6px 10px; border-bottom: 1px solid #ddd; background: #fafafa; }
  #controls .grp { display: flex; gap: 6px; align-items: center; white-space: nowrap; }
  #knobs { display: flex; flex-wrap: wrap; gap: 4px 16px; align-items: center; padding: 5px 10px; border-bottom: 1px solid #ddd; background: #f4f6fa; }
  #knobs .kg { display: flex; gap: 5px; align-items: center; white-space: nowrap; }
  #knobs .kg input[type=range] { width: 96px; }
  #knobs .kval { font-family: monospace; font-size: 12px; min-width: 50px; text-align: right; }
  #knobs .ktitle { font-weight: 600; color: #2b4a8b; }
  #seekrow { display: flex; gap: 10px; align-items: center; padding: 4px 10px; border-bottom: 1px solid #ddd; background: #fafafa; }
  #seek { flex: 1; }
  #readout { min-width: 150px; font-family: monospace; font-size: 12px; text-align: right; }
  #hint { font-size: 11px; color: #777; padding: 3px 10px 6px; background: #fafafa; border-bottom: 1px solid #ddd; }
  #main { flex: 1; display: flex; min-height: 0; }
  #canvaswrap { flex: 1; position: relative; min-width: 0; background: #fdfdfd; }
  #cv { position: absolute; inset: 0; width: 100%; height: 100%; }
  /* 運動方程式オーバーレイ（地図右上・半透明・クリック透過） */
  #eqpanel { position: absolute; top: 8px; right: 8px; max-width: 460px; background: rgba(255,255,255,0.92);
             border: 1px solid #ccd; border-radius: 5px; padding: 7px 10px; font-size: 11px; line-height: 1.5;
             font-family: "SFMono-Regular", Menlo, Consolas, monospace; color: #222; pointer-events: none;
             box-shadow: 0 1px 4px rgba(0,0,0,0.12); }
  #eqpanel b { color: #2b4a8b; }
  #eqpanel .vals { color: #b00; }
  #eqpanel .note { color: #777; font-size: 10px; }
  #plots { flex: none; height: 540px; position: relative; border-top: 1px solid #ddd; background: #fff; }
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
    <span class="grp">
      ステア源
      <select id="steersrc">
        <option value="sim" selected>シミュレーション δ</option>
        <option value="obs">観測 δ</option>
      </select>
    </span>
    <span class="grp">
      最小二乗最適化:
      <button id="opt_all">全体</button>
      <button id="opt_lon">縦</button>
      <button id="opt_steer">横</button>
      <button id="opt_bike">自転車</button>
      <span id="optstatus" style="font-family:monospace;font-size:11px;color:#2b4a8b"></span>
    </span>
  </div>
  <div id="knobs">
    <span class="kg"><span class="ktitle">縦throttle</span></span>
    <span class="kg">T <input type="range" id="k_t_acc_thr" min="0" max="0.5" step="0.005" value="0.1"><span class="kval" id="v_t_acc_thr"></span></span>
    <span class="kg">τ <input type="range" id="k_tau_acc_thr" min="0.02" max="1.5" step="0.005" value="0.26"><span class="kval" id="v_tau_acc_thr"></span></span>
    <span class="kg" style="margin-left:6px"><span class="ktitle">縦brake</span></span>
    <span class="kg">T <input type="range" id="k_t_acc_brk" min="0" max="0.5" step="0.005" value="0.07"><span class="kval" id="v_t_acc_brk"></span></span>
    <span class="kg">τ <input type="range" id="k_tau_acc_brk" min="0.02" max="1.5" step="0.005" value="0.15"><span class="kval" id="v_tau_acc_brk"></span></span>
    <span class="kg">τ速度傾き <input type="range" id="k_tau_slope" min="-0.05" max="0.05" step="0.001" value="0"><span class="kval" id="v_tau_slope"></span></span>
    <span class="kg" style="margin-left:6px"><span class="ktitle">縦補正(多項式)</span></span>
    <span class="kg"><label><input type="checkbox" id="on_poly0">p₀</label> <input type="range" id="k_poly0" min="-1" max="1" step="0.01" value="0" disabled><span class="kval" id="v_poly0"></span></span>
    <span class="kg"><label><input type="checkbox" id="on_poly1">p₁·v</label> <input type="range" id="k_poly1" min="-0.1" max="0.1" step="0.001" value="0" disabled><span class="kval" id="v_poly1"></span></span>
    <span class="kg"><label><input type="checkbox" id="on_poly2">p₂·v²</label> <input type="range" id="k_poly2" min="-0.01" max="0.01" step="0.0001" value="0" disabled><span class="kval" id="v_poly2"></span></span>
    <span class="kg" style="margin-left:6px"><span class="ktitle">横(ステア)</span></span>
    <span class="kg">T <input type="range" id="k_t_steer" min="0" max="0.5" step="0.005" value="0.03"><span class="kval" id="v_t_steer"></span></span>
    <span class="kg">τ <input type="range" id="k_tau_steer" min="0.02" max="1.5" step="0.005" value="0.50"><span class="kval" id="v_tau_steer"></span></span>
    <span class="kg" style="margin-left:6px"><span class="ktitle">自転車</span></span>
    <span class="kg">k_us <input type="range" id="k_kus" min="0" max="0.05" step="0.001" value="0"><span class="kval" id="v_kus"></span></span>
    <span class="kg">β <input type="range" id="k_bias" min="-2" max="2" step="0.01" value="0"><span class="kval" id="v_bias"></span></span>
  </div>
  <div id="seekrow">
    <input type="range" id="seek" min="0" max="10000" value="0">
    <span id="readout"></span>
  </div>
  <div id="hint"></div>
  <div id="main">
    <div id="canvaswrap"><canvas id="cv"></canvas><div id="eqpanel"></div></div>
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
  const L = DATA.wheelbase;        // wheelbase [m]
  const DEG = Math.PI / 180, RAD2DEG = 180 / Math.PI;
  const SIM_COLOR = "#d62728";     // シミュレーション線 (赤)
  const CMD_COLOR = "#888";        // 指令線 (灰・破線)
  const PREVIEW_SEC = 1.5;         // 速度矢印 = PREVIEW_SEC 秒後の到達距離 [m]
  const SUBSTEP = 10;              // Euler sub-step 数/グリッドセル (tau が小さくても発振させない)

  // 検証パネル定義（2列グリッド）。meas/cmd は run.ch のキー、sim は rollout 出力のキー。
  const PANELS = [
    { label: "縦加速度",   unit: "m/s²",  meas: "accel",   cmd: "cmd_accel", sim: "a" },
    { label: "速度",       unit: "m/s",   meas: "lon_vel", cmd: "cmd_vel",   sim: "v" },
    { label: "ステア角",   unit: "deg",   meas: "steer",   cmd: "cmd_steer", sim: "deltaDeg" },
    { label: "ヨーレート", unit: "rad/s", meas: "wz",      cmd: "cmd_wz",    sim: "omega" },
    { label: "横G(横加速度)", unit: "m/s²", meas: "ay",     cmd: null,        sim: "ay" },
  ];
  const PLOT_COLS = 2;
  const PLOT_ROWS = Math.ceil(PANELS.length / PLOT_COLS);

  // ----------------------------------------------------------------- state
  let playing = false;
  let speedMul = 1;
  let plotWindowS = 8;   // 予測窓幅 [s]。窓 = [curT, curT + plotWindowS] (左端=起点)。
  let curT = 0;          // 現在時刻 (= シミュレーション起点) [s]
  let steerSource = "sim"; // 自転車モデルへ渡す δ の源: "sim"(シミュレーション) | "obs"(観測)
  const cam = { cx: 0, cy: 0, viewW: 100, targetViewW: 100, follow: true, init: false };
  // model: tau/T [s], tau_acc_slope [s/(m/s)], steer_bias [deg](UI 単位; rollout で rad 変換), k_us [s^2/m]
  // 縦は throttle/brake で tau・T を分離。tau_eff = tau0 + slope*v (v は観測)。
  const model = {
    tau_acc_thr: DATA.model_seed.tau_acc_thr, t_acc_thr: DATA.model_seed.t_acc_thr,
    tau_acc_brk: DATA.model_seed.tau_acc_brk, t_acc_brk: DATA.model_seed.t_acc_brk,
    tau_acc_slope: DATA.model_seed.tau_acc_slope,
    poly0: DATA.model_seed.poly0, poly1: DATA.model_seed.poly1, poly2: DATA.model_seed.poly2,
    polyOn0: false, polyOn1: false, polyOn2: false, // 多項式補正の各次 ON/OFF (既定 OFF)
    tau_steer: DATA.model_seed.tau_steer, t_steer: DATA.model_seed.t_steer,
    k_us: DATA.model_seed.k_us,
    steer_bias: DATA.model_seed.steer_bias * RAD2DEG, // rad→deg (β つまみは度表示)
  };
  // 縦 定常補正 多項式 poly(v)=p0+p1·v+p2·v²（有効な次のみ加算）。a_target に足す。
  function polyAccel(v) {
    let p = 0;
    if (model.polyOn0) p += model.poly0;
    if (model.polyOn1) p += model.poly1 * v;
    if (model.polyOn2) p += model.poly2 * v * v;
    return p;
  }

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
    const Lp = (a) => a[i] + (a[j] - a[i]) * f;
    const tEnd = (run.n_valid - 1) / RATE;
    const ended = t > tEnd + 1e-9;
    return { x: Lp(run.x), y: Lp(run.y), yaw: Lp(run.yaw), v: ended ? 0 : Lp(run.v), ended };
  }

  // チャンネル値を時刻 t [s] で全域線形補間。範囲外/null は null。
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

  // 運動方程式を起点 t0 の観測値から窓右端 t1 まで前方積算する（sub-step Euler で安定化）。
  //   縦: a' = -(a - a_cmd(t-T_a))/τ_a ,  v' = a
  //   横: δ' = -(δ - δ_cmd(t-T_δ))/τ_δ (rad),  ω = v·tan(δ_src+β)/(L+k_us·v²),
  //       θ' = ω,  x' = v·cosθ,  y' = v·sinθ,  a_y = v·ω
  // 横チェーンの v は観測 lon_vel を使用（縦誤差を分離）。δ_src は steerSource で sim/観測 を切替。
  // 戻り値: {t,a,v,deltaDeg,omega,ay,x,y}(各配列, RATE 解像度) / IC 欠損なら null。
  function rollout(t0, t1) {
    let a = chanAt("accel", t0);
    let v = chanAt("lon_vel", t0);
    const dDeg0 = chanAt("steer", t0);
    const st0 = sampleAtT(t0);
    if (a == null || v == null || dDeg0 == null) return null;
    let delta = dDeg0 * DEG;        // rad
    let yaw = st0.yaw, x = st0.x, y = st0.y;
    // 縦: throttle/brake で tau・T を分離。tau_eff = tau0 + slope*v (v は観測, 下限 0.02s)。
    const tauThr = model.tau_acc_thr, tauBrk = model.tau_acc_brk, tauSlope = model.tau_acc_slope;
    const TaThr = model.t_acc_thr, TaBrk = model.t_acc_brk;
    const tauD = Math.max(model.tau_steer, 1e-3);
    const Td = model.t_steer;
    const beta = model.steer_bias * DEG;   // deg→rad
    const kus = model.k_us;
    const simSteer = (steerSource === "sim");
    const outDt = 1 / RATE, h = outDt / SUBSTEP;

    let lastDrive = chanAt("cmd_accel", t0); if (lastDrive == null) lastDrive = a; // 指令ホールド用
    let lastUd = chanAt("cmd_steer", t0 - Td); // deg
    const out = { t: [], a: [], v: [], deltaDeg: [], omega: [], ay: [], x: [], y: [] };

    const omegaAt = (tt, dRad) => {
      const vl = chanAt("lon_vel", tt);
      const vv = (vl != null) ? vl : v;
      const dsrc = simSteer ? dRad
        : ((chanAt("steer", tt) != null) ? chanAt("steer", tt) * DEG : dRad);
      return vv * Math.tan(dsrc + beta) / (L + kus * vv * vv);
    };
    const record = (tt) => {
      const om = omegaAt(tt, delta);
      const vl = chanAt("lon_vel", tt);
      const vv = (vl != null) ? vl : v;
      out.t.push(tt); out.a.push(a); out.v.push(v);
      out.deltaDeg.push(delta * RAD2DEG);
      out.omega.push(om); out.ay.push(vv * om);
      out.x.push(x); out.y.push(y);
    };

    record(t0);
    for (let t = t0; t < t1 - 1e-9; t += outDt) {
      for (let s = 0; s < SUBSTEP; s++) {
        const tt = t + s * h;
        const vl = chanAt("lon_vel", tt);
        const vv = (vl != null) ? vl : v;   // 観測速度 (tau(v)・横チェーンで共用)
        // 縦: throttle(a_cmd>=0)/brake(a_cmd<0) を判定し、該当 delay の指令 u と tau0 を採用。
        // throttle 側の遅延指令の符号で判定 (差は僅少、null は brake 側→ホールドで fallback)。
        const uThr = chanAt("cmd_accel", tt - TaThr);
        const uBrk = chanAt("cmd_accel", tt - TaBrk);
        let throttle;
        if (uThr != null) throttle = (uThr >= 0);
        else if (uBrk != null) throttle = (uBrk >= 0);
        else throttle = (lastDrive >= 0);
        let u = throttle ? uThr : uBrk;
        if (u == null) u = lastDrive;
        lastDrive = u;
        const tau0 = throttle ? tauThr : tauBrk;
        const tauA = Math.max(tau0 + tauSlope * vv, 0.02); // tau(v), 下限 0.02s
        a += h * (-(a - (u + polyAccel(vv))) / tauA); // a_target = a_cmd + poly(v)
        v += h * a;
        // ステア(rad)
        const ud = chanAt("cmd_steer", tt - Td); if (ud != null) lastUd = ud;
        const driveD = (lastUd != null) ? lastUd * DEG : delta;
        delta += h * (-(delta - driveD) / tauD);
        // 横運動学（v は観測 vv, δ_src は sim/観測）
        const om = omegaAt(tt, delta);
        yaw += h * om;
        x += h * vv * Math.cos(yaw);
        y += h * vv * Math.sin(yaw);
      }
      record(Math.min(t + outDt, t1));
    }
    return out;
  }

  // ------------------------------------------------------------------ 操作
  function setHint() {
    $("hint").textContent =
      "シーク時刻を起点に運動方程式で前方積算したシミュレーション(赤)を観測(実線)・指令(破線灰)と重ねる。" +
      "縦 a→v、横 δ→ω→θ→位置（横の v は観測値を使用）。地図の赤線=シミュレーション軌跡。" +
      "つまみ(T・τ・k_us・β)で即追従。位置が乖離してもモデル不良と即断せず予測窓を縮めて各段を切り分ける。";
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
  $("steersrc").addEventListener("change", (e) => { steerSource = e.target.value; updateEquations(); markDirty(); });

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
    markDirty(); // y レンジは全域固定なので geom 再計算不要
  });

  // つまみセットアップ。初期値は model をスライダ範囲にクランプして反映。fmt は表示整形。
  // knobReg: modelKey -> {sl,vEl,fmt}（最適化後に slider/表示を model 値へ同期するため）。
  function clamp(v, lo, hi) { return Math.min(Math.max(v, lo), hi); }
  const knobReg = {};
  function setupKnob(sliderId, valId, modelKey, fmt) {
    const sl = $(sliderId), vEl = $(valId);
    const lo = parseFloat(sl.min), hi = parseFloat(sl.max);
    sl.value = clamp(model[modelKey], lo, hi);
    model[modelKey] = parseFloat(sl.value);
    knobReg[modelKey] = { sl, vEl, fmt };
    const show = () => { vEl.textContent = fmt(parseFloat(sl.value)); };
    show();
    sl.addEventListener("input", () => {
      model[modelKey] = sl.valueAsNumber;
      show(); updateEquations();
      markDirty(); // sim のみ再計算 → geom 再計算不要
    });
  }
  const fmtS = (v) => v.toFixed(3) + "s";
  const fmtKus = (v) => v.toFixed(3);
  const fmtDeg = (v) => v.toFixed(2) + "°";
  const fmtSlope = (v) => v.toFixed(3);
  setupKnob("k_t_acc_thr", "v_t_acc_thr", "t_acc_thr", fmtS);
  setupKnob("k_tau_acc_thr", "v_tau_acc_thr", "tau_acc_thr", fmtS);
  setupKnob("k_t_acc_brk", "v_t_acc_brk", "t_acc_brk", fmtS);
  setupKnob("k_tau_acc_brk", "v_tau_acc_brk", "tau_acc_brk", fmtS);
  setupKnob("k_tau_slope", "v_tau_slope", "tau_acc_slope", fmtSlope);
  setupKnob("k_t_steer", "v_t_steer", "t_steer", fmtS);
  setupKnob("k_tau_steer", "v_tau_steer", "tau_steer", fmtS);
  setupKnob("k_kus", "v_kus", "k_us", fmtKus);
  setupKnob("k_bias", "v_bias", "steer_bias", fmtDeg);
  setupKnob("k_poly0", "v_poly0", "poly0", (v) => v.toFixed(3));
  setupKnob("k_poly1", "v_poly1", "poly1", (v) => v.toFixed(4));
  setupKnob("k_poly2", "v_poly2", "poly2", (v) => v.toFixed(5));

  // 多項式補正の各次 ON/OFF。OFF 時は係数を model から除外（=0扱い）し最適化対象からも外す。
  function setupPolyToggle(cbId, sliderId, onKey) {
    const cb = $(cbId), sl = $(sliderId);
    cb.addEventListener("change", () => {
      model[onKey] = cb.checked;
      sl.disabled = !cb.checked;
      updateEquations(); markDirty();
    });
  }
  setupPolyToggle("on_poly0", "k_poly0", "polyOn0");
  setupPolyToggle("on_poly1", "k_poly1", "polyOn1");
  setupPolyToggle("on_poly2", "k_poly2", "polyOn2");

  // 運動方程式オーバーレイ（#1 deliverable）。つまみ/トグル変更でライブ更新。
  function updateEquations() {
    const m = model;
    const srcLabel = steerSource === "sim" ? "シミュレーション δ" : "観測 δ";
    let poly = "";
    if (m.polyOn0) poly += " + " + m.poly0.toFixed(3);
    if (m.polyOn1) poly += " + " + m.poly1.toFixed(4) + "·v";
    if (m.polyOn2) poly += " + " + m.poly2.toFixed(5) + "·v²";
    $("eqpanel").innerHTML =
      "<b>運動方程式</b>（起点=シーク時刻から前方積算）<br>" +
      "縦&nbsp; ȧ = −(a − a_target)/τ(v) ,&nbsp; a_target = a_cmd(t−T)" + poly + " ,&nbsp; v̇ = a<br>" +
      "&nbsp;&nbsp;&nbsp; <span class='note'>[throttle/brake で T・τ 分離, τ(v)=τ₀+slope·v, 多項式補正は ON の次のみ]</span><br>" +
      "横&nbsp; δ̇ = −(δ − δ_cmd(t−T_δ))/τ_δ<br>" +
      "&nbsp;&nbsp;&nbsp; ω = v·tan(δ+β)/(L + k_us·v²) ,&nbsp; θ̇ = ω<br>" +
      "&nbsp;&nbsp;&nbsp; ẋ = v·cosθ , ẏ = v·sinθ ,&nbsp; a_y = v·ω<br>" +
      "<span class='vals'>縦throttle T=" + m.t_acc_thr.toFixed(3) + "s τ₀=" + m.tau_acc_thr.toFixed(3) + "s&nbsp; " +
      "brake T=" + m.t_acc_brk.toFixed(3) + "s τ₀=" + m.tau_acc_brk.toFixed(3) + "s&nbsp; slope=" + m.tau_acc_slope.toFixed(3) + "<br>" +
      "横 T_δ=" + m.t_steer.toFixed(3) + "s τ_δ=" + m.tau_steer.toFixed(3) + "s&nbsp; " +
      "k_us=" + m.k_us.toFixed(3) + " β=" + m.steer_bias.toFixed(2) + "° L=" + L.toFixed(3) + "m</span><br>" +
      "<span class='note'>※ ω・位置・τ(v) の v は観測値を使用（縦誤差を分離）／ステア源: " + srcLabel + "</span>";
  }
  updateEquations();

  // -------------------------------------------------- 最小二乗最適化（全区間・出力誤差）
  // 各サブシステムを独立にフィット（運動方程式が階層的に分離できるため）:
  //   縦a: 1次遅れ(throttle/brake分離・τ(v)) を観測 accel に, 横δ: 1次遅れを観測 steer に,
  //   自転車ω: v·tan(δ+β)/(L+k_us·v²)(代数式・観測δ/v) を観測 wz に当て込む。
  // a・δ は安定フィルタ, ω は代数式なので全区間積分してもドリフトせず, 出力誤差最小化が
  // 表示中の赤線一致と一致する（速度・位置は積分でドリフトするため目的量にしない）。
  // 最適化は coordinate descent + golden-section（つまみ範囲内・無駄時間 T も対象）。
  function goldenMin(f, a, b, iters) {
    const gr = (Math.sqrt(5) - 1) / 2;
    let c = b - gr * (b - a), d = a + gr * (b - a);
    let fc = f(c), fd = f(d);
    for (let i = 0; i < iters; i++) {
      if (fc < fd) { b = d; d = c; fd = fc; c = b - gr * (b - a); fc = f(c); }
      else { a = c; c = d; fc = fd; d = a + gr * (b - a); fd = f(d); }
    }
    return (a + b) / 2;
  }

  // 縦 accel を全区間で前方積算（throttle/brake 分離・τ(v)）。戻り: Float64Array(N) (NaN=無効)。
  function simAccelSeries() {
    let i0 = 0; while (i0 < N && run.ch.accel[i0] == null) i0++;
    const out = new Float64Array(N).fill(NaN);
    if (i0 >= N) return out;
    let a = run.ch.accel[i0];
    const tauThr = model.tau_acc_thr, tauBrk = model.tau_acc_brk, slope = model.tau_acc_slope;
    const Tthr = model.t_acc_thr, Tbrk = model.t_acc_brk;
    const outDt = 1 / RATE, h = outDt / SUBSTEP;
    let lastDrive = chanAt("cmd_accel", i0 / RATE); if (lastDrive == null) lastDrive = a;
    out[i0] = a;
    for (let i = i0; i < N - 1; i++) {
      const t = i / RATE;
      for (let s = 0; s < SUBSTEP; s++) {
        const tt = t + s * h;
        const vl = chanAt("lon_vel", tt); const vv = (vl != null) ? vl : 0;
        const uThr = chanAt("cmd_accel", tt - Tthr), uBrk = chanAt("cmd_accel", tt - Tbrk);
        let throttle;
        if (uThr != null) throttle = (uThr >= 0);
        else if (uBrk != null) throttle = (uBrk >= 0);
        else throttle = (lastDrive >= 0);
        let u = throttle ? uThr : uBrk;
        if (u == null) u = lastDrive;
        lastDrive = u;
        const tau0 = throttle ? tauThr : tauBrk;
        const tau = Math.max(tau0 + slope * vv, 0.02);
        a += h * (-(a - (u + polyAccel(vv))) / tau); // a_target = a_cmd + poly(v)
      }
      out[i + 1] = a;
    }
    return out;
  }
  // 横 δ を全区間で前方積算（deg）。戻り: Float64Array(N)。
  function simSteerSeries() {
    let i0 = 0; while (i0 < N && run.ch.steer[i0] == null) i0++;
    const out = new Float64Array(N).fill(NaN);
    if (i0 >= N) return out;
    let dDeg = run.ch.steer[i0];
    const tau = Math.max(model.tau_steer, 1e-3), Td = model.t_steer;
    const outDt = 1 / RATE, h = outDt / SUBSTEP;
    let lastU = chanAt("cmd_steer", i0 / RATE); if (lastU == null) lastU = dDeg;
    out[i0] = dDeg;
    for (let i = i0; i < N - 1; i++) {
      const t = i / RATE;
      for (let s = 0; s < SUBSTEP; s++) {
        const tt = t + s * h;
        const u = chanAt("cmd_steer", tt - Td); if (u != null) lastU = u;
        const drive = (lastU != null) ? lastU : dDeg;
        dDeg += h * (-(dDeg - drive) / tau);
      }
      out[i + 1] = dDeg;
    }
    return out;
  }
  // 平均二乗誤差（sim 系列 vs 観測チャンネル）。
  function mseSeries(sim, obsKey) {
    const arr = run.ch[obsKey]; let se = 0, n = 0;
    for (let i = 0; i < N; i++) {
      const o = arr[i], s = sim[i];
      if (o != null && isFinite(s)) { se += (s - o) * (s - o); n++; }
    }
    return n ? se / n : 1e9;
  }
  const residAccel = () => mseSeries(simAccelSeries(), "accel");
  const residSteer = () => mseSeries(simSteerSeries(), "steer");
  // 自転車 ω=v·tan(δ_obs+β)/(L+k_us·v²) を観測 wz に当て込む（代数式・積分なし）。
  function residBicycle() {
    const beta = model.steer_bias * DEG, kus = model.k_us;
    const wzA = run.ch.wz, dA = run.ch.steer, vA = run.ch.lon_vel;
    let se = 0, n = 0;
    for (let i = 0; i < N; i++) {
      const wz = wzA[i], dd = dA[i], vv = vA[i];
      if (wz == null || dd == null || vv == null) continue;
      const om = vv * Math.tan(dd * DEG + beta) / (L + kus * vv * vv);
      se += (om - wz) * (om - wz); n++;
    }
    return n ? se / n : 1e9;
  }

  function syncKnob(k) {
    const r = knobReg[k]; if (!r) return;
    const lo = parseFloat(r.sl.min), hi = parseFloat(r.sl.max);
    r.sl.value = clamp(model[k], lo, hi);
    model[k] = parseFloat(r.sl.value);
    r.vEl.textContent = r.fmt(model[k]);
  }
  // coordinate descent + golden-section（各パラメータをつまみ範囲内で順に最小化、数 sweep）。
  function optimizeSubset(keys, resid) {
    const before = Math.sqrt(resid());
    for (let sweep = 0; sweep < 4; sweep++) {
      for (const k of keys) {
        const r = knobReg[k]; if (!r) continue;
        const lo = parseFloat(r.sl.min), hi = parseFloat(r.sl.max);
        model[k] = goldenMin((val) => { model[k] = val; return resid(); }, lo, hi, 22);
      }
    }
    const after = Math.sqrt(resid());
    for (const k of keys) syncKnob(k);
    updateEquations(); markDirty();
    return { before, after };
  }
  // 縦の最適化キー: 基本(T/τ/slope) + 有効な多項式補正の各次のみ。
  function lonKeys() {
    const k = ["t_acc_thr", "tau_acc_thr", "t_acc_brk", "tau_acc_brk", "tau_acc_slope"];
    if (model.polyOn0) k.push("poly0");
    if (model.polyOn1) k.push("poly1");
    if (model.polyOn2) k.push("poly2");
    return k;
  }
  const STEER_KEYS = ["t_steer", "tau_steer"];
  const BIKE_KEYS = ["k_us", "steer_bias"];
  const showOpt = (name, r, unit) => {
    $("optstatus").textContent = " " + name + " RMSE " + r.before.toFixed(3) + "→" + r.after.toFixed(3) + " " + unit;
  };
  $("opt_lon").addEventListener("click", () => showOpt("縦a", optimizeSubset(lonKeys(), residAccel), "m/s²"));
  $("opt_steer").addEventListener("click", () => showOpt("横δ", optimizeSubset(STEER_KEYS, residSteer), "deg"));
  $("opt_bike").addEventListener("click", () => showOpt("ω", optimizeSubset(BIKE_KEYS, residBicycle), "rad/s"));
  $("opt_all").addEventListener("click", () => {
    const a = optimizeSubset(lonKeys(), residAccel);
    const s = optimizeSubset(STEER_KEYS, residSteer);
    const b = optimizeSubset(BIKE_KEYS, residBicycle);
    $("optstatus").textContent = " 縦a " + a.before.toFixed(3) + "→" + a.after.toFixed(3) +
      " / 横δ " + s.before.toFixed(2) + "→" + s.after.toFixed(2) +
      " / ω " + b.before.toFixed(3) + "→" + b.after.toFixed(3);
  });

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

  // 地図 1 フレーム描画。sim = rollout 結果 (null 可)。カメラ移動中なら true。
  function drawMap(sim) {
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

    // シミュレーション軌跡（赤・起点 t0 から窓いっぱい）
    if (sim && sim.x.length > 1) {
      ctx.strokeStyle = SIM_COLOR; ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(SX(sim.x[0]), SY(sim.y[0]));
      for (let k = 1; k < sim.x.length; k++) ctx.lineTo(SX(sim.x[k]), SY(sim.y[k]));
      ctx.stroke();
    }

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

  // パネル矩形 + y オートスケール (観測+指令の全域)。2列グリッド。構造変化時のみ。
  function computePlotGeom() {
    plotGeom = [];
    const w = plotsEl.clientWidth, h = plotsEl.clientHeight;
    if (w === 0 || h === 0) return;
    const dpr = window.devicePixelRatio || 1;
    const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (plotCv.width !== pw || plotCv.height !== ph) { plotCv.width = pw; plotCv.height = ph; }
    const outpad = 6, gapX = 10, gapY = 10;
    const padL = 46, padR = 12, padT = 18, padB = 18;
    const cellW = (w - outpad * 2 - gapX * (PLOT_COLS - 1)) / PLOT_COLS;
    const cellH = (h - outpad * 2 - gapY * (PLOT_ROWS - 1)) / PLOT_ROWS;
    PANELS.forEach((panel, pi) => {
      const col = pi % PLOT_COLS, rowi = Math.floor(pi / PLOT_COLS);
      const ox = outpad + col * (cellW + gapX);
      const oy = outpad + rowi * (cellH + gapY);
      const px0 = ox + padL, py0 = oy + padT;
      const plotW = cellW - padL - padR, plotH = cellH - padT - padB;
      let ymin = Infinity, ymax = -Infinity;
      for (const key of [panel.meas, panel.cmd]) {
        const arr = key && run.ch && run.ch[key];
        if (!arr) continue;
        for (let i = 0; i < arr.length; i++) {
          const v = arr[i];
          if (v != null) { if (v < ymin) ymin = v; if (v > ymax) ymax = v; }
        }
      }
      if (!isFinite(ymin) || !isFinite(ymax)) { ymin = -1; ymax = 1; }
      if (ymax - ymin < 1e-6) { const cc = (ymin + ymax) / 2; ymin = cc - 1; ymax = cc + 1; }
      const mg = (ymax - ymin) * 0.08; ymin -= mg; ymax += mg;
      const Y = (v) => py0 + (1 - (v - ymin) / (ymax - ymin)) * plotH;
      plotGeom.push({ ox, oy, cellW, cellH, px0, py0, plotW, plotH, panel, ymin, ymax, Y });
    });
  }

  function drawPlots(sim, t0, t1) {
    const w = plotsEl.clientWidth, h = plotsEl.clientHeight;
    if (w === 0 || h === 0 || !plotGeom.length) return;
    const dpr = window.devicePixelRatio || 1;
    const c = plotCtx;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);

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
      c.fillText(g.panel.label + " [" + g.panel.unit + "]", g.ox + 2, g.oy + 13);

      // パネル矩形でクリップ
      c.save();
      c.beginPath(); c.rect(g.px0, g.py0, g.plotW, g.plotH); c.clip();

      // 観測 (実線・実機色) / 指令 (破線・灰)。窓内 [iLo,iHi] のみ。null で break。
      const lines = [
        { key: g.panel.meas, color: run.color, dash: false, alpha: 1.0, lw: 1.6 },
        { key: g.panel.cmd, color: CMD_COLOR, dash: true, alpha: 0.9, lw: 1.2 },
      ];
      for (const sp of lines) {
        const arr = sp.key && run.ch && run.ch[sp.key];
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

      // シミュレーション (赤・太め): rollout 出力の該当系列。
      const sarr = sim && sim[g.panel.sim];
      if (sarr && sarr.length > 1) {
        c.strokeStyle = SIM_COLOR; c.lineWidth = 2.0;
        c.beginPath();
        c.moveTo(X(sim.t[0]), g.Y(sarr[0]));
        for (let i = 1; i < sarr.length; i++) c.lineTo(X(sim.t[i]), g.Y(sarr[i]));
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
      const [t0, t1] = plotWindow();
      const sim = rollout(t0, t1);     // 1 フレーム 1 回だけ積分し地図・プロットで共有
      const cont = drawMap(sim);
      if (plotStaticDirty) { computePlotGeom(); plotStaticDirty = false; }
      drawPlots(sim, t0, t1);
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
