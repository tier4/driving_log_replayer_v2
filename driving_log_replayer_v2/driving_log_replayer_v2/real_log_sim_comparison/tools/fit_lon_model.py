#!/usr/bin/env python3
"""縦方向モデルの高速イテレーション用ハーネス (real.lite を直読み、少数データセットで数秒)。

2 つの用途を兼ねる:

(1) C++ 拡張モデルのパラメータ同定 (従来機能):
    a_target(t) = a_cmd(t - T) + (c0 + c1·v + c2·v²)
    ȧ = -(a - a_target) / τ,   τ = τ_thr (a_cmd>=0) / τ_brk (a_cmd<0)
    前方 Euler 積分による出力 (a) 誤差最小化。poly は τ 固定で LS、τ_thr/τ_brk はグリッド探索。

(2) 高速イテレーション用の縦方向フィット＋診断プロット (--plot):
    vehicle_model_fitting のテクニックを移植。検証側 SSOT (lib/_physical_validity.py) と同じ
    一次遅れ＋重力補正モデル:
        d(a_corr)/dt = (a_cmd(t - T) - a_corr) / τ,   a_corr = a_act - g·sin(pitch)

    τ の点推定は **出力誤差 (シミュレーション誤差) 形** で行う (方程式を1回積分した形)。
    当初は「瞬時残差」= LHS - RHS (LHS=ȧ_corr の4次中央差分, RHS=τ⁻¹(a_cmd(t-T)-a_corr)) で
    τ を線形一発解する計画だったが、実データで corr(ȧ,Δ)≈0.25 の SNR 壁 (微分ノイズ増幅) により
    ill-conditioned で τ が過大化することを実測で確認。積分形 (出力誤差) が微分ノイズを正則化する
    (教科書的な equation-error vs output-error system ID)。出力誤差は載荷 (real.lite 少数直読み) 律速で
    瞬時形と同じく数秒。SSOT fit_long_single と同形なので rollout 目的とも整合する。

    診断プロットは「構造発見」と「コンディショニング」の両方を出す:
        - 出力誤差残差 (a_sim - a_corr) vs v / a_cmd / g·sin(pitch) / 時刻 — well-posed。
          構造 (走行抵抗, throttle/brake 非対称, 重力ゲイン) が見えたらモデルに項が足りない = 改良の入口
        - 瞬時 LHS vs RHS の散布 — コンディショニング診断に降格 (縦潰れが上記 SNR 壁を可視化)
        - シミュレーション a_sim vs 実測 a_corr の時系列重ね描き (rollout 目的)

注意 (この高速プロキシの割り切り): gear=DRIVE マスクは省略し vx / |ȧ_cmd| 閾値のみ使う。
最終検証は従来どおり local_multidataset_cloud_run フルパイプラインが担う。

使い方:
    python3 tools/fit_lon_model.py LITE_DIR [LITE_DIR ...]              # 従来: C++ パラメータ同定
    python3 tools/fit_lon_model.py --plot LITE_DIR [...]               # 瞬時残差診断を表示
    python3 tools/fit_lon_model.py --plot --plot-out fig.png LITE_DIR  # PNG 保存 (ヘッドレス)
"""
from __future__ import annotations

import argparse
import glob
import math

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

DT = 0.02          # 50 Hz リサンプル
T_DEADTIME = 0.10  # acc_time_delay spec (C++ は単一キュー)
GRAVITY = 9.81     # [m/s²] 重力加速度 (lib/_physical_validity._GRAVITY と一致)

# 瞬時残差フィットの動的マスク閾値 (lib/_physical_validity の _VX_MIN_FIT / _DA_THRESH_FIT と一致)
VX_MIN_FIT = 0.5
DA_THRESH_FIT = 0.15
LOWPASS_W = 7      # 瞬時形コンディショニング診断の両辺移動平均窓 [サンプル] (50Hz で約 0.14s)
DELAY_GRID = np.arange(0.0, 0.31, DT)  # 出力誤差フィットの純粋むだ時間 T 探索グリッド [s]


def _pitch_from_quat(q) -> float:
    """クォータニオン (x,y,z,w) から pitch [rad] を算出 (asin をクランプして定義域外を防ぐ)。"""
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def _read(litedir: str) -> dict:
    mc = glob.glob(litedir + "/*.mcap")[0]
    r = SequentialReader()
    r.open(StorageOptions(uri=mc, storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out: dict = {k: ([], []) for k in ("a", "v", "cmd", "wz", "pitch")}
    # 1 トピックから複数系列を抽出できるよう値は (key, fn) のリスト。
    topic_map = {
        "/localization/acceleration": [("a", lambda m: m.accel.accel.linear.x)],
        "/vehicle/status/velocity_status": [("v", lambda m: m.longitudinal_velocity)],
        "/control/command/control_cmd": [("cmd", lambda m: m.longitudinal.acceleration)],
        "/localization/kinematic_state": [
            ("wz", lambda m: m.twist.twist.angular.z),
            ("pitch", lambda m: _pitch_from_quat(m.pose.pose.orientation)),
        ],
    }
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in topic_map:
            try:
                msg = deserialize_message(data, get_message(types[topic]))
            except Exception:
                continue
            for key, fn in topic_map[topic]:
                try:
                    val = fn(msg)
                except Exception:
                    continue
                out[key][0].append(t / 1e9)
                out[key][1].append(val)
    return {k: (np.asarray(ts), np.asarray(vs)) for k, (ts, vs) in out.items()}


def _resample(series: dict) -> dict:
    """共通時間グリッド (DT) に線形補間。全系列が存在する重なり区間に限定。"""
    t0 = max(series[k][0].min() for k in series)
    t1 = min(series[k][0].max() for k in series)
    grid = np.arange(t0, t1, DT)
    res = {"t": grid}
    for k, (ts, vs) in series.items():
        res[k] = np.interp(grid, ts, vs)
    return res


def _integrate(target: np.ndarray, tau: np.ndarray, a0: float) -> np.ndarray:
    """ȧ = -(a - target)/tau を前方 Euler 積分 (時変 tau)。"""
    a = np.empty_like(target)
    a[0] = a0
    for i in range(len(target) - 1):
        a[i + 1] = a[i] + DT * (-(a[i] - target[i]) / tau[i])
    return a


# ---------------------------------------------------------------------------
# 瞬時残差 (instantaneous residual) フィット + 診断 (vehicle_model_fitting 由来)
# ---------------------------------------------------------------------------
def _central_diff4(x: np.ndarray, dt: float) -> np.ndarray:
    """4 次精度中央差分。境界近傍 (両端 2 点) は np.gradient(2 次) に縮退。"""
    d = np.gradient(x, dt)
    if len(x) >= 5:
        d[2:-2] = (-x[4:] + 8 * x[3:-1] - 8 * x[1:-3] + x[:-4]) / (12 * dt)
    return d


def _moving_avg(x: np.ndarray, w: int) -> np.ndarray:
    """両側移動平均ローパス (edge-padding で端を保つ)。瞬時残差は両辺に同一フィルタを掛ける。"""
    if w <= 1:
        return x
    pad = w // 2
    xp = np.pad(x, pad, mode="edge")
    kern = np.ones(w) / w
    return np.convolve(xp, kern, mode="valid")[: len(x)]


def _shift(cmd: np.ndarray, n: int) -> np.ndarray:
    """指令を n サンプル遅延 (先頭は cmd[0] で埋める)。"""
    if n <= 0:
        return cmd.copy()
    out = np.empty_like(cmd)
    out[:n] = cmd[0]
    out[n:] = cmd[:-n]
    return out


def _sim_first_order(cmd: np.ndarray, tau: float, n_delay: int, a0: float) -> np.ndarray:
    """純粋遅延 + 一次遅れ (lfilter, 初期値 a0)。SSOT lib/_physical_validity._sim_first_order と同形。"""
    cmd_del = _shift(cmd, n_delay)
    alpha = 1.0 if tau <= 0.0 else float(np.clip(DT / tau, 0.0, 1.0))
    zi = [float(a0) - alpha * float(cmd_del[0])]
    out, _ = lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del, zi=zi)
    return out


def fit_long_diagnostic(datasets: list[dict], lowpass_w: int = LOWPASS_W) -> dict:
    """縦方向一次遅れ+重力補正モデルを **出力誤差 (シミュレーション誤差)** で同定し、診断配列も返す。

    モデル (SSOT lib/_physical_validity.fit_long_single と同一):
        d(a_corr)/dt = (a_cmd(t-T) - a_corr)/τ,   a_corr = a_act - g·sin(pitch)

    τ 点推定は出力誤差形 (方程式を1回積分した形) を使う。瞬時 (微分) 形は ȧ のノイズ増幅で
    ill-conditioned (実測データで corr(ȧ,Δ)≈0.25 の SNR 壁) なため点推定には使わない。
    瞬時残差 (LHS vs RHS) はコンディショニング診断としてプロットにのみ残す。

    構造発見 (欠けている項) は出力誤差残差 (a_sim - a_corr) vs v/a_cmd/勾配 で見る (well-posed)。
    """
    pre = []
    for d in datasets:
        gsin = GRAVITY * np.sin(d["pitch"])
        a_corr = d["a"] - gsin              # 出力誤差フィットは生信号 (積分が正則化)
        cmd = d["cmd"]
        dcmd = np.abs(np.gradient(cmd, DT))
        mask = (d["v"] > VX_MIN_FIT) & (dcmd > DA_THRESH_FIT)
        if int(mask.sum()) < 50:
            continue                        # 空/過少マスクのデータセットは除外
        pre.append({"t": d["t"], "a_corr": a_corr, "cmd": cmd, "v": d["v"],
                    "gsin": gsin, "mask": mask})
    if not pre:
        raise SystemExit("有効な縦方向動的サンプルを持つデータセットがありません。")

    # 出力誤差フィット: delay グリッド × log(τ) の bounded 最適化で pooled MSE 最小
    def pooled_mse(log_tau: float, n: int) -> float:
        tau = float(np.exp(log_tau))
        num = cnt = 0.0
        for p in pre:
            a_sim = _sim_first_order(p["cmd"], tau, n, float(p["a_corr"][0]))
            m = p["mask"]
            num += float(np.sum((a_sim[m] - p["a_corr"][m]) ** 2))
            cnt += int(m.sum())
        return num / cnt

    best = None
    for delay in DELAY_GRID:
        n = int(round(delay / DT))
        r = minimize_scalar(lambda lt, nn=n: pooled_mse(lt, nn),
                            bounds=(np.log(0.03), np.log(5.0)), method="bounded")
        if best is None or r.fun < best["mse"]:
            best = {"mse": r.fun, "tau": float(np.exp(r.x)), "delay": float(delay), "n": n}

    tau, n = best["tau"], best["n"]
    rmse = math.sqrt(best["mse"])

    # 診断配列を組み立て
    res_a, v_a, cmd_a, gsin_a, t_a = ([] for _ in range(5))      # 出力誤差残差 (構造発見の主役)
    lhs_a, rhs_a = [], []                                        # 瞬時 LHS/RHS (コンディショニング診断)
    t_off = 0.0
    rep = None  # オーバーレイ用代表データセット (マスク最多)
    for p in pre:
        m = p["mask"]
        a_sim = _sim_first_order(p["cmd"], tau, n, float(p["a_corr"][0]))
        res = a_sim - p["a_corr"]
        res_a.append(res[m]); v_a.append(p["v"][m]); cmd_a.append(_shift(p["cmd"], n)[m])
        gsin_a.append(p["gsin"][m]); t_a.append(p["t"][m] - p["t"][0] + t_off)
        # 瞬時形 (両辺ローパス、best τ で評価): LHS=ȧ_corr, RHS=(a_cmd(t-T)-a_corr)/τ
        a_corr_lp = _moving_avg(p["a_corr"], lowpass_w)
        cmd_lp = _moving_avg(p["cmd"], lowpass_w)
        lhs = _central_diff4(a_corr_lp, DT)
        rhs = (_shift(cmd_lp, n) - a_corr_lp) / tau
        mi = m.copy(); mi[:2] = False; mi[-2:] = False
        lhs_a.append(lhs[mi]); rhs_a.append(rhs[mi])
        t_off += (p["t"][-1] - p["t"][0]) + 1.0
        if rep is None or int(m.sum()) > rep["nm"]:
            rep = {"nm": int(m.sum()), "t": p["t"] - p["t"][0],
                   "a_corr": p["a_corr"], "cmd_del": _shift(p["cmd"], n), "a_sim": a_sim}

    return {
        "tau": tau, "delay": best["delay"], "rmse": rmse, "n": sum(int(p["mask"].sum()) for p in pre),
        "res": np.concatenate(res_a), "v": np.concatenate(v_a), "cmd": np.concatenate(cmd_a),
        "gsin": np.concatenate(gsin_a), "t": np.concatenate(t_a),
        "lhs": np.concatenate(lhs_a), "rhs": np.concatenate(rhs_a), "rep": rep,
    }


def plot_diagnostics(fit_res: dict, out_path: str | None = None) -> None:
    """縦方向診断プロット (2×3)。出力誤差残差で構造を見て、瞬時散布で SNR を見る。"""
    import matplotlib
    if out_path:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm

    # 日本語フォント自動検出 (無ければ既定のまま。ラベルは英数字も併記済み)
    for cand in ("Noto Sans CJK JP", "Noto Sans JP", "IPAGothic", "IPAPGothic",
                 "TakaoGothic", "VL Gothic", "Yu Gothic"):
        if any(cand in f.name for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False

    rng = np.random.default_rng(0)

    def _sub(*arrs):
        n = len(arrs[0])
        idx = rng.choice(n, size=min(n, 6000), replace=False) if n > 6000 else np.arange(n)
        return [a[idx] for a in arrs]

    res, v, cmd, gsin, t = _sub(fit_res["res"], fit_res["v"], fit_res["cmd"], fit_res["gsin"], fit_res["t"])
    lhs, rhs = _sub(fit_res["lhs"], fit_res["rhs"])

    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(
        f"縦方向モデル診断 (出力誤差フィット)  τ={fit_res['tau']:.3f}s  T={fit_res['delay']:.3f}s  "
        f"sim RMSE={fit_res['rmse']:.4f} m/s²  (n={fit_res['n']})", fontsize=13)

    # 出力誤差残差 vs 説明変数 (構造が見えたらモデルに項が足りない = 改良の入口)
    for a_, xv, xl, ttl in [
        (ax[0, 0], v, "v [m/s]", "残差 vs 速度 (勾配→走行抵抗 c1·v+c2·v²)"),
        (ax[0, 1], cmd, "a_cmd(t-T) [m/s²]", "残差 vs 指令 (符号依存→throttle/brake τ 非対称)"),
        (ax[0, 2], gsin, "g·sin(pitch) [m/s²]", "残差 vs 勾配 (勾配→重力ゲイン k_g≠1)"),
        (ax[1, 0], t, "t [s] (全データセット連結)", "残差 vs 時刻 (ドリフト/過渡の確認)"),
    ]:
        a_.scatter(xv, res, s=3, alpha=0.3)
        a_.axhline(0, color="r", lw=0.8)
        a_.set_xlabel(xl); a_.set_ylabel("残差 = a_sim - a_corr [m/s²]"); a_.set_title(ttl)

    # (1,1) 瞬時 LHS vs RHS (コンディショニング診断: 縦潰れ = 微分 SNR 壁)
    r = np.corrcoef(lhs, rhs)[0, 1]
    ax[1, 1].scatter(rhs, lhs, s=3, alpha=0.3)
    lim = [min(lhs.min(), rhs.min()), max(lhs.max(), rhs.max())]
    ax[1, 1].plot(lim, lim, "r-", lw=1)
    ax[1, 1].set_xlabel("RHS = (a_cmd(t-T) - a_corr)/τ"); ax[1, 1].set_ylabel("LHS = ȧ_corr")
    ax[1, 1].set_title(f"瞬時形 コンディショニング (corr={r:.2f}; 縦潰れ=微分SNR壁)")

    # (1,2) シミュレーション重ね描き (rollout 目的、代表データセット=マスク最多)
    s = fit_res["rep"]
    ax[1, 2].plot(s["t"], s["a_corr"], label="a_corr (実測-重力)", lw=1)
    ax[1, 2].plot(s["t"], s["cmd_del"], label="a_cmd(t-T)", lw=0.8, alpha=0.6)
    ax[1, 2].plot(s["t"], s["a_sim"], label="a_sim", lw=1)
    ax[1, 2].set_xlabel("t [s]"); ax[1, 2].set_ylabel("a [m/s²]")
    ax[1, 2].set_title("シミュレーション vs 実測 (rollout 目的)"); ax[1, 2].legend(fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if out_path:
        fig.savefig(out_path, dpi=110)
        print(f"診断プロットを保存しました: {out_path}")
    else:
        plt.show()


def fit(datasets: list[dict]) -> dict:
    # プール: 各データセットの基底応答を連結して 1 回の LS で解く
    shift = int(round(T_DEADTIME / DT))
    grid_thr = np.arange(0.10, 0.81, 0.05)
    grid_brk = np.arange(0.10, 0.81, 0.05)
    best = None
    for tau_thr in grid_thr:
        for tau_brk in grid_brk:
            A_blocks, y_blocks = [], []
            for d in datasets:
                cmd = d["cmd"]
                cmd_del = np.concatenate([np.full(shift, cmd[0]), cmd[:-shift]]) if shift else cmd
                tau = np.where(cmd_del >= 0.0, tau_thr, tau_brk)
                v, wz, a = d["v"], d["wz"], d["a"]
                # 指令応答 (IC = 観測初期 a)
                a_cmd_resp = _integrate(cmd_del, tau, a[0])
                # 基底応答 (IC=0): poly1,poly2,coupling
                # poly0(定数)は除外: 停止時(v=0)に spurious 加速を生むと closed-loop で誤発進する。
                # 走行抵抗は v=0 でゼロが物理的に正しい (poly(v)=c1·v+c2·v²)。
                bases = [v, v * v]
                basis_resp = [_integrate(b, tau, 0.0) for b in bases]
                A_blocks.append(np.column_stack(basis_resp))
                y_blocks.append(a - a_cmd_resp)
            A = np.vstack(A_blocks)
            y = np.concatenate(y_blocks)
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            resid = y - A @ coef
            rmse = float(np.sqrt(np.mean(resid ** 2)))
            if best is None or rmse < best["rmse"]:
                best = {"rmse": rmse, "tau_thr": float(tau_thr), "tau_brk": float(tau_brk),
                        "c0": 0.0, "c1": float(coef[0]),
                        "c2": float(coef[1])}
    return best


def baseline_rmse(datasets: list[dict], tau_single: float) -> float:
    """単一 τ・poly/coupling なし (現 best_normal acc_time_constant=0.30 相当) の当てはめ RMSE。"""
    shift = int(round(T_DEADTIME / DT))
    errs = []
    for d in datasets:
        cmd = d["cmd"]
        cmd_del = np.concatenate([np.full(shift, cmd[0]), cmd[:-shift]]) if shift else cmd
        tau = np.full_like(cmd_del, tau_single)
        a_sim = _integrate(cmd_del, tau, d["a"][0])
        errs.append((a_sim - d["a"]) ** 2)
    return float(np.sqrt(np.mean(np.concatenate(errs))))


def main() -> None:
    ap = argparse.ArgumentParser(description="縦方向モデル 高速イテレーションハーネス")
    ap.add_argument("lite_dirs", nargs="+", help="real.lite ディレクトリ")
    ap.add_argument("--plot", action="store_true", help="瞬時残差の診断プロットを出す")
    ap.add_argument("--plot-out", type=str, default=None, help="診断プロットの PNG 保存先 (指定でヘッドレス保存)")
    ap.add_argument("--lowpass", type=int, default=LOWPASS_W, help=f"両辺ローパス窓 [サンプル] (既定 {LOWPASS_W})")
    args = ap.parse_args()

    labels = ["odaiba", "takanawa", "ds3", "ds4"]
    datasets = []
    for i, lite in enumerate(args.lite_dirs):
        s = _resample(_read(lite))
        datasets.append(s)
        lab = labels[i] if i < len(labels) else f"ds{i}"
        print(f"[{lab}] {lite.split('/out/')[-1].split('/')[0]}: {len(s['t'])} samples "
              f"({s['t'][-1]-s['t'][0]:.0f}s), a∈[{s['a'].min():.2f},{s['a'].max():.2f}] "
              f"cmd∈[{s['cmd'].min():.2f},{s['cmd'].max():.2f}] "
              f"pitch∈[{s['pitch'].min():.3f},{s['pitch'].max():.3f}]")
    print()

    # (1) 従来の C++ パラメータ同定
    base = baseline_rmse(datasets, 0.30)
    res = fit(datasets)
    print(f"baseline (単一τ=0.30, poly/coupling なし) accel-fit RMSE = {base:.4f} m/s²")
    print(f"fitted  RMSE = {res['rmse']:.4f} m/s²  (改善 {100*(1-res['rmse']/base):.1f}%)")
    print()
    print("=== C++ 同定パラメータ (best_normal の縦方向に設定) ===")
    print(f"  acc_time_constant (throttle τ): {res['tau_thr']:.3f}")
    print(f"  brake_time_constant (brake τ) : {res['tau_brk']:.3f}")
    print(f"  lon_drag_c0  : {res['c0']:+.4f}")
    print(f"  lon_drag_c1  : {res['c1']:+.5f}")
    print(f"  lon_drag_c2  : {res['c2']:+.6f}")
    print()

    # (2) 出力誤差フィット (一次遅れ+重力補正、SSOT 準拠) + 瞬時残差診断
    ir = fit_long_diagnostic(datasets, lowpass_w=args.lowpass)
    print("=== 出力誤差フィット (一次遅れ+重力補正、SSOT fit_long_single 相当) ===")
    print(f"  tau (一次遅れ時定数)   : {ir['tau']:.4f} s")
    print(f"  T   (むだ時間)         : {ir['delay']:.4f} s")
    print(f"  sim RMSE               : {ir['rmse']:.4f} m/s²  (n={ir['n']})")
    print("  (注: 瞬時形は微分 SNR 壁で ill-conditioned のため点推定には非使用。診断のみ)")

    if args.plot or args.plot_out:
        plot_diagnostics(ir, out_path=args.plot_out)


if __name__ == "__main__":
    main()
