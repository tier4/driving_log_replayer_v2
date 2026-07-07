"""一次遅れ + 純粋遅延モデルの同定カーネル (SSOT)。

かつて縦/操舵の「遅延グリッド探索 × log-τ の bounded 最適化 (+ 任意の射影スケール /
サブサンプル遅延再最適化)」が physical_tuning.py (production)・lib/_physical_validity.py
(検証再同定)・tools/fit_lon_model.py (実験) の3箇所に別実装され乖離していた。ここに
純粋な数値カーネルとして一本化し、各呼び出し側は「ロード構造」と「DT・スケール有無・
勾配補正・fractional delay の差分」だけをフラグで渡す。

- sim_first_order / sim_first_order_frac : 純粋遅延 + 一次遅れシミュレーション
- fit_first_order_delay                 : 遅延グリッド × log-τ 同定 (+ 任意スケール/frac) — 出力誤差型 (推定量)
- savgol_derivative / savgol_smooth     : 状態微分 (LHS) の SG 平滑化・回帰子平滑化
- equation_residual_at_params           : 与パラメータでの ODE 残差 E=RHS−LHS を評価 (整合診断・同定はしない)

パラメータ推定は出力誤差型 (fit_first_order_delay = 一次遅れシミュレーションと実測の差の最小化) で行う。
実データでは方程式残差 (RHS−LHS) を目的関数にすると、むだ時間と時定数のトレードオフにより τ が構造的に
膨張する (縦・操舵とも出力誤差型の 3〜15 倍) ため、推定量には用いない。方程式残差は
`equation_residual_at_params` で「同定結果が ODE をどれだけ満たすか」の整合診断 (レポート統一記法
`E = RHS(param) − LHS`) として評価する。参照設計 ~/software/vehicle_model_fitting も同じ残差形だが、
非線形 least_squares の bounded 最適化で τ を境界にクランプして初めて抑えている点に注意。
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import lfilter, savgol_filter


def delay_shift(cmd: np.ndarray, n_delay: int) -> np.ndarray:
    """指令 cmd を n_delay サンプルぶん遅延させる (先頭は cmd[0] で埋める)。"""
    if n_delay <= 0:
        return cmd.copy()
    n = len(cmd)
    cmd_del = np.empty(n)
    cmd_del[:n_delay] = cmd[0]
    cmd_del[n_delay:] = cmd[:-n_delay]
    return cmd_del


def delay_shift_frac(cmd: np.ndarray, t_s: np.ndarray, delay: float) -> np.ndarray:
    """指令 cmd を非整数サンプル (連続値) delay [s] ぶん遅延させる (線形補間)。

    T のグリッドサーチ (delay_shift, 1サンプル刻み) では表現できないサブサンプル精度での
    T 再最適化 substep 用。t_s より前を参照する場合は cmd[0] で外挿する。
    """
    return np.interp(t_s - delay, t_s, cmd, left=cmd[0])


def sim_first_order(
    cmd: np.ndarray,
    tau: float,
    n_delay: int,
    dt: float,
    y0: float | None = None,
) -> np.ndarray:
    """純粋遅延 + 一次遅れシミュレーション。

    y0 を指定した場合は出力初期値を観測状態に合わせ、k=1 以降を一次遅れで更新する。
    未指定時はゼロ初期状態の lfilter として評価する。tau<=0 は瞬時追従 (alpha=1) にフォールバック。
    """
    cmd_del = delay_shift(cmd, n_delay)
    alpha = 1.0 if tau <= 0.0 else float(np.clip(dt / tau, 0.0, 1.0))
    if y0 is not None:
        if len(cmd_del) == 0:
            return cmd_del.astype(float)
        zi = [float(y0) - alpha * float(cmd_del[0])]
        out, _ = lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del, zi=zi)
        return out
    return lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del)


def sim_first_order_frac(
    cmd: np.ndarray,
    tau: float,
    delay: float,
    t_s: np.ndarray,
    dt: float,
    y0: float | None = None,
) -> np.ndarray:
    """`sim_first_order` の連続 delay [s] 版 (T 再最適化 substep 専用、tau は呼び出し側で固定)。"""
    cmd_del = delay_shift_frac(cmd, t_s, delay)
    alpha = 1.0 if tau <= 0.0 else float(np.clip(dt / tau, 0.0, 1.0))
    if y0 is not None:
        if len(cmd_del) == 0:
            return cmd_del.astype(float)
        zi = [float(y0) - alpha * float(cmd_del[0])]
        out, _ = lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del, zi=zi)
        return out
    return lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del)


def fit_first_order_delay(
    cmd: np.ndarray,
    act: np.ndarray,
    mask: np.ndarray,
    dt: float,
    *,
    tau_bounds: tuple[float, float],
    delay_candidates: np.ndarray,
    y0: float | None = None,
    fit_scale: bool = False,
    scale_bounds: tuple[float, float] = (0.8, 1.2),
    frac_delay: bool = False,
    t_s: np.ndarray | None = None,
) -> dict | None:
    """一次遅れ + 純粋遅延モデルを遅延グリッド × log-τ で同定する。

    各遅延候補で τ を `minimize_scalar`(bounded, log 空間) 最適化し、mask 上の MSE を最小化する
    (τ, delay) を採る。オプション:
      - fit_scale=True : 各 τ で射影スケール s=<sim·act>/<sim·sim> を scale_bounds でクリップして
        当てはめる (production の debug_*_scaling_factor)。
      - y0            : 出力初期値を観測に合わせる (勾配補正済み信号のフィット等)。
      - frac_delay    : 最良 (τ,delay) の近傍 ±1グリッドで delay を連続再最適化 (サブサンプル精度)。

    Returns: {"tau", "delay", "scale", "rmse", "mse"} | None (τ が求まらない場合)。
    """
    log_lo, log_hi = np.log(tau_bounds[0]), np.log(tau_bounds[1])

    def _mse_scale(tau: float, n_delay: int) -> tuple[float, float]:
        base = sim_first_order(cmd, tau, n_delay, dt, y0=y0)
        if fit_scale:
            s2 = float(np.sum(base[mask] ** 2))
            scale = (
                float(np.clip(np.sum(base[mask] * act[mask]) / s2, *scale_bounds))
                if s2 > 1e-5
                else 1.0
            )
        else:
            scale = 1.0
        mse = float(np.mean((scale * base[mask] - act[mask]) ** 2))
        return mse, scale

    best_mse, best_tau, best_delay, best_scale = np.inf, np.nan, np.nan, 1.0
    for delay_s in delay_candidates:
        n_delay = int(round(float(delay_s) / dt))
        res = minimize_scalar(
            lambda lt, nd=n_delay: _mse_scale(float(np.exp(lt)), nd)[0],
            bounds=(log_lo, log_hi),
            method="bounded",
        )
        if res.fun < best_mse:
            tau = float(np.exp(res.x))
            _, scale = _mse_scale(tau, n_delay)
            best_mse, best_tau, best_delay, best_scale = res.fun, tau, float(delay_s), scale

    if np.isnan(best_tau):
        return None

    if frac_delay and t_s is not None:
        lo = max(0.0, best_delay - dt)
        hi = min(float(delay_candidates[-1]), best_delay + dt)

        def _mse_frac(delay: float) -> float:
            sim = sim_first_order_frac(cmd, best_tau, delay, t_s, dt, y0=y0)
            return float(np.mean((sim[mask] - act[mask]) ** 2))

        res_d = minimize_scalar(_mse_frac, bounds=(lo, hi), method="bounded")
        if res_d.fun < best_mse:
            best_mse, best_delay = res_d.fun, float(res_d.x)

    return {
        "tau": best_tau,
        "delay": best_delay,
        "scale": best_scale,
        "rmse": float(np.sqrt(best_mse)),
        "mse": best_mse,
    }


# ---------------------------------------------------------------------------
# 方程式残差の SG 平滑化微分 (LHS) と診断評価
#
# 参照設計 ~/software/vehicle_model_fitting/design.md: 各 ODE を E = RHS(param) − LHS に整理する
# (LHS = 状態微分)。本モジュールではこの残差を「推定量」には使わず (実データで τ が膨張するため)、
# 出力誤差型で同定したパラメータの整合診断として `equation_residual_at_params` で評価する。
# SG 平滑化微分 (savgol_derivative) は加速度/操舵微分のノイズを抑えて LHS を作るために用いる。
# ---------------------------------------------------------------------------


def _savgol_window(n: int, dt: float, window_s: float, polyorder: int) -> int | None:
    """window_s [s] を有効な SG 窓長 (奇数・polyorder<win<=n) に換算する。取れない場合 None。"""
    win = int(round(window_s / dt))
    if win % 2 == 0:
        win += 1
    if win <= polyorder:
        win = polyorder + 1 if (polyorder + 1) % 2 == 1 else polyorder + 2
    if win > n:
        win = n if n % 2 == 1 else n - 1
    if win <= polyorder:
        return None
    return win


def savgol_derivative(
    y: np.ndarray, dt: float, window_s: float = 0.2, polyorder: int = 2,
) -> np.ndarray:
    """Savitzky-Golay 平滑化1階微分 (窓は秒指定)。窓が取れない短信号は np.gradient に縮退する。

    方程式残差診断の LHS (状態微分) 生成用。窓を秒で定義するのは DT が異なる呼び出し側 (production
    DT=0.01 と検証 1/30) で挙動を揃えるため。窓は同定対象 τ 下限より短く保つこと (立ち上がりを
    過平滑化すると τ を過大評価する)。
    """
    y = np.asarray(y, dtype=float)
    win = _savgol_window(len(y), dt, window_s, polyorder)
    if win is None:
        return np.gradient(y, dt)
    return savgol_filter(y, window_length=win, polyorder=polyorder, deriv=1, delta=dt)


def savgol_smooth(
    y: np.ndarray, dt: float, window_s: float = 0.2, polyorder: int = 2,
) -> np.ndarray:
    """Savitzky-Golay 平滑化 (deriv=0)。診断残差で RHS の回帰子 act にも LHS と同じ窓を適用し一貫させる。"""
    y = np.asarray(y, dtype=float)
    win = _savgol_window(len(y), dt, window_s, polyorder)
    if win is None:
        return y.copy()
    return savgol_filter(y, window_length=win, polyorder=polyorder, deriv=0, delta=dt)


def equation_residual_at_params(
    cmd: np.ndarray,
    act: np.ndarray,
    mask: np.ndarray,
    dt: float,
    *,
    tau: float,
    delay: float,
    scale: float = 1.0,
    bias: float = 0.0,
    window_s: float = 0.2,
    polyorder: int = 2,
    t_s: np.ndarray | None = None,
) -> dict:
    """与えられたパラメータでの ODE 方程式残差 `E[k] = RHS − LHS` を評価する (同定はしない)。

    full-RHS 遅延: cmd・act の両方を同じ遅延 T で評価する。
    `dot_act = (s·cmd(t−T) − (act(t−T) − β))/τ`
    これにより 1-1 の状態方程式（full-RHS 遅延）および vehicle_model_fitting の残差式と一致する。
    LHS は act の SG 平滑化微分、RHS は上式の右辺。目的関数ではなく整合診断・レポート統一記法用。

    Returns: {"resid": E[mask] 配列, "rmse_resid": √mean(E²), "n": サンプル数}。
    """
    cmd = np.asarray(cmd, dtype=float)
    act = np.asarray(act, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    if not np.isfinite(tau) or tau <= 0 or int(np.count_nonzero(mask)) == 0:
        return {"resid": np.array([]), "rmse_resid": float("nan"), "n": 0}

    lhs = savgol_derivative(act, dt, window_s, polyorder)
    act_s = savgol_smooth(act, dt, window_s, polyorder)
    if t_s is not None:
        cmd_del = delay_shift_frac(cmd,   t_s, float(delay))
        act_del = delay_shift_frac(act_s, t_s, float(delay))
    else:
        n_steps = int(round(float(delay) / dt))
        cmd_del = delay_shift(cmd,   n_steps)
        act_del = delay_shift(act_s, n_steps)
    rhs = (scale * cmd_del - (act_del - bias)) / tau
    r = (rhs - lhs)[mask]
    return {
        "resid": r,
        "rmse_resid": float(np.sqrt(np.mean(r ** 2))),
        "n": int(r.size),
    }


def _moving_avg(x: np.ndarray, w: int) -> np.ndarray:
    """中央移動平均フィルタ（窓幅 w、端は edge-padding）。

    np.convolve mode='same' を利用して前後均等のフィルタを適用する。
    """
    if w <= 1:
        return np.asarray(x, dtype=float).copy()
    x = np.asarray(x, dtype=float)
    half = w // 2
    padded = np.pad(x, (half, w - 1 - half), mode="edge")
    return np.convolve(padded, np.ones(w) / w, mode="valid")


def fit_first_order_delay_residual_3phase(
    cmd: np.ndarray,
    act: np.ndarray,
    mask: np.ndarray,
    dt: float,
    *,
    tau_bounds: tuple[float, float],
    delay_candidates: np.ndarray,
    fit_scale: bool = False,
    scale_bounds: tuple[float, float] = (0.8, 1.2),
    filter_w: int = 1,
    x0_dict: dict | None = None,
    fixed_phase1_tau: float | None = None,
    skip_phase3: bool = False,
) -> dict | None:
    """3段階交互最適化による一次遅れ＋純粋遅延の方程式残差最小二乗フィッティング。

    最適化変数:
      - fit_scale = False: [tau_inv]
      - fit_scale = True : [tau_inv, scale]

    逆数変数 tau_inv = 1 / tau を用いることで勾配爆発を防ぐ。

    Phase 1: d=0 固定で least_squares 最適化（`fixed_phase1_tau` 指定時はこの最適化自体を
             スキップし、その値を tau として固定した上で Phase 2 に渡す）
    Phase 2: Phase 1 パラメータ固定で delay_candidates グリッドサーチ (MSE最小)
    Phase 3: 最良 delay 固定で least_squares 再最適化
    """
    cmd = np.asarray(cmd, dtype=float)
    act = np.asarray(act, dtype=float)
    mask = np.asarray(mask, dtype=bool)

    if int(np.count_nonzero(mask)) == 0:
        return None

    # 状態微分（LHS）の算出
    dot_act = np.gradient(act, dt)

    # 移動平均（LPF）の適用
    if filter_w > 1:
        cmd_f = _moving_avg(cmd, filter_w)
        act_f = _moving_avg(act, filter_w)
        dot_act_f = _moving_avg(dot_act, filter_w)
    else:
        cmd_f = cmd.copy()
        act_f = act.copy()
        dot_act_f = dot_act.copy()

    # ウォームスタート（初期値）設定
    tau_init = 0.2
    scale_init = 1.0
    delay_init = 0.0
    if x0_dict is not None:
        for t_key in ["tau", "tau_a", "tau_delta"]:
            if t_key in x0_dict and x0_dict[t_key] is not None and x0_dict[t_key] > 0:
                tau_init = float(x0_dict[t_key])
                break
        if "scale" in x0_dict and x0_dict["scale"] is not None:
            scale_init = float(x0_dict["scale"])
        for d_key in ["delay", "T", "acc_time_delay", "steer_time_delay"]:
            if d_key in x0_dict and x0_dict[d_key] is not None and x0_dict[d_key] >= 0:
                delay_init = float(x0_dict[d_key])
                break

    tau_inv_init = 1.0 / tau_init
    n_steps_init = int(round(delay_init / dt))

    x0 = [tau_inv_init, scale_init] if fit_scale else [tau_inv_init]

    # Bounds 設定
    tau_inv_min = 1.0 / tau_bounds[1]
    tau_inv_max = 1.0 / tau_bounds[0]

    if fit_scale:
        bounds = (
            [tau_inv_min, scale_bounds[0]],
            [tau_inv_max, scale_bounds[1]]
        )
    else:
        bounds = (
            [tau_inv_min],
            [tau_inv_max]
        )

    def _residuals(x, n_steps):
        tau_inv = float(x[0])
        s = float(x[1]) if fit_scale else 1.0

        c_del = delay_shift(cmd_f, n_steps)
        a_del = delay_shift(act_f, n_steps)

        E = tau_inv * (s * c_del - a_del) - dot_act_f
        return E[mask]

    # Phase 1: d = delay_init 固定で最適化（fixed_phase1_tau 指定時は最適化をスキップし、
    # その値を tau として固定した x をそのまま Phase 2 に渡す）
    if fixed_phase1_tau is not None:
        phase1_tau = float(fixed_phase1_tau)
        res1_x = [1.0 / phase1_tau, scale_init] if fit_scale else [1.0 / phase1_tau]
    else:
        res1 = least_squares(lambda x: _residuals(x, n_steps_init), x0, bounds=bounds)
        phase1_tau = 1.0 / float(res1.x[0])
        res1_x = res1.x

    # Phase 2: 得られたパラメータ固定で delay をグリッドサーチ
    best_cost = None
    best_n = 0
    for delay_s in delay_candidates:
        n_steps = int(round(float(delay_s) / dt))
        cost = float(np.sum(_residuals(res1_x, n_steps) ** 2))
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_n = n_steps
    phase2_delay = float(best_n * dt)

    # Phase 3: 最良 delay 固定でパラメータを再最適化
    if skip_phase3:
        tau_inv_fit = float(res1_x[0])
        tau_fit = 1.0 / tau_inv_fit
        scale_fit = float(res1_x[1]) if fit_scale else 1.0
        delay_fit = phase2_delay
    else:
        res3 = least_squares(lambda x: _residuals(x, best_n), res1_x, bounds=bounds)
        tau_inv_fit = float(res3.x[0])
        tau_fit = 1.0 / tau_inv_fit
        scale_fit = float(res3.x[1]) if fit_scale else 1.0
        delay_fit = float(best_n * dt)

    n_samples = int(np.count_nonzero(mask))
    rmse_fit = float(np.sqrt(best_cost / n_samples)) if n_samples > 0 else float("nan")

    # LPF 後の (平滑化済み) cmd_f, act_f, dot_act_f を用いて、最終パラメータにおける残差を計算する
    # (同定はLPF後で行われているため、診断残差もLPF後を主値とする)
    resid_final = []
    resid_final_raw = []
    if n_samples > 0:
        c_del_f = delay_shift(cmd_f, best_n)
        a_del_f = delay_shift(act_f, best_n)
        E_f = tau_inv_fit * (scale_fit * c_del_f - a_del_f) - dot_act_f
        resid_final = E_f[mask].tolist()

        # 生の (平滑化していない) 残差は時系列グラフでの参考値として別途保持
        c_del_raw = delay_shift(cmd, best_n)
        a_del_raw = delay_shift(act, best_n)
        dot_act_raw = np.gradient(act, dt)
        E_raw = tau_inv_fit * (scale_fit * c_del_raw - a_del_raw) - dot_act_raw
        resid_final_raw = E_raw[mask].tolist()

    return {
        "tau": tau_fit,
        "delay": delay_fit,
        "scale": scale_fit,
        "rmse": rmse_fit,
        "resid_samples": resid_final,        # LPF 後の残差（主値：ヒスト集計・RMSE 計算用）
        "resid_samples_raw": resid_final_raw,  # 生信号での残差（時系列グラフ参考値用）
        "cost": best_cost,
        "phase1_tau": phase1_tau,
        "phase1_delay": delay_init,
        "phase2_delay": phase2_delay,
        "phase3_tau": tau_fit,
        "phase3_delay": delay_fit,
    }

