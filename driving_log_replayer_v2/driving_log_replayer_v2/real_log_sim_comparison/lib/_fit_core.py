"""一次遅れ + 純粋遅延モデルの同定カーネル (SSOT)。

かつて縦/操舵の「遅延グリッド探索 × log-τ の bounded 最適化 (+ 任意の射影スケール /
サブサンプル遅延再最適化)」が physical_tuning.py (production)・lib/_physical_validity.py
(検証再同定)・tools/fit_lon_model.py (実験) の3箇所に別実装され乖離していた。ここに
純粋な数値カーネルとして一本化し、各呼び出し側は「ロード構造」と「DT・スケール有無・
勾配補正・fractional delay の差分」だけをフラグで渡す。

- sim_first_order / sim_first_order_frac : 純粋遅延 + 一次遅れシミュレーション
- fit_first_order_delay                 : 遅延グリッド × log-τ 同定 (+ 任意スケール/frac)
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter


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
