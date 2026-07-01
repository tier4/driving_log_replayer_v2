"""実機ログからの車両モデル物理妥当性同定 (縦方向 / 操舵 / 横方向 k_us).

`physical_validity_report.py`（スタンドアロン検証スクリプト）の数値計算ロジックを
per-dataset 解析 (step6_analyze_cases) / collection 横断解析 (step13_cross_dataset)
の双方から再利用できる形に抽出したもの。plotly 描画は含まない (`lib._figures._physical_validity`
が担当)。ROS 非依存の関数と、MCAP を読む関数 (`lib._io` 経由) が混在するため、
notebook (rclpy 無し kernel) からの利用は個別関数の依存を確認すること。

同定モデル:
    縦方向: da/dt = (a_cmd(t-T) - a_corr) / tau,  a_corr = a_act - g*sin(pitch)
             (a_act は速度の運動学的微分で重力分力を内包するため、pitch から重力分力を
              引いた a_corr をフィット対象にする。二重計上防止のため a_act 自体には加算しない)
    操舵:   ddelta/dt = (delta_cmd(t-T) - delta_act) / tau
    横方向: tan(steer_eff) = (L/v + k_us(v)*v) * wz  (原点回帰、k_us は速度ビン別)

collection 横断の k_us(v) は十分統計量 (sum_wz2, sum_wz_ts) の加算プールで再構成できる
(原点回帰の正規方程式が線形加算的なため、生サンプル再読込は不要)。縦方向の横断フィットは
非線形遅延グリッドサーチのため十分統計量に還元できず、n_dyn 上位データセットのみ MCAP を
再読込する (`fit_long_cross_dataset_bounded`)。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter

from ._io import load_accel, load_cmd, load_kinematic, load_steering, load_velocity
from ._kus_profile import VX_EDGES, _kus_band_label, _kus_step_profile  # noqa: F401  (re-export)
from ._params_utils import load_sim_params

CMD_TOPIC = "/control/command/control_cmd"

# ---------------------------------------------------------------------------
# 定数 (physical_validity_report.py / identify_long_dynamics.py / identify_steer_dynamics.py と同値)
# ---------------------------------------------------------------------------
WHEELBASE = 4.76012   # [m]
STEER_BIAS = 0.0005   # [rad]
VX_MIN_CURVE = 1.5    # [m/s]
WZ_MIN = 0.02         # [rad/s]
DWZ_MAX = 0.30        # [rad/s²]
K_US_CLIP = 0.5

_FIT_DT = 0.01          # モデルフィット用リサンプリング DT [s]
_N_DYN_MIN = 100        # 動的フィット対象の最小サンプル数
_MIN_SPAN_NS = 2e9      # タイムスパン下限 [ns] (2 秒)
_GRAVITY = 9.81         # 重力加速度 [m/s²]

_FIT_N_DATASET = 2      # 横断 best/worst 表示に使う代表データセット数 (最良・最悪それぞれ)
_N_CROSS_FIT_DATASET = 10   # 縦方向横断最小二乗法に使うデータセット数 (n_dyn 上位)


def merged_model_params(model_params: dict) -> dict:
    """モデルの明示 params を load_sim_params() の既定値にマージする。

    `lib._cases_config` の schema docstring 通り、Conditions.models.<name>.params は
    「base に上書きする差分」であって完全なパラメータ集合ではない (例: acc_time_constant のみ
    指定し acc_time_delay は既定値に委ねるケースがある)。マージ無しで `.get("acc_time_delay")`
    すると該当モデルが None 判定で overlay から黙って脱落するため、必ずここでマージしてから使う。
    """
    return {**load_sim_params(), **model_params}

# 縦方向 (identify_long_dynamics.py と同値)
_DA_THRESH_FIT = 0.15
_VX_MIN_FIT = 0.5
_DELAY_CANDIDATES_LONG = np.arange(0.0, 0.31 + 1e-9, 0.01)
_TAU_BOUNDS_LONG = (0.01, 5.0)

# 操舵 (identify_steer_dynamics.py と同値)
_DSTEER_MIN = 0.001
_DELAY_CANDIDATES_STEER = np.arange(0.0, 0.15 + 1e-9, 0.01)
_TAU_BOUNDS_STEER = (0.01, 2.0)


# ---------------------------------------------------------------------------
# 共通ヘルパー
# ---------------------------------------------------------------------------
def _sim_first_order(cmd: np.ndarray, tau: float, n_delay: int, dt: float = _FIT_DT) -> np.ndarray:
    """純粋遅延 + 一次遅れシミュレーション (lfilter 版)。"""
    if n_delay > 0:
        n = len(cmd)
        cmd_del = np.empty(n)
        cmd_del[:n_delay] = cmd[0]
        cmd_del[n_delay:] = cmd[:-n_delay]
    else:
        cmd_del = cmd.copy()
    # tau<=0 は "no_delay" 系モデル等が意図する瞬時追従 (一次遅れ極限) を表すため、
    # 0除算を避けて alpha=1.0 (即時追従) にフォールバックする。
    alpha = 1.0 if tau <= 0.0 else float(np.clip(dt / tau, 0.0, 1.0))
    return lfilter([alpha], [1.0, -(1.0 - alpha)], cmd_del)


def _common_timebase(*dfs: pd.DataFrame, min_span_ns: float = 0.0) -> tuple[np.ndarray, float] | None:
    """複数 df の共通タイムベース [s] と t0 [ns] を返す。span 不足時は None。"""
    t0 = float(max(d["t_ns"].iloc[0] for d in dfs))
    t1 = float(min(d["t_ns"].iloc[-1] for d in dfs))
    if (t1 - t0) < min_span_ns:
        return None
    t_ns = np.arange(t0, t1, _FIT_DT * 1e9, dtype=np.float64)
    t_s = (t_ns - t0) * 1e-9
    return t_s, t0


def _resample(df: pd.DataFrame, col: str, t_s: np.ndarray, t0: float) -> np.ndarray:
    """df[col] を共通タイムグリッド t_s へ線形補間する。"""
    return np.interp(t_s, (df["t_ns"].values - t0) * 1e-9, df[col].values)


def _mask_stopped(arr: np.ndarray, moving: np.ndarray) -> list:
    """停車中サンプルを NaN でマスクしたリストを返す (moving=True のみ有効)。"""
    out = np.where(moving, arr.astype(float), np.nan)
    return out.tolist()


def _slope_acc_on_grid(bag_path: Path, t_s: np.ndarray, t0: float) -> np.ndarray:
    """MCAP から pitch を読み込み、勾配加速度 (g*sin(pitch)) をグリッドに補間して返す。

    読み込み失敗・pitch 列欠如時はゼロ配列を返す。
    """
    slope_acc = np.zeros_like(t_s)
    try:
        df_kin = load_kinematic(bag_path)
        if not df_kin.empty and "pitch" in df_kin.columns:
            slope_acc = np.interp(
                t_s, (df_kin["t_ns"].values - t0) * 1e-9,
                _GRAVITY * np.sin(df_kin["pitch"].values),
                left=0.0, right=0.0,
            )
    except Exception:
        pass
    return slope_acc


def _pitch_range(bag_path: Path) -> tuple[float, float]:
    """MCAP の pitch 範囲 [rad] を返す (取得失敗時は (0.0, 0.0))。"""
    try:
        df_kin = load_kinematic(bag_path)
        if not df_kin.empty and "pitch" in df_kin.columns:
            pitch = df_kin["pitch"].to_numpy()
            if len(pitch):
                return float(np.min(pitch)), float(np.max(pitch))
    except Exception:
        pass
    return 0.0, 0.0


# ---------------------------------------------------------------------------
# 縦方向: 単一データセット同定 (路面勾配補正込み)
# ---------------------------------------------------------------------------
def fit_long_single(bag_path: Path) -> dict | None:
    """縦方向一次遅れモデルの単一データセット同定 (路面勾配補正込み)。

    Returns: {"tau", "delay", "rmse_mps2", "n_dyn", "pitch_min", "pitch_max"} | None
    """
    try:
        df_cmd = load_cmd(bag_path, CMD_TOPIC)
        df_accel = load_accel(bag_path)
        df_vel = load_velocity(bag_path)
    except Exception:
        return None
    if df_cmd.empty or df_accel.empty or df_vel.empty:
        return None

    timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
    a_act_arr = _resample(df_accel, "accel", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    slope_acc_arr = _slope_acc_on_grid(bag_path, t_s, t0)
    a_act_corr = a_act_arr - slope_acc_arr

    d_cmd = np.abs(np.gradient(a_cmd_arr, _FIT_DT))
    mask_dyn = (vx > _VX_MIN_FIT) & (d_cmd > _DA_THRESH_FIT)
    if mask_dyn.sum() < 50:
        return None

    def _mse(log_tau: float, n_delay: int) -> float:
        tau = float(np.exp(log_tau))
        a_sim = _sim_first_order(a_cmd_arr, tau, n_delay)
        diff = a_sim[mask_dyn] - a_act_corr[mask_dyn]
        return float(np.mean(diff ** 2))

    log_lo, log_hi = np.log(_TAU_BOUNDS_LONG[0]), np.log(_TAU_BOUNDS_LONG[1])
    best_mse, best_tau, best_delay = float("inf"), float("nan"), float("nan")
    for delay_s in _DELAY_CANDIDATES_LONG:
        n_delay = int(round(delay_s / _FIT_DT))
        res = minimize_scalar(
            lambda lt, nd=n_delay: _mse(lt, nd), bounds=(log_lo, log_hi), method="bounded",
        )
        if res.fun < best_mse:
            best_mse, best_tau, best_delay = res.fun, float(np.exp(res.x)), delay_s

    if np.isnan(best_tau):
        return None

    pitch_min, pitch_max = _pitch_range(bag_path)
    return {
        "tau": best_tau,
        "delay": best_delay,
        "rmse_mps2": float(np.sqrt(best_mse)),
        "n_dyn": int(mask_dyn.sum()),
        "pitch_min": pitch_min,
        "pitch_max": pitch_max,
    }


# ---------------------------------------------------------------------------
# 操舵: 単一データセット同定
# ---------------------------------------------------------------------------
def fit_steer_single(bag_path: Path) -> dict | None:
    """操舵一次遅れモデルの単一データセット同定。

    Returns: {"tau", "delay", "rmse_mrad", "n_dyn"} | None
    """
    try:
        df_cmd = load_cmd(bag_path, CMD_TOPIC)
        df_steer = load_steering(bag_path)
        df_vel = load_velocity(bag_path)
    except Exception:
        return None
    if df_cmd.empty or df_steer.empty or df_vel.empty:
        return None

    timebase = _common_timebase(df_cmd, df_steer, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    d_cmd = _resample(df_cmd, "cmd_steer", t_s, t0)
    d_act = _resample(df_steer, "steer", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)

    dd_cmd = np.abs(np.gradient(d_cmd, _FIT_DT))
    mask_dyn = (vx > _VX_MIN_FIT) & (dd_cmd > _DSTEER_MIN / _FIT_DT)
    if mask_dyn.sum() < 50:
        return None

    def _mse(log_tau: float, n_delay: int) -> float:
        tau = float(np.exp(log_tau))
        d_sim = _sim_first_order(d_cmd, tau, n_delay)
        diff = d_sim[mask_dyn] - d_act[mask_dyn]
        return float(np.mean(diff ** 2))

    log_lo, log_hi = np.log(_TAU_BOUNDS_STEER[0]), np.log(_TAU_BOUNDS_STEER[1])
    best_mse, best_tau, best_delay = float("inf"), float("nan"), float("nan")
    for delay_s in _DELAY_CANDIDATES_STEER:
        n_delay = int(round(delay_s / _FIT_DT))
        res = minimize_scalar(
            lambda lt, nd=n_delay: _mse(lt, nd), bounds=(log_lo, log_hi), method="bounded",
        )
        if res.fun < best_mse:
            best_mse, best_tau, best_delay = res.fun, float(np.exp(res.x)), delay_s

    if np.isnan(best_tau):
        return None

    return {
        "tau": best_tau,
        "delay": best_delay,
        "rmse_mrad": float(np.sqrt(best_mse)) * 1000.0,
        "n_dyn": int(mask_dyn.sum()),
    }


# ---------------------------------------------------------------------------
# 横方向 k_us(v): 速度ビン別最小二乗法 (十分統計量つき、単一/複数データセット両対応)
# ---------------------------------------------------------------------------
def _extract_kus_arrays(bag_path: Path) -> dict | None:
    """k_us 分析用の vx/wz/steer_eff/dwz を単一データセットから抽出する。"""
    try:
        df_kin = load_kinematic(bag_path)
        df_steer = load_steering(bag_path)
    except Exception:
        return None
    if df_kin.empty or df_steer.empty or len(df_kin) < 10:
        return None

    t_k = df_kin["t_ns"].values * 1e-9
    vx = df_kin["vx"].values
    wz = df_kin["wz"].values

    steer_raw = df_steer["steer"].values
    t_s = df_steer["t_ns"].values * 1e-9
    steer = np.interp(t_k, t_s, steer_raw)
    steer_eff = steer - STEER_BIAS

    dt = np.diff(t_k)
    dt_safe = np.where(dt > 0, dt, 1e-3)
    dwz = np.zeros_like(wz)
    if len(dt_safe) > 0:
        dwz_mid = np.diff(wz) / dt_safe
        if len(dwz_mid) > 0:
            dwz[0] = dwz_mid[0]
            dwz[-1] = dwz_mid[-1]
            dwz[1:-1] = 0.5 * (dwz_mid[:-1] + dwz_mid[1:])

    return {"vx": vx, "wz": wz, "steer_eff": steer_eff, "dwz": dwz}


def compute_kus_bins(records: list[dict]) -> dict:
    """速度ビン別 最小二乗法回帰で k_us(v) を推定する。

    モデル: tan(δ_eff) = (L/v + k_us·v)·ω (原点回帰)。
    records は `_extract_kus_arrays` の返り値の list (単一データセットなら 1 要素)。
    十分統計量 (sum_wz2/sum_wz_ts) を含めるため、collection 横断側で生サンプルなしに
    加算的にプール再構成できる (`compute_kus_bins_from_sufficient_stats`)。
    """
    all_vx = np.concatenate([r["vx"] for r in records])
    all_wz = np.concatenate([r["wz"] for r in records])
    all_steer_eff = np.concatenate([r["steer_eff"] for r in records])
    all_dwz = np.concatenate([r["dwz"] for r in records])

    mask_ok = (
        (np.abs(all_wz) > WZ_MIN)
        & (np.abs(all_dwz) < DWZ_MAX)
        & (all_vx > VX_MIN_CURVE)
    )
    vx_f = all_vx[mask_ok]
    wz_f = all_wz[mask_ok]
    steer_f = all_steer_eff[mask_ok]
    tan_steer = np.tan(np.clip(steer_f, -0.8, 0.8))

    n_bins = len(VX_EDGES) - 1
    vx_mid = np.empty(n_bins)
    kus_ols = np.full(n_bins, np.nan)
    kus_p25 = np.full(n_bins, np.nan)
    kus_p75 = np.full(n_bins, np.nan)
    n_pts = np.zeros(n_bins, dtype=int)
    sum_wz2 = np.zeros(n_bins)
    sum_wz_ts = np.zeros(n_bins)

    for i in range(n_bins):
        lo, hi = VX_EDGES[i], VX_EDGES[i + 1]
        mask_bin = (vx_f >= lo) & (vx_f < hi)
        n = int(mask_bin.sum())
        n_pts[i] = n
        vx_mid[i] = (lo + hi) / 2
        if n < 10:
            continue
        vx_b = vx_f[mask_bin]
        wz_b = wz_f[mask_bin]
        ts_b = tan_steer[mask_bin]
        vm = float(np.median(vx_b))
        vx_mid[i] = vm
        sum_wz2[i] = float(np.sum(wz_b ** 2))
        sum_wz_ts[i] = float(np.sum(wz_b * ts_b))
        # 最小二乗法原点回帰: tan(steer) = C * wz => k_us = (C - L/v) / v
        C_ols = sum_wz_ts[i] / sum_wz2[i] if sum_wz2[i] > 0 else float("nan")
        kus_ols[i] = (C_ols - WHEELBASE / vm) / vm
        # 個別サンプル percentile (外れ値確認用)
        with np.errstate(divide="ignore", invalid="ignore"):
            kus_each = (
                ts_b / np.where(np.abs(wz_b) > 1e-6, wz_b, np.nan) - WHEELBASE / vx_b
            ) / vx_b
        kus_each = kus_each[np.isfinite(kus_each)]
        kus_each = np.clip(kus_each, -K_US_CLIP, K_US_CLIP)
        if len(kus_each) > 10:
            kus_p25[i] = float(np.percentile(kus_each, 25))
            kus_p75[i] = float(np.percentile(kus_each, 75))

    return {
        "vx_mid": vx_mid,
        "kus_ols": kus_ols,
        "kus_p25": kus_p25,
        "kus_p75": kus_p75,
        "n_pts": n_pts,
        "sum_wz2": sum_wz2,
        "sum_wz_ts": sum_wz_ts,
    }


def compute_kus_bins_single(bag_path: Path) -> dict | None:
    """単一データセットの k_us(v) 速度ビン別回帰 (十分統計量つき)。"""
    rec = _extract_kus_arrays(bag_path)
    if rec is None:
        return None
    return compute_kus_bins([rec])


def compute_kus_bins_from_sufficient_stats(per_ds_bins: list[dict]) -> dict:
    """複数データセットの十分統計量 (sum_wz2/sum_wz_ts) を加算的にプールし k_us(v) を再構成する。

    原点回帰 tan(steer)=C*wz の正規方程式 C=Σ(wz·ts)/Σ(wz²) は加算的に成り立つため、
    生サンプルの再読込なしに全データセット結合と同一の最小二乗解が得られる。
    IQR (p25/p75) は個別サンプル分布に依存し十分統計量から再構成できないため省略する
    (NaN のまま返す。描画側は IQR バンドを描かない)。
    """
    n_bins = len(VX_EDGES) - 1
    if not per_ds_bins:
        return {
            "vx_mid": (VX_EDGES[:-1] + VX_EDGES[1:]) / 2,
            "kus_ols": np.full(n_bins, np.nan),
            "kus_p25": np.full(n_bins, np.nan),
            "kus_p75": np.full(n_bins, np.nan),
            "n_pts": np.zeros(n_bins, dtype=int),
        }

    sum_wz2 = np.zeros(n_bins)
    sum_wz_ts = np.zeros(n_bins)
    n_pts = np.zeros(n_bins, dtype=int)
    weighted_vx_mid = np.zeros(n_bins)
    for b in per_ds_bins:
        b_n_pts = np.asarray(b["n_pts"], dtype=int)
        sum_wz2 += np.nan_to_num(np.asarray(b["sum_wz2"], dtype=float))
        sum_wz_ts += np.nan_to_num(np.asarray(b["sum_wz_ts"], dtype=float))
        weighted_vx_mid += b_n_pts * np.nan_to_num(np.asarray(b["vx_mid"], dtype=float))
        n_pts += b_n_pts

    bin_center = (VX_EDGES[:-1] + VX_EDGES[1:]) / 2
    with np.errstate(invalid="ignore", divide="ignore"):
        vx_mid = np.where(n_pts > 0, weighted_vx_mid / np.maximum(n_pts, 1), bin_center)

    kus_ols = np.full(n_bins, np.nan)
    for i in range(n_bins):
        if n_pts[i] < 10 or sum_wz2[i] <= 0:
            continue
        C_ols = sum_wz_ts[i] / sum_wz2[i]
        kus_ols[i] = (C_ols - WHEELBASE / vx_mid[i]) / vx_mid[i]

    return {
        "vx_mid": vx_mid,
        "kus_ols": kus_ols,
        "kus_p25": np.full(n_bins, np.nan),
        "kus_p75": np.full(n_bins, np.nan),
        "n_pts": n_pts,
    }


# ---------------------------------------------------------------------------
# 時系列 (実測/指令/フィット/チューン値) — per-dataset 図・collection 横断図の共通計算
# ---------------------------------------------------------------------------
def compute_long_timeseries(bag_path: Path, fit: dict | None, models: dict[str, dict]) -> dict | None:
    """1 データセットの縦方向時系列 (実測/指令/フィット/モデル別チューン値、路面勾配補正込み) を計算する。

    models: {モデル名: raw params (case.params、base 未マージ)}。base マージは内部で行う。
    """
    try:
        df_cmd = load_cmd(bag_path, CMD_TOPIC)
        df_accel = load_accel(bag_path)
        df_vel = load_velocity(bag_path)
    except Exception:
        return None
    if df_cmd.empty or df_accel.empty or df_vel.empty:
        return None
    timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
    a_act_arr = _resample(df_accel, "accel", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    slope_acc_arr = _slope_acc_on_grid(bag_path, t_s, t0)
    moving = vx > VX_MIN_CURVE

    a_sim_fit = None
    if fit is not None and np.isfinite(fit.get("tau", float("nan"))):
        a_sim_fit = _sim_first_order(
            a_cmd_arr, fit["tau"], int(round(fit["delay"] / _FIT_DT))
        ) + slope_acc_arr

    a_sim_models: dict[str, list] = {}
    model_tune: dict[str, dict] = {}
    for name, raw_params in (models or {}).items():
        merged = merged_model_params(raw_params)
        tau_tune = merged.get("acc_time_constant")
        T_tune = merged.get("acc_time_delay")
        if tau_tune is None or T_tune is None:
            continue
        sim = _sim_first_order(
            a_cmd_arr, float(tau_tune), int(round(float(T_tune) / _FIT_DT))
        ) + slope_acc_arr
        a_sim_models[name] = _mask_stopped(sim, moving)
        model_tune[name] = {"tau": float(tau_tune), "T": float(T_tune)}

    return {
        "t": t_s.tolist(),
        "a_cmd": _mask_stopped(a_cmd_arr, moving),
        "a_act": _mask_stopped(a_act_arr, moving),
        "a_sim_fit": _mask_stopped(a_sim_fit, moving) if a_sim_fit is not None else None,
        "a_sim_models": a_sim_models,
        "model_tune": model_tune,
    }


def compute_steer_timeseries(bag_path: Path, fit: dict | None, models: dict[str, dict]) -> dict | None:
    """1 データセットの操舵時系列 (実測/フィット/モデル別チューン値) を計算する。

    models: {モデル名: raw params (case.params、base 未マージ)}。base マージは内部で行う。
    """
    try:
        df_cmd = load_cmd(bag_path, CMD_TOPIC)
        df_steer = load_steering(bag_path)
        df_vel = load_velocity(bag_path)
    except Exception:
        return None
    if df_cmd.empty or df_steer.empty or df_vel.empty:
        return None
    timebase = _common_timebase(df_cmd, df_steer, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    d_cmd = _resample(df_cmd, "cmd_steer", t_s, t0)
    d_act = _resample(df_steer, "steer", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    moving = vx > VX_MIN_CURVE

    d_sim_fit = None
    if fit is not None and np.isfinite(fit.get("tau", float("nan"))):
        d_sim_fit = _sim_first_order(d_cmd, fit["tau"], int(round(fit["delay"] / _FIT_DT)))

    d_sim_models: dict[str, list] = {}
    model_tune: dict[str, dict] = {}
    for name, raw_params in (models or {}).items():
        merged = merged_model_params(raw_params)
        tau_tune = merged.get("steer_time_constant")
        T_tune = merged.get("steer_time_delay")
        if tau_tune is None or T_tune is None:
            continue
        sim = _sim_first_order(d_cmd, float(tau_tune), int(round(float(T_tune) / _FIT_DT)))
        d_sim_models[name] = _mask_stopped(sim, moving)
        model_tune[name] = {"tau": float(tau_tune), "T": float(T_tune)}

    return {
        "t": t_s.tolist(),
        "d_act": _mask_stopped(d_act, moving),
        "d_sim_fit": _mask_stopped(d_sim_fit, moving) if d_sim_fit is not None else None,
        "d_sim_models": d_sim_models,
        "model_tune": model_tune,
    }


# ---------------------------------------------------------------------------
# collection 横断: best/worst データセット選定・縦方向横断フィット (有界 MCAP 再読込)
# ---------------------------------------------------------------------------
def pick_best_worst_entries(
    entries: list, per_ds_fit: dict[str, dict], rmse_key: str, n: int = _FIT_N_DATASET,
) -> list[tuple]:
    """rmse_key (rmse_mps2/rmse_mrad) でソートし最良・最悪 n 件の (entry, "最良"|"最悪") を返す。

    n_dyn >= _N_DYN_MIN の候補が 2n 未満なら全データセットにフォールバックする。
    """
    candidates = [
        (e, per_ds_fit[e.dataset_id]) for e in entries
        if e.dataset_id in per_ds_fit and per_ds_fit[e.dataset_id].get(rmse_key) is not None
        and np.isfinite(per_ds_fit[e.dataset_id][rmse_key])
    ]
    ok = [c for c in candidates if c[1].get("n_dyn", 0) >= _N_DYN_MIN]
    if len(ok) < n * 2:
        ok = candidates
    ok_sorted = sorted(ok, key=lambda c: c[1][rmse_key])
    best = ok_sorted[:n]
    best_ids = {e.dataset_id for e, _ in best}
    worst = [c for c in ok_sorted[::-1][:n] if c[0].dataset_id not in best_ids]
    return [(e, "最良") for e, _ in best] + [(e, "最悪") for e, _ in worst]


def fit_long_cross_dataset_bounded(
    entries: list, per_ds_long: dict[str, dict], n_top: int = _N_CROSS_FIT_DATASET,
) -> dict:
    """縦方向一次遅れモデルの横断最小二乗法同定 (n_dyn 上位 n_top データセットのみ MCAP 再読込)。

    十分統計量に還元できない非線形遅延グリッドサーチのため、MCAP 再読込を明示的に
    上位 n_top 件へ限定する (collection 横断解析における JSON-only 集計の唯一の例外)。
    """
    ranked = sorted(
        (e for e in entries if e.real_lite is not None and e.dataset_id in per_ds_long),
        key=lambda e: per_ds_long[e.dataset_id].get("n_dyn", 0),
        reverse=True,
    )[:n_top]

    pooled: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    all_pitch: list[float] = []
    for e in ranked:
        try:
            df_cmd = load_cmd(e.real_lite, CMD_TOPIC)
            df_accel = load_accel(e.real_lite)
            df_vel = load_velocity(e.real_lite)
        except Exception:
            continue
        if df_cmd.empty or df_accel.empty or df_vel.empty:
            continue
        timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
        a_act_arr = _resample(df_accel, "accel", t_s, t0)
        vx = _resample(df_vel, "lon_vel", t_s, t0)
        slope_acc_arr = _slope_acc_on_grid(e.real_lite, t_s, t0)

        p_min, p_max = _pitch_range(e.real_lite)
        if p_min != 0.0 or p_max != 0.0:
            all_pitch.extend([p_min, p_max])

        a_act_corr_arr = a_act_arr - slope_acc_arr
        d_cmd = np.abs(np.gradient(a_cmd_arr, _FIT_DT))
        mask = (vx > _VX_MIN_FIT) & (d_cmd > _DA_THRESH_FIT)
        if mask.sum() < 50:
            continue
        pooled.append((a_cmd_arr, a_act_corr_arr, mask))

    pitch_min = float(min(all_pitch)) if all_pitch else 0.0
    pitch_max = float(max(all_pitch)) if all_pitch else 0.0
    if not pooled:
        return {
            "tau": float("nan"), "delay": float("nan"), "rmse_mps2": float("nan"),
            "pitch_min": pitch_min, "pitch_max": pitch_max, "n_datasets": 0,
        }

    def _total_mse(log_tau: float, n_delay: int) -> float:
        tau = float(np.exp(log_tau))
        sq_sum, n_sum = 0.0, 0
        for a_cmd_arr, a_act_corr_arr, mask in pooled:
            a_sim = _sim_first_order(a_cmd_arr, tau, n_delay)
            diff = a_sim[mask] - a_act_corr_arr[mask]
            sq_sum += float(np.dot(diff, diff))
            n_sum += int(mask.sum())
        return sq_sum / n_sum if n_sum > 0 else float("inf")

    log_lo, log_hi = np.log(_TAU_BOUNDS_LONG[0]), np.log(_TAU_BOUNDS_LONG[1])
    best_mse, best_tau, best_delay = float("inf"), float("nan"), float("nan")
    for delay_s in _DELAY_CANDIDATES_LONG:
        n_delay = int(round(delay_s / _FIT_DT))
        res = minimize_scalar(
            lambda lt, nd=n_delay: _total_mse(lt, nd), bounds=(log_lo, log_hi), method="bounded",
        )
        if res.fun < best_mse:
            best_mse, best_tau, best_delay = res.fun, float(np.exp(res.x)), delay_s

    return {
        "tau": best_tau, "delay": best_delay, "rmse_mps2": float(np.sqrt(best_mse)),
        "pitch_min": pitch_min, "pitch_max": pitch_max, "n_datasets": len(pooled),
    }


def compute_cross_long_rows(
    entries: list, per_ds_long: dict[str, dict], cross_fit: dict, models: dict | None,
) -> list[dict]:
    """best/worst データセットの縦方向時系列 (横断フィット + モデル別チューン値重ね描き用)。"""
    picks = pick_best_worst_entries(entries, per_ds_long, "rmse_mps2")
    rows: list[dict] = []
    for entry, case_tag in picks:
        if entry.real_lite is None:
            continue
        try:
            df_cmd = load_cmd(entry.real_lite, CMD_TOPIC)
            df_accel = load_accel(entry.real_lite)
            df_vel = load_velocity(entry.real_lite)
        except Exception:
            continue
        if df_cmd.empty or df_accel.empty or df_vel.empty:
            continue
        timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
        a_act_arr = _resample(df_accel, "accel", t_s, t0)
        vx = _resample(df_vel, "lon_vel", t_s, t0)
        slope_acc_arr = _slope_acc_on_grid(entry.real_lite, t_s, t0)
        moving = vx > VX_MIN_CURVE

        a_sim_cross = None
        if np.isfinite(cross_fit.get("tau", float("nan"))):
            a_sim_cross = _sim_first_order(
                a_cmd_arr, cross_fit["tau"], int(round(cross_fit["delay"] / _FIT_DT))
            ) + slope_acc_arr

        a_sim_models: dict[str, list] = {}
        for name, spec in (models or {}).items():
            merged = merged_model_params(spec.params)
            tau = merged.get("acc_time_constant")
            delay = merged.get("acc_time_delay")
            if tau is None or delay is None:
                continue
            sim = _sim_first_order(a_cmd_arr, float(tau), int(round(float(delay) / _FIT_DT))) + slope_acc_arr
            a_sim_models[name] = _mask_stopped(sim, moving)

        fit = per_ds_long.get(entry.dataset_id, {})
        rmse = fit.get("rmse_mps2", float("nan"))
        rows.append({
            "label": f"[{case_tag}] {entry.dataset_id[:8]}  RMSE={rmse:.3f} m/s²",
            "t": t_s.tolist(),
            "a_cmd": _mask_stopped(a_cmd_arr, moving),
            "a_act": _mask_stopped(a_act_arr, moving),
            "a_sim_cross": _mask_stopped(a_sim_cross, moving) if a_sim_cross is not None else None,
            "a_sim_models": a_sim_models,
        })
    return rows


def compute_cross_steer_rows(
    entries: list, per_ds_steer: dict[str, dict], models: dict | None,
) -> list[dict]:
    """best/worst データセットの操舵時系列 (per-dataset フィット + モデル別チューン値重ね描き用)。"""
    picks = pick_best_worst_entries(entries, per_ds_steer, "rmse_mrad")
    rows: list[dict] = []
    for entry, case_tag in picks:
        if entry.real_lite is None:
            continue
        try:
            df_cmd = load_cmd(entry.real_lite, CMD_TOPIC)
            df_steer = load_steering(entry.real_lite)
            df_vel = load_velocity(entry.real_lite)
        except Exception:
            continue
        if df_cmd.empty or df_steer.empty or df_vel.empty:
            continue
        timebase = _common_timebase(df_cmd, df_steer, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        d_cmd = _resample(df_cmd, "cmd_steer", t_s, t0)
        d_act = _resample(df_steer, "steer", t_s, t0)
        vx = _resample(df_vel, "lon_vel", t_s, t0)
        moving = vx > VX_MIN_CURVE

        fit = per_ds_steer.get(entry.dataset_id, {})
        d_sim_fit = None
        if np.isfinite(fit.get("tau", float("nan"))):
            d_sim_fit = _sim_first_order(d_cmd, fit["tau"], int(round(fit["delay"] / _FIT_DT)))

        d_sim_models: dict[str, list] = {}
        for name, spec in (models or {}).items():
            merged = merged_model_params(spec.params)
            tau = merged.get("steer_time_constant")
            delay = merged.get("steer_time_delay")
            if tau is None or delay is None:
                continue
            sim = _sim_first_order(d_cmd, float(tau), int(round(float(delay) / _FIT_DT)))
            d_sim_models[name] = _mask_stopped(sim, moving)

        rmse = fit.get("rmse_mrad", float("nan"))
        rows.append({
            "label": f"[{case_tag}] {entry.dataset_id[:8]}  RMSE={rmse:.1f} mrad",
            "t": t_s.tolist(),
            "d_act": _mask_stopped(d_act, moving),
            "d_sim_fit": _mask_stopped(d_sim_fit, moving) if d_sim_fit is not None else None,
            "d_sim_models": d_sim_models,
        })
    return rows
