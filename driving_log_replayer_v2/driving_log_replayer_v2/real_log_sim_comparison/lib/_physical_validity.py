"""実機ログからの車両モデル物理妥当性同定 (縦方向 / 操舵 / 横方向 k_us).

数値計算ロジックと物理定数の SSOT (single source of truth)。
`physical_validity_report.py`（スタンドアロン検証スクリプト）、per-dataset 解析
(step6_analyze_cases)、collection 横断解析 (step13_cross_dataset) はいずれも
本モジュールから import する。plotly 描画は含まない (`lib._figures._physical_validity`
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

from ._io import (
    load_accel,
    load_cmd,
    load_gear_status,
    load_kinematic,
    load_steering,
    load_velocity,
    require_drive_gear_mask,
)
from ._kus_profile import VX_EDGES, _kus_band_label, _kus_step_profile  # noqa: F401  (re-export)
from ._params_utils import load_sim_params
from ._validation import require_non_empty_df

CMD_TOPIC = "/control/command/control_cmd"

# ---------------------------------------------------------------------------
# 定数 (本モジュールが SSOT: single source of truth。physical_validity_report.py 等は
#       ここから import する。値の由来は旧同定スクリプト identify_long_dynamics.py /
#       identify_steer_dynamics.py の同定手順 — 削除済みのため git 履歴を参照)
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

    `lib._models_config` の schema docstring 通り、Conditions.models.<name>.params は
    「base に上書きする差分」であって完全なパラメータ集合ではない (例: acc_time_constant のみ
    指定し acc_time_delay は既定値に委ねるケースがある)。マージ無しで `.get("acc_time_delay")`
    すると該当モデルが None 判定で overlay から黙って脱落するため、必ずここでマージしてから使う。
    """
    return {**load_sim_params(), **model_params}

# 縦方向 (SSOT は本モジュール。由来は旧 identify_long_dynamics.py の同定手順、git 履歴参照)
_DA_THRESH_FIT = 0.15
_VX_MIN_FIT = 0.5
_DELAY_CANDIDATES_LONG = np.arange(0.0, 0.31 + 1e-9, 0.01)
_TAU_BOUNDS_LONG = (0.01, 5.0)

# 操舵 (SSOT は本モジュール。由来は旧 identify_steer_dynamics.py の同定手順、git 履歴参照)
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


def _drive_mask_on_grid(bag_path: Path, t_s: np.ndarray, t0: float, *, context: str) -> np.ndarray:
    """共通時間グリッド上で DRIVE 系 gear の mask を返す。gear_status 欠落はエラー。"""
    df_gear = load_gear_status(bag_path)
    target_t_ns = (t0 + t_s * 1e9).astype(np.int64)
    return require_drive_gear_mask(df_gear, target_t_ns, context=context)


def _require_dfs(context: str, **dfs: pd.DataFrame) -> None:
    """必須 DataFrame 群が空でないことをまとめて検証する。"""
    for name, df in dfs.items():
        require_non_empty_df(df, name=name, context=context)


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
    _require_dfs(
        f"fit_long_single:{bag_path}",
        control_cmd=df_cmd,
        localization_acceleration=df_accel,
        velocity_status=df_vel,
    )

    timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
    a_act_arr = _resample(df_accel, "accel", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    gear_drive = _drive_mask_on_grid(bag_path, t_s, t0, context=f"fit_long_single:{bag_path}")
    slope_acc_arr = _slope_acc_on_grid(bag_path, t_s, t0)
    a_act_corr = a_act_arr - slope_acc_arr

    d_cmd = np.abs(np.gradient(a_cmd_arr, _FIT_DT))
    mask_dyn = gear_drive & (vx > _VX_MIN_FIT) & (d_cmd > _DA_THRESH_FIT)
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
    _require_dfs(
        f"fit_steer_single:{bag_path}",
        control_cmd=df_cmd,
        steering_status=df_steer,
        velocity_status=df_vel,
    )

    timebase = _common_timebase(df_cmd, df_steer, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    d_cmd = _resample(df_cmd, "cmd_steer", t_s, t0)
    d_act = _resample(df_steer, "steer", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    gear_drive = _drive_mask_on_grid(bag_path, t_s, t0, context=f"fit_steer_single:{bag_path}")

    dd_cmd = np.abs(np.gradient(d_cmd, _FIT_DT))
    mask_dyn = gear_drive & (vx > _VX_MIN_FIT) & (dd_cmd > _DSTEER_MIN / _FIT_DT)
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
    """k_us 分析用の vx/wz/steer_eff/dwz を単一データセットから抽出する。

    付随情報として kinematic 時刻 t [s] と yaw [rad] も返す
    (physical_validity_report の worker がカーブカバレッジ・cmd_steer 間引きに使う。
    `compute_kus_bins` は vx/wz/steer_eff/dwz のみ参照し、余剰キーは無視される)。
    """
    try:
        df_kin = load_kinematic(bag_path)
        df_steer = load_steering(bag_path)
    except Exception:
        return None
    _require_dfs(
        f"_extract_kus_arrays:{bag_path}",
        kinematic_state=df_kin,
        steering_status=df_steer,
    )
    if len(df_kin) < 10:
        return None

    t_k = df_kin["t_ns"].values * 1e-9
    vx = df_kin["vx"].values
    wz = df_kin["wz"].values
    gear_drive = require_drive_gear_mask(
        load_gear_status(bag_path),
        df_kin["t_ns"].values,
        context=f"_extract_kus_arrays:{bag_path}",
    )

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

    return {
        "vx": vx,
        "wz": wz,
        "steer_eff": steer_eff,
        "dwz": dwz,
        "gear_drive": gear_drive,
        "t": t_k,
        "yaw": df_kin["yaw"].values,
    }


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
    all_gear_drive = np.concatenate([r.get("gear_drive", np.ones_like(r["vx"], dtype=bool)) for r in records])

    mask_ok = (
        all_gear_drive
        & (np.abs(all_wz) > WZ_MIN)
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
    _require_dfs(
        f"compute_long_timeseries:{bag_path}",
        control_cmd=df_cmd,
        localization_acceleration=df_accel,
        velocity_status=df_vel,
    )
    timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
    a_act_arr = _resample(df_accel, "accel", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    gear_drive = _drive_mask_on_grid(bag_path, t_s, t0, context=f"compute_long_timeseries:{bag_path}")
    slope_acc_arr = _slope_acc_on_grid(bag_path, t_s, t0)
    moving = gear_drive & (vx > VX_MIN_CURVE)

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
    _require_dfs(
        f"compute_steer_timeseries:{bag_path}",
        control_cmd=df_cmd,
        steering_status=df_steer,
        velocity_status=df_vel,
    )
    timebase = _common_timebase(df_cmd, df_steer, df_vel, min_span_ns=_MIN_SPAN_NS)
    if timebase is None:
        return None
    t_s, t0 = timebase

    d_cmd = _resample(df_cmd, "cmd_steer", t_s, t0)
    d_act = _resample(df_steer, "steer", t_s, t0)
    vx = _resample(df_vel, "lon_vel", t_s, t0)
    gear_drive = _drive_mask_on_grid(bag_path, t_s, t0, context=f"compute_steer_timeseries:{bag_path}")
    moving = gear_drive & (vx > VX_MIN_CURVE)

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
        _require_dfs(
            f"fit_long_cross_dataset_bounded:{e.dataset_id}",
            control_cmd=df_cmd,
            localization_acceleration=df_accel,
            velocity_status=df_vel,
        )
        timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
        a_act_arr = _resample(df_accel, "accel", t_s, t0)
        vx = _resample(df_vel, "lon_vel", t_s, t0)
        gear_drive = _drive_mask_on_grid(e.real_lite, t_s, t0, context=f"fit_long_cross_dataset_bounded:{e.dataset_id}")
        slope_acc_arr = _slope_acc_on_grid(e.real_lite, t_s, t0)

        p_min, p_max = _pitch_range(e.real_lite)
        if p_min != 0.0 or p_max != 0.0:
            all_pitch.extend([p_min, p_max])

        a_act_corr_arr = a_act_arr - slope_acc_arr
        d_cmd = np.abs(np.gradient(a_cmd_arr, _FIT_DT))
        mask = gear_drive & (vx > _VX_MIN_FIT) & (d_cmd > _DA_THRESH_FIT)
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
        _require_dfs(
            f"compute_cross_long_rows:{entry.dataset_id}",
            control_cmd=df_cmd,
            localization_acceleration=df_accel,
            velocity_status=df_vel,
        )
        timebase = _common_timebase(df_cmd, df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        a_cmd_arr = _resample(df_cmd, "cmd_accel", t_s, t0)
        a_act_arr = _resample(df_accel, "accel", t_s, t0)
        vx = _resample(df_vel, "lon_vel", t_s, t0)
        gear_drive = _drive_mask_on_grid(entry.real_lite, t_s, t0, context=f"compute_cross_long_rows:{entry.dataset_id}")
        slope_acc_arr = _slope_acc_on_grid(entry.real_lite, t_s, t0)
        moving = gear_drive & (vx > VX_MIN_CURVE)

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
        _require_dfs(
            f"compute_cross_steer_rows:{entry.dataset_id}",
            control_cmd=df_cmd,
            steering_status=df_steer,
            velocity_status=df_vel,
        )
        timebase = _common_timebase(df_cmd, df_steer, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        d_cmd = _resample(df_cmd, "cmd_steer", t_s, t0)
        d_act = _resample(df_steer, "steer", t_s, t0)
        vx = _resample(df_vel, "lon_vel", t_s, t0)
        gear_drive = _drive_mask_on_grid(entry.real_lite, t_s, t0, context=f"compute_cross_steer_rows:{entry.dataset_id}")
        moving = gear_drive & (vx > VX_MIN_CURVE)

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


# 理想追従評価用定数 (本モジュールが SSOT。physical_validity_report.py はここから import する)
_PERF_STRIDE = 5
_DRIFT_A_TH = 0.3


def _valid_k0s(
    gt_vx: np.ndarray,
    horizon: int,
    stride: int = _PERF_STRIDE,
) -> np.ndarray:
    """VX_MIN_CURVE フィルタ済みの rollout 開始点インデックスを返す。"""
    n = len(gt_vx)
    k0s = np.arange(0, n - horizon, stride)
    return k0s[gt_vx[k0s] > VX_MIN_CURVE]


def _bicycle_nstep_perf(
    gt_x: np.ndarray,
    gt_y: np.ndarray,
    gt_yaw: np.ndarray,
    gt_vx: np.ndarray,
    gt_steer: np.ndarray,
    params: dict,
    horizon: int,
    dt: float,
    stride: int = _PERF_STRIDE,
) -> np.ndarray:
    """純粋自転車モデルの N-step 横方向誤差（ベクトル化）。

    gt_steer: 実測操舵角 [rad]（生センサ値。steer_bias は params から適用）
    params.steer_bias / k_us(v) を C++ モデルと同一式で評価し積分する。
    Returns: |横方向誤差| [m] 配列（vx > VX_MIN_CURVE な開始点のみ）
    """
    L = WHEELBASE
    beta = float(params.get("steer_bias", 0.0))
    k_us_arr = _kus_step_profile(gt_vx, params)
    denom_arr = L + k_us_arr * gt_vx ** 2
    wz_arr = gt_vx * np.tan(gt_steer + beta) / denom_arr

    n = len(gt_x)
    k0s = _valid_k0s(gt_vx, horizon, stride)
    if len(k0s) == 0:
        return np.array([], dtype=float)

    bx   = gt_x[k0s].astype(float).copy()
    by   = gt_y[k0s].astype(float).copy()
    byaw = gt_yaw[k0s].astype(float).copy()

    for j in range(horizon):
        ki = np.clip(k0s + j, 0, n - 1)
        vxi = gt_vx[ki]
        wzi = wz_arr[ki]
        bx   = bx   + vxi * np.cos(byaw) * dt
        by   = by   + vxi * np.sin(byaw) * dt
        byaw = byaw + wzi * dt

    k_end = np.minimum(k0s + horizon, n - 1)
    dx  = bx - gt_x[k_end]
    dy  = by - gt_y[k_end]
    lat = np.abs(-dx * np.sin(gt_yaw[k_end]) + dy * np.cos(gt_yaw[k_end]))
    return lat


def _bicycle_trajectory_full(
    gt_vx: np.ndarray,
    gt_steer: np.ndarray,
    x0: float,
    y0: float,
    yaw0: float,
    params: dict,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """自転車モデルの全区間連続積分軌跡（リセットなし）。
    初期状態を GT に合わせ、実測 vx + steer で積分する（アクチュエータ遅れなし仮定）。
    """
    L = WHEELBASE
    beta = float(params.get("steer_bias", 0.0))
    k_us_arr = _kus_step_profile(gt_vx, params)
    denom_arr = L + k_us_arr * gt_vx ** 2
    wz_arr = gt_vx * np.tan(gt_steer + beta) / denom_arr

    n = len(gt_vx)
    xs = np.empty(n)
    ys = np.empty(n)
    x, y, yaw = x0, y0, yaw0
    for i in range(n):
        xs[i] = x
        ys[i] = y
        x   = x   + float(gt_vx[i]) * float(np.cos(yaw)) * dt
        y   = y   + float(gt_vx[i]) * float(np.sin(yaw)) * dt
        yaw = yaw + float(wz_arr[i]) * dt
    return xs, ys


def _long_nstep_perf(
    gt_vx: np.ndarray,
    a_act: np.ndarray,
    horizon: int,
    dt: float,
    stride: int = _PERF_STRIDE,
) -> np.ndarray:
    """縦方向モデル構造限界評価: 実測加速度 a_act を直接積分し GT 変位と比較。"""
    n = len(gt_vx)
    k0s = _valid_k0s(gt_vx, horizon, stride)
    if len(k0s) == 0:
        return np.array([], dtype=float)

    vx_sim = gt_vx[k0s].astype(float).copy()
    s_sim  = np.zeros(len(k0s))
    s_gt   = np.zeros(len(k0s))

    for j in range(horizon):
        ki = np.clip(k0s + j, 0, n - 1)
        s_sim  += vx_sim * dt
        vx_sim += a_act[ki] * dt
        s_gt   += gt_vx[ki] * dt

    return np.abs(s_sim - s_gt)


def _long_drift_profile(
    gt_vx: np.ndarray,
    a_act: np.ndarray,
    horizon: int,
    dt: float,
    stride: int = _PERF_STRIDE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """縦方向ドリフトプロファイル"""
    n = len(gt_vx)
    k0s = _valid_k0s(gt_vx, horizon, stride)
    if len(k0s) == 0:
        return None

    m = len(k0s)
    verr = np.zeros((m, horizon + 1), dtype=float)
    serr = np.zeros((m, horizon + 1), dtype=float)

    vx_sim = gt_vx[k0s].astype(float).copy()
    s_sim  = np.zeros(m)
    s_gt   = np.zeros(m)

    for j in range(horizon):
        ki = np.clip(k0s + j, 0, n - 1)
        ki1 = np.clip(k0s + j + 1, 0, n - 1)
        s_sim  += vx_sim * dt
        vx_sim += a_act[ki] * dt
        s_gt   += gt_vx[ki] * dt
        verr[:, j + 1] = vx_sim - gt_vx[ki1]
        serr[:, j + 1] = s_sim - s_gt

    ki_mean = np.minimum(k0s[:, None] + np.arange(horizon)[None, :], n - 1)
    mean_a = a_act[ki_mean].mean(axis=1)
    phase = np.where(mean_a < -_DRIFT_A_TH, 0,
             np.where(mean_a > _DRIFT_A_TH, 2, 1)).astype(int)

    return verr, serr, phase


_PERF_HORIZONS = (10, 20, 50, 100)


def _entry_real_mcap(entry) -> Path | None:
    """DatasetEntry.real_lite から代表 MCAP を解決する。

    resolve_lite_bag の返り値は単一 `.mcap` ファイル / rosbag2 dir の両方があり得るため、
    file はそのまま、dir は `real.lite_0.mcap` → 先頭 `*.mcap` の順で解決する。
    """
    rl = entry.real_lite
    if rl is None:
        return None
    rl = Path(rl)
    if rl.is_file():
        return rl
    mcap = rl / "real.lite_0.mcap"
    if mcap.exists():
        return mcap
    mcaps = list(rl.glob("*.mcap"))
    return mcaps[0] if mcaps else None


def compute_long_perf_data(
    entries: list,
) -> dict:
    """縦方向モデル構造限界評価用のデータ算出を行う。"""
    h_labels = [f"{h * _FIT_DT:.2f}s" for h in _PERF_HORIZONS]
    per_h_errors: dict[int, list[float]] = {h: [] for h in _PERF_HORIZONS}
    _H_MAX = _PERF_HORIZONS[-1]

    all_verr: list[np.ndarray] = []
    all_serr: list[np.ndarray] = []
    all_phase: list[np.ndarray] = []

    map_x_pool: list[np.ndarray] = []
    map_y_pool: list[np.ndarray] = []
    map_serr_pool: list[np.ndarray] = []

    n_dataset = 0
    for entry in entries[:20]:
        mcap = _entry_real_mcap(entry)
        if mcap is None:
            continue
        try:
            df_accel = load_accel(mcap)
            df_vel   = load_velocity(mcap)
            df_kin   = load_kinematic(mcap)
        except Exception:
            continue
        _require_dfs(
            f"compute_long_perf_data:{entry.dataset_id}",
            localization_acceleration=df_accel,
            velocity_status=df_vel,
        )

        timebase = _common_timebase(df_accel, df_vel, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase
        gt_vx = _resample(df_vel, "lon_vel", t_s, t0)
        a_act = _resample(df_accel, "accel", t_s, t0)
        gear_drive = _drive_mask_on_grid(mcap, t_s, t0, context=f"compute_perfect_longitudinal_data:{entry.dataset_id}")
        if len(gt_vx) < 50:
            continue
        gt_vx = np.where(gear_drive, gt_vx, 0.0)
        a_act = np.where(gear_drive, a_act, 0.0)

        n_dataset += 1
        for h in _PERF_HORIZONS:
            errs = _long_nstep_perf(gt_vx, a_act, h, _FIT_DT)
            per_h_errors[h].extend(errs.tolist())

        drift = _long_drift_profile(gt_vx, a_act, _H_MAX, _FIT_DT)
        if drift is not None:
            verr, serr, phase = drift
            all_verr.append(verr)
            all_serr.append(serr)
            all_phase.append(phase)

            if not df_kin.empty:
                gt_x = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["x"].values)
                gt_y = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["y"].values)
                k0s_map = _valid_k0s(gt_vx, _H_MAX, _PERF_STRIDE)
                if len(k0s_map) > 0:
                    map_x_pool.append(gt_x[k0s_map])
                    map_y_pool.append(gt_y[k0s_map])
                    map_serr_pool.append(serr[:, _H_MAX])

    res = {
        "n_dataset": n_dataset,
        "h_labels": h_labels,
        "per_h_errors": {str(h): v for h, v in per_h_errors.items()},
    }

    if all_verr:
        res["verr_pool"] = np.vstack(all_verr).tolist()
        res["serr_pool"] = np.vstack(all_serr).tolist()
        res["phase_pool"] = np.concatenate(all_phase).tolist()

    if map_x_pool:
        res["map_x"] = np.concatenate(map_x_pool).tolist()
        res["map_y"] = np.concatenate(map_y_pool).tolist()
        res["map_serr"] = np.concatenate(map_serr_pool).tolist()

    return res


def compute_perfect_tracking_data(
    entries: list,
    params: dict,
) -> dict:
    """操舵理想追従評価用のデータ算出を行う。"""
    h_labels = [f"{h * _FIT_DT:.2f}s" for h in _PERF_HORIZONS]
    per_h_errors: dict[int, list[float]] = {h: [] for h in _PERF_HORIZONS}

    traj_data: list[dict] = []
    n_dataset = 0
    for entry in entries[:10]:
        mcap = _entry_real_mcap(entry)
        if mcap is None:
            continue
        try:
            df_kin   = load_kinematic(mcap)
            df_steer = load_steering(mcap)
        except Exception:
            continue
        _require_dfs(
            f"compute_perfect_tracking_data:{entry.dataset_id}",
            kinematic_state=df_kin,
            steering_status=df_steer,
        )
        if len(df_kin) < 50:
            continue

        timebase = _common_timebase(df_kin, df_steer, min_span_ns=_MIN_SPAN_NS)
        if timebase is None:
            continue
        t_s, t0 = timebase

        gt_x     = _resample(df_kin,   "x",     t_s, t0)
        gt_y     = _resample(df_kin,   "y",     t_s, t0)
        gt_yaw   = _resample(df_kin,   "yaw",   t_s, t0)
        gt_vx    = _resample(df_kin,   "vx",    t_s, t0)
        gt_steer = _resample(df_steer, "steer", t_s, t0)
        gear_drive = _drive_mask_on_grid(mcap, t_s, t0, context=f"compute_perfect_tracking_data:{entry.dataset_id}")

        n_dataset += 1
        for h in _PERF_HORIZONS:
            lat_errs = _bicycle_nstep_perf(gt_x, gt_y, gt_yaw, gt_vx, gt_steer, params, h, _FIT_DT)
            per_h_errors[h].extend(lat_errs.tolist())

        if len(traj_data) < 3:
            bx, by = _bicycle_trajectory_full(
                gt_vx, gt_steer, float(gt_x[0]), float(gt_y[0]), float(gt_yaw[0]), params, _FIT_DT
            )
            moving = gear_drive & (gt_vx > VX_MIN_CURVE)
            _PLOT_STRIDE = 5
            traj_data.append({
                "uuid": entry.dataset_id[:8],
                "gt_x": ((gt_x - gt_x[0])[::_PLOT_STRIDE]).tolist(),
                "gt_y": ((gt_y - gt_y[0])[::_PLOT_STRIDE]).tolist(),
                "bx":   ((bx   - gt_x[0])[::_PLOT_STRIDE]).tolist(),
                "by":   ((by   - gt_y[0])[::_PLOT_STRIDE]).tolist(),
                "moving": (moving[::_PLOT_STRIDE]).tolist(),
            })

    return {
        "n_dataset": n_dataset,
        "h_labels": h_labels,
        "per_h_errors": {str(h): v for h, v in per_h_errors.items()},
        "traj_data": traj_data,
    }


def physical_validity_jsonable(pv: dict | None) -> dict | None:
    """physical_validity を cases/cross metrics JSON に埋め込める形へ変換する。

    kus_bins は十分統計量のみを残す。step13 の横断集約はこの情報から再構成でき、
    生サンプルの再読込を不要にする。
    """
    if pv is None:
        return None

    def _finite_or_none(x) -> float | None:
        xf = float(x)
        return xf if np.isfinite(xf) else None

    def _fit(fit: dict | None) -> dict | None:
        if fit is None:
            return None
        return {k: (_finite_or_none(v) if isinstance(v, float) else v) for k, v in fit.items()}

    kus_bins = pv.get("kus_bins")
    kus_bins_json = None
    if kus_bins is not None:
        kus_bins_json = {
            "vx_mid": [_finite_or_none(v) for v in kus_bins["vx_mid"]],
            "n_pts": [int(v) for v in kus_bins["n_pts"]],
            "sum_wz2": [_finite_or_none(v) for v in kus_bins["sum_wz2"]],
            "sum_wz_ts": [_finite_or_none(v) for v in kus_bins["sum_wz_ts"]],
        }

    return {"long": _fit(pv.get("long")), "steer": _fit(pv.get("steer")), "kus_bins": kus_bins_json}
