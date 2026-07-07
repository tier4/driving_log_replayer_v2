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
    横方向: tan(steer_eff) = (L/v + k_us*v) * wz  (原点回帰、k_us はスカラー)

collection 横断の k_us は十分統計量 (sum_x2, sum_xy) の加算プールで再構成できる
(原点回帰の正規方程式が線形加算的なため、生サンプル再読込は不要)。縦方向の横断フィットは
非線形遅延グリッドサーチのため十分統計量に還元できず、n_dyn 上位データセットのみ MCAP を
再読込する (`fit_long_cross_dataset_bounded`)。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

from ._io import (
    load_accel,
    load_cmd,
    load_gear_status,
    load_kinematic,
    load_steering,
    load_velocity,
    require_drive_gear_mask,
)
from ._fit_core import (
    delay_shift as _delay_shift,
    delay_shift_frac as _delay_shift_frac,
    equation_residual_at_params,
    fit_first_order_delay,
    fit_first_order_delay_residual_3phase,
)
from ._fit_core import sim_first_order as _core_sim_first_order
from ._fit_core import sim_first_order_frac as _core_sim_first_order_frac
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

_FIT_DT = 1.0 / 30.0    # モデルフィット用リサンプリング DT [s] (制御コマンド周期 30 Hz)
_N_DYN_MIN = int(round(1.0 / _FIT_DT))  # 動的フィット対象の最小サンプル数 (約 1 秒)
_MIN_FIT_SAMPLES = int(round(0.5 / _FIT_DT))  # 単一フィット可否判定の最小サンプル数 (約 0.5 秒)
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
_DELAY_CANDIDATES_LONG = np.arange(0.0, 0.30 + 1e-9, _FIT_DT)
_TAU_BOUNDS_LONG = (_FIT_DT, 0.5)

# 操舵 (SSOT は本モジュール。由来は旧 identify_steer_dynamics.py の同定手順、git 履歴参照)
_DSTEER_MIN = 0.001
_DELAY_CANDIDATES_STEER = np.arange(0.0, 0.15 + 1e-9, _FIT_DT)
_TAU_BOUNDS_STEER = (_FIT_DT, 2.0)

# 方程式残差診断 (equation_residual_at_params) の LHS を作る Savitzky-Golay 平滑化微分パラメータ。
# 窓は「対象 τ 下限より短く」するのが原則 (立ち上がりの過平滑化で残差の位相を歪める)。
# 縦 0.2s / 操舵 0.1s。推定量ではなく診断用なので、同定値には影響しない。
_SG_POLYORDER = 2
_SG_WINDOW_LONG = 0.2    # [s]
_SG_WINDOW_STEER = 0.1   # [s]


# ---------------------------------------------------------------------------
# 共通ヘルパー
# ---------------------------------------------------------------------------
# 一次遅れ + 純粋遅延の数値カーネルは lib._fit_core (SSOT)。ここでは検証側の既定 DT
# (_FIT_DT = 1/30) を注入する薄いラッパーだけ置く (多数の呼び出しが dt を省略するため)。
def _sim_first_order(cmd, tau, n_delay, dt=_FIT_DT, y0=None):
    return _core_sim_first_order(cmd, tau, n_delay, dt, y0=y0)


def _sim_first_order_frac(cmd, tau, delay, t_s, dt=_FIT_DT, y0=None):
    return _core_sim_first_order_frac(cmd, tau, delay, t_s, dt, y0=y0)


def _pointwise_tau_estimate(
    a_cmd_arr: np.ndarray, a_act_corr: np.ndarray, mask_dyn: np.ndarray, n_delay: int, dt: float = _FIT_DT,
) -> np.ndarray:
    """離散更新式 a[k] = (1-α)a[k-1] + α u[k] を隣接 2 サンプルから逆算した瞬時 τ 推定値。

    τ[k] = dt * (u[k] - a[k-1]) / (a[k] - a[k-1])。分母が小さい (停車・定常区間)・
    mask_dyn 外・τ が非物理的 (負値) なサンプルは NaN にする。戻り値は a_act_corr と同じ長さ
    (先頭は差分が取れないため NaN)。
    """
    u = _delay_shift(a_cmd_arr, n_delay)
    da = np.diff(a_act_corr)
    numer = dt * (u[1:] - a_act_corr[:-1])
    with np.errstate(divide="ignore", invalid="ignore"):
        tau_raw = numer / da
    valid = mask_dyn[1:] & (np.abs(da) > 1e-3) & np.isfinite(tau_raw) & (tau_raw > 0)
    out = np.full(len(a_act_corr), np.nan)
    out[1:][valid] = tau_raw[valid]
    return out


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
    return require_drive_gear_mask(df_gear, target_t_ns, context=context, allow_leading_gap=True)


def _require_dfs(context: str, **dfs: pd.DataFrame) -> None:
    """必須 DataFrame 群が空でないことをまとめて検証する。"""
    for name, df in dfs.items():
        require_non_empty_df(df, name=name, context=context)


def _mask_stopped(arr: np.ndarray, moving: np.ndarray) -> list:
    """停車中サンプルを NaN でマスクしたリストを返す (moving=True のみ有効)。"""
    out = np.where(moving, arr.astype(float), np.nan)
    return out.tolist()


def _mask_moving(arr: np.ndarray, moving: np.ndarray) -> list:
    """走行中サンプルを NaN でマスクしたリストを返す (moving=False のみ有効)。"""
    out = np.where(~moving, arr.astype(float), np.nan)
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

    Step 1 (グリッドサーチ): T を _DELAY_CANDIDATES_LONG (1/30s 刻み) 上に固定し、
        各 T について tau を NLS で連続最適化、MSE 最小の (T, tau) ペアを採る。
    Step 2 (substep): Step 1 の tau を固定し、T を近傍 ±1グリッド幅の範囲で
        fractional delay (線形補間) により連続最適化する。T はサンプル整数倍に
        量子化されているため、Step 1 だけでは T の分解能がグリッド幅 (1/30s) に
        制限される。tau を固定して T のみ連続最適化することで、そのグリッド量子化
        誤差をサブサンプル精度で補正する (tau は再最適化しない = 純粋な T 再探索)。

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
    if mask_dyn.sum() < _MIN_FIT_SAMPLES:
        return None

    # パラメータ推定: 3段階交互最適化＋方程式残差最小二乗法で同定する。
    # 縦方向 (加速度) には LPF (窓幅 10ステップ) を適用しノイズを低減。
    fit = fit_first_order_delay_residual_3phase(
        a_cmd_arr, a_act_corr, mask_dyn, _FIT_DT,
        tau_bounds=_TAU_BOUNDS_LONG, delay_candidates=_DELAY_CANDIDATES_LONG,
        filter_w=10, x0_dict={"tau": 0.1},
    )
    if fit is None:
        return None

    # 診断 (方針D): 上で同定した (τ, T) における dot a_act 方程式の残差 E[k]=RHS−LHS を評価。
    # 目的関数ではなく「同定結果が ODE をどれだけ満たすか」の整合診断・レポート統一記法用。
    resid = equation_residual_at_params(
        a_cmd_arr, a_act_corr, mask_dyn, _FIT_DT,
        tau=fit["tau"], delay=fit["delay"], scale=1.0, bias=0.0,
        window_s=_SG_WINDOW_LONG, polyorder=_SG_POLYORDER, t_s=t_s,
    )

    pitch_min, pitch_max = _pitch_range(bag_path)
    return {
        "tau": fit["tau"],
        "delay": fit["delay"],
        "rmse_mps2": fit["rmse"],
        "rmse_resid": resid["rmse_resid"],       # 診断: 方程式残差 RMSE [m/s³]
        "resid_samples": resid["resid"].tolist(),  # 診断: mask 内残差 (ヒスト集約用)
        "n_dyn": int(mask_dyn.sum()),
        "pitch_min": pitch_min,
        "pitch_max": pitch_max,
        "cmd_arr": a_cmd_arr,
        "act_arr": a_act_corr,
        "mask_arr": mask_dyn,
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
    if mask_dyn.sum() < _MIN_FIT_SAMPLES:
        return None

    # パラメータ推定: 3段階交互最適化＋方程式残差最小二乗法で同定する。
    fit = fit_first_order_delay_residual_3phase(
        d_cmd, d_act, mask_dyn, _FIT_DT,
        tau_bounds=_TAU_BOUNDS_STEER, delay_candidates=_DELAY_CANDIDATES_STEER,
        filter_w=1, x0_dict={"tau": 0.1},
    )
    if fit is None:
        return None

    # 診断 (方針D): 同定した (τ, T) における dot δ_act 方程式の残差 E[k]=RHS−LHS を評価。
    # 出力誤差型は bias β を同定しないため scale=1/bias=0 で評価する (残差の直流成分は未モデル化の
    # 中点ズレを反映する = 診断シグナル)。
    resid = equation_residual_at_params(
        d_cmd, d_act, mask_dyn, _FIT_DT,
        tau=fit["tau"], delay=fit["delay"], scale=1.0, bias=0.0,
        window_s=_SG_WINDOW_STEER, polyorder=_SG_POLYORDER, t_s=t_s,
    )

    return {
        "tau": fit["tau"],
        "delay": fit["delay"],
        "rmse_mrad": fit["rmse"] * 1000.0,
        "rmse_resid": resid["rmse_resid"],        # 診断: 方程式残差 RMSE [rad/s]
        "resid_samples": resid["resid"].tolist(),   # 診断: mask 内残差 (ヒスト集約用)
        "n_dyn": int(mask_dyn.sum()),
        "cmd_arr": d_cmd,
        "act_arr": d_act,
        "mask_arr": mask_dyn,
    }


# ---------------------------------------------------------------------------
# 横方向 k_us: 全速度域一括の原点回帰 (スカラー、十分統計量つき、単一/複数データセット両対応)
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
        allow_leading_gap=True,
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
    """全速度域一括の最小二乗法回帰でスカラー k_us を推定する。

    モデル: tan(δ_eff) = (L/v + k_us·v)·ω (原点回帰)。整理すると
    y = tan(δ_eff) − L·ω/v を x = v·ω に回帰して k_us = Σ(xy)/Σ(x²)。
    records は `_extract_kus_arrays` の返り値の list (単一データセットなら 1 要素)。
    十分統計量 (sum_x2/sum_xy) は加算的なので、collection 横断側で生サンプルなしに
    プール再構成できる (`compute_kus_bins_from_sufficient_stats`)。
    """
    all_vx = np.concatenate([r["vx"] for r in records]) if records else np.empty(0)
    all_wz = np.concatenate([r["wz"] for r in records]) if records else np.empty(0)
    all_steer_eff = np.concatenate([r["steer_eff"] for r in records]) if records else np.empty(0)
    all_dwz = np.concatenate([r["dwz"] for r in records]) if records else np.empty(0)
    all_gear_drive = np.concatenate([r.get("gear_drive", np.ones_like(r["vx"], dtype=bool)) for r in records]) if records else np.empty(0, dtype=bool)

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

    # 原点回帰 y = k_us·x,  x = v·ω,  y = tan(δ_eff) − L·ω/v
    x = vx_f * wz_f
    y = tan_steer - WHEELBASE * wz_f / vx_f if len(vx_f) else np.empty(0)
    sum_x2 = float(np.sum(x * x))
    sum_xy = float(np.sum(x * y))
    n_pts = int(len(vx_f))
    k_us = sum_xy / sum_x2 if (n_pts >= 10 and sum_x2 > 0) else float("nan")

    # 個別サンプル percentile (外れ値確認用)
    with np.errstate(divide="ignore", invalid="ignore"):
        kus_each = (
            tan_steer / np.where(np.abs(wz_f) > 1e-6, wz_f, np.nan) - WHEELBASE / vx_f
        ) / vx_f
    kus_each = kus_each[np.isfinite(kus_each)]
    kus_each = np.clip(kus_each, -K_US_CLIP, K_US_CLIP)
    kus_p25 = float(np.percentile(kus_each, 25)) if len(kus_each) > 10 else float("nan")
    kus_p75 = float(np.percentile(kus_each, 75)) if len(kus_each) > 10 else float("nan")

    return {
        "k_us": k_us,
        "n_pts": n_pts,
        "sum_x2": sum_x2,
        "sum_xy": sum_xy,
        "kus_p25": kus_p25,
        "kus_p75": kus_p75,
    }


def compute_kus_bins_single(bag_path: Path) -> dict | None:
    """単一データセットのスカラー k_us 回帰 (十分統計量つき)。"""
    rec = _extract_kus_arrays(bag_path)
    if rec is None:
        return None
    return compute_kus_bins([rec])


def compute_kus_bins_from_sufficient_stats(per_ds_bins: list[dict]) -> dict:
    """複数データセットの十分統計量 (sum_x2/sum_xy) を加算的にプールしスカラー k_us を再構成する。

    原点回帰 y=k_us·x の正規方程式 k_us=Σ(xy)/Σ(x²) は加算的に成り立つため、
    生サンプルの再読込なしに全データセット結合と同一の最小二乗解が得られる。
    IQR (p25/p75) は個別サンプル分布に依存し十分統計量から再構成できないため省略する。
    """
    if not per_ds_bins:
        return {
            "k_us": float("nan"),
            "n_pts": 0,
            "sum_x2": 0.0,
            "sum_xy": 0.0,
            "kus_p25": float("nan"),
            "kus_p75": float("nan"),
        }

    sum_x2 = 0.0
    sum_xy = 0.0
    n_pts = 0
    for b in per_ds_bins:
        sum_x2 += float(np.nan_to_num(b.get("sum_x2", 0.0)))
        sum_xy += float(np.nan_to_num(b.get("sum_xy", 0.0)))
        n_pts += int(b.get("n_pts", 0))

    k_us = sum_xy / sum_x2 if (n_pts >= 10 and sum_x2 > 0) else float("nan")
    return {
        "k_us": k_us,
        "n_pts": n_pts,
        "sum_x2": sum_x2,
        "sum_xy": sum_xy,
        "kus_p25": float("nan"),
        "kus_p75": float("nan"),
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
            a_cmd_arr,
            fit["tau"],
            int(round(fit["delay"] / _FIT_DT)),
            y0=float(a_act_arr[0] - slope_acc_arr[0]),
        ) + slope_acc_arr

    a_sim_models: dict[str, list] = {}
    a_sim_models_low: dict[str, list] = {}
    model_tune: dict[str, dict] = {}
    for name, raw_params in (models or {}).items():
        merged = merged_model_params(raw_params)
        tau_tune = merged.get("acc_time_constant")
        T_tune = merged.get("acc_time_delay")
        if tau_tune is None or T_tune is None:
            continue
        sim = _sim_first_order(
            a_cmd_arr, float(tau_tune), int(round(float(T_tune) / _FIT_DT)),
            y0=float(a_act_arr[0] - slope_acc_arr[0]),
        ) + slope_acc_arr
        a_sim_models[name] = _mask_stopped(sim, moving)
        a_sim_models_low[name] = _mask_moving(sim, moving)
        model_tune[name] = {"tau": float(tau_tune), "T": float(T_tune)}

    return {
        "t": t_s.tolist(),
        "moving": moving.tolist(),
        "a_cmd": _mask_stopped(a_cmd_arr, moving),
        "a_cmd_low": _mask_moving(a_cmd_arr, moving),
        "a_act": _mask_stopped(a_act_arr, moving),
        "a_act_low": _mask_moving(a_act_arr, moving),
        "a_sim_fit": _mask_stopped(a_sim_fit, moving) if a_sim_fit is not None else None,
        "a_sim_fit_low": _mask_moving(a_sim_fit, moving) if a_sim_fit is not None else None,
        "a_sim_models": a_sim_models,
        "a_sim_models_low": a_sim_models_low,
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
# collection 横断: データセット選定・縦方向横断フィット (有界 MCAP 再読込)
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


def _valid_fit_entries(entries: list, per_ds_fit: dict[str, dict], rmse_key: str) -> list:
    """時系列表示候補になる per-dataset fit 済み entry を返す。"""
    return [
        e for e in entries
        if e.real_lite is not None
        and e.dataset_id in per_ds_fit
        and per_ds_fit[e.dataset_id].get(rmse_key) is not None
        and np.isfinite(per_ds_fit[e.dataset_id][rmse_key])
    ]


_CONTIGUOUS_GAP_NS = 2.0 * _FIT_DT * 1e9


def _merge_contiguous_timeseries_rows(rows: list[dict]) -> list[dict]:
    """ROS 絶対時刻が連続するデータセットの時系列行を結合する。"""
    valid = [r for r in rows if r.get("t") and r.get("_t0_ns") is not None]
    if not valid:
        return rows

    ordered = sorted(valid, key=lambda r: float(r["_t0_ns"]))
    groups: list[list[dict]] = []
    for row in ordered:
        if groups:
            prev_end_ns = max(
                float(r["_t0_ns"]) + float(r["t"][-1]) * 1e9 for r in groups[-1]
            )
            if float(row["_t0_ns"]) - prev_end_ns <= _CONTIGUOUS_GAP_NS:
                groups[-1].append(row)
                continue
        groups.append([row])

    merged_rows: list[dict] = []
    for group in groups:
        if len(group) == 1:
            row = dict(group[0])
            row["dataset_ids"] = [row.get("dataset_id", "")]
            merged_rows.append(row)
            continue

        scalar_keys: set[str] = set()
        mapping_keys: dict[str, set[str]] = {}
        for row in group:
            n = len(row["t"])
            for key, value in row.items():
                if isinstance(value, list) and len(value) == n:
                    scalar_keys.add(key)
                elif isinstance(value, dict):
                    names = {
                        name for name, series in value.items()
                        if isinstance(series, list) and len(series) == n
                    }
                    if names:
                        mapping_keys.setdefault(key, set()).update(names)
        scalar_keys.discard("t")

        group_t0_ns = float(group[0]["_t0_ns"])
        t_merged: list[float] = []
        scalar_merged = {key: [] for key in scalar_keys}
        mapping_merged = {
            key: {name: [] for name in names}
            for key, names in mapping_keys.items()
        }
        last_ns = float("-inf")
        last_out_s = float("-inf")
        for row in group:
            t_abs_ns = float(row["_t0_ns"]) + np.asarray(row["t"], dtype=float) * 1e9
            keep = t_abs_ns > last_ns + 0.5 * _FIT_DT * 1e9
            if np.any(keep):
                indices = np.flatnonzero(keep)
                t_out = (t_abs_ns[indices] - group_t0_ns) * 1e-9
            else:
                # Some extracted real.lite MCAPs reset/overlap ROS record timestamps per
                # dataset. They are still a selected contiguous group, so append by each
                # row's relative time instead of dropping the whole row as duplicate time.
                indices = np.arange(len(row["t"]))
                row_t = np.asarray(row["t"], dtype=float)
                t_out = last_out_s + _FIT_DT + (row_t[indices] - row_t[indices[0]])
            t_merged.extend(t_out.tolist())
            for key in scalar_keys:
                value = row.get(key)
                source = (
                    value
                    if isinstance(value, list) and len(value) == len(row["t"])
                    else [np.nan] * len(row["t"])
                )
                scalar_merged[key].extend(np.asarray(source)[indices].tolist())
            for key, named_series in mapping_merged.items():
                source_map = row.get(key) or {}
                for name, target in named_series.items():
                    source = source_map.get(name, [np.nan] * len(row["t"]))
                    target.extend(np.asarray(source)[indices].tolist())
            last_ns = float(t_abs_ns[indices[-1]])
            last_out_s = float(t_merged[-1])

        dataset_ids = [str(row.get("dataset_id", "")) for row in group]
        case_tags = list(dict.fromkeys(str(row.get("case_tag", "")) for row in group))
        merged = {
            key: value for key, value in group[0].items()
            if key not in scalar_keys and key not in mapping_keys and key != "t"
        }
        merged.update(scalar_merged)
        merged.update(mapping_merged)
        merged.update({
            "t": t_merged,
            "_t0_ns": group_t0_ns,
            "dataset_ids": dataset_ids,
            "label": (
                f"[{'/'.join(filter(None, case_tags))}] "
                f"{dataset_ids[0][:8]} → {dataset_ids[-1][:8]}（{len(dataset_ids)}件連続）"
            ),
        })
        merged_rows.append(merged)
    return merged_rows


def _pick_longest_contiguous_timeseries_row(rows: list[dict]) -> list[dict]:
    """連続結合後の最長時系列だけを返す。"""
    merged = _merge_contiguous_timeseries_rows(rows)
    if not merged:
        return []

    def _duration(row: dict) -> float:
        t = row.get("t") or []
        return float(t[-1] - t[0]) if len(t) >= 2 else 0.0

    best = max(
        merged,
        key=lambda row: (
            _duration(row),
            len(row.get("dataset_ids", [row.get("dataset_id", "")])),
            len(row.get("t") or []),
        ),
    )
    row = dict(best)
    dataset_ids = [str(x) for x in row.get("dataset_ids", [row.get("dataset_id", "")])]
    duration_s = _duration(row)
    row["case_tag"] = "最長連続"
    row["label"] = (
        f"[最長連続] {dataset_ids[0][:8]} → {dataset_ids[-1][:8]}"
        f"（{len(dataset_ids)}件連続、{duration_s:.1f}s）"
    )
    return [row]


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
        if mask.sum() < _MIN_FIT_SAMPLES:
            continue
        pooled.append((a_cmd_arr, a_act_corr_arr, mask))

    pitch_min = float(min(all_pitch)) if all_pitch else 0.0
    pitch_max = float(max(all_pitch)) if all_pitch else 0.0
    if not pooled:
        return {
            "tau": float("nan"), "delay": float("nan"), "rmse_mps2": float("nan"),
            "pitch_min": pitch_min, "pitch_max": pitch_max, "n_datasets": 0,
            "tau_pointwise_all": [],
        }

    from scipy.optimize import least_squares

    # 各データセットについて、状態微分を算出し LPF (移動平均 10ステップ) を適用
    processed_pooled = []
    for a_cmd_arr, a_act_corr_arr, mask in pooled:
        dot_act = np.gradient(a_act_corr_arr, _FIT_DT)
        cmd_f = _moving_avg(a_cmd_arr, 10)
        act_f = _moving_avg(a_act_corr_arr, 10)
        dot_act_f = _moving_avg(dot_act, 10)
        processed_pooled.append((cmd_f, act_f, dot_act_f, mask))

    def _residuals(x, n_steps):
        tau_inv = float(x[0])
        errs = []
        for cmd_f, act_f, dot_act_f, mask in processed_pooled:
            c_del = _delay_shift(cmd_f, n_steps)
            a_del = _delay_shift(act_f, n_steps)
            E = tau_inv * (c_del - a_del) - dot_act_f
            errs.append(E[mask])
        return np.concatenate(errs)

    tau_inv_min = 1.0 / _TAU_BOUNDS_LONG[1]
    tau_inv_max = 1.0 / _TAU_BOUNDS_LONG[0]
    bounds = ([tau_inv_min], [tau_inv_max])

    # Phase 1: d = 0 固定で最適化
    res1 = least_squares(lambda x: _residuals(x, 0), [1.0 / 0.2], bounds=bounds)

    # Phase 2: 得られたパラメータ固定で delay をグリッドサーチ
    best_cost = None
    best_n = 0
    for delay_s in _DELAY_CANDIDATES_LONG:
        n_delay = int(round(delay_s / _FIT_DT))
        cost = float(np.sum(_residuals(res1.x, n_delay) ** 2))
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_n = n_delay

    # Phase 3: 最良 delay 固定でパラメータを再最適化
    res3 = least_squares(lambda x: _residuals(x, best_n), res1.x, bounds=bounds)

    best_tau = 1.0 / float(res3.x[0])
    best_delay = float(best_n * _FIT_DT)
    n_total_samples = sum(int(mask.sum()) for _, _, _, mask in processed_pooled)
    best_rmse = float(np.sqrt(best_cost / n_total_samples)) if n_total_samples > 0 else float("nan")

    # 横断フィットに使った全 pooled データセットについて、最適無駄時間 (best_delay) 固定で
    # 瞬時 τ_a 推定値 (点ごとの逆算) を計算しプールする。分布の可視化 (ヒストグラム) 用。
    tau_pointwise_all: list[float] = []
    if np.isfinite(best_delay):
        n_delay_best = int(round(best_delay / _FIT_DT))
        for a_cmd_arr, a_act_corr_arr, mask in pooled:
            tp = _pointwise_tau_estimate(a_cmd_arr, a_act_corr_arr, mask, n_delay_best)
            tau_pointwise_all.extend(tp[np.isfinite(tp)].tolist())

    return {
        "tau": best_tau, "delay": best_delay, "rmse_mps2": best_rmse,
        "pitch_min": pitch_min, "pitch_max": pitch_max, "n_datasets": len(pooled),
        "tau_pointwise_all": tau_pointwise_all,
    }


def compute_cross_long_rows(
    entries: list, per_ds_long: dict[str, dict], cross_fit: dict, models: dict | None,
) -> list[dict]:
    """最長連続データセットの縦方向時系列 (横断フィット + モデル別チューン値重ね描き用)。"""
    candidates = _valid_fit_entries(entries, per_ds_long, "rmse_mps2")
    rows: list[dict] = []
    for entry in candidates:
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
        a_act_corr = a_act_arr - slope_acc_arr
        d_cmd = np.abs(np.gradient(a_cmd_arr, _FIT_DT))
        mask_dyn = gear_drive & (vx > _VX_MIN_FIT) & (d_cmd > _DA_THRESH_FIT)
        moving = gear_drive & (vx > VX_MIN_CURVE)

        a_sim_cross = None
        a_sim_cross_corr = None
        tau_pointwise = None
        if np.isfinite(cross_fit.get("tau", float("nan"))):
            n_delay_cross = int(round(cross_fit["delay"] / _FIT_DT))
            a_sim_cross_corr = _sim_first_order(
                a_cmd_arr, cross_fit["tau"], n_delay_cross, y0=float(a_act_corr[0]),
            )
            a_sim_cross = a_sim_cross_corr + slope_acc_arr
            tau_pointwise = _pointwise_tau_estimate(a_cmd_arr, a_act_corr, mask_dyn, n_delay_cross)

        a_sim_models: dict[str, list] = {}
        a_sim_models_low: dict[str, list] = {}
        a_sim_models_raw: dict[str, list] = {}
        dot_a_sim_models: dict[str, list] = {}
        for name, spec in (models or {}).items():
            merged = merged_model_params(spec.params)
            tau = merged.get("acc_time_constant")
            delay = merged.get("acc_time_delay")
            if tau is None or delay is None:
                continue
            sim_corr = _sim_first_order(
                a_cmd_arr, float(tau), int(round(float(delay) / _FIT_DT)),
                y0=float(a_act_arr[0] - slope_acc_arr[0]),
            )
            sim = sim_corr + slope_acc_arr
            a_sim_models[name] = _mask_stopped(sim, moving)
            a_sim_models_low[name] = _mask_moving(sim, moving)
            a_sim_models_raw[name] = sim.tolist()
            dot_a_sim_models[name] = np.gradient(sim_corr, _FIT_DT).tolist()

        fit = per_ds_long.get(entry.dataset_id, {})
        rmse = fit.get("rmse_mps2", float("nan"))
        rows.append({
            "label": f"[最長連続候補] {entry.dataset_id[:8]}  RMSE={rmse:.3f} m/s²",
            "dataset_id": entry.dataset_id,
            "case_tag": "最長連続候補",
            "_t0_ns": t0,
            "t": t_s.tolist(),
            "moving": moving.tolist(),
            "gear_drive": gear_drive.tolist(),
            "mask_dyn": mask_dyn.tolist(),
            "vx": vx.tolist(),
            "a_cmd_raw": a_cmd_arr.tolist(),
            "a_act_raw": a_act_arr.tolist(),
            "a_act_corr": a_act_corr.tolist(),
            "slope_acc": slope_acc_arr.tolist(),
            "dot_a_cmd": np.gradient(a_cmd_arr, _FIT_DT).tolist(),
            "dot_a_act": np.gradient(a_act_corr, _FIT_DT).tolist(),
            "dot_a_sim_cross": (
                np.gradient(a_sim_cross_corr, _FIT_DT).tolist()
                if a_sim_cross_corr is not None else None
            ),
            "a_cmd": _mask_stopped(a_cmd_arr, moving),
            "a_cmd_low": _mask_moving(a_cmd_arr, moving),
            "a_act": _mask_stopped(a_act_arr, moving),
            "a_act_low": _mask_moving(a_act_arr, moving),
            "a_sim_cross": _mask_stopped(a_sim_cross, moving) if a_sim_cross is not None else None,
            "a_sim_cross_raw": a_sim_cross.tolist() if a_sim_cross is not None else None,
            "a_sim_cross_low": _mask_moving(a_sim_cross, moving) if a_sim_cross is not None else None,
            "a_sim_models": a_sim_models,
            "a_sim_models_raw": a_sim_models_raw,
            "a_sim_models_low": a_sim_models_low,
            "dot_a_sim_models": dot_a_sim_models,
            "tau_pointwise": tau_pointwise.tolist() if tau_pointwise is not None else None,
        })
    return _pick_longest_contiguous_timeseries_row(rows)


def compute_cross_steer_rows(
    entries: list, per_ds_steer: dict[str, dict], models: dict | None,
) -> list[dict]:
    """最長連続データセットの操舵時系列 (per-dataset フィット + モデル別チューン値重ね描き用)。"""
    candidates = _valid_fit_entries(entries, per_ds_steer, "rmse_mrad")
    rows: list[dict] = []
    for entry in candidates:
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
        dot_d_cmd = np.gradient(d_cmd, _FIT_DT)
        dot_d_act = np.gradient(d_act, _FIT_DT)
        mask_dyn = gear_drive & (vx > _VX_MIN_FIT) & (np.abs(dot_d_cmd) > _DSTEER_MIN / _FIT_DT)
        moving = gear_drive & (vx > VX_MIN_CURVE)

        fit = per_ds_steer.get(entry.dataset_id, {})
        d_sim_fit = None
        dot_d_open_fit = None
        if np.isfinite(fit.get("tau", float("nan"))):
            n_delay_fit = int(round(fit["delay"] / _FIT_DT))
            d_sim_fit = _sim_first_order(d_cmd, fit["tau"], n_delay_fit)
            dot_d_open_fit = (_delay_shift(d_cmd, n_delay_fit) - d_act) / max(float(fit["tau"]), 1e-9)

        d_sim_models: dict[str, list] = {}
        d_sim_models_raw: dict[str, list] = {}
        dot_d_sim_models: dict[str, list] = {}
        for name, spec in (models or {}).items():
            merged = merged_model_params(spec.params)
            tau = merged.get("steer_time_constant")
            delay = merged.get("steer_time_delay")
            if tau is None or delay is None:
                continue
            sim = _sim_first_order(d_cmd, float(tau), int(round(float(delay) / _FIT_DT)))
            d_sim_models[name] = _mask_stopped(sim, moving)
            d_sim_models_raw[name] = sim.tolist()
            dot_d_sim_models[name] = np.gradient(sim, _FIT_DT).tolist()

        rmse = fit.get("rmse_mrad", float("nan"))
        rows.append({
            "label": f"[最長連続候補] {entry.dataset_id[:8]}  RMSE={rmse:.1f} mrad",
            "dataset_id": entry.dataset_id,
            "case_tag": "最長連続候補",
            "_t0_ns": t0,
            "t": t_s.tolist(),
            "moving": moving.tolist(),
            "gear_drive": gear_drive.tolist(),
            "mask_dyn": mask_dyn.tolist(),
            "vx": vx.tolist(),
            "d_cmd": d_cmd.tolist(),
            "dot_d_cmd": dot_d_cmd.tolist(),
            "dot_d_act": dot_d_act.tolist(),
            "dot_d_sim_fit": np.gradient(d_sim_fit, _FIT_DT).tolist() if d_sim_fit is not None else None,
            "dot_d_open_fit": dot_d_open_fit.tolist() if dot_d_open_fit is not None else None,
            "d_act": _mask_stopped(d_act, moving),
            "d_act_raw": d_act.tolist(),
            "d_sim_fit": _mask_stopped(d_sim_fit, moving) if d_sim_fit is not None else None,
            "d_sim_fit_raw": d_sim_fit.tolist() if d_sim_fit is not None else None,
            "d_sim_models": d_sim_models,
            "d_sim_models_raw": d_sim_models_raw,
            "dot_d_sim_models": dot_d_sim_models,
        })
    return _pick_longest_contiguous_timeseries_row(rows)


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
    params.steer_bias / スカラー k_us を C++ モデルと同一式で評価し積分する。
    Returns: |横方向誤差| [m] 配列（vx > VX_MIN_CURVE な開始点のみ）
    """
    L = WHEELBASE
    beta = float(params.get("steer_bias", 0.0))
    k_us = float(params.get("k_us", 0.0))
    denom_arr = L + k_us * gt_vx ** 2
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
    k_us = float(params.get("k_us", 0.0))
    denom_arr = L + k_us * gt_vx ** 2
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
        "dt_s": _FIT_DT,
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
        "dt_s": _FIT_DT,
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
            "k_us": _finite_or_none(kus_bins["k_us"]),
            "n_pts": int(kus_bins["n_pts"]),
            "sum_x2": _finite_or_none(kus_bins["sum_x2"]),
            "sum_xy": _finite_or_none(kus_bins["sum_xy"]),
        }

    return {"long": _fit(pv.get("long")), "steer": _fit(pv.get("steer")), "kus_bins": kus_bins_json}
