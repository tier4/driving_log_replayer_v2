"""N-step オープンループロールアウト評価。"""
from __future__ import annotations

import ctypes
import numpy as np
import pandas as pd

from ..lib._events import find_autonomous_start as _find_autonomous_start
from ..lib._nstep_common import METRIC_KEYS, interp_or_zeros, local_ds, rms, to_seconds
from ..lib._params_utils import load_sim_params
from ..lib._validation import require_non_empty_df
from ..lib._vehicle_models import VehicleModel
from .gear import require_drive_gear_mask
from .physical_constants import VX_MIN_CURVE
from .parameter_constraints import PARAMETER_CONSTRAINTS
from .settings import (
    BAD_INTERVAL_MAX_S,
    BAD_INTERVAL_MIN_S,
    KINEMATIC_STEER_VX_MIN,
    ROLLING_SMOOTH_WINDOW_S,
    ROLL_OUT_CONTEXT,
    ROLLOUT_SUB_DT,
)

SUB_DT: float = ROLLOUT_SUB_DT


def build_params(wheelbase: float | None = None) -> dict:
    """`vehicle_info.param.yaml` + N-step 解析固有の上書きで params dict を構築する。"""
    base = load_sim_params()
    base.setdefault(
        "wheelbase",
        base.get("wheel_base", PARAMETER_CONSTRAINTS["wheelbase"].default),
    )
    base["sub_dt"] = SUB_DT
    if wheelbase is not None:
        base["wheelbase"] = float(wheelbase)
    return base


def find_autonomous_start(data: dict) -> int:
    """解析開始 t_ns を返す (`lib._events.find_autonomous_start` 経由)。"""
    df_vel = data["vel"].rename(columns={"vx": "lon_vel"}) if "vx" in data["vel"].columns else data["vel"]
    return _find_autonomous_start(data["mode"], df_vel)


# N-step オープンループ解析


def _prepare_gt(data: dict, t0_ns: int, params: dict) -> dict:
    """rollout 評価の GT 準備。AUTONOMOUS 開始 (t0_ns) から制御コマンド系列末尾までを
    cmd タイムスタンプ上に補間した GT 系列・過去コマンド系列を dict で返す。
    """
    df_kin = to_seconds(data["kin"], t0_ns).sort_values("t").reset_index(drop=True)
    df_acc = to_seconds(data["acc"], t0_ns).sort_values("t").reset_index(drop=True)
    df_steer = to_seconds(data["steer"], t0_ns).sort_values("t").reset_index(drop=True)
    df_cmd_raw = data["cmd"].sort_values("t_ns").reset_index(drop=True)
    df_cmd = to_seconds(df_cmd_raw, t0_ns).sort_values("t").reset_index(drop=True)
    df_gear = data.get("gear")

    require_non_empty_df(df_cmd, name="/control/command/control_cmd", context=ROLL_OUT_CONTEXT)
    require_non_empty_df(df_kin, name="/localization/kinematic_state", context=ROLL_OUT_CONTEXT)

    t_lo = 0.0
    t_hi_candidates = [df_cmd["t"].max(), df_kin["t"].max()]
    if not df_acc.empty:
        t_hi_candidates.append(df_acc["t"].max())
    if not df_steer.empty:
        t_hi_candidates.append(df_steer["t"].max())
    t_hi = float(min(t_hi_candidates))

    max_delay_sec = max(params["acc_time_delay"], params["steer_time_delay"]) + SUB_DT
    df_cmd_full = df_cmd[(df_cmd["t"] >= t_lo - max_delay_sec) & (df_cmd["t"] <= t_hi)].reset_index(drop=True)
    df_cmd = df_cmd[(df_cmd["t"] >= t_lo) & (df_cmd["t"] <= t_hi)].reset_index(drop=True)
    df_kin = df_kin[(df_kin["t"] >= t_lo - 1) & (df_kin["t"] <= t_hi + 1)].reset_index(drop=True)
    df_acc = df_acc[(df_acc["t"] >= t_lo - 1) & (df_acc["t"] <= t_hi + 1)].reset_index(drop=True)
    df_steer = df_steer[(df_steer["t"] >= t_lo - 1) & (df_steer["t"] <= t_hi + 1)].reset_index(drop=True)

    require_non_empty_df(df_cmd, name="/control/command/control_cmd", context=ROLL_OUT_CONTEXT)
    require_non_empty_df(df_kin, name="/localization/kinematic_state", context=ROLL_OUT_CONTEXT)

    _t_kin_raw = df_kin["t"].values
    _yaw_raw = np.unwrap(df_kin["yaw"].values)
    _vx_map = np.gradient(df_kin["x"].values, _t_kin_raw)
    _vy_map = np.gradient(df_kin["y"].values, _t_kin_raw)
    _vy_body = -_vx_map * np.sin(_yaw_raw) + _vy_map * np.cos(_yaw_raw)
    _dt_mean = float(np.mean(np.diff(_t_kin_raw)))
    _half_win = max(3, int(round(ROLLING_SMOOTH_WINDOW_S / _dt_mean)))
    _win = 2 * _half_win + 1
    _vy_smooth = pd.Series(_vy_body).rolling(_win, center=True, min_periods=1).mean().values
    _ay_body = np.gradient(_vy_smooth, _t_kin_raw)
    _ay_smooth = pd.Series(_ay_body).rolling(_win, center=True, min_periods=1).mean().values
    df_kin = df_kin.copy()
    df_kin["vy_pos"] = _vy_smooth
    df_kin["ay_pos"] = _ay_smooth

    t_cmd = df_cmd["t"].values
    t_kin = df_kin["t"].values
    t_acc = df_acc["t"].values
    t_steer = df_steer["t"].values

    gt_x = np.interp(t_cmd, t_kin, df_kin["x"].values)
    gt_y = np.interp(t_cmd, t_kin, df_kin["y"].values)
    gt_yaw = np.interp(t_cmd, t_kin, np.unwrap(df_kin["yaw"].values))
    gt_vx = np.interp(t_cmd, t_kin, df_kin["vx"].values)
    gt_wz = np.interp(t_cmd, t_kin, df_kin["wz"].values)
    gt_vy = np.interp(t_cmd, t_kin, df_kin["vy_pos"].values)
    gt_ax = interp_or_zeros(t_cmd, t_acc, df_acc["ax"].values)
    gt_ay = np.interp(t_cmd, t_kin, df_kin["ay_pos"].values)
    gt_steer = interp_or_zeros(t_cmd, t_steer, df_steer["steer"].values)

    gt_dwz = np.gradient(gt_wz, t_cmd)

    _wb = params["wheelbase"]
    gt_steer_kinematic = np.where(
        gt_vx > KINEMATIC_STEER_VX_MIN,
        np.arctan(gt_wz * _wb / np.where(gt_vx > KINEMATIC_STEER_VX_MIN, gt_vx, 1.0)),
        gt_steer,
    )

    t_cmd_full = df_cmd_full["t"].values
    accel_des_full = df_cmd_full["accel_des"].values
    steer_des_full = df_cmd_full["steer_des"].values

    _sub_dt_h = SUB_DT
    _n_h = len(t_cmd)

    _iv_arr = np.diff(t_cmd)
    _nfull_arr = (_iv_arr / _sub_dt_h).astype(np.int32)
    _rem_arr = _iv_arr - _nfull_arr * _sub_dt_h

    _bad_iv = (_iv_arr <= BAD_INTERVAL_MIN_S) | (_iv_arr > BAD_INTERVAL_MAX_S)
    _bad_iv_cumsum = np.cumsum(np.concatenate([[0], _bad_iv.view(np.uint8)])).astype(np.intp)

    _acc_q_sz = round(params["acc_time_delay"] / _sub_dt_h)
    _steer_q_sz = round(params["steer_time_delay"] / _sub_dt_h)
    if _acc_q_sz > 0 and _n_h > 0:
        _off_acc = (_acc_q_sz - np.arange(_acc_q_sz)) * _sub_dt_h
        _tq_acc = t_cmd[:, None] - _off_acc[None, :]
        _acc_hist_all = np.interp(
            _tq_acc.ravel(), t_cmd_full, accel_des_full,
            left=accel_des_full[0], right=accel_des_full[-1],
        ).reshape(_n_h, _acc_q_sz)
    else:
        _acc_hist_all = np.empty((_n_h, 0), dtype=np.float64)
    if _steer_q_sz > 0 and _n_h > 0:
        _off_steer = (_steer_q_sz - np.arange(_steer_q_sz)) * _sub_dt_h
        _tq_steer = t_cmd[:, None] - _off_steer[None, :]
        _steer_hist_all = np.interp(
            _tq_steer.ravel(), t_cmd_full, steer_des_full,
            left=steer_des_full[0], right=steer_des_full[-1],
        ).reshape(_n_h, _steer_q_sz)
    else:
        _steer_hist_all = np.empty((_n_h, 0), dtype=np.float64)

    _cos_y_arr = np.cos(gt_yaw)
    _sin_y_arr = np.sin(gt_yaw)

    _accel_des_arr = df_cmd["accel_des"].values
    _steer_des_arr = df_cmd["steer_des"].values
    _valid_gear = require_drive_gear_mask(
        df_gear if df_gear is not None else pd.DataFrame(),
        df_cmd_raw["t_ns"].values,
        context=ROLL_OUT_CONTEXT,
        allow_leading_gap=True,
    )
    _bad_gear_cumsum = np.cumsum(np.concatenate([[0], (~_valid_gear).view(np.uint8)])).astype(np.intp)

    return {
        "t_cmd": t_cmd,
        "gt_x": gt_x, "gt_y": gt_y, "gt_yaw": gt_yaw, "gt_vx": gt_vx, "gt_wz": gt_wz,
        "gt_vy": gt_vy, "gt_ax": gt_ax, "gt_ay": gt_ay, "gt_steer": gt_steer, "gt_dwz": gt_dwz,
        "gt_steer_kinematic": gt_steer_kinematic,
        "iv_arr": _iv_arr, "nfull_arr": _nfull_arr, "rem_arr": _rem_arr,
        "bad_iv_cumsum": _bad_iv_cumsum,
        "acc_hist_all": _acc_hist_all, "steer_hist_all": _steer_hist_all,
        "cos_y_arr": _cos_y_arr, "sin_y_arr": _sin_y_arr,
        "accel_des_arr": _accel_des_arr, "steer_des_arr": _steer_des_arr,
        "valid_gear_arr": _valid_gear, "bad_gear_cumsum": _bad_gear_cumsum,
    }


def eval_rollout_rmse(
    data: dict,
    t0_ns: int,
    params: dict,
    model_type: str,
    horizons: tuple[int, ...],
    stride: int,
    gt: dict | None = None,
) -> dict[int, dict[str, float]]:
    """free-running N-step rollout の horizon 別終端誤差 RMSE を返す。

    返り値: {h: {"pos"[cm],"long"[cm],"lat"[cm],"yaw"[deg],"steer"[deg],"vx"[m/s],"ax"[m/s^2]}}
    """
    g = gt if gt is not None else _prepare_gt(data, t0_ns, params)

    t_cmd = g["t_cmd"]
    gt_x, gt_y, gt_yaw = g["gt_x"], g["gt_y"], g["gt_yaw"]
    gt_vx, gt_steer = g["gt_vx"], g["gt_steer"]
    gt_wz, gt_ax, gt_vy = g["gt_wz"], g["gt_ax"], g["gt_vy"]

    nfull_arr, rem_arr = g["nfull_arr"], g["rem_arr"]
    bc, bg = g["bad_iv_cumsum"], g["bad_gear_cumsum"]
    acc_hist_all, steer_hist_all = g["acc_hist_all"], g["steer_hist_all"]
    cos_y_arr, sin_y_arr = g["cos_y_arr"], g["sin_y_arr"]
    accel_des, steer_des = g["accel_des_arr"], g["steer_des_arr"]

    steer_bias = params["steer_bias"]
    model = VehicleModel(params, SUB_DT, model_type)

    n = len(t_cmd)
    sorted_horizons = sorted(horizons)
    min_h = sorted_horizons[0]

    gt_x_list = gt_x.tolist()
    gt_y_list = gt_y.tolist()
    gt_yaw_list = gt_yaw.tolist()
    gt_vx_list = gt_vx.tolist()
    gt_ax_list = gt_ax.tolist()
    gt_wz_list = gt_wz.tolist()
    gt_vy_list = gt_vy.tolist()
    bc_list = bc.tolist()
    bg_list = bg.tolist()

    n_acc = acc_hist_all.shape[1]
    n_steer = steer_hist_all.shape[1]
    acc_base = acc_hist_all.ctypes.data
    steer_base = steer_hist_all.ctypes.data

    _cdbl_p = ctypes.POINTER(ctypes.c_double)
    acc_ptrs = [ctypes.cast(acc_base + k0 * n_acc * 8, _cdbl_p) for k0 in range(n)]
    steer_ptrs = [ctypes.cast(steer_base + k0 * n_steer * 8, _cdbl_p) for k0 in range(n)]

    _cint_p = ctypes.POINTER(ctypes.c_int)
    _p_ad = accel_des.ctypes.data_as(_cdbl_p)
    _p_sd = steer_des.ctypes.data_as(_cdbl_p)
    _p_nf = nfull_arr.ctypes.data_as(_cint_p)
    _p_rem = rem_arr.ctypes.data_as(_cdbl_p)
    _n_sh = len(sorted_horizons)
    _h_arr = np.array(sorted_horizons, dtype=np.int32)
    _p_h = _h_arr.ctypes.data_as(_cint_p)

    _x_out = np.empty(_n_sh, dtype=np.float64)
    _y_out = np.empty(_n_sh, dtype=np.float64)
    _yaw_out = np.empty(_n_sh, dtype=np.float64)
    _vx_out = np.empty(_n_sh, dtype=np.float64)
    _ax_out = np.empty(_n_sh, dtype=np.float64)
    _steer_buf = np.empty(_n_sh, dtype=np.float64)
    _p_xo = _x_out.ctypes.data_as(_cdbl_p)
    _p_yo = _y_out.ctypes.data_as(_cdbl_p)
    _p_yawo = _yaw_out.ctypes.data_as(_cdbl_p)
    _p_vxo = _vx_out.ctypes.data_as(_cdbl_p)
    _p_axo = _ax_out.ctypes.data_as(_cdbl_p)
    _p_steero = _steer_buf.ctypes.data_as(_cdbl_p)

    k0_range = range(0, n - min_h, stride)
    num_steps = len(k0_range)
    sim_x = np.full((num_steps, _n_sh), np.nan, dtype=np.float64)
    sim_y = np.full((num_steps, _n_sh), np.nan, dtype=np.float64)
    sim_yaw = np.full((num_steps, _n_sh), np.nan, dtype=np.float64)
    sim_vx = np.full((num_steps, _n_sh), np.nan, dtype=np.float64)
    sim_ax = np.full((num_steps, _n_sh), np.nan, dtype=np.float64)
    sim_steer = np.full((num_steps, _n_sh), np.nan, dtype=np.float64)

    for step_idx, k0 in enumerate(k0_range):
        if gt_vx_list[k0] <= VX_MIN_CURVE:
            continue
        model.reset_with_history_ptr(
            x=gt_x_list[k0], y=gt_y_list[k0], yaw=gt_yaw_list[k0], vx=gt_vx_list[k0],
            steer_actual=gt_steer[k0] + steer_bias, ax=gt_ax_list[k0],
            acc_ptr=acc_ptrs[k0], n_acc=n_acc, steer_ptr=steer_ptrs[k0], n_steer=n_steer,
            wz=gt_wz_list[k0], vy=gt_vy_list[k0],
        )
        n_valid = 0
        for h in sorted_horizons:
            if k0 + h >= n:
                break
            if bc_list[k0 + h] > bc_list[k0]:
                break
            if bg_list[k0 + h + 1] > bg_list[k0]:
                break
            n_valid += 1
        if n_valid == 0:
            continue

        model._lib.vm_integrate_to_horizons(
            model._ptr, ctypes.c_int(sorted_horizons[n_valid - 1]), ctypes.c_int(k0),
            _p_ad, _p_sd, _p_nf, _p_rem, 1e-6, ctypes.c_int(n_valid), _p_h,
            _p_xo, _p_yo, _p_yawo, _p_vxo, _p_axo, _p_steero,
        )

        sim_x[step_idx, :n_valid] = _x_out[:n_valid]
        sim_y[step_idx, :n_valid] = _y_out[:n_valid]
        sim_yaw[step_idx, :n_valid] = _yaw_out[:n_valid]
        sim_vx[step_idx, :n_valid] = _vx_out[:n_valid]
        sim_ax[step_idx, :n_valid] = _ax_out[:n_valid]
        sim_steer[step_idx, :n_valid] = _steer_buf[:n_valid]

    res = {}
    k0_arr = np.array(list(k0_range), dtype=np.intp)
    for i, h in enumerate(sorted_horizons):
        valid_mask = ~np.isnan(sim_x[:, i])
        if not np.any(valid_mask):
            res[h] = {metric: float("inf") for metric in METRIC_KEYS}
            continue

        mx, my, myaw = sim_x[valid_mask, i], sim_y[valid_mask, i], sim_yaw[valid_mask, i]
        mvx, max_val, msteer = sim_vx[valid_mask, i], sim_ax[valid_mask, i], sim_steer[valid_mask, i]

        k_end_arr = k0_arr[valid_mask] + h
        g_x, g_y, g_yaw = gt_x[k_end_arr], gt_y[k_end_arr], gt_yaw[k_end_arr]
        g_vx, g_ax, g_steer = gt_vx[k_end_arr], gt_ax[k_end_arr], gt_steer[k_end_arr]
        g_x0, g_y0 = gt_x[k0_arr[valid_mask]], gt_y[k0_arr[valid_mask]]

        dx, dy = g_x - mx, g_y - my
        pos_err = np.hypot(dx, dy)
        yaw_err = ((g_yaw - myaw + np.pi) % (2 * np.pi)) - np.pi

        real_dx, real_dy = g_x - g_x0, g_y - g_y0
        sim_dx, sim_dy = mx - g_x0, my - g_y0
        cos_y, sin_y = cos_y_arr[k_end_arr], sin_y_arr[k_end_arr]

        real_ds_long, real_ds_lat = local_ds(real_dx, real_dy, cos_y, sin_y)
        sim_ds_long, sim_ds_lat = local_ds(sim_dx, sim_dy, cos_y, sin_y)

        ds_long_err = real_ds_long - sim_ds_long
        ds_lat_err = real_ds_lat - sim_ds_lat
        steer_err = g_steer - msteer
        vx_err = g_vx - mvx
        ax_err = g_ax - max_val

        res[h] = {
            "pos": rms(pos_err) * 100.0,
            "long": rms(ds_long_err) * 100.0,
            "lat": rms(ds_lat_err) * 100.0,
            "yaw": rms(np.degrees(yaw_err)),
            "steer": rms(steer_err) * 180.0 / np.pi,
            "vx": rms(vx_err),
            "ax": rms(ax_err),
        }

    return res
