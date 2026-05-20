#!/usr/bin/env python3
"""カーブ② 区間の per-step delta 分析（C++ 車両モデル使用）.

手法:
  各制御コマンド区間 (t_k, t_{k+1}) について:
    1. C++ 車両モデルを t_k の実機状態にリセット（累積誤差を排除）
    2. 実機制御コマンドを ZOH で適用
    3. 区間終端の予測状態を実機状態と比較（車両ローカル座標系）

使用モデル: DELAY_STEER_ACC_GEARED_WO_FALL_GUARD (C++ ctypes 経由)
  ライブラリ: libvehicle_model_wrapper.so (simple_sensor_simulator パッケージが提供)
  パラメータ: analysis/best_model_description/config/simulator_model.param.yaml

出力:
  comparison/curve2_per_step/
    overview.png, error_timeseries.png, error_vs_speed.png,
    map_distribution.png, summary.txt, per_step_delta.csv
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ._events import find_autonomous_start as _find_autonomous_start
from ._events import find_curve2_launch as _find_curve2_launch_window
from ._io import (
    align_time,
    iter_bag_messages,
    load_operation_mode,
    load_steering,
    load_velocity,
    resolve_topic,
)
from ._map import load_map_ways as _load_map_ways_impl
from ._map import resolve_map_osm
from ._params_utils import add_params_annotation, load_sim_params, setup_jp_font
from ._runtime_config import RuntimeConfig, add_common_cli_arguments, build_runtime_config

setup_jp_font()

# モジュールレベル設定 (main() で RuntimeConfig 経由で上書きされる)
BASE = Path(os.environ.get("BEST_MODEL_BASE_DIR") or Path(__file__).parent)
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison" / "curve2_per_step"

# 実機 lite bag は単一ファイルでも directory bag でも受け付ける。
REAL_BAG_DIR = LITE_DIR / "real.lite.mcap"

# カーブ②発進前後の解析窓
T_PRE = 3.0  # [s]
T_POST = 35.0  # [s]

# curve2 window モジュール変数 (main で上書き)
_CURVE2_WINDOW: tuple[float, float] = (20.0, 120.0)

# curve2 中心 (main で上書き)
_CURVE2_CX: int = 89301
_CURVE2_CY: int = 43085

# per_step 専用の上書き値 (load_sim_params() のシム既定値に上塗りする)
# 元コードは vehicle_info の wheel_base を 4.76012、PARAMS 内では明示で steer_bias=0.01
# (load_sim_params() の既定 0.0005 を per_step では意図的に大きく取る) を採用していた。
_PARAMS_OVERRIDES = {
    "steer_bias": 0.01,
    "sub_dt": 1.0 / 30.0,
}


def _build_params(cfg: RuntimeConfig | None = None) -> dict:
    """`vehicle_info.param.yaml` + per_step 専用上書きで PARAMS を構築する。

    `_PARAMS_OVERRIDES` は per_step 固有の意図的な差分なので維持する。
    """
    base = load_sim_params()
    base.setdefault("wheelbase", base.get("wheel_base", 4.76012))
    base.update(_PARAMS_OVERRIDES)
    if cfg is not None and cfg.wheelbase_sim:
        base["wheelbase"] = float(cfg.wheelbase_sim)
    return base


PARAMS = _build_params()
SUB_DT = PARAMS["sub_dt"]  # 積分ステップ幅 [s]（FMU_DT 相当）


# ---------------------------------------------------------------------------
# DELAY_STEER_ACC_GEARED_WO_FALL_GUARD の C++ ctypes ラッパー
# ---------------------------------------------------------------------------


def _resolve_so_path() -> Path:
    """libvehicle_model_wrapper.so の場所を解決する。

    優先順:
      1. VEHICLE_MODEL_SO_PATH 環境変数
      2. ament_index_python で best_model_comparison パッケージ share を解決
      3. ソース隣接 (ローカル開発用フォールバック)
    """
    env = os.environ.get("VEHICLE_MODEL_SO_PATH")
    if env:
        p = Path(env)
        if p.exists():
            return p
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("simple_sensor_simulator"))
        p = share / "libvehicle_model_wrapper.so"
        if p.exists():
            return p
    except Exception:  # noqa: BLE001
        pass
    return Path(__file__).parent / "libvehicle_model_wrapper.so"


def _load_lib() -> ctypes.CDLL:
    so = _resolve_so_path()
    if not so.exists():
        raise FileNotFoundError(
            f"{so} が見つかりません。simple_sensor_simulator を colcon build してください。"
        )
    lib = ctypes.CDLL(str(so))

    c_double = ctypes.c_double
    c_void_p = ctypes.c_void_p

    lib.vm_create.restype = c_void_p
    lib.vm_create.argtypes = [c_double] * 12

    lib.vm_reset_full.restype = None
    lib.vm_reset_full.argtypes = [c_void_p] + [c_double] * 6

    # State-only reset (queues untouched) + explicit queue setter
    lib.vm_reset_state.restype = None
    lib.vm_reset_state.argtypes = [c_void_p] + [c_double] * 6

    lib.vm_set_queues.restype = None
    lib.vm_set_queues.argtypes = [
        c_void_p,
        ctypes.POINTER(c_double),
        ctypes.c_int,
        ctypes.POINTER(c_double),
        ctypes.c_int,
    ]

    lib.vm_get_acc_q_size.restype = ctypes.c_int
    lib.vm_get_acc_q_size.argtypes = [c_void_p]
    lib.vm_get_steer_q_size.restype = ctypes.c_int
    lib.vm_get_steer_q_size.argtypes = [c_void_p]

    lib.vm_set_input.restype = None
    lib.vm_set_input.argtypes = [c_void_p, c_double, c_double]

    lib.vm_step.restype = None
    lib.vm_step.argtypes = [c_void_p]

    for fn in (
        "vm_get_x",
        "vm_get_y",
        "vm_get_yaw",
        "vm_get_vx",
        "vm_get_steer",
        "vm_get_ax",
        "vm_get_wz",
        "vm_get_vy",
        "vm_get_ay",
    ):
        getattr(lib, fn).restype = c_double
        getattr(lib, fn).argtypes = [c_void_p]

    lib.vm_step_dt.restype = None
    lib.vm_step_dt.argtypes = [c_void_p, c_double]

    lib.vm_destroy.restype = None
    lib.vm_destroy.argtypes = [c_void_p]

    return lib


class VehicleModel:
    """DELAY_STEER_ACC_GEARED_WO_FALL_GUARD の C++ ctypes ラッパー."""

    _lib: ctypes.CDLL | None = None

    @classmethod
    def _get_lib(cls) -> ctypes.CDLL:
        if cls._lib is None:
            cls._lib = _load_lib()
        return cls._lib

    def __init__(self, params: dict, sub_dt: float):
        p = params
        lib = self._get_lib()
        self._lib = lib
        self._ptr = lib.vm_create(
            p["vel_lim"],
            p["steer_lim"],
            p["vel_rate_lim"],
            p["steer_rate_lim"],
            p["wheelbase"],
            sub_dt,
            p["acc_time_delay"],
            p["acc_time_constant"],
            p["steer_time_delay"],
            p["steer_time_constant"],
            p["steer_dead_band"],
            p["steer_bias"],
        )
        self._sub_dt = sub_dt
        self._steer_bias = p["steer_bias"]

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr and hasattr(self, "_lib") and self._lib:
            self._lib.vm_destroy(self._ptr)
            self._ptr = None

    @property
    def acc_q_size(self) -> int:
        return self._lib.vm_get_acc_q_size(self._ptr)

    @property
    def steer_q_size(self) -> int:
        return self._lib.vm_get_steer_q_size(self._ptr)

    def reset_with_history(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        steer_actual: float,
        ax: float,
        acc_history: list[float],
        steer_history: list[float],
    ) -> None:
        """
        状態と delay queue を実際の過去コマンド履歴でリセット。

        acc_history   : accel_des [oldest→newest], len == acc_q_size
        steer_history : steer_des [oldest→newest], len == steer_q_size
        """
        self._lib.vm_reset_state(self._ptr, x, y, yaw, vx, steer_actual, ax)

        n_acc = len(acc_history)
        n_steer = len(steer_history)
        ArrAcc = (ctypes.c_double * n_acc)(*acc_history)
        ArrSteer = (ctypes.c_double * n_steer)(*steer_history)
        self._lib.vm_set_queues(
            self._ptr,
            ArrAcc,
            ctypes.c_int(n_acc),
            ArrSteer,
            ctypes.c_int(n_steer),
        )

    def step(self, accel_des: float, steer_des: float) -> None:
        """Euler 1 ステップ積分（sub_dt 秒）。"""
        self._lib.vm_set_input(self._ptr, accel_des, steer_des)
        self._lib.vm_step(self._ptr)

    def step_dt(self, accel_des: float, steer_des: float, dt: float) -> None:
        """
        Euler 1 ステップ積分（任意 dt 秒）。delay queue は sub_dt 単位で設計されているため
        端数補正にのみ使用し、dt は SUB_DT より十分小さい範囲で呼ぶこと。
        """
        self._lib.vm_set_input(self._ptr, accel_des, steer_des)
        self._lib.vm_step_dt(self._ptr, ctypes.c_double(dt))

    @property
    def x(self) -> float:
        return self._lib.vm_get_x(self._ptr)

    @property
    def y(self) -> float:
        return self._lib.vm_get_y(self._ptr)

    @property
    def yaw(self) -> float:
        return self._lib.vm_get_yaw(self._ptr)

    @property
    def vx(self) -> float:
        return self._lib.vm_get_vx(self._ptr)

    @property
    def wz(self) -> float:
        return self._lib.vm_get_wz(self._ptr)

    @property
    def vy(self) -> float:
        return self._lib.vm_get_vy(self._ptr)

    @property
    def ay(self) -> float:
        return self._lib.vm_get_ay(self._ptr)

    @property
    def steer_state(self) -> float:
        return self._lib.vm_get_steer(self._ptr) - self._steer_bias


# ---------------------------------------------------------------------------
# データ読み込み
# ---------------------------------------------------------------------------


def load_real_bag(path: Path) -> dict[str, pd.DataFrame]:
    """real lite bag から必要トピックを読み込む（sub-less / sub-prefixed 両対応）。"""
    df_mode = load_operation_mode(path)
    df_vel = load_velocity(path).rename(columns={"lon_vel": "vx"})
    df_steer = load_steering(path)

    kin_topic = resolve_topic(
        path,
        ["/localization/kinematic_state", "/sub/localization/kinematic_state"],
    )
    rows_kin = []
    if kin_topic is not None:
        for t_ns, ros in iter_bag_messages(path, [kin_topic]):
            p = ros.pose.pose.position
            o = ros.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (o.w * o.z + o.x * o.y),
                1.0 - 2.0 * (o.y * o.y + o.z * o.z),
            )
            rows_kin.append({
                "t_ns": t_ns,
                "x": p.x,
                "y": p.y,
                "yaw": yaw,
                "vx": ros.twist.twist.linear.x,
                "vy": ros.twist.twist.linear.y,
                "wz": ros.twist.twist.angular.z,
            })

    acc_topic = resolve_topic(
        path,
        ["/localization/acceleration", "/sub/localization/acceleration"],
    )
    rows_acc = []
    if acc_topic is not None:
        for t_ns, ros in iter_bag_messages(path, [acc_topic]):
            rows_acc.append({
                "t_ns": t_ns,
                "ax": ros.accel.accel.linear.x,
                "ay": ros.accel.accel.linear.y,
            })

    cmd_topic = resolve_topic(
        path,
        ["/control/command/control_cmd", "/sub/control/command/control_cmd"],
    )
    rows_cmd = []
    if cmd_topic is not None:
        for t_ns, ros in iter_bag_messages(path, [cmd_topic]):
            rows_cmd.append({
                "t_ns": t_ns,
                "accel_des": ros.longitudinal.acceleration,
                "steer_des": ros.lateral.steering_tire_angle,
            })

    return {
        "mode": df_mode,
        "vel": df_vel,
        "steer": df_steer,
        "kin": pd.DataFrame(rows_kin),
        "acc": pd.DataFrame(rows_acc),
        "cmd": pd.DataFrame(rows_cmd),
    }


def find_autonomous_start(data: dict) -> int:
    """AUTONOMOUS モードが最初に現れる t_ns を返す (`_events.find_autonomous_start` 経由)。"""
    # `_events.find_autonomous_start` は `lon_vel` 列を想定するが、ここでは `vx` 列なので
    # 一時的にリネームしてフォールバック用 DF を作る。
    df_vel = data["vel"].rename(columns={"vx": "lon_vel"}) if "vx" in data["vel"].columns else data["vel"]
    return _find_autonomous_start(data["mode"], df_vel)


def find_curve2_launch(df_vel: pd.DataFrame) -> float | None:
    """カーブ② 直前停止からの発進時刻を返す (`_events.find_curve2_launch` 経由)。

    df_vel は列 't', 'vx' を持つ前提（per_step 内部表記）。
    """
    df = df_vel.rename(columns={"vx": "lon_vel"})
    return _find_curve2_launch_window(df, window=_CURVE2_WINDOW)


# ---------------------------------------------------------------------------
# per-step delta 分析
# ---------------------------------------------------------------------------


def run_per_step(data: dict, t0_ns: int, t_launch: float, params: dict) -> pd.DataFrame:
    """per-step delta を実行し結果 DataFrame を返す。"""

    # -- タイムスタンプを秒に変換 --
    def to_sec(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()
        df["t"] = (df["t_ns"] - t0_ns) / 1e9
        return df.drop(columns=["t_ns"])

    df_kin = to_sec(data["kin"]).sort_values("t").reset_index(drop=True)
    df_acc = to_sec(data["acc"]).sort_values("t").reset_index(drop=True)
    df_steer = to_sec(data["steer"]).sort_values("t").reset_index(drop=True)
    df_cmd = to_sec(data["cmd"]).sort_values("t").reset_index(drop=True)

    t_lo = t_launch - T_PRE
    t_hi = t_launch + T_POST

    # delay queue 分だけ過去コマンドも取得できるよう cmd 窓を広げる
    max_delay_sec = max(params["acc_time_delay"], params["steer_time_delay"]) + SUB_DT
    df_cmd_full = df_cmd[(df_cmd["t"] >= t_lo - max_delay_sec) & (df_cmd["t"] <= t_hi)].reset_index(
        drop=True
    )
    df_cmd = df_cmd[(df_cmd["t"] >= t_lo) & (df_cmd["t"] <= t_hi)].reset_index(drop=True)
    df_kin = df_kin[(df_kin["t"] >= t_lo - 1) & (df_kin["t"] <= t_hi + 1)].reset_index(drop=True)
    df_acc = df_acc[(df_acc["t"] >= t_lo - 1) & (df_acc["t"] <= t_hi + 1)].reset_index(drop=True)
    df_steer = df_steer[(df_steer["t"] >= t_lo - 1) & (df_steer["t"] <= t_hi + 1)].reset_index(
        drop=True
    )

    if df_cmd.empty:
        raise RuntimeError("制御コマンドが空です")
    if df_kin.empty:
        raise RuntimeError("運動学状態が空です")

    # -- 位置微分による body frame 横速度・横加速度の計算 --
    # EKF の vy/ay は no-slip 拘束によりほぼゼロになるため、
    # pose position を微分して body frame に変換する。
    _t_kin_raw = df_kin["t"].values
    _yaw_raw = np.unwrap(df_kin["yaw"].values)
    _vx_map = np.gradient(df_kin["x"].values, _t_kin_raw)
    _vy_map = np.gradient(df_kin["y"].values, _t_kin_raw)
    _vy_body = -_vx_map * np.sin(_yaw_raw) + _vy_map * np.cos(_yaw_raw)
    _dt_mean = float(np.mean(np.diff(_t_kin_raw)))
    _half_win = max(3, int(round(0.3 / _dt_mean)))  # 片側約 0.3s
    _win = 2 * _half_win + 1
    _vy_smooth = pd.Series(_vy_body).rolling(_win, center=True, min_periods=1).mean().values
    _ay_body = np.gradient(_vy_smooth, _t_kin_raw)
    _ay_smooth = pd.Series(_ay_body).rolling(_win, center=True, min_periods=1).mean().values
    df_kin = df_kin.copy()
    df_kin["vy_pos"] = _vy_smooth
    df_kin["ay_pos"] = _ay_smooth

    # -- GT を cmd タイムスタンプに線形補間 --
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
    gt_ax = (
        np.interp(t_cmd, t_acc, df_acc["ax"].values) if not df_acc.empty else np.zeros_like(t_cmd)
    )
    gt_ay = np.interp(t_cmd, t_kin, df_kin["ay_pos"].values)
    gt_steer = (
        np.interp(t_cmd, t_steer, df_steer["steer"].values)
        if not df_steer.empty
        else np.zeros_like(t_cmd)
    )

    # 角加速度: wz の時間微分
    gt_dwz = np.gradient(gt_wz, t_cmd)

    # 運動学ステア: ego_entity_simulation.cpp と同じ初期化方式
    # state(4) = atan(wz * wb / vx)。低速では sensor 値にフォールバック。
    _wb = params["wheelbase"]
    _vx_thresh = 0.5  # [m/s]
    gt_steer_kinematic = np.where(
        gt_vx > _vx_thresh,
        np.arctan(gt_wz * _wb / np.where(gt_vx > _vx_thresh, gt_vx, 1.0)),
        gt_steer,
    )

    # 過去コマンド補間用（queue 分の過去を含む全区間）
    t_cmd_full = df_cmd_full["t"].values
    accel_des_full = df_cmd_full["accel_des"].values
    steer_des_full = df_cmd_full["steer_des"].values

    model = VehicleModel(params, SUB_DT)
    acc_q_size = model.acc_q_size  # = round(acc_time_delay / SUB_DT)
    steer_q_size = model.steer_q_size  # = round(steer_time_delay / SUB_DT)

    records = []
    n = len(t_cmd)
    for k in range(n - 1):
        interval = t_cmd[k + 1] - t_cmd[k]
        if interval <= 0.001 or interval > 1.0:
            continue

        accel_des = float(df_cmd["accel_des"].iloc[k])
        steer_des = float(df_cmd["steer_des"].iloc[k])

        # -- delay queue に実際の過去コマンド履歴を設定 --
        # acc_q[i] = accel_des at t_k - (acc_q_size - i) * SUB_DT  (oldest→newest)
        # update() はこの oldest を delayed として消費する
        acc_history = [
            float(
                np.interp(
                    t_cmd[k] - (acc_q_size - i) * SUB_DT,
                    t_cmd_full,
                    accel_des_full,
                    left=accel_des_full[0],
                    right=accel_des_full[-1],
                )
            )
            for i in range(acc_q_size)
        ]
        steer_history = [
            float(
                np.interp(
                    t_cmd[k] - (steer_q_size - i) * SUB_DT,
                    t_cmd_full,
                    steer_des_full,
                    left=steer_des_full[0],
                    right=steer_des_full[-1],
                )
            )
            for i in range(steer_q_size)
        ]

        # -- モデルを t_k の実機状態にリセット（過去履歴を delay queue にセット） --
        # ego_entity_simulation.cpp と同じ: state(4) = atan(wz*wb/vx) をキネマティック初期値として使用。
        # vm_reset_state 内: state(4) = steer_actual - steer_bias → steer_actual = steer_kinematic + steer_bias
        model.reset_with_history(
            x=gt_x[k],
            y=gt_y[k],
            yaw=gt_yaw[k],
            vx=gt_vx[k],
            steer_actual=float(gt_steer_kinematic[k]) + params["steer_bias"],
            ax=gt_ax[k],
            acc_history=acc_history,
            steer_history=steer_history,
        )

        # -- interval 分だけ正確に積分 --
        # n_full 回 SUB_DT ステップ + 余り時間を端数ステップで補正し、
        # モデル積分時間が実際の elapsed time と一致するようにする。
        n_full = int(interval / SUB_DT)
        for i in range(n_full):
            model.step(accel_des, steer_des)
        remainder = interval - n_full * SUB_DT
        if remainder > 1e-6:
            model.step_dt(accel_des, steer_des, remainder)

        # -- delta 計算 --
        real_dx = gt_x[k + 1] - gt_x[k]
        real_dy = gt_y[k + 1] - gt_y[k]
        sim_dx = model.x - gt_x[k]
        sim_dy = model.y - gt_y[k]

        cos_y = math.cos(gt_yaw[k])
        sin_y = math.sin(gt_yaw[k])
        real_ds_long = real_dx * cos_y + real_dy * sin_y
        real_ds_lat = -real_dx * sin_y + real_dy * cos_y
        sim_ds_long = sim_dx * cos_y + sim_dy * sin_y
        sim_ds_lat = -sim_dx * sin_y + sim_dy * cos_y

        sim_steer_kp1 = model.steer_state + params["steer_bias"]
        sim_wz = model.wz
        sim_vy = model.vy
        sim_ay = model.ay
        records.append(
            {
                "timestamp": t_cmd[k],
                "tr": t_cmd[k] - t_launch,
                "accel_des": accel_des,
                "steer_des": steer_des,
                "interval_sec": interval,
                "pos_x": gt_x[k],
                "pos_y": gt_y[k],
                "real_vx": gt_vx[k],
                "real_steer_k": gt_steer[k],  # t_k の実機ステア（リセット時の初期値）
                "real_steer_kp1": gt_steer[k + 1],  # t_{k+1} の実機ステア（予測の比較対象）
                "sim_steer_kp1": sim_steer_kp1,  # モデルが予測した t_{k+1} のステア
                "err_steer": gt_steer[k + 1] - sim_steer_kp1,  # 予測誤差 [rad]
                "real_ax": gt_ax[k],
                "real_ay": gt_ay[k + 1],  # t_{k+1} — err_steer 規約に統一
                "real_vy": gt_vy[k + 1],
                "real_wz": gt_wz[k + 1],
                "real_dwz": gt_dwz[k + 1],
                "real_wz_k": gt_wz[k],  # t_k — 散布図の Y 軸（ステア角と同時刻）
                "real_ay_k": gt_ay[k],
                "real_vy_k": gt_vy[k],
                "sim_vx": model.vx,
                "sim_ay": sim_ay,
                "sim_vy": sim_vy,
                "sim_wz": sim_wz,
                "err_ay": gt_ay[k + 1] - sim_ay,
                "err_vy": gt_vy[k + 1] - sim_vy,
                "err_wz": gt_wz[k + 1] - sim_wz,
                "real_ds_long": real_ds_long,
                "real_ds_lat": real_ds_lat,
                "sim_ds_long": sim_ds_long,
                "sim_ds_lat": sim_ds_lat,
                "err_ds_long": real_ds_long - sim_ds_long,
                "err_ds_lat": real_ds_lat - sim_ds_lat,
            }
        )

        if (k + 1) % 200 == 0:
            print(f"  {k + 1}/{n - 1} ...")

    return pd.DataFrame(records)


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------


def _save(fig: plt.Figure, name: str) -> None:
    path = OUT_DIR / f"{name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


def _set_title(
    ax: plt.Axes, title: str, source: str, title_fs: int = 9, source_fs: float = 6.5
) -> None:
    """
    サブプロットタイトルとデータソース注を別フォントサイズで表示する。

    source は小フォント・グレーでタイトル直下に配置する。
    """
    ax.set_title(title, fontsize=title_fs, pad=18)
    ax.text(
        0.5,
        1.0,
        source,
        transform=ax.transAxes,
        fontsize=source_fs,
        ha="center",
        va="bottom",
        color="#888888",
        clip_on=False,
    )


def plot_overview(df: pd.DataFrame, params: dict) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    # 速度
    ax = axes[0, 0]
    ax.plot(tr, df["real_vx"].values, color="blue", lw=1.2, label="実機 vx")
    ax.plot(tr, df["sim_vx"].values, color="red", lw=1.0, ls="--", label="モデル vx")
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("速度 [m/s]")
    _set_title(
        ax, "速度: 実機 vs モデル", "実機: kinematic_state/twist.linear.x  モデル: state_[3]"
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 加速度指令 vs 実機 ax
    ax = axes[0, 1]
    ax.plot(tr, df["accel_des"].values, color="gray", lw=0.8, label="指令 accel_des")
    ax.plot(tr, df["real_ax"].values, color="blue", lw=1.0, label="実機 ax")
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("加速度 [m/s²]")
    _set_title(
        ax,
        "加速度: 指令 vs 実機",
        "指令: control_cmd/longitudinal.acceleration  実機: acceleration/accel.linear.x",
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 縦方向誤差
    ax = axes[1, 0]
    err_s = df["err_ds_long"] * 100
    ma = err_s.rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, err_s.values, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma, color="red", lw=1.5, label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("縦方向誤差 [cm]")
    _set_title(
        ax, "1ステップ縦方向誤差", "実機: kinematic_state/pose.position  モデル: state_[0,1]"
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 横方向誤差
    ax = axes[1, 1]
    err_s = df["err_ds_lat"] * 100
    ma = err_s.rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, err_s.values, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma, color="red", lw=1.5, label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("横方向誤差 [cm]")
    _set_title(
        ax, "1ステップ横方向誤差", "実機: kinematic_state/pose.position  モデル: state_[0,1]"
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(
        "カーブ② per-step delta 分析\n(各ステップで実機状態にリセット — 計画挙動の差を除外)",
        fontsize=11,
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "overview")


def plot_error_timeseries(df: pd.DataFrame, params: dict) -> None:
    rad2deg = 180.0 / math.pi
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    for ax, vals, ylabel, title, color, source in [
        (
            axes[0],
            df["err_ds_long"].values * 100,
            "縦方向誤差 [cm]",
            "1ステップ縦方向位置誤差 (実機 − モデル)",
            "red",
            "実機: kinematic_state/pose.position  モデル: state_[0,1]",
        ),
        (
            axes[1],
            df["err_ds_lat"].values * 100,
            "横方向誤差 [cm]",
            "1ステップ横方向位置誤差 (実機 − モデル)",
            "red",
            "実機: kinematic_state/pose.position  モデル: state_[0,1]",
        ),
        (
            axes[2],
            df["err_steer"].values * rad2deg,
            "ステア予測誤差 [deg]",
            "1ステップステア予測誤差 (actual[k+1] − pred[k+1])",
            "purple",
            "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias",
        ),
    ]:
        ma = pd.Series(vals).rolling(window, center=True, min_periods=1).mean().values
        ax.plot(tr, vals, color="gray", lw=0.4, alpha=0.4, label="raw")
        ax.plot(tr, ma, color=color, lw=1.5, label=f"移動平均(w={window})")
        ax.axhline(0, color="black", lw=0.8)
        ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
        ax.set_ylabel(ylabel)
        _set_title(ax, title, source)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[2].set_xlabel("発進からの時刻 [s]")
    fig.suptitle("カーブ② per-step delta 誤差時系列", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "error_timeseries")


def plot_error_vs_speed(df: pd.DataFrame, params: dict) -> None:
    rad2deg = 180.0 / math.pi
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    vx = df["real_vx"].values
    speed_bins = [0.0, 2.0, 5.0, 8.0, 50.0]
    colors = ["#4472C4", "#ED7D31", "#A9D18E", "#FF0000"]

    for ax, vals, ylabel, source in [
        (
            axes[0],
            df["err_ds_long"].values * 100,
            "縦方向誤差 [cm]",
            "実機: kinematic_state/pose.position  モデル: state_[0,1]  速度: twist.linear.x",
        ),
        (
            axes[1],
            df["err_ds_lat"].values * 100,
            "横方向誤差 [cm]",
            "実機: kinematic_state/pose.position  モデル: state_[0,1]  速度: twist.linear.x",
        ),
        (
            axes[2],
            df["err_steer"].values * rad2deg,
            "ステア予測誤差 [deg]",
            "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias  速度: twist.linear.x",
        ),
    ]:
        for i, (lo, hi) in enumerate(zip(speed_bins[:-1], speed_bins[1:])):
            mask = (vx >= lo) & (vx < hi)
            if mask.sum() == 0:
                continue
            lbl = f"v={lo:.0f}–{hi:.0f} m/s" if hi < 50 else f"v≥{lo:.0f} m/s"
            ax.scatter(
                vx[mask], vals[mask], s=4, alpha=0.5, color=colors[i % len(colors)], label=lbl
            )
        ax.axhline(0, color="black", lw=0.8)
        ax.set_xlabel("速度 [m/s]")
        ax.set_ylabel(ylabel)
        _set_title(ax, f"{ylabel} vs 速度", source)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig.suptitle("カーブ② per-step delta: 速度依存性", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "error_vs_speed")


def plot_steering_analysis(df: pd.DataFrame, params: dict) -> None:
    """ステア 1ステップ予測の詳細分析（4パネル）."""
    rad2deg = 180.0 / math.pi
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # --- (0,0) ステア角の時系列: 実機 t_{k+1} vs モデル予測 t_{k+1} vs 指令 ---
    ax = axes[0, 0]
    ax.plot(
        tr, df["real_steer_kp1"].values * rad2deg, color="blue", lw=1.2, label="実機 steer[k+1]"
    )
    ax.plot(
        tr,
        df["sim_steer_kp1"].values * rad2deg,
        color="red",
        lw=1.0,
        ls="--",
        label="予測 steer[k+1]",
    )
    ax.plot(
        tr,
        df["steer_des"].values * rad2deg,
        color="gray",
        lw=0.7,
        ls=":",
        label="指令 steer_des[k]",
    )
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("ステア角 [deg]")
    _set_title(
        ax,
        "ステア角: 実機[k+1] vs モデル予測[k+1] vs 指令[k]",
        "実機: steering_status/tire_angle  モデル: state_[4]+bias  指令: control_cmd/lat.tire_angle",
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- (0,1) ステア追従誤差（指令 vs 実機）: 実機のステア制御性能 ---
    ax = axes[0, 1]
    steer_follow_err = (df["real_steer_kp1"].values - df["steer_des"].values) * rad2deg
    ma_f = pd.Series(steer_follow_err).rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, steer_follow_err, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma_f, color="blue", lw=1.5, label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("追従誤差 [deg]")
    _set_title(
        ax,
        "実機ステア追従誤差 (actual[k+1] − cmd[k])",
        "実機: steering_status/tire_angle  指令: control_cmd/lat.tire_angle",
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- (1,0) ステア予測誤差の時系列 ---
    ax = axes[1, 0]
    err_deg = df["err_steer"].values * rad2deg
    ma_e = pd.Series(err_deg).rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, err_deg, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma_e, color="red", lw=1.5, label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
    rmse_deg = float(np.sqrt(np.mean(err_deg**2)))
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_ylabel("予測誤差 [deg]")
    _set_title(
        ax,
        f"1ステップ ステア予測誤差 (actual[k+1] − pred[k+1])  RMSE={rmse_deg:.4f}°",
        "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias",
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- (1,1) ステア予測誤差 vs 指令ステア角（大入力時の精度確認）---
    ax = axes[1, 1]
    cmd_deg = df["steer_des"].values * rad2deg
    speed_bins = [0.0, 2.0, 5.0, 8.0, 50.0]
    colors = ["#4472C4", "#ED7D31", "#A9D18E", "#FF0000"]
    vx = df["real_vx"].values
    for i, (lo, hi) in enumerate(zip(speed_bins[:-1], speed_bins[1:])):
        mask = (vx >= lo) & (vx < hi)
        if mask.sum() == 0:
            continue
        lbl = f"v={lo:.0f}–{hi:.0f} m/s" if hi < 50 else f"v≥{lo:.0f} m/s"
        ax.scatter(
            cmd_deg[mask], err_deg[mask], s=4, alpha=0.5, color=colors[i % len(colors)], label=lbl
        )
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xlabel("指令ステア角 [deg]")
    ax.set_ylabel("予測誤差 [deg]")
    _set_title(
        ax,
        "ステア予測誤差 vs 指令ステア角（色=速度域）",
        "誤差: steering_status/tire_angle − state_[4]+bias  指令: control_cmd/lat.tire_angle",
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(
        "カーブ② per-step ステアリング分析\n(1ステップ予測誤差: actual[k+1] − model_pred[k+1])",
        fontsize=11,
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "steering_analysis")


def _resolve_real_mcap(bag_dir: Path) -> Path:
    """rosbag2 directory bag から実 .mcap ファイルパスを取得する。

    - 単一の `.mcap` ファイルならそのまま返す。
    - ディレクトリならその中の `.mcap` を返す。
    - 入力が `<stem>.lite.mcap` で存在しない場合は `<stem>.lite` ディレクトリも試す。
    """
    if bag_dir.is_file() and bag_dir.suffix == ".mcap":
        return bag_dir
    if bag_dir.is_dir():
        mcaps = sorted(bag_dir.glob("*.mcap"))
        if not mcaps:
            raise FileNotFoundError(f"No .mcap inside {bag_dir}")
        return mcaps[0]
    # `.lite.mcap` 名のディレクトリ形式 fallback
    if bag_dir.suffix == ".mcap":
        alt = bag_dir.with_suffix("")  # `real.lite`
        if alt.is_dir():
            mcaps = sorted(alt.glob("*.mcap"))
            if mcaps:
                return mcaps[0]
    raise FileNotFoundError(f"Real bag not found: {bag_dir}")


def _resolve_map_osm() -> Path | None:
    """lanelet2_map.osm を `_map.resolve_map_osm` の三状態モデルで解決する。"""
    return resolve_map_osm(os.environ.get("MAP_OSM_PATH"))


def _load_map_ways(osm_path: Path | None) -> list[np.ndarray] | None:
    if osm_path is None:
        return None
    ways = _load_map_ways_impl(osm_path)
    return ways if ways else None


def plot_map_distribution(df: pd.DataFrame, params: dict) -> None:
    rad2deg = 180.0 / math.pi
    map_ways = _load_map_ways(_resolve_map_osm())

    columns = [
        (
            df["err_ds_long"].values * 100,
            "縦方向誤差",
            "cm",
            "kinematic_state/pose.position vs state_[0,1]",
        ),
        (
            df["err_ds_lat"].values * 100,
            "横方向誤差",
            "cm",
            "kinematic_state/pose.position vs state_[0,1]",
        ),
        (
            df["err_steer"].values * rad2deg,
            "ステア予測誤差",
            "deg",
            "steering_status/tire_angle vs state_[4]+bias",
        ),
        (df["err_ay"].values, "横加速度誤差", "m/s²", "acceleration/accel.linear.y vs vx·wz"),
        (df["err_vy"].values, "横速度誤差", "m/s", "kinematic_state/twist.linear.y vs 0.0"),
        (
            df["err_wz"].values,
            "角速度誤差",
            "rad/s",
            "kinematic_state/twist.angular.z vs vx·tan(δ)/wb",
        ),
    ]

    fig, axes = plt.subplots(1, 6, figsize=(36, 6))
    cx, cy = _CURVE2_CX, _CURVE2_CY

    for ax, (vals, label, unit, source) in zip(axes, columns):
        if map_ways:
            for pts in map_ways:
                wx, wy = pts[:, 0], pts[:, 1]
                if wx.max() < cx - 80 or wx.min() > cx + 80:
                    continue
                if wy.max() < cy - 80 or wy.min() > cy + 80:
                    continue
                ax.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

        vmax = max(abs(vals).max(), 1e-6)
        sc = ax.scatter(
            df["pos_x"], df["pos_y"], c=vals, cmap="RdBu_r", vmin=-vmax, vmax=vmax, s=8, zorder=3
        )
        plt.colorbar(sc, ax=ax, label=unit)
        ax.set_xlim(cx - 80, cx + 80)
        ax.set_ylim(cy - 80, cy + 80)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        _set_title(ax, f"{label} [{unit}]", source)
        ax.grid(True, lw=0.5, alpha=0.5)

    fig.suptitle("カーブ② per-step delta: 地図上の誤差分布 (実機 − モデル)", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "map_distribution")


def plot_lateral_dynamics_timeseries(df: pd.DataFrame, params: dict) -> None:
    """横方向諸量の時系列: ay / vy / wz / dwz の実機 vs モデル."""
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    rows = [
        (
            "real_ay",
            "sim_ay",
            "横加速度 ay [m/s²]",
            "横加速度 ay: 実機 vs モデル",
            "実機: 位置微分(2階)→移動平均スムージング  モデル: vx·wz (求心加速度)",
        ),
        (
            "real_vy",
            "sim_vy",
            "横速度 vy [m/s]",
            "横速度 vy: 実機 vs モデル",
            "実機: 位置微分→body frame変換→移動平均スムージング  モデル: vy状態変数",
        ),
        (
            "real_wz",
            "sim_wz",
            "角速度 wz [rad/s]",
            "角速度 wz: 実機 vs モデル",
            "実機: kinematic_state/twist.angular.z  モデル: vx·tan(δ)/wb",
        ),
        (
            "real_dwz",
            None,
            "角加速度 dwz [rad/s²]",
            "角加速度 dwz: 実機",
            "実機: d/dt(kinematic_state/twist.angular.z)  np.gradient",
        ),
    ]
    fig, axes = plt.subplots(4, 1, figsize=(13, 16), sharex=True)

    for ax, (real_col, sim_col, ylabel, title, source) in zip(axes, rows):
        real_vals = df[real_col].values
        ma_real = pd.Series(real_vals).rolling(window, center=True, min_periods=1).mean().values
        ax.plot(tr, real_vals, color="#aaaaaa", lw=0.4, alpha=0.5, label="実機 raw")
        ax.plot(tr, ma_real, color="black", lw=1.5, label="実機 MA")
        if sim_col:
            sim_vals = df[sim_col].values
            ma_sim = pd.Series(sim_vals).rolling(window, center=True, min_periods=1).mean().values
            ax.plot(tr, sim_vals, color="#ffaaaa", lw=0.4, alpha=0.5)
            ax.plot(tr, ma_sim, color="red", lw=1.5, ls="--", label="モデル MA")
        ax.axhline(0, color="black", lw=0.6, ls=":")
        ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
        ax.set_ylabel(ylabel)
        _set_title(ax, title, source)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("発進からの時刻 [s]")
    fig.suptitle(
        "カーブ② per-step delta: 横方向諸量 時系列\n(各ステップで実機状態にリセット)", fontsize=11
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "lateral_dynamics_timeseries")


def plot_steer_vs_lateral_scatter(df: pd.DataFrame, params: dict) -> None:
    """横軸ステア角 × 縦軸横方向諸量の散布図（速度域別・1次フィッティング付き）."""
    rad2deg = 180.0 / math.pi
    steer_deg = df["real_steer_k"].values * rad2deg
    vx = df["real_vx"].values
    speed_bins = [0.0, 2.0, 5.0, 8.0, 50.0]
    colors = ["#4472C4", "#ED7D31", "#A9D18E", "#FF0000"]

    cols = [
        (
            "real_ay",
            "sim_ay",
            "横加速度 ay [m/s²]",
            "横軸: steering_status/tire_angle(t_k)  縦軸: 位置微分(2階) / vx·wz",
        ),
        (
            "real_vy",
            "sim_vy",
            "横速度 vy [m/s]",
            "横軸: steering_status/tire_angle(t_k)  縦軸: 位置微分body frame / vy状態変数",
        ),
        (
            "real_wz",
            "sim_wz",
            "角速度 wz [rad/s]",
            "横軸: steering_status/tire_angle(t_k)  縦軸: kinematic_state/twist.angular.z / vx·tan(δ)/wb",
        ),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    for ax, (real_col, sim_col, ylabel, source) in zip(axes, cols):
        real_vals = df[real_col].values
        sim_vals = df[sim_col].values if sim_col else None

        for i, (lo, hi) in enumerate(zip(speed_bins[:-1], speed_bins[1:])):
            mask = (vx >= lo) & (vx < hi)
            if mask.sum() == 0:
                continue
            lbl = f"v={lo:.0f}–{hi:.0f} m/s" if hi < 50 else f"v≥{lo:.0f} m/s"
            ax.scatter(
                steer_deg[mask],
                real_vals[mask],
                s=4,
                alpha=0.5,
                color=colors[i % len(colors)],
                label=lbl,
            )

        # 1次フィッティング（実機全点）
        x_fit = np.linspace(steer_deg.min(), steer_deg.max(), 100)
        valid = np.isfinite(steer_deg) & np.isfinite(real_vals)
        if valid.sum() >= 2:
            coeffs = np.polyfit(steer_deg[valid], real_vals[valid], 1)
            ax.plot(
                x_fit,
                np.polyval(coeffs, x_fit),
                color="black",
                lw=2,
                label=f"実機 fit: {coeffs[0]:.4f}·θ + {coeffs[1]:.4f}",
            )

        # シムの1次フィッティング
        if sim_vals is not None:
            valid_s = np.isfinite(steer_deg) & np.isfinite(sim_vals)
            if valid_s.sum() >= 2:
                coeffs_s = np.polyfit(steer_deg[valid_s], sim_vals[valid_s], 1)
                ax.plot(
                    x_fit,
                    np.polyval(coeffs_s, x_fit),
                    color="red",
                    lw=1.5,
                    ls="--",
                    label=f"モデル fit: {coeffs_s[0]:.4f}·θ",
                )

        ax.axhline(0, color="black", lw=0.6)
        ax.axvline(0, color="black", lw=0.6)
        ax.set_xlabel("ステア角 [deg]")
        ax.set_ylabel(ylabel)
        _set_title(ax, f"ステア角 vs {ylabel}", source)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        "カーブ② per-step delta: ステア角 vs 横方向諸量 散布図\n(スケール誤差・バイアス確認)",
        fontsize=11,
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "steer_vs_lateral_scatter")


def plot_cascade_error(df: pd.DataFrame, params: dict) -> None:
    """段階的誤差プロット: ステア指示→ステア応答→ay→vy→横位置の連鎖."""
    rad2deg = 180.0 / math.pi
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    def _ma(vals):
        return pd.Series(vals).rolling(window, center=True, min_periods=1).mean().values

    rows = [
        # (real_vals, sim_vals, err_vals, ylabel, title, source)
        (
            df["real_steer_kp1"].values * rad2deg,
            df["sim_steer_kp1"].values * rad2deg,
            df["err_steer"].values * rad2deg,
            "ステア角 [deg]",
            "① ステア応答 (cmd→actual): 実機 vs モデル",
            "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias",
        ),
        (
            df["real_ay"].values,
            df["sim_ay"].values,
            df["err_ay"].values,
            "横加速度 ay [m/s²]",
            "② 横加速度 ay: 実機 vs モデル",
            "実機: acceleration/accel.linear.y  モデル: vx·wz",
        ),
        (
            df["real_vy"].values,
            df["sim_vy"].values,
            df["err_vy"].values,
            "横速度 vy [m/s]",
            "③ 横速度 vy: 実機 vs モデル",
            "実機: kinematic_state/twist.linear.y  モデル: 0.0",
        ),
        (
            df["real_wz"].values,
            df["sim_wz"].values,
            df["err_wz"].values,
            "角速度 wz [rad/s]",
            "④ 角速度 wz: 実機 vs モデル",
            "実機: kinematic_state/twist.angular.z  モデル: vx·tan(δ)/wb",
        ),
        (
            df["real_ds_lat"].values * 100,
            df["sim_ds_lat"].values * 100,
            df["err_ds_lat"].values * 100,
            "横方向 Δpos [cm]",
            "⑤ 横方向 1ステップ変位: 実機 vs モデル",
            "実機: kinematic_state/pose.position  モデル: state_[0,1]",
        ),
    ]

    fig, axes = plt.subplots(5, 2, figsize=(16, 20), gridspec_kw={"width_ratios": [3, 1]})

    for row_idx, (real_v, sim_v, err_v, ylabel, title, source) in enumerate(rows):
        ax_ts = axes[row_idx, 0]
        ax_err = axes[row_idx, 1]

        # 時系列パネル
        ax_ts.plot(tr, real_v, color="#aaaaaa", lw=0.4, alpha=0.4)
        ax_ts.plot(tr, _ma(real_v), color="black", lw=1.5, label="実機 MA")
        ax_ts.plot(tr, sim_v, color="#ffaaaa", lw=0.4, alpha=0.4)
        ax_ts.plot(tr, _ma(sim_v), color="red", lw=1.5, ls="--", label="モデル MA")
        ax_ts.axhline(0, color="black", lw=0.5, ls=":")
        ax_ts.axvline(0, color="green", lw=0.8, ls=":", alpha=0.8)
        ax_ts.set_ylabel(ylabel)
        _set_title(ax_ts, title, source)
        ax_ts.legend(fontsize=8)
        ax_ts.grid(True, alpha=0.3)

        # 誤差時系列パネル（右）
        rmse = float(np.sqrt(np.nanmean(err_v**2)))
        ax_err.plot(tr, err_v, color="#aaaaaa", lw=0.4, alpha=0.4)
        ax_err.plot(tr, _ma(err_v), color="purple", lw=1.5, label=f"誤差 MA\nRMSE={rmse:.4g}")
        ax_err.axhline(0, color="black", lw=0.8)
        ax_err.axvline(0, color="green", lw=0.8, ls=":", alpha=0.8)
        ax_err.set_ylabel(f"誤差 [{ylabel.split('[')[1]}")
        _set_title(ax_err, "誤差 (実機 − モデル)", source)
        ax_err.legend(fontsize=8)
        ax_err.grid(True, alpha=0.3)

    for col in range(2):
        axes[-1, col].set_xlabel("発進からの時刻 [s]")

    fig.suptitle(
        "カーブ② per-step delta: 段階的誤差プロット\nステア指示 → ステア応答 → ay → vy → 横位置",
        fontsize=12,
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "cascade_error")


def save_summary(df: pd.DataFrame) -> None:
    rad2deg = 180.0 / math.pi
    tr = df["tr"].values

    def rmse_cm(col, mask=None):
        v = df[col].values if mask is None else df[col].values[mask]
        return float(np.sqrt(np.mean(v**2))) * 100

    def rmse_deg(col, mask=None):
        v = df[col].values if mask is None else df[col].values[mask]
        return float(np.sqrt(np.mean(v**2))) * rad2deg

    def mean_deg(col, mask=None):
        v = df[col].values if mask is None else df[col].values[mask]
        return float(np.mean(v)) * rad2deg

    lines = [
        "=== カーブ② per-step delta 分析 サマリ ===",
        f"有効ステップ数: {len(df)}",
        f"解析窓: tr={tr[0]:.1f}〜{tr[-1]:.1f}s",
        "",
        "--- 全区間: 位置 ---",
        f"縦方向 RMSE: {rmse_cm('err_ds_long'):.3f} cm",
        f"横方向 RMSE: {rmse_cm('err_ds_lat'):.3f} cm",
        "",
        "--- 全区間: ステア予測 (actual[k+1] − pred[k+1]) ---",
        f"ステア予測 RMSE: {rmse_deg('err_steer'):.4f} deg",
        f"ステア予測 平均誤差: {mean_deg('err_steer'):.4f} deg  (正=実機が指令より大/負=小)",
        "",
        "--- 発進後時間帯別 (縦方向 RMSE) ---",
    ]
    for lbl, lo, hi in [
        ("t=0〜5s  (発進直後)", 0.0, 5.0),
        ("t=5〜15s (カーブ進入)", 5.0, 15.0),
        ("t=15〜35s (カーブ奥)", 15.0, 35.0),
    ]:
        mask = (tr >= lo) & (tr < hi)
        if mask.sum() > 0:
            lines.append(f"  {lbl}: {rmse_cm('err_ds_long', mask):.3f} cm  (n={mask.sum()})")

    lines += ["", "--- 発進後時間帯別 (ステア予測 RMSE) ---"]
    for lbl, lo, hi in [
        ("t=0〜5s  (発進直後)", 0.0, 5.0),
        ("t=5〜15s (カーブ進入)", 5.0, 15.0),
        ("t=15〜35s (カーブ奥)", 15.0, 35.0),
    ]:
        mask = (tr >= lo) & (tr < hi)
        if mask.sum() > 0:
            lines.append(f"  {lbl}: {rmse_deg('err_steer', mask):.4f} deg  (n={mask.sum()})")

    lines += ["", "--- 速度域別 ---"]
    for lo, hi in [(0, 2), (2, 5), (5, 8), (8, 100)]:
        lbl = f"v={lo}〜{hi} m/s" if hi < 100 else f"v≥{lo} m/s"
        mask = (df["real_vx"].values >= lo) & (df["real_vx"].values < hi)
        if mask.sum() > 0:
            lines.append(
                f"  {lbl:22s}: 縦={rmse_cm('err_ds_long', mask):.3f} cm, "
                f"横={rmse_cm('err_ds_lat', mask):.3f} cm, "
                f"ステア={rmse_deg('err_steer', mask):.4f} deg  (n={mask.sum()})"
            )

    text = "\n".join(lines)
    print(text)
    (OUT_DIR / "summary.txt").write_text(text + "\n", encoding="utf-8")
    print(f"  Saved: {OUT_DIR / 'summary.txt'}")


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------


def _apply_runtime_config(cfg: RuntimeConfig) -> dict:
    """RuntimeConfig をモジュールレベル変数に反映し、PARAMS を返す。"""
    global BASE, LITE_DIR, OUT_DIR, REAL_BAG_DIR  # noqa: PLW0603
    global _CURVE2_WINDOW, _CURVE2_CX, _CURVE2_CY, PARAMS, SUB_DT  # noqa: PLW0603

    BASE = cfg.base_dir
    LITE_DIR = cfg.lite_dir
    OUT_DIR = cfg.out_dir / "curve2_per_step"
    # 入力 bag のデフォルトパス (実体は _resolve_real_mcap が file/dir 両対応)
    REAL_BAG_DIR = LITE_DIR / "real.lite.mcap"

    _CURVE2_WINDOW = cfg.curve2_window
    c2 = cfg.curve2
    if c2 is not None:
        _CURVE2_CX = int(c2["cx"])
        _CURVE2_CY = int(c2["cy"])

    PARAMS = _build_params(cfg)
    SUB_DT = PARAMS["sub_dt"]
    return PARAMS


def main() -> None:
    parser = argparse.ArgumentParser(description="カーブ② per-step delta 分析")
    add_common_cli_arguments(parser)
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    params = _apply_runtime_config(cfg)

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # 入力 bag は単一 `.mcap` ファイル or `.lite` ディレクトリの両方を試す
    try:
        real_mcap = _resolve_real_mcap(REAL_BAG_DIR)
    except FileNotFoundError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Loading: {real_mcap}")
    data = load_real_bag(real_mcap)

    t0_ns = find_autonomous_start(data)
    print(f"AUTONOMOUS 開始: t0_ns={t0_ns}")

    df_vel = data["vel"].copy()
    df_vel["t"] = (df_vel["t_ns"] - t0_ns) / 1e9
    df_vel = df_vel[df_vel["t"] >= 0].sort_values("t").reset_index(drop=True)

    t_launch = find_curve2_launch(df_vel)
    if t_launch is None:
        print("ERROR: カーブ② 発進時刻を検出できませんでした", file=sys.stderr)
        sys.exit(1)
    print(f"カーブ②発進: t={t_launch:.1f}s")

    print("\n=== per-step delta 分析開始 ===")
    df = run_per_step(data, t0_ns, t_launch, params)
    print(f"有効ステップ: {len(df)}")

    if df.empty:
        print("ERROR: 有効なステップが 0 件でした", file=sys.stderr)
        sys.exit(1)

    print("\n=== 出力生成 ===")
    df.to_csv(OUT_DIR / "per_step_delta.csv", index=False)
    print(f"  Saved: {OUT_DIR / 'per_step_delta.csv'}")

    save_summary(df)
    plot_overview(df, params)
    plot_error_timeseries(df, params)
    plot_error_vs_speed(df, params)
    plot_steering_analysis(df, params)
    plot_map_distribution(df, params)
    plot_lateral_dynamics_timeseries(df, params)
    plot_steer_vs_lateral_scatter(df, params)
    plot_cascade_error(df, params)

    print(f"\n完了。出力先: {OUT_DIR}")


if __name__ == "__main__":
    main()
