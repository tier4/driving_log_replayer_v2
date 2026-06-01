#!/usr/bin/env python3
"""全走行区間の per-step delta 分析（C++ 車両モデル使用）.

手法:
  各制御コマンド区間 (t_k, t_{k+1}) について:
    1. C++ 車両モデルを t_k の実機状態にリセット（累積誤差を排除）
    2. 実機制御コマンドを ZOH で適用
    3. 区間終端の予測状態を実機状態と比較（車両ローカル座標系）

解析窓: AUTONOMOUS 開始から制御コマンド系列末尾まで。
時系列軸 tr は AUTONOMOUS 開始時刻からの経過時間 [s]。

使用モデル: DELAY_STEER_ACC_GEARED_WO_FALL_GUARD (C++ ctypes 経由)
  ライブラリ: libvehicle_model_wrapper.so (simple_sensor_simulator パッケージが提供)
  パラメータ: j6_gen2_description パッケージの config/simulator_model.param.yaml

出力:
  comparison/per_step/
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

from .lib._events import find_autonomous_start as _find_autonomous_start
from .lib._io import (
    align_time,
    iter_bag_messages,
    load_operation_mode,
    load_steering,
    load_velocity,
    resolve_topic,
)
from .lib._map import load_map_ways as _load_map_ways_impl
from .lib._map import resolve_map_osm
from .lib._params_utils import add_params_annotation, load_sim_params, setup_jp_font
from .lib._runtime_config import RuntimeConfig, add_common_cli_arguments, build_runtime_config

setup_jp_font()

# モジュールレベル設定 (main() で RuntimeConfig 経由で上書きされる)
BASE = Path(os.environ.get("BEST_MODEL_BASE_DIR") or Path(__file__).parent)
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison" / "per_step"

# 実機 lite bag は単一ファイルでも directory bag でも受け付ける。
REAL_BAG_DIR = LITE_DIR / "real.lite.mcap"

# 地図プロットの bbox margin [m] (走行軌跡の min/max からの余白)
MAP_BBOX_MARGIN = 10.0

# per_step 専用の上書き値 (load_sim_params() のシム既定値に上塗りする)。
# sub_dt は per_step の積分刻み (30Hz) で per_step 固有の設定。
#
# 旧コードは steer_bias=0.01 をここで上塗りしていたが、これはシム仕様値
# (simulator_model.param.yaml の steer_bias ≈ 0.0005 rad) と乖離した非物理的な
# phantom bias で、ideal_steer 以外の全ケース (baseline/kus0020/shorter_wb) の
# err_steer を一律に汚染し、図の注釈 (load_sim_params 由来 0.0005 を表示) とも
# 自己矛盾していた。Stage3 closed-loop sim と整合させるため override から除外し、
# load_sim_params() のシム仕様 steer_bias をそのまま使う。ケース固有値は cases.yaml の
# params で明示上書きできる (ideal_steer は C++ 側が bias を持たないため 0.0 を明示)。
_PARAMS_OVERRIDES = {
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


# Placeholder; 実際の値は main() 内 _apply_runtime_config(cfg, case_tag, case_params) で
# 上書きされる。module import 時に load_sim_params() (YAML 読込) を走らせない
# ことで cases ループ起動時のオーバーヘッドを削減する。
PARAMS: dict = {"sub_dt": 1.0 / 30.0}
SUB_DT: float = PARAMS["sub_dt"]


# ---------------------------------------------------------------------------
# DELAY_STEER_ACC_GEARED_WO_FALL_GUARD の C++ ctypes ラッパー
# ---------------------------------------------------------------------------

# libvehicle_model_wrapper.so が vm_get_wz を export しているか (_load_lib で確定)。
_HAS_WZ = False


def _resolve_so_path() -> Path:
    """libvehicle_model_wrapper.so の場所を解決する。

    優先順:
      1. VEHICLE_MODEL_SO_PATH 環境変数
      2. ament_index_python で simple_sensor_simulator パッケージ share を解決
    """
    env = os.environ.get("VEHICLE_MODEL_SO_PATH")
    if env:
        p = Path(env)
        if p.exists():
            return p
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("simple_sensor_simulator"))
    return share / "libvehicle_model_wrapper.so"


def _load_lib() -> ctypes.CDLL:
    so = _resolve_so_path()
    if not so.exists():
        raise FileNotFoundError(
            f"{so} が見つかりません。simple_sensor_simulator を colcon build してください。"
        )
    lib = ctypes.CDLL(str(so))

    c_double = ctypes.c_double
    c_void_p = ctypes.c_void_p

    # model-specific factories (returns VmModel * as c_void_p)
    lib.vm_create_ideal_steer_acc.restype = c_void_p
    lib.vm_create_ideal_steer_acc.argtypes = [c_double, c_double]  # wheelbase, sub_dt

    lib.vm_create_delay_steer_acc_geared_wo_fall_guard.restype = c_void_p
    lib.vm_create_delay_steer_acc_geared_wo_fall_guard.argtypes = [c_double] * 15

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

    lib.vm_step_dt.restype = None
    lib.vm_step_dt.argtypes = [c_void_p, c_double]

    for fn in (
        "vm_get_x",
        "vm_get_y",
        "vm_get_yaw",
        "vm_get_vx",
        "vm_get_steer",
        "vm_get_ax",
    ):
        getattr(lib, fn).restype = c_double
        getattr(lib, fn).argtypes = [c_void_p]

    # wz (yaw rate) は新しめのラッパーのみ export (vehicle_model_c_wrapper.cpp の vm_get_wz)。
    # k_us 依存の calc_yaw_rate を経由するため per-step でも understeer の寄与を観測できる。
    # 古い .so との後方互換のため存在時のみ束縛し、_HAS_WZ フラグで制御する。
    global _HAS_WZ  # noqa: PLW0603
    if hasattr(lib, "vm_get_wz"):
        lib.vm_get_wz.restype = c_double
        lib.vm_get_wz.argtypes = [c_void_p]
        _HAS_WZ = True
    else:
        _HAS_WZ = False
        print(
            "[WARN] libvehicle_model_wrapper.so に vm_get_wz が無いため sim_wz は NaN。"
            "simple_sensor_simulator を再ビルドしてください。",
            file=sys.stderr,
        )

    # vm_step_dt はラッパーに未 export のため使わない。残ステップ (remainder) は無視。
    lib.vm_destroy.restype = None
    lib.vm_destroy.argtypes = [c_void_p]

    return lib


class VehicleModel:
    """SimModelInterface 派生の C ラッパーを model_type で dispatch."""

    _lib: ctypes.CDLL | None = None

    @classmethod
    def _get_lib(cls) -> ctypes.CDLL:
        if cls._lib is None:
            cls._lib = _load_lib()
        return cls._lib

    def __init__(self, params: dict, sub_dt: float, model_type: str):
        """
        model_type:
          - "delay_steer_acc_geared_wo_fall_guard": 旧既定。15 引数 (params の wheelbase,
            steer_bias, time_constant 等 + debug_*_scaling_factor, k_us)
          - "ideal_steer_acc": wheelbase のみ
        params は load_sim_params() の base に cases.yaml の case.params が上書きされた dict。

        注: ideal_steer_acc は d(vx)/dt = ax (加速度指令を直接積分; 停止/ギア拘束なし) のため、
        停止中 (real_vx≈0) にブレーキ指令を与える step で sim_vx がわずかに負になり得る
        (実測: 停止中ブレーキの 672 step で sim_vx 平均 -0.05 m/s)。これは dynamics-free な ideal
        モデルの仕様で、「停止中に後退する」という微小な予測誤差を忠実に表すもの (クランプすると
        その誤差を隠す)。1 step (~0.03s) の位置寄与は ~mm で err_ds への影響は無視可能
        (負 step の err_ds_long RMSE ≈ 0.09cm vs 全体 1.38cm)。delay_steer_acc_geared はギア拘束で発生しない。
        """
        p = params
        lib = self._get_lib()
        self._lib = lib
        self._model_type = model_type
        if model_type == "ideal_steer_acc":
            self._ptr = lib.vm_create_ideal_steer_acc(p["wheelbase"], sub_dt)
            self._steer_bias = 0.0
        elif model_type == "delay_steer_acc_geared_wo_fall_guard":
            self._ptr = lib.vm_create_delay_steer_acc_geared_wo_fall_guard(
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
                p.get("debug_acc_scaling_factor", 1.0),
                p.get("debug_steer_scaling_factor", 1.0),
                p.get("k_us", 0.0),
            )
            self._steer_bias = p["steer_bias"]
        else:
            raise ValueError(
                f"未対応の model_type: {model_type!r}. "
                "対応: 'ideal_steer_acc', 'delay_steer_acc_geared_wo_fall_guard'"
            )
        self._sub_dt = sub_dt

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
        """Euler 1 ステップ積分（任意 dt 秒）。
        端数補正 (interval < SUB_DT) 用。dt は SUB_DT より十分小さい範囲で呼ぶこと。"""
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
        """yaw rate [rad/s]。ラッパーが vm_get_wz を export していなければ NaN。"""
        if not _HAS_WZ:
            return float("nan")
        return self._lib.vm_get_wz(self._ptr)

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


# ---------------------------------------------------------------------------
# per-step delta 分析
# ---------------------------------------------------------------------------


def _prepare_gt(data: dict, t0_ns: int, params: dict) -> dict:
    """run_per_step / run_free_rollout 共通の GT 準備。

    AUTONOMOUS 開始 (t0_ns) から制御コマンド系列末尾までを cmd タイムスタンプ上に
    補間した GT 系列・過去コマンド系列を dict で返す。
    """

    # -- タイムスタンプを秒に変換 --
    def to_sec(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()
        df["t"] = (df["t_ns"] - t0_ns) / 1e9
        return df.drop(columns=["t_ns"])

    df_kin = to_sec(data["kin"]).sort_values("t").reset_index(drop=True)
    df_acc = to_sec(data["acc"]).sort_values("t").reset_index(drop=True)
    df_steer = to_sec(data["steer"]).sort_values("t").reset_index(drop=True)
    df_cmd = to_sec(data["cmd"]).sort_values("t").reset_index(drop=True)

    if df_cmd.empty:
        raise RuntimeError("制御コマンドが空です")
    if df_kin.empty:
        raise RuntimeError("運動学状態が空です")

    # AUTONOMOUS 開始 (tr=0) から各系列末尾の min まで解析対象とする。
    # cmd だけ末尾まで取ると kin/acc/steer 側で np.interp が右端値に張り付き
    # 末尾区間で誤差が不自然に 0 にクリップされるため、安全側で min を取る。
    t_lo = 0.0
    t_hi_candidates = [df_cmd["t"].max(), df_kin["t"].max()]
    if not df_acc.empty:
        t_hi_candidates.append(df_acc["t"].max())
    if not df_steer.empty:
        t_hi_candidates.append(df_steer["t"].max())
    t_hi = float(min(t_hi_candidates))

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

    return {
        "t_cmd": t_cmd,
        "df_cmd": df_cmd,
        "t_cmd_full": t_cmd_full,
        "accel_des_full": accel_des_full,
        "steer_des_full": steer_des_full,
        "gt_x": gt_x,
        "gt_y": gt_y,
        "gt_yaw": gt_yaw,
        "gt_vx": gt_vx,
        "gt_wz": gt_wz,
        "gt_vy": gt_vy,
        "gt_ax": gt_ax,
        "gt_ay": gt_ay,
        "gt_steer": gt_steer,
        "gt_dwz": gt_dwz,
        "gt_steer_kinematic": gt_steer_kinematic,
    }


def _delay_history(
    t_k: float, q_size: int, t_full: np.ndarray, val_full: np.ndarray
) -> list[float]:
    """delay queue 用の過去コマンド履歴 (oldest→newest) を実コマンドから補間生成。"""
    return [
        float(
            np.interp(
                t_k - (q_size - i) * SUB_DT,
                t_full,
                val_full,
                left=val_full[0],
                right=val_full[-1],
            )
        )
        for i in range(q_size)
    ]


def run_per_step(data: dict, t0_ns: int, params: dict, model_type: str) -> pd.DataFrame:
    """per-step delta を実行し結果 DataFrame を返す。

    AUTONOMOUS 開始 (t0_ns) から制御コマンド系列末尾までを解析対象とする。
    時系列軸 tr は AUTONOMOUS 開始からの経過時間 [s]。各ステップで実機状態にリセットする
    1-step 予測 (累積誤差を排除)。多段の累積誤差は run_free_rollout を参照。
    """
    g = _prepare_gt(data, t0_ns, params)
    t_cmd = g["t_cmd"]
    df_cmd = g["df_cmd"]
    t_cmd_full = g["t_cmd_full"]
    accel_des_full = g["accel_des_full"]
    steer_des_full = g["steer_des_full"]
    gt_x = g["gt_x"]
    gt_y = g["gt_y"]
    gt_yaw = g["gt_yaw"]
    gt_vx = g["gt_vx"]
    gt_wz = g["gt_wz"]
    gt_vy = g["gt_vy"]
    gt_ax = g["gt_ax"]
    gt_ay = g["gt_ay"]
    gt_steer = g["gt_steer"]
    gt_dwz = g["gt_dwz"]
    gt_steer_kinematic = g["gt_steer_kinematic"]

    model = VehicleModel(params, SUB_DT, model_type)
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
        # interval が SUB_DT より小さい場合 (cmd 30Hz ≒ SUB_DT) は n_full=0 で
        # step_dt のみが効くため、端数ステップを必ず実行する。
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
        # sim_wz: モデルが予測した t_{k+1} の yaw rate (vm_get_wz, k_us 依存)。
        # ラッパーが未 export なら NaN (sim_vy/ay は getVy()=0 / 未実装のため引き続き省略)。
        #
        # 【重要・k_us が異なるケース間で err_wz を比較してはいけない】
        # リセット時の steer は gt_steer_kinematic = atan(wz*wb/vx) (k_us=0 の bicycle 逆算) を
        # 使う。よって baseline(k_us=0) は構造上 sim_wz≈wz_real となり err_wz が小さく、k_us>0 は
        # understeer オフセットを負って err_wz が大きく出る (実車の understeer 有無とは無関係な
        # seeding バイアス)。k_us/understeer の同定には err_wz ではなく run_free_rollout
        # (seed は step0 のみで N ステップ後は真の dynamics が支配) を用いること。
        sim_wz = model.wz
        records.append(
            {
                "timestamp": t_cmd[k],
                "tr": t_cmd[k],  # AUTONOMOUS 開始からの経過時間 [s]
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
                "sim_wz": sim_wz,  # モデル予測 yaw rate (k_us 依存; per-step でも k_us 感度あり)
                "err_wz": gt_wz[k + 1] - sim_wz,  # yaw rate 予測誤差 [rad/s]
                "real_dwz": gt_dwz[k + 1],
                "real_wz_k": gt_wz[k],  # t_k — 散布図の Y 軸（ステア角と同時刻）
                "real_ay_k": gt_ay[k],
                "real_vy_k": gt_vy[k],
                "sim_vx": model.vx,
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


def run_free_rollout(
    data: dict,
    t0_ns: int,
    params: dict,
    model_type: str,
    horizons: tuple[int, ...] = (1, 2, 5, 10, 20),
    stride: int = 5,
) -> pd.DataFrame:
    """多段 (free-running) ロールアウト誤差を計算する。

    per-step は各ステップで実機状態にリセットするため、yaw 積分に効く k_us / wheelbase の
    累積差を検出できない (全 dynamics ケースで位置 RMSE がほぼ同一になる既知の限界)。
    本関数は各開始点 k0 で実機状態にリセットした後、実コマンド系列を N ステップ連続適用
    (途中リセット無し) し、N ステップ後の終端位置・yaw の対実機誤差を測る。horizon N を
    増やすほど dynamics 差が累積し顕在化するため、k_us/wheelbase/時定数の同定に使える。
    """
    g = _prepare_gt(data, t0_ns, params)
    t_cmd = g["t_cmd"]
    t_cmd_full = g["t_cmd_full"]
    accel_des_full = g["accel_des_full"]
    steer_des_full = g["steer_des_full"]
    gt_x = g["gt_x"]
    gt_y = g["gt_y"]
    gt_yaw = g["gt_yaw"]
    gt_vx = g["gt_vx"]
    gt_ax = g["gt_ax"]
    gt_steer_kinematic = g["gt_steer_kinematic"]
    accel_des = g["df_cmd"]["accel_des"].values
    steer_des = g["df_cmd"]["steer_des"].values

    model = VehicleModel(params, SUB_DT, model_type)
    acc_q_size = model.acc_q_size
    steer_q_size = model.steer_q_size
    steer_bias = params["steer_bias"]

    n = len(t_cmd)
    recs: list[dict] = []
    for horizon in horizons:
        for k0 in range(0, n - 1 - horizon, stride):
            ivs = [t_cmd[k0 + j + 1] - t_cmd[k0 + j] for j in range(horizon)]
            if any(iv <= 0.001 or iv > 1.0 for iv in ivs):
                continue
            model.reset_with_history(
                x=gt_x[k0],
                y=gt_y[k0],
                yaw=gt_yaw[k0],
                vx=gt_vx[k0],
                steer_actual=float(gt_steer_kinematic[k0]) + steer_bias,
                ax=gt_ax[k0],
                acc_history=_delay_history(t_cmd[k0], acc_q_size, t_cmd_full, accel_des_full),
                steer_history=_delay_history(t_cmd[k0], steer_q_size, t_cmd_full, steer_des_full),
            )
            # 実コマンド系列を N 区間連続適用 (途中リセット無し)
            for j in range(horizon):
                k = k0 + j
                iv = ivs[j]
                ad = float(accel_des[k])
                sd = float(steer_des[k])
                n_full = int(iv / SUB_DT)
                for _ in range(n_full):
                    model.step(ad, sd)
                rem = iv - n_full * SUB_DT
                if rem > 1e-6:
                    model.step_dt(ad, sd, rem)
            k_end = k0 + horizon
            dx = model.x - gt_x[k_end]
            dy = model.y - gt_y[k_end]
            yaw_err = (model.yaw - gt_yaw[k_end] + math.pi) % (2 * math.pi) - math.pi
            cos_e = math.cos(gt_yaw[k_end])
            sin_e = math.sin(gt_yaw[k_end])
            recs.append({
                "horizon": horizon,
                "k0": k0,
                "tr": float(t_cmd[k0]),
                "elapsed": float(t_cmd[k_end] - t_cmd[k0]),
                "pos_err": math.hypot(dx, dy),
                "err_long": dx * cos_e + dy * sin_e,
                "err_lat": -dx * sin_e + dy * cos_e,
                "yaw_err_deg": math.degrees(yaw_err),
            })
    return pd.DataFrame(recs)


def save_rollout_summary(df_roll: pd.DataFrame) -> None:
    """multi-step rollout の horizon 別 RMSE を summary.txt に追記し rollout.csv を保存。"""
    if df_roll.empty:
        print("  (rollout: 有効サンプル無し、スキップ)")
        return
    lines = ["", "--- multi-step rollout (free-running, リセット無し N ステップ後の対実機誤差) ---"]
    for horizon in sorted(df_roll["horizon"].unique()):
        sub = df_roll[df_roll["horizon"] == horizon]
        pos = float(np.sqrt(np.mean(sub["pos_err"].values ** 2))) * 100.0
        lon = float(np.sqrt(np.mean(sub["err_long"].values ** 2))) * 100.0
        lat = float(np.sqrt(np.mean(sub["err_lat"].values ** 2))) * 100.0
        yaw = float(np.sqrt(np.mean(sub["yaw_err_deg"].values ** 2)))
        el = float(sub["elapsed"].mean())
        lines.append(
            f"  N={int(horizon):2d} (≈{el:.2f}s): 位置 RMSE={pos:.2f} cm "
            f"(縦={lon:.2f}, 横={lat:.2f} cm)  yaw RMSE={yaw:.3f} deg  (n={len(sub)})"
        )
    text = "\n".join(lines)
    print(text)
    with (OUT_DIR / "summary.txt").open("a", encoding="utf-8") as f:
        f.write(text + "\n")
    df_roll.to_csv(OUT_DIR / "rollout.csv", index=False)
    print(f"  Saved: {OUT_DIR / 'rollout.csv'}")


def plot_rollout_growth(df_roll: pd.DataFrame, params: dict) -> None:
    """rollout 長 N に対する位置・yaw 誤差 RMSE の成長を描く (dynamics 差の顕在化)。"""
    if df_roll.empty:
        return
    horizons = sorted(df_roll["horizon"].unique())
    pos = [float(np.sqrt(np.mean(df_roll[df_roll["horizon"] == h]["pos_err"].values ** 2))) * 100.0
           for h in horizons]
    yaw = [float(np.sqrt(np.mean(df_roll[df_roll["horizon"] == h]["yaw_err_deg"].values ** 2)))
           for h in horizons]
    fig, ax1 = plt.subplots(figsize=(9, 5))
    fig.suptitle("多段ロールアウト誤差成長 (free-running)", fontsize=11)
    ax1.plot(horizons, pos, "o-", color="#1f77b4", label="位置 RMSE [cm]")
    ax1.set_xlabel("rollout 長 N [step]  (N × SUB_DT 秒相当)")
    ax1.set_ylabel("位置 RMSE [cm]", color="#1f77b4")
    ax1.tick_params(axis="y", labelcolor="#1f77b4")
    ax1.grid(True, lw=0.5, alpha=0.6)
    ax2 = ax1.twinx()
    ax2.plot(horizons, yaw, "s--", color="#d62728", label="yaw RMSE [deg]")
    ax2.set_ylabel("yaw RMSE [deg]", color="#d62728")
    ax2.tick_params(axis="y", labelcolor="#d62728")
    add_params_annotation(fig, params)
    fig.tight_layout()
    _save(fig, "rollout_error_growth")


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
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="AUTONOMOUS 開始")
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
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
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
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
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
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
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
    ax.set_ylabel("横方向誤差 [cm]")
    _set_title(
        ax, "1ステップ横方向誤差", "実機: kinematic_state/pose.position  モデル: state_[0,1]"
    )
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(
        "全走行 per-step delta 分析\n(各ステップで実機状態にリセット — 計画挙動の差を除外)",
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
        ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="AUTONOMOUS 開始")
        ax.set_ylabel(ylabel)
        _set_title(ax, title, source)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[2].set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
    fig.suptitle("全走行 per-step delta 誤差時系列", fontsize=11)
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

    fig.suptitle("全走行 per-step delta: 速度依存性", fontsize=11)
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
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="AUTONOMOUS 開始")
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
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
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
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
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="AUTONOMOUS 開始")
    rmse_deg = float(np.sqrt(np.mean(err_deg**2)))
    ax.set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
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
        "全走行 per-step ステアリング分析\n(1ステップ予測誤差: actual[k+1] − model_pred[k+1])",
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
        # err_ay / err_vy / err_wz は sim_* がラッパー未 export のため一旦スキップ
    ]

    # columns は最低 3 要素 (err_ds_long / err_ds_lat / err_steer) ある前提
    fig, axes = plt.subplots(1, len(columns), figsize=(6 * len(columns), 6))

    # 走行軌跡の bbox + margin で地図プロット範囲を自動算出
    x_min = float(df["pos_x"].min()) - MAP_BBOX_MARGIN
    x_max = float(df["pos_x"].max()) + MAP_BBOX_MARGIN
    y_min = float(df["pos_y"].min()) - MAP_BBOX_MARGIN
    y_max = float(df["pos_y"].max()) + MAP_BBOX_MARGIN

    for ax, (vals, label, unit, source) in zip(axes, columns):
        if map_ways:
            for pts in map_ways:
                wx, wy = pts[:, 0], pts[:, 1]
                if wx.max() < x_min or wx.min() > x_max:
                    continue
                if wy.max() < y_min or wy.min() > y_max:
                    continue
                ax.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

        vmax = max(abs(vals).max(), 1e-6)
        sc = ax.scatter(
            df["pos_x"], df["pos_y"], c=vals, cmap="RdBu_r", vmin=-vmax, vmax=vmax, s=8, zorder=3
        )
        plt.colorbar(sc, ax=ax, label=unit)
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        _set_title(ax, f"{label} [{unit}]", source)
        ax.grid(True, lw=0.5, alpha=0.5)

    fig.suptitle("全走行 per-step delta: 地図上の誤差分布 (実機 − モデル)", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "map_distribution")


def plot_lateral_dynamics_timeseries(df: pd.DataFrame, params: dict) -> None:
    """横方向諸量の時系列: ay / vy / wz / dwz の実機 vs モデル."""
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    # sim_ay/vy/wz はラッパー未 export のため実機のみ表示
    rows = [
        (
            "real_ay",
            None,
            "横加速度 ay [m/s²]",
            "横加速度 ay: 実機",
            "実機: 位置微分(2階)→移動平均スムージング",
        ),
        (
            "real_vy",
            None,
            "横速度 vy [m/s]",
            "横速度 vy: 実機",
            "実機: 位置微分→body frame変換→移動平均スムージング",
        ),
        (
            "real_wz",
            None,
            "角速度 wz [rad/s]",
            "角速度 wz: 実機",
            "実機: kinematic_state/twist.angular.z",
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
        ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="AUTONOMOUS 開始")
        ax.set_ylabel(ylabel)
        _set_title(ax, title, source)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("AUTONOMOUS 開始からの時刻 [s]")
    fig.suptitle(
        "全走行 per-step delta: 横方向諸量 時系列\n(各ステップで実機状態にリセット)", fontsize=11
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

    # sim_ay/vy/wz はラッパー未 export のため実機のみ散布
    cols = [
        (
            "real_ay",
            None,
            "横加速度 ay [m/s²]",
            "横軸: steering_status/tire_angle(t_k)  縦軸: 位置微分(2階)",
        ),
        (
            "real_vy",
            None,
            "横速度 vy [m/s]",
            "横軸: steering_status/tire_angle(t_k)  縦軸: 位置微分body frame",
        ),
        (
            "real_wz",
            None,
            "角速度 wz [rad/s]",
            "横軸: steering_status/tire_angle(t_k)  縦軸: kinematic_state/twist.angular.z",
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
        "全走行 per-step delta: ステア角 vs 横方向諸量 散布図\n(スケール誤差・バイアス確認)",
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
        # ② ay / ③ vy / ④ wz は sim_* がラッパー未 export のため一旦スキップ
        (
            df["real_ds_lat"].values * 100,
            df["sim_ds_lat"].values * 100,
            df["err_ds_lat"].values * 100,
            "横方向 Δpos [cm]",
            "② 横方向 1ステップ変位: 実機 vs モデル",
            "実機: kinematic_state/pose.position  モデル: state_[0,1]",
        ),
    ]

    fig, axes = plt.subplots(len(rows), 2, figsize=(16, 4 * len(rows)), gridspec_kw={"width_ratios": [3, 1]})

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
        axes[-1, col].set_xlabel("AUTONOMOUS 開始からの時刻 [s]")

    fig.suptitle(
        "全走行 per-step delta: 段階的誤差プロット\nステア指示 → ステア応答 → 横位置",
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
        "=== 全走行 per-step delta 分析 サマリ ===",
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
    ]
    # yaw rate 予測 RMSE (vm_get_wz export 時のみ)。steer/位置と異なり k_us 感度を持つ。
    if "err_wz" in df.columns and np.isfinite(df["err_wz"].values).any():
        wz_rmse = float(np.sqrt(np.nanmean(df["err_wz"].values ** 2))) * rad2deg
        lines += [
            "--- 全区間: yaw rate 予測 (vm_get_wz, k_us 感度あり) ---",
            f"yaw rate 予測 RMSE: {wz_rmse:.4f} deg/s",
            "  (注: steer を k_us=0 の運動学逆算で seed するため、k_us が異なるケース間の "
            "err_wz 比較は seeding バイアスを含む。k_us 同定は rollout を参照)",
            "",
        ]
    lines.append("--- 時間帯別 (4 等分 equal-time bins / 縦・横・ステア RMSE) ---")
    # 時間軸を 4 等分した equal-time bins で RMSE を集計する
    t_min, t_max = float(tr[0]), float(tr[-1])
    T_span = t_max - t_min
    bands: list[tuple[str, float, float]] = []
    if T_span > 0:
        for i in range(4):
            lo = t_min + T_span * i / 4
            hi = t_min + T_span * (i + 1) / 4
            lbl = f"区間{i + 1} [{lo:.1f}〜{hi:.1f}s]"
            bands.append((lbl, lo, hi))

    for lbl, lo, hi in bands:
        # 最終バンドは右端を含める (rightmost open interval を回避)
        mask = (tr >= lo) & ((tr <= hi) if hi == t_max else (tr < hi))
        if mask.sum() == 0:
            continue
        lines.append(
            f"  {lbl}: 縦={rmse_cm('err_ds_long', mask):.3f} cm  "
            f"横={rmse_cm('err_ds_lat', mask):.3f} cm  "
            f"ステア={rmse_deg('err_steer', mask):.4f} deg  (n={mask.sum()})"
        )

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

    # 透明化: sim_vx<0 (ideal_steer_acc が停止中ブレーキで微小後退する仕様) の件数を注記。
    # バグではなく dynamics-free モデルの忠実な挙動で、位置誤差寄与は無視可能 (VehicleModel docstring 参照)。
    if "sim_vx" in df.columns:
        n_neg = int((df["sim_vx"].values < 0).sum())
        if n_neg > 0:
            lines += [
                "",
                f"--- 注記: sim_vx<0 が {n_neg}/{len(df)} step ---",
                "  ideal_steer_acc の仕様 (停止中ブレーキで微小後退; ギア/停止拘束なし)。"
                "バグではなく位置誤差寄与は無視可能。",
            ]

    text = "\n".join(lines)
    print(text)
    (OUT_DIR / "summary.txt").write_text(text + "\n", encoding="utf-8")
    print(f"  Saved: {OUT_DIR / 'summary.txt'}")


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------


def _apply_runtime_config(cfg: RuntimeConfig, case_tag: str, case_params: dict) -> dict:
    """RuntimeConfig + cases.yaml の case エントリ をモジュールレベル変数に反映。

    OUT_DIR は per_step/<case_tag>/ に固定。PARAMS は load_sim_params()
    の base に case_params を上書きしたもの。
    """
    global BASE, LITE_DIR, OUT_DIR, REAL_BAG_DIR, PARAMS, SUB_DT  # noqa: PLW0603

    BASE = cfg.base_dir
    LITE_DIR = cfg.lite_dir
    OUT_DIR = cfg.out_dir / "per_step" / case_tag
    # 入力 bag のデフォルトパス (実体は _resolve_real_mcap が file/dir 両対応)
    REAL_BAG_DIR = LITE_DIR / "real.lite.mcap"

    PARAMS = _build_params(cfg)
    PARAMS.update(case_params)
    SUB_DT = PARAMS["sub_dt"]
    return PARAMS


def main() -> None:
    parser = argparse.ArgumentParser(description="全走行 per-step delta 分析 (1 ケース 1 run)")
    add_common_cli_arguments(parser)
    parser.add_argument(
        "--case-tag",
        default=os.environ.get("CASE_TAG", ""),
        help="cases.yaml の対象ケース tag (必須; env: CASE_TAG)",
    )
    parser.add_argument(
        "--cases-config",
        default=os.environ.get("CASES_CONFIG_YAML", ""),
        help="cases.yaml のパス (必須; env: CASES_CONFIG_YAML)",
    )
    args = parser.parse_args()

    if not args.case_tag:
        print("ERROR: --case-tag (or CASE_TAG env) が未指定です", file=sys.stderr)
        sys.exit(2)
    if not args.cases_config:
        print(
            "ERROR: --cases-config (or CASES_CONFIG_YAML env) が未指定です。"
            "cases.yaml を scenario.yaml の Conditions.cases_config で指定してください",
            file=sys.stderr,
        )
        sys.exit(2)

    from driving_log_replayer_v2.real_log_sim_comparison.lib._cases_config import (  # noqa: PLC0415
        load_cases_config,
    )

    try:
        cases_cfg = load_cases_config(args.cases_config)
        case = cases_cfg.find_case(args.case_tag)
    except (FileNotFoundError, ValueError, KeyError) as e:
        print(f"ERROR: cases.yaml: {e}", file=sys.stderr)
        sys.exit(2)
    print(f"[case] tag={case.tag}, vehicle_model={case.vehicle_model}, params={case.params}")

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    params = _apply_runtime_config(cfg, case.tag, case.params)

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

    print(f"\n=== per-step delta 分析開始 (case={case.tag}) ===")
    df = run_per_step(data, t0_ns, params, case.vehicle_model)
    print(f"有効ステップ: {len(df)}")

    if df.empty:
        print("ERROR: 有効なステップが 0 件でした", file=sys.stderr)
        sys.exit(1)

    print("\n=== 出力生成 ===")
    df.to_csv(OUT_DIR / "per_step_delta.csv", index=False)
    print(f"  Saved: {OUT_DIR / 'per_step_delta.csv'}")

    save_summary(df)

    # 多段 rollout (D1): per-step では検出できない k_us/wheelbase/時定数の累積差を顕在化。
    print(f"\n=== multi-step rollout 分析 (case={case.tag}) ===")
    df_roll = run_free_rollout(data, t0_ns, params, case.vehicle_model)
    save_rollout_summary(df_roll)
    plot_rollout_growth(df_roll, params)
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
