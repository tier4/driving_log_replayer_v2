#!/usr/bin/env python3
"""全走行区間の N-step オープンループ解析（C++ 車両モデル使用）.

手法 (run_rollout で N=1 と N>1 を統一):
  各開始点 k0 について:
    1. C++ 車両モデルを t_{k0} の実機状態にリセット（過去コマンド履歴を delay queue にセット）
    2. 実機制御コマンド系列を N 区間連続で ZOH 適用（途中リセット無し = free-running）
    3. 終端 t_{k0+N} の予測状態を実機状態と比較（k0 ヨー基準の車両ローカル座標系・実機 − モデル）

  N=1 (stride=1, 全ステップ) が従来の per-step delta（毎ステップリセット、累積誤差を排除した
  1 ステップ予測精度）。N>1 (stride=5) は dynamics 差 (k_us/wheelbase/時定数) の累積を顕在化する。

解析窓: AUTONOMOUS 開始から制御コマンド系列末尾まで。
時系列軸 tr は AUTONOMOUS 開始時刻からの経過時間 [s]。

使用モデル: DELAY_STEER_ACC_GEARED_WO_FALL_GUARD (C++ ctypes 経由)
  ライブラリ: libvehicle_model_wrapper.so (simple_sensor_simulator パッケージが提供)
  パラメータ: j6_gen2_description パッケージの config/simulator_model.param.yaml

出力:
  comparison/nstep/<tag>/
    nstep_delta.csv (全 horizon 統一スキーマ), summary.txt,
    overview.fig.json, map_distribution.fig.json
  （ケース横断の比較図は step6 の cases/overlay/ が担う）
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import plotly.graph_objects as go

from .lib._events import find_autonomous_start as _find_autonomous_start
from .lib._io import (
    align_time,
    iter_bag_messages,
    load_operation_mode,
    load_steering,
    load_velocity,
    resolve_lite_bag,
    resolve_topic,
)
from .lib._map import load_map_ways as _load_map_ways_impl
from .lib._map import map_ways_in_bbox, resolve_map_osm
from .lib._fig_io import write_fig_json
from .lib._figures import build_fig_overview
from .lib._nstep_common import (
    metrics_description_md,
    n1,
    rmse_by_horizon,
)
from .lib._params_utils import load_sim_params
from .lib._plotly_utils import FIG_HEIGHTS, lanes_to_trace
from .lib._runtime_config import RuntimeConfig, add_common_cli_arguments, build_runtime_config

# モジュールレベル設定 (main() で RuntimeConfig 経由で上書きされる)
BASE = Path(os.environ.get("BEST_MODEL_BASE_DIR") or Path(__file__).parent)
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison" / "nstep"

# 実機 lite bag は単一ファイルでも directory bag でも受け付ける。
REAL_BAG_DIR = LITE_DIR / "real.lite.mcap"

# 地図プロットの bbox margin [m] (走行軌跡の min/max からの余白)
MAP_BBOX_MARGIN = 10.0

# 本ステージ専用の上書き値 (load_sim_params() のシム既定値に上塗りする)。
# sub_dt は N-step 解析の積分刻み (30Hz) で本ステージ固有の設定。
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
    "n_substep": 1,
}


def _build_params(cfg: RuntimeConfig | None = None) -> dict:
    """`vehicle_info.param.yaml` + 本ステージ専用上書きで PARAMS を構築する。

    `_PARAMS_OVERRIDES` は N-step 解析固有の意図的な差分なので維持する。
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
    # 15 base args + 5 verification-viewer-parity longitudinal terms
    # (brake_time_constant, lon_drag_c0, lon_drag_c1, lon_drag_c2, lon_lat_coupling)
    # + 1 int n_substep (Euler sub-steps per outer update() call; 1 = original behaviour)
    lib.vm_create_delay_steer_acc_geared_wo_fall_guard.argtypes = [c_double] * 20 + [
        ctypes.c_int
    ]

    # taiga_dyn: 14 共通引数 (wo_fall_guard の k_us を除く) + 7 物理パラメータ
    # (mass, inertia_z, lf, lr, cornering_stiffness_front, cornering_stiffness_rear, vx_min_dyn)
    lib.vm_create_taiga_dyn.restype = c_void_p
    lib.vm_create_taiga_dyn.argtypes = [c_double] * 21

    # taiga_x (PhysX backend)。physx_vendor 経由で常時ビルドされる。
    # 引数: wheelbase, track_width, mass, inertia_z, cg_offset_x, max_steer,
    #       max_accel, max_brake, wheel_radius, sub_dt, fixed_dt
    lib.vm_create_taiga_x.restype = c_void_p
    lib.vm_create_taiga_x.argtypes = [c_double] * 11

    # reset は末尾に wz (実測 yaw rate) を取り、動的モデルの yaw rate state を seed する。
    lib.vm_reset_full.restype = None
    lib.vm_reset_full.argtypes = [c_void_p] + [c_double] * 7

    # State-only reset (queues untouched) + explicit queue setter
    lib.vm_reset_state.restype = None
    lib.vm_reset_state.argtypes = [c_void_p] + [c_double] * 7

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
        "vm_get_vy",
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

    # vm_integrate_to_horizons: batch-integrate cmd intervals in C for open-loop tuning.
    # Requires colcon rebuild with simple_sensor_simulator C++ addition.
    _c_dbl_p = ctypes.POINTER(c_double)
    _c_int_p = ctypes.POINTER(ctypes.c_int)
    if not hasattr(lib, "vm_integrate_to_horizons"):
        raise RuntimeError(
            "libvehicle_model_wrapper.so に vm_integrate_to_horizons が未 export です。"
            " simple_sensor_simulator を再ビルドしてください。"
        )
    lib.vm_integrate_to_horizons.restype = None
    lib.vm_integrate_to_horizons.argtypes = [
        c_void_p,         # m
        ctypes.c_int,     # n_intervals
        ctypes.c_int,     # k0
        _c_dbl_p,         # accel_des (full array)
        _c_dbl_p,         # steer_des (full array)
        _c_int_p,         # n_full    (full array, int32)
        _c_dbl_p,         # rem       (full array)
        c_double,         # rem_eps
        ctypes.c_int,     # n_horizons
        _c_int_p,         # horizons  (sorted ascending)
        _c_dbl_p,         # x_out
        _c_dbl_p,         # y_out
        _c_dbl_p,         # yaw_out
        _c_dbl_p,         # vx_out
        _c_dbl_p,         # ax_out
        _c_dbl_p,         # steer_out (raw getSteer() = steer_state + steer_bias)
    ]

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
                # verification-viewer-parity longitudinal terms (default neutral)
                p.get("brake_time_constant", 0.0),
                p.get("lon_drag_c0", 0.0),
                p.get("lon_drag_c1", 0.0),
                p.get("lon_drag_c2", 0.0),
                p.get("lon_lat_coupling", 0.0),
                # Euler sub-steps per outer update() call (1 = original single-step behaviour)
                int(p["n_substep"]),
            )
            self._steer_bias = p["steer_bias"]
        elif model_type == "taiga_dyn":
            # 動的自転車モデル。共通の縦・操舵パラメータ + 物理パラメータ (質量・ヨー慣性・
            # 重心位置・前後コーナリング剛性・低速フォールバック閾値)。物理パラメータは妥当な
            # 車両物理値を既定とし cases.yaml で上書き可能。
            # NOTE: 物理パラメータ既定値 (mass/inertia_z/lf/lr/... と下の taiga_x の
            # track_width/cg_offset_x/wheel_radius/max_accel/max_brake/fixed_dt) は
            # scenario_simulator の ego_entity_simulation.cpp makeSimulationModel
            # (TAIGA_DYN / TAIGA_X case) のフォールバック値と一致させること。両者が乖離すると
            # この open-loop ハーネスと closed-loop sim の挙動がずれる。
            wb = p["wheelbase"]
            self._ptr = lib.vm_create_taiga_dyn(
                p["vel_lim"],
                p["steer_lim"],
                p["vel_rate_lim"],
                p["steer_rate_lim"],
                wb,
                sub_dt,
                p["acc_time_delay"],
                p["acc_time_constant"],
                p["steer_time_delay"],
                p["steer_time_constant"],
                p["steer_dead_band"],
                p["steer_bias"],
                p.get("debug_acc_scaling_factor", 1.0),
                p.get("debug_steer_scaling_factor", 1.0),
                p.get("mass", 6560.0),
                p.get("inertia_z", 25868.2318),
                p.get("lf", wb * 0.5 + 0.94323),
                p.get("lr", wb * 0.5 - 0.94323),
                p.get("cornering_stiffness_front", 115830.0),
                p.get("cornering_stiffness_rear", 535860.0),
                p.get("vx_min_dyn", 1.0),
            )
            self._steer_bias = p["steer_bias"]
        elif model_type == "taiga_x":
            # 高忠実 PhysX backend (physx_vendor 経由で常時ビルド)。物理パラメータは
            # 妥当な車両物理値を既定とし cases.yaml で上書き可能。
            wb = p["wheelbase"]
            self._ptr = lib.vm_create_taiga_x(
                wb,
                p.get("track_width", 1.754),
                p.get("mass", 6560.0),
                p.get("inertia_z", 25868.2318),
                p.get("cg_offset_x", -0.94323),
                p["steer_lim"],
                p.get("max_accel", 2.3),
                p.get("max_brake", 5.9),
                p.get("wheel_radius", 0.3725),
                sub_dt,
                p.get("taiga_x_fixed_dt", 1.0 / 1200.0),
            )
            self._steer_bias = 0.0
        else:
            raise ValueError(
                f"未対応の model_type: {model_type!r}. 対応: 'ideal_steer_acc', "
                "'delay_steer_acc_geared_wo_fall_guard', 'taiga_dyn', 'taiga_x'"
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
        wz: float = 0.0,
    ) -> None:
        """
        状態と delay queue を実際の過去コマンド履歴でリセット。

        acc_history   : accel_des [oldest→newest], len == acc_q_size
        steer_history : steer_des [oldest→newest], len == steer_q_size
        wz            : 実測 yaw rate [rad/s]。taiga_dyn など yaw rate を慣性付き state
                        として持つ動的モデルの初期 yaw rate を seed する (毎 reset で 0 に
                        すると小 N の per-step 誤差が立ち上がり過渡に支配されるため)。
                        kinematic モデルでは無視される。
        """
        self._lib.vm_reset_state(self._ptr, x, y, yaw, vx, steer_actual, ax, wz)

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
    def ax(self) -> float:
        """縦方向加速度 state_[5] [m/s²]。vm_get_ax は常時バインド (_load_lib)。

        ideal_steer_acc は加速度状態を持たず指令をそのまま返す (1 次遅れ無し)。
        """
        return self._lib.vm_get_ax(self._ptr)

    @property
    def vy(self) -> float:
        """横速度 [m/s]。kinematic モデルは 0、taiga_dyn は横速度 state を返す。"""
        return self._lib.vm_get_vy(self._ptr)

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
# N-step オープンループ解析
# ---------------------------------------------------------------------------


def _prepare_gt(data: dict, t0_ns: int, params: dict) -> dict:
    """run_rollout の GT 準備。

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

    # ---------------------------------------------------------------------------
    # Hoisted invariant arrays for eval_rollout_rmse
    # ---------------------------------------------------------------------------
    # swept params (k_us, steer_time_constant, debug_steer_scaling_factor,
    # acc_time_constant) は _GT_KEYS に含まれないため、同一 gt-key の eval 呼び出し間で
    # これらの配列は不変。_prepare_gt で一度だけ計算し gt dict に保持することで
    # eval_rollout_rmse の per-eval 再計算コストをゼロにする。
    _sub_dt_h = SUB_DT  # 前計算時点の値を固定 (eval_rollout_rmse と一致)
    _n_h = len(t_cmd)

    # per-interval arrays: iv_arr[k] = t_cmd[k+1] - t_cmd[k]
    _iv_arr = np.diff(t_cmd)                             # shape (_n_h - 1,)
    _nfull_arr = (_iv_arr / _sub_dt_h).astype(np.int32)   # int(iv / sub_dt); int32 for C API
    _rem_arr = _iv_arr - _nfull_arr * _sub_dt_h           # 端数 (iv mod sub_dt)

    # bad-interval 累積和: bad_iv_cumsum[k0+h] - bad_iv_cumsum[k0] > 0 iff
    # 区間 [k0, k0+h-1] に iv<=0.001 or iv>1.0 なエントリが存在する
    _bad_iv = (_iv_arr <= 0.001) | (_iv_arr > 1.0)
    _bad_iv_cumsum = np.cumsum(
        np.concatenate([[0], _bad_iv.view(np.uint8)])
    ).astype(np.intp)  # shape (_n_h,)

    # 全 k0 の delay history を一括ベクトル計算 (q_size は gt-key 不変)
    _acc_q_sz = round(params["acc_time_delay"] / _sub_dt_h)
    _steer_q_sz = round(params["steer_time_delay"] / _sub_dt_h)
    if _acc_q_sz > 0 and _n_h > 0:
        _off_acc = (_acc_q_sz - np.arange(_acc_q_sz)) * _sub_dt_h   # (acc_q_sz,)
        _tq_acc = t_cmd[:, None] - _off_acc[None, :]                 # (_n_h, acc_q_sz)
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

    # k0 ヨー基準ローカル座標分解用 cos/sin
    _cos_y_arr = np.cos(gt_yaw)
    _sin_y_arr = np.sin(gt_yaw)

    # コマンド配列の直接参照 (per-eval df.values アクセスを回避)
    _accel_des_arr = df_cmd["accel_des"].values
    _steer_des_arr = df_cmd["steer_des"].values

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
        # hoisted invariant arrays (eval_rollout_rmse 専用)
        "iv_arr": _iv_arr,
        "nfull_arr": _nfull_arr,
        "rem_arr": _rem_arr,
        "bad_iv_cumsum": _bad_iv_cumsum,
        "acc_hist_all": _acc_hist_all,
        "steer_hist_all": _steer_hist_all,
        "cos_y_arr": _cos_y_arr,
        "sin_y_arr": _sin_y_arr,
        "accel_des_arr": _accel_des_arr,
        "steer_des_arr": _steer_des_arr,
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


def eval_rollout_rmse(
    data: dict,
    t0_ns: int,
    params: dict,
    model_type: str,
    horizons: tuple[int, ...],
    stride: int,
    gt: dict | None = None,
) -> dict[int, dict[str, float]]:
    """run_rollout + rmse_by_horizon の高速等価版 (tuning 専用)。

    gt dict 内のホイスト済み配列を利用し、DataFrame 構築・35キーdict・per-eval の
    _delay_history 再計算を省略する。
    返り値スキーマは rmse_by_horizon と完全一致:
      {h: {"pos" [cm], "long" [cm], "lat" [cm], "yaw" [deg], "steer" [deg],
           "vx" [m/s], "ax" [m/s²]}}

    ビット同一保証:
    - 積分・状態取得は run_rollout と同じ Python/ctypes コールパスを辿る。
    - RMSE の加算順は k0 昇順に固定 (run_rollout の recs.sort 後の行順と一致)。
    - bad-interval 判定は bad_iv_cumsum の O(1) 累積チェックで完全等価。
    - yaw wrapping・steer_bias 加算・各種算術は run_rollout と一致。
    """
    g = gt if gt is not None else _prepare_gt(data, t0_ns, params)

    t_cmd = g["t_cmd"]
    gt_x = g["gt_x"]
    gt_y = g["gt_y"]
    gt_yaw = g["gt_yaw"]
    gt_vx = g["gt_vx"]
    gt_steer = g["gt_steer"]
    gt_steer_kinematic = g["gt_steer_kinematic"]
    gt_wz = g["gt_wz"]
    gt_ax = g["gt_ax"]

    # hoisted arrays (pre-computed in _prepare_gt for this gt-key)
    nfull_arr = g["nfull_arr"]
    rem_arr = g["rem_arr"]
    bc = g["bad_iv_cumsum"]  # bc[k0+h] - bc[k0] > 0 iff bad interval in [k0, k0+h)
    acc_hist_all = g["acc_hist_all"]
    steer_hist_all = g["steer_hist_all"]
    cos_y_arr = g["cos_y_arr"]
    sin_y_arr = g["sin_y_arr"]
    accel_des = g["accel_des_arr"]
    steer_des = g["steer_des_arr"]

    steer_bias = params["steer_bias"]
    model = VehicleModel(params, SUB_DT, model_type)

    n = len(t_cmd)
    sorted_horizons = sorted(horizons)
    min_h = sorted_horizons[0]

    # horizon 別誤差リスト (k0 昇順 append; rmse_by_horizon の加算順と一致)
    errs: dict[int, dict[str, list]] = {
        h: {
            "pos": [],
            "ds_long": [],
            "ds_lat": [],
            "yaw_deg": [],
            "steer": [],
            "vx": [],
            "ax": [],
        }
        for h in sorted_horizons
    }

    # --- C バッチパス (vm_integrate_to_horizons) ---
    # ctypes ポインタをループ外で一度だけ変換しオーバーヘッドを最小化。
    # per-k0 の ctypes.data_as 呼び出しは ~12 回 → 1 回 (k0 と n_valid のみ) に削減。
    _cint_p = ctypes.POINTER(ctypes.c_int)
    _cdbl_p = ctypes.POINTER(ctypes.c_double)
    _lib_fn = model._lib.vm_integrate_to_horizons
    _p_ad  = accel_des.ctypes.data_as(_cdbl_p)
    _p_sd  = steer_des.ctypes.data_as(_cdbl_p)
    _p_nf  = nfull_arr.ctypes.data_as(_cint_p)
    _p_rem = rem_arr.ctypes.data_as(_cdbl_p)
    _n_sh  = len(sorted_horizons)
    _h_arr = np.array(sorted_horizons, dtype=np.int32)
    _p_h   = _h_arr.ctypes.data_as(_cint_p)
    # 出力バッファを一度だけ確保 (eval 間で再利用)
    _x_out     = np.empty(_n_sh, dtype=np.float64)
    _y_out     = np.empty(_n_sh, dtype=np.float64)
    _yaw_out   = np.empty(_n_sh, dtype=np.float64)
    _vx_out    = np.empty(_n_sh, dtype=np.float64)
    _ax_out    = np.empty(_n_sh, dtype=np.float64)
    _steer_buf = np.empty(_n_sh, dtype=np.float64)
    _p_xo     = _x_out.ctypes.data_as(_cdbl_p)
    _p_yo     = _y_out.ctypes.data_as(_cdbl_p)
    _p_yawo   = _yaw_out.ctypes.data_as(_cdbl_p)
    _p_vxo    = _vx_out.ctypes.data_as(_cdbl_p)
    _p_axo    = _ax_out.ctypes.data_as(_cdbl_p)
    _p_steero = _steer_buf.ctypes.data_as(_cdbl_p)

    for k0 in range(0, n - min_h, stride):
        model.reset_with_history(
            x=float(gt_x[k0]),
            y=float(gt_y[k0]),
            yaw=float(gt_yaw[k0]),
            vx=float(gt_vx[k0]),
            steer_actual=float(gt_steer_kinematic[k0]) + steer_bias,
            ax=float(gt_ax[k0]),
            acc_history=acc_hist_all[k0],
            steer_history=steer_hist_all[k0],
            wz=float(gt_wz[k0]),
        )
        cos_y = cos_y_arr[k0]
        sin_y = sin_y_arr[k0]

        # 有効 horizon 数を決定 (昇順のため最大 len(sorted_horizons) 回のループ)
        n_valid = 0
        for h in sorted_horizons:
            if k0 + h >= n:
                break
            if bc[k0 + h] > bc[k0]:
                break
            n_valid += 1
        if n_valid == 0:
            continue

        # C 関数に k0 とフル配列ポインタを渡し、一括積分 + horizon スナップ
        _lib_fn(
            model._ptr,
            ctypes.c_int(sorted_horizons[n_valid - 1]),
            ctypes.c_int(k0),
            _p_ad, _p_sd, _p_nf, _p_rem,
            1e-6,
            ctypes.c_int(n_valid),
            _p_h,
            _p_xo, _p_yo, _p_yawo, _p_vxo, _p_axo, _p_steero,
        )

        # horizon ごとの終端誤差を集計 (run_rollout と同一算術式)
        for i in range(n_valid):
            h = sorted_horizons[i]
            k_end = k0 + h
            mx = _x_out[i]
            my = _y_out[i]
            dx = gt_x[k_end] - mx
            dy = gt_y[k_end] - my
            yaw_err = (gt_yaw[k_end] - _yaw_out[i] + math.pi) % (2 * math.pi) - math.pi
            real_dx = gt_x[k_end] - gt_x[k0]
            real_dy = gt_y[k_end] - gt_y[k0]
            sim_dx = mx - gt_x[k0]
            sim_dy = my - gt_y[k0]
            real_ds_long = real_dx * cos_y + real_dy * sin_y
            real_ds_lat = -real_dx * sin_y + real_dy * cos_y
            sim_ds_long = sim_dx * cos_y + sim_dy * sin_y
            sim_ds_lat = -sim_dx * sin_y + sim_dy * cos_y
            e = errs[h]
            e["pos"].append(math.hypot(dx, dy))
            e["ds_long"].append(real_ds_long - sim_ds_long)
            e["ds_lat"].append(real_ds_lat - sim_ds_lat)
            e["yaw_deg"].append(math.degrees(yaw_err))
            # steer_buf[i] = raw getSteer() = steer_state + steer_bias
            # ∴ gt_steer - steer_buf[i] = gt_steer - (steer_state + steer_bias)
            e["steer"].append(gt_steer[k_end] - float(_steer_buf[i]))
            e["vx"].append(float(gt_vx[k_end]) - _vx_out[i])
            e["ax"].append(float(gt_ax[k_end]) - _ax_out[i])

    def _rms(lst: list) -> float:
        v = np.asarray(lst, dtype=np.float64)
        return float(np.sqrt(np.nanmean(v * v)))

    return {
        h: {
            "pos": _rms(errs[h]["pos"]) * 100.0,
            "long": _rms(errs[h]["ds_long"]) * 100.0,
            "lat": _rms(errs[h]["ds_lat"]) * 100.0,
            "yaw": _rms(errs[h]["yaw_deg"]),
            "steer": _rms(errs[h]["steer"]) * 180.0 / math.pi,
            "vx": _rms(errs[h]["vx"]),
            "ax": _rms(errs[h]["ax"]),
        }
        for h in sorted_horizons
    }


def run_rollout(
    data: dict,
    t0_ns: int,
    params: dict,
    model_type: str,
    horizons: tuple[int, ...],
    stride: int,
    gt: dict | None = None,
) -> pd.DataFrame:
    """N-step オープンループ rollout を実行し統一スキーマの DataFrame を返す。

    gt には _prepare_gt(data, t0_ns, params) の結果を渡せる (省略時は内部で計算)。
    GT は params の delay/wheelbase にのみ依存するため、同一条件で複数回呼ぶ場合は
    呼び出し側で 1 回計算して共有するとよい (main の 2 回呼び・step7 sweep)。

    各開始点 k0 (stride 間隔) で実機状態にリセット（過去コマンド履歴を delay queue にセット）
    した後、実コマンド系列を horizon (=N) 区間連続適用 (途中リセット無し = free-running) し、
    終端 k_end = k0 + N の予測状態を実機と比較する。

    - N=1 が従来の per-step delta（毎ステップリセットの 1 ステップ予測; 累積誤差を排除）。
    - N>1 では yaw 積分に効く k_us / wheelbase / 時定数の累積差が顕在化する
      (N=1 では全 dynamics ケースで位置 RMSE がほぼ同一になる既知の限界)。

    誤差の符号規約は「実機 − モデル」。ds_* の縦横分解は k0 時点の実機ヨー基準の
    車両ローカル座標系 (N=1 で旧 per-step delta と同一)。pos_err / yaw_err_deg は座標系不変。

    【重要・k_us が異なるケース間で小 N の err_wz / yaw_err を比較してはいけない】
    リセット時の steer は gt_steer_kinematic = atan(wz*wb/vx) (k_us=0 の bicycle 逆算) を
    使う。よって k_us=0 のケースは構造上小 N で sim_wz≈wz_real となり誤差が小さく、k_us>0 は
    understeer オフセットを負って誤差が大きく出る (実車の understeer 有無とは無関係な
    seeding バイアス)。k_us/understeer の同定には大 N (seed は k0 のみで N ステップ後は
    真の dynamics が支配) を用いること。
    """
    g = gt if gt is not None else _prepare_gt(data, t0_ns, params)
    t_cmd = g["t_cmd"]
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
    accel_des = g["df_cmd"]["accel_des"].values
    steer_des = g["df_cmd"]["steer_des"].values

    model = VehicleModel(params, SUB_DT, model_type)
    acc_q_size = model.acc_q_size  # = round(acc_time_delay / SUB_DT)
    steer_q_size = model.steer_q_size  # = round(steer_time_delay / SUB_DT)
    steer_bias = params["steer_bias"]

    n = len(t_cmd)
    recs: list[dict] = []

    # horizon 融合: k0 ごとに最大 horizon まで1回だけ積分し、各 horizon 境界で記録する。
    # free-running open-loop では長い horizon の途中状態（step h）は短い horizon の終端と
    # ビット単位で同一のため、結果は元のループ構造（horizon 外側）と完全一致する。
    #
    # 元の horizon-major 出力順を再現するため、入力 horizons の出現インデックスでソートする。
    # これにより (40,20) のような降順入力でも元コードと同じ行順が保たれる。
    h_order = {h: i for i, h in enumerate(horizons)}
    sorted_horizons = sorted(horizons)
    min_h = sorted_horizons[0]

    # horizon 別の記録数カウンタ (進捗 print 用)
    h_counts = {h: 0 for h in horizons}

    # k_end = k0 + horizon ≤ n-1 まで使う (最小 horizon で outer range を決定)
    for k0 in range(0, n - min_h, stride):
        # -- モデルを t_{k0} の実機状態にリセット（過去履歴を delay queue にセット）--
        # ego_entity_simulation.cpp と同じ: state(4) = atan(wz*wb/vx) をキネマティック初期値に。
        # vm_reset_state 内: state(4) = steer_actual - steer_bias
        model.reset_with_history(
            x=gt_x[k0],
            y=gt_y[k0],
            yaw=gt_yaw[k0],
            vx=gt_vx[k0],
            steer_actual=float(gt_steer_kinematic[k0]) + steer_bias,
            ax=gt_ax[k0],
            acc_history=_delay_history(t_cmd[k0], acc_q_size, t_cmd_full, accel_des_full),
            steer_history=_delay_history(t_cmd[k0], steer_q_size, t_cmd_full, steer_des_full),
            wz=float(gt_wz[k0]),
        )

        # 各 horizon 境界まで増分ステップし、区間不正が見つかった時点で打ち切る。
        # 不正区間は元コードの per-horizon 事前チェック (ivs 全域 any(...)) と同等:
        # 区間 j の iv が不正なら j を含む horizon 以上はすべて記録しない。
        stepped = 0  # これまでにステップした区間数 (各 h まで積算)
        cos_y = math.cos(gt_yaw[k0])
        sin_y = math.sin(gt_yaw[k0])
        for h in sorted_horizons:
            if k0 >= n - h:
                # この k0 では horizon h が範囲外 (昇順なので以降も全て範囲外)
                break
            # -- 実コマンド系列を [stepped, h) 区間だけ追加適用 --
            # 各区間は n_full 回の SUB_DT ステップ + 端数ステップで正確に積分する
            # (区間が SUB_DT より短い場合は n_full=0 で端数ステップのみ)。
            bad = False
            for j in range(stepped, h):
                iv = t_cmd[k0 + j + 1] - t_cmd[k0 + j]
                if iv <= 0.001 or iv > 1.0:
                    bad = True
                    break
                ad = float(accel_des[k0 + j])
                sd = float(steer_des[k0 + j])
                n_full = int(iv / SUB_DT)
                for _ in range(n_full):
                    model.step(ad, sd)
                rem = iv - n_full * SUB_DT
                if rem > 1e-6:
                    model.step_dt(ad, sd, rem)
            if bad:
                # 不正区間 → h 以上の horizon はすべて記録しない (元コードの continue と同等)
                break
            stepped = h
            k_end = k0 + h

            # -- 終端誤差 (実機 − モデル) --
            dx = gt_x[k_end] - model.x
            dy = gt_y[k_end] - model.y
            yaw_err = (gt_yaw[k_end] - model.yaw + math.pi) % (2 * math.pi) - math.pi

            # -- 変位の k0 ヨー基準ローカル分解 --
            real_dx = gt_x[k_end] - gt_x[k0]
            real_dy = gt_y[k_end] - gt_y[k0]
            sim_dx = model.x - gt_x[k0]
            sim_dy = model.y - gt_y[k0]
            real_ds_long = real_dx * cos_y + real_dy * sin_y
            real_ds_lat = -real_dx * sin_y + real_dy * cos_y
            sim_ds_long = sim_dx * cos_y + sim_dy * sin_y
            sim_ds_lat = -sim_dx * sin_y + sim_dy * cos_y

            sim_steer_kend = model.steer_state + steer_bias
            # sim_wz: モデルが予測した終端 yaw rate (vm_get_wz, k_us 依存)。未 export なら NaN
            # (sim_vy/ay は getVy()=0 / 未実装のため引き続き省略)。
            sim_wz = model.wz
            recs.append({
                "horizon": h,
                "k0": k0,
                "tr": t_cmd[k0],  # AUTONOMOUS 開始からの経過時間 [s]
                "elapsed": t_cmd[k_end] - t_cmd[k0],
                "accel_des": float(accel_des[k_end - 1]),  # 最終適用コマンド (N=1 で旧 per-step と同一)
                "steer_des": float(steer_des[k_end - 1]),
                "pos_x": gt_x[k0],
                "pos_y": gt_y[k0],
                "real_vx": gt_vx[k0],
                "real_steer_k0": gt_steer[k0],  # リセット時の実機ステア
                "real_steer_kend": gt_steer[k_end],  # 終端の実機ステア（予測の比較対象）
                "sim_steer_kend": sim_steer_kend,  # モデルが予測した終端ステア
                "err_steer": gt_steer[k_end] - sim_steer_kend,  # 予測誤差 [rad]
                "real_ax": gt_ax[k0],
                "real_ay": gt_ay[k_end],  # 終端 — err_steer 規約に統一
                "real_vy": gt_vy[k_end],
                "real_wz": gt_wz[k_end],
                "err_wz": gt_wz[k_end] - sim_wz,  # yaw rate 予測誤差 [rad/s] (sim_wz は実機値と err から導出可)
                "real_dwz": gt_dwz[k_end],
                "sim_vx": model.vx,
                # 終端の縦方向 GT との比較 (sweep 目的メトリクス用)。real_vx/real_ax は k0 値の
                # ままにし意味を変えない。err_ax は加速度トピックが無いログ (gt_ax=zeros) では
                # 無意味になるが、縦方向 sweep 対象データには /localization/acceleration がある前提。
                "real_vx_kend": gt_vx[k_end],
                "err_vx": gt_vx[k_end] - model.vx,  # 速度予測誤差 [m/s]
                "real_ax_kend": gt_ax[k_end],
                "sim_ax": model.ax,
                "err_ax": gt_ax[k_end] - model.ax,  # 加速度予測誤差 [m/s²]
                "real_ds_long": real_ds_long,
                "real_ds_lat": real_ds_lat,
                "sim_ds_long": sim_ds_long,
                "sim_ds_lat": sim_ds_lat,
                "err_ds_long": real_ds_long - sim_ds_long,
                "err_ds_lat": real_ds_lat - sim_ds_lat,
                "pos_err": math.hypot(dx, dy),
                "yaw_err_deg": math.degrees(yaw_err),
            })
            h_counts[h] += 1

    # 入力 horizon の出現順 (h_order) + k0 でソートし、元の horizon-major 出力順を再現する。
    recs.sort(key=lambda r: (h_order[r["horizon"]], r["k0"]))

    for h in horizons:
        print(f"  horizon N={h}: {h_counts[h]} starts (stride={stride})")
    return pd.DataFrame(recs)


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------


def plot_overview(df: pd.DataFrame, params: dict) -> None:
    fig = build_fig_overview(df, params=params, limits_df=LIMITS_DF)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, OUT_DIR / "overview")


# ケース横断の軸範囲統一用 (step6 が全ケース連結 DataFrame をセットして再描画する)。
# None のとき各プロットは従来どおり自動スケール (step5 単体実行時)。
LIMITS_DF: pd.DataFrame | None = None


def _unified_absmax(col: str, scale: float = 1.0, horizon: int | None = None) -> float | None:
    """LIMITS_DF から col の |最大値| を返す (map_distribution の colorbar 統一用)。"""
    if LIMITS_DF is None:
        return None
    df = LIMITS_DF if horizon is None else LIMITS_DF[LIMITS_DF["horizon"] == horizon]
    if col not in df.columns:
        return None
    vals = df[col].to_numpy(dtype=float) * scale
    vals = vals[np.isfinite(vals)]
    return float(np.abs(vals).max()) if len(vals) else None


def _repr_horizons(df: pd.DataFrame) -> list[int]:
    """行表示に使う代表 horizon: [N=min] または [N=min, N=max]。"""
    h_min = int(df["horizon"].min())
    h_max = int(df["horizon"].max())
    return [h_min] if h_max == h_min else [h_min, h_max]


def _resolve_map_osm() -> Path | None:
    """lanelet2_map.osm を `_map.resolve_map_osm` の三状態モデルで解決する。"""
    return resolve_map_osm(os.environ.get("MAP_OSM_PATH"))


def _load_map_ways(osm_path: Path | None) -> list[np.ndarray] | None:
    if osm_path is None:
        return None
    ways = _load_map_ways_impl(osm_path)
    return ways if ways else None


def plot_map_distribution(df: pd.DataFrame, params: dict) -> None:
    """地図上の誤差分布 (上段 N=1, 下段 N=max。マーカー位置は rollout 開始点 k0)。"""
    rad2deg = 180.0 / math.pi
    map_ways = _load_map_ways(_resolve_map_osm())

    row_horizons = _repr_horizons(df)
    h_min = row_horizons[0]

    # 各行 = horizon、各列 = (列名, ラベル, 単位, データソース注)。
    # err_ay / err_vy / err_wz は sim_* がラッパー未 export のため一旦スキップ。
    # 下段 (N=max) はステアより累積 yaw 誤差が本質なので 3 列目を yaw に差し替える。
    def _cols(h: int) -> list[tuple[str, float, str, str, str]]:
        # (列名, スケール, ラベル, 単位, データソース注)
        cols = [
            ("err_ds_long", 100.0, f"縦方向誤差 (N={h})", "cm",
             "kinematic_state/pose.position vs rollout 終端 state_[0,1]"),
            ("err_ds_lat", 100.0, f"横方向誤差 (N={h})", "cm",
             "kinematic_state/pose.position vs rollout 終端 state_[0,1]"),
        ]
        if h == h_min:
            cols.append(("err_steer", rad2deg, f"ステア予測誤差 (N={h})", "deg",
                         "steering_status/tire_angle vs state_[4]+bias"))
        else:
            cols.append(("yaw_err_deg", 1.0, f"yaw 誤差 (N={h})", "deg",
                         "kinematic_state/pose.orientation vs rollout 終端 state_[2]"))
        return cols

    # 走行軌跡の bbox + margin で地図プロット範囲を自動算出
    x_min = float(df["pos_x"].min()) - MAP_BBOX_MARGIN
    x_max = float(df["pos_x"].max()) + MAP_BBOX_MARGIN
    y_min = float(df["pos_y"].min()) - MAP_BBOX_MARGIN
    y_max = float(df["pos_y"].max()) + MAP_BBOX_MARGIN

    # plotly インタラクティブ HTML（ズーム・パン・ホバー可）として出力する。
    from plotly.subplots import make_subplots  # noqa: PLC0415

    n_rows = len(row_horizons)
    n_cols = 3
    titles = []
    for h in row_horizons:
        titles += [f"{label} [{unit}]<br><sub>{source}</sub>" for _, _, label, unit, source in _cols(h)]
    fig = make_subplots(
        rows=n_rows,
        cols=n_cols,
        subplot_titles=titles,
        horizontal_spacing=0.10,
        vertical_spacing=0.14,
    )
    lane_ways = map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max)) if map_ways else []

    # レーン trace の NaN 区切り座標構築は全セル共通なので 1 回だけ行い、
    # 各セルにはそのコピーを add_trace する（trace は 1 軸にしか紐づけられないため）。
    lane_template = lanes_to_trace(lane_ways) if lane_ways else None
    for row_idx, h in enumerate(row_horizons, start=1):
        sub = df[df["horizon"] == h]
        pos_x = sub["pos_x"].to_numpy()
        pos_y = sub["pos_y"].to_numpy()
        for col_idx, (col_name, scale_, label, unit, source) in enumerate(_cols(h), start=1):
            vals = sub[col_name].values * scale_
            # plotly の軸名は 1 番目のみサフィックス無し（xaxis, x / xaxis2, x2 ...）
            axis_i = (row_idx - 1) * n_cols + col_idx
            axis_suffix = str(axis_i) if axis_i > 1 else ""
            if lane_template is not None:
                fig.add_trace(go.Scatter(lane_template), row=row_idx, col=col_idx)
            # ケース横断の colorbar 統一 (LIMITS_DF 設定時)
            unified = _unified_absmax(col_name, scale_, horizon=h)
            vmax = unified if unified is not None else max(abs(vals).max(), 1e-6)
            # 各セルの colorbar をサブプロット右端に配置（x/y axis domain を使う）
            x_dom = fig.layout[f"xaxis{axis_suffix}"].domain
            y_dom = fig.layout[f"yaxis{axis_suffix}"].domain
            fig.add_trace(
                go.Scatter(
                    x=pos_x,
                    y=pos_y,
                    mode="markers",
                    marker=dict(
                        color=vals,
                        colorscale="RdBu",
                        reversescale=True,  # matplotlib RdBu_r 相当
                        cmin=-vmax,
                        cmax=vmax,
                        size=5,
                        colorbar=dict(
                            title=unit,
                            x=x_dom[1] + 0.005,
                            y=(y_dom[0] + y_dom[1]) / 2,
                            len=y_dom[1] - y_dom[0],
                            thickness=12,
                        ),
                    ),
                    showlegend=False,
                    hovertemplate=(
                        f"x=%{{x:.1f}}m y=%{{y:.1f}}m<br>{label}=%{{marker.color:.2f}}{unit}"
                        "<extra></extra>"
                    ),
                ),
                row=row_idx,
                col=col_idx,
            )
            fig.update_xaxes(title_text="x [m]", range=[x_min, x_max], row=row_idx, col=col_idx)
            fig.update_yaxes(
                title_text="y [m]",
                range=[y_min, y_max],
                scaleanchor=f"x{axis_suffix}",
                scaleratio=1,
                row=row_idx,
                col=col_idx,
            )

    fig.update_layout(
        title=dict(
            text="N-step オープンループ: 地図上の誤差分布 (実機 − モデル、点=rollout 開始位置)",
            font=dict(size=14),
        ),
        # step11 はケースタブ（display:none）内で reveal-render する。responsive:true で
        # 表示時にリサイズされるが、初期幅 0 を避けるため明示サイズを与えておく。
        autosize=False,
        width=355 * n_cols,
        height=FIG_HEIGHTS["map_distribution"],
        margin=dict(l=60, r=40, t=110, b=40),
    )
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_fig_json(fig, OUT_DIR / "map_distribution")


def save_summary(df: pd.DataFrame) -> None:
    rad2deg = 180.0 / math.pi
    df1 = n1(df)
    tr = df1["tr"].values

    def rmse_cm(col, mask=None):
        v = df1[col].values if mask is None else df1[col].values[mask]
        return float(np.sqrt(np.mean(v**2))) * 100

    def rmse_deg(col, mask=None):
        v = df1[col].values if mask is None else df1[col].values[mask]
        return float(np.sqrt(np.mean(v**2))) * rad2deg

    def mean_deg(col, mask=None):
        v = df1[col].values if mask is None else df1[col].values[mask]
        return float(np.mean(v)) * rad2deg

    # メトリクス説明を先頭にコメント (# ) 行として埋め込む
    desc_lines = ["# " + ln if ln else "#" for ln in metrics_description_md().splitlines()]
    lines = [
        *desc_lines,
        "",
        "=== 全走行 N-step オープンループ解析 サマリ ===",
        f"horizons: {sorted(int(h) for h in df['horizon'].unique())}  (N=1 = 毎ステップリセット)",
        f"解析窓: tr={tr[0]:.1f}〜{tr[-1]:.1f}s",
        "",
        f"=== N=1 (per-step) 詳細  有効ステップ数: {len(df1)} ===",
        "",
        "--- 全区間: 位置 ---",
        f"縦方向 RMSE: {rmse_cm('err_ds_long'):.3f} cm",
        f"横方向 RMSE: {rmse_cm('err_ds_lat'):.3f} cm",
        "",
        "--- 全区間: ステア予測 (actual[kend] − pred[kend]) ---",
        f"ステア予測 RMSE: {rmse_deg('err_steer'):.4f} deg",
        f"ステア予測 平均誤差: {mean_deg('err_steer'):.4f} deg  (正=実機が指令より大/負=小)",
        "",
    ]
    # yaw rate 予測 RMSE (vm_get_wz export 時のみ)。steer/位置と異なり k_us 感度を持つ。
    if "err_wz" in df1.columns and np.isfinite(df1["err_wz"].values).any():
        wz_rmse = float(np.sqrt(np.nanmean(df1["err_wz"].values ** 2))) * rad2deg
        lines += [
            "--- 全区間: yaw rate 予測 (vm_get_wz, k_us 感度あり) ---",
            f"yaw rate 予測 RMSE: {wz_rmse:.4f} deg/s",
            "  (注: steer を k_us=0 の運動学逆算で seed するため、k_us が異なるケース間の "
            "小 N の err_wz 比較は seeding バイアスを含む。k_us 同定は大 N を参照)",
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
        mask = (df1["real_vx"].values >= lo) & (df1["real_vx"].values < hi)
        if mask.sum() > 0:
            lines.append(
                f"  {lbl:22s}: 縦={rmse_cm('err_ds_long', mask):.3f} cm, "
                f"横={rmse_cm('err_ds_lat', mask):.3f} cm, "
                f"ステア={rmse_deg('err_steer', mask):.4f} deg  (n={mask.sum()})"
            )

    # --- horizon 別 RMSE (free-running, リセット無し N ステップ後の対実機誤差) ---
    lines += ["", "=== horizon 別 終端誤差 RMSE (free-running) ==="]
    rmse = rmse_by_horizon(df)
    for horizon, r in rmse.items():
        sub = df[df["horizon"] == horizon]
        el = float(sub["elapsed"].mean())
        lines.append(
            f"  N={horizon:2d} (≈{el:.2f}s): 位置 RMSE={r['pos']:.2f} cm "
            f"(縦={r['long']:.2f}, 横={r['lat']:.2f} cm)  yaw RMSE={r['yaw']:.3f} deg  (n={len(sub)})"
        )

    # 透明化: sim_vx<0 (ideal_steer_acc が停止中ブレーキで微小後退する仕様) の件数を注記。
    # バグではなく dynamics-free モデルの忠実な挙動で、位置誤差寄与は無視可能 (VehicleModel docstring 参照)。
    if "sim_vx" in df1.columns:
        n_neg = int((df1["sim_vx"].values < 0).sum())
        if n_neg > 0:
            lines += [
                "",
                f"--- 注記: sim_vx<0 が {n_neg}/{len(df1)} step (N=1) ---",
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

    OUT_DIR は nstep/<case_tag>/ に固定。PARAMS は load_sim_params()
    の base に case_params を上書きしたもの。
    """
    global BASE, LITE_DIR, OUT_DIR, REAL_BAG_DIR, PARAMS, SUB_DT  # noqa: PLW0603

    BASE = cfg.base_dir
    LITE_DIR = cfg.lite_dir
    OUT_DIR = cfg.out_dir / "nstep" / case_tag
    # 入力 bag のデフォルトパス (実体は _resolve_real_mcap が file/dir 両対応)
    REAL_BAG_DIR = LITE_DIR / "real.lite.mcap"

    PARAMS = _build_params(cfg)
    PARAMS.update(case_params)
    SUB_DT = PARAMS["sub_dt"]
    return PARAMS


def main() -> None:
    parser = argparse.ArgumentParser(description="全走行 N-step オープンループ解析 (1 ケース 1 run)")
    add_common_cli_arguments(parser)
    # --scenario は add_common_cli_arguments が追加済み (env: SCENARIO_CONFIG_YAML)。
    parser.add_argument(
        "--case-tag",
        default=os.environ.get("CASE_TAG", ""),
        help="Conditions.cases の対象 tag (必須; env: CASE_TAG)",
    )
    args = parser.parse_args()

    if not args.case_tag:
        print("ERROR: --case-tag (or CASE_TAG env) が未指定です", file=sys.stderr)
        sys.exit(2)
    if not args.scenario:
        print(
            "ERROR: --scenario (or SCENARIO_CONFIG_YAML env) が未指定です。"
            "scenario.yaml の Conditions.models / cases に定義してください",
            file=sys.stderr,
        )
        sys.exit(2)

    from driving_log_replayer_v2.real_log_sim_comparison.lib._cases_config import (  # noqa: PLC0415
        load_cases_config,
    )

    try:
        cases_cfg = load_cases_config(args.scenario)
        case = cases_cfg.find_case(args.case_tag)
    except (FileNotFoundError, ValueError, KeyError) as e:
        print(f"ERROR: scenario.yaml (Conditions.cases): {e}", file=sys.stderr)
        sys.exit(2)
    print(f"[case] tag={case.tag}, vehicle_model_type={case.vehicle_model_type}, params={case.params}")

    cfg = build_runtime_config(args, default_base_dir=Path(__file__).parent)
    params = _apply_runtime_config(cfg, case.tag, case.params)

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # 入力 bag は単一 `.mcap` ファイル or `.lite` ディレクトリの両方を試す (lib._io 共通リゾルバ)
    real_bag = resolve_lite_bag(LITE_DIR, "real")
    if real_bag is None:
        print(f"ERROR: real lite bag が見つかりません: {LITE_DIR}", file=sys.stderr)
        sys.exit(1)
    print(f"Loading: {real_bag}")
    data = load_real_bag(real_bag)

    t0_ns = find_autonomous_start(data)
    print(f"AUTONOMOUS 開始: t0_ns={t0_ns}")

    print(f"\n=== N-step オープンループ解析開始 (case={case.tag}) ===")
    # N=1 は全ステップ (stride=1) で従来 per-step delta の解像度、
    # N>1 は dynamics 累積差の検出用に stride=5 でサンプリングする。
    # GT 準備は両呼び出しで同一なので 1 回だけ計算して共有する。
    gt = _prepare_gt(data, t0_ns, params)
    df1 = run_rollout(data, t0_ns, params, case.vehicle_model_type, horizons=(1,), stride=1, gt=gt)
    dfn = run_rollout(
        data, t0_ns, params, case.vehicle_model_type, horizons=(2, 5, 10, 20, 40), stride=5, gt=gt
    )
    df = pd.concat([df1, dfn], ignore_index=True)

    if df1.empty:
        print("ERROR: 有効なステップが 0 件でした", file=sys.stderr)
        sys.exit(1)

    print("\n=== 出力生成 ===")
    df.to_csv(OUT_DIR / "nstep_delta.csv", index=False)
    print(f"  Saved: {OUT_DIR / 'nstep_delta.csv'}")

    save_summary(df)

    # ケース別に残す図は概観 + 地図分布の 2 つ（ケース横断比較は step6 の overlay が担う）
    plot_map_distribution(df, params)
    plot_overview(df1, params)

    print(f"\n完了。出力先: {OUT_DIR}")


if __name__ == "__main__":
    main()
