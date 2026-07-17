"""入力 bag から抽出した ``reidentify_cache.csv`` を読み込む。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from .csv_schema import CACHE_NAME
from .csv_schema import SIGNAL_COLUMNS
from .gear import require_drive_gear_mask
from .settings import ACCEL_SOURCE
from ..lib._accel_source import accel_on_grid


def read_dataset_csv(csv_path: Path) -> dict[str, pd.DataFrame]:
    """CSV キャッシュを topic 別 DataFrame 群に復元する。"""
    raw = pd.read_csv(csv_path)
    out: dict[str, pd.DataFrame] = {}
    for topic, value_cols in SIGNAL_COLUMNS.items():
        cols = ["t_ns", *value_cols]
        sub = (
            raw[raw["topic"] == topic][cols]
            .dropna(how="any")
            .sort_values("t_ns")
            .reset_index(drop=True)
        )
        sub["t_ns"] = sub["t_ns"].astype(np.int64)
        if topic == "gear":
            sub["gear"] = sub["gear"].astype(int)
        if topic == "mode":
            sub["mode"] = sub["mode"].astype(int)
        out[topic] = sub
    return out


def discover_cached_datasets(collection_dir: Path) -> list[tuple[str, Path]]:
    """CSV キャッシュ (reidentify_cache.csv) の存在するデータセットを列挙する。"""
    datasets_dir = collection_dir / "datasets"
    if not datasets_dir.is_dir():
        return []
    result: list[tuple[str, Path]] = []
    for dataset_dir in sorted(datasets_dir.iterdir()):
        if not dataset_dir.is_dir():
            continue
        csv_path = dataset_dir / CACHE_NAME
        if csv_path.exists():
            result.append((dataset_dir.name, csv_path))
    return result


def build_resampled(
    dfs: dict[str, pd.DataFrame], dt: float, *, context: str,
    acceleration_source: str = ACCEL_SOURCE,
) -> dict | None:
    """
    同定用の信号を一定周期のグリッドへ補間する。

    戻り値: {a_cmd, a_act, d_cmd, d_act, v_cmd, vx, wz, pitch, gear_drive} (float32配列) | None。
    """
    df_cmd = dfs["cmd"]
    df_accel = dfs["accel"]
    df_steer = dfs["steering"]
    df_vel = dfs["velocity"]
    df_kin = dfs["kinematic"]
    df_gear = dfs["gear"]

    if df_cmd.empty or df_accel.empty or df_steer.empty or df_vel.empty or df_kin.empty:
        return None
    t0 = max(df_cmd["t_ns"].iloc[0], df_accel["t_ns"].iloc[0], df_steer["t_ns"].iloc[0],
              df_vel["t_ns"].iloc[0], df_kin["t_ns"].iloc[0])
    t1 = min(df_cmd["t_ns"].iloc[-1], df_accel["t_ns"].iloc[-1], df_steer["t_ns"].iloc[-1],
              df_vel["t_ns"].iloc[-1], df_kin["t_ns"].iloc[-1])
    if (t1 - t0) < 2e9:
        return None

    t_ns = np.arange(t0, t1, dt * 1e9, dtype=np.float64)
    t_s = (t_ns - t0) * 1e-9

    a_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_accel"].values)

    a_act = accel_on_grid(
        acceleration_source,
        df_accel=df_accel,
        df_vel=df_vel,
        df_kin=df_kin,
        t_s=t_s,
        t0_ns=t0,
        dt=dt,
    )

    d_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_steer"].values)
    v_cmd = np.interp(t_s, (df_cmd["t_ns"].values - t0) * 1e-9, df_cmd["cmd_vel"].values)
    d_act = np.interp(t_s, (df_steer["t_ns"].values - t0) * 1e-9, df_steer["steer"].values)
    vx = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)
    wz = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["wz"].values)
    if "pitch" in df_kin.columns:
        pitch = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["pitch"].values)
    else:
        # 旧スキーマの CSV キャッシュ互換 (pitch 列なし)。
        pitch = np.zeros_like(t_s)
    gear_drive = require_drive_gear_mask(
        df_gear, t_ns.astype(np.int64), context=context, allow_leading_gap=True,
    )

    return {
        "a_cmd": a_cmd.astype(np.float32),
        "a_act": a_act.astype(np.float32),
        "d_cmd": d_cmd.astype(np.float32),
        "d_act": d_act.astype(np.float32),
        "v_cmd": v_cmd.astype(np.float32),
        "vx": vx.astype(np.float32),
        "wz": wz.astype(np.float32),
        "pitch": pitch.astype(np.float32),
        "gear_drive": gear_drive,
    }


def build_rollout_data(
    dfs: dict[str, pd.DataFrame], *,
    acceleration_source: str = "accel",
    steering_source: str = "steer",
    observation_frame: str = "raw",
) -> dict[str, pd.DataFrame]:
    """Rollout 評価が使う DataFrame 群を組み立てる。

    observation_frame="localization_consistent" のとき、kinematic_state を pose アンカーの
    整合フレームへ補正してから GT を組み立てる (pose +POSE_LAG_S 前進 / twist vx ×0.996、
    lib/_localization_observation 参照)。kinematic_* 系の加速度 GT は補正済み vx から導出
    されるため、cmd→ax→vx→long の縦系チェーンが単一系統で閉じる。velocity トピック
    (車両側 velocity report) と accel 生トピックは localization 系ではないため補正しない。
    """
    df_vel = dfs["velocity"].rename(columns={"lon_vel": "vx"})
    from ..lib._accel_source import accel_dataframe_from_source
    from ..lib._localization_observation import consistent_kinematic_frame, normalize_observation_frame
    from ..lib._steer_source import steer_dataframe_from_source

    frame = normalize_observation_frame(observation_frame)
    df_kin = dfs["kinematic"]
    if frame == "localization_consistent":
        df_kin = consistent_kinematic_frame(df_kin)

    df_acc = accel_dataframe_from_source(
        acceleration_source,
        df_accel=dfs["accel"],
        df_vel=dfs["velocity"],
        df_kin=df_kin,
        out_col="ax",
    )
    df_acc["ay"] = 0.0
    df_steer = steer_dataframe_from_source(steering_source, df_steer=dfs["steering"])
    df_cmd = dfs["cmd"].rename(columns={"cmd_accel": "accel_des", "cmd_steer": "steer_des"})
    df_cmd = df_cmd[["t_ns", "accel_des", "steer_des"]]
    return {
        "mode": dfs["mode"],
        "vel": df_vel,
        "steer": df_steer,
        "kin": df_kin,
        "acc": df_acc,
        "cmd": df_cmd,
        "gear": dfs["gear"],
    }
