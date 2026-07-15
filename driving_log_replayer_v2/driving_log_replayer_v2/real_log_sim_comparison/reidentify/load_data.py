"""入力 bag から抽出した ``reidentify_cache.csv`` を読み込む。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from ..lib._accel_source import accel_on_grid
from .csv_schema import CACHE_NAME, SIGNAL_COLUMNS
from .gear import require_drive_gear_mask
from .settings import ACCEL_SOURCE


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
    """同定用の信号を一定周期のグリッドへ補間する。

    戻り値: {a_cmd, a_act, d_cmd, d_act, vx, wz, gear_drive} (float32配列) | None。
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
    d_act = np.interp(t_s, (df_steer["t_ns"].values - t0) * 1e-9, df_steer["steer"].values)
    vx = np.interp(t_s, (df_vel["t_ns"].values - t0) * 1e-9, df_vel["lon_vel"].values)
    wz = np.interp(t_s, (df_kin["t_ns"].values - t0) * 1e-9, df_kin["wz"].values)
    gear_drive = require_drive_gear_mask(
        df_gear, t_ns.astype(np.int64), context=context, allow_leading_gap=True,
    )

    return {
        "a_cmd": a_cmd.astype(np.float32),
        "a_act": a_act.astype(np.float32),
        "d_cmd": d_cmd.astype(np.float32),
        "d_act": d_act.astype(np.float32),
        "vx": vx.astype(np.float32),
        "wz": wz.astype(np.float32),
        "gear_drive": gear_drive,
    }


def build_rollout_data(
    dfs: dict[str, pd.DataFrame], *, acceleration_source: str = "accel"
) -> dict[str, pd.DataFrame]:
    """rollout 評価が使う DataFrame 群を組み立てる。"""
    df_vel = dfs["velocity"].rename(columns={"lon_vel": "vx"})
    from ..lib._accel_source import accel_dataframe_from_source

    df_acc = accel_dataframe_from_source(
        acceleration_source,
        df_accel=dfs["accel"],
        df_vel=dfs["velocity"],
        df_kin=dfs["kinematic"],
        out_col="ax",
    )
    df_acc["ay"] = 0.0
    df_cmd = dfs["cmd"].rename(columns={"cmd_accel": "accel_des", "cmd_steer": "steer_des"})
    df_cmd = df_cmd[["t_ns", "accel_des", "steer_des"]]
    return {
        "mode": dfs["mode"],
        "vel": df_vel,
        "steer": dfs["steering"],
        "kin": dfs["kinematic"],
        "acc": df_acc,
        "cmd": df_cmd,
        "gear": dfs["gear"],
    }
