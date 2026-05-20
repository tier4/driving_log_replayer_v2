"""rosbag2 / MCAP の統一 I/O ヘルパー.

`real.lite.mcap` (単一ファイル) と `real.lite/` (rosbag2 ディレクトリ) の
どちらの形式の bag も読める。トピック名候補リストを受け取って bag に
存在する最初のものを使う仕組みで、`/sub/...` を含む旧ログとも互換。
"""

from __future__ import annotations

from collections.abc import Iterator
import math
from pathlib import Path
import sys

import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def _detect_storage_id(bag_path: Path) -> str:
    """rosbag2_py に渡す storage_id を bag パスから推定する。"""
    if bag_path.is_dir():
        return ""  # rosbag2 metadata.yaml から自動判定
    ext = bag_path.suffix.lower()
    if ext == ".db3":
        return "sqlite3"
    # .mcap / 拡張子なし / その他は mcap として試行
    return "mcap"


def _open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=_detect_storage_id(bag_path)),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def list_bag_topics(bag_path: Path) -> dict[str, str]:
    """bag に含まれるトピック名 → メッセージ型名 の辞書を返す。"""
    reader = _open_reader(Path(bag_path))
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def resolve_topic(bag_path: Path, candidates: list[str]) -> str | None:
    """candidates のうち bag に存在する最初のトピック名を返す。なければ None。"""
    available = set(list_bag_topics(bag_path).keys())
    for c in candidates:
        if c in available:
            return c
    return None


def iter_bag_messages(bag_path: Path, topics: list[str]) -> Iterator[tuple[int, object]]:
    """指定トピックのメッセージを (timestamp_ns, deserialized_msg) として反復する。

    bag_path はディレクトリ形式と単一ファイル形式の両方を受け付ける。
    bag に存在しないトピックは無視される（INFO ログのみ）。
    """
    bag_path = Path(bag_path)
    reader = _open_reader(bag_path)
    topic_type_map = {
        t.name: get_message(t.type)
        for t in reader.get_all_topics_and_types()
        if t.name in topics
    }
    missing = set(topics) - set(topic_type_map.keys())
    if missing:
        print(
            f"[INFO] iter_bag_messages: bag '{bag_path.name}' にトピックなし: {sorted(missing)}",
            file=sys.stderr,
        )
    if not topic_type_map:
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topic_type_map)))
    while reader.has_next():
        topic_name, msg_bytes, timestamp = reader.read_next()
        if topic_name in topic_type_map:
            yield timestamp, deserialize_message(msg_bytes, topic_type_map[topic_name])


def iter_to_df(
    bag_path: Path,
    topic: str | list[str],
    row_fn,
    columns: list[str],
) -> pd.DataFrame:
    """単一トピックから DataFrame を構築する。

    topic がリストの場合は bag に存在する最初の候補を使う。どれも無ければ空 DF。
    """
    bag_path = Path(bag_path)
    if isinstance(topic, str):
        resolved: str | None = topic
    else:
        resolved = resolve_topic(bag_path, topic)
        if resolved is None:
            return pd.DataFrame(columns=columns)
    rows = [row_fn(t_ns, msg) for t_ns, msg in iter_bag_messages(bag_path, [resolved])]
    if not rows:
        return pd.DataFrame(columns=columns)
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 高位 loader 群 — 戻り値は (t_ns: int64, ...) 形式の DataFrame
# ---------------------------------------------------------------------------

# 既定のトピック候補 (sub-less / sub-prefixed 両対応)
DEFAULT_TOPICS = {
    "operation_mode": ["/system/operation_mode/state"],
    "velocity": [
        "/vehicle/status/velocity_status",
        "/sub/vehicle/status/velocity_status",
    ],
    "steering": [
        "/vehicle/status/steering_status",
        "/sub/vehicle/status/steering_status",
    ],
    "kinematic": [
        "/localization/kinematic_state",
        "/sub/localization/kinematic_state",
    ],
    "accel": [
        "/localization/acceleration",
        "/sub/localization/acceleration",
    ],
}


def load_operation_mode(bag_path: Path) -> pd.DataFrame:
    return iter_to_df(
        bag_path,
        DEFAULT_TOPICS["operation_mode"],
        lambda t_ns, m: {"t_ns": t_ns, "mode": m.mode},
        ["t_ns", "mode"],
    )


def load_velocity(bag_path: Path, topic: str | list[str] | None = None) -> pd.DataFrame:
    return iter_to_df(
        bag_path,
        topic if topic is not None else DEFAULT_TOPICS["velocity"],
        lambda t_ns, m: {"t_ns": t_ns, "lon_vel": m.longitudinal_velocity},
        ["t_ns", "lon_vel"],
    )


def load_steering(bag_path: Path, topic: str | list[str] | None = None) -> pd.DataFrame:
    return iter_to_df(
        bag_path,
        topic if topic is not None else DEFAULT_TOPICS["steering"],
        lambda t_ns, m: {"t_ns": t_ns, "steer": m.steering_tire_angle},
        ["t_ns", "steer"],
    )


def load_kinematic(bag_path: Path, topic: str | list[str] | None = None) -> pd.DataFrame:
    def row(t_ns, m):
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        return {"t_ns": t_ns, "x": p.x, "y": p.y, "yaw": yaw}

    return iter_to_df(
        bag_path,
        topic if topic is not None else DEFAULT_TOPICS["kinematic"],
        row,
        ["t_ns", "x", "y", "yaw"],
    )


def load_accel(bag_path: Path, topic: str | list[str] | None = None) -> pd.DataFrame:
    return iter_to_df(
        bag_path,
        topic if topic is not None else DEFAULT_TOPICS["accel"],
        lambda t_ns, m: {"t_ns": t_ns, "accel": m.accel.accel.linear.x},
        ["t_ns", "accel"],
    )


def load_cmd(bag_path: Path, topic: str | list[str]) -> pd.DataFrame:
    """control_cmd 系のロード。

    トピック名は呼び出し側で明示すること
    (実機=`/control/command/control_cmd`,
     シム=`/control/trajectory_follower/control_cmd` 等で異なるため)。
    """
    return iter_to_df(
        bag_path,
        topic,
        lambda t_ns, m: {
            "t_ns": t_ns,
            "cmd_vel": m.longitudinal.velocity,
            "cmd_accel": m.longitudinal.acceleration,
            "cmd_steer": m.lateral.steering_tire_angle,
        },
        ["t_ns", "cmd_vel", "cmd_accel", "cmd_steer"],
    )


# ---------------------------------------------------------------------------
# 時刻整列・幾何ヘルパー
# ---------------------------------------------------------------------------


def align_time(df: pd.DataFrame, t0_ns: int) -> pd.DataFrame:
    """`t_ns` を `t0_ns` で相対化し、列 't' [s] を付与する。負の時刻は除外。"""
    if df.empty:
        return pd.DataFrame(columns=[*df.columns, "t"])
    df = df.copy()
    df["t"] = (df["t_ns"] - t0_ns) / 1e9
    return df[df["t"] >= 0].reset_index(drop=True)


def nearest_point_distance(ref_xy: np.ndarray, query_xy: np.ndarray) -> np.ndarray:
    """各 query 点に対する ref_xy の最近傍距離を返す。"""
    from scipy.spatial import cKDTree

    tree = cKDTree(ref_xy)
    dists, _ = tree.query(query_xy)
    return dists
