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

# DiffusionPlanner 出力軌跡トピック (step4 playback / step8 比較が共用)。
# `/sub` プレフィックス付き旧ログは resolve_topic の候補リストで吸収する。
DP_TRAJ_TOPIC = (
    "/planning/trajectory_generator/neural_network_based_planner/"
    "diffusion_planner_node/output/trajectory"
)


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


def resolve_lite_bag(lite_dir: Path, name_stem: str) -> Path | None:
    """`<name_stem>.lite.mcap` (単一ファイル) → `<name_stem>.lite` (rosbag2 dir) の順で解決。

    どちらも存在しなければ None。返り値は lib._io の loader 群がそのまま受け取れる
    (file / dir 両対応)。step5/8/9/10 で重複していた bag 解決をここに集約。
    """
    lite_dir = Path(lite_dir)
    for cand in (lite_dir / f"{name_stem}.lite.mcap", lite_dir / f"{name_stem}.lite"):
        if cand.exists():
            return cand
    return None


def sim_tag_from_bag(bag: Path) -> str:
    """sim lite bag のパスから run tag を取り出す (例: sim_normal.lite.mcap → sim_normal)。

    プロットの凡例・タイトルに「どのシミュレータか」を表示するための表示名として使う。
    """
    name = Path(bag).name
    if name.endswith(".mcap"):
        name = name[: -len(".mcap")]
    if name.endswith(".lite"):
        name = name[: -len(".lite")]
    return name


def resolve_primary_sim_bag(lite_dir: Path) -> Path | None:
    """比較対象の sim lite を自動検出する (real を除く sim_*.lite、sim_normal を優先)。

    旧実装は専用シナリオ由来の `sim_curve2.lite` 固定だったが、現パイプラインは sim_runs.yaml の
    各 run (sim_normal/sim_kus0020/sim_perfect/sim_godot 等) を生成するため、それらから 1 つ
    (sim_normal 優先) を選ぶ。
    """
    lite_dir = Path(lite_dir)
    stems: list[str] = []
    for pat in ("sim_*.lite", "sim_*.lite.mcap"):
        for p in lite_dir.glob(pat):
            stem = sim_tag_from_bag(p)
            if stem and stem not in stems:
                stems.append(stem)
    if not stems:
        return None
    stems.sort(key=lambda s: (s != "sim_normal", s))  # sim_normal を先頭に
    return resolve_lite_bag(lite_dir, stems[0])


def cumulative_arc_length(x, y) -> np.ndarray:
    """(x, y) 系列に沿った累積走行距離 [m] を返す（先頭は 0）。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) == 0:
        return np.zeros(0)
    seg = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate([[0.0], np.cumsum(seg)])


def filter_localization(
    df: pd.DataFrame,
    *,
    max_step_speed: float = 60.0,
    max_step_jump: float = 50.0,
    origin_eps: float = 1.0,
) -> tuple[pd.DataFrame, int]:
    """localization 軌跡から無効フレームを除外する。

    sim 由来 bag は localization 初期化前に原点 (0,0) を出力したり、初期化時に
    巨大ジャンプ (瞬間移動) を挟むことがあり、これが arc-length / bbox / 空間統計を
    破壊する（実機 bag は通常 0 件）。次の 2 段で除外する:

      1. 原点近傍 (|x|<origin_eps かつ |y|<origin_eps)
      2. ロバスト中心 (median) に最も近い有効点を起点に前後へ走査し、直前の有効点から
         `max_step_speed` [m/s] を超える瞬間移動となる点（実走行ではあり得ない）

    戻り値: (除外後 df, 除外フレーム数)。`t_ns` 列があれば実時間で、無ければ等間隔で評価。
    """
    n0 = len(df)
    if n0 == 0 or "x" not in df.columns or "y" not in df.columns:
        return df, 0
    work = df.reset_index(drop=True)
    x = work["x"].to_numpy(dtype=float)
    y = work["y"].to_numpy(dtype=float)
    valid = ~((np.abs(x) < origin_eps) & (np.abs(y) < origin_eps))
    if "t_ns" in work.columns:
        t = work["t_ns"].to_numpy(dtype=float) / 1e9
    else:
        t = np.arange(n0, dtype=float)
    idx_valid = np.where(valid)[0]
    if len(idx_valid) >= 2:
        mx = float(np.median(x[idx_valid]))
        my = float(np.median(y[idx_valid]))
        d2 = (x - mx) ** 2 + (y - my) ** 2
        d2[~valid] = np.inf
        seed = int(np.argmin(d2))

        def _walk(order: range) -> None:
            anchor = seed
            for i in order:
                if not valid[i]:
                    continue
                dt = abs(t[i] - t[anchor])
                dist = math.hypot(x[i] - x[anchor], y[i] - y[anchor])
                # dt>0 は速度ゲートで teleport を捕捉 (正当な計測ギャップも実速度内なら通過する
                # ためカスケード除外しない)。dt==0 (同一タイムスタンプ) は速度ゲートが効かないので、
                # その場合のみ絶対ジャンプ上限 max_step_jump で teleport を落とす。
                if dt > 0:
                    invalid = dist / dt > max_step_speed
                else:
                    invalid = dist > max_step_jump
                if invalid:
                    valid[i] = False
                else:
                    anchor = i

        _walk(range(seed + 1, n0))
        _walk(range(seed - 1, -1, -1))
    clean = work[valid].reset_index(drop=True)
    return clean, n0 - len(clean)
