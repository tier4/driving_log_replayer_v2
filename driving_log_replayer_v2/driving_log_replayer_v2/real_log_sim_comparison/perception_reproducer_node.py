#!/usr/bin/env python3
# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""ego-pose 同期 perception reproducer (real_log_sim_comparison の Stage 3 伴走ノード)。

実機 input_bag から ego_odom (/localization/kinematic_state) と
tracked objects (/perception/object_recognition/tracking/objects, DiffusionPlanner が購読) を読み、
scenario_test_runner sim の ego pose に同期して objects を再生する。これにより NPC を持たない
auto-generated scenario に実機の先行車を注入し、sim ego が実機の先行車追従挙動 (発進前/カーブ等の
停止・加減速) を再現できる。

再生アルゴリズム (デッドロック/runaway 両回避):
  - 走行中 (ego_v > stop_v): playhead = ego 最近傍の記録フレーム (pose-sync)。lead は実機の
    相対位置に置かれ、ego が動けば lead も動く -> ego は実ペースで追従し runaway しない。
  - 停止中 (ego_v <= stop_v): playhead を記録 real-pace で時間前進 -> 記録の dwell->departure を
    再生し lead が動き出す -> ego が解放される (実機の「停止 -> 発進」を再現)。
  - playhead は単調 (後退しない)。周回経路の自己近接で遠い arc を誤マッチしないよう、nearest は
    playhead 近傍窓で探索する。
  - 公開 objects の stamp は受信した sim ego の stamp (sim time) に合わせる (use_sim_time 不要)。

注意: sim 側 perception も同 topic を (NPC 無なら空で) publish するが、depth-1 latest-wins + 高 rate
publish で実用上支配でき、追従は成立する (sidecar 検証済 2026-06)。
"""
from __future__ import annotations

import argparse
import bisect
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message

import rosbag2_py
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrackedObjects

EGO_TOPIC = "/localization/kinematic_state"
OBJ_TOPIC = "/perception/object_recognition/tracking/objects"


def _detect_storage_id(bag_dir: str) -> str:
    """input_bag ディレクトリ内のファイルから storage_id を推定 (mcap / sqlite3)。"""
    from pathlib import Path

    p = Path(bag_dir)
    if list(p.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def _load_bag(bag_dir: str) -> tuple[list, list]:
    """input_bag から ego_odom (t_ns, x, y) と tracked objects (t_ns, msg) を読み込む。"""
    storage_id = _detect_storage_id(bag_dir)
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    co = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[EGO_TOPIC, OBJ_TOPIC]))
    ego: list = []
    objs: list = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == EGO_TOPIC:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            ego.append((t_ns, p.x, p.y))
        elif topic == OBJ_TOPIC:
            objs.append((t_ns, deserialize_message(data, TrackedObjects)))
    return ego, objs


class PerceptionReproducer(Node):
    def __init__(self, ego: list, objs: list, stop_v: float, window_back: int, window_fwd: int):
        super().__init__("perception_reproducer")
        self._ex = [e[1] for e in ego]
        self._ey = [e[2] for e in ego]
        self._n = len(ego)
        # arc-length (情報ログ用)
        self._arc = [0.0]
        for i in range(1, self._n):
            self._arc.append(self._arc[-1] + math.hypot(self._ex[i] - self._ex[i - 1],
                                                         self._ey[i] - self._ey[i - 1]))
        # ego_idx -> 最近傍時刻の objects index
        obj_t = [t for t, _ in objs]
        self._objs = [m for _, m in objs]
        self._obj_for = []
        for t, _, _ in ego:
            j = bisect.bisect_left(obj_t, t)
            cands = [k for k in (j - 1, j) if 0 <= k < len(obj_t)]
            self._obj_for.append(min(cands, key=lambda k: abs(obj_t[k] - t)) if cands else 0)

        self._stop_v = stop_v
        self._wb = window_back
        self._wf = window_fwd
        self._latest = None  # (x, y, v, stamp)
        self._playhead: float | None = None
        self._n_pub = 0

        avg = (ego[-1][0] - ego[0][0]) / 1e9 / max(1, self._n - 1) if self._n > 1 else 0.05
        self._pub = self.create_publisher(TrackedObjects, OBJ_TOPIC, 1)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self._sub = self.create_subscription(Odometry, EGO_TOPIC, self._on_ego, qos)
        self._timer = self.create_timer(max(avg, 0.01), self._on_timer)
        self.get_logger().info(
            f"perception_reproducer ready: ego={self._n} objs={len(self._objs)} "
            f"arc={self._arc[-1]:.0f}m interval={avg * 1000:.1f}ms stop_v={stop_v}")

    def _on_ego(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if abs(p.x) < 1.0 and abs(p.y) < 1.0:
            return  # localization 未初期化 (原点) は無視
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._latest = (p.x, p.y, v, msg.header.stamp)

    def _nearest_idx(self, x: float, y: float, lo: int, hi: int) -> int:
        lo = max(0, lo)
        hi = min(self._n, hi)
        if hi <= lo:
            lo, hi = 0, self._n
        best = (1e18, lo)
        for i in range(lo, hi):
            d = (self._ex[i] - x) ** 2 + (self._ey[i] - y) ** 2
            if d < best[0]:
                best = (d, i)
        return best[1]

    def _on_timer(self) -> None:
        if self._latest is None or self._n == 0:
            return
        x, y, ego_v, stamp = self._latest
        if self._playhead is None:
            self._playhead = float(self._nearest_idx(x, y, 0, self._n))
        ph = int(self._playhead)
        ni = self._nearest_idx(x, y, ph - self._wb, ph + self._wf)
        if ego_v > self._stop_v:
            self._playhead = float(max(ph, ni))                    # 走行中: pose-sync (単調)
        else:
            self._playhead = min(self._playhead + 1.0, self._n - 1)  # 停止中: 時間前進
        idx = int(self._playhead)
        msg = self._objs[self._obj_for[idx]]
        msg.header.stamp = stamp
        self._pub.publish(msg)
        self._n_pub += 1
        if self._n_pub % 200 == 0:
            self.get_logger().info(
                f"ego_arc={self._arc[ni]:.0f} v={ego_v:.1f} play_arc={self._arc[idx]:.0f} "
                f"{len(msg.objects)}obj (#{self._n_pub})")


def main() -> None:
    parser = argparse.ArgumentParser(description="ego-pose 同期 perception reproducer")
    parser.add_argument("--bag", required=True, help="実機 input_bag ディレクトリ")
    parser.add_argument("--stop-v", type=float, default=0.5,
                        help="この速度以下を停止とみなし time-advance する [m/s]")
    parser.add_argument("--window-back", type=int, default=50,
                        help="nearest 探索の後方窓 [frame]")
    parser.add_argument("--window-fwd", type=int, default=600,
                        help="nearest 探索の前方窓 [frame]")
    args, _ = parser.parse_known_args()

    print(f"[perception_reproducer] loading bag: {args.bag}", flush=True)
    ego, objs = _load_bag(args.bag)
    print(f"[perception_reproducer] loaded ego={len(ego)} objs={len(objs)}", flush=True)
    if not ego or not objs:
        print("[perception_reproducer] ego/objects が空のため再生しない", flush=True)
        return

    rclpy.init()
    node = PerceptionReproducer(ego, objs, args.stop_v, args.window_back, args.window_fwd)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
