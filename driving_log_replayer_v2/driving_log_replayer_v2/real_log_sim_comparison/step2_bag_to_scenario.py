"""Stage 2: 実機 input_bag → OpenSCENARIO yaml 自動生成.

実機 rosbag から:
- /localization/kinematic_state の AUTONOMOUS 区間先頭/末尾を start/goal pose とし、
  map 上で一意 lanelet に解決 (LanePosition) して OpenSCENARIO の TeleportAction と
  RoutingAction.AssignRouteAction (start + goal の 2 点) に注入。経路自体は
  Autoware mission_planner に計算させる。
- /perception/traffic_light_recognition/traffic_signals の信号タイムシリーズを抽出し、
  状態変化点ごとに TrafficSignalStateAction を SimulationTimeCondition でトリガー。

終了条件は「厳密さより同じコースで確実に切り上げること」を優先する (_build_end_condition_act):
ゴール厳密到達 (exitSuccess) に加え、一定時間経過後にゴール近傍 (寛容 tolerance) へ入れば
停止/通過/周回いずれでも切り上げる。これでゴール付近で止まってもタイムアウトせず、通過しても
周回し続けない。ループ経路 (start≈goal) での開始直後の誤発火は実走 course 時間に基づく時間
ゲートで防ぐ。長時間静止での切り上げは opt-in (既定無効; reproduce_perception の dwell を壊す
ため)。TraveledDistanceCondition は scenario_simulator_v2 未サポートのため使わない。

位置を WorldPosition ではなく LanePosition で渡すのは、 並走レーンが重複する curve で
WorldPosition が一意 lane に解決できず scenario_simulator が落ちるため
(_world_to_lane_position 参照)。中間 Waypoint は入れない (mission_planner の
ペアワイズ routing を壊し ego 静止を招くため。 build_scenario_dict 参照)。

Usage:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.step2_bag_to_scenario \\
        --input-bag <input_bag_dir> \\
        --map <lanelet2_map.osm absolute path> \\
        --output <auto_scenario.yaml> \\
        [--scenario-name "auto"] [--sim-timeout 600] [--goal-tolerance 15]

注意: step1_make_lite の TOPICS には route / 信号が含まれないため、本ツールは
lite ではなく **生の input_bag** を読む。evaluator_node からは
`t4_dataset_path/input_bag` を渡す。
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Any
import xml.etree.ElementTree as ET

import yaml

from .lib._io import iter_bag_messages, list_bag_topics, load_kinematic, load_operation_mode, load_velocity
from .lib._events import find_autonomous_start


# ---------------------------------------------------------------------------
# Topic candidates
# ---------------------------------------------------------------------------

_ROUTE_TOPICS = [
    "/planning/mission_planning/route",
]
_SIGNAL_TOPICS = [
    "/perception/traffic_light_recognition/traffic_signals",
    "/perception/traffic_light_recognition/traffic_lights",
]


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """quaternion → yaw (rad)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _pose_to_world(pose) -> dict:
    """geometry_msgs/Pose → OpenSCENARIO WorldPosition dict."""
    p = pose.position
    o = pose.orientation
    return {
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "h": float(_quat_to_yaw(o.x, o.y, o.z, o.w)),
        "p": 0.0,
        "r": 0.0,
    }


# ---------------------------------------------------------------------------
# Route extraction
# ---------------------------------------------------------------------------


def _extract_route_info(input_bag: Path) -> tuple[dict | None, dict | None, list[int]]:
    """Route トピックの最初のメッセージから (start_pose, goal_pose, lane_ids) を返す。

    以下の 2 つのメッセージ型に対応:
    - autoware_planning_msgs/msg/LaneletRoute: start_pose / goal_pose / seg.preferred_primitive.id
    - autoware_adapi_v1_msgs/msg/Route:        data[0].start / data[0].goal / seg.preferred.id

    - start_pose: route の geometry 上の起点。InitialPose にこれを使うことで sim ego が
      route の起点から Waypoint sequence を順に辿れる。実機の物理位置 (kinematic) とは異なる場合あり。
    - goal_pose: ユーザが指定した目的地 (route 計算時も走行中も不変)。
    - lane_ids: segments の preferred lane id (経路全体、絶対要件: 全保持)。

    route topic 不在時は (None, None, [])。
    """
    topics_in_bag = list_bag_topics(input_bag)
    route_topic = next((t for t in _ROUTE_TOPICS if t in topics_in_bag), None)
    if route_topic is None:
        return None, None, []
    for _t_ns, msg in iter_bag_messages(input_bag, [route_topic]):
        # autoware_adapi_v1_msgs/msg/Route: data[0].start / data[0].goal / seg.preferred.id
        # autoware_planning_msgs/msg/LaneletRoute: start_pose / goal_pose / seg.preferred_primitive.id
        if hasattr(msg, "start_pose"):
            start = _pose_to_world(msg.start_pose)
            goal = _pose_to_world(msg.goal_pose)
            lane_ids: list[int] = []
            for seg in msg.segments:
                if hasattr(seg, "preferred_primitive") and hasattr(seg.preferred_primitive, "id"):
                    lane_ids.append(int(seg.preferred_primitive.id))
        elif hasattr(msg, "data") and msg.data:
            start = _pose_to_world(msg.data[0].start)
            goal = _pose_to_world(msg.data[0].goal)
            lane_ids = []
            for seg in msg.data[0].segments:
                if hasattr(seg, "preferred") and hasattr(seg.preferred, "id"):
                    lane_ids.append(int(seg.preferred.id))
        else:
            return None, None, []
        return start, goal, lane_ids
    return None, None, []


def _initial_pose_from_kinematic(input_bag: Path, t0_ns: int) -> dict:
    """InitialPose: AUTONOMOUS 開始時刻 (t0_ns) に最も近い kinematic_state pose。

    make_lite が抽出する real.lite (= AUTONOMOUS 区間) の先頭と一致するため、
    sim と実機が同じ場所・同じ区間を走る。
    """
    df_kin = load_kinematic(input_bag)
    if df_kin.empty:
        raise RuntimeError(
            f"kinematic_state が無いため InitialPose を取得できません: {input_bag}"
        )
    try:
        row = df_kin.iloc[(df_kin["t_ns"] - t0_ns).abs().argsort().iloc[0]]
    except Exception:
        row = df_kin.iloc[0]
    return {"x": float(row.x), "y": float(row.y), "z": 0.0,
            "h": float(row.yaw), "p": 0.0, "r": 0.0}


def _goal_pose_from_kinematic(input_bag: Path, t0_ns: int) -> dict:
    """GoalPose: AUTONOMOUS 区間 (t_ns >= t0_ns) の末尾 kinematic_state pose。

    real.lite の終端と一致させる。bag 末尾ではなく AUTONOMOUS 窓の末尾を使うことで、
    AUTONOMOUS 解除後の post-roll が含まれていても goal が overshoot しない。
    """
    df_kin = load_kinematic(input_bag)
    if df_kin.empty:
        raise RuntimeError(f"kinematic_state が無いため GoalPose を取得不能: {input_bag}")
    df_auto = df_kin[df_kin["t_ns"] >= t0_ns]
    row = (df_auto if not df_auto.empty else df_kin).iloc[-1]
    return {"x": float(row.x), "y": float(row.y), "z": 0.0,
            "h": float(row.yaw), "p": 0.0, "r": 0.0}


def _course_duration_s(input_bag: Path, t0_ns: int) -> float:
    """実走 AUTONOMOUS 区間 (t_ns >= t0_ns) の継続時間 [s]。終了条件の時間ゲート算出に使う。"""
    df_kin = load_kinematic(input_bag)
    if df_kin.empty:
        return 0.0
    df_auto = df_kin[df_kin["t_ns"] >= t0_ns]
    df = df_auto if not df_auto.empty else df_kin
    return float((df["t_ns"].iloc[-1] - df["t_ns"].iloc[0]) / 1e9)


def _select_apex_indices(
    xs: list[float], ys: list[float], sx: float, sy: float, gx: float, gy: float, n: int
) -> list[int]:
    """start-goal 弦からの垂直距離が大きい点 (周回の膨らみ) を n 個、経路順に選ぶ純関数。

    経路 (xs, ys) を n 区間に等分し、各区間で弦からの垂直距離が最大の index を採る。
    n<=0 / 点数不足 / 弦長ゼロ なら []。shortest だけでは再現できない周回経路を強制するための
    中間 waypoint 候補 index を返す (build_scenario_dict が LanePosition 解決して挿入)。
    """
    m = len(xs)
    if n <= 0 or m < n + 2:
        return []
    cx, cy = gx - sx, gy - sy
    clen = math.hypot(cx, cy)
    if clen < 1e-6:
        return []
    perp = [abs(((xs[i] - sx) * cy - (ys[i] - sy) * cx) / clen) for i in range(m)]
    out: list[int] = []
    for k in range(n):
        lo = m * k // n
        hi = m * (k + 1) // n
        if hi <= lo:
            continue
        out.append(max(range(lo, hi), key=lambda i: perp[i]))
    return out


def _select_loop_waypoints(
    input_bag: Path, t0_ns: int, start_world: dict, goal_world: dict, n: int
) -> list[dict]:
    """実走 AUTONOMOUS 軌跡から周回の膨らみ位置を n 個選び、heading 付き world dict 群を返す。

    n<=0 なら []。各 waypoint は _world_to_lane_position 用に heading (軌跡 yaw) を持つ。
    """
    if n <= 0:
        return []
    df = load_kinematic(input_bag)
    if df.empty:
        return []
    df = df[df["t_ns"] >= t0_ns].reset_index(drop=True)
    if len(df) < n + 2:
        return []
    xs = df["x"].tolist()
    ys = df["y"].tolist()
    yaws = df["yaw"].tolist()
    z = float(start_world.get("z", 0.0))
    idxs = _select_apex_indices(
        xs, ys, start_world["x"], start_world["y"], goal_world["x"], goal_world["y"], n
    )
    return [{"x": xs[i], "y": ys[i], "z": z, "h": yaws[i], "p": 0.0, "r": 0.0} for i in idxs]


# ---------------------------------------------------------------------------
# Map parsing: WorldPosition → LanePosition 解決 (中央線ジオメトリ)
# ---------------------------------------------------------------------------


# WorldPosition→LanePosition 変換で使う、map 単位の全 lanelet 中央線キャッシュ。
_CENTERLINE_CACHE: dict[str, dict[int, list[tuple[float, float]]]] = {}


def _load_lanelet_centerlines(map_osm_path: Path) -> dict[int, list[tuple[float, float]]]:
    """各 lanelet の中央線ポリライン {lanelet_id: [(x, y), ...]} を返す (キャッシュ付き)。

    lanelet2 osm の relation type=lanelet について、left/right way の各 node 中点を
    並べた折れ線を中央線とする (left/right の点数が異なる場合は短い方に合わせる)。
    """
    key = str(map_osm_path)
    cached = _CENTERLINE_CACHE.get(key)
    if cached is not None:
        return cached
    root = ET.parse(key).getroot()
    nodes: dict[int, tuple[float, float]] = {}
    for n in root.iter("node"):
        nx = ny = None
        for tag in n.findall("tag"):
            k, v = tag.get("k"), tag.get("v")
            if k == "local_x":
                nx = float(v)
            elif k == "local_y":
                ny = float(v)
        if nx is not None and ny is not None:
            nodes[int(n.get("id"))] = (nx, ny)
    ways: dict[int, list[tuple[float, float]]] = {}
    for w in root.iter("way"):
        pts = [nodes[int(nd.get("ref"))] for nd in w.findall("nd")
               if int(nd.get("ref")) in nodes]
        if pts:
            ways[int(w.get("id"))] = pts
    centerlines: dict[int, list[tuple[float, float]]] = {}
    for rel in root.iter("relation"):
        if not any(t.get("k") == "type" and t.get("v") == "lanelet" for t in rel.findall("tag")):
            continue
        left = right = None
        for m in rel.findall("member"):
            if m.get("type") == "way":
                role = m.get("role")
                if role == "left":
                    left = ways.get(int(m.get("ref")))
                elif role == "right":
                    right = ways.get(int(m.get("ref")))
        if not left or not right:
            continue
        n = min(len(left), len(right))
        center = [((left[i][0] + right[i][0]) / 2, (left[i][1] + right[i][1]) / 2)
                  for i in range(n)]
        if len(center) >= 2:
            centerlines[int(rel.get("id"))] = center
    _CENTERLINE_CACHE[key] = centerlines
    return centerlines


def _world_to_lane_position(map_osm_path: Path, world: dict) -> dict | None:
    """WorldPosition dict (x, y, h) を OpenSCENARIO LanePosition dict に変換する。

    並走レーンが重複する curve では WorldPosition が一意 lane に解決できず
    scenario_simulator が InternalError を投げて sim 全体が落ちる (teleport 失敗) か、
    route が引けず ego が動かない。そこで heading が逆向き (差 > π/2) の lane を除外し、
    中央線への最近接距離で一意 lanelet を選び、弧長 s を算出して LanePosition を返す。
    候補が無ければ None (呼び出し側で WorldPosition にフォールバック)。
    """
    centerlines = _load_lanelet_centerlines(map_osm_path)
    x, y, h = world["x"], world["y"], world.get("h", 0.0)
    best: tuple[float, int, float] | None = None  # (dist, lanelet_id, s)
    for lid, center in centerlines.items():
        s_acc = 0.0
        nearest: tuple[float, float, float] | None = None  # (dist, heading, s)
        for i in range(len(center) - 1):
            ax, ay = center[i]
            bx, by = center[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            if seg2 == 0:
                continue
            seg = math.sqrt(seg2)
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            px, py = ax + t * dx, ay + t * dy
            dist = math.hypot(x - px, y - py)
            if nearest is None or dist < nearest[0]:
                nearest = (dist, math.atan2(dy, dx), s_acc + t * seg)
            s_acc += seg
        if nearest is None:
            continue
        dist, heading, s = nearest
        # heading が逆向き (差 > π/2) の lane は対向 lane への誤マッチなので除外
        if abs(((heading - h) + math.pi) % (2 * math.pi) - math.pi) > math.pi / 2:
            continue
        if best is None or dist < best[0]:
            best = (dist, lid, s)
    if best is None:
        return None
    _, lid, s = best
    return {
        "roadId": "",
        "laneId": str(lid),
        "s": round(s, 2),
        "Orientation": {"type": "relative", "h": 0, "p": 0, "r": 0},
    }


def _signal_to_lanelet_map(map_osm_path: Path, target_sig_ids: set[int]) -> dict[int, int]:
    """各信号 regulatory_element ID について、それを参照する最初の lanelet ID を返す.

    lanelet2 osm 形式の各 relation を見て:
    - type=lanelet なら、その lanelet が member として参照している relation のうち
      `role="regulatory_element"` のものを探す
    - その relation ref が target_sig_ids に含まれていれば map に登録

    結果: {sig_id: lanelet_id} (sig_id ごとに 1 つの lanelet を採用、最初に見つけたもの)
    """
    if not target_sig_ids:
        return {}
    tree = ET.parse(str(map_osm_path))
    root = tree.getroot()

    result: dict[int, int] = {}
    for rel in root.findall("relation"):
        # この relation が lanelet 型か確認
        rel_type = None
        for tag in rel.findall("tag"):
            if tag.get("k") == "type":
                rel_type = tag.get("v")
                break
        if rel_type != "lanelet":
            continue
        lanelet_id = int(rel.get("id"))
        # member のうち regulatory_element role を持つものを探す
        for member in rel.findall("member"):
            if (member.get("type") == "relation"
                    and member.get("role") == "regulatory_element"):
                ref = int(member.get("ref"))
                if ref in target_sig_ids and ref not in result:
                    result[ref] = lanelet_id
    return result


# ---------------------------------------------------------------------------
# Traffic signal extraction
# ---------------------------------------------------------------------------


# Autoware の signal color / shape / status enum → OpenSCENARIO state 文字列
# autoware_perception_msgs/msg/TrafficLightElement の定数と一致 (UNKNOWN=0)
_COLOR_NAMES = {1: "red", 2: "amber", 3: "green", 4: "white"}
_SHAPE_NAMES = {1: "circle", 2: "leftArrow", 3: "rightArrow", 4: "upArrow",
                5: "downArrow", 6: "downLeftArrow", 7: "downRightArrow",
                8: "upLeftArrow", 9: "upRightArrow", 10: "cross"}
_STATUS_NAMES = {1: "solidOff", 2: "solidOn", 3: "flashing"}


def _signal_state_str(element) -> str | None:
    """1 element (color/shape/status) → OpenSCENARIO state 文字列。
    color=UNKNOWN(0) のときは前状態を保持するため None を返す。
    """
    color_int = int(getattr(element, "color", 0))
    if color_int == 0:  # UNKNOWN
        return None
    color = _COLOR_NAMES.get(color_int, "green")
    status = _STATUS_NAMES.get(int(getattr(element, "status", 0)), "solidOn")
    shape = _SHAPE_NAMES.get(int(getattr(element, "shape", 0)), "circle")
    return f"{color} {status} {shape}"


def _msg_groups(msg) -> list:
    """TrafficLightGroupArray (traffic_light_groups) と TrafficSignalArray (signals) 両対応."""
    return (getattr(msg, "traffic_light_groups", None)
            or getattr(msg, "signals", None) or [])


def _group_id(item) -> int:
    """新 (traffic_light_group_id) / 旧 (traffic_signal_id) 両対応."""
    return int(getattr(item, "traffic_light_group_id", None)
               or getattr(item, "traffic_signal_id", None)
               or getattr(item, "map_primitive_id", 0))


def _extract_signal_timeseries(input_bag: Path, t0_ns: int) -> dict[int, list[tuple[float, str]]]:
    """各 traffic_light_group_id について、状態変化のタイムシリーズ
    `[(t_rel_sec, state_str), ...]` を返す。連続同状態は最初のみ採用、UNKNOWN はスキップ。
    """
    topics_in_bag = list_bag_topics(input_bag)
    signal_topic = next((t for t in _SIGNAL_TOPICS if t in topics_in_bag), None)
    if signal_topic is None:
        return {}

    by_id: dict[int, list[tuple[float, str]]] = {}
    for t_ns, msg in iter_bag_messages(input_bag, [signal_topic]):
        t_rel = (t_ns - t0_ns) / 1e9
        if t_rel < 0:
            continue
        for item in _msg_groups(msg):
            sig_id = _group_id(item)
            elements = getattr(item, "elements", None) or []
            if not elements:
                continue
            state = _signal_state_str(elements[0])
            if state is None:  # UNKNOWN は前状態保持
                continue
            prev = by_id.get(sig_id)
            if not prev or prev[-1][1] != state:
                by_id.setdefault(sig_id, []).append((t_rel, state))
    return by_id


# ---------------------------------------------------------------------------
# OpenSCENARIO yaml builder
# ---------------------------------------------------------------------------


def _build_initial_signal_actions(
    signals: dict[int, list[tuple[float, str]]],
    *, force_all_green: bool = False,
) -> list[dict]:
    """Init.GlobalAction として信号状態をセット.

    force_all_green=False (replay): bag の AUTONOMOUS 開始時の信号状態をそのまま再現。
    force_all_green=True (green): 全信号を常時 green にする。`traffic_signals: green`
        (Conditions / --traffic-signals green) で有効化される **サポート対象オプション**。
        理由: 実機信号タイムシリーズの replay は sim ego の到達時刻が実機とずれる
        (initialize_duration + planner pacing 差) ため、実機が green で通過した信号に
        sim ego が赤で当たり永久停止することがある (D0 の真因)。実機が実際に通過した
        ことを再現するには全 green が忠実 (実機が赤停止した信号は別途要検討)。
    """
    actions = []
    for sig_id, series in signals.items():
        if not series:
            continue
        state = "green solidOn circle" if force_all_green else series[0][1]
        actions.append({
            "InfrastructureAction": {
                "TrafficSignalAction": {
                    "TrafficSignalStateAction": {"name": str(sig_id), "state": state}
                }
            }
        })
    return actions


def _build_signal_story_acts(
    signals: dict[int, list[tuple[float, str]]],
    signal_to_lanelet: dict[int, int],
    *, reach_tolerance: float = 30.0, force_all_green: bool = False,
) -> list[dict]:
    """各信号について「ego が関連 lanelet に到達したら bag 終端状態に切替」する Act 列.

    scenario_test_runner サンプルの典型パターン (ReachPositionCondition trigger) と
    同じ構造。bag のタイムシリーズ各時刻ではなく、「ego が信号を見える距離に来た
    時点で実機が最終的に観測した状態」に切り替える。これで sim の simulationTime と
    実機 AUTONOMOUS 開始時刻の zero point ずれを回避する。

    Args:
        signals: 信号 ID → [(t_rel, state)] の dict (bag から抽出)
        signal_to_lanelet: 信号 ID → ego が到達したら trigger される lanelet ID の dict
        reach_tolerance: ReachPositionCondition の tolerance [m]
        force_all_green: `traffic_signals: green` モード (True なら Story 空、Init で全 green 固定)
    """
    if force_all_green:
        return []
    acts: list[dict] = []
    for sig_id, series in signals.items():
        if not series:
            continue
        # bag の最終観測状態 (= 実機がその信号を最後に見た時の状態) に切替
        # 多くの場合 red → green の遷移を再現する
        final_state = series[-1][1]
        lanelet_id = signal_to_lanelet.get(sig_id)
        if lanelet_id is None:
            # 関連 lanelet が map から見つからなければ scenario に書けない (trigger なし)
            continue
        event_name = f"switch_signal_{sig_id}_to_final"
        acts.append({
            "name": f"act_signal_{sig_id}",
            "ManeuverGroup": [{
                "maximumExecutionCount": 1,
                "name": f"act_signal_{sig_id}",
                "Actors": {
                    "selectTriggeringEntities": False,
                    "EntityRef": [{"entityRef": "ego"}],
                },
                "Maneuver": [{
                    "name": "",
                    "Event": [{
                        "name": event_name,
                        "priority": "parallel",
                        "Action": [{
                            "name": "",
                            "GlobalAction": {
                                "InfrastructureAction": {
                                    "TrafficSignalAction": {
                                        "TrafficSignalStateAction": {
                                            "name": str(sig_id),
                                            "state": final_state,
                                        }
                                    }
                                }
                            },
                        }],
                        "StartTrigger": {
                            "ConditionGroup": [{
                                "Condition": [{
                                    "name": f"ego_reached_lanelet_{lanelet_id}",
                                    "delay": 0,
                                    "conditionEdge": "none",
                                    "ByEntityCondition": {
                                        "TriggeringEntities": {
                                            "triggeringEntitiesRule": "any",
                                            "EntityRef": [{"entityRef": "ego"}],
                                        },
                                        "EntityCondition": {
                                            "ReachPositionCondition": {
                                                "Position": {
                                                    "LanePosition": {
                                                        "roadId": "",
                                                        "laneId": str(lanelet_id),
                                                        "s": 0,
                                                        "Orientation": {
                                                            "type": "relative",
                                                            "h": 0, "p": 0, "r": 0,
                                                        },
                                                    }
                                                },
                                                "tolerance": float(reach_tolerance),
                                            }
                                        },
                                    },
                                }]
                            }]
                        },
                    }],
                }],
            }],
            "StartTrigger": {
                "ConditionGroup": [{
                    "Condition": [{
                        "name": "",
                        "delay": 0,
                        "conditionEdge": "none",
                        "ByValueCondition": {
                            "SimulationTimeCondition": {"value": 0, "rule": "greaterThan"}
                        },
                    }]
                }]
            },
        })
    return acts


def _ego_entity_condition(entity_cond: dict, name: str = "") -> dict:
    """ego を対象とした ByEntityCondition の Condition dict を作る。"""
    return {
        "name": name,
        "delay": 0,
        "conditionEdge": "none",
        "ByEntityCondition": {
            "TriggeringEntities": {
                "triggeringEntitiesRule": "any",
                "EntityRef": [{"entityRef": "ego"}],
            },
            "EntityCondition": entity_cond,
        },
    }


def _reach_goal_condition(success_pos: dict, tolerance: float, name: str = "") -> dict:
    """ego がゴール位置に tolerance 以内へ到達する ReachPositionCondition。"""
    return _ego_entity_condition(
        {"ReachPositionCondition": {"Position": success_pos, "tolerance": float(tolerance)}},
        name,
    )


def _standstill_condition(duration: float, name: str = "") -> dict:
    """ego が duration 秒以上静止する StandStillCondition。"""
    return _ego_entity_condition({"StandStillCondition": {"duration": float(duration)}}, name)


def _sim_time_condition(value: float, name: str = "", rule: str = "greaterThan") -> dict:
    """シミュレーション時刻の ByValueCondition。"""
    return {
        "name": name,
        "delay": 0,
        "conditionEdge": "none",
        "ByValueCondition": {
            "SimulationTimeCondition": {"value": float(value), "rule": rule}
        },
    }


def _exit_event(name: str, condition_groups: list[dict], exit_type: str) -> dict:
    """StartTrigger を満たすと CustomCommandAction(exit_type) を発火する Event。

    condition_groups は ConditionGroup のリスト。1 グループ内に複数 Condition を入れると
    AND、グループを分けると OR になる (OpenSCENARIO の論理)。
    """
    return {
        "name": name,
        "priority": "parallel",
        "StartTrigger": {"ConditionGroup": condition_groups},
        "Action": [{
            "name": "",
            "UserDefinedAction": {"CustomCommandAction": {"type": exit_type}},
        }],
    }


def _build_end_condition_act(
    success_pos: dict,
    tolerance: float,
    sim_timeout: float,
    *,
    vicinity_tolerance: float,
    settle_gate: float,
    standstill_timeout: float,
) -> dict:
    """シナリオ終了判定の Act。

    「終了条件の厳密さより、同じコースで確実に切り上げること」を優先した設計:

    - ego_reached_goal      : ゴールに tolerance 以内で到達 (厳密・即時)。exitSuccess。
    - ego_reached_goal_vicinity: 一定時間経過後にゴール近傍 (vicinity_tolerance、寛容) へ
      到達。停止せず通過/周回するケースも、ゴール付近で止まるケースも捕捉する中核条件。
      ループ経路 (start≈goal) で開始直後に誤発火しないよう SimulationTime > settle_gate で
      ゲートする。exitSuccess。
    - ego_settled           : settle_gate 後に standstill_timeout 秒以上静止 (近傍外で停止/
      ハングした場合の保険)。**既定 standstill_timeout=0 で無効** (opt-in)。ゴール付近停止/通過
      周回は vicinity 条件で捕捉済みのため通常不要。reproduce_perception の先行車 dwell を
      途中で切り上げてしまうため有効化しないこと。exitSuccess。
    - sim_timeout           : 上記いずれも起きないまま sim_timeout 経過。exitFailure。

    success_pos は teleport/routing と同じ _world_to_lane_position で解決した
    {"LanePosition": ...} を受け取る。**LanePosition にするのは ReachPositionCondition が
    3D の hypot(x,y,z) で距離判定し、WorldPosition の z はシナリオ記載値を literal 使用する
    ため** (simulator_core.hpp)。map が標高 ~40m を持つ dataset では goal の z=0 と ego の
    map-pose z(~40) の差が常に tolerance を超え、ReachPosition が永久に発火しなかった
    (2026-06-03 判明)。LanePosition なら target/entity とも z を map から得るので dz≈0 で発火する。
    """
    events: list[dict] = [
        # 厳密到達 (ゲート無し・即時)
        _exit_event(
            "ego_reached_goal",
            [{"Condition": [_reach_goal_condition(success_pos, tolerance)]}],
            "exitSuccess",
        ),
        # 中核: 一定時間経過 AND ゴール近傍 (寛容)。停止/通過/周回いずれも切り上げる。
        _exit_event(
            "ego_reached_goal_vicinity",
            [{"Condition": [
                _sim_time_condition(settle_gate),
                _reach_goal_condition(success_pos, vicinity_tolerance),
            ]}],
            "exitSuccess",
        ),
    ]
    if standstill_timeout > 0:
        # 保険: 一定時間経過 AND 長時間静止 (近傍外で停止/ハング時に切り上げる)。
        events.append(_exit_event(
            "ego_settled",
            [{"Condition": [
                _sim_time_condition(settle_gate),
                _standstill_condition(standstill_timeout),
            ]}],
            "exitSuccess",
        ))
    # タイムアウト (どの終了条件にも掛からなかった = 本当に終わらなかった)。exitFailure。
    events.append(_exit_event(
        "sim_timeout",
        [{"Condition": [_sim_time_condition(sim_timeout)]}],
        "exitFailure",
    ))

    return {
        "name": "_EndCondition",
        "ManeuverGroup": [{
            "maximumExecutionCount": 1,
            "name": "",
            "Actors": {
                "selectTriggeringEntities": False,
                "EntityRef": [{"entityRef": "ego"}],
            },
            "Maneuver": [{"name": "", "Event": events}],
        }],
        "StartTrigger": {
            "ConditionGroup": [{
                "Condition": [_sim_time_condition(0)]
            }]
        },
    }


def build_scenario_dict(
    map_osm_abspath: Path,
    start_world: dict,
    goal_world: dict,
    lane_ids: list[int],
    signals: dict[int, list[tuple[float, str]]],
    signal_to_lanelet: dict[int, int],
    *,
    scenario_name: str = "auto_generated",
    sim_timeout: float = 600.0,
    goal_tolerance: float = 5.0,
    goal_vicinity_tolerance: float = 30.0,
    settle_gate: float = 30.0,
    standstill_timeout: float = 0.0,
    signal_reach_tolerance: float = 30.0,
    force_all_green: bool = False,
    mid_waypoints: list[dict] | None = None,
) -> dict[str, Any]:
    """OpenSCENARIO 文書を Python dict として組み立てる。

    force_all_green=False (replay): Init で bag の初期状態、Story で「ego が信号関連
    lanelet に到達したら bag 終端状態に切替」する (ReachPositionCondition trigger)。
    force_all_green=True (green): 全信号 green 固定。`traffic_signals: green` で有効化する
    サポート対象オプション (replay の到達時刻 desync で ego が赤に当たり停止する D0 を回避)。
    """
    init_global_actions = _build_initial_signal_actions(signals, force_all_green=force_all_green)
    init_global = {"GlobalAction": init_global_actions} if init_global_actions else {}

    # TeleportAction / RoutingAction の位置は LanePosition で指定する。
    # WorldPosition は並走レーンが重複する curve で一意 lane に解決できず、
    # scenario_simulator が InternalError を投げて sim 全体が落ちる
    # (ローカル実機 map の teleport で再現、 2026-06-01)。_world_to_lane_position が
    # heading + 中央線最近接距離で一意 lanelet を解決する。解決不能時のみ
    # WorldPosition にフォールバック。
    def _position(world: dict) -> dict:
        lane = _world_to_lane_position(map_osm_abspath, world)
        if lane:
            return {"LanePosition": lane}
        return {"WorldPosition": {
            "x": world["x"], "y": world["y"], "z": world["z"],
            "h": world["h"], "p": 0, "r": 0,
        }}

    private_actions: list[dict] = [
        {"TeleportAction": {"Position": _position(start_world)}}
    ]
    # RoutingAction: start + goal の 2 点のみを渡し (OpenSCENARIO schema は
    # Route.Waypoint minOccurs=2)、 ego_entity の Pose ベース routing (else 分岐) で
    # `SetRoutePoints` service を呼んで Autoware mission_planner に経路計算を任せる。
    #
    # 既定 (mid_waypoints 空) は start + goal の 2 点のみ。 kinematic 軌跡を密に間引いた中間点
    # (23 点) を挟むと mission_planner の連続 checkpoint ペアワイズ routing が非連結 lanelet 区間で
    # 「Failed to find a proper route」となり ego 静止した (2026-06-01 判別)。また segments の
    # preferred_primitive.id 全 dump も 7 箇所の急ヘディングで DiffusionPlanner NN が走行不能と
    # 判断し静止した (2026-05-29)。これらを避けるため既定は start+goal のみ (ego 241m 走行で停止)。
    #
    # ただし start+goal の shortest だけでは実機の「周回」経路 (弦より大きく膨らむ) を再現できず
    # sim が最短直行で早期停止する (D0)。これを緩和する opt-in として、--loop-waypoints N で
    # 実走軌跡の膨らみ位置に少数 (N) の中間 LanePosition waypoint を挿入し周回方向を強制できる。
    # start/goal と同じ _world_to_lane_position (heading フィルタ+中央線最近接) で解決し連結性を担保。
    # **要 live sim 検証** (上記の過去失敗があるため既定 N=0 で無効、有効化は sim 環境で確認のこと)。
    mids = mid_waypoints or []
    waypoints = [{"Position": _position(start_world), "routeStrategy": "shortest"}]
    waypoints += [{"Position": _position(mw), "routeStrategy": "shortest"} for mw in mids]
    waypoints.append({"Position": _position(goal_world), "routeStrategy": "shortest"})
    private_actions.append({
        "RoutingAction": {
            "AssignRouteAction": {
                "Route": {
                    "name": "",
                    "closed": False,
                    "Waypoint": waypoints,
                }
            }
        }
    })

    init = {
        "Actions": {
            **init_global,
            "Private": [{"entityRef": "ego", "PrivateAction": private_actions}],
        }
    }

    story_acts: list[dict] = _build_signal_story_acts(
        signals, signal_to_lanelet,
        reach_tolerance=signal_reach_tolerance, force_all_green=force_all_green,
    )
    # 終了判定のゴールは teleport/routing と同じ _position (LanePosition 優先) を使う。
    # ReachPositionCondition は 3D 距離判定で WorldPosition の z を literal 使用するため、
    # z=0 の WorldPosition では map 標高との差で永久に発火しない (上記 _build_end_condition_act)。
    story_acts.append(_build_end_condition_act(
        _position(goal_world), goal_tolerance, sim_timeout,
        vicinity_tolerance=goal_vicinity_tolerance,
        settle_gate=settle_gate,
        standstill_timeout=standstill_timeout,
    ))

    return {
        "ScenarioModifiers": {"ScenarioModifier": []},
        "OpenSCENARIO": {
            "FileHeader": {
                "revMajor": 1,
                "revMinor": 1,
                "date": "2026-05-28T00:00:00.000Z",
                "description": scenario_name,
                "author": "step2_bag_to_scenario.py (auto-generated)",
            },
            "ParameterDeclarations": {
                "ParameterDeclaration": [{
                    # WorldPosition goal を 1 個渡して Autoware mission_planner に route 計算を
                    # 任せるため lane_ids ベース routing は無効化 (=ego_entity の Pose ベース分岐)。
                    "name": "RoutingAction__use_lane_ids_for_routing",
                    "parameterType": "boolean",
                    "value": False,
                }]
            },
            "CatalogLocations": {
                "VehicleCatalog": {
                    "Directory": {
                        "path": "$(ros2 pkg prefix --share openscenario_experimental_catalog)/vehicle"
                    }
                }
            },
            "RoadNetwork": {"LogicFile": {"filepath": str(map_osm_abspath)}},
            "Entities": {
                "ScenarioObject": [{
                    "name": "ego",
                    "CatalogReference": {
                        "catalogName": "sample_vehicle",
                        "entryName": "sample_vehicle",
                    },
                    "ObjectController": {
                        "Controller": {
                            "name": "",
                            "Properties": {
                                "Property": [{"name": "isEgo", "value": "true"}]
                            },
                        }
                    },
                }]
            },
            "Storyboard": {
                "Init": init,
                "Story": [{"name": "", "Act": story_acts}],
                "StopTrigger": {"ConditionGroup": []},
            },
        },
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage 2: 実機 bag → OpenSCENARIO yaml 自動生成")
    parser.add_argument("--input-bag", required=True,
                        help="実機 input_bag ディレクトリ (mcap or db3)")
    parser.add_argument("--map", required=True,
                        help="lanelet2_map.osm の絶対パス (RoadNetwork.LogicFile)")
    parser.add_argument("--output", required=True, help="出力 scenario yaml パス")
    parser.add_argument("--scenario-name",
                        default=os.environ.get("SCENARIO_NAME", "auto_generated"))
    parser.add_argument("--sim-timeout", type=float, default=600.0,
                        help="exitFailure の SimulationTimeCondition [s] (既定 600)")
    parser.add_argument("--goal-tolerance", type=float, default=15.0,
                        help="goal ReachPositionCondition の厳密到達 tolerance [m] (既定 15)")
    parser.add_argument("--goal-vicinity-tolerance", type=float, default=30.0,
                        help="ゴール近傍切り上げの寛容 tolerance [m] (既定 30)。一定時間経過後に"
                             "この範囲へ入れば停止/通過/周回いずれでも exitSuccess で切り上げる。")
    parser.add_argument("--standstill-timeout", type=float, default=0.0,
                        help="一定時間経過後に ego がこの秒数以上静止したら切り上げる保険 [s] "
                             "(既定 0=無効の opt-in)。ゴール付近停止/通過周回は vicinity 条件で"
                             "捕捉済みのため通常不要。reproduce_perception では先行車 dwell の"
                             "忠実再現を途中で切り上げてしまうため有効化しないこと。")
    parser.add_argument("--traffic-signals", choices=["replay", "green", "none"],
                        default=os.environ.get("TRAFFIC_SIGNALS", "replay") or "replay",
                        help="信号の扱い (env: TRAFFIC_SIGNALS)。replay (既定)=実機 bag の信号"
                             "タイムシリーズを再現。green=全信号常時 green。none=scenario 側で信号を"
                             "一切セットしない (perception_reproducer が信号を pose-sync 再生して所有する"
                             "場合に使用)。replay は sim ego の到達時刻 desync で実機が green 通過した"
                             "信号に赤で当たり ego が永久停止する (D0) ことがあり、green/none で回避できる。")
    parser.add_argument("--loop-waypoints", type=int,
                        default=int(os.environ.get("LOOP_WAYPOINTS", "0") or "0"),
                        help="opt-in (既定 0=start+goal のみ): 実走軌跡の膨らみ位置に N 個の中間 "
                             "LanePosition waypoint を挿入し周回経路を強制 (D0 緩和)。**要 live sim 検証**")
    parser.add_argument("--replay-preroll", type=float,
                        default=float(os.environ.get("REPLAY_PREROLL", "0") or "0"),
                        help="ego replay の開始アンカーを AUTONOMOUS 開始 (t0) より何秒前に取るか [s] "
                             "(env: REPLAY_PREROLL, 既定 0)。TeleportAction の start pose も同じ "
                             "アンカー時刻の kinematic から取り、replay 注入開始時の pose ジャンプを防ぐ。")
    args = parser.parse_args()

    input_bag = Path(args.input_bag)
    map_osm = Path(args.map).resolve()
    output_yaml = Path(args.output)
    output_yaml.parent.mkdir(parents=True, exist_ok=True)

    if not input_bag.exists():
        print(f"ERROR: input_bag が見つかりません: {input_bag}", file=sys.stderr)
        sys.exit(1)
    if not map_osm.exists():
        print(f"ERROR: lanelet2_map.osm が見つかりません: {map_osm}", file=sys.stderr)
        sys.exit(1)

    # AUTONOMOUS 開始時刻 t0_ns を確定 (信号タイムシリーズ基準 + InitialPose 取得)
    df_mode = load_operation_mode(input_bag)
    df_vel = load_velocity(input_bag)
    try:
        t0_ns = find_autonomous_start(df_mode, df_vel)
    except Exception:
        t0_ns = 0

    # route 抽出は lane_ids の informational ログにのみ使う (start/goal には使わない)。
    _, _, lane_ids = _extract_route_info(input_bag)

    # ego replay のアンカー時刻。preroll>0 なら t0 より前から replay を始める
    # (start pose も同じアンカーから取り、注入開始時の pose ジャンプを防ぐ)。
    preroll_s = max(0.0, args.replay_preroll)
    t_anchor_ns = t0_ns - int(preroll_s * 1e9)

    # InitialPose / GoalPose は実機が AUTONOMOUS 区間で実際にいた位置 (kinematic) から取る。
    # LaneletRoute.start_pose はミッション計画上の起点で、記録された AUTONOMOUS 区間
    # (make_lite が抽出する real.lite) の外を指すことがある (過去ミッションの stale な
    # start_pose 等)。それを InitialPose にすると sim と実機が全く別の場所を走るため使わない。
    start_world = _initial_pose_from_kinematic(input_bag, t_anchor_ns)
    start_source = (f"kinematic@AUTONOMOUS開始-{preroll_s:.1f}s" if preroll_s > 0.0
                    else "kinematic@AUTONOMOUS開始")
    goal_world = _goal_pose_from_kinematic(input_bag, t0_ns)
    goal_source = "kinematic@AUTONOMOUS終了"

    print(f"[step2_bag_to_scenario] start=({start_world['x']:.1f}, {start_world['y']:.1f}, "
          f"h={start_world['h']:.3f}) ({start_source})")
    print(f"[step2_bag_to_scenario] goal=({goal_world['x']:.1f}, {goal_world['y']:.1f}, "
          f"h={goal_world['h']:.3f}) ({goal_source})")

    # 終了条件の時間ゲート: 実走 course 継続時間の半分 (ただし launch 遅延を超える 30s 以上)。
    # ループ経路は start≈goal のため近傍/静止条件が開始直後に誤発火する。これを時間で防ぐ。
    course_duration = _course_duration_s(input_bag, t0_ns)
    settle_gate = max(30.0, 0.5 * course_duration)
    print(f"[step2_bag_to_scenario] course_duration={course_duration:.1f}s, "
          f"end-condition settle_gate={settle_gate:.1f}s, "
          f"goal_tolerance={args.goal_tolerance}, vicinity={args.goal_vicinity_tolerance}, "
          f"standstill_timeout={args.standstill_timeout}, sim_timeout={args.sim_timeout}")

    # RoutingAction は start + goal の 2 点のみを LanePosition で渡し、 mission_planner に
    # 経路計算を任せる (build_scenario_dict 参照)。 LaneletRoute.segments は map graph 上の
    # lane 連結 (実走行の lane sequence ではない) で不自然な chain を含むため sim では使わない。
    print(f"[step2_bag_to_scenario] route lane_ids extracted (informational)={len(lane_ids)}, "
          f"using start+goal LanePosition routing (lane_ids 破棄)")
    lane_ids = []  # goal 到達条件は WorldPosition tolerance ベース (build_scenario_dict)
    signals = _extract_signal_timeseries(input_bag, t0_ns)
    print(f"[step2_bag_to_scenario] signals: {len(signals)} ids, "
          f"total state changes={sum(len(v) for v in signals.values())} "
          f"(traffic_signals={args.traffic_signals})")
    if args.traffic_signals == "green":
        print("[step2_bag_to_scenario] traffic_signals=green: 全信号を常時 green に固定 "
              "(replay の到達時刻 desync で ego が赤停止する D0 を回避)")
    elif args.traffic_signals == "none":
        # scenario 側で信号を一切セットしない (perception_reproducer が pose-sync 再生して所有)。
        signals = {}
        print("[step2_bag_to_scenario] traffic_signals=none: scenario 側で信号を設定しない "
              "(perception_reproducer が信号を pose-sync 再生して所有する前提)")

    # opt-in: 周回経路を強制する中間 waypoint (D0 緩和; 既定 N=0 で無効・要 live sim 検証)
    mid_waypoints = _select_loop_waypoints(input_bag, t0_ns, start_world, goal_world, args.loop_waypoints)
    if mid_waypoints:
        pts = ", ".join(f"({w['x']:.1f},{w['y']:.1f})" for w in mid_waypoints)
        print(f"[step2_bag_to_scenario] loop_waypoints={len(mid_waypoints)} 挿入: {pts} "
              "(**要 live sim 検証**)")

    # 信号 ID → ego が到達したら trigger される lanelet ID (map.osm から逆引き)
    signal_to_lanelet = _signal_to_lanelet_map(map_osm, set(signals.keys()))
    unresolved = set(signals.keys()) - set(signal_to_lanelet.keys())
    print(f"[step2_bag_to_scenario] signal→lanelet: {len(signal_to_lanelet)}/{len(signals)} 解決, "
          f"未解決 (Story trigger 出ない) = {sorted(unresolved)}")

    # yaml 構築
    doc = build_scenario_dict(
        map_osm_abspath=map_osm,
        start_world=start_world,
        goal_world=goal_world,
        lane_ids=lane_ids,
        signals=signals,
        signal_to_lanelet=signal_to_lanelet,
        scenario_name=args.scenario_name,
        sim_timeout=args.sim_timeout,
        goal_tolerance=args.goal_tolerance,
        goal_vicinity_tolerance=args.goal_vicinity_tolerance,
        settle_gate=settle_gate,
        standstill_timeout=args.standstill_timeout,
        force_all_green=(args.traffic_signals == "green"),
        mid_waypoints=mid_waypoints,
    )

    with output_yaml.open("w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
    print(f"[step2_bag_to_scenario] Saved: {output_yaml}")

    # ego/perception replay 用サイドカー。step3 が読み、bag metadata 開始時刻との差から
    # scenario_test_runner の replay_start_time (= scenario time 0 に対応する bag 相対秒) を出す。
    replay_json = output_yaml.with_suffix(".replay.json")
    with replay_json.open("w", encoding="utf-8") as f:
        json.dump({"t0_ns": int(t0_ns), "t_anchor_ns": int(t_anchor_ns),
                   "preroll_s": preroll_s}, f, indent=2)
    print(f"[step2_bag_to_scenario] Saved: {replay_json} "
          f"(t0_ns={t0_ns}, t_anchor_ns={t_anchor_ns}, preroll={preroll_s}s)")


if __name__ == "__main__":
    main()
