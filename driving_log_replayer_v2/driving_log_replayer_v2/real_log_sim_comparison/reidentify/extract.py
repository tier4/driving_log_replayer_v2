#!/usr/bin/env python3
"""raw rosbag2 (DB3/MCAP) → dataset 毎 CSV キャッシュ抽出。

`<collection>/datasets/<id>/input_bag` を 1 回だけ逐次走査し、同定に必要な
7 topic を wide-sparse 形式の `reidentify_cache.csv` へ直接書き出す。
中間 rosbag は作らない。ROS 依存は本ファイル内だけに閉じる。
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Literal

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from .csv_schema import CACHE_NAME, CSV_COLUMNS, SIGNAL_COLUMNS, SIGNAL_TOPICS

SUPPORTED_BAG_SUFFIXES = frozenset({".db3", ".mcap"})
MIN_COMMON_DURATION_NS = 2_000_000_000
# mode/gear は低頻度の状態通知なので、空でないことだけを必須とする。
# 2 秒判定は、後段が連続補間する 5 系列の共通区間で行う。
FIT_SIGNAL_TAGS = ("velocity", "steering", "kinematic", "accel", "cmd")


class DatasetSkip(ValueError):
    """Dataset 固有の入力不備。他 dataset の抽出は続行できる。"""


@dataclass(frozen=True, slots=True)
class BagInput:
    uri: Path
    storage_id: str


@dataclass(frozen=True, slots=True)
class ExtractionResult:
    dataset_id: str
    status: Literal["cached", "extracted", "skipped", "failed"]
    path: Path | None = None
    reason: str = ""
    n_rows: int = 0
    common_duration_s: float = 0.0


def _resolve_bag_input(dataset_dir: Path) -> BagInput:
    """input_bag の rosbag2 URI と storage id を解決する。"""
    input_bag = dataset_dir / "input_bag"
    if not input_bag.is_dir():
        raise DatasetSkip("input_bag ディレクトリがありません")

    bag_files = sorted(
        path
        for path in input_bag.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_BAG_SUFFIXES
    )
    if not bag_files:
        raise DatasetSkip("input_bag 直下に .db3/.mcap がありません")

    # 標準 rosbag2 は metadata.yaml を使って split bag 全体を 1 回で読む。
    if (input_bag / "metadata.yaml").is_file():
        return BagInput(input_bag, "")
    if len(bag_files) > 1:
        raise DatasetSkip("split bag の metadata.yaml がありません")

    # 単一 DB3/MCAP だけ配置した最小入力も許容する。
    bag_file = bag_files[0]
    storage_id = "sqlite3" if bag_file.suffix.lower() == ".db3" else "mcap"
    return BagInput(bag_file, storage_id)


def _open_reader(bag: BagInput) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.uri), storage_id=bag.storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def _kinematic_row(timestamp: int, msg: object) -> dict[str, int | float | str]:
    pose = msg.pose.pose
    orientation = pose.orientation
    yaw = math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    sin_pitch = max(
        -1.0,
        min(1.0, 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)),
    )
    twist = msg.twist.twist
    return {
        "t_ns": timestamp,
        "topic": "kinematic",
        "x": pose.position.x,
        "y": pose.position.y,
        "yaw": yaw,
        "pitch": math.asin(sin_pitch),
        "vx": twist.linear.x,
        "vy": twist.linear.y,
        "wz": twist.angular.z,
    }


def _message_row(tag: str, timestamp: int, msg: object) -> dict[str, int | float | str]:
    if tag == "mode":
        return {"t_ns": timestamp, "topic": tag, "mode": int(msg.mode)}
    if tag == "velocity":
        return {
            "t_ns": timestamp,
            "topic": tag,
            "lon_vel": msg.longitudinal_velocity,
        }
    if tag == "steering":
        return {"t_ns": timestamp, "topic": tag, "steer": msg.steering_tire_angle}
    if tag == "gear":
        return {"t_ns": timestamp, "topic": tag, "gear": int(msg.report)}
    if tag == "kinematic":
        return _kinematic_row(timestamp, msg)
    if tag == "accel":
        return {"t_ns": timestamp, "topic": tag, "accel": msg.accel.accel.linear.x}
    if tag == "cmd":
        return {
            "t_ns": timestamp,
            "topic": tag,
            "cmd_vel": msg.longitudinal.velocity,
            "cmd_accel": msg.longitudinal.acceleration,
            "cmd_steer": msg.lateral.steering_tire_angle,
        }
    raise AssertionError(f"未定義の signal タグ: {tag}")


def _row_values_are_valid(tag: str, row: dict) -> bool:
    """タグ固有値が後段で安全に数値化できる有限値かを確認する。"""
    try:
        values = [float(row[column]) for column in SIGNAL_COLUMNS[tag]]
    except (KeyError, TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in values):
        return False
    if tag in {"mode", "gear"} and not all(value.is_integer() for value in values):
        return False
    return True


def _extract_to_csv(bag: BagInput, temp_csv: Path) -> tuple[int, float]:
    """Bag を 1 回逐次走査し、検証済み CSV の一時ファイルを作る。"""
    reader = _open_reader(bag)
    available_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = [topic for topic in SIGNAL_TOPICS.values() if topic not in available_types]
    if missing:
        raise DatasetSkip(f"必須 topic 欠落: {', '.join(missing)}")

    selected_topics = list(SIGNAL_TOPICS.values())
    message_types = {topic: get_message(available_types[topic]) for topic in selected_topics}
    topic_tags = {topic: tag for tag, topic in SIGNAL_TOPICS.items()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topics))

    counts = dict.fromkeys(SIGNAL_TOPICS, 0)
    first_t_ns: dict[str, int] = {}
    last_t_ns: dict[str, int] = {}
    n_rows = 0
    with temp_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, extrasaction="raise")
        writer.writeheader()
        while reader.has_next():
            topic_name, msg_bytes, timestamp = reader.read_next()
            tag = topic_tags.get(topic_name)
            if tag is None:
                continue
            msg = deserialize_message(msg_bytes, message_types[topic_name])
            row = _message_row(tag, int(timestamp), msg)
            if not _row_values_are_valid(tag, row):
                continue
            writer.writerow(row)
            n_rows += 1
            counts[tag] += 1
            timestamp = int(timestamp)
            first_t_ns[tag] = min(timestamp, first_t_ns.get(tag, timestamp))
            last_t_ns[tag] = max(timestamp, last_t_ns.get(tag, timestamp))

    empty_topics = [SIGNAL_TOPICS[tag] for tag, count in counts.items() if count == 0]
    if empty_topics:
        raise DatasetSkip(f"必須 topic が空: {', '.join(empty_topics)}")

    common_start_ns = max(first_t_ns[tag] for tag in FIT_SIGNAL_TAGS)
    common_end_ns = min(last_t_ns[tag] for tag in FIT_SIGNAL_TAGS)
    common_duration_ns = max(0, common_end_ns - common_start_ns)
    if common_duration_ns < MIN_COMMON_DURATION_NS:
        duration_s = common_duration_ns / 1e9
        raise DatasetSkip(f"同定可能な共通区間が2秒未満 ({duration_s:.3f}s)")
    return n_rows, common_duration_ns / 1e9


def _cache_is_valid(path: Path) -> bool:
    """旧形式・空・途中書き込みの CSV を再利用しない。"""
    try:
        with path.open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != CSV_COLUMNS:
                return False
            counts = dict.fromkeys(SIGNAL_TOPICS, 0)
            first_t_ns: dict[str, int] = {}
            last_t_ns: dict[str, int] = {}
            for row in reader:
                tag = row.get("topic", "")
                if tag not in SIGNAL_TOPICS:
                    return False
                if not _row_values_are_valid(tag, row):
                    return False
                timestamp = int(row["t_ns"])
                counts[tag] += 1
                first_t_ns[tag] = min(timestamp, first_t_ns.get(tag, timestamp))
                last_t_ns[tag] = max(timestamp, last_t_ns.get(tag, timestamp))
            if any(count == 0 for count in counts.values()):
                return False
            common_start_ns = max(first_t_ns[tag] for tag in FIT_SIGNAL_TAGS)
            common_end_ns = min(last_t_ns[tag] for tag in FIT_SIGNAL_TAGS)
            return common_end_ns - common_start_ns >= MIN_COMMON_DURATION_NS
    except (KeyError, OSError, UnicodeError, ValueError, csv.Error):
        return False


def _extract_dataset(dataset_dir: Path, *, force: bool) -> ExtractionResult:
    dataset_id = dataset_dir.name
    out_csv = dataset_dir / CACHE_NAME
    if out_csv.is_file() and not force:
        if _cache_is_valid(out_csv):
            return ExtractionResult(dataset_id, "cached", path=out_csv)
        print(f"[extract] {dataset_id}: invalid cache; rebuilding", file=sys.stderr)
        out_csv.unlink()

    temp_csv = out_csv.with_suffix(f"{out_csv.suffix}.tmp")
    try:
        bag = _resolve_bag_input(dataset_dir)
        temp_csv.unlink(missing_ok=True)
        n_rows, duration_s = _extract_to_csv(bag, temp_csv)
        temp_csv.replace(out_csv)
        return ExtractionResult(
            dataset_id,
            "extracted",
            path=out_csv,
            n_rows=n_rows,
            common_duration_s=duration_s,
        )
    except DatasetSkip as exc:
        if force:
            out_csv.unlink(missing_ok=True)
        return ExtractionResult(dataset_id, "skipped", reason=str(exc))
    except Exception as exc:  # noqa: BLE001 (backend/type errors are dataset-local)
        if force:
            out_csv.unlink(missing_ok=True)
        return ExtractionResult(
            dataset_id,
            "failed",
            reason=f"{type(exc).__name__}: {exc}",
        )
    finally:
        temp_csv.unlink(missing_ok=True)


def _print_result(result: ExtractionResult) -> None:
    if result.status == "cached":
        print(f"[extract] {result.dataset_id}: cached ({CACHE_NAME})")
    elif result.status == "extracted":
        print(
            f"[extract] {result.dataset_id}: extracted "
            f"({result.n_rows} rows, common={result.common_duration_s:.3f}s)"
        )
    else:
        print(
            f"[extract] {result.dataset_id}: {result.status} - {result.reason}",
            file=sys.stderr,
        )


def extract_collection(collection_dir: Path, *, force: bool = False) -> dict:
    """`datasets/<id>` を全て抽出し、dataset 単位の診断を返す。"""
    collection_dir = Path(collection_dir)
    datasets_dir = collection_dir / "datasets"
    dataset_dirs = (
        sorted(path for path in datasets_dir.iterdir() if path.is_dir())
        if datasets_dir.is_dir()
        else []
    )
    results = [_extract_dataset(path, force=force) for path in dataset_dirs]
    for result in results:
        _print_result(result)

    valid = [result for result in results if result.status in {"cached", "extracted"}]
    skipped = [result for result in results if result.status == "skipped"]
    failed = [result for result in results if result.status == "failed"]
    summary = {
        "n_datasets": len(results),
        "n_cached": sum(result.status == "cached" for result in results),
        "n_extracted": sum(result.status == "extracted" for result in results),
        "n_skipped": len(skipped),
        "n_failed": len(failed),
        "valid_datasets": [result.dataset_id for result in valid],
        "skipped": [
            {"dataset_id": result.dataset_id, "reason": result.reason} for result in skipped
        ],
        "failed": [
            {"dataset_id": result.dataset_id, "reason": result.reason} for result in failed
        ],
    }
    print(
        f"[extract] datasets={summary['n_datasets']} "
        f"cached={summary['n_cached']} extracted={summary['n_extracted']} "
        f"skipped={summary['n_skipped']} failed={summary['n_failed']}"
    )
    if not valid:
        raise RuntimeError(
            f"有効な dataset が0件です: {datasets_dir} "
            f"(skipped={len(skipped)}, failed={len(failed)})"
        )
    return summary
