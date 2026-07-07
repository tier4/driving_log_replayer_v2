#!/usr/bin/env python3
"""[Step1] real.lite (rosbag2/mcap) → dataset 毎 CSV キャッシュ抽出。

reidentify パイプラインで rclpy/rosbag2_py/rosidl_runtime_py に触れるのは
本ファイルだけ。以降の fit_lon/fit_steer/rollout/fit_merge/report は
本ファイルが書き出す CSV のみを読み、ROS に依存しない
(vehicle_model_fitting の extract_bags.py → real_data.csv と同じ役割分担)。

CSV は「1行 = 1メッセージ、そのトピックの列だけ埋まる」wide-sparse 形式で、
`lib/_io.py` の各 loader (SSOT) が返す列をそのまま保持する。読込側
(load_data.py) は topic 列でフィルタして元の DataFrame 群を復元する。
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import pandas as pd

from ..lib._collection import DatasetEntry, discover_collection
from ..lib._io import (
    load_accel,
    load_cmd,
    load_gear_status,
    load_kinematic,
    load_operation_mode,
    load_steering,
    load_velocity,
)
from .csv_schema import CACHE_NAME, CSV_COLUMNS

CMD_TOPIC = "/control/command/control_cmd"


def _load_signals(bag: Path) -> dict[str, pd.DataFrame]:
    return {
        "mode": load_operation_mode(bag),
        "velocity": load_velocity(bag),
        "steering": load_steering(bag),
        "kinematic": load_kinematic(bag),
        "accel": load_accel(bag),
        "cmd": load_cmd(bag, CMD_TOPIC),
        "gear": load_gear_status(bag),
    }


def _to_wide_sparse(signals: dict[str, pd.DataFrame]) -> pd.DataFrame:
    frames = []
    for topic, df in signals.items():
        if df.empty:
            continue
        sub = df.copy()
        sub["topic"] = topic
        frames.append(sub)
    if not frames:
        return pd.DataFrame(columns=CSV_COLUMNS)
    out = pd.concat(frames, ignore_index=True, sort=False)
    out = out.reindex(columns=CSV_COLUMNS)
    return out.sort_values("t_ns").reset_index(drop=True)


def extract_one(dataset_dir: Path, *, force: bool = False) -> Path | None:
    """1 データセットの real.lite を CSV キャッシュに変換する。既存なら (force なしで) スキップ。"""
    bag = None
    for cand in (dataset_dir / "real.lite.mcap", dataset_dir / "real.lite"):
        if cand.exists():
            bag = cand
            break
    if bag is None:
        return None

    out_csv = dataset_dir / CACHE_NAME
    if out_csv.exists() and not force:
        return out_csv

    signals = _load_signals(bag)
    df = _to_wide_sparse(signals)
    if df.empty:
        print(f"[WARN] {dataset_dir.name}: 有効なトピックが読めませんでした", file=sys.stderr)
        return None
    df.to_csv(out_csv, index=False)
    return out_csv


def extract_collection(collection_dir: Path, *, force: bool = False) -> dict:
    """collection 配下の全データセットを CSV キャッシュ化する。"""
    entries: list[DatasetEntry] = [
        e for e in discover_collection(collection_dir) if e.dir is not None
    ]
    n_cached, n_extracted, n_skipped = 0, 0, 0
    for e in entries:
        out_csv = e.dir / CACHE_NAME
        already_cached = out_csv.exists() and not force
        result = extract_one(e.dir, force=force)
        if result is None:
            n_skipped += 1
        elif already_cached:
            n_cached += 1
        else:
            n_extracted += 1
    summary = {
        "n_datasets": len(entries),
        "n_cached": n_cached,
        "n_extracted": n_extracted,
        "n_skipped": n_skipped,
    }
    print(
        f"[extract] datasets={summary['n_datasets']} "
        f"cached(既存)={summary['n_cached']} 新規抽出={summary['n_extracted']} "
        f"skip(real.lite無し等)={summary['n_skipped']}"
    )
    return summary


def main() -> None:
    ap = argparse.ArgumentParser(description="real.lite → CSV キャッシュ抽出")
    ap.add_argument("--collection-dir", type=Path, required=True)
    ap.add_argument("--force", action="store_true", help="既存の CSV キャッシュも再抽出する")
    args = ap.parse_args()
    extract_collection(args.collection_dir, force=args.force)


if __name__ == "__main__":
    main()
