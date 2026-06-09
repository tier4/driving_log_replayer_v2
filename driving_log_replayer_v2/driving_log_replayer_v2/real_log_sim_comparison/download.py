"""
rosbag と地図を webauto CLI で取得し、local_cloud_run 用のディレクトリ構造を作る。

Usage:
    python3 download.py --environment-id <env_id> --rosbag-id <rosbag_id>

ディレクトリ構成:
    <work_dir>/          <- webauto のダウンロード先（生データ置き場）
        rosbag/
        map/
    <work_dir>/dataset/<uuid>/   <- local_cloud_run に渡す最終構造
        input_bag/
        map/
"""

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import uuid as uuid_module
from pathlib import Path

PROJECT_ID = "x2_dev"


def run(cmd: list[str]) -> str:
    """コマンドを実行して stdout を返す。失敗時は終了。"""
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"[ERROR] {' '.join(cmd)}", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(1)
    return result.stdout

def describe_log_file(log_file_id: str) -> dict:
    out = run([
        "webauto", "data", "log-file", "describe",
        "--project-id", PROJECT_ID,
        "--log-file-id", log_file_id,
        "--output", "json"
    ])
    return json.loads(out)

def describe_rosbag(environment_id: str, rosbag_id: str) -> dict:
    """webauto data describe で rosbag のメタ情報を取得する。"""
    out = run([
        "webauto", "data", "rosbag", "describe",
        "--project-id", PROJECT_ID,
        "--environment-id", environment_id,
        "--rosbag-id", rosbag_id,
        "--output", "json",
    ])
    return json.loads(out)


def pull_rosbag(rosbag_id: list[str], topic: list[str], target_dir: Path, start: str, end: str) -> None:
    """rosbag を target_dir に落とす。"""
    command = [
        "webauto", "data", "log-file", "pull-filtered-rosbag",
        "--project-id", PROJECT_ID,
        "--log-file-ids", ','.join(rosbag_id),
        "--target-dir", str(target_dir),
        "--topics", ','.join(topic),
    ]
    if start is not None:
        command += ["--slicing-start-timestamp", start]
    if end is not None:
        command += ["--slicing-end-timestamp", end]
    run(command)


def pull_map(area_map_id: str, area_map_version_id: str, asset_dir: Path) -> None:
    """地図を asset_dir に落とす。"""
    command = [
        "webauto", "map", "area-map", "pull",
        "--project-id", PROJECT_ID,
        "--area-map-id", area_map_id,
        "--area-map-version-id", area_map_version_id,
        "--asset-dir", str(asset_dir),
    ]
    run(command)


def _collect_bag(rosbag_dir: Path, input_bag_dir: Path) -> None:
    """rosbag ディレクトリを input_bag/ にコピーする。"""
    for path in rosbag_dir.rglob("*"):
        if path.is_file():
            shutil.copy(path, input_bag_dir)


def _collect_map(work_dir: Path, area_map_id: str, area_map_version_id: str, map_dir: Path) -> None:
    """地図ディレクトリを map/ にコピーする。"""
    src = work_dir / "map" / PROJECT_ID / area_map_id / area_map_version_id
    shutil.copytree(src, map_dir, dirs_exist_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rosbag-id", nargs='+', required=True)
    parser.add_argument("--topic", nargs='+', required=True)
    parser.add_argument("--start")
    parser.add_argument("--end")
    parser.add_argument("--work-dir", default="work", help="作業ディレクトリ (default: work)")
    args = parser.parse_args()

    work_dir = Path(args.work_dir)

    # Step 1: メタ情報を取得して地図 ID を抜き出す
    print("[INFO] Fetching metadata...")
    environment_id = describe_log_file(args.rosbag_id[0])["environment_id"]
    meta = describe_rosbag(environment_id, args.rosbag_id[0])

    area_map_id = str(meta["area_map"]["area_map_id"])
    area_map_version_id = str(meta["area_map"]["area_map_version_id"])

    print(f"  rosbag_id          : {args.rosbag_id}")
    print(f"  area_map_id        : {area_map_id}")
    print(f"  area_map_version_id: {area_map_version_id}")

    # Step 2: rosbag を作業ディレクトリに落とす
    print("[INFO] Pulling rosbag...")
    rosbag_dir = Path(tempfile.mkdtemp(prefix="filtered_", dir=work_dir / "rosbag"))
    pull_rosbag(args.rosbag_id, args.topic, rosbag_dir, args.start, args.end)

    # Step 3: 地図を作業ディレクトリに落とす
    print("[INFO] Pulling map...")
    pull_map(area_map_id, area_map_version_id, work_dir)

    # Step 4: dataset ディレクトリに local_cloud_run 用の構造を構築する
    uuid = str(uuid_module.uuid4())
    dataset_path = work_dir / "dataset" / uuid
    input_bag_dir = dataset_path / "input_bag"
    map_dir = dataset_path / "map"
    input_bag_dir.mkdir(parents=True, exist_ok=True)
    map_dir.mkdir(parents=True, exist_ok=True)

    print("[INFO] Building dataset structure...")
    _collect_bag(rosbag_dir, input_bag_dir)
    _collect_map(work_dir, area_map_id, area_map_version_id, map_dir)

    dataset_root = (work_dir / "dataset").resolve()
    print(f"[INFO] Done.")
    print(f"[INFO] UUID: {uuid}")
    print(f"[INFO] Run:")
    print(f"  make local_cloud_run WEBAUTO_T4_ROOT={dataset_root}")
    print(f"  (scenario.yaml の UUID を '{uuid}' に設定してください)")


if __name__ == "__main__":
    main()
