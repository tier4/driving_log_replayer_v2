#!/usr/bin/env python3
import argparse
from datetime import datetime, timezone, timedelta
import os
import pathlib
import re
import sys
import yaml

def get_dataset_date(dataset_path: pathlib.Path) -> str | None:
    # 1. db3 または mcap ファイル名から日付を正規表現で抽出
    bag_dir = dataset_path / "input_bag"
    if not bag_dir.is_dir():
        return None
    
    # db3 または mcap ファイルを検索
    for entry in bag_dir.iterdir():
        if entry.is_file() and entry.suffix in (".db3", ".mcap"):
            # filtered_e14508f0-5cc8-410c-886b-23aea4a8d5d8_2026-06-23-11-33-34_2026-06-23-11-34-34_p0900_0.db3
            # YYYY-MM-DD を抽出
            match = re.search(r"(\d{4}-\d{2}-\d{2})", entry.name)
            if match:
                return match.group(1)

    # 2. metadata.yaml から starting_time を読み取る
    metadata_yaml = bag_dir / "metadata.yaml"
    if metadata_yaml.is_file():
        try:
            data = yaml.safe_load(metadata_yaml.read_text(encoding="utf-8"))
            info = data.get("rosbag2_bagfile_information", {})
            start_ns = info.get("starting_time", {}).get("nanoseconds_since_epoch")
            if start_ns:
                # JST (UTC+9) に変換
                dt = datetime.fromtimestamp(start_ns / 1e9, tz=timezone(timedelta(hours=9)))
                return dt.strftime("%Y-%m-%d")
        except Exception as e:
            print(f"[WARN] Failed to parse {metadata_yaml}: {e}", file=sys.stderr)

    return None

def main():
    parser = argparse.ArgumentParser(description="Filter datasets by date and create symlinks.")
    parser.add_argument("--src-dir", required=True, help="Source datasets directory")
    parser.add_argument("--dst-dir", required=True, help="Destination datasets directory")
    parser.add_argument("--date", default="2026-06-16", help="Threshold date (YYYY-MM-DD)")
    args = parser.parse_args()

    src_path = pathlib.Path(args.src_dir).resolve()
    dst_path = pathlib.Path(args.dst_dir).resolve()

    if not src_path.is_dir():
        print(f"[ERROR] Source directory does not exist: {src_path}", file=sys.stderr)
        sys.exit(1)

    dst_path.mkdir(parents=True, exist_ok=True)

    print(f"Scanning datasets in {src_path} ...")
    print(f"Target date: on or after {args.date}")

    count = 0
    created = 0
    # src_path 直下にあるサブディレクトリ（UUID名）を走査
    for item in sorted(src_path.iterdir()):
        if not item.is_dir():
            continue
        
        # シンボリックリンク切れの場合は実体が存在しないため除外
        try:
            real_item = item.resolve(strict=True)
        except FileNotFoundError:
            continue
            
        count += 1
        date_str = get_dataset_date(real_item)
        if not date_str:
            continue
            
        if date_str >= args.date:
            # シンボリックリンクの作成
            link_name = dst_path / item.name
            if link_name.exists() or link_name.is_symlink():
                link_name.unlink()
            link_name.symlink_to(real_item)
            created += 1
            print(f"[LINK] {item.name} ({date_str}) -> {real_item}")

    print(f"Processed {count} directories. Created {created} symlinks in {dst_path}")

if __name__ == "__main__":
    main()
