#!/usr/bin/env python3
"""vehicle_id で生rosbagを収集し open-loop 同定用の real.lite を生成する.

webauto data log-file search で対象車両の全ロスバッグを検索し、選別してから
pull-filtered-rosbag でトピックを絞って取得する。その後 step1_make_lite で
real.lite に変換し、multi_dataset_tune の軽量ループで使えるようにする。

map は open-loop 解析に不要なので取得しない。

ディレクトリ構成:
    <root>/
    ├── datasets/
    │   └── <synthetic_id>/
    │       ├── input_bag/       # pull-filtered-rosbag の出力
    │       └── real.lite        # step1_make_lite の出力 (multi_dataset_tune 入力)
    └── raw_index.yaml           # 収集記録 (rosbag_id ↔ synthetic_id 対応)

synthetic_id は rosbag_id から uuid.uuid5 で決定的に生成する (再実行で安定)。

使い方 (make collect_raw_rosbags 経由を推奨):
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.collect_raw_rosbags \\
        --vehicle-id e14508f0-5cc8-410c-886b-23aea4a8d5d8 \\
        --root /path/to/collection_root \\
        --date-from 2026-01-01 \\
        --date-to 2026-06-01 \\
        --limit 20

次のステップ (軽量ループ):
    make open_loop_tune ROOT=<root>        # multi_dataset_tune で横断ロバスト同定
    make open_loop_batch ROOT=<root>       # (任意) curated subset で full launch + 集約レポート
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid as uuid_module

import yaml

PROJECT_ID = "x2_dev"
_PKG = "driving_log_replayer_v2.real_log_sim_comparison"

# open-loop モデル同定に必要なトピックセット。
# download.py の _DEFAULT_TOPICS (実走 pull で実績あり) をベースとし、
# open-loop 追加候補 /control/trajectory_follower/control_cmd を付加する。
#
# ⚠️ pull-filtered-rosbag が不在 topic で失敗するかは Phase 0 で要確認。
# 不確かなトピック (gear_status / actuation_status / sensing/imu 等) は
# Phase 0 実測後に確認が取れてから追加すること。
_OPENLOOP_TOPICS: list[str] = [
    # -- download.py _DEFAULT_TOPICS と同一 (pull 実績あり) --
    "/system/operation_mode/state",
    "/vehicle/status/velocity_status",
    "/vehicle/status/steering_status",
    "/localization/kinematic_state",
    "/localization/acceleration",
    "/control/command/control_cmd",
    "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
    "/perception/object_recognition/tracking/objects",
    "/perception/object_recognition/detection/objects",
    "/perception/traffic_light_recognition/traffic_signals",
    "/planning/trajectory",
    # -- open-loop 追加候補 (Phase 0 で bag 内存在を確認後に拡張) --
    "/control/trajectory_follower/control_cmd",
]

_MIN_DURATION_S_DEFAULT = 30.0  # この秒数未満の bag はスキップ (AUTONOMOUS 区間なし扱い)
_PAGE_SIZE = 50                  # log-file search の 1 ページあたり件数


def _run(cmd: list[str]) -> str:
    """コマンドを実行して stdout を返す。失敗時は RuntimeError。"""
    result = subprocess.run(cmd, capture_output=True, text=True)  # noqa: S603
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed: {' '.join(cmd)}\nstderr: {result.stderr.strip()}"
        )
    return result.stdout


def _search_log_files(
    vehicle_id: str,
    *,
    date_from: str | None,
    date_to: str | None,
    page_token: str | None,
) -> tuple[list[dict], str | None]:
    """webauto data log-file search で 1 ページ分の rosbag 一覧を取得する。

    返り値: (items, next_page_token or None)
    """
    cmd = [
        "webauto", "data", "log-file", "search",
        "--project-id", PROJECT_ID,
        "--vehicle-ids", vehicle_id,
        "--file-types", "rosbag",
        "--page-size", str(_PAGE_SIZE),
        "--output", "json",
    ]
    if date_from:
        cmd += ["--datetime-from", f"{date_from}T00:00:00Z"]
    if date_to:
        cmd += ["--datetime-to", f"{date_to}T23:59:59Z"]
    if page_token:
        cmd += ["--page-token", page_token]

    raw = _run(cmd)

    # レスポンス形式を吸収 (list / dict with "logFiles"/"items" etc.)
    data = json.loads(raw)
    if isinstance(data, list):
        items: list[dict] = data
        next_token: str | None = None
    else:
        items = (
            data.get("logFiles")
            or data.get("log_files")
            or data.get("items")
            or data.get("results")
            or []
        )
        next_token = data.get("nextPageToken") or data.get("next_page_token") or None

    return items, next_token


def _parse_timestamp(ts: str | None) -> float | None:
    """ISO8601 タイムスタンプを float epoch に変換する (不明は None)。"""
    if not ts:
        return None
    try:
        s = ts.replace("Z", "+00:00")
        return datetime.fromisoformat(s).timestamp()
    except (ValueError, TypeError):
        return None


def _get_field(item: dict, *keys: str) -> str | None:
    """camelCase / snake_case どちらでも取れる多候補フォールバック。"""
    for k in keys:
        v = item.get(k)
        if v is not None:
            return v
    return None


def _get_duration(item: dict) -> float | None:
    """ログファイルの recording 時間 [s] を返す。不明なら None。

    log-file search の実フィールド名は start_timestamp / end_timestamp (snake_case)。
    rosbag search との互換のため camelCase 候補も残す。
    """
    start = _parse_timestamp(
        _get_field(item, "start_timestamp", "startTimestamp",
                   "recordingStartTime", "recording_start_time",
                   "recordingStart", "recording_start")
    )
    end = _parse_timestamp(
        _get_field(item, "end_timestamp", "endTimestamp",
                   "recordingEndTime", "recording_end_time",
                   "recordingEnd", "recording_end")
    )
    if start is None or end is None:
        return None
    return max(0.0, end - start)


def _build_synthetic_id(rosbag_id: str) -> str:
    """rosbag_id から決定的に synthetic dataset UUID を生成する (R5: 安定性)。

    同一 rosbag_id は常に同一 synthetic_id に写像されるため、
    再実行しても datasets/ ディレクトリが重複しない。
    """
    return str(uuid_module.uuid5(uuid_module.NAMESPACE_URL, f"raw_rosbag:{rosbag_id}"))


def _pull_rosbag(rosbag_id: str, topics: list[str], pull_dir: Path) -> None:
    """pull-filtered-rosbag で rosbag を pull_dir に取得する (生出力ディレクトリ)。

    pull-filtered-rosbag はネストした rosbag2 ディレクトリを pull_dir 内に作成する。
    呼び出し元は _collect_bag でファイルを input_bag_dir に flatten すること
    (download.py の _collect_bag + tempdir パターンに準拠)。
    """
    cmd = [
        "webauto", "data", "log-file", "pull-filtered-rosbag",
        "--project-id", PROJECT_ID,
        "--log-file-ids", rosbag_id,
        "--target-dir", str(pull_dir),
        "--topics", ",".join(topics),
    ]
    _run(cmd)


def _collect_bag(pull_dir: Path, input_bag_dir: Path) -> None:
    """pull_dir 以下の全ファイルを input_bag_dir に flatten コピーする (download.py と同一処理)。

    pull-filtered-rosbag はネスト dir を作成するため、rglob で再帰的に全ファイルを
    input_bag_dir に平坦化する。metadata.yaml と *.mcap が直下に揃うことで
    step1_make_lite._open_reader が bag を認識できる。
    """
    for path in pull_dir.rglob("*"):
        if path.is_file():
            shutil.copy(path, input_bag_dir)


def _run_step1_make_lite(input_bag: Path, output_path: Path) -> bool:
    """step1_make_lite --kind real で real.lite を生成する。成功で True。

    rosbag2_py が必要なため ROS 環境 (install/setup.bash) が source 済みであること。
    """
    cmd = [
        sys.executable, "-m", f"{_PKG}.step1_make_lite",
        "--kind", "real",
        "--input", str(input_bag),
        "--output", str(output_path),
    ]
    proc = subprocess.run(cmd, check=False, env=os.environ.copy())  # noqa: S603
    return proc.returncode == 0


def _write_index(root: Path, records: list[dict]) -> Path:
    """raw_index.yaml を書く。既存を synthetic_id でマージ (べき等)。"""
    path = root / "raw_index.yaml"
    existing: dict[str, dict] = {}
    if path.is_file():
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for rec in data.get("records", []):
            existing[rec.get("synthetic_id", "")] = rec
    for rec in records:
        existing[rec["synthetic_id"]] = rec
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "records": sorted(existing.values(), key=lambda r: r.get("synthetic_id", "")),
            },
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return path


def _print_coverage_summary(selected: list[dict]) -> None:
    """収集セットの日付分布・時間統計を出力する (カバレッジ確認用)。

    量≠汎化: 同一日への偏りは状態空間の多様性を損なうため、日付分布を確認する。
    """
    dates: set[str] = set()
    durations: list[float] = []
    for item in selected:
        ts = _parse_timestamp(
            _get_field(item, "start_timestamp", "startTimestamp",
                       "recordingStartTime", "recording_start_time",
                       "recordingStart", "recording_start")
        )
        if ts:
            dates.add(datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d"))
        dur = _get_duration(item)
        if dur is not None:
            durations.append(dur)

    print("\n--- 収集セット カバレッジ概要 ---")
    if dates:
        sorted_dates = sorted(dates)
        print(f"  日付分布: {sorted_dates[0]} 〜 {sorted_dates[-1]} ({len(sorted_dates)} 日)")
        if len(sorted_dates) == 1:
            print(
                "  ⚠️  全データが同一日に集中しています。"
                "状態空間の多様性のため --date-from/--date-to で期間を広げることを推奨します"
            )
    if durations:
        total = sum(durations)
        print(
            f"  recording 時間: 合計={total/60:.1f}分 "
            f"(最短={min(durations):.0f}s / 中央値={sorted(durations)[len(durations)//2]:.0f}s"
            f" / 最長={max(durations):.0f}s)"
        )


def collect(
    vehicle_id: str,
    root: Path,
    *,
    date_from: str | None = None,
    date_to: str | None = None,
    limit: int = 20,
    min_duration_s: float = _MIN_DURATION_S_DEFAULT,
    topics: list[str] | None = None,
    dry_run: bool = False,
) -> list[dict]:
    """vehicle_id で rosbag を収集し real.lite を生成する。

    Args:
        vehicle_id: 対象 vehicle の UUID
        root: 収集ルートディレクトリ (<root>/datasets/<synthetic_id>/ を作成)
        date_from: 収集開始日 (YYYY-MM-DD)。日付を跨ぐことを推奨 (同一日は過学習リスク)
        date_to: 収集終了日 (YYYY-MM-DD)
        limit: 最大収集本数 (DL 量の最終ガード)
        min_duration_s: この秒数未満の bag をスキップ
        topics: pull する topic リスト (None で _OPENLOOP_TOPICS を使用)
        dry_run: True なら search/選別のみ実行し pull/lite 化はしない

    Returns:
        収集レコード (raw_index.yaml に記録する dict リスト)
    """
    if topics is None:
        topics = _OPENLOOP_TOPICS

    print(
        f"[collect] vehicle_id={vehicle_id} limit={limit} "
        f"date={date_from or '(最古)'}~{date_to or '(最新)'} "
        f"min_duration={min_duration_s}s"
    )

    # --- Step 1: search (全ページ, limit*3 まで検索してから選別) ---
    all_items: list[dict] = []
    page_token: str | None = None
    page = 0
    search_limit = max(limit * 3, _PAGE_SIZE)  # 十分な候補を集めてから選別
    while len(all_items) < search_limit:
        try:
            items, next_token = _search_log_files(
                vehicle_id, date_from=date_from, date_to=date_to, page_token=page_token
            )
        except RuntimeError as e:
            print(f"[WARN] search 失敗 (page {page}): {e}", file=sys.stderr)
            break
        if not items:
            break
        all_items.extend(items)
        page += 1
        print(f"  search page {page}: +{len(items)} → 累計 {len(all_items)} 件")
        if not next_token:
            break
        page_token = next_token

    print(f"[collect] search 完了: {len(all_items)} 件")

    # --- Step 2: 選別 (duration フィルタ + limit) ---
    selected: list[dict] = []
    skip_no_ts = 0
    skip_short = 0
    for item in all_items:
        if len(selected) >= limit:
            break
        dur = _get_duration(item)
        if dur is None:
            skip_no_ts += 1
            continue
        if dur < min_duration_s:
            skip_short += 1
            continue
        selected.append(item)

    n_limit_over = len(all_items) - len(selected) - skip_no_ts - skip_short
    print(
        f"[collect] 選別: {len(selected)}/{len(all_items)} 件 "
        f"(除外: 時刻なし={skip_no_ts} / 短い={skip_short} / limit超過={max(0, n_limit_over)})"
    )

    if not selected:
        print(
            "[WARN] 収集対象が 0 件です。"
            "--date-from/--date-to / --limit / --min-duration を調整してください",
            file=sys.stderr,
        )
        return []

    _print_coverage_summary(selected)

    if dry_run:
        print("\n[dry-run] pull/lite 化はスキップしました")
        records = [
            {
                "synthetic_id": _build_synthetic_id(item.get("id", "")),
                "rosbag_id": item.get("id", ""),
                "file_name": _get_field(item, "log_file_name", "fileName", "file_name") or "",
                "duration_s": _get_duration(item),
                "status": "dry_run",
            }
            for item in selected
        ]
        _write_index(root, records)
        return records

    # --- Step 3: 各 bag を pull → step1_make_lite で real.lite 化 ---
    records: list[dict] = []
    now_iso = datetime.now(timezone.utc).isoformat(timespec="seconds")

    for i, item in enumerate(selected, start=1):
        rosbag_id = item.get("id", "")
        file_name = _get_field(item, "log_file_name", "fileName", "file_name") or rosbag_id
        dur = _get_duration(item)
        synthetic_id = _build_synthetic_id(rosbag_id)

        print(
            f"\n=== [{i}/{len(selected)}] {rosbag_id[:16]}... (dur={dur:.0f}s) ===",
            flush=True,
        )

        ds_dir = root / "datasets" / synthetic_id
        input_bag_dir = ds_dir / "input_bag"
        lite_path = ds_dir / "real.lite"

        rec: dict = {
            "synthetic_id": synthetic_id,
            "rosbag_id": rosbag_id,
            "file_name": file_name,
            "duration_s": dur,
            "collected_at": now_iso,
            "status": "pending",
        }

        # 既に real.lite が存在すればスキップ (--resume 相当)
        if lite_path.exists() or (ds_dir / "real.lite.mcap").exists():
            print(f"  [SKIP] real.lite 既存: {ds_dir}")
            rec["status"] = "already_exists"
            records.append(rec)
            continue

        input_bag_dir.mkdir(parents=True, exist_ok=True)

        # input_bag に .db3 が既存なら pull をスキップ (中断後の再開)
        existing_db3 = list(input_bag_dir.glob("*.db3"))
        if existing_db3:
            print(f"  [SKIP pull] input_bag 既存 ({existing_db3[0].name})")
        else:
            # pull-filtered-rosbag: tempdir に落として _collect_bag で flatten
            # (download.py と同一パターン。pull 出力はネストした rosbag2 dir を作るため直接 input_bag_dir を
            # 渡すと step1_make_lite が bag を見つけられない)
            tmp_pull = Path(tempfile.mkdtemp(prefix="pull_", dir=ds_dir))
            try:
                print(f"  [pull] {file_name} → {tmp_pull.name}/ → {input_bag_dir.name}/")
                _pull_rosbag(rosbag_id, topics, tmp_pull)
                _collect_bag(tmp_pull, input_bag_dir)
            except RuntimeError as e:
                print(f"  [WARN] pull 失敗: {e}", file=sys.stderr)
                rec["status"] = "pull_failed"
                records.append(rec)
                continue
            finally:
                shutil.rmtree(tmp_pull, ignore_errors=True)

        # step1_make_lite (ROS 環境が source 済みであること)
        print(f"  [lite] {input_bag_dir} → {lite_path}")
        ok = _run_step1_make_lite(input_bag_dir, lite_path)
        if not ok:
            print(f"  [WARN] step1_make_lite 失敗: {ds_dir}", file=sys.stderr)
            rec["status"] = "lite_failed"
        else:
            rec["status"] = "success"
            print(f"  [OK] {synthetic_id[:16]}...")

        records.append(rec)

    # --- Step 4: raw_index.yaml 更新 ---
    index_path = _write_index(root, records)

    # --- Step 5: サマリー出力 ---
    n_ok = sum(1 for r in records if r["status"] == "success")
    n_skip = sum(1 for r in records if r["status"] == "already_exists")
    n_fail = len(records) - n_ok - n_skip
    print(
        f"\n=== 収集完了: 成功={n_ok} / スキップ(既存)={n_skip} / 失敗={n_fail} ===\n"
        f"  raw_index : {index_path}\n"
        f"  datasets/ : {root / 'datasets'}"
    )
    if n_ok > 0 or n_skip > 0:
        print(
            f"\n次のステップ:\n"
            f"  軽量ループ (主): make open_loop_tune ROOT={root}\n"
            f"  重量ループ (subset): make open_loop_batch ROOT={root}"
        )

    return records


def main() -> None:
    ap = argparse.ArgumentParser(
        description="vehicle_id で生rosbagを収集し open-loop 同定用の real.lite を生成する",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--vehicle-id", required=True,
        help="対象 vehicle の UUID (J6 #15 = e14508f0-5cc8-410c-886b-23aea4a8d5d8)",
    )
    ap.add_argument(
        "--root", required=True,
        help="収集ルートディレクトリ (<root>/datasets/<synthetic_id>/ に出力)",
    )
    ap.add_argument(
        "--date-from", default=None, metavar="YYYY-MM-DD",
        help="収集開始日 (inclusive)。状態空間の多様性のため日付を跨ぐことを強く推奨",
    )
    ap.add_argument(
        "--date-to", default=None, metavar="YYYY-MM-DD",
        help="収集終了日 (inclusive)",
    )
    ap.add_argument(
        "--limit", type=int, default=20,
        help="最大収集本数 (デフォルト: 20)。DL 量の最終ガード",
    )
    ap.add_argument(
        "--min-duration", type=float, default=_MIN_DURATION_S_DEFAULT,
        metavar="SECONDS",
        help=f"最低 recording 時間 [s] (デフォルト: {_MIN_DURATION_S_DEFAULT})。"
        "これより短い bag は AUTONOMOUS 区間なしとみなしてスキップ",
    )
    ap.add_argument(
        "--topics", nargs="+", default=None,
        help="pull する topic リスト (省略時: open-loop モデル同定用 superset を使用)",
    )
    ap.add_argument(
        "--dry-run", action="store_true",
        help="search と選別のみ実行し pull/lite 化はしない (件数・日付分布の事前確認用)",
    )
    args = ap.parse_args()

    root = Path(args.root).resolve()
    root.mkdir(parents=True, exist_ok=True)

    records = collect(
        args.vehicle_id,
        root,
        date_from=args.date_from,
        date_to=args.date_to,
        limit=args.limit,
        min_duration_s=args.min_duration,
        topics=args.topics,
        dry_run=args.dry_run,
    )

    if not records:
        sys.exit(1)


if __name__ == "__main__":
    main()
