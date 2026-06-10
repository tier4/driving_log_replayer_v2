#!/usr/bin/env python3
"""
per-dataset の real.lite を収集ディレクトリに集める (マルチデータセット評価の collect 段)。

クラウドは 1 評価ジョブ = 1 データセットで、各ジョブが result_archive に lite/real.lite を出力する。
本スクリプトは複数ジョブ (= 複数データセット) の real.lite を 1 つの収集ディレクトリに集約し、
`multi_dataset_tune.py` が dataset 横断で open-loop N-step rollout 誤差を集計できるようにする。

収集レイアウト (resolve_lite_bag がそのまま読める形):
    <collection-dir>/<dataset_id>/real.lite        (rosbag2 dir 形式)
    <collection-dir>/<dataset_id>/real.lite.mcap   (単一ファイル形式)

使い方:
    # バンドル (out/<ts> または result_archive/...) から dataset_id を auto 推定して収集
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.collect_real_lite \
        --bundle sample/out/20260604_173001 \
        --bundle sample/out/20260604_145811

    # dataset_id を明示
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.collect_real_lite \
        --add f20a29ab=sample/out/20260604_173001 \
        --add 8edfbb02=sample/out/20260604_145811

クラウドモード (後続): webauto で評価ジョブの result_archive 中間生成物 (lite/real.lite) を
DL してから --bundle で指す (DL 経路は別途整備)。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

from .lib._io import resolve_lite_bag

_DEFAULT_COLLECTION = Path(__file__).parent / "sample" / "multi"
_UUID_RE = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")


def _resolve_bundle(raw: Path) -> Path:
    """lite/ を含むバンドルディレクトリを解決 (run_analysis._resolve_bundle_dir と同方針)。"""
    for c in (
        raw,
        raw / "result_archive" / "real_log_sim_comparison",
        raw / "real_log_sim_comparison",
    ):
        if (c / "lite").is_dir():
            return c.resolve()
    raise FileNotFoundError(f"lite/ を含むバンドルが見つかりません: {raw}")


def _infer_dataset_id(bundle: Path) -> str | None:
    """バンドルの scenarios/auto_scenario.yaml の map filepath に含まれる UUID を dataset_id とみなす。"""
    scenario = bundle / "scenarios" / "auto_scenario.yaml"
    if not scenario.is_file():
        return None
    text = scenario.read_text(encoding="utf-8", errors="ignore")
    m = _UUID_RE.search(text)
    return m.group(0) if m else None


def _link_real_lite(dataset_id: str, bundle: Path, collection_dir: Path) -> Path:
    """bundle/lite の real.lite を collection_dir/<dataset_id>/ に symlink する。"""
    real = resolve_lite_bag(bundle / "lite", "real")
    if real is None:
        raise FileNotFoundError(f"real.lite が見つかりません: {bundle / 'lite'}")
    dst_dir = collection_dir / dataset_id
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / real.name  # real.lite または real.lite.mcap
    if dst.is_symlink() or dst.exists():
        dst.unlink()
    dst.symlink_to(real)
    return dst


def main() -> None:
    ap = argparse.ArgumentParser(description="per-dataset real.lite をマルチ評価用に収集")
    ap.add_argument(
        "--bundle",
        action="append",
        default=[],
        help="バンドル (out/<ts> or result_archive/...)。dataset_id は auto_scenario.yaml から推定",
    )
    ap.add_argument(
        "--add",
        action="append",
        default=[],
        metavar="DATASET_ID=BUNDLE",
        help="dataset_id を明示して収集 (複数指定可)",
    )
    ap.add_argument(
        "--collection-dir",
        default=str(_DEFAULT_COLLECTION),
        help=f"収集先ディレクトリ (既定: {_DEFAULT_COLLECTION})",
    )
    args = ap.parse_args()

    collection_dir = Path(args.collection_dir)
    entries: list[tuple[str | None, Path]] = []
    for spec in args.add:
        if "=" not in spec:
            print(f"ERROR: --add は DATASET_ID=BUNDLE 形式: {spec}", file=sys.stderr)
            sys.exit(2)
        ds_id, raw = spec.split("=", 1)
        entries.append((ds_id.strip(), Path(raw.strip())))
    for raw in args.bundle:
        entries.append((None, Path(raw)))

    if not entries:
        print("ERROR: --bundle か --add を 1 つ以上指定してください", file=sys.stderr)
        sys.exit(2)

    collected = 0
    for ds_id, raw in entries:
        try:
            bundle = _resolve_bundle(raw)
        except FileNotFoundError as e:
            print(f"[WARN] {e}", file=sys.stderr)
            continue
        if ds_id is None:
            ds_id = _infer_dataset_id(bundle)
            if ds_id is None:
                print(
                    f"[WARN] dataset_id を推定できません (--add で明示してください): {bundle}",
                    file=sys.stderr,
                )
                continue
        dst = _link_real_lite(ds_id, bundle, collection_dir)
        print(f"[OK] {ds_id}: {dst} -> {dst.resolve()}")
        collected += 1

    print(f"\n収集完了: {collected} データセット -> {collection_dir}")
    if collected == 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
