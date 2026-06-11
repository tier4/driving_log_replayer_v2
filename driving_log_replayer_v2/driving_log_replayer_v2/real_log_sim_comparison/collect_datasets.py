#!/usr/bin/env python3
"""per-dataset バンドル成果物を collection ディレクトリに収集する (マルチデータセット評価の collect 段)。

クラウドは 1 評価ジョブ = 1 データセットで、各ジョブが result_archive に
lite/real.lite と comparison/ (解析成果物 + metrics JSON) を出力する。本スクリプトは
複数ジョブ (= 複数データセット) の成果物を 1 つの collection に symlink で集約し、

- `multi_dataset_tune.py` が dataset 横断で open-loop rollout 誤差を集計 (real.lite)
- `step13_cross_dataset.py` が rollout 再実行なしで横断分析 (comparison/ の metrics JSON)
- `step11_build_html_report.py --collection-dir` がマルチ DS 単一レポートを生成

できるようにする。収集レイアウト (lib._collection が SSOT):

    <collection-dir>/collection.yaml             (manifest)
    <collection-dir>/datasets/<dataset_id>/
        ├── real.lite[.mcap]   -> bundle/lite/real.lite
        ├── comparison         -> bundle/comparison
        └── scenarios          -> bundle/scenarios

使い方:
    # バンドル (out/<ts> または result_archive/...) から dataset_id を auto 推定して収集
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.collect_datasets \
        --bundle sample/out/20260604_173001 \
        --bundle sample/out/20260604_145811 \
        --collection-dir sample/out/multi_eval

    # dataset_id を明示
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.collect_datasets \
        --add f20a29ab=sample/out/20260604_173001

クラウドモード: webauto で評価ジョブの result_archive 中間生成物を DL してから --bundle で指す。
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import re
import sys

import yaml

from .lib._collection import load_manifest, write_manifest
from .lib._io import resolve_bundle_dir as _resolve_bundle, resolve_lite_bag

_DEFAULT_COLLECTION = Path(__file__).parent / "sample" / "multi"
_UUID_RE = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")


def _infer_dataset_id(bundle: Path) -> str | None:
    """バンドルの scenarios/auto_scenario.yaml の RoadNetwork.LogicFile.filepath から dataset UUID を取り出す。

    auto_scenario.yaml は step2 が生成する OpenSCENARIO YAML で、
    `OpenSCENARIO.RoadNetwork.LogicFile.filepath` に lanelet2 マップの絶対パスが入る。
    このパスは `annotation_dataset/<UUID>/0/map/lanelet2_map.osm` 形式のため UUID が一意に特定できる。
    全文 regex スキャンだと他フィールドの UUID-like 文字列に誤マッチする恐れがあるため、
    構造的に特定フィールドを読む。
    """
    scenario = bundle / "scenarios" / "auto_scenario.yaml"
    if not scenario.is_file():
        return None
    try:
        doc = yaml.safe_load(scenario.read_text(encoding="utf-8", errors="ignore"))
    except yaml.YAMLError:
        return None
    filepath = (
        ((doc or {}).get("OpenSCENARIO") or {})
        .get("RoadNetwork", {})
        .get("LogicFile", {})
        .get("filepath", "")
    )
    m = _UUID_RE.search(str(filepath))
    return m.group(0) if m else None


def _relink(dst: Path, src: Path) -> None:
    if dst.is_symlink() or dst.exists():
        dst.unlink()
    dst.symlink_to(src)


def collect_bundle(bundle: Path, dataset_id: str, collection_dir: Path) -> dict:
    """1 バンドルの成果物を <collection>/datasets/<dataset_id>/ に symlink し manifest レコードを返す。

    real.lite が無ければ FileNotFoundError。comparison/scenarios は存在するものだけ張る
    (sim だけ成功し解析未完了のバンドルも real.lite ベースの tune には使えるため)。
    """
    real = resolve_lite_bag(bundle / "lite", "real")
    if real is None:
        raise FileNotFoundError(f"real.lite が見つかりません: {bundle / 'lite'}")
    dst_dir = collection_dir / "datasets" / dataset_id
    dst_dir.mkdir(parents=True, exist_ok=True)
    _relink(dst_dir / real.name, real)  # real.lite または real.lite.mcap
    linked = [real.name]
    for name in ("comparison", "scenarios"):
        src = bundle / name
        if src.is_dir():
            _relink(dst_dir / name, src)
            linked.append(name)
    return {
        "dataset_id": dataset_id,
        "bundle": str(bundle),
        "linked": linked,
        "status": "success",
        "collected_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }


def update_manifest(collection_dir: Path, records: list[dict]) -> Path:
    """既存 manifest に収集レコードを dataset_id でマージして書き戻す。"""
    manifest = load_manifest(collection_dir) or {"schema_version": 1, "datasets": []}
    by_id = {rec.get("dataset_id"): rec for rec in manifest.get("datasets", [])}
    for rec in records:
        by_id[rec["dataset_id"]] = rec
    manifest["datasets"] = [by_id[k] for k in sorted(by_id)]
    return write_manifest(collection_dir, manifest)


def main() -> None:
    ap = argparse.ArgumentParser(description="per-dataset バンドル成果物をマルチ評価用に収集")
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

    records: list[dict] = []
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
        try:
            rec = collect_bundle(bundle, ds_id, collection_dir)
        except FileNotFoundError as e:
            print(f"[WARN] {e}", file=sys.stderr)
            continue
        print(f"[OK] {ds_id}: {', '.join(rec['linked'])} <- {bundle}")
        records.append(rec)

    if records:
        manifest_path = update_manifest(collection_dir, records)
        print(f"\n収集完了: {len(records)} データセット -> {collection_dir}")
        print(f"manifest: {manifest_path}")
    else:
        print("収集 0 件", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
