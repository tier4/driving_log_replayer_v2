"""マルチデータセット collection レイアウトの SSOT (発見・manifest・メトリクス読み).

collection ディレクトリは複数データセットの per-dataset バンドル成果物を束ねる:

    <collection>/
    ├── collection.yaml              # 収集 manifest (status / provenance 記録)
    ├── runs/<dataset_id>/           # run_batch の per-dataset launch 出力 (実体)
    ├── datasets/<dataset_id>/       # 収集ビュー (collect_datasets.py が symlink を張る)
    │   ├── real.lite[.mcap]         # multi_dataset_tune._discover 互換
    │   ├── comparison/              # step4〜10 成果物 (fig.json / metrics JSON)
    │   └── scenarios/
    ├── cross_dataset/               # step13 出力
    └── report.html                  # step11 マルチ DS モード出力

発見の正は実ディレクトリ (datasets/ 走査)。manifest は status (sim 失敗等で datasets/ に
入らなかった DS の記録) を補完する。datasets/ サブディレクトリが無い場合は collection 直下を
走査する (collect_datasets.py --flat や手作り collection 互換)。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import yaml

from ._io import resolve_lite_bag

MANIFEST_NAME = "collection.yaml"
CROSS_DIR_NAME = "cross_dataset"

# collection 直下を走査するときにデータセットディレクトリとみなさないサブディレクトリ名 (SSOT)。
_COLLECTION_RESERVED_DIRS: frozenset[str] = frozenset(
    {"runs", "datasets", CROSS_DIR_NAME, "__pycache__"}
)


@dataclass
class DatasetEntry:
    """collection 内の 1 データセット。"""

    dataset_id: str
    dir: Path | None              # <collection>/datasets/<id> (manifest のみの失敗 DS は None)
    real_lite: Path | None
    comparison_dir: Path | None
    scenarios_dir: Path | None
    status: str                   # success / sim_failed / analysis_failed / collect_failed


def _entry_from_dir(sub: Path, status: str = "success") -> DatasetEntry:
    comparison = sub / "comparison"
    scenarios = sub / "scenarios"
    return DatasetEntry(
        dataset_id=sub.name,
        dir=sub,
        real_lite=resolve_lite_bag(sub, "real"),
        comparison_dir=comparison if comparison.is_dir() else None,
        scenarios_dir=scenarios if scenarios.is_dir() else None,
        status=status,
    )


def datasets_root(collection_dir: Path) -> Path:
    """収集ビューのルート (datasets/ があればそこ、無ければ collection 直下)。"""
    sub = collection_dir / "datasets"
    return sub if sub.is_dir() else collection_dir


def load_manifest(collection_dir: Path) -> dict | None:
    path = collection_dir / MANIFEST_NAME
    if not path.is_file():
        return None
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def write_manifest(collection_dir: Path, manifest: dict) -> Path:
    path = collection_dir / MANIFEST_NAME
    path.write_text(
        yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    return path


def discover_collection(collection_dir: Path) -> list[DatasetEntry]:
    """collection 配下のデータセットを列挙する。

    実ディレクトリ走査を正とし、manifest の status を補完する。manifest にあるが
    ディレクトリに無い DS (sim 失敗等) も status 付きで返す (レポートで欠損を明示するため)。
    """
    root = datasets_root(collection_dir)
    skip = _COLLECTION_RESERVED_DIRS
    entries: dict[str, DatasetEntry] = {}
    if root.is_dir():
        for sub in sorted(root.iterdir()):
            if not sub.is_dir() or sub.name in skip:
                continue
            e = _entry_from_dir(sub)
            if e.real_lite is None and e.comparison_dir is None:
                continue  # データセットの体を成していないディレクトリは無視
            entries[e.dataset_id] = e

    manifest = load_manifest(collection_dir)
    for rec in (manifest or {}).get("datasets", []):
        ds_id = rec.get("dataset_id")
        if not ds_id:
            continue
        status = rec.get("status", "success")
        if ds_id in entries:
            entries[ds_id].status = status
        else:
            entries[ds_id] = DatasetEntry(
                dataset_id=ds_id, dir=None, real_lite=None,
                comparison_dir=None, scenarios_dir=None, status=status,
            )
    return sorted(entries.values(), key=lambda e: e.dataset_id)


def load_dataset_metrics(e: DatasetEntry) -> dict | None:
    """per-dataset の機械可読メトリクスを読む。

    返り値 {"closed": metrics_closed_loop.json, "cases": cases_metrics.json}。
    どちらか欠損なら None (step4/step6 が未実行 = 解析未完了として呼び出し側が除外・明示する)。
    """
    if e.comparison_dir is None:
        return None
    closed_path = e.comparison_dir / "metrics_closed_loop.json"
    cases_path = e.comparison_dir / "cases" / "cases_metrics.json"
    if not closed_path.is_file() or not cases_path.is_file():
        return None
    return {
        "closed": json.loads(closed_path.read_text(encoding="utf-8")),
        "cases": json.loads(cases_path.read_text(encoding="utf-8")),
    }
