"""dataset の dev / holdout 決定的分割。

v3 開発の統計的規律: 分析・フィットは dev のみで行い、v2 vs v3 の最終採否判定は
holdout の robust_score でも確認する。分割は dataset_id の sha1 ハッシュによる
決定的分割で、データセットの追加・削除に対して既存 id の所属が変わらない。
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import pandas as pd

# holdout 比率 (%)。318 dataset なら holdout ≈ 64 本。
HOLDOUT_PCT = 20


def split_of(dataset_id: str) -> str:
    """dataset_id の所属 split ("dev" | "holdout") を返す (決定的)。"""
    digest = hashlib.sha1(dataset_id.encode("utf-8")).hexdigest()
    return "holdout" if int(digest, 16) % 100 < HOLDOUT_PCT else "dev"


def split_dataset_ids(dataset_ids: list[str]) -> tuple[list[str], list[str]]:
    """(dev, holdout) の 2 リストへ分割する。"""
    dev = [ds for ds in dataset_ids if split_of(ds) == "dev"]
    holdout = [ds for ds in dataset_ids if split_of(ds) == "holdout"]
    return dev, holdout


def write_split_csv(path: Path, dataset_ids: list[str]) -> pd.DataFrame:
    """全 dataset の所属 split を CSV に固定出力する (holdout_split.csv)。"""
    df = pd.DataFrame({
        "dataset_id": sorted(dataset_ids),
    })
    df["split"] = df["dataset_id"].map(split_of)
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False)
    return df
