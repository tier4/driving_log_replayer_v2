# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison._dataset.resolve_t4_dataset_path` の解決規則検証."""

from __future__ import annotations

from pathlib import Path

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib import _dataset

_UUID = "b126c24f-4891-40d3-87cb-d1be36dc6db6"


def _make_input_bag(dataset_dir: Path) -> None:
    """`dataset_dir/input_bag/` に有効な bag (db3) を 1 つ作る。"""
    input_bag = dataset_dir / "input_bag"
    input_bag.mkdir(parents=True)
    (input_bag / "input_bag.db3").write_bytes(b"")
    (input_bag / "metadata.yaml").write_text("", encoding="utf-8")


def test_resolve_flat_cloud_layout(tmp_path: Path) -> None:
    # クラウド相当: <root>/<uuid>/input_bag/ が直下にある
    uuid_dir = tmp_path / _UUID
    _make_input_bag(uuid_dir)
    assert _dataset.resolve_t4_dataset_path(tmp_path, _UUID) == uuid_dir.resolve()


def test_resolve_frame_local_layout(tmp_path: Path) -> None:
    # ローカル相当: <root>/<uuid>/0/input_bag/
    frame = tmp_path / _UUID / "0"
    _make_input_bag(frame)
    assert _dataset.resolve_t4_dataset_path(tmp_path, _UUID) == frame.resolve()


def test_resolve_multi_frame_picks_latest(tmp_path: Path) -> None:
    # 複数 frame は最新 (名前 sort 末尾 = "1") を採用する
    for frame_name in ("0", "1"):
        _make_input_bag(tmp_path / _UUID / frame_name)
    result = _dataset.resolve_t4_dataset_path(tmp_path, _UUID)
    assert result == (tmp_path / _UUID / "1").resolve()


def test_resolve_uuid_dir_not_directly_under_root(tmp_path: Path) -> None:
    # UUID dir が root 直下でなくても再帰探索で見つける (.lock は除外)
    frame = tmp_path / "nested" / _UUID / "0"
    _make_input_bag(frame)
    assert _dataset.resolve_t4_dataset_path(tmp_path, _UUID) == frame.resolve()


def test_resolve_missing_uuid_raises(tmp_path: Path) -> None:
    with pytest.raises(_dataset.DatasetResolutionError, match="見つかりません"):
        _dataset.resolve_t4_dataset_path(tmp_path, _UUID)


def test_resolve_missing_input_bag_raises(tmp_path: Path) -> None:
    # UUID dir はあるが input_bag が無い → intermediate artifacts 不足の案内
    (tmp_path / _UUID / "0").mkdir(parents=True)
    with pytest.raises(_dataset.DatasetResolutionError, match="include-intermediate-artifacts"):
        _dataset.resolve_t4_dataset_path(tmp_path, _UUID)


def test_resolve_input_bag_without_bag_files_is_invalid(tmp_path: Path) -> None:
    # input_bag/ はあるが *.mcap / *.db3 が無い → 無効扱い
    (tmp_path / _UUID / "0" / "input_bag").mkdir(parents=True)
    with pytest.raises(_dataset.DatasetResolutionError):
        _dataset.resolve_t4_dataset_path(tmp_path, _UUID)
