# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison._map.resolve_map_osm` の三状態モデル検証."""

from __future__ import annotations

from pathlib import Path

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib import _map


def test_resolve_map_osm_with_explicit_path(tmp_path: Path) -> None:
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("<osm/>", encoding="utf-8")
    assert _map.resolve_map_osm(str(osm)) == osm


def test_resolve_map_osm_with_empty_string_returns_none() -> None:
    assert _map.resolve_map_osm("") is None


def test_resolve_map_osm_with_nonexistent_path_returns_none() -> None:
    with pytest.warns(UserWarning):
        result = _map.resolve_map_osm("/no/such/path.osm")
    assert result is None


def test_resolve_map_osm_with_none_uses_default_dir(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    # _DEFAULT_MAP_DIR を tmp に差し替えて、その下に複数バージョン配置
    fake_dir = tmp_path / "fake_map"
    (fake_dir / "v1").mkdir(parents=True)
    (fake_dir / "v2").mkdir(parents=True)
    (fake_dir / "v1" / "lanelet2_map.osm").write_text("<osm/>")
    (fake_dir / "v2" / "lanelet2_map.osm").write_text("<osm/>")

    monkeypatch.setattr(_map, "_DEFAULT_MAP_DIR", fake_dir)
    result = _map.resolve_map_osm(None)
    # 降順 sort で最新 (v2) を選ぶ
    assert result is not None
    assert result.parent.name == "v2"


def test_resolve_map_osm_with_none_returns_none_when_default_dir_missing(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(_map, "_DEFAULT_MAP_DIR", tmp_path / "does_not_exist")
    assert _map.resolve_map_osm(None) is None
