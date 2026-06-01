# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison._runtime_config.build_runtime_config` のテスト."""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.lib import _map
from driving_log_replayer_v2.real_log_sim_comparison.lib._runtime_config import (
    _DEFAULT_CURVE_CENTERS,
    _DEFAULT_WHEELBASE_VALIDATION,
    add_common_cli_arguments,
    build_runtime_config,
)


def _empty_ns(**overrides) -> argparse.Namespace:
    defaults = {
        "base_dir": None,
        "map_osm": "",  # 地図描画スキップを既定として、map glob を回避
        "scenario_name": "",
        "wheelbase": 0.0,
        "curve_config": None,
        "topic_config": "",
    }
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def test_defaults_when_no_args(tmp_path: Path) -> None:
    cfg = build_runtime_config(_empty_ns(), default_base_dir=tmp_path)
    assert cfg.base_dir == tmp_path
    assert cfg.lite_dir == tmp_path / "lite"
    assert cfg.out_dir == tmp_path / "comparison"
    assert cfg.figs_dir == tmp_path / "comparison" / "figures"
    assert cfg.scenario_name == "real_log_sim_comparison"
    assert cfg.wheelbase_validation == _DEFAULT_WHEELBASE_VALIDATION
    assert cfg.curve_centers == _DEFAULT_CURVE_CENTERS
    assert cfg.curve2_index == 1
    assert cfg.curve2_window == (20.0, 120.0)


def test_curve_config_empty_string_skips_centers(tmp_path: Path) -> None:
    cfg = build_runtime_config(
        _empty_ns(curve_config=""), default_base_dir=tmp_path
    )
    assert cfg.curve_centers is None


def test_curve_config_yaml_overrides_centers(tmp_path: Path) -> None:
    yml = tmp_path / "curve.yaml"
    yml.write_text(
        """
curve_centers:
  - {label: "X", cx: 100, cy: 200, margin: 30}
curve2_index: 0
curve2_window:
  start: 5.0
  end: 80.0
scenario_name: "TestScenario"
wheelbase_validation: 4.2
""",
        encoding="utf-8",
    )
    cfg = build_runtime_config(
        _empty_ns(curve_config=str(yml)), default_base_dir=tmp_path
    )
    assert cfg.curve_centers == [{"label": "X", "cx": 100, "cy": 200, "margin": 30}]
    assert cfg.curve2_index == 0
    assert cfg.curve2_window == (5.0, 80.0)
    assert cfg.scenario_name == "TestScenario"
    assert cfg.wheelbase_validation == 4.2


def test_cli_wheelbase_overrides_yaml(tmp_path: Path) -> None:
    yml = tmp_path / "curve.yaml"
    yml.write_text("wheelbase_validation: 4.0\n", encoding="utf-8")
    cfg = build_runtime_config(
        _empty_ns(curve_config=str(yml), wheelbase=5.5), default_base_dir=tmp_path
    )
    assert cfg.wheelbase_validation == 5.5


def test_cli_scenario_name_overrides_yaml(tmp_path: Path) -> None:
    yml = tmp_path / "curve.yaml"
    yml.write_text("scenario_name: yaml_name\n", encoding="utf-8")
    cfg = build_runtime_config(
        _empty_ns(curve_config=str(yml), scenario_name="cli_name"),
        default_base_dir=tmp_path,
    )
    assert cfg.scenario_name == "cli_name"


def test_curve2_property_returns_indexed_dict(tmp_path: Path) -> None:
    cfg = build_runtime_config(_empty_ns(), default_base_dir=tmp_path)
    assert cfg.curve2 == _DEFAULT_CURVE_CENTERS[1]


def test_map_osm_explicit_path(tmp_path: Path) -> None:
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("<osm/>", encoding="utf-8")
    cfg = build_runtime_config(
        _empty_ns(map_osm=str(osm)), default_base_dir=tmp_path
    )
    assert cfg.map_osm_path == osm


def test_curve_config_map_osm_path_picked_up(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # curve_config YAML 内 map_osm_path を CLI 未指定で拾うか
    osm = tmp_path / "from_yaml.osm"
    osm.write_text("<osm/>", encoding="utf-8")
    yml = tmp_path / "curve.yaml"
    yml.write_text(f'map_osm_path: "{osm}"\n', encoding="utf-8")

    # CLI map_osm を None にして「未指定 = YAML から拾う」のフォールバックに入らせる
    cfg = build_runtime_config(
        _empty_ns(curve_config=str(yml), map_osm=None),
        default_base_dir=tmp_path,
    )
    assert cfg.map_osm_path == osm
