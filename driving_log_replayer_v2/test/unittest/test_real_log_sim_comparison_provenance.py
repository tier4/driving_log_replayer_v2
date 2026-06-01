# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""`real_log_sim_comparison.lib._provenance` と Stage 7/9 共通の `_parabolic_min` のテスト."""

from __future__ import annotations

from pathlib import Path

from driving_log_replayer_v2.real_log_sim_comparison.lib._provenance import (
    capture_dp_provenance,
    format_provenance_line,
    read_provenance,
    write_provenance,
)
from driving_log_replayer_v2.real_log_sim_comparison.step7_identify_kus import _parabolic_min


# --- provenance ---


def test_format_provenance_line_none() -> None:
    assert "unknown" in format_provenance_line(None)


def test_format_provenance_line_full() -> None:
    line = format_provenance_line({
        "dp_exp_name": "expA",
        "dp_onnx_sha8": "abc12345",
        "autoware_version": "v1.2.3",
        "dp_train_set": "ds_x",
    })
    assert "expA" in line
    assert "abc12345" in line
    assert "v1.2.3" in line
    assert "ds_x" in line


def test_capture_missing_onnx_is_graceful() -> None:
    # 明示パスが存在しない場合も例外を出さず None 系で返す (onnx.stat() でクラッシュしない)。
    prov = capture_dp_provenance("/nonexistent/diffusion_planner.onnx")
    assert prov["dp_onnx_sha8"] is None
    assert prov["dp_onnx_size"] is None
    assert prov["dp_exp_name"] is None
    assert "autoware_version" in prov  # version は別系統で取得 (None ではなく文字列)


def test_write_read_roundtrip(tmp_path: Path) -> None:
    written = write_provenance(tmp_path, extra={"tag": "sim_x"})
    assert (tmp_path / "provenance.json").exists()
    read_back = read_provenance(tmp_path)
    assert read_back == written
    assert read_back["tag"] == "sim_x"


def test_read_provenance_missing_returns_none(tmp_path: Path) -> None:
    assert read_provenance(tmp_path) is None


# --- _parabolic_min (Stage 7/9 の同定で使う放物線サブグリッド推定) ---


def test_parabolic_min_interior_vertex() -> None:
    # 対称な凸 3 点 → 頂点は中央 x=1.0
    v = _parabolic_min([0.0, 1.0, 2.0], [1.0, 0.0, 1.0])
    assert v is not None
    assert abs(v - 1.0) < 1e-9


def test_parabolic_min_asymmetric_vertex_between_points() -> None:
    v = _parabolic_min([0.0, 1.0, 2.0], [1.0, 0.2, 0.6])
    assert v is not None
    assert 0.0 < v < 2.0


def test_parabolic_min_edge_returns_none() -> None:
    # 最小がグリッド端 (index 0) → None
    assert _parabolic_min([0.0, 1.0, 2.0], [0.0, 0.5, 1.0]) is None
