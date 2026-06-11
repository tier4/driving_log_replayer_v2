"""env / CLI を統合する `RuntimeConfig` dataclass.

ロード優先順位（高 → 低）:
  1. CLI 引数
  2. 環境変数
  3. ハードコードされたデフォルト

WHEELBASE は意図的に二系統で保持する:
  - `wheelbase_validation` = 5.15 m
    実機ログから運動学（yaw_rate = v * tan(steer) / L）を逆算する検証グラフ用。
    実データから推定された値で、`step4_compare_logs.py` の bicycle model 検証や
    `tools/analyze_real_curve2.py` のステア追従誤差解析で使う。
  - `wheelbase_sim` = 4.76012 m
    シミュレータの仕様値 (`vehicle_info.param.yaml::wheel_base`)。
    `step5_analyze_nstep.py` の C++ 車両モデル再計算で使う。

scenario_config:
  scenario.yaml のパス。step4_compare_logs が sim 重ね描き (sim_runs) に使う。
  旧 sim_runs_config (sim_runs.yaml への直接パス) を置き換えた。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import os
from pathlib import Path
import warnings
from typing import Any

from ._map import resolve_map_osm


_DEFAULT_SCENARIO_NAME = "real_log_sim_comparison"
_DEFAULT_WHEELBASE_VALIDATION = 5.15
_DEFAULT_WHEELBASE_SIM = 4.76012


@dataclass
class RuntimeConfig:
    """実行時設定 (env / CLI から構築)."""

    base_dir: Path                                  # lite/ と comparison/ の親
    lite_dir: Path                                  # base_dir/lite
    out_dir: Path                                   # base_dir/comparison
    figs_dir: Path                                  # base_dir/comparison/figures
    scenario_name: str = _DEFAULT_SCENARIO_NAME
    map_osm_path: Path | None = None                # resolve_map_osm 適用後
    wheelbase_validation: float = _DEFAULT_WHEELBASE_VALIDATION
    wheelbase_sim: float = _DEFAULT_WHEELBASE_SIM
    # scenario.yaml のパス (step4_compare_logs が Conditions.sim_runs から sim 重ね描きに使う)
    scenario_config: Path | None = None
    topic_overrides: dict[str, Any] = field(default_factory=dict)


def add_common_cli_arguments(parser: argparse.ArgumentParser) -> None:
    """共通の CLI 引数を `parser` に追加する。デフォルト値は対応する env から取得。"""
    parser.add_argument(
        "--base-dir",
        default=os.environ.get("BEST_MODEL_BASE_DIR"),
        help="lite/ と comparison/ の親ディレクトリ (env: BEST_MODEL_BASE_DIR)",
    )
    parser.add_argument(
        "--map-osm",
        default=os.environ.get("MAP_OSM_PATH"),
        help=(
            "lanelet2_map.osm の絶対パス (env: MAP_OSM_PATH)。"
            "未指定=デフォルトディレクトリを試行、空文字=地図なし、パス=そのパスを使用。"
        ),
    )
    parser.add_argument(
        "--scenario-name",
        default=os.environ.get("SCENARIO_NAME", ""),
        help="図タイトルに表示するシナリオ名 (env: SCENARIO_NAME)",
    )
    parser.add_argument(
        "--wheelbase",
        type=float,
        default=float(os.environ.get("WHEELBASE", "0") or "0"),
        help=(
            "bicycle model 検証用ホイールベース [m]。"
            "0 なら既定値を使用 (env: WHEELBASE)"
        ),
    )
    parser.add_argument(
        "--topic-config",
        default=os.environ.get("TOPIC_CONFIG_YAML", ""),
        help="トピック設定 YAML パス (env: TOPIC_CONFIG_YAML)",
    )
    parser.add_argument(
        "--scenario",
        default=os.environ.get("SCENARIO_CONFIG_YAML"),
        help=(
            "scenario.yaml パス (env: SCENARIO_CONFIG_YAML)。"
            "step4_compare_logs が Conditions.sim_runs から sim 重ね描きに使う。"
            "未指定なら実機 single-log のみ"
        ),
    )


def _load_yaml(path: str) -> dict | None:
    """YAML を読む。失敗時は警告して None。"""
    try:
        import yaml as _yaml  # noqa: PLC0415
    except ImportError:
        warnings.warn("PyYAML がインストールされていません", stacklevel=2)
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return _yaml.safe_load(f) or {}
    except Exception as e:  # noqa: BLE001
        warnings.warn(f"YAML 読み込み失敗 {path}: {e}", stacklevel=2)
        return None


def build_runtime_config(
    args: argparse.Namespace | None = None,
    *,
    default_base_dir: Path | None = None,
) -> RuntimeConfig:
    """`add_common_cli_arguments` で得た `args` から `RuntimeConfig` を構築する。

    `args=None` でも env からの構築が可能。
    `default_base_dir` は `--base-dir` 未指定時のフォールバック。
    """
    if args is None:
        ns = argparse.Namespace(
            base_dir=os.environ.get("BEST_MODEL_BASE_DIR"),
            map_osm=os.environ.get("MAP_OSM_PATH"),
            scenario_name=os.environ.get("SCENARIO_NAME", ""),
            wheelbase=float(os.environ.get("WHEELBASE", "0") or "0"),
            topic_config=os.environ.get("TOPIC_CONFIG_YAML", ""),
        )
    else:
        ns = args

    # --- base_dir ---
    if ns.base_dir:
        base_dir = Path(ns.base_dir)
    elif default_base_dir is not None:
        base_dir = Path(default_base_dir)
    else:
        base_dir = Path.cwd()

    lite_dir = base_dir / "lite"
    out_dir = base_dir / "comparison"
    figs_dir = out_dir / "figures"

    cfg = RuntimeConfig(
        base_dir=base_dir,
        lite_dir=lite_dir,
        out_dir=out_dir,
        figs_dir=figs_dir,
    )

    # --- scenario_name ---
    if ns.scenario_name:
        cfg.scenario_name = ns.scenario_name

    # --- wheelbase_validation ---
    if getattr(ns, "wheelbase", 0):
        cfg.wheelbase_validation = float(ns.wheelbase)

    # --- wheelbase_sim (シム仕様値) ---
    try:
        from ._params_utils import load_sim_params  # noqa: PLC0415

        sim_params = load_sim_params()
        wb = sim_params.get("wheel_base")
        if wb is not None:
            cfg.wheelbase_sim = float(wb)
    except Exception:  # noqa: BLE001
        pass

    # --- map_osm ---
    cfg.map_osm_path = resolve_map_osm(ns.map_osm)

    # --- topic_overrides ---
    if ns.topic_config:
        loaded = _load_yaml(ns.topic_config)
        if loaded:
            cfg.topic_overrides = loaded

    # --- scenario_config (step4_compare_logs が Conditions.sim_runs から sim 重ね描きに使う) ---
    if getattr(ns, "scenario", None):
        sc = Path(ns.scenario)
        cfg.scenario_config = sc if sc.exists() else None
        if cfg.scenario_config is None:
            warnings.warn(f"scenario.yaml が存在しません: {sc} (sim 重ね描きスキップ)")

    return cfg
