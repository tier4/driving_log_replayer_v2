"""env / CLI / curve_config YAML を統合する `RuntimeConfig` dataclass.

ロード優先順位（高 → 低）:
  1. CLI 引数
  2. 環境変数
  3. `--curve-config <yaml>` で読み込んだ YAML（または `CURVE_CONFIG_YAML` env）
  4. ハードコードされたデフォルト

WHEELBASE は意図的に二系統で保持する:
  - `wheelbase_validation` = 5.15 m
    実機ログから運動学（yaw_rate = v * tan(steer) / L）を逆算する検証グラフ用。
    実データから推定された値で、`compare_logs.py` の bicycle model 検証や
    `analyze_real_curve2.py` のステア追従誤差解析で使う。
  - `wheelbase_sim` = 4.76012 m
    シミュレータの仕様値 (`vehicle_info.param.yaml::wheel_base`)。
    `analyze_per_step.py` の C++ 車両モデル再計算で使う。
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
_DEFAULT_CURVE2_INDEX = 1
_DEFAULT_CURVE2_WINDOW: tuple[float, float] = (20.0, 120.0)

# x2_dev/2231 のカーブ中心（後方互換デフォルト）
_DEFAULT_CURVE_CENTERS: list[dict] = [
    {"label": "カーブ①（右折）", "cx": 89440, "cy": 43200, "margin": 20},
    {"label": "カーブ②（左折）", "cx": 89301, "cy": 43085, "margin": 20},
    {"label": "カーブ③（右折）", "cx": 89372, "cy": 42830, "margin": 40},
]


@dataclass
class RuntimeConfig:
    """実行時設定 (env / CLI / curve_config YAML から構築)."""

    base_dir: Path                                  # lite/ と comparison/ の親
    lite_dir: Path                                  # base_dir/lite
    out_dir: Path                                   # base_dir/comparison
    figs_dir: Path                                  # base_dir/comparison/figures
    scenario_name: str = _DEFAULT_SCENARIO_NAME
    map_osm_path: Path | None = None                # resolve_map_osm 適用後
    wheelbase_validation: float = _DEFAULT_WHEELBASE_VALIDATION
    wheelbase_sim: float = _DEFAULT_WHEELBASE_SIM
    curve_centers: list[dict] | None = field(default_factory=lambda: list(_DEFAULT_CURVE_CENTERS))
    curve2_index: int = _DEFAULT_CURVE2_INDEX
    curve2_window: tuple[float, float] = _DEFAULT_CURVE2_WINDOW
    # カーブ別プロット対象の指定。各要素 = {"index": int, "launch_window": (start, end)}
    # build_runtime_config で curve_config YAML から読み込まれる。未指定なら
    # curve2_index/curve2_window から `[{"index": curve2_index, "launch_window": curve2_window}]`
    # を組み立てて埋める（既存 curve2 のみ動作と等価）。
    plot_curves: list[dict] = field(default_factory=list)
    topic_overrides: dict[str, Any] = field(default_factory=dict)

    @property
    def curve2(self) -> dict | None:
        """`curve_centers[curve2_index]` を返す。範囲外なら None。"""
        if not self.curve_centers:
            return None
        if 0 <= self.curve2_index < len(self.curve_centers):
            return self.curve_centers[self.curve2_index]
        return None


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
        "--curve-config",
        default=os.environ.get("CURVE_CONFIG_YAML"),
        help=(
            "カーブ設定 YAML パス (env: CURVE_CONFIG_YAML)。"
            "未指定=後方互換デフォルト、空文字=カーブ別解析スキップ"
        ),
    )
    parser.add_argument(
        "--topic-config",
        default=os.environ.get("TOPIC_CONFIG_YAML", ""),
        help="トピック設定 YAML パス (env: TOPIC_CONFIG_YAML)",
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
            curve_config=os.environ.get("CURVE_CONFIG_YAML"),
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

    # --- curve_config YAML ---
    cc: dict = {}
    if ns.curve_config is None:
        # 未指定 → デフォルト値を維持
        pass
    elif ns.curve_config == "":
        # 明示スキップ
        cfg.curve_centers = None
    else:
        loaded = _load_yaml(ns.curve_config)
        if loaded is None:
            cfg.curve_centers = None
        else:
            cc = loaded
            if cc.get("curve_centers"):
                cfg.curve_centers = cc["curve_centers"]
            else:
                cfg.curve_centers = None
            if "curve2_index" in cc:
                cfg.curve2_index = int(cc["curve2_index"])
            if "curve2_window" in cc:
                w = cc["curve2_window"]
                cfg.curve2_window = (
                    float(w.get("start", _DEFAULT_CURVE2_WINDOW[0])),
                    float(w.get("end", _DEFAULT_CURVE2_WINDOW[1])),
                )

    # --- plot_curves（curve_centers が無ければ空のまま）---
    if cfg.curve_centers:
        raw_plot_curves = cc.get("plot_curves") if cc else None
        if raw_plot_curves:
            normalized: list[dict] = []
            for entry in raw_plot_curves:
                if "index" not in entry:
                    warnings.warn(f"plot_curves エントリに index が無いためスキップ: {entry}")
                    continue
                idx = int(entry["index"])
                if not (0 <= idx < len(cfg.curve_centers)):
                    warnings.warn(
                        f"plot_curves.index={idx} が curve_centers 範囲外のためスキップ"
                    )
                    continue
                lw = entry.get("launch_window") or {}
                normalized.append({
                    "index": idx,
                    "launch_window": (
                        float(lw.get("start", _DEFAULT_CURVE2_WINDOW[0])),
                        float(lw.get("end", _DEFAULT_CURVE2_WINDOW[1])),
                    ),
                })
            cfg.plot_curves = normalized
        else:
            # 後方互換: 未指定なら curve2 のみを対象に
            cfg.plot_curves = [{
                "index": cfg.curve2_index,
                "launch_window": cfg.curve2_window,
            }]

    # --- scenario_name ---
    if ns.scenario_name:
        cfg.scenario_name = ns.scenario_name
    elif cc.get("scenario_name"):
        cfg.scenario_name = str(cc["scenario_name"])

    # --- wheelbase_validation ---
    if getattr(ns, "wheelbase", 0):
        cfg.wheelbase_validation = float(ns.wheelbase)
    elif "wheelbase_validation" in cc:
        cfg.wheelbase_validation = float(cc["wheelbase_validation"])
    elif "wheelbase" in cc:
        # curve_config 内 `wheelbase` キーは後方互換として受け付ける
        cfg.wheelbase_validation = float(cc["wheelbase"])

    # --- wheelbase_sim (シム仕様値) ---
    if "wheelbase_sim" in cc:
        cfg.wheelbase_sim = float(cc["wheelbase_sim"])
    else:
        # vehicle_info.param.yaml::wheel_base から読み取り
        try:
            from ._params_utils import load_sim_params  # noqa: PLC0415

            sim_params = load_sim_params()
            wb = sim_params.get("wheel_base")
            if wb is not None:
                cfg.wheelbase_sim = float(wb)
        except Exception:  # noqa: BLE001
            pass

    # --- map_osm ---
    # CLI/env 引数が未指定なら curve_config の map_osm_path も試す
    map_osm_arg = ns.map_osm
    if map_osm_arg is None and "map_osm_path" in cc:
        map_osm_arg = cc["map_osm_path"]
    cfg.map_osm_path = resolve_map_osm(map_osm_arg)

    # --- topic_overrides ---
    if ns.topic_config:
        loaded = _load_yaml(ns.topic_config)
        if loaded:
            cfg.topic_overrides = loaded

    return cfg
