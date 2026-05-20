"""lanelet2 OSM 地図のローダーと `map_osm` パス三状態解決.

三状態モデル:
  - `override is None` → デフォルトディレクトリを glob、見つかればそのパス／無ければ None
  - `override == ""`   → None を返す（明示「地図なし」）
  - `override == path` → そのパスを使用（存在しなければ警告して None）

`compare_logs.py` 既存の挙動を全スクリプトに横展開するためのモジュール。
"""

from __future__ import annotations

from pathlib import Path
import warnings

from lxml import etree
import numpy as np


# 後方互換デフォルト（x2_dev/2231）。三状態モデルの「未設定」状態でのみ参照される。
# 他地図対応のためには `--map-osm` CLI / `MAP_OSM_PATH` env / curve_config YAML の
# `map_osm_path` を指定すること。
_DEFAULT_MAP_DIR = Path.home() / ".webauto/simulation/data/map/x2_dev/2231"


def resolve_map_osm(override: str | Path | None) -> Path | None:
    """三状態モデルで lanelet2_map.osm のパスを解決する。

    Args:
        override:
          - None: 後方互換フォールバック (`_DEFAULT_MAP_DIR` を glob)
          - 空文字: 明示的に「地図なし」
          - パス文字列: そのパスを使用 (存在しなければ警告して None)

    Returns:
        解決された OSM パス、または None。
    """
    if override is None:
        if not _DEFAULT_MAP_DIR.exists():
            return None
        candidates = sorted(
            _DEFAULT_MAP_DIR.glob("*/lanelet2_map.osm"),
            key=lambda p: p.parent.name,
            reverse=True,
        )
        return candidates[0] if candidates else None
    s = str(override)
    if s == "":
        return None
    p = Path(s)
    if not p.exists():
        warnings.warn(f"map_osm パスが存在しません: {p}", stacklevel=2)
        return None
    return p


def load_map_ways(osm_path: Path) -> list[np.ndarray]:
    """OSM をパースし、lanelet relation で使われている way の (x, y) 座標列リストを返す。"""
    osm_path = Path(osm_path)
    tree = etree.parse(str(osm_path))
    root = tree.getroot()

    node_xy: dict[int, tuple[float, float]] = {}
    for node in root.findall("node"):
        nid = int(node.get("id"))
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            node_xy[nid] = (float(tags["local_x"]), float(tags["local_y"]))

    way_coords: dict[int, np.ndarray] = {}
    for way in root.findall("way"):
        wid = int(way.get("id"))
        refs = [int(nd.get("ref")) for nd in way.findall("nd")]
        pts = [node_xy[r] for r in refs if r in node_xy]
        if len(pts) >= 2:
            way_coords[wid] = np.array(pts)

    used_way_ids: set[int] = set()
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "lanelet":
            for member in rel.findall("member"):
                if member.get("type") == "way":
                    used_way_ids.add(int(member.get("ref")))

    return [way_coords[wid] for wid in used_way_ids if wid in way_coords]


def map_ways_in_bbox(
    map_ways: list[np.ndarray],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
) -> list[np.ndarray]:
    """`x_range` × `y_range` の bbox と交差する way だけを返すヘルパー。"""
    x_min, x_max = x_range
    y_min, y_max = y_range
    result = []
    for pts in map_ways:
        wx, wy = pts[:, 0], pts[:, 1]
        if wx.max() < x_min or wx.min() > x_max:
            continue
        if wy.max() < y_min or wy.min() > y_max:
            continue
        result.append(pts)
    return result
