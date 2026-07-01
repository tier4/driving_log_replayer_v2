"""k_us(v) 速度帯プロファイル (ROS 非依存・純粋関数).

`lib._physical_validity` (MCAP 読み込みを伴う、rosbag2_py 依存) と
`lib._figures._physical_validity` (plotly 描画のみ、notebook のベア kernel からも import 可)
の両方が使う共通ロジックをここに集約する。numpy と dict のみに依存し、
`lib._io` (rosbag2_py 依存) は絶対に import しないこと
(`lib/_figures/_common.py` の ROS 非依存 invariant を守るため)。
"""

from __future__ import annotations

import numpy as np

VX_EDGES = np.array([0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0])


def _resolve_kus_bands(params: dict) -> tuple[list | None, list | None]:
    """params から k_us 速度帯 (bands, thresholds) を解決する。"""
    return params.get("k_us_bands"), params.get("k_us_thresholds")


def _kus_step_profile(vx: np.ndarray, params: dict) -> np.ndarray:
    """params から N 段ステップの k_us プロファイルを計算して返す。"""
    bands, thresholds = _resolve_kus_bands(params)

    if bands is not None and thresholds is not None and len(bands) > 0:
        result = np.full_like(vx, bands[-1], dtype=float)
        for i, thr in enumerate(thresholds):
            result = np.where(vx < thr, bands[i], result)
        return result

    # レガシー: 単一 k_us (ランプなし)
    k_us = params.get("k_us", 0.0)
    return np.full_like(vx, k_us, dtype=float)


def _kus_band_label(params: dict) -> str:
    """速度帯パラメータを凡例文字列に変換。"""
    bands, thresholds = _resolve_kus_bands(params)

    if bands is not None and thresholds is not None:
        parts = []
        for i, b in enumerate(bands):
            if i < len(thresholds):
                parts.append(f"<{thresholds[i]:.1f}m/s: {b:.4f}")
            else:
                parts.append(f"≥{thresholds[-1]:.1f}m/s: {b:.4f}")
        return " | ".join(parts)

    return f"k_us={params.get('k_us', 0.0):.4f}"
