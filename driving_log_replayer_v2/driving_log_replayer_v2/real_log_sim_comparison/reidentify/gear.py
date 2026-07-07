"""DRIVE 系 gear 判定 (ROS フリー)。

ロジックの SSOT は `lib/_io.py` の `GEAR_*` 定数・`is_drive_gear`・
`require_drive_gear_mask` だが、`lib/_io.py` はモジュール先頭で
`rosbag2_py`/`rclpy` を import するため、そこから1関数だけ import しても
reidentify 側の ROS フリー方針が壊れてしまう。値・ロジックが乖離しないよう
そのまま複製する (autoware_vehicle_msgs/GearReport の DRIVE 系 enum 値は
安定した契約であり、乖離リスクは小さい)。
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from ..lib._validation import MissingRequiredGearError, require_asof_mask

GEAR_DRIVE = 2
GEAR_DRIVE_18 = 19


def is_drive_gear(gear: np.ndarray | pd.Series) -> np.ndarray:
    """DRIVE 系 gear (2..19) の bool mask を返す。"""
    arr = np.asarray(gear)
    return (arr >= GEAR_DRIVE) & (arr <= GEAR_DRIVE_18)


def require_drive_gear_mask(
    df_gear: pd.DataFrame,
    target_t_ns: np.ndarray | pd.Series,
    *,
    context: str,
    allow_leading_gap: bool = False,
) -> np.ndarray:
    """target 時刻ごとの gear_status を直前値で対応付け、DRIVE 系 mask を返す。"""
    try:
        leading_gap_fill = -1 if allow_leading_gap else None
        return require_asof_mask(
            df_gear,
            target_t_ns,
            value_col="gear",
            predicate=is_drive_gear,
            name="/vehicle/status/gear_status",
            context=context,
            missing_error=MissingRequiredGearError,
            leading_gap_fill=leading_gap_fill,
        )
    except MissingRequiredGearError as exc:
        raise MissingRequiredGearError(f"{exc}. real.lite を再生成してください") from exc
