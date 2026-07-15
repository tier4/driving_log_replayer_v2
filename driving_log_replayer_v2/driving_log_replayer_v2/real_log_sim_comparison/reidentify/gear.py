"""ROS に依存しない DRIVE 系 gear 判定。"""
from __future__ import annotations

import numpy as np
import pandas as pd

from ..lib._validation import MissingRequiredGearError, require_asof_mask
from .csv_schema import SIGNAL_TOPICS

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
            name=SIGNAL_TOPICS["gear"],
            context=context,
            missing_error=MissingRequiredGearError,
            leading_gap_fill=leading_gap_fill,
        )
    except MissingRequiredGearError as exc:
        raise MissingRequiredGearError(f"{exc}. input_bag を確認し、キャッシュを再生成してください") from exc
