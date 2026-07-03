"""real_log_sim_comparison pipeline の入力バリデーション共通部品。

目的:
- 欠損トピック、空 DataFrame、必須列、時刻対応付けの検証を 1 箇所に集約する。
- gear だけを特別扱いせず、他の必須トピックにも同じ fail-fast 方針を適用できるようにする。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
import pandas as pd


class DataValidationError(ValueError):
    """pipeline 入力データが処理の前提を満たさない。"""


class MissingRequiredDataError(DataValidationError):
    """必須トピック・必須系列・必須時刻範囲が欠損している。"""


class MissingRequiredGearError(MissingRequiredDataError):
    """必須の gear_status が欠損している、または評価時刻範囲を覆えない。"""


@dataclass(frozen=True)
class RequiredTopic:
    topic: str
    reason: str


REQUIRED_REAL_LITE_TOPICS: tuple[RequiredTopic, ...] = (
    RequiredTopic("/system/operation_mode/state", "AUTONOMOUS 区間の切り出し"),
    RequiredTopic("/vehicle/status/velocity_status", "速度応答"),
    RequiredTopic("/vehicle/status/steering_status", "操舵応答"),
    RequiredTopic("/vehicle/status/gear_status", "DRIVE 系区間だけを同定・評価に使うため"),
    RequiredTopic("/localization/kinematic_state", "自車位置・姿勢・yaw rate"),
    RequiredTopic("/localization/acceleration", "縦加速度応答"),
    RequiredTopic("/control/command/control_cmd", "post-gate 制御指令"),
)


REQUIRED_SIM_LITE_TOPICS: tuple[RequiredTopic, ...] = (
    RequiredTopic("/system/operation_mode/state", "AUTONOMOUS 区間の切り出し"),
    RequiredTopic("/vehicle/status/velocity_status", "速度応答"),
    RequiredTopic("/vehicle/status/steering_status", "操舵応答"),
    RequiredTopic("/vehicle/status/gear_status", "DRIVE 系区間だけを同定・評価に使うため"),
    RequiredTopic("/localization/kinematic_state", "自車位置・姿勢・yaw rate"),
    RequiredTopic("/localization/acceleration", "縦加速度応答"),
    RequiredTopic("/control/command/control_cmd", "post-gate 制御指令"),
)


def validate_required_topics(
    present_topics: set[str],
    requirements: list[RequiredTopic],
    *,
    context: str,
    regeneration_hint: str = "入力 bag / lite bag を再生成してください",
) -> None:
    """必須トピックが存在することを検証する。"""
    missing = [req for req in requirements if req.topic not in present_topics]
    if not missing:
        return
    lines = [f"{context}: 必須トピックが欠損しています:"]
    lines.extend(f"  - {req.topic}: {req.reason}" for req in missing)
    lines.append(regeneration_hint)
    raise MissingRequiredDataError("\n".join(lines))


def require_non_empty_df(df: pd.DataFrame, *, name: str, context: str) -> None:
    """DataFrame が空でないことを検証する。"""
    if df.empty:
        raise MissingRequiredDataError(f"{context}: {name} が空です")


def require_columns(df: pd.DataFrame, columns: list[str], *, name: str, context: str) -> None:
    """DataFrame が必須列を持つことを検証する。"""
    missing = [c for c in columns if c not in df.columns]
    if missing:
        raise DataValidationError(f"{context}: {name} に必須列がありません: {missing}")


def require_sample_count(mask: np.ndarray | pd.Series, min_count: int, *, name: str, context: str) -> None:
    """bool mask の True 数が閾値以上であることを検証する。"""
    n = int(np.asarray(mask, dtype=bool).sum())
    if n < min_count:
        raise MissingRequiredDataError(f"{context}: {name} の有効サンプル数が不足しています ({n} < {min_count})")


def asof_values(
    df: pd.DataFrame,
    target_t_ns: np.ndarray | pd.Series,
    *,
    value_col: str,
    name: str,
    context: str,
    missing_error: type[MissingRequiredDataError] = MissingRequiredDataError,
) -> np.ndarray:
    """target 時刻ごとの直前値を返す。時刻範囲を覆えない場合は例外。"""
    require_non_empty_df(df, name=name, context=context)
    require_columns(df, ["t_ns", value_col], name=name, context=context)

    target = np.asarray(target_t_ns, dtype=np.int64)
    if target.size == 0:
        return np.empty(0)

    src = (
        df[["t_ns", value_col]]
        .dropna()
        .sort_values("t_ns")
        .drop_duplicates("t_ns", keep="last")
    )
    if src.empty:
        raise missing_error(f"{context}: {name} が有効サンプルを持ちません")

    t_src = src["t_ns"].to_numpy(dtype=np.int64)
    values = src[value_col].to_numpy()
    idx = np.searchsorted(t_src, target, side="right") - 1
    if np.any(idx < 0):
        raise missing_error(f"{context}: {name} が target 時刻範囲の先頭を覆っていません")

    return values[idx]


def require_asof_mask(
    df: pd.DataFrame,
    target_t_ns: np.ndarray | pd.Series,
    *,
    value_col: str,
    predicate: Callable[[np.ndarray], np.ndarray],
    name: str,
    context: str,
    missing_error: type[MissingRequiredDataError] = MissingRequiredDataError,
) -> np.ndarray:
    """target 時刻ごとの直前値に predicate を適用し bool mask を返す。"""
    values = asof_values(
        df,
        target_t_ns,
        value_col=value_col,
        name=name,
        context=context,
        missing_error=missing_error,
    )
    return np.asarray(predicate(values), dtype=bool)
