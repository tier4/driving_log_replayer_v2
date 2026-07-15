"""Small validation helpers used by the reidentify pipeline."""

from __future__ import annotations

from typing import Callable

import numpy as np
import pandas as pd


class DataValidationError(ValueError):
    """pipeline 入力データが処理の前提を満たさない。"""


class MissingRequiredDataError(DataValidationError):
    """必須トピック・必須系列・必須時刻範囲が欠損している。"""


class MissingRequiredGearError(MissingRequiredDataError):
    """必須の gear_status が欠損している、または評価時刻範囲を覆えない。"""


def require_non_empty_df(df: pd.DataFrame, *, name: str, context: str) -> None:
    """DataFrame が空でないことを検証する。"""
    if df.empty:
        raise MissingRequiredDataError(f"{context}: {name} が空です")


def require_columns(df: pd.DataFrame, columns: list[str], *, name: str, context: str) -> None:
    """DataFrame が必須列を持つことを検証する。"""
    missing = [c for c in columns if c not in df.columns]
    if missing:
        raise DataValidationError(f"{context}: {name} に必須列がありません: {missing}")


def asof_values(
    df: pd.DataFrame,
    target_t_ns: np.ndarray | pd.Series,
    *,
    value_col: str,
    name: str,
    context: str,
    missing_error: type[MissingRequiredDataError] = MissingRequiredDataError,
    leading_gap_fill: object | None = None,
) -> np.ndarray:
    """target 時刻ごとの直前値を返す。時刻範囲を覆えない場合は例外。

    leading_gap_fill が与えられた場合、source の先頭より前の target はその値で埋める。
    """
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
        if leading_gap_fill is not None:
            clipped_idx = np.clip(idx, 0, len(values) - 1)
            out = values[clipped_idx]
            out[idx < 0] = leading_gap_fill
            return out
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
    leading_gap_fill: object | None = None,
) -> np.ndarray:
    """target 時刻ごとの直前値に predicate を適用し bool mask を返す。"""
    values = asof_values(
        df,
        target_t_ns,
        value_col=value_col,
        name=name,
        context=context,
        missing_error=missing_error,
        leading_gap_fill=leading_gap_fill,
    )
    return np.asarray(predicate(values), dtype=bool)
