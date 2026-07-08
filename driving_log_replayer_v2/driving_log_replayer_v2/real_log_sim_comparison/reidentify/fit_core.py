"""一次遅れ + 純粋遅延モデルの同定カーネル (SSOT の re-export)。

数値カーネル本体は ``lib/_fit_core.py`` (reidentify 以外のパイプラインとも
共有される SSOT、外部依存は numpy/scipy のみ) にある。ここでは複製せず
そのまま re-export することで、同定式が新旧パイプラインで乖離することを防ぐ。
"""
from __future__ import annotations

from ..lib._fit_core import (
    delay_shift,
    delay_shift_frac,
    build_first_order_residual_datasets,
    equation_residual_at_params,
    fit_first_order_delay,
    fit_first_order_delay_residual_3phase,
    savgol_derivative,
    savgol_smooth,
    sim_first_order,
    sim_first_order_frac,
)

__all__ = [
    "delay_shift",
    "delay_shift_frac",
    "build_first_order_residual_datasets",
    "equation_residual_at_params",
    "fit_first_order_delay",
    "fit_first_order_delay_residual_3phase",
    "savgol_derivative",
    "savgol_smooth",
    "sim_first_order",
    "sim_first_order_frac",
]
