"""First-order response fit used by longitudinal and steering identification."""
from __future__ import annotations

from collections.abc import Iterable

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter


def _delay(cmd: np.ndarray, samples: int) -> np.ndarray:
    if samples <= 0:
        return cmd.copy()
    shifted = np.empty(len(cmd), dtype=float)
    shifted[:samples] = cmd[0]
    shifted[samples:] = cmd[:-samples]
    return shifted


def _simulate(cmd: np.ndarray, tau: float, delay_samples: int, dt: float) -> np.ndarray:
    delayed = _delay(cmd, delay_samples)
    alpha = float(np.clip(dt / tau, 0.0, 1.0)) if tau > 0.0 else 1.0
    return lfilter([alpha], [1.0, -(1.0 - alpha)], delayed)


def fit_first_order_delay(
    cmd: np.ndarray,
    actual: np.ndarray,
    mask: np.ndarray,
    dt: float,
    *,
    tau_bounds: tuple[float, float],
    delay_candidates: Iterable[float],
    fit_scale: bool = False,
    scale_bounds: tuple[float, float] = (0.8, 1.2),
) -> dict[str, float] | None:
    """Fit delay, time constant and optional output scale by masked MSE."""
    if len(cmd) == 0 or len(actual) != len(cmd) or not np.any(mask):
        return None
    log_bounds = tuple(float(np.log(bound)) for bound in tau_bounds)

    def evaluate(tau: float, delay_samples: int) -> tuple[float, float]:
        simulated = _simulate(cmd, tau, delay_samples, dt)
        scale = 1.0
        if fit_scale:
            energy = float(np.sum(simulated[mask] ** 2))
            if energy > 1.0e-5:
                projection = np.sum(simulated[mask] * actual[mask]) / energy
                scale = float(np.clip(projection, *scale_bounds))
        mse = float(np.mean((scale * simulated[mask] - actual[mask]) ** 2))
        return mse, scale

    best: tuple[float, float, float, float] | None = None
    for delay in delay_candidates:
        delay = float(delay)
        delay_samples = int(round(delay / dt))
        result = minimize_scalar(
            lambda log_tau: evaluate(float(np.exp(log_tau)), delay_samples)[0],
            bounds=log_bounds,
            method="bounded",
        )
        tau = float(np.exp(result.x))
        mse, scale = evaluate(tau, delay_samples)
        if best is None or mse < best[0]:
            best = (mse, tau, delay, scale)

    if best is None:
        return None
    mse, tau, delay, scale = best
    return {
        "tau": tau,
        "delay": delay,
        "scale": scale,
        "rmse": float(np.sqrt(mse)),
        "mse": mse,
    }
