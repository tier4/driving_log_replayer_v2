"""Physical-validity calculations on the current reidentify cache contract.

The old report consumed ``real.lite`` MCAPs directly.  This module deliberately
works on the resampled dictionaries returned by :mod:`reidentify.load_data`,
so the numerical definitions remain usable without ROS or rosbag imports.
"""
from __future__ import annotations

from collections.abc import Iterable

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.signal import savgol_filter

from ..reidentify import fit_core

_FIT_DT = 0.01
_FIT_DT_NS = _FIT_DT * 1e9
WHEELBASE = 4.76012
K_US_CLIP = 0.5
_XY_HEADING_RATE_COEFF_BOUNDS = (-5.0, 5.0)


def _finite_mask(*arrays: np.ndarray) -> np.ndarray:
    mask = np.ones(len(arrays[0]), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(array)
    return mask


def _savgol_derivative(values: np.ndarray, dt: float = _FIT_DT) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if len(values) < 5:
        return np.gradient(values, dt) if len(values) > 1 else np.zeros_like(values)
    window = min(len(values) if len(values) % 2 else len(values) - 1, 21)
    window = max(window, 5)
    return savgol_filter(values, window, 2, deriv=1, delta=dt, mode="interp")


def _simulate_first_order(cmd: np.ndarray, tau: float, delay: float, dt: float) -> np.ndarray:
    delayed = np.asarray(cmd, dtype=float).copy()
    samples = max(0, int(round(float(delay) / dt)))
    if samples:
        delayed[samples:] = delayed[:-samples]
        delayed[:samples] = delayed[0]
    alpha = min(1.0, dt / max(float(tau), 1e-9))
    out = np.empty_like(delayed)
    if len(out) == 0:
        return out
    out[0] = alpha * delayed[0]
    for i in range(1, len(out)):
        out[i] = out[i - 1] + alpha * (delayed[i] - out[i - 1])
    return out


def _fit_response(cmd: np.ndarray, actual: np.ndarray, mask: np.ndarray,
                  *, tau_bounds: tuple[float, float], delays: Iterable[float]) -> dict | None:
    mask = np.asarray(mask, dtype=bool) & _finite_mask(np.asarray(cmd), np.asarray(actual))
    if int(mask.sum()) < 5:
        return None
    best: tuple[float, float, float] | None = None
    for delay in delays:
        result = minimize_scalar(
            lambda tau: float(np.mean((_simulate_first_order(cmd, tau, delay, _FIT_DT)[mask] - actual[mask]) ** 2)),
            bounds=tau_bounds, method="bounded",
        )
        if result.success and (best is None or result.fun < best[0]):
            best = (float(result.fun), float(result.x), float(delay))
    if best is None:
        return None
    mse, tau, delay = best
    return {"tau": tau, "delay": delay, "rmse": float(np.sqrt(mse)), "n": int(mask.sum())}


def fit_longitudinal(dataset: dict) -> dict | None:
    """Fit the longitudinal first-order response for one resampled dataset."""
    cmd, actual = dataset["a_cmd"], dataset["a_act"]
    mask = dataset["gear_drive"] & (dataset["vx"] > 0.5) & (np.abs(np.gradient(cmd, _FIT_DT)) > 0.15)
    return _fit_response(cmd, actual, mask, tau_bounds=(0.01, 5.0), delays=np.arange(0.0, 0.301, 0.01))


def fit_steering(dataset: dict) -> dict | None:
    """Fit steering response after removing the measured straight-line bias."""
    cmd, actual = dataset["d_cmd"], dataset["d_act"]
    straight = dataset["gear_drive"] & (dataset["vx"] > 3.0) & (np.abs(dataset["wz"]) < 0.005)
    bias = float(np.mean(actual[straight])) if int(straight.sum()) >= 20 else 0.0
    mask = dataset["gear_drive"] & (dataset["vx"] > 0.5) & (np.abs(np.gradient(cmd, _FIT_DT)) > 0.1)
    result = _fit_response(cmd, actual - bias, mask, tau_bounds=(0.01, 2.0), delays=np.arange(0.0, 0.151, 0.01))
    if result is not None:
        result["bias"] = bias
    return result


def yaw_residual(dataset: dict, *, k_us: float, wheelbase: float = WHEELBASE) -> np.ndarray:
    vx, wz, steer = dataset["vx"], dataset["wz"], dataset["d_act"]
    rhs = vx * np.tan(np.clip(steer, -0.8, 0.8)) / (wheelbase + k_us * vx * vx)
    mask = dataset["gear_drive"] & (vx > 1.5) & np.isfinite(rhs) & np.isfinite(wz)
    return (rhs - wz)[mask]


def fit_k_us(datasets: list[dict], *, wheelbase: float = WHEELBASE) -> dict:
    def objective(k: float) -> float:
        values = [yaw_residual(ds, k_us=k, wheelbase=wheelbase) for ds in datasets]
        joined = np.concatenate([v for v in values if len(v)]) if any(len(v) for v in values) else np.empty(0)
        return float(np.mean(joined * joined)) if len(joined) else float("inf")
    result = minimize_scalar(objective, bounds=(0.0, K_US_CLIP), method="bounded")
    k = float(np.clip(result.x, 0.0, K_US_CLIP)) if result.success else 0.0
    parts = [yaw_residual(ds, k_us=k, wheelbase=wheelbase) for ds in datasets]
    parts = [part for part in parts if len(part)]
    residual = np.concatenate(parts) if parts else np.empty(0)
    return {"k_us": k, "rmse": float(np.sqrt(np.mean(residual ** 2))) if len(residual) else float("nan"), "n": int(len(residual))}


def _xy_arrays(dataset: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if "xy" in dataset:
        x, y, yaw, vx, wz = (np.asarray(value, dtype=float) for value in dataset["xy"])
        return np.arange(len(x), dtype=float) * _FIT_DT_NS, x, y, yaw, vx, wz
    kin = dataset["kin"]
    t = kin["t_ns"].to_numpy(dtype=float)
    x = kin["x"].to_numpy(dtype=float)
    y = kin["y"].to_numpy(dtype=float)
    yaw = kin["yaw"].to_numpy(dtype=float)
    vx = kin["vx"].to_numpy(dtype=float)
    wz = kin["wz"].to_numpy(dtype=float)
    return t, x, y, yaw, vx, wz


def xy_residual(dataset: dict, coeff: float) -> tuple[np.ndarray, np.ndarray]:
    _t, x, y, yaw, vx, wz = _xy_arrays(dataset)
    lhs_x, lhs_y = _savgol_derivative(x), _savgol_derivative(y)
    effective_yaw = yaw - coeff * vx * wz
    mask = dataset["gear_drive"][:len(x)] & (vx > 0.5) & _finite_mask(lhs_x, lhs_y, yaw, vx, wz)
    return (vx * np.cos(effective_yaw) - lhs_x)[mask], (vx * np.sin(effective_yaw) - lhs_y)[mask]


def fit_xy_heading_rate_coeff(datasets: list[dict], initial: float = 0.0) -> dict:
    def objective(coeff: float) -> float:
        parts = [np.concatenate(xy_residual(ds, coeff)) for ds in datasets]
        values = np.concatenate([part for part in parts if len(part)]) if any(len(p) for p in parts) else np.empty(0)
        return float(np.mean(values * values)) if len(values) else float("inf")
    result = minimize_scalar(objective, bounds=_XY_HEADING_RATE_COEFF_BOUNDS, method="bounded")
    coeff = float(result.x) if result.success else float(initial)
    parts = [np.concatenate(xy_residual(ds, coeff)) for ds in datasets]
    values = np.concatenate([part for part in parts if len(part)]) if any(len(p) for p in parts) else np.empty(0)
    return {"xy_heading_rate_coeff": coeff, "rmse": float(np.sqrt(np.mean(values ** 2))) if len(values) else float("nan"), "n": int(len(values))}


def analyze_dataset(dataset: dict, *, wheelbase: float = WHEELBASE) -> dict:
    long_fit = fit_longitudinal(dataset)
    steer_fit = fit_steering(dataset)
    return {
        "longitudinal": long_fit,
        "steering": steer_fit,
        "yaw_rmse": float(np.sqrt(np.mean(yaw_residual(dataset, k_us=0.0, wheelbase=wheelbase) ** 2))) if len(yaw_residual(dataset, k_us=0.0, wheelbase=wheelbase)) else float("nan"),
    }


def _merge_contiguous_timeseries_rows(rows: list[dict]) -> list[dict]:
    """Merge rows whose timestamps touch, preserving all compatible series."""
    if not rows:
        return []
    ordered = sorted(rows, key=lambda row: (float(row.get("_t0_ns", 0.0)), str(row.get("dataset_id", ""))))
    result: list[dict] = []
    for source in ordered:
        row = dict(source)
        row["dataset_ids"] = [str(row.get("dataset_id", row.get("label", "dataset")))]
        if result:
            previous = result[-1]
            expected = float(previous.get("_t0_ns", 0.0)) + len(previous.get("t", [])) * _FIT_DT_NS
            actual = float(row.get("_t0_ns", 0.0))
            if abs(actual - expected) <= _FIT_DT_NS * 0.1 or abs(actual - float(previous.get("_t0_ns", 0.0))) <= _FIT_DT_NS * 0.1:
                if abs(actual - float(previous.get("_t0_ns", 0.0))) <= _FIT_DT_NS * 0.1:
                    row["t"] = [float(t) + len(previous.get("t", [])) * _FIT_DT for t in row.get("t", [])]
                for key, value in row.items():
                    if key in {"dataset_ids", "_t0_ns", "label", "case_tag"}:
                        continue
                    if isinstance(value, list) and isinstance(previous.get(key), list):
                        previous[key].extend(value)
                    elif isinstance(value, dict) and isinstance(previous.get(key), dict):
                        for nested, nested_value in value.items():
                            if isinstance(nested_value, list):
                                previous[key].setdefault(nested, []).extend(nested_value)
                previous["dataset_ids"].extend(row["dataset_ids"])
                previous["label"] = " + ".join(previous["dataset_ids"])
                continue
        row.setdefault("t", [])
        result.append(row)
    return result


def _pick_longest_contiguous_timeseries_row(rows: list[dict]) -> list[dict]:
    merged = _merge_contiguous_timeseries_rows(rows)
    if not merged:
        return []
    longest = max(merged, key=lambda row: len(row.get("t", [])))
    longest = dict(longest)
    longest["case_tag"] = "最長連続"
    longest["label"] = f"最長連続: {longest.get('label', '')}"
    return [longest]
