#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

plt.rcParams["lines.linewidth"] = 0.8

DT_RESAMPLE = 0.01
MAX_LAG_S = 0.50
MIN_DURATION_S = 2.0


try:
    from driving_log_replayer_v2.real_log_sim_comparison.reidentify.settings import (
        ACCEL_SAVGOL_POLYORDER,
        ACCEL_SAVGOL_WINDOW_S,
        ACCEL_SOURCE,
    )
except Exception:
    ACCEL_SOURCE = "kinematic_diff"
    ACCEL_SAVGOL_WINDOW_S = 0.2
    ACCEL_SAVGOL_POLYORDER = 2


@dataclass
class AccelSource:
    name: str
    label: str
    color: str
    accel: np.ndarray


@dataclass
class SourceMetrics:
    name: str
    label: str
    # Positive means this source is later than /localization/acceleration.
    lag_s: float
    corr: float
    accel_rmse: float
    vel_rmse_status: float
    vel_rmse_kinematic: float
    vel_drift_status: float


def _topic_df(raw: pd.DataFrame, topic: str, cols: list[str]) -> pd.DataFrame:
    sub = raw[raw["topic"] == topic][["t_ns", *cols]].dropna(how="any")
    return sub.sort_values("t_ns").reset_index(drop=True)


def _add_time(df: pd.DataFrame, t0_ns: float) -> pd.DataFrame:
    out = df.copy()
    out["t_s"] = (out["t_ns"].astype(float) - t0_ns) * 1e-9
    return out


def _rolling_mean(values: np.ndarray, window: int) -> np.ndarray:
    return (
        pd.Series(np.asarray(values, dtype=float))
        .rolling(window=window, min_periods=1, center=True)
        .mean()
        .to_numpy()
    )


def _diff_like_reidentify(t_s: np.ndarray, values: np.ndarray, *, smooth_window: int) -> np.ndarray:
    t_s = np.asarray(t_s, dtype=float)
    values = np.asarray(values, dtype=float)
    out = np.full_like(values, np.nan, dtype=float)
    if len(values) < 2:
        return out
    dt = np.diff(t_s)
    dv = np.diff(values)
    valid = np.isfinite(dt) & np.isfinite(dv) & (dt > 1e-6)
    out[0] = 0.0
    out[1:][valid] = dv[valid] / dt[valid]
    return _rolling_mean(out, smooth_window)


def _savgol_window(n: int, dt: float, window_s: float, polyorder: int) -> int | None:
    win = int(round(window_s / dt))
    if win % 2 == 0:
        win += 1
    if win <= polyorder:
        win = polyorder + 1 if (polyorder + 1) % 2 == 1 else polyorder + 2
    if win > n:
        win = n if n % 2 == 1 else n - 1
    if win <= polyorder:
        return None
    return win


def _fill_nan_linear(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    mask = np.isfinite(values)
    if np.count_nonzero(mask) == 0:
        return values.copy()
    if np.count_nonzero(mask) == len(values):
        return values.copy()
    idx = np.arange(len(values), dtype=float)
    return np.interp(idx, idx[mask], values[mask])


def _savgol_derivative_grid(
    values: np.ndarray,
    *,
    deriv: int,
    dt: float = DT_RESAMPLE,
    window_s: float = ACCEL_SAVGOL_WINDOW_S,
    polyorder: int = ACCEL_SAVGOL_POLYORDER,
) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    out = np.full_like(values, np.nan, dtype=float)
    finite = np.isfinite(values)
    if np.count_nonzero(finite) < polyorder + 2:
        return out
    filled = _fill_nan_linear(values)
    win = _savgol_window(len(filled), dt, window_s, polyorder)
    if win is None:
        return np.gradient(filled, dt) if deriv == 1 else np.gradient(np.gradient(filled, dt), dt)
    out = savgol_filter(filled, window_length=win, polyorder=polyorder, deriv=deriv, delta=dt)
    out[~finite] = np.nan
    return out


def _resample(t_s: np.ndarray, values: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    t_s = np.asarray(t_s, dtype=float)
    values = np.asarray(values, dtype=float)
    mask = np.isfinite(t_s) & np.isfinite(values)
    if np.count_nonzero(mask) < 2:
        return np.full_like(t_grid, np.nan, dtype=float)
    return np.interp(t_grid, t_s[mask], values[mask], left=np.nan, right=np.nan)


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    mask = np.isfinite(a) & np.isfinite(b)
    if np.count_nonzero(mask) == 0:
        return float("nan")
    diff = np.asarray(a)[mask] - np.asarray(b)[mask]
    return float(np.sqrt(np.mean(diff * diff)))


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    mask = np.isfinite(a) & np.isfinite(b)
    if np.count_nonzero(mask) < 3:
        return float("nan")
    aa = np.asarray(a)[mask]
    bb = np.asarray(b)[mask]
    if np.std(aa) < 1e-9 or np.std(bb) < 1e-9:
        return float("nan")
    return float(np.corrcoef(aa, bb)[0, 1])


def _best_lag_metrics(ref: np.ndarray, src: np.ndarray, dt: float) -> tuple[float, float, float]:
    ref = np.asarray(ref, dtype=float)
    src = np.asarray(src, dtype=float)
    max_shift = max(1, int(round(MAX_LAG_S / dt)))

    best_lag = 0.0
    best_corr = float("nan")
    best_abs_corr = -np.inf
    best_rmse = _rmse(ref, src)

    for shift in range(-max_shift, max_shift + 1):
        if shift > 0:
            # Positive shift means the reference acceleration topic is later than src.
            ref_seg = ref[shift:]
            src_seg = src[:-shift]
        elif shift < 0:
            ref_seg = ref[:shift]
            src_seg = src[-shift:]
        else:
            ref_seg = ref
            src_seg = src

        c = _corr(ref_seg, src_seg)
        if not np.isfinite(c):
            continue
        abs_c = abs(c)
        if abs_c > best_abs_corr:
            best_abs_corr = abs_c
            best_corr = c
            best_lag = shift * dt
            best_rmse = _rmse(ref_seg, src_seg)

    return best_lag, best_corr, best_rmse


def _integrate_accel(accel: np.ndarray, dt: float, v0: float) -> np.ndarray:
    accel = np.asarray(accel, dtype=float)
    out = np.full_like(accel, np.nan, dtype=float)
    if len(accel) == 0 or not np.isfinite(v0):
        return out
    finite = np.isfinite(accel)
    if not np.any(finite):
        return out
    filled = accel.copy()
    filled[~finite] = 0.0
    out[0] = v0
    if len(accel) > 1:
        increments = 0.5 * (filled[:-1] + filled[1:]) * dt
        out[1:] = v0 + np.cumsum(increments)
    out[~np.maximum.accumulate(finite)] = np.nan
    return out


def _finite_percentile_ylim(arrays: list[np.ndarray], pct: float = 99.0) -> tuple[float, float] | None:
    vals = []
    for arr in arrays:
        a = np.asarray(arr, dtype=float)
        vals.append(a[np.isfinite(a)])
    vals = [v for v in vals if len(v) > 0]
    if not vals:
        return None
    flat = np.concatenate(vals)
    lim = float(np.nanpercentile(np.abs(flat), pct))
    if not np.isfinite(lim) or lim <= 0:
        return None
    lim = max(0.5, lim * 1.15)
    return -lim, lim


def _fmt(value: float, digits: int = 3) -> str:
    if not np.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def _metrics_table(metrics: list[SourceMetrics]) -> str:
    lines = ["source              src_lag corr   a_rmse  int_v_rmse"]
    for m in metrics:
        marker = "*" if m.name == ACCEL_SOURCE else " "
        lines.append(
            f"{marker}{m.name:<17} {m.lag_s:+.3f}  {_fmt(m.corr):>5}  "
            f"{_fmt(m.accel_rmse):>6}  {_fmt(m.vel_rmse_status):>10}"
        )
    return "\n".join(lines)


def analyze_and_plot(csv_path: Path, output_png: Path) -> list[SourceMetrics] | None:
    try:
        raw = pd.read_csv(csv_path)
    except Exception as exc:
        print(f"Failed to read {csv_path}: {exc}")
        return None

    df_accel = _topic_df(raw, "accel", ["accel"])
    df_vel = _topic_df(raw, "velocity", ["lon_vel"])
    df_kin = _topic_df(raw, "kinematic", ["x", "y", "yaw", "vx"])
    df_cmd = _topic_df(raw, "cmd", ["cmd_accel"])

    if df_accel.empty or df_vel.empty or df_kin.empty:
        print(f"Warning: Missing required topics in {csv_path.parent.name}")
        return None

    t0_ns = min(df_accel["t_ns"].iloc[0], df_vel["t_ns"].iloc[0], df_kin["t_ns"].iloc[0])
    df_accel = _add_time(df_accel, t0_ns)
    df_vel = _add_time(df_vel, t0_ns)
    df_kin = _add_time(df_kin, t0_ns)
    df_cmd = _add_time(df_cmd, t0_ns) if not df_cmd.empty else df_cmd

    t_min = max(df_accel["t_s"].iloc[0], df_vel["t_s"].iloc[0], df_kin["t_s"].iloc[0])
    t_max = min(df_accel["t_s"].iloc[-1], df_vel["t_s"].iloc[-1], df_kin["t_s"].iloc[-1])
    if (t_max - t_min) < MIN_DURATION_S:
        print(f"Warning: Dataset {csv_path.parent.name} is too short ({t_max - t_min:.1f}s)")
        return None

    t_grid = np.arange(t_min, t_max, DT_RESAMPLE)
    if len(t_grid) < 3:
        print(f"Warning: Dataset {csv_path.parent.name} has no usable common time grid")
        return None

    t_vel = df_vel["t_s"].to_numpy(dtype=float)
    lon_vel = df_vel["lon_vel"].to_numpy(dtype=float)
    t_kin = df_kin["t_s"].to_numpy(dtype=float)
    kin_vx = df_kin["vx"].to_numpy(dtype=float)

    dt_kin = np.gradient(t_kin)
    dt_kin = np.where(np.abs(dt_kin) > 1e-6, dt_kin, np.nan)
    vx_map = np.gradient(df_kin["x"].to_numpy(dtype=float)) / dt_kin
    vy_map = np.gradient(df_kin["y"].to_numpy(dtype=float)) / dt_kin
    yaw = df_kin["yaw"].to_numpy(dtype=float)
    pose_v_lon = vx_map * np.cos(yaw) + vy_map * np.sin(yaw)
    pose_v_lon_smooth = _rolling_mean(pose_v_lon, 15)

    accel_topic = _resample(
        df_accel["t_s"].to_numpy(dtype=float),
        df_accel["accel"].to_numpy(dtype=float),
        t_grid,
    )
    vel_status_grid = _resample(t_vel, lon_vel, t_grid)
    kin_vx_grid = _resample(t_kin, kin_vx, t_grid)
    pose_v_grid = _resample(t_kin, pose_v_lon_smooth, t_grid)
    cmd_accel_grid = (
        _resample(df_cmd["t_s"].to_numpy(dtype=float), df_cmd["cmd_accel"].to_numpy(dtype=float), t_grid)
        if not df_cmd.empty
        else None
    )

    vel_diff_accel = _resample(
        t_vel,
        _diff_like_reidentify(t_vel, lon_vel, smooth_window=10),
        t_grid,
    )
    kin_diff_accel = _resample(
        t_kin,
        _diff_like_reidentify(t_kin, kin_vx, smooth_window=10),
        t_grid,
    )
    pos_2diff_accel = _resample(
        t_kin,
        _diff_like_reidentify(t_kin, pose_v_lon_smooth, smooth_window=15),
        t_grid,
    )
    vel_savgol_accel = _savgol_derivative_grid(vel_status_grid, deriv=1)
    kin_savgol_accel = _savgol_derivative_grid(kin_vx_grid, deriv=1)
    pos_savgol_accel = _savgol_derivative_grid(pose_v_grid, deriv=1)

    sources = [
        AccelSource("accel", "/localization/acceleration", "#d62728", accel_topic),
        AccelSource("kinematic_diff", "kinematic_state.vx diff+rolling", "#2ca02c", kin_diff_accel),
        AccelSource("kinematic_savgol", "kinematic_state.vx SavGol d/dt", "#006d2c", kin_savgol_accel),
        AccelSource("velocity_diff", "velocity_status.lon_vel diff+rolling", "#1f77b4", vel_diff_accel),
        AccelSource("velocity_savgol", "velocity_status.lon_vel SavGol d/dt", "#08519c", vel_savgol_accel),
        AccelSource("position_2diff", "kinematic pose diff+rolling", "#ff7f0e", pos_2diff_accel),
        AccelSource("position_savgol", "kinematic pose velocity SavGol d/dt", "#8c510a", pos_savgol_accel),
    ]

    metrics: list[SourceMetrics] = []
    for src in sources:
        ref_lag_s, corr, accel_rmse = (
            (0.0, 1.0, 0.0)
            if src.name == "accel"
            else _best_lag_metrics(accel_topic, src.accel, DT_RESAMPLE)
        )
        source_lag_s = -ref_lag_s
        v0_status = vel_status_grid[np.where(np.isfinite(vel_status_grid))[0][0]]
        v0_kin = kin_vx_grid[np.where(np.isfinite(kin_vx_grid))[0][0]]
        vel_from_accel = _integrate_accel(src.accel, DT_RESAMPLE, float(v0_status))
        vel_from_accel_kin0 = _integrate_accel(src.accel, DT_RESAMPLE, float(v0_kin))
        residual = vel_from_accel - vel_status_grid
        finite_res = residual[np.isfinite(residual)]
        drift = float(finite_res[-1] - finite_res[0]) if len(finite_res) >= 2 else float("nan")
        metrics.append(
            SourceMetrics(
                name=src.name,
                label=src.label,
                lag_s=source_lag_s,
                corr=corr,
                accel_rmse=accel_rmse,
                vel_rmse_status=_rmse(vel_from_accel, vel_status_grid),
                vel_rmse_kinematic=_rmse(vel_from_accel_kin0, kin_vx_grid),
                vel_drift_status=drift,
            )
        )

    metrics_by_name = {m.name: m for m in metrics}
    current = next((s for s in sources if s.name == ACCEL_SOURCE), sources[1])
    current_metrics = metrics_by_name.get(current.name)

    fig, axes = plt.subplots(5, 1, figsize=(13, 15), sharex=True)
    fig.suptitle(
        f"Acceleration Source Consistency - {csv_path.parent.name} "
        f"(current reidentify ACCEL_SOURCE={ACCEL_SOURCE})",
        fontsize=13,
    )

    axes[0].plot(t_grid, vel_status_grid, label="velocity_status.lon_vel", color="#1f77b4")
    axes[0].plot(t_grid, kin_vx_grid, label="kinematic_state.twist.linear.x", color="#2ca02c", linestyle="--")
    axes[0].plot(t_grid, pose_v_grid, label="kinematic pose 1st diff", color="#ff7f0e", linestyle=":")
    axes[0].set_ylabel("Velocity [m/s]")
    axes[0].set_title("Velocity consistency")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    for src in sources:
        lw = 1.3 if src.name == ACCEL_SOURCE else 0.8
        alpha = 0.95 if src.name == ACCEL_SOURCE else 0.75
        axes[1].plot(t_grid, src.accel, label=src.label, color=src.color, linewidth=lw, alpha=alpha)
    if cmd_accel_grid is not None:
        axes[1].plot(t_grid, cmd_accel_grid, label="cmd_accel", color="black", linestyle=":", alpha=0.55)
    axes[1].set_ylabel("Accel [m/s^2]")
    axes[1].set_title("Acceleration sources")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(t_grid, current.accel, label=f"{current.label} (current)", color=current.color)
    if current_metrics is not None:
        shifted_accel_topic = _resample(t_grid + current_metrics.lag_s, accel_topic, t_grid)
        axes[2].plot(
            t_grid,
            shifted_accel_topic,
            label=f"/localization/acceleration shifted by {current_metrics.lag_s:+.3f}s to align with source",
            color="#9467bd",
            linestyle="--",
        )
    axes[2].text(
        0.01,
        0.98,
        _metrics_table(metrics),
        transform=axes[2].transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=8,
        bbox={"facecolor": "white", "edgecolor": "#cccccc", "alpha": 0.88},
    )
    axes[2].set_ylabel("Accel [m/s^2]")
    axes[2].set_title("Lag/correlation against /localization/acceleration (positive src_lag = source is later)")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    integrated_by_name: dict[str, np.ndarray] = {}
    axes[3].plot(t_grid, vel_status_grid, label="velocity_status.lon_vel", color="black", linewidth=1.2)
    for src in sources:
        v0 = vel_status_grid[np.where(np.isfinite(vel_status_grid))[0][0]]
        vel_from_accel = _integrate_accel(src.accel, DT_RESAMPLE, float(v0))
        integrated_by_name[src.name] = vel_from_accel
        lw = 1.3 if src.name == ACCEL_SOURCE else 0.8
        axes[3].plot(
            t_grid,
            vel_from_accel,
            label=f"integral({src.name})",
            color=src.color,
            linewidth=lw,
            alpha=0.8,
        )
    axes[3].set_ylabel("Velocity [m/s]")
    axes[3].set_title("Velocity reconstructed by integrating each acceleration source")
    axes[3].legend(loc="upper right")
    axes[3].grid(True)

    for src in sources:
        residual = integrated_by_name[src.name] - vel_status_grid
        lw = 1.3 if src.name == ACCEL_SOURCE else 0.8
        axes[4].plot(
            t_grid,
            residual,
            label=f"{src.name}: RMSE={_fmt(metrics_by_name[src.name].vel_rmse_status)} "
            f"drift={_fmt(metrics_by_name[src.name].vel_drift_status)}",
            color=src.color,
            linewidth=lw,
            alpha=0.85,
        )
    axes[4].axhline(0.0, color="black", linewidth=0.7)
    axes[4].set_xlabel("Time [s]")
    axes[4].set_ylabel("Velocity error [m/s]")
    axes[4].set_title("Integrated velocity residual against velocity_status.lon_vel")
    axes[4].legend(loc="upper right")
    axes[4].grid(True)

    accel_ylim = _finite_percentile_ylim([s.accel for s in sources], pct=98.0)
    if accel_ylim is not None:
        axes[1].set_ylim(*accel_ylim)
        axes[2].set_ylim(*accel_ylim)
    residual_ylim = _finite_percentile_ylim(
        [integrated_by_name[s.name] - vel_status_grid for s in sources],
        pct=95.0,
    )
    if residual_ylim is not None:
        axes[4].set_ylim(*residual_ylim)

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    output_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_png, dpi=140)
    plt.close(fig)

    ranked = sorted(metrics, key=lambda m: (not np.isfinite(m.vel_rmse_status), m.vel_rmse_status))
    print(f"{csv_path.parent.name[:8]}: current={ACCEL_SOURCE}, best_integral={ranked[0].name}")
    for m in ranked:
        marker = "*" if m.name == ACCEL_SOURCE else " "
        print(
            f"  {marker}{m.name:<15} src_lag={m.lag_s:+.3f}s corr={_fmt(m.corr)} "
            f"a_rmse={_fmt(m.accel_rmse)} v_rmse={_fmt(m.vel_rmse_status)} "
            f"v_drift={_fmt(m.vel_drift_status)}"
        )
    return metrics


def _run_self_test() -> None:
    t_grid = np.arange(0.0, 10.0, DT_RESAMPLE)
    accel = 0.4 * np.sin(2.0 * np.pi * t_grid / 5.0)
    vel = _integrate_accel(accel, DT_RESAMPLE, 2.0)
    derived = _diff_like_reidentify(t_grid, vel, smooth_window=1)
    lag_s, corr, rmse = _best_lag_metrics(accel, derived, DT_RESAMPLE)
    if abs(lag_s) > DT_RESAMPLE or corr < 0.99 or rmse > 0.02:
        raise AssertionError(f"self-test failed: lag={lag_s}, corr={corr}, rmse={rmse}")
    sg = _savgol_derivative_grid(vel, deriv=1)
    sg_lag_s, sg_corr, sg_rmse = _best_lag_metrics(accel, sg, DT_RESAMPLE)
    if abs(sg_lag_s) > DT_RESAMPLE or sg_corr < 0.99 or sg_rmse > 0.02:
        raise AssertionError(f"savgol self-test failed: lag={sg_lag_s}, corr={sg_corr}, rmse={sg_rmse}")
    print("self-test passed")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare acceleration sources and integral/differential consistency.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--csv", type=Path, help="Path to a single reidentify_cache.csv")
    group.add_argument("--collection-dir", type=Path, help="Path to the collection directory containing dataset folders")
    group.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--out", type=Path, help="Path to output plot PNG (only applicable with --csv)")
    args = parser.parse_args()

    if args.self_test:
        _run_self_test()
        return

    if args.csv:
        out_png = args.out if args.out else args.csv.parent / "reidentify_topic_consistency.png"
        metrics = analyze_and_plot(args.csv, out_png)
        if metrics is None:
            raise SystemExit(1)
        print(f"Saved plot to {out_png}")
        return

    csv_paths = sorted(args.collection_dir.rglob("reidentify_cache.csv"))
    print(f"Found {len(csv_paths)} datasets with cache in {args.collection_dir}. Starting batch generation...")

    success = 0
    failed = 0
    current_lags = []
    current_vel_rmse = []

    for csv_path in csv_paths:
        out_png = csv_path.parent / "reidentify_topic_consistency.png"
        try:
            metrics = analyze_and_plot(csv_path, out_png)
            if metrics is None:
                failed += 1
                continue
            success += 1
            current_metric = next((m for m in metrics if m.name == ACCEL_SOURCE), None)
            if current_metric is not None:
                current_lags.append(current_metric.lag_s)
                current_vel_rmse.append(current_metric.vel_rmse_status)
        except Exception as exc:
            print(f"[Error] Failed to process {csv_path.parent.name}: {exc}")
            failed += 1

    print("\nBatch Generation Summary:")
    print(f"  Successfully generated: {success} plots")
    print(f"  Failed/Skipped: {failed}")
    if current_lags:
        print(
            f"  Current source ({ACCEL_SOURCE}) src_lag: "
            f"{np.mean(current_lags):.3f} s (std: {np.std(current_lags):.3f})"
        )
    if current_vel_rmse:
        print(
            f"  Current source ({ACCEL_SOURCE}) integral velocity RMSE: "
            f"{np.mean(current_vel_rmse):.3f} m/s (std: {np.std(current_vel_rmse):.3f})"
        )


if __name__ == "__main__":
    main()
