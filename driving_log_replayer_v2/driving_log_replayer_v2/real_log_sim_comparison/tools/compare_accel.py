#!/usr/bin/env python3
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.signal import correlate

def analyze_and_plot(csv_path: Path, output_png: Path):
    df = pd.read_csv(csv_path)
    
    # Extract topics
    df_accel = df[df["topic"] == "accel"][["t_ns", "accel"]].dropna().sort_values("t_ns").reset_index(drop=True)
    df_vel = df[df["topic"] == "velocity"][["t_ns", "lon_vel"]].dropna().sort_values("t_ns").reset_index(drop=True)
    df_kin = df[df["topic"] == "kinematic"][["t_ns", "x", "y", "yaw", "vx"]].dropna().sort_values("t_ns").reset_index(drop=True)
    df_cmd = df[df["topic"] == "cmd"][["t_ns", "cmd_accel"]].dropna().sort_values("t_ns").reset_index(drop=True)
    
    if df_accel.empty or df_vel.empty or df_kin.empty:
        print(f"Error: Missing required topics in {csv_path}")
        return
        
    t0_ns = min(df_accel["t_ns"].iloc[0], df_vel["t_ns"].iloc[0], df_kin["t_ns"].iloc[0])
    
    df_accel["t_s"] = (df_accel["t_ns"] - t0_ns) * 1e-9
    df_vel["t_s"] = (df_vel["t_ns"] - t0_ns) * 1e-9
    df_kin["t_s"] = (df_kin["t_ns"] - t0_ns) * 1e-9
    df_cmd["t_s"] = (df_cmd["t_ns"] - t0_ns) * 1e-9
    
    # 1. Differential of /vehicle/status/velocity_status (lon_vel)
    dt_vel = df_vel["t_s"].diff()
    df_vel["accel_diff"] = df_vel["lon_vel"].diff() / dt_vel
    df_vel["accel_diff_lpf"] = df_vel["accel_diff"].rolling(window=10, min_periods=1, center=True).mean()
    
    # 2. Differential of /localization/kinematic_state (vx)
    dt_kin = df_kin["t_s"].diff()
    df_kin["accel_diff"] = df_kin["vx"].diff() / dt_kin
    df_kin["accel_diff_lpf"] = df_kin["accel_diff"].rolling(window=10, min_periods=1, center=True).mean()
    
    # 3. Double differential of position (x, y) from kinematic_state
    # Compute velocity vector from x, y
    dx = df_kin["x"].diff()
    dy = df_kin["y"].diff()
    vx_pos = dx / dt_kin
    vy_pos = dy / dt_kin
    
    # Project to body longitudinal direction using yaw
    cos_yaw = np.cos(df_kin["yaw"])
    sin_yaw = np.sin(df_kin["yaw"])
    v_lon_pos = vx_pos * cos_yaw + vy_pos * sin_yaw
    
    # Differentiate longitudinal velocity to get longitudinal acceleration
    df_kin["accel_pos_2diff"] = v_lon_pos.diff() / dt_kin
    
    # Position double diff is extremely noisy, apply a larger window moving average
    df_kin["accel_pos_2diff_lpf"] = df_kin["accel_pos_2diff"].rolling(window=15, min_periods=1, center=True).mean()
    
    # Compute cross-correlation lag for EKF vx diff LPF
    t_min = max(df_accel["t_s"].iloc[0], df_vel["t_s"].iloc[0], df_kin["t_s"].iloc[0])
    t_max = min(df_accel["t_s"].iloc[-1], df_vel["t_s"].iloc[-1], df_kin["t_s"].iloc[-1])
    dt_resample = 0.01
    t_grid = np.arange(t_min, t_max, dt_resample)
    
    accel_orig_r = np.interp(t_grid, df_accel["t_s"].values, df_accel["accel"].values)
    accel_kin_diff_lpf_r = np.interp(t_grid, df_kin["t_s"].values[1:], df_kin["accel_diff_lpf"].values[1:])
    
    a_orig_norm = accel_orig_r - np.mean(accel_orig_r)
    a_kin_norm = accel_kin_diff_lpf_r - np.mean(accel_kin_diff_lpf_r)
    corr_kin = correlate(a_orig_norm, a_kin_norm, mode='full')
    lags = np.arange(-len(a_orig_norm) + 1, len(a_orig_norm))
    lag_kin_s = lags[np.argmax(corr_kin)] * dt_resample
    
    print(f"Computed lag (/localization/acceleration vs kinematic_state vx diff): {lag_kin_s:.3f} s")
    
    # Plotting
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    
    # Plot velocities
    axes[0].plot(df_vel["t_s"].values, df_vel["lon_vel"].values, label="velocity_status (lon_vel)", color="blue")
    axes[0].plot(df_kin["t_s"].values, df_kin["vx"].values, label="kinematic_state (vx)", color="green", linestyle="--")
    axes[0].plot(df_kin["t_s"].values, v_lon_pos.values, label="kinematic position 1st diff (v_lon)", color="orange", linestyle=":")
    axes[0].set_ylabel("Velocity [m/s]")
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_title(f"Velocity Profiles Comparison - {csv_path.parent.name}")
    
    # Plot original acceleration
    axes[1].plot(df_accel["t_s"].values, df_accel["accel"].values, label="/localization/acceleration", color="red")
    if not df_cmd.empty:
        axes[1].plot(df_cmd["t_s"].values, df_cmd["cmd_accel"].values, label="cmd_accel", color="black", linestyle=":", alpha=0.7)
    axes[1].set_ylabel("Accel [m/s^2]")
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_title("Original Acceleration and Command")
    
    # Plot differentiated accelerations (filtered)
    axes[2].plot(df_accel["t_s"].values, df_accel["accel"].values, label="/localization/acceleration", color="red", alpha=0.5)
    axes[2].plot(df_vel["t_s"].values, df_vel["accel_diff_lpf"].values, label="velocity_status diff (LPF)", color="blue")
    axes[2].plot(df_kin["t_s"].values, df_kin["accel_diff_lpf"].values, label="kinematic_state vx diff (LPF)", color="green")
    axes[2].plot(df_kin["t_s"].values, df_kin["accel_pos_2diff_lpf"].values, label="position 2nd diff (LPF)", color="orange")
    axes[2].set_ylabel("Accel [m/s^2]")
    axes[2].legend()
    axes[2].grid(True)
    axes[2].set_title("Differentiated Accelerations (Filtered)")
    
    # Plot aligned accelerations
    axes[3].plot(df_kin["t_s"].values, df_kin["accel_diff_lpf"].values, label="kinematic_state vx diff (LPF)", color="green", alpha=0.7)
    axes[3].plot(df_kin["t_s"].values, df_kin["accel_pos_2diff_lpf"].values, label="position 2nd diff (LPF)", color="orange", alpha=0.7)
    # Shifted original acceleration
    t_shifted = df_accel["t_s"].values - lag_kin_s
    axes[3].plot(t_shifted, df_accel["accel"].values, label=f"/localization/acceleration shifted (lag={lag_kin_s:.3f}s)", color="purple", linestyle="--")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("Accel [m/s^2]")
    axes[3].legend()
    axes[3].grid(True)
    axes[3].set_title("Aligned Accelerations (Original shifted to align with vx diff)")
    
    plt.tight_layout()
    plt.savefig(output_png)
    plt.close()
    print(f"Saved plot to {output_png}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare acceleration signals and compute delay.")
    parser.add_argument("--csv", type=Path, required=True, help="Path to reidentify_cache.csv")
    parser.add_argument("--out", type=Path, required=True, help="Path to output plot PNG")
    args = parser.parse_args()
    
    analyze_and_plot(args.csv, args.out)
