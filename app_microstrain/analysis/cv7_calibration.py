"""
CV7-AHRS Static Calibration & Noise Analysis
=============================================
Place the sensor perfectly still on a flat surface.
Run this script to measure:
  - Gyroscope bias (should be < 0.01 °/s)
  - Accelerometer noise floor
  - Allan Deviation (gyro drift rate)
  - Magnetometer heading stability
  - EKF convergence time

Usage:
  python cv7_calibration.py --csv path/to/cv7_20260518_XXXXXX.csv
  python cv7_calibration.py --live  (reads cv7_live.csv in real-time)

Output:
  - Calibration report printed to terminal
  - calibration_results.json saved for use in EKF config
  - Allan deviation plot saved as PNG
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except ImportError:
    HAS_PLT = False
    print("[WARN] matplotlib not found — plots disabled")


# ─── Load data ────────────────────────────────────────────────────────────────

def load_csv(path: str) -> dict:
    data = {}
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                data.setdefault(k, [])
                try:
                    data[k].append(float(v))
                except (ValueError, TypeError):
                    data[k].append(0.0)
    return {k: np.array(v) for k, v in data.items()}


# ─── Statistics ───────────────────────────────────────────────────────────────

def stats(arr: np.ndarray, label: str) -> dict:
    return {
        "label":  label,
        "mean":   float(np.mean(arr)),
        "std":    float(np.std(arr)),
        "min":    float(np.min(arr)),
        "max":    float(np.max(arr)),
        "p2p":    float(np.ptp(arr)),
        "rms":    float(np.sqrt(np.mean(arr**2))),
    }


def allan_deviation(data: np.ndarray, sample_rate: float) -> tuple:
    """Compute Allan deviation for a time series."""
    dt   = 1.0 / sample_rate
    taus, sigmas = [], []
    max_n = len(data) // 2

    for n in [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]:
        if n >= max_n:
            break
        chunks = len(data) // n
        means  = np.array([np.mean(data[i*n:(i+1)*n])
                           for i in range(chunks)])
        if len(means) < 2:
            break
        diffs  = np.diff(means)
        sigma  = math.sqrt(float(np.mean(diffs**2)) / 2.0)
        tau    = n * dt
        taus.append(tau)
        sigmas.append(sigma)

    return np.array(taus), np.array(sigmas)


# ─── Report ───────────────────────────────────────────────────────────────────

def print_section(title: str):
    print(f"\n{'═'*58}")
    print(f"  {title}")
    print('═'*58)


def print_stat(s: dict, unit: str, limit: float = None):
    ok = ""
    if limit is not None:
        ok = "  ✓" if abs(s["std"]) < limit else "  ⚠ EXCEEDS LIMIT"
    print(f"  {s['label']:<20s} "
          f"mean={s['mean']:+9.5f}  "
          f"std={s['std']:8.5f}  "
          f"p2p={s['p2p']:8.5f} {unit}{ok}")


def run_analysis(d: dict, output_dir: str = "."):

    # ── Sample rate ──────────────────────────────────────────────────────
    t     = d.get("t", np.arange(len(d.get("roll", [0]))))
    dt    = np.diff(t)
    dt    = dt[dt > 0]
    if len(dt) == 0:
        print("[ERROR] Not enough time data")
        return
    sr    = 1.0 / np.mean(dt)
    n     = len(t)
    dur   = float(t[-1] - t[0])

    print_section("DATA SUMMARY")
    print(f"  Samples:      {n}")
    print(f"  Duration:     {dur:.1f} s")
    print(f"  Sample rate:  {sr:.1f} Hz")

    # ── Gyroscope ────────────────────────────────────────────────────────
    print_section("GYROSCOPE BIAS (static — rover must be STILL)")
    print("  CV7 spec: ARW < 0.003 °/s/√Hz, bias < 0.1 °/hr")
    for ax, key in [("X", "gx"), ("Y", "gy"), ("Z", "gz")]:
        arr = d.get(key, np.zeros(n))
        s   = stats(arr, f"gyro_{ax}")
        print_stat(s, "°/s", limit=0.05)

    print(f"\n  Gyro Z bias (yaw drift): "
          f"{np.mean(d.get('gz', [0])):.6f} °/s  "
          f"= {np.mean(d.get('gz', [0]))*3600:.3f} °/hr")

    # ── Accelerometer ────────────────────────────────────────────────────
    print_section("ACCELEROMETER")
    print("  Expected: |az| ≈ 9.807 m/s² (flat), |ax|,|ay| ≈ 0")
    for ax, key in [("X", "ax"), ("Y", "ay"), ("Z", "az")]:
        arr = d.get(key, np.zeros(n))
        s   = stats(arr, f"accel_{ax}")
        print_stat(s, "m/s²")

    az   = d.get("az", np.ones(n) * 9.807)
    gtot = np.sqrt(d.get("ax", np.zeros(n))**2 +
                   d.get("ay", np.zeros(n))**2 + az**2)
    g_err = np.abs(np.mean(gtot) - 9.807)
    g_ok  = "✓" if g_err < 0.05 else "⚠"
    print(f"\n  Gravity magnitude: mean={np.mean(gtot):.4f} m/s²  "
          f"(error={g_err:.4f}) {g_ok}")

    # ── Euler angles ─────────────────────────────────────────────────────
    print_section("ORIENTATION STABILITY (static — EKF output)")
    for ax, key in [("Roll", "roll"), ("Pitch", "pitch"), ("Yaw", "yaw")]:
        arr = d.get(key, np.zeros(n))
        s   = stats(arr, ax)
        print_stat(s, "°", limit=0.1)

    # ── Quaternion norm ──────────────────────────────────────────────────
    print_section("QUATERNION NORM (should be 1.0000)")
    qnorm = d.get("qnorm", np.ones(n))
    print(f"  Mean:    {np.mean(qnorm):.6f}")
    print(f"  Std:     {np.std(qnorm):.8f}")
    print(f"  Min/Max: {np.min(qnorm):.6f} / {np.max(qnorm):.6f}")
    norm_ok = "✓ PASS" if abs(np.mean(qnorm) - 1.0) < 0.001 else "✗ FAIL"
    print(f"  Status:  {norm_ok}")

    # ── Magnetometer ─────────────────────────────────────────────────────
    print_section("MAGNETOMETER")
    mx = d.get("mx", np.zeros(n))
    my = d.get("my", np.zeros(n))
    mz = d.get("mz", np.zeros(n))
    if np.any(mx != 0):
        mag_str = np.sqrt(mx**2 + my**2 + mz**2)
        print(f"  |B| mean: {np.mean(mag_str):.5f} Gauss")
        print(f"  |B| std:  {np.std(mag_str):.5f} Gauss")
        heading = np.degrees(np.arctan2(my, mx))
        print(f"  Heading mean: {np.mean(heading):+.2f}°")
        print(f"  Heading std:  {np.std(heading):.3f}°")
    else:
        print("  No magnetometer data (enable /imu/mag topic)")

    # ── Allan Deviation ──────────────────────────────────────────────────
    print_section("ALLAN DEVIATION — Gyro Z (yaw noise)")
    gz = d.get("gz", np.zeros(n))
    taus, sigmas = allan_deviation(gz, sr)
    if len(taus) > 0:
        print(f"  {'τ (s)':>10s}  {'σ (°/s)':>12s}  {'σ (°/hr)':>12s}")
        for tau, sigma in zip(taus, sigmas):
            print(f"  {tau:10.3f}  {sigma:12.6f}  {sigma*3600:12.4f}")
        print("  (CV7 spec: ARW ≈ 0.003°/s/√Hz ≈ 0.15°/√hr)")

        if HAS_PLT:
            fig, ax = plt.subplots(figsize=(8, 5),
                                   facecolor="#0D1117")
            ax.set_facecolor("#161B22")
            ax.loglog(taus, sigmas, "o-", color="#58A6FF",
                      linewidth=2, markersize=6, label="Measured")
            ax.loglog(taus, sigmas[0]/np.sqrt(taus/taus[0]),
                      "--", color="#3FB950", alpha=0.5,
                      label="ARW slope (-½)")
            ax.set_xlabel("τ (s)", color="#8B949E")
            ax.set_ylabel("σ (°/s)", color="#8B949E")
            ax.set_title("Allan Deviation — Gyro Z (Yaw)",
                         color="#E6EDF3", fontsize=12)
            ax.legend(facecolor="#161B22", labelcolor="#E6EDF3")
            ax.tick_params(colors="#8B949E")
            ax.grid(True, color="#30363D", alpha=0.5, which="both")
            out = os.path.join(output_dir, "allan_deviation.png")
            fig.savefig(out, dpi=150, bbox_inches="tight",
                        facecolor="#0D1117")
            print(f"\n  Allan deviation plot saved: {out}")

    # ── EKF covariance recommendations ──────────────────────────────────
    print_section("RECOMMENDED EKF COVARIANCES (for robot_localization)")

    gyro_var   = float(np.var(d.get("gz", [0.001])))
    accel_var  = float(np.var(d.get("az", [0.001])))
    orient_var = float(np.var(d.get("yaw", [0.001])))

    rec = {
        "imu_orientation_cov": [orient_var, 0, 0,
                                 0, orient_var, 0,
                                 0, 0, orient_var],
        "imu_gyro_cov":        [gyro_var, 0, 0,
                                 0, gyro_var, 0,
                                 0, 0, gyro_var],
        "imu_accel_cov":       [accel_var, 0, 0,
                                 0, accel_var, 0,
                                 0, 0, accel_var],
    }

    print(f"  imu_orientation_cov: [{orient_var:.6f}, ...]")
    print(f"  imu_gyro_cov:        [{gyro_var:.8f}, ...]")
    print(f"  imu_accel_cov:       [{accel_var:.6f}, ...]")

    out_json = os.path.join(output_dir, "calibration_results.json")
    with open(out_json, "w") as jf:
        json.dump({
            "sample_rate_hz": float(sr),
            "duration_s":     dur,
            "n_samples":      n,
            "gyro_bias_dps":  {
                "x": float(np.mean(d.get("gx", [0]))),
                "y": float(np.mean(d.get("gy", [0]))),
                "z": float(np.mean(d.get("gz", [0]))),
            },
            "gravity_error_ms2": g_err,
            "quaternion_norm_mean": float(np.mean(qnorm)),
            "ekf_covariances": rec,
        }, jf, indent=2)
    print(f"\n  Full results saved: {out_json}")
    print("═"*58)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="CV7-AHRS Static Calibration Analysis")
    parser.add_argument("--csv",  default=None,
                        help="Path to recorded CSV")
    parser.add_argument("--live", action="store_true",
                        help="Analyze live CSV after 120s collection")
    parser.add_argument("--duration", type=int, default=120,
                        help="Collection duration for --live (seconds)")
    args = parser.parse_args()

    if args.live:
        live = str(Path(__file__).parent.parent /
                   "microstrain_inertial" / "imu_logs" / "cv7_live.csv")
        print(f"Collecting live data for {args.duration}s...")
        print("  Keep the rover PERFECTLY STILL!")
        print(f"  Reading from: {live}")
        time.sleep(args.duration)
        csv_path = live
    elif args.csv:
        csv_path = args.csv
    else:
        # Look for most recent CSV in imu_logs
        log_dir = (Path(__file__).parent.parent /
                   "microstrain_inertial" / "imu_logs")
        csvs = sorted(log_dir.glob("cv7_2*.csv"))
        if not csvs:
            print("[ERROR] No CSV found. Run the logger first.")
            sys.exit(1)
        csv_path = str(csvs[-1])
        print(f"Using most recent log: {csv_path}")

    print(f"\nAnalyzing: {csv_path}")
    d = load_csv(csv_path)
    output_dir = str(Path(csv_path).parent)
    run_analysis(d, output_dir)


if __name__ == "__main__":
    main()
