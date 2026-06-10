#!/usr/bin/env python3
"""
CV7-AHRS ROS2 Logger Node
==========================
Runs INSIDE the Docker container.
Subscribes to all IMU topics and:
  - Logs everything to CSV (accessible from Windows via mounted folder)
  - Prints live statistics to terminal
  - Computes and displays Euler angles from quaternion

Usage (inside Docker container):
  source /home/microstrain/catkin_ws/install/setup.bash
  python3 /home/microstrain/catkin_ws/src/cv7_ros2_logger.py

Output CSV: /home/microstrain/catkin_ws/src/imu_logs/cv7_live.csv
           (accessible on Windows at: Navigation/microstrain_inertial/imu_logs/)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import math
import csv
import os
import time
from datetime import datetime

try:
    from microstrain_inertial_msgs.msg import HumanReadableStatus
    HAS_STATUS = True
except ImportError:
    HAS_STATUS = False


def quat_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    # Roll (x-axis)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    # Pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_norm(x, y, z, w):
    return math.sqrt(x*x + y*y + z*z + w*w)


class CV7Logger(Node):

    LOG_DIR = "/home/microstrain/catkin_ws/src/imu_logs"
    UPDATE_HZ = 10   # terminal display rate

    def __init__(self):
        super().__init__("cv7_logger")

        # ── State ────────────────────────────────────────────────────────
        self.imu_count    = 0
        self.last_imu     = {}
        self.last_mag     = {}
        self.filter_state = "unknown"
        self.t_start      = time.monotonic()
        self.t_display    = time.monotonic()

        # ── CSV setup ────────────────────────────────────────────────────
        os.makedirs(self.LOG_DIR, exist_ok=True)
        ts        = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path  = os.path.join(self.LOG_DIR, f"cv7_{ts}.csv")
        self.live = os.path.join(self.LOG_DIR, "cv7_live.csv")

        self._log_f = open(log_path, "w", newline="")
        self._csv   = csv.writer(self._log_f)
        self._csv.writerow([
            "t_monotonic", "ros_sec", "ros_nsec",
            "roll_deg", "pitch_deg", "yaw_deg",
            "qw", "qx", "qy", "qz", "qnorm",
            "ax", "ay", "az",           # m/s²
            "gx_dps", "gy_dps", "gz_dps",  # deg/s
            "mx", "my", "mz",           # Gauss
            "filter_state"
        ])

        # Live file (overwritten every row — visualizer reads this)
        self._live_f = open(self.live, "w", newline="")
        self._live   = csv.writer(self._live_f)
        self._live.writerow([
            "t", "roll", "pitch", "yaw",
            "qw", "qx", "qy", "qz", "qnorm",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "mx", "my", "mz",
            "filter_state"
        ])

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Imu, "/imu/data",
                                 self._imu_cb, 10)
        self.create_subscription(Imu, "/imu/data_raw",
                                 self._imu_raw_cb, 10)

        if HAS_STATUS:
            self.create_subscription(
                HumanReadableStatus, "/ekf/status",
                self._status_cb, 1)

        # Optional magnetometer topic
        self.create_subscription(MagneticField, "/imu/mag",
                                 self._mag_cb, 10)

        # ── Display timer ────────────────────────────────────────────────
        self.create_timer(1.0 / self.UPDATE_HZ, self._display)

        self.get_logger().info("=" * 56)
        self.get_logger().info("  CV7-AHRS Logger started")
        self.get_logger().info(f"  Logging to: {log_path}")
        self.get_logger().info(f"  Live file:  {self.live}")
        self.get_logger().info("=" * 56)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        self.imu_count += 1
        q  = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        t  = msg.header.stamp

        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        norm = quat_norm(q.x, q.y, q.z, q.w)

        mx = self.last_mag.get("x", 0.0)
        my = self.last_mag.get("y", 0.0)
        mz = self.last_mag.get("z", 0.0)

        row = [
            time.monotonic() - self.t_start,
            t.sec, t.nanosec,
            roll, pitch, yaw,
            q.w, q.x, q.y, q.z, norm,
            la.x, la.y, la.z,
            math.degrees(av.x), math.degrees(av.y), math.degrees(av.z),
            mx, my, mz,
            self.filter_state
        ]
        self._csv.writerow(row)

        # Overwrite live file
        self._live_f.seek(0)
        self._live_f.truncate()
        self._live.writerow([
            "t", "roll", "pitch", "yaw",
            "qw", "qx", "qy", "qz", "qnorm",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "mx", "my", "mz",
            "filter_state"
        ])
        self._live.writerow([
            time.monotonic() - self.t_start,
            roll, pitch, yaw,
            q.w, q.x, q.y, q.z, norm,
            la.x, la.y, la.z,
            math.degrees(av.x), math.degrees(av.y), math.degrees(av.z),
            mx, my, mz,
            self.filter_state
        ])
        self._live_f.flush()

        self.last_imu = {
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "norm": norm,
            "ax": la.x, "ay": la.y, "az": la.z,
            "gx": math.degrees(av.x),
            "gy": math.degrees(av.y),
            "gz": math.degrees(av.z),
        }

    def _imu_raw_cb(self, msg: Imu):
        pass  # raw data logged separately if needed

    def _mag_cb(self, msg: MagneticField):
        self.last_mag = {
            "x": msg.magnetic_field.x,
            "y": msg.magnetic_field.y,
            "z": msg.magnetic_field.z,
        }

    def _status_cb(self, msg):
        self.filter_state = msg.filter_state

    # ── Display ──────────────────────────────────────────────────────────

    def _display(self):
        if not self.last_imu:
            return

        now = time.monotonic()
        elapsed = now - self.t_start
        hz = self.imu_count / max(0.001, elapsed)
        d  = self.last_imu
        mx = self.last_mag.get("x", 0.0)
        my = self.last_mag.get("y", 0.0)
        mz = self.last_mag.get("z", 0.0)
        mag_strength = math.sqrt(mx**2 + my**2 + mz**2)

        # Gravity check
        accel_total = math.sqrt(d["ax"]**2 + d["ay"]**2 + d["az"]**2)
        gravity_ok  = "✓" if abs(accel_total - 9.807) < 0.5 else "⚠"
        norm_ok     = "✓" if abs(d["norm"] - 1.0) < 0.001 else "⚠"

        print("\033[2J\033[H", end="")  # clear screen
        print("╔══════════════════════════════════════════════════════╗")
        print("║          3DM-CV7-AHRS  Live Monitor                 ║")
        print("╠══════════════════════════════════════════════════════╣")
        print(f"║  Filter: {self.filter_state:<20s}  Rate: {hz:6.1f} Hz    ║")
        print(f"║  Packets: {self.imu_count:<10d}  Elapsed: {elapsed:7.1f}s        ║")
        print("╠══════════════════════════════════════════════════════╣")
        print("║  ORIENTATION (AHRS EKF)                              ║")
        print(f"║    Roll  : {d['roll']:+8.3f}°                              ║")
        print(f"║    Pitch : {d['pitch']:+8.3f}°                              ║")
        print(f"║    Yaw   : {d['yaw']:+8.3f}°  (heading)                   ║")
        print(f"║    |q|   : {d['norm']:.6f}  {norm_ok}                          ║")
        print("╠══════════════════════════════════════════════════════╣")
        print("║  ACCELEROMETER (m/s²)                                ║")
        print(f"║    X: {d['ax']:+8.4f}   Y: {d['ay']:+8.4f}   Z: {d['az']:+8.4f}  ║")
        print(f"║    |a|: {accel_total:.4f} m/s²  {gravity_ok} (expect 9.807)      ║")
        print("╠══════════════════════════════════════════════════════╣")
        print("║  GYROSCOPE (°/s) — should be ≈ 0 when still         ║")
        print(f"║    X: {d['gx']:+8.4f}   Y: {d['gy']:+8.4f}   Z: {d['gz']:+8.4f}  ║")
        print("╠══════════════════════════════════════════════════════╣")
        print("║  MAGNETOMETER (Gauss)                                ║")
        print(f"║    X: {mx:+8.5f}   Y: {my:+8.5f}   Z: {mz:+8.5f}  ║")
        print(f"║    |m|: {mag_strength:.5f} Gauss                           ║")
        print("╠══════════════════════════════════════════════════════╣")
        print(f"║  Log: imu_logs/cv7_live.csv                         ║")
        print("╚══════════════════════════════════════════════════════╝")

    def destroy_node(self):
        self._log_f.close()
        self._live_f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CV7Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\n[DONE] Logging stopped.")


if __name__ == "__main__":
    main()
