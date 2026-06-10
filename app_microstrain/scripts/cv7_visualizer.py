"""
CV7-AHRS Live Visualizer
=========================
Runs on WINDOWS — reads CSV logged by cv7_ros2_logger.py
Shows real-time:
  - 3D rotating box showing IMU orientation
  - Euler angles (roll, pitch, yaw) time series
  - Accelerometer XYZ + gravity norm
  - Gyroscope XYZ
  - Magnetometer XYZ + heading
  - Filter status dashboard

Install: pip install numpy matplotlib

Usage:
  python cv7_visualizer.py
  python cv7_visualizer.py --log path/to/cv7_live.csv
  python cv7_visualizer.py --replay path/to/cv7_20260518_123456.csv
"""

import argparse
import csv
import math
import os
import time
import sys
from collections import deque
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ─── Configuration ────────────────────────────────────────────────────────────

DEFAULT_LOG = (
    Path(__file__).parent.parent
    / "microstrain_inertial" / "imu_logs" / "cv7_live.csv"
)

WINDOW   = 200    # number of samples to show in time-series plots
FPS      = 20     # display update rate
HISTORY  = 2000   # max samples to keep in memory

# ─── Colors ───────────────────────────────────────────────────────────────────

C = {
    "bg":      "#0D1117",
    "panel":   "#161B22",
    "border":  "#30363D",
    "text":    "#E6EDF3",
    "muted":   "#8B949E",
    "green":   "#3FB950",
    "blue":    "#58A6FF",
    "amber":   "#D29922",
    "red":     "#F85149",
    "purple":  "#BC8CFF",
    "roll":    "#FF7B72",
    "pitch":   "#79C0FF",
    "yaw":     "#56D364",
    "ax":      "#FF7B72",
    "ay":      "#79C0FF",
    "az":      "#56D364",
    "gx":      "#D2A8FF",
    "gy":      "#FFB66D",
    "gz":      "#56D364",
    "mx":      "#FF7B72",
    "my":      "#79C0FF",
    "mz":      "#FFD700",
}

# ─── IMU Box geometry ─────────────────────────────────────────────────────────

def make_box(sx=1.5, sy=0.9, sz=0.3):
    """Return vertices of a box centered at origin."""
    x, y, z = sx/2, sy/2, sz/2
    return np.array([
        [-x, -y, -z], [ x, -y, -z], [ x,  y, -z], [-x,  y, -z],
        [-x, -y,  z], [ x, -y,  z], [ x,  y,  z], [-x,  y,  z],
    ], dtype=float)

BOX_FACES = [
    [0,1,2,3], [4,5,6,7],  # bottom / top
    [0,1,5,4], [2,3,7,6],  # front / back
    [0,3,7,4], [1,2,6,5],  # left / right
]

FACE_COLORS = ["#1C3A5E", "#1C3A5E", "#2D5A3D", "#2D5A3D",
               "#5A2D1C", "#5A2D1C"]
FACE_ALPHA  = 0.85


def quat_to_rot(qw, qx, qy, qz):
    """Quaternion to 3×3 rotation matrix."""
    R = np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ])
    return R


def rotate_box(verts, R):
    return (R @ verts.T).T


# ─── Data reader ──────────────────────────────────────────────────────────────

class DataReader:
    """Reads CSV produced by cv7_ros2_logger.py or cv7_test.py."""

    FIELDS = ["t", "roll", "pitch", "yaw",
              "qw", "qx", "qy", "qz", "qnorm",
              "ax", "ay", "az",
              "gx", "gy", "gz",
              "mx", "my", "mz",
              "filter_state"]

    def __init__(self, path: str, replay: bool = False):
        self.path    = path
        self.replay  = replay
        self.history = {k: deque(maxlen=HISTORY) for k in self.FIELDS}
        self.latest  = {}
        self._last_t = None

    def update(self):
        """Read all rows from CSV."""
        if not os.path.exists(self.path):
            return False
        try:
            with open(self.path, "r", newline="") as f:
                reader = csv.DictReader(f)
                rows   = list(reader)
        except Exception:
            return False

        if not rows:
            return False

        if self.replay:
            # Feed row by row for replay
            if not hasattr(self, "_ri"):
                self._ri = 0
            if self._ri < len(rows):
                self._add_row(rows[self._ri])
                self._ri += 1
        else:
            # Live mode: add all new rows
            for row in rows:
                try:
                    t = float(row.get("t", 0))
                    if self._last_t is None or t > self._last_t:
                        self._add_row(row)
                        self._last_t = t
                except ValueError:
                    pass

        if self.history["t"]:
            self.latest = {k: list(self.history[k])[-1]
                           for k in self.FIELDS}
        return True

    def _add_row(self, row: dict):
        for k in self.FIELDS:
            v = row.get(k, "")
            try:
                self.history[k].append(float(v))
            except (ValueError, TypeError):
                self.history[k].append(v if v != "" else 0.0)

    def get_series(self, key, n=WINDOW):
        vals = list(self.history[key])[-n:]
        return np.array(vals, dtype=float) if vals else np.array([0.0])

    @property
    def n(self):
        return len(self.history["t"])


# ─── Visualizer ───────────────────────────────────────────────────────────────

class CV7Visualizer:

    def __init__(self, log_path: str, replay: bool = False):
        self.reader = DataReader(log_path, replay)
        self._setup_figure()

    def _setup_figure(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(18, 10), facecolor=C["bg"])
        self.fig.canvas.manager.set_window_title(
            "BEARS Rover — 3DM-CV7-AHRS Live Monitor")

        gs = gridspec.GridSpec(
            3, 4,
            figure=self.fig,
            hspace=0.42, wspace=0.35,
            left=0.05, right=0.97,
            top=0.93, bottom=0.07
        )

        # ── 3D orientation (large, left) ─────────────────────────────────
        self.ax3d = self.fig.add_subplot(gs[:, 0], projection="3d")
        self._setup_3d()

        # ── Euler angles ─────────────────────────────────────────────────
        self.ax_euler = self.fig.add_subplot(gs[0, 1:3])
        self._style_ax(self.ax_euler, "Euler Angles (°)", ylabel="Degrees")

        # ── Accelerometer ────────────────────────────────────────────────
        self.ax_accel = self.fig.add_subplot(gs[1, 1:3])
        self._style_ax(self.ax_accel, "Accelerometer (m/s²)", ylabel="m/s²")

        # ── Gyroscope ────────────────────────────────────────────────────
        self.ax_gyro = self.fig.add_subplot(gs[2, 1:3])
        self._style_ax(self.ax_gyro, "Gyroscope (°/s)", ylabel="°/s")

        # ── Magnetometer ─────────────────────────────────────────────────
        self.ax_mag = self.fig.add_subplot(gs[0, 3])
        self._style_ax(self.ax_mag, "Magnetometer (Gauss)", ylabel="Gauss")

        # ── Quaternion norm ──────────────────────────────────────────────
        self.ax_norm = self.fig.add_subplot(gs[1, 3])
        self._style_ax(self.ax_norm, "Quaternion Norm ‖q‖", ylabel="‖q‖")

        # ── Dashboard ────────────────────────────────────────────────────
        self.ax_dash = self.fig.add_subplot(gs[2, 3])
        self.ax_dash.set_facecolor(C["panel"])
        self.ax_dash.axis("off")

        # Title
        self.fig.text(0.5, 0.97,
                      "BEARS Rover  ·  3DM-CV7-AHRS  ·  Live Data Monitor",
                      ha="center", va="top",
                      color=C["text"], fontsize=13, fontweight="bold",
                      fontfamily="monospace")

        self._init_lines()
        self._box_poly = None

    def _setup_3d(self):
        ax = self.ax3d
        ax.set_facecolor(C["bg"])
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor(C["border"])
        ax.yaxis.pane.set_edgecolor(C["border"])
        ax.zaxis.pane.set_edgecolor(C["border"])
        ax.tick_params(colors=C["muted"], labelsize=7)
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("X", color=C["roll"],  fontsize=8)
        ax.set_ylabel("Y", color=C["pitch"], fontsize=8)
        ax.set_zlabel("Z", color=C["yaw"],   fontsize=8)
        ax.set_title("IMU Orientation", color=C["text"],
                     fontsize=9, pad=4)

        # Draw fixed axes
        for v, c in [([1,0,0], C["roll"]),
                     ([0,1,0], C["pitch"]),
                     ([0,0,1], C["yaw"])]:
            ax.quiver(0,0,0, *v, length=1.1, color=c,
                      alpha=0.3, linewidth=1)

    def _style_ax(self, ax, title, ylabel=""):
        ax.set_facecolor(C["panel"])
        ax.tick_params(colors=C["muted"], labelsize=8)
        ax.set_title(title, color=C["text"], fontsize=9, pad=3)
        ax.set_ylabel(ylabel, color=C["muted"], fontsize=8)
        ax.set_xlabel("samples", color=C["muted"], fontsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor(C["border"])
        ax.grid(True, color=C["border"], alpha=0.4, linewidth=0.5)

    def _init_lines(self):
        t = np.array([0.0])

        # Euler
        self.l_roll,  = self.ax_euler.plot(t, t, color=C["roll"],  lw=1.2, label="Roll")
        self.l_pitch, = self.ax_euler.plot(t, t, color=C["pitch"], lw=1.2, label="Pitch")
        self.l_yaw,   = self.ax_euler.plot(t, t, color=C["yaw"],   lw=1.2, label="Yaw")
        self.ax_euler.legend(loc="upper right", fontsize=7,
                             facecolor=C["panel"], labelcolor=C["text"])

        # Accel
        self.l_ax, = self.ax_accel.plot(t, t, color=C["ax"], lw=1, label="X")
        self.l_ay, = self.ax_accel.plot(t, t, color=C["ay"], lw=1, label="Y")
        self.l_az, = self.ax_accel.plot(t, t, color=C["az"], lw=1.4, label="Z (≈±g)")
        self.ax_accel.axhline(9.807,  color=C["az"], lw=0.5, ls="--", alpha=0.4)
        self.ax_accel.axhline(-9.807, color=C["az"], lw=0.5, ls="--", alpha=0.4)
        self.ax_accel.legend(loc="upper right", fontsize=7,
                             facecolor=C["panel"], labelcolor=C["text"])

        # Gyro
        self.l_gx, = self.ax_gyro.plot(t, t, color=C["gx"], lw=1, label="X")
        self.l_gy, = self.ax_gyro.plot(t, t, color=C["gy"], lw=1, label="Y")
        self.l_gz, = self.ax_gyro.plot(t, t, color=C["gz"], lw=1, label="Z (yaw rate)")
        self.ax_gyro.axhline(0, color=C["muted"], lw=0.5, alpha=0.4)
        self.ax_gyro.legend(loc="upper right", fontsize=7,
                            facecolor=C["panel"], labelcolor=C["text"])

        # Mag
        self.l_mx, = self.ax_mag.plot(t, t, color=C["mx"], lw=1, label="X")
        self.l_my, = self.ax_mag.plot(t, t, color=C["my"], lw=1, label="Y")
        self.l_mz, = self.ax_mag.plot(t, t, color=C["mz"], lw=1, label="Z")
        self.ax_mag.legend(loc="upper right", fontsize=7,
                           facecolor=C["panel"], labelcolor=C["text"])

        # Norm
        self.l_norm, = self.ax_norm.plot(t, t, color=C["green"], lw=1)
        self.ax_norm.axhline(1.0, color=C["amber"], lw=0.8, ls="--")
        self.ax_norm.set_ylim(0.98, 1.02)

    def _update_3d(self, qw, qx, qy, qz, roll, pitch, yaw):
        ax = self.ax3d
        R  = quat_to_rot(qw, qx, qy, qz)
        v  = rotate_box(make_box(), R)

        if self._box_poly:
            self._box_poly.remove()

        faces = [[v[i] for i in f] for f in BOX_FACES]
        poly  = Poly3DCollection(faces,
                                 facecolors=FACE_COLORS,
                                 edgecolors=C["blue"],
                                 linewidths=0.6,
                                 alpha=FACE_ALPHA)
        self._box_poly = ax.add_collection3d(poly)

        # Draw body axes
        if hasattr(self, "_body_axes"):
            for q in self._body_axes:
                q.remove()

        axes_dirs = R @ np.eye(3)
        self._body_axes = [
            ax.quiver(0,0,0, *axes_dirs[:,0], length=0.9,
                      color=C["roll"],  linewidth=2),
            ax.quiver(0,0,0, *axes_dirs[:,1], length=0.9,
                      color=C["pitch"], linewidth=2),
            ax.quiver(0,0,0, *axes_dirs[:,2], length=0.9,
                      color=C["yaw"],  linewidth=2),
        ]

        ax.set_title(
            f"Roll={roll:+6.1f}°  Pitch={pitch:+6.1f}°  Yaw={yaw:+6.1f}°",
            color=C["text"], fontsize=8, pad=2)

    def _update_dashboard(self, d: dict):
        ax = self.ax_dash
        ax.cla()
        ax.set_facecolor(C["panel"])
        ax.axis("off")

        fs    = str(d.get("filter_state", "?"))
        fs_c  = C["green"] if "Nav" in fs or "Gyro" in fs else C["amber"]
        norm  = float(d.get("qnorm", 0))
        norm_c = C["green"] if abs(norm - 1.0) < 0.005 else C["red"]

        az    = float(d.get("az", 0))
        ay    = float(d.get("ay", 0))
        axv   = float(d.get("ax", 0))
        gtot  = math.sqrt(axv**2 + ay**2 + az**2)
        g_c   = C["green"] if abs(gtot - 9.807) < 0.3 else C["amber"]

        lines = [
            ("Filter", fs,              fs_c),
            ("|q|",    f"{norm:.5f}",   norm_c),
            ("|a|",    f"{gtot:.3f} m/s²", g_c),
            ("Roll",   f"{d.get('roll',0):+.2f}°",  C["roll"]),
            ("Pitch",  f"{d.get('pitch',0):+.2f}°", C["pitch"]),
            ("Yaw",    f"{d.get('yaw',0):+.2f}°",   C["yaw"]),
        ]

        ax.text(0.5, 0.97, "Dashboard",
                ha="center", va="top", transform=ax.transAxes,
                color=C["text"], fontsize=9, fontweight="bold")

        for i, (label, val, color) in enumerate(lines):
            y = 0.82 - i * 0.13
            ax.text(0.05, y, label + ":",
                    transform=ax.transAxes,
                    color=C["muted"], fontsize=9, va="center")
            ax.text(0.55, y, str(val),
                    transform=ax.transAxes,
                    color=color, fontsize=9, va="center",
                    fontweight="bold")

    def _animate(self, _frame):
        ok = self.reader.update()
        if not ok or self.reader.n < 2:
            return

        d  = self.reader.latest
        n  = WINDOW

        t_arr    = np.arange(min(n, self.reader.n))
        roll_s   = self.reader.get_series("roll",  n)
        pitch_s  = self.reader.get_series("pitch", n)
        yaw_s    = self.reader.get_series("yaw",   n)
        ax_s     = self.reader.get_series("ax",    n)
        ay_s     = self.reader.get_series("ay",    n)
        az_s     = self.reader.get_series("az",    n)
        gx_s     = self.reader.get_series("gx",    n)
        gy_s     = self.reader.get_series("gy",    n)
        gz_s     = self.reader.get_series("gz",    n)
        mx_s     = self.reader.get_series("mx",    n)
        my_s     = self.reader.get_series("my",    n)
        mz_s     = self.reader.get_series("mz",    n)
        norm_s   = self.reader.get_series("qnorm", n)

        ta = np.arange(len(roll_s))

        def upd(line, x, y):
            line.set_data(x[:len(y)], y)

        upd(self.l_roll,  ta, roll_s)
        upd(self.l_pitch, ta, pitch_s)
        upd(self.l_yaw,   ta, yaw_s)
        for ax_obj in [self.ax_euler, self.ax_accel,
                       self.ax_gyro, self.ax_mag, self.ax_norm]:
            ax_obj.relim()
            ax_obj.autoscale_view()

        upd(self.l_ax, ta, ax_s)
        upd(self.l_ay, ta, ay_s)
        upd(self.l_az, ta, az_s)
        upd(self.l_gx, ta, gx_s)
        upd(self.l_gy, ta, gy_s)
        upd(self.l_gz, ta, gz_s)
        upd(self.l_mx, ta, mx_s)
        upd(self.l_my, ta, my_s)
        upd(self.l_mz, ta, mz_s)
        upd(self.l_norm, ta, norm_s)
        self.ax_norm.set_ylim(
            min(0.98, np.min(norm_s) - 0.002),
            max(1.02, np.max(norm_s) + 0.002)
        )

        # 3D box
        try:
            qw = float(d.get("qw", 1))
            qx = float(d.get("qx", 0))
            qy = float(d.get("qy", 0))
            qz = float(d.get("qz", 0))
            self._update_3d(qw, qx, qy, qz,
                            float(d.get("roll",  0)),
                            float(d.get("pitch", 0)),
                            float(d.get("yaw",   0)))
        except Exception:
            pass

        self._update_dashboard(d)

    def run(self):
        self.anim = FuncAnimation(
            self.fig, self._animate,
            interval=1000 // FPS,
            cache_frame_data=False
        )
        plt.show()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="CV7-AHRS Live Visualizer")
    parser.add_argument("--log",    default=str(DEFAULT_LOG),
                        help="Path to cv7_live.csv (default: auto)")
    parser.add_argument("--replay", default=None,
                        help="Replay a recorded CSV file")
    args = parser.parse_args()

    if args.replay:
        path   = args.replay
        replay = True
    else:
        path   = args.log
        replay = False

    print(f"[CV7 Visualizer] Reading from: {path}")
    if not os.path.exists(path):
        print(f"[WARN] File not found yet: {path}")
        print("       Start the ROS2 logger inside Docker first:")
        print("       python3 cv7_ros2_logger.py")
        print("       Waiting for file to appear...")

    vis = CV7Visualizer(path, replay=replay)
    vis.run()


if __name__ == "__main__":
    main()
