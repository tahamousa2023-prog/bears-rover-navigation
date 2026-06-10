# app_microstrain — CV7-AHRS IMU for ROS2

ROS2 Humble driver package for the **MicroStrain 3DM-CV7-AHRS** tactical-grade IMU, deployed as a Docker Compose service on NVIDIA Jetson Orin for the BEARS rover (TU Berlin, IRC 2026).

---

## Hardware

| Property | Value |
|---|---|
| Sensor | MicroStrain 3DM-CV7-AHRS |
| Part | 6286-9960 |
| Interface | USB (`/dev/ttyACM0`) |
| Output rate | 500 Hz |
| Filter | Internal AHRS EKF |

---

## What this package does

- Launches the MicroStrain ROS2 driver with a working config for the **AHRS variant** (no GPS, no PPS)
- Publishes `/imu/data` at 500 Hz with filtered orientation, angular velocity, and linear acceleration
- Publishes a static TF transform: `chassis_link → imu_link`
- Follows the BEARS team's Docker Compose pattern for the Jetson Orin navigation stack

---

## Key config fixes (AHRS-specific)

The CV7-AHRS is the inertial-only variant of the CV7 family. Several parameters that default to GPS/PPS modes cause the driver to crash on startup:

```yaml
pps_source: 0                  # AHRS has no PPS input
filter_pps_source: 0
filter_declination_source: 1   # manual (0 and 2 both fail on AHRS)
filter_declination: 0.23       # Berlin magnetic declination (radians)
tf_mode: 0                     # no global position available
```

Without these, the driver exits with `Failed to configure node`.

---

## Topics

| Topic | Type | Rate |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 500 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | 500 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 100 Hz |
| `/ekf/status` | `microstrain_inertial_msgs/HumanReadableStatus` | 1 Hz |

---

## Verified results

```
average rate: 500.03 Hz
std dev:       0.00022 s     ← tactical grade timing
filter_state:  Vertical Gyro + Stable
quaternion norm: 1.0000
```

---

## Usage on Jetson

```bash
# Clone repo and navigate
cd ~/navigation_ws/erc25/navigation_ws/src

# Start the IMU container
docker compose up app_microstrain -d

# Verify
docker exec app_microstrain_container \
  bash -c "source /opt/ros/humble/setup.bash && \
  ros2 topic hz /imu/data"
# → ~500 Hz

# Check filter state
docker exec app_microstrain_container \
  bash -c "source /opt/ros/humble/setup.bash && \
  ros2 topic echo /ekf/status --once"
# → filter_state: Vertical Gyro
```

---

## File structure

```
app_microstrain/
├── Dockerfile                    # ROS2 Humble + microstrain driver via apt
├── postcreatecommand.sh          # builds workspace + launches on container start
├── config/
│   └── cv7_ahrs.yml              # working AHRS config (see key fixes above)
├── launch/
│   └── app_microstrain.launch.py # driver node + static TF chassis_link→imu_link
├── scripts/
│   ├── cv7_ros2_logger.py        # logs all IMU topics to CSV (runs in Docker)
│   └── cv7_visualizer.py         # live 3D visualiser (runs on Windows/Linux)
├── analysis/
│   └── cv7_calibration.py        # Allan deviation + EKF covariance analysis
└── docs/
    ├── TEAM_CHEATSHEET.md        # topics, Python/C++ code snippets, EKF config
    └── COMPLETE_SETUP_GUIDE.md   # full setup from scratch
```

---

## EKF integration (robot_localization)

To fuse `/imu/data` into the navigation EKF:

```yaml
imu0: /imu/data
imu0_config: [false, false, false,
               true,  true,  true,
               false, false, false,
               true,  true,  true,
               true,  true,  true]
imu0_differential: false
imu0_remove_gravitational_acceleration: true
```

---

## Context

Part of the BEARS rover navigation stack for IRC 2026. The full stack runs on NVIDIA Jetson Orin with Docker Compose services for each sensor: ZED 2i camera, Ouster LiDAR, CV7-AHRS IMU, and wheel encoders — fused through RTABmap SLAM and a robot_localization EKF into Nav2.

---

**BEARS Navigation Team · TU Berlin · IRC 2026**
