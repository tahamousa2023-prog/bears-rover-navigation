# app_microstrain

ROS2 package for the **MicroStrain 3DM-CV7-AHRS** IMU on the BEARS rover.

## Hardware
- Sensor: 3DM-CV7-AHRS (Part: 6286-9960, Serial: 6286.189967)
- Interface: USB → `/dev/ttyACM0` on Jetson
- Output: 500 Hz IMU data, 100 Hz EKF filter

## Launch

```bash
# On Jetson
ros2 launch app_microstrain app_microstrain.launch.py
```

## Topics

| Topic | Type | Rate |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 500 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 100 Hz |
| `/ekf/status` | `microstrain_inertial_msgs/HumanReadableStatus` | 1 Hz |

## Tools

```bash
# Log all IMU data to CSV
python3 scripts/cv7_ros2_logger.py

# Live visualization (Windows)
python scripts/cv7_visualizer.py

# Calibration analysis
python analysis/cv7_calibration.py
```

## Docs
See `docs/COMPLETE_SETUP_GUIDE.md` and `docs/TEAM_CHEATSHEET.md`
