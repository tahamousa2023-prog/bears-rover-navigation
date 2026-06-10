# BEARS Rover — 3DM-CV7-AHRS Complete Setup Guide
**TU Berlin BEARS Team | Navigation Subsystem | IRC 2026**
*Written: May 2026 | Author: Navigation Team*

---

## Hardware

| Property | Value |
|---|---|
| Sensor | MicroStrain 3DM-CV7-AHRS |
| Part number | 6286-9960 |
| Serial number | 6286.189967 |
| Firmware | 1.0.07 |
| Interface | USB (VID:PID 0483:5740) |
| Output rate | Up to 1000 Hz (configured: 500 Hz) |
| Filter state achieved | Vertical Gyro → Full Nav (outdoors) |

---

## System Architecture

```
Windows PC
├── Docker Desktop (WSL2 backend)
│   └── Container: vsc-microstrain_inertial (ROS2 Jazzy)
│       ├── microstrain_inertial_driver  ← publishes /imu/data @ 500Hz
│       ├── /ekf/status                 ← filter health
│       └── /imu/mag                    ← magnetometer
├── Ubuntu-22.04 WSL  ← USB passthrough via usbipd
└── Python visualizer  ← reads CSV from shared folder
```

---

## One-Time Setup (already done — for reference)

### 1. Install tools
```powershell
winget install usbipd
winget install Git.Git
```

### 2. Configure WSL memory limits
```powershell
notepad "$env:USERPROFILE\.wslconfig"
```
```ini
[wsl2]
memory=6GB
processors=4
swap=4GB
```
```powershell
wsl --shutdown
```

### 3. Clone the driver
```powershell
cd C:\Users\engta\Desktop\2026\Bears\Rover\Navigation
git clone --recursive --branch ros2 https://github.com/LORD-MicroStrain/microstrain_inertial.git
cd microstrain_inertial
```

### 4. Edit devcontainer.json — remove Linux-only mounts
Remove these two lines from `.devcontainer/devcontainer.json`:
```json
"source=/dev,target=/dev,type=bind,consistency=cached",
"source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached",
```

### 5. Build in Docker
```powershell
# Open VS Code devcontainer, then in container terminal:
sudo apt install -y ros-jazzy-nmea-msgs ros-jazzy-rtcm-msgs \
  ros-jazzy-mavros-msgs ros-jazzy-geographic-msgs \
  ros-jazzy-robot-localization ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-sensor-msgs ros-jazzy-imu-tools ros-jazzy-rviz2

MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O1 -g0"
```

---

## Daily Startup Procedure

### Step 1 — Start Ubuntu WSL (keep window open)
```powershell
wsl -d Ubuntu-22.04
```

### Step 2 — Plug in CV7 USB, then attach to WSL
```powershell
# In new PowerShell as Admin
usbipd list                          # find BUSID (1-1 for CV7)
usbipd bind --busid 1-1
usbipd attach --busid 1-1 -w Ubuntu-22.04
```

### Step 3 — Verify USB in WSL
```bash
# In Ubuntu WSL window
ls /dev/ttyACM*    # should show /dev/ttyACM0
```

### Step 4 — Start Docker container with device
```powershell
docker run -it --rm `
  --name microstrain_imu `
  --device=/dev/ttyACM0 `
  --privileged `
  -v "C:\Users\engta\Desktop\2026\Bears\Rover\Navigation\microstrain_inertial:/home/microstrain/catkin_ws/src" `
  77a3004f6879 `
  bash
```

### Step 5 — Launch driver inside container
```bash
source /home/microstrain/catkin_ws/install/setup.bash
ros2 launch microstrain_inertial_driver microstrain_launch.py \
  port:=/dev/ttyACM0 \
  configure:=true \
  activate:=true \
  params_file:=/home/microstrain/catkin_ws/src/config/cv7_ahrs.yml
```

### Step 6 — Verify in second terminal
```powershell
docker exec -it microstrain_imu bash
```
```bash
source /home/microstrain/catkin_ws/install/setup.bash
ros2 topic hz /imu/data        # expect: ~500 Hz
ros2 topic echo /ekf/status    # expect: filter_state: Vertical Gyro or Full Nav
```

---

## Topics Published

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 500 Hz | Quaternion, angular velocity, linear accel |
| `/imu/data_raw` | `sensor_msgs/Imu` | 500 Hz | Raw unfiltered data |
| `/ekf/status` | `microstrain_inertial_msgs/HumanReadableStatus` | 1 Hz | Filter state, device info |
| `/tf` | TF transform | 500 Hz | imu_link → base_link |

---

## Working Config File (cv7_ahrs.yml)

```yaml
microstrain_inertial_driver:
  ros__parameters:
    port: /dev/ttyACM0
    baudrate: 115200
    pps_source: 0
    filter_pps_source: 0
    filter_declination_source: 1
    filter_declination: 0.23
    filter_heading_source: 1
    filter_auto_heading_alignment_selector: 1
    imu_data_rate: 500
    filter_data_rate: 100
```

---

## Filter States Explained

| State | Meaning | Trust orientation? |
|---|---|---|
| `Startup` | Booting | No |
| `Vertical Gyro` | Accel+gyro only, no mag heading | Roll/Pitch YES, Yaw NO |
| `Full Nav` | Full AHRS with magnetometer | YES — all axes |

**For rover competition:** Vertical Gyro is sufficient for IDMO and terrain traversal. Full Nav needed for RADO GPS navigation (get outdoors to initialize mag).

---

## Data Quality Reference

| Metric | Measured | Expected | Status |
|---|---|---|---|
| Output rate | 500.3 Hz | 500 Hz | ✅ |
| Rate std dev | 0.00027s | < 0.001s | ✅ |
| Accel Z (flat) | 9.807 m/s² | 9.807 m/s² | ✅ |
| Gyro noise (static) | < 0.006 rad/s | < 0.01 rad/s | ✅ |
| Quaternion norm | 1.0000 | 1.0000 | ✅ |
| Filter status | Stable | Stable | ✅ |

---

## Next Steps — Move to Jetson

```bash
# On Jetson (Ubuntu, ROS2 Humble)
sudo apt install ros-humble-microstrain-inertial-driver

# Set udev rules
wget https://raw.githubusercontent.com/LORD-MicroStrain/microstrain_inertial/ros2/microstrain_inertial_driver/debian/udev
sudo cp udev /etc/udev/rules.d/100-microstrain.rules
sudo udevadm control --reload-rules

# Copy config
cp cv7_ahrs.yml ~/navigation_ws/erc25/navigation_ws/src/app_microstrain/config/

# Launch
ros2 launch app_microstrain app_microstrain.launch.py
```
