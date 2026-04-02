# AGV OVP811 Navigation System — Technical Handover

**Project:** Autonomous Guided Vehicle (AGV) — Visual SLAM + Nav2 Navigation
**Platform:** IFM O3R OVP811
**ROS2 Distro:** Jazzy
**Date:** April 2026
**Author:** Bart

---

## 1. System Overview

This system implements a full autonomous navigation stack for an AGV using the IFM O3R OVP811 as the sole perception sensor. It provides:

- **SLAM** — simultaneous localisation and map building (RTAB-Map, primary)
- **Localisation** — AMCL particle filter on a pre-built map
- **Path Planning** — Nav2 with Regulated Pure Pursuit controller
- **Person Detection** — YOLOv8n on RGB camera with ToF depth for proximity zones
- **Operator HMI** — Browser-based interface with turn-by-turn instructions via rosbridge

An alternative ORB-SLAM3 RGBD pipeline exists in `agv_navigation_orbslam/` but is not production-ready (placeholder camera calibration).

---

## 2. Hardware

| Component | Detail |
|-----------|--------|
| Compute unit | OVP811 (runs VPU firmware, connects over Ethernet) |
| 3D ToF camera head | O3R225, port0, PCIC 50010 |
| RGB camera head | O3R225, port2, PCIC 50012 |
| IMU | IIM42652, onboard OVP811, port6, PCIC 50016 |
| Camera mount height | 815 mm above floor (ifm_base_link) |
| Device IP | `10.167.53.27` |
| Host requirement | Same subnet (10.167.53.x), static IP |

> **Critical:** The device IP `10.167.53.27` is hardcoded in **four places** — see Section 8.

---

## 3. Workspace Layout

```
colcon_ws/
├── src/
│   ├── agv_description/        # Robot URDF / TF tree
│   ├── agv_hmi/                # HMI instruction node + web frontend
│   ├── agv_imu/                # OVP811 port6 IMU publisher
│   ├── agv_navigation/         # PRIMARY: RTAB-Map + EKF + Nav2
│   ├── agv_navigation_orbslam/ # ALTERNATIVE: ORB-SLAM3 (not production-ready)
│   ├── agv_person_detection/   # YOLOv8 person detector
│   ├── ifm3d-ros2/             # External: IFM camera driver (v1.1.0)
│   └── orb_slam3_ros2/         # External: ORB-SLAM3 ROS2 wrapper
```

---

## 4. Package Descriptions

### `agv_description`
Defines the robot's TF frame hierarchy via xacro URDF. All camera and IMU offsets are currently set to zero pending physical MCC calibration. The frame tree is:
```
base_footprint → base_link → ifm_base_link → camera_mounting_link → camera_optical_link
                                           → camera_rgb_mounting_link
                           → imu_link
```

### `agv_hmi`
- `instruction_node.py` — subscribes to Nav2 path and converts it to simple turn-by-turn JSON published on `/hmi/instruction`
- `hmi/index.html` — browser frontend, connects to rosbridge on port 9090
- Output JSON format: `{"action": "forward|turn_left|turn_right|arrived", "distance_to_next": 2.3, "distance_to_goal": 8.1, "heading": "north"}`
- Turn threshold: 20°, arrival radius: 0.3 m, update rate: 2 Hz

### `agv_imu`
Reads the OVP811's onboard IIM42652 IMU via PCIC binary protocol on port 50016. Batches up to 128 samples per frame and publishes each individually. Uses VPU hardware timestamps anchored to ROS clock on first frame. The IMU is currently **disabled in icp_odometry** (`subscribe_imu: false`) due to an unresolved PCIC port6 access issue — however the node itself starts and publishes `/imu/data` correctly and the EKF does receive it.

### `agv_navigation`
The primary navigation stack:
- ICP odometry on ToF point cloud → `/odom_icp`
- EKF fuses `/odom_icp` + `/imu/data` → `/odom`
- RTAB-Map SLAM on `/ifm3d/camera/cloud` → `/rtabmap/grid_map`, `/scan`
- Nav2 (map_server + AMCL + planner) in separate terminal
- rosbridge WebSocket on port 9090

### `agv_navigation_orbslam`
Alternative stack using ORB-SLAM3 for RGBD SLAM instead of RTAB-Map. **Not production-ready** — camera intrinsic calibration in `config/o3r_orbslam3.yaml` uses placeholder values (fx=fy=565.44). Real calibration must be extracted from the O3R factory calibration data.

### `agv_person_detection`
YOLOv8n (nano) running on RGB frames at up to 10 Hz. Depth from the ToF camera is used to estimate person distance. Publishes proximity alerts with three zones:

| Zone | Distance |
|------|----------|
| Danger | < 1.0 m |
| Warning | < 2.5 m |
| Caution | < 4.0 m |

### `ifm3d-ros2`
External IFM-provided ROS2 driver (v1.1.0). Uses **lifecycle nodes** — the launch file handles the configure → activate transition automatically. Do not manually call lifecycle transitions while it is running.

---

## 5. Sensor Fusion Architecture

```
[ToF cloud 20Hz]──► icp_odometry ──► /odom_icp ──┐
                                                   ├──► EKF (100Hz) ──► /odom ──► Nav2
[IMU 1kHz]─────────────────────────── /imu/data ──┘

[ToF cloud 20Hz]──► rtabmap ──► /rtabmap/grid_map, /map
[/odom]────────────────────────────────────────────────►

/scan (from pointcloud_to_laserscan) ──► AMCL, Nav2 costmaps
```

**Known weakness:** No wheel odometry. The EKF relies solely on ICP-derived odometry from the camera, which degrades in feature-poor environments (bare walls, open floors). This is a known limitation — see the technical assessment report for detail.

---

## 6. Configuration Files

| File | Purpose |
|------|---------|
| `ifm3d-ros2/config/camera_default_parameters.yaml` | Camera IP, PCIC port, buffer list |
| `agv_navigation/config/icp_odometry.yaml` | ICP tuning for 20 Hz O3R ToF |
| `agv_navigation/config/ekf_params.yaml` | EKF sensor fusion config |
| `agv_navigation/config/rtabmap.yaml` | SLAM parameters, DB path |
| `agv_navigation/config/nav2_params.yaml` | AMCL, planner, costmap, controller |
| `agv_navigation/config/pointcloud_to_laserscan.yaml` | Cloud → 2D scan conversion |
| `agv_person_detection/config/person_detection.yaml` | YOLO confidence, proximity zones |
| `agv_person_detection/config/camera_rgb.yaml` | RGB camera PCIC config (port2) |

---

## 7. Building the Workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

The `--symlink-install` flag means Python scripts and config files are symlinked, not copied — edits in `src/` take effect without rebuilding. C++ changes still require a rebuild.

---

## 8. Known Issues and Gotchas

### 8.1 Hardcoded IP Address
The O3R device IP `10.167.53.27` appears in four places. If the network changes, all four must be updated:

| File | Line/Parameter |
|------|---------------|
| `ifm3d-ros2/config/camera_default_parameters.yaml` | `ip:` |
| `agv_person_detection/config/camera_rgb.yaml` | `ip:` |
| `agv_imu/agv_imu/imu_publisher.py` | hardcoded in constructor |
| `agv_navigation/launch/bringup.launch.py` | launch argument default |

### 8.2 RTAB-Map Database Hardcoded Path
`agv_navigation/config/rtabmap.yaml` hardcodes:
```
/home/bart/.ros/rtabmap.db
```
Change this if running as a different user. The database grows unbounded — delete it to start fresh mapping.

### 8.3 IMU Disabled in ICP Odometry
`icp_odometry.yaml` has `subscribe_imu: false` because of an unresolved PCIC port6 timing issue. The EKF still receives IMU data — it is only disabled as an ICP initialisation hint. Do not enable it without resolving the port6 PCIC access race condition first.

### 8.4 ifm3d-ros2 Lifecycle Nodes
The camera driver uses ROS2 lifecycle nodes. If the camera node appears to start but publishes nothing, it has likely failed the `configure` → `activate` transition. Check:
```bash
ros2 lifecycle get /ifm3d/camera
```
Expected state: `active`. If stuck at `unconfigured`, check network connectivity to `10.167.53.27`.

### 8.5 Camera Calibration (ORB-SLAM3 Pipeline)
`agv_navigation_orbslam/config/o3r_orbslam3.yaml` contains placeholder intrinsic values. Real calibration must be extracted from the OVP811 via:
```bash
ros2 topic echo /ifm3d/camera/camera_info
```
Copy `K` matrix values to the yaml before using ORB-SLAM3.

### 8.6 No Wheel Odometry
The EKF has no wheel encoder source. ICP odometry is the sole positional input. This causes localisation degradation in feature-poor environments and cannot support dead reckoning through camera blackouts. Process noise covariance values in `ekf_params.yaml` are intentionally inflated (0.05 in X/Y) to compensate.

### 8.7 Nav2 Map File Path
The default map path is hardcoded to `/home/bart/maps/agv_map.yaml`. Override at launch:
```bash
ros2 launch agv_navigation nav2_localization.launch.py map:=/path/to/your/map.yaml
```

### 8.8 YOLOv8 Model File
`yolov8n.pt` must be present at `~/colcon_ws/yolov8n.pt`. It is auto-downloaded on first run if missing (requires internet). Copy it manually on air-gapped systems.

### 8.9 OVP811 Camera Offsets Not Calibrated
All TF offsets in `agv_description/urdf/agv.urdf.xacro` are set to zero. The camera is physically mounted 815 mm above ground (ifm_base_link height is set), but the lateral and angular offsets between the camera heads and base_link have not been physically measured and entered. This affects the accuracy of occupancy grid projections.

---

## 9. Map Management

Maps are saved by RTAB-Map automatically to `/home/bart/.ros/rtabmap.db`.

To export a 2D occupancy grid for Nav2:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/agv_map
```
This generates `agv_map.pgm` + `agv_map.yaml`. Pass the yaml to `nav2_localization.launch.py`.

To clear the map and restart from scratch, delete the database:
```bash
rm ~/.ros/rtabmap.db
```

---

## 10. Network Ports Reference

| Port | Protocol | Service |
|------|----------|---------|
| 80 | HTTP/XML-RPC | O3R configuration API |
| 50010 | TCP/PCIC | ToF camera data stream (port0) |
| 50012 | TCP/PCIC | RGB camera data stream (port2) |
| 50016 | TCP/PCIC | IMU data stream (port6) |
| 9090 | WebSocket | rosbridge (ROS2 ↔ browser) |
| 8080 | HTTP | HMI web server (manual launch) |

---

## 11. Dependencies

All ROS2 dependencies are declared in each package's `package.xml`. Key non-obvious dependencies:

- `ifm3d` C++ library — must be installed from `.deb` before building (`ifm3d-ubuntu-24.04-amd64-1.6.12.deb`)
- `rtabmap_ros` — install via `sudo apt install ros-jazzy-rtabmap-ros`
- `nav2` — `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
- `robot_localization` — `sudo apt install ros-jazzy-robot-localization`
- `rosbridge_suite` — `sudo apt install ros-jazzy-rosbridge-suite`
- `ultralytics` (YOLOv8) — `pip install ultralytics` in the active Python environment
