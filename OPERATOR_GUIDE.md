# AGV OVP811 — Operator Guide

**System:** IFM O3R OVP811 AGV Navigation
**ROS2:** Jazzy
**Camera IP:** `10.167.53.27`

---

## Before You Start

Every terminal must have the workspace sourced. Your `.bashrc` does this automatically for new terminals. If something says a node or topic is not found, run:

```bash
source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash
```

Check the camera is reachable before launching anything:

```bash
ping 10.167.53.27
```

If the ping fails, check the Ethernet cable and confirm your PC has an IP in the `10.167.53.x` range.

---

## Mode 1: SLAM (Building a New Map)

Use this when entering a new environment or the existing map is out of date. The robot will build a map as it moves.

Open **6 separate terminals** and run one command per terminal in order.

---

### Terminal 1 — Camera
```bash
ros2 run ifm3d_ros2 camera_standalone --ros-args \
  --namespace ifm3d \
  --name camera \
  --params-file ~/colcon_ws/src/ifm3d-ros2/config/camera_default_parameters.yaml
```
Wait until you see `[camera]: State transition: active` before continuing.

---

### Terminal 2 — Robot Description (TF Tree)
```bash
ros2 run robot_state_publisher robot_state_publisher
```

---

### Terminal 3 — IMU
```bash
ros2 run agv_imu imu_publisher --ros-args \
  --name imu_publisher \
  -p ip:=10.167.53.27 \
  -p pcic_port:=50016 \
  -p frame_id:=imu_link
```

---

### Terminal 4 — ICP Odometry
```bash
ros2 run rtabmap_odom icp_odometry --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/icp_odometry.yaml \
  --remap scan_cloud:=/ifm3d/camera/cloud \
  --remap odom:=/odom_icp
```

---

### Terminal 5 — EKF (Sensor Fusion)
```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/ekf_params.yaml \
  --remap odometry/filtered:=/odom
```

---

### Terminal 6 — RTAB-Map SLAM
```bash
ros2 run rtabmap_slam rtabmap --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/rtabmap.yaml \
  --remap scan_cloud:=/ifm3d/camera/cloud \
  --remap odom:=/odom
```

---

### Terminal 7 — LaserScan (for Nav2 costmaps)
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/pointcloud_to_laserscan.yaml \
  --remap cloud_in:=/ifm3d/camera/cloud \
  --remap scan:=/scan
```

---

### When Mapping is Complete — Save the Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/agv_map
```

This saves `agv_map.pgm` and `agv_map.yaml` to `~/maps/`. You will use these in Mode 2.

To delete the RTAB-Map database and start fresh next time:
```bash
rm ~/.ros/rtabmap.db
```

---

## Mode 2: Navigation (Using an Existing Map)

Use this for normal operation when a map already exists. The robot localises on the saved map and accepts navigation goals.

---

### Terminal 1 — Camera
```bash
ros2 run ifm3d_ros2 camera_standalone --ros-args \
  --namespace ifm3d \
  --name camera \
  --params-file ~/colcon_ws/src/ifm3d-ros2/config/camera_default_parameters.yaml
```

---

### Terminal 2 — Robot Description
```bash
ros2 run robot_state_publisher robot_state_publisher
```

---

### Terminal 3 — IMU
```bash
ros2 run agv_imu imu_publisher --ros-args \
  --name imu_publisher \
  -p ip:=10.167.53.27 \
  -p pcic_port:=50016 \
  -p frame_id:=imu_link
```

---

### Terminal 4 — ICP Odometry
```bash
ros2 run rtabmap_odom icp_odometry --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/icp_odometry.yaml \
  --remap scan_cloud:=/ifm3d/camera/cloud \
  --remap odom:=/odom_icp
```

---

### Terminal 5 — EKF
```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/ekf_params.yaml \
  --remap odometry/filtered:=/odom
```

---

### Terminal 6 — LaserScan
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/pointcloud_to_laserscan.yaml \
  --remap cloud_in:=/ifm3d/camera/cloud \
  --remap scan:=/scan
```

---

### Terminal 7 — Nav2 Stack
```bash
ros2 run nav2_map_server map_server --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/nav2_params.yaml \
  -p yaml_filename:=$HOME/maps/agv_map.yaml

ros2 run nav2_amcl amcl --ros-args \
  --params-file ~/colcon_ws/src/agv_navigation/config/nav2_params.yaml \
  --remap scan:=/scan

ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:=[map_server,amcl]
```

> These three can run in the same terminal separated by `&`, or each in their own terminal.

---

### Terminal 8 — rosbridge (for HMI)
```bash
ros2 run rosbridge_server rosbridge_websocket --ros-args \
  -p port:=9090
```

---

### Terminal 9 — HMI Web Server
```bash
python3 -m http.server 8080 --directory ~/colcon_ws/src/agv_hmi/hmi
```

Open a browser at: `http://localhost:8080`

---

### Terminal 10 — HMI Instruction Node
```bash
ros2 run agv_hmi instruction_node --ros-args --name instruction_node
```

---

### Terminal 11 — Person Detection (Optional)
```bash
ros2 run agv_person_detection person_detector --ros-args \
  --name person_detector \
  --params-file ~/colcon_ws/src/agv_person_detection/config/person_detection.yaml
```

---

## Visualisation (Optional — any mode)

```bash
ros2 run rviz2 rviz2 -d ~/colcon_ws/src/ifm3d-ros2/etc/ifm3d.rviz
```

---

## Sending a Navigation Goal

Via the HMI browser at `http://localhost:8080`, or via command line:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Startup Order Rules

The order matters. Do not skip steps or launch out of sequence:

1. **Camera must be active** before ICP odometry starts — ICP will crash if no point cloud arrives at startup
2. **ICP odometry must publish `/odom_icp`** before EKF starts — EKF will timeout and disable that source if it is not present within `sensor_timeout: 0.1 s`
3. **EKF must publish `/odom`** before RTAB-Map starts — RTAB-Map requires the odom TF chain to be live
4. **RTAB-Map (or at least `/scan`)** must be running before Nav2 localisation starts

---

## Troubleshooting

### Camera node starts but no topics appear
The lifecycle node failed to activate. Check:
```bash
ros2 lifecycle get /ifm3d/camera
```
If not `active`, the device is unreachable. Verify `ping 10.167.53.27`.

### ICP odometry keeps resetting / printing "Too much odometry error"
The point cloud does not have enough features for ICP to converge. Common causes:
- Robot is stationary next to a blank wall — move to an area with more geometry
- Point cloud is empty — check `/ifm3d/camera/cloud` has data: `ros2 topic hz /ifm3d/camera/cloud`
- Robot moved too fast between frames — slow down, max reliable speed is approximately 0.5 m/s

### AMCL not converging / robot lost on map
AMCL has lost track of position. Manually initialise the pose estimate:
```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}'
```
Replace `x` and `y` with the robot's approximate known position on the map.

### HMI page loads but shows "ROS disconnected"
rosbridge is not running or is on the wrong port. Ensure Terminal 8 (rosbridge) is running and check the browser console for connection errors. Default port is 9090.

### Person detector not starting / YOLO model error
The model file `yolov8n.pt` is missing. Copy it from:
```bash
cp ~/yolov8n.pt ~/colcon_ws/
```
Or allow it to download automatically (requires internet connection).

### EKF not publishing `/odom`
Check that both input topics are alive:
```bash
ros2 topic hz /odom_icp
ros2 topic hz /imu/data
```
If `/odom_icp` is missing, ICP odometry has not started or is crashing. If `/imu/data` is missing, the IMU node failed to connect to port 50016 — the EKF will still run on ICP alone but with degraded yaw estimation.

---

## System Limits

| Parameter | Value |
|-----------|-------|
| Max reliable speed | ~0.5 m/s (ICP constraint) |
| Nav2 commanded speed | 0.2 m/s (nav2_params.yaml) |
| Camera range | 0.1 – 4.0 m |
| LaserScan range | 0.1 – 4.0 m, ±52.5° |
| Map resolution | 0.05 m/cell |
| AMCL particles | 500 – 2000 |
| Arrival tolerance | 0.5 m (planner), 0.3 m (HMI instruction) |

---

## Quick Reference — Topic Checklist

Run these to confirm the stack is healthy before sending goals:

```bash
ros2 topic hz /ifm3d/camera/cloud     # expect ~20 Hz
ros2 topic hz /imu/data               # expect ~1000 Hz
ros2 topic hz /odom_icp               # expect ~20 Hz
ros2 topic hz /odom                   # expect ~100 Hz
ros2 topic hz /scan                   # expect ~10 Hz
ros2 topic hz /amcl_pose              # expect ~1-2 Hz once localised
```
