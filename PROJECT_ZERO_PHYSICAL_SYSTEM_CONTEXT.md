# SAFiR Lab — Project Zero: Physical Robot System Master Handover & Context

**Robot Model**: Clearpath Jackal J100  
**Serial / Hostname**: `cpr-j100-0751`  
**Robot Namespace**: `j100_0751`  
**ROS 2 Distribution**: ROS 2 Humble (Robot)  
**Git Repository**: `mrknowZ/Project-Zero`  
**Working Branch**: `project_zero`  
**Network Frequency**: `ROS_DOMAIN_ID=0` (Clearpath Hardware Default)  

---

## 1. Physical Hardware & Sensor Architecture

| Sensor / Component | Physical Device | Published Topic | Message Type |
| :--- | :--- | :--- | :--- |
| **3D LiDAR (Planar 2D)** | Velodyne VLP-16 / Puck | `/j100_0751/sensors/lidar3d_0/scan` | `sensor_msgs/msg/LaserScan` |
| **3D LiDAR (Point Cloud)** | Velodyne VLP-16 / Puck | `/j100_0751/sensors/lidar3d_0/points` | `sensor_msgs/msg/PointCloud2` |
| **Front RGB Camera** | RealSense / USB Cam | `/j100_0751/sensors/camera_0/color/image` | `sensor_msgs/msg/Image` |
| **YOLO Vision Detections** | YOLOv8n (CPU Inference) | `/j100_0751/vision/detections_image` | `sensor_msgs/msg/Image` |
| **Wheel Encoders & EKF** | Clearpath Microcontroller | `/j100_0751/platform/odom` | `nav_msgs/msg/Odometry` |
| **Transform Tree (TF)** | Robot State Publisher / EKF | `/j100_0751/tf` & `tf_static` | `tf2_msgs/msg/TFMessage` |

---

## 2. Key Fixes & Root Cause History

1. **Hardware TF Tree Connectivity (`odom` $\rightarrow$ `base_link`)**:
   - *Problem*: Robot base driver was not broadcasting `odom -> base_link`.
   - *Fix*: In `/etc/clearpath/platform/config/localization.yaml`, set `publish_tf: True` and restarted `clearpath-platform.service`.
2. **3D Velodyne Topic Routing for AMCL & Nav2**:
   - *Problem*: AMCL and Costmaps listened for `sensors/lidar2d_0/scan` (simulation only) and received 0 scans, preventing `map -> odom` transform generation.
   - *Fix*: Configured dynamic scan topic routing in `system.launch.py`, `mapping.launch.py`, `localization.yaml`, and `nav2.yaml` to route `/j100_0751/sensors/lidar3d_0/scan` on hardware (`sim:=false`).
3. **Nav2 RewrittenYaml Topic Collision**:
   - *Problem*: `nav2.launch.py` was rewriting all `topic` keys, forcing `PointCloud2` voxel layer to subscribe to `LaserScan`, crashing `local_costmap`.
   - *Fix*: Removed global topic rewrite in `nav2.launch.py`, allowing `LaserScan` and `PointCloud2` to bind cleanly.
4. **Local Costmap Independence**:
   - *Problem*: `local_costmap` had `static_layer` enabled, making local rolling obstacle avoidance fail if `map` frame dropped.
   - *Fix*: Removed `static_layer` from `local_costmap.plugins` (`plugins: ["voxel_layer", "inflation_layer"]`).
5. **Network Domain Synchronization**:
   - *Problem*: Hardware platform services run on `ROS_DOMAIN_ID=0`. Setting terminal or laptop to `ROS_DOMAIN_ID=42` resulted in zero topic discovery.
   - *Fix*: Standardized all hardware and visualization terminals to `ROS_DOMAIN_ID=0`.
6. **WSL2 / Laptop Visualization (Foxglove WebSocket)**:
   - *Problem*: Windows WSL2 NAT virtual switch blocks DDS Multicast packets over physical Wi-Fi.
   - *Fix*: Integrated `ros-humble-foxglove-bridge` on port `8765`, enabling real-time browser-based visualization in Chrome/Edge at `ws://<ROBOT_IP>:8765`.

---

## 3. Complete Step-by-Step Field Runbook

### Step 1: Pre-Flight Hardware Health Check
On the robot (SSH):
```bash
sudo systemctl restart clearpath-platform.service
sudo systemctl restart clearpath-sensors.service

cd ~/ali/Project-Zero
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Run one-click system diagnostic
python3 scripts/diagnose_mapping.py
```
*(Verify that 3D LiDAR, Wheel Odom, and TF show ✅ ACTIVE).*

---

### Step 2: PS5 Teleop / Manual Driving
1. **Connect PS5 Controller**: Plug via USB-C to the robot or pair via Bluetooth (`bluetoothctl`).
2. **Drive**: Hold **L1** (deadman switch) and push **Left Stick**.
3. *(Alternative Keyboard Teleop)*:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/j100_0751/cmd_vel
   ```

---

### Step 3: SLAM Mapping & Map Generation
On the robot:
```bash
cd ~/ali/Project-Zero
git pull origin project_zero
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch jackal_mission mapping.launch.py \
    sim:=false \
    use_sim_time:=false \
    setup_path:=/etc/clearpath/ \
    use_rviz:=false
```
*Drive the robot in a closed loop around the environment with the PS5 controller.*

**Save the completed map**:
In another SSH terminal on the robot:
```bash
cd ~/ali/Project-Zero
./scripts/save_map.sh outdoor_map j100_0751
```
*(Saves `maps/outdoor_map.yaml` and `maps/outdoor_map.pgm`).*

---

### Step 4: Autonomous Navigation & YOLO Vision Mission
On the robot:
```bash
cd ~/ali/Project-Zero
git pull origin project_zero
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch jackal_mission system.launch.py \
    sim:=false \
    use_sim_time:=false \
    setup_path:=/etc/clearpath/ \
    map:=$(pwd)/maps/outdoor_map.yaml \
    rviz:=false
```

---

### Step 5: Web Visualization in Chrome / Edge (Foxglove Studio)
On the robot:
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

On your laptop (Windows Chrome/Edge):
1. Open **[https://app.foxglove.dev](https://app.foxglove.dev)**
2. Click **Open connection** $\rightarrow$ **Foxglove WebSocket** $\rightarrow$ enter `ws://<ROBOT_IP>:8765`.
3. View 3D LiDAR, SLAM map, YOLO camera detections, and send 2D Nav2 Goals!
