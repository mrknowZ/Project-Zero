# Implementation Notes

## Overview

This repository is a ROS2 colcon workspace for the SAFiR Lab Project Zero assignment: build a
complete autonomy pipeline (mapping, localization, navigation, vision) on a Clearpath Jackal,
first in simulation, then on the real robot.

`src/` contains the required upstream Clearpath packages alongside custom autonomy and perception packages:

- `clearpath_common` — robot description (URDF), control, and platform-level packages shared across Clearpath robots.
- `clearpath_simulator` — Gazebo Harmonic integration (`clearpath_gz`) that spawns the Jackal and simulation worlds.
- `clearpath_nav2_demos` — SLAM Toolbox / Nav2 launch files and configs tuned for Clearpath platforms.
- `jackal_vision` — **YOLOv8 object detection** package, publishing `vision_msgs/Detection2DArray` and annotated camera streams.
- `jackal_mission` — **Mission orchestrator**: waypoint navigation + detection collection + report generation. Includes the **top-level launch file** (`mission.launch.py`) for the full demo.

---

## Stack / Dependencies

| Component | Version / Package |
|---|---|
| OS | Ubuntu 22.04 (WSL2 or native) |
| ROS2 distro | Humble |
| Simulator | Gazebo Harmonic, via `ros-gz` |
| Build tool | `colcon` |
| Dependency resolver | `rosdep` |
| Navigation stack | Nav2 |
| Mapping | SLAM Toolbox |
| Vision | YOLOv8n via `ultralytics`, publishing `vision_msgs/Detection2DArray` |
| Hardware target | Clearpath Jackal (J100), Velodyne VLP-16, onboard i5 + RTX 3070 |

Install Python dependencies:
```bash
pip install ultralytics opencv-python-headless
```

---

## Build & Setup

```bash
# 1. Source ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. Resolve dependencies
rosdep update
rosdep install -r --from-paths src -i -y

# 3. Build workspace
colcon build --symlink-install

# 4. Source local environment
source install/setup.bash
```

---

## Running the Project

### 1. Combined Execution (Full Mission)

Launch everything with 4 easy terminal commands:

```bash
# Terminal 1 — Start Gazebo Simulation
ros2 launch clearpath_gz simulation.launch.py

# Terminal 2 — Unpause Simulation Clock
ign service -s /world/warehouse/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'

# Terminal 3 — Pre-configured RViz2 (Nav2 + LiDAR + Camera + YOLO Detections)
ros2 run rviz2 rviz2 \
    -d src/jackal_vision/config/mission_viz.rviz \
    --ros-args -r __ns:=/j100_0000 -p use_sim_time:=true

# Terminal 4 — Full Mission (PointCloud Bridge + AMCL + Nav2 + YOLO + Mission Node)
ros2 launch jackal_mission mission.launch.py \
    use_sim_time:=true \
    map:=$(pwd)/maps/warehouse_map.yaml
```

---

### 2. Individual / Modular Component Execution

For modular testing, launch each component in its own terminal:

```bash
# 1. Simulation
ros2 launch clearpath_gz simulation.launch.py

# 2. Unpause Clock
ign service -s /world/warehouse/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'

# 3. PointCloud to LaserScan Bridge
ros2 launch launch/pointcloud_to_laserscan.launch.py

# 4. SLAM Mapping (Optional, to generate new maps)
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=$HOME/clearpath/

# 5. Localization (AMCL)
ros2 launch clearpath_nav2_demos localization.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/ \
    map:=$(pwd)/maps/warehouse_map.yaml

# 6. Navigation (Nav2)
ros2 launch clearpath_nav2_demos nav2.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/

# 7. Vision (YOLOv8)
ros2 launch jackal_vision vision.launch.py namespace:=j100_0000 use_sim_time:=true

# 8. RViz2 Visualizer
ros2 run rviz2 rviz2 \
    -d src/jackal_vision/config/mission_viz.rviz \
    --ros-args -r __ns:=/j100_0000 -p use_sim_time:=true

# 9. Mission Sequencer
ros2 run jackal_mission mission_node --ros-args -r __ns:=/j100_0000 -p use_sim_time:=true
```

---

## Sensor Topics & Interfaces

| Sensor | Topic | Message Type |
|---|---|---|
| Velodyne VLP-16 (3D) | `/j100_0000/sensors/lidar3d_0/points` | `sensor_msgs/PointCloud2` |
| Converted 2D Scan | `/j100_0000/sensors/lidar2d_0/scan` | `sensor_msgs/LaserScan` |
| RGB Camera | `/j100_0000/sensors/camera_0/color/image` | `sensor_msgs/Image` |
| YOLO Detections | `/j100_0000/yolo_detector/detections` | `vision_msgs/Detection2DArray` |
| Annotated Image | `/j100_0000/yolo_detector/detections_image` | `sensor_msgs/Image` |
| Filtered Odometry | `/j100_0000/platform/odom/filtered` | `nav_msgs/Odometry` |
| AMCL Estimated Pose | `/j100_0000/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
