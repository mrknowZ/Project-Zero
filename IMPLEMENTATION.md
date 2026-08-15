# Implementation Notes

## Overview

This repository is a ROS2 colcon workspace for the SAFiR Lab Project Zero assignment: build a
complete autonomy pipeline (mapping, localization, navigation, vision) on a Clearpath Jackal,
first in simulation, then on the real robot.

`src/` contains the required upstream Clearpath packages, pinned as regular folders
(not submodules), so cloning this repo gives you a ready-to-build workspace with no separate
`git clone` steps for dependencies:

- `clearpath_common` — robot description (URDF), control, and platform-level packages shared
  across Clearpath robots.
- `clearpath_simulator` — Gazebo Harmonic integration (`clearpath_gz`) that spawns the Jackal
  and simulation worlds.
- `clearpath_nav2_demos` — SLAM Toolbox / Nav2 launch files and configs already tuned for
  Clearpath platforms.
- `jackal_vision` — **YOLOv8 object detection** node, publishes `vision_msgs/Detection2DArray`.
- `jackal_mission` — **Mission orchestrator**: waypoint navigation + detection collection +
  report generation. Includes the **top-level launch file** for the full demo.

## Stack / Dependencies

| Component         | Version / Package                                    |
|-------------------|------------------------------------------------------|
| OS                | Ubuntu 22.04 (WSL2 or native)                        |
| ROS2 distro       | Humble                                               |
| Simulator         | Gazebo Harmonic, via `ros-gz`                        |
| Build tool        | `colcon`                                             |
| Dependency resolver | `rosdep`                                           |
| Navigation stack  | Nav2                                                 |
| Mapping           | SLAM Toolbox                                         |
| Vision            | YOLOv8n via `ultralytics`, publishing `vision_msgs/Detection2DArray` |
| Hardware target   | Clearpath Jackal (J100), Velodyne VLP-16, onboard i5 + RTX 3070 |

All ROS2 package-level dependencies (Nav2, SLAM Toolbox, `ros-gz`, etc.) are declared in each
package's `package.xml` and are resolved automatically by `rosdep` in the setup steps below —
no manual `apt`/`pip` list is needed beyond ROS2 and Gazebo themselves.

**Python dependencies** (not managed by `rosdep`):
```bash
pip install ultralytics opencv-python-headless
```

## Setup

### 1. Install ROS2 Humble and Gazebo Harmonic

```bash
# ROS2 Humble: follow https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt-get install ros-humble-ros-gz
```

### 2. Clone this repository

```bash
git clone https://github.com/mrknowZ/Project-Zero.git
cd Project-Zero
```

### 3. Resolve dependencies with rosdep

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -r --from-paths src -i -y
```

### 4. Install Python dependencies

```bash
pip install ultralytics opencv-python-headless
```

### 5. Build the workspace

```bash
colcon build --symlink-install
```

If your machine is memory-constrained, build one package at a time instead:
```bash
colcon build --packages-select <package_name>
```

### 6. Source the workspace

```bash
source install/setup.bash
```

### 7. Robot configuration (`robot.yaml`)

The simulator and Nav2 stack both read a single Clearpath Configuration YAML file at
`~/clearpath/robot.yaml`, defining the robot model, sensors, and mounts.

```bash
mkdir -p ~/clearpath
# copy or write robot.yaml here — a J100 (Jackal) sample is available from
# https://github.com/clearpathrobotics/clearpath_config/blob/main/clearpath_config/sample/j100/j100_sample.yaml
```
This project's config was trimmed to a single `velodyne_lidar` (VLP-16) sensor to match the
lab's actual hardware. **For vision**: add a camera sensor to `robot.yaml` so image topics are
published in simulation (see TECHNICAL_NOTE.md § 4 for details).

## Running the Project

### Simulation (Gazebo)

```bash
ros2 launch clearpath_gz simulation.launch.py
```

Optional world selection (`warehouse` is default):
```bash
ros2 launch clearpath_gz simulation.launch.py world:=pipeline
```
Available worlds: `construction`, `office`, `orchard`, `pipeline`, `solar_farm`, `warehouse`.

### Pointcloud → LaserScan bridge

Required before SLAM or Nav2 (converts VLP-16 3D pointcloud to 2D laserscan):
```bash
ros2 launch launch/pointcloud_to_laserscan.launch.py
```

### SLAM (mapping)

```bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=$HOME/clearpath/
```
Drive the robot around (teleop) to build the map, then save it:
```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map --ros-args -p use_sim_time:=true
```

### Localization (AMCL)

```bash
ros2 launch clearpath_nav2_demos localization.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/ \
    map:=$(pwd)/maps/warehouse_map.yaml
```

### Navigation (Nav2)

```bash
ros2 launch clearpath_nav2_demos nav2.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/
```
Send goals via RViz2's "Nav2 Goal" tool, or the `/goal_pose` topic.

### Vision (YOLO object detection)

```bash
ros2 launch jackal_vision vision.launch.py namespace:=j100_0000
```
Publishes:
- `~/detections` — `vision_msgs/Detection2DArray`
- `~/detections_image` — annotated camera image for RViz2

### Full Mission (end-to-end demo)

One command to bring up the complete pipeline (pointcloud bridge + AMCL + Nav2 + YOLO + mission):
```bash
ros2 launch jackal_mission mission.launch.py \
    use_sim_time:=true \
    map:=$(pwd)/maps/warehouse_map.yaml
```
Reports are written to `/tmp/jackal_mission_reports/`.

### Real Jackal

Same launch files apply on hardware; `robot.yaml` on the robot's onboard computer already
matches the physical sensor layout. Confirm the hardware safety briefing (deadman switch,
floor clearance) before running on the real robot.

## Sensor Topics

| Sensor      | Topic                                         | Message Type            |
|-------------|-----------------------------------------------|-------------------------|
| VLP-16 3D   | `/j100_0000/sensors/lidar3d_0/points`         | `sensor_msgs/PointCloud2` |
| 2D scan     | `/j100_0000/sensors/lidar2d_0/scan`           | `sensor_msgs/LaserScan`   |
| Camera RGB  | `/j100_0000/sensors/camera_0/color/image`     | `sensor_msgs/Image`       |
| Detections  | `/j100_0000/yolo_detector/detections`         | `vision_msgs/Detection2DArray` |
| Odometry    | `/j100_0000/platform/odom/filtered`           | `nav_msgs/Odometry`      |

## TF Tree

The TF tree is verified via `ros2 run tf2_tools view_frames`. Key frames:

```
map → odom → base_link → [sensor frames]
                 ├── lidar3d_0_laser
                 ├── camera_0_color_optical_frame
                 └── ... (wheels, IMU, etc.)
```

The `map → odom` transform is published by AMCL (localization) or SLAM Toolbox (mapping).

## Map Parameters

| Parameter       | Value  | Rationale                                  |
|-----------------|--------|--------------------------------------------|
| Resolution      | 0.05 m | Good balance of detail vs. file size       |
| Max laser range | 20.0 m | Matches VLP-16 effective indoor range      |
| Loop closure    | enabled | Corrects drift in large environments      |
| Occupied thresh | 0.65   | Standard Nav2 default                      |
| Free thresh     | 0.196  | Standard Nav2 default                      |
