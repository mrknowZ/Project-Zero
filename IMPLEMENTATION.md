# Implementation Notes

## Overview

This repository is a ROS2 colcon workspace for the SAFiR Lab Project Zero assignment: build a
complete autonomy pipeline (mapping, localization, navigation, vision) on a Clearpath Jackal,
first in simulation, then on the real robot.

`src/` already contains the required upstream Clearpath packages, pinned as regular folders
(not submodules), so cloning this repo gives you a ready-to-build workspace with no separate
`git clone` steps for dependencies:

- `clearpath_common` — robot description (URDF), control, and platform-level packages shared
  across Clearpath robots.
- `clearpath_simulator` — Gazebo Harmonic integration (`clearpath_gz`) that spawns the Jackal
  and simulation worlds.
- `clearpath_nav2_demos` — SLAM Toolbox / Nav2 launch files and configs already tuned for
  Clearpath platforms.

## Stack / Dependencies

| Component        | Version / Package                                   |
|-------------------|------------------------------------------------------|
| OS                | Ubuntu 24.04 (WSL2 or native)                        |
| ROS2 distro       | Jazzy                                                 |
| Simulator         | Gazebo Harmonic, via `ros-gz`                        |
| Build tool        | `colcon`                                              |
| Dependency resolver | `rosdep`                                            |
| Navigation stack  | Nav2                                                  |
| Mapping           | SLAM Toolbox                                          |
| Vision            | YOLO (or similar), publishing `vision_msgs/Detection2D` |
| Hardware target   | Clearpath Jackal (J100), 2D LiDAR, Velodyne VLP-16, onboard i5 CPU + RTX 3070 GPU |

All ROS2 package-level dependencies (Nav2, SLAM Toolbox, `ros-gz`, etc.) are declared in each
package's `package.xml` and are resolved automatically by `rosdep` in the setup steps below —
no manual `apt`/`pip` list is needed beyond ROS2 and Gazebo themselves.

## Setup

### 1. Install ROS2 Jazzy and Gazebo Harmonic

```bash
# ROS2 Jazzy: follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
sudo apt-get install ros-jazzy-ros-gz
```

### 2. Clone this repository

```bash
git clone https://github.com/mrknowZ/Project-Zero.git
cd Project-Zero
```

### 3. Resolve dependencies with rosdep

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install -r --from-paths src -i -y
```

### 4. Build the workspace

```bash
colcon build --symlink-install
```

If your machine is memory-constrained, build one package at a time instead:
```bash
colcon build --packages-select <package_name>
```

### 5. Source the workspace

```bash
source install/setup.bash
```

### 6. Robot configuration (`robot.yaml`)

The simulator and Nav2 stack both read a single Clearpath Configuration YAML file at
`~/clearpath/robot.yaml`, defining the robot model, sensors, and mounts.

```bash
mkdir -p ~/clearpath
# copy or write robot.yaml here — a J100 (Jackal) sample is available from
# https://github.com/clearpathrobotics/clearpath_config/blob/main/clearpath_config/sample/j100/j100_sample.yaml
```
This project's config was trimmed to a single `velodyne_lidar` (VLP-16) sensor to match the
lab's actual hardware.

## Running the Project

### Simulation

```bash
ros2 launch clearpath_gz simulation.launch.py
```

Optional world selection (`warehouse` is default):
```bash
ros2 launch clearpath_gz simulation.launch.py world:=pipeline
```
Available worlds: `construction`, `office`, `orchard`, `pipeline`, `solar_farm`, `warehouse`.

### SLAM (mapping)

```bash
ros2 launch clearpath_nav2_demos slam.launch.py
```
Drive the robot around (teleop) to build the map, then save it with `nav2_map_server`'s
`map_saver_cli`.

### Localization + Navigation

```bash
ros2 launch clearpath_nav2_demos nav2.launch.py map:=<path_to_saved_map>.yaml
```
Send goals via RViz2's "Nav2 Goal" tool, or the `/goal_pose` topic.

### Vision

Object detection node publishes `vision_msgs/Detection2D` messages — launch file and package
name to be added once the vision pipeline is implemented.

### Real Jackal

Same launch files apply on hardware; `robot.yaml` on the robot's onboard computer already
matches the physical sensor layout. Confirm the hardware safety briefing (deadman switch,
floor clearance) before running on the real robot.

## Known Limitations / TODO

- Vision pipeline (YOLO integration) not yet wired into a launch file.
- Frontier exploration / semantic mapping extensions not yet attempted.
- End-to-end mission script (map → localize → navigate → detect → report) not yet automated
  into a single top-level launch file.
