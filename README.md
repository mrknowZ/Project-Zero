# SAFiR Lab – Project Zero (Jackal Autonomy & Vision Pipeline)

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![Platform](https://img.shields.io/badge/Platform-Clearpath%20Jackal%20J100-yellow.svg)](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/)
[![Vision](https://img.shields.io/badge/Vision-YOLOv8%20Realtime-green.svg)](https://github.com/ultralytics/ultralytics)
[![Status](https://img.shields.io/badge/Release-v1.3.0--stable-brightgreen.svg)](https://github.com/mrknowZ/Project-Zero)

An end-to-end autonomous navigation, mapping, localization, dynamic obstacle avoidance, and real-time computer vision pipeline for the **Clearpath Jackal (J100)** mobile robot.

Designed for **100% plug-and-play reproducibility** in both **Gazebo Harmonic simulation** and on **physical Jackal robot hardware**.

---

## 🚀 Quick Start (Plug & Play)

### 1. Prerequisites & Dependencies
```bash
# Sourced ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Install Python vision dependencies
pip install ultralytics opencv-python-headless

# Install ROS 2 system dependencies
cd ~/ali/Project-Zero
rosdep install -r --from-paths src -i -y
```

### 2. Build Workspace
```bash
cd ~/ali/Project-Zero
colcon build --symlink-install
source install/setup.bash
```

---

## ⚡ Running the System

### Option A: ONE-COMMAND Top-Level Bringup (Autonomous Full Mission)
Runs the entire stack with a single command: Gazebo simulation, clock unpause, LiDAR bridge, AMCL localization, Nav2 autonomy, YOLOv8 vision, RViz2 visualizer, and the multi-waypoint mission sequencer:

```bash
ros2 launch jackal_mission system.launch.py
```

### Option B: Interactive Mode (Manual 2D Goal Pose & Pose Estimation in RViz)
Brings up all sensors, costmaps, localization, vision, and RViz without auto-starting waypoints, allowing manual goal setting:

```bash
ros2 launch jackal_mission system.launch.py mission:=false
```

### Option C: Physical Jackal Deployment (Real Hardware)
Run directly on the real Jackal robot connected to onboard sensors:

```bash
ros2 launch jackal_mission system.launch.py \
    sim:=false \
    use_sim_time:=false \
    setup_path:=/etc/clearpath/ \
    map:=/path/to/lab_map.yaml
```

---

## 🎮 Driving with PlayStation (PS4 / PS5) Controller (Real Robot)

Clearpath robots enforce a **Deadman Safety Switch**: the robot will **never move** unless the deadman button is held down.

```text
         [ L1 ] (Deadman Enable)           [ R1 ] (Turbo Boost)
         [ L2 ]                            [ R2 ]
      _=====_                           _=====_
     / _____ \                         / _____ \
   +.-'_____'-.-----------------------.-'_____'-.+
  /   |     |  '.    S O N Y        .'  |  △  |   \
 |  <---   ---> |                   |  □     ○  |
  \   |  ▲  |  /  _   ( PS )   _     \  |  ✕  |  /
   |  |  ▼  | / ,'" "',     ,'" "',   \ |_____| /
   |  '-...-'/  |  LJ |-----|  RJ |    \       |
    \       /   \ ___ /     \ ___ /     \     /
     \_____/                             \___/
```

| Action | Control / Button |
|---|---|
| **Drive (Normal Speed)** | **Hold `L1`** + Push **Left Thumbstick** (Up/Down for Speed, Left/Right for Steering) |
| **Drive (Turbo Speed)** | **Hold `R1`** + Push **Left Thumbstick** |
| **Instant Emergency Stop** | **Release `L1` / `R1`** (Robot brakes immediately) |

### Bluetooth Pairing Steps:
1. Turn controller OFF. Hold **`SHARE` (or `CREATE`) + `PS` button** for 5 seconds until the light bar rapidly blinks white.
2. On Jackal onboard terminal:
   ```bash
   sudo bluetoothctl
   [bluetooth]# agent on
   [bluetooth]# default-agent
   [bluetooth]# scan on
   [bluetooth]# pair <MAC_ADDRESS>
   [bluetooth]# trust <MAC_ADDRESS>
   [bluetooth]# connect <MAC_ADDRESS>
   ```

---

## 🗺️ Interactive RViz2 Navigation & Goal Setting

When running in interactive mode (`mission:=false`):

1. **Set Initial Pose**:
   - Click **`2D Pose Estimate`** (key `p`) in RViz top toolbar.
   - Click at `(0, 0)` on the map and drag the green arrow pointing **Right ($+X$ axis)**.
   - Green AMCL particle swarm will snap tightly to the robot chassis.
2. **Send Navigation Goal**:
   - Click **`2D Goal Pose`** / **`Nav2 Goal`** (key `g`) in RViz top toolbar.
   - Click any open corridor and drag the arrow in the direction you want the robot to face.
   - Nav2 planner draws the global route (Red) and controller drives the robot along the local trajectory (Blue).

---

## 🧱 Costmap & Obstacle Avoidance System

- **Global Costmap (`/j100_0000/global_costmap/costmap`)**: Renders inflated safety buffers across static walls.
- **Local Costmap (`/j100_0000/local_costmap/costmap`)**: 3m × 3m rolling window updated dynamically at 10 Hz via LiDAR to evade dynamic obstacles.
- **Collision Footprint (`/j100_0000/local_costmap/published_footprint`)**: Real-time rectangular boundary of the Jackal chassis.

```bash
# Manually clear costmaps if needed:
ros2 service call /j100_0000/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{request: {}}"
ros2 service call /j100_0000/local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{request: {}}"
```

---

## 🧹 Process Management & Clean Reset

To ensure clean restarts and prevent port/topic collisions:
```bash
./scripts/clean_all.sh
```

---

## 📦 Modular Component Execution Table

| Component | Command |
|---|---|
| **1. All-in-One SLAM Mapping** | `./scripts/run_mapping.sh` |
| **2. Save SLAM Map** | `./scripts/save_map.sh maps/warehouse_map` |
| **3. Clean Reset Processes** | `./scripts/clean_all.sh` |
| **4. AMCL Localization** | `ros2 launch clearpath_nav2_demos localization.launch.py use_sim_time:=true map:=$(pwd)/maps/warehouse_map.yaml` |
| **5. Nav2 Autonomy Stack** | `ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true` |
| **6. YOLOv8 Vision Pipeline** | `ros2 launch jackal_vision vision.launch.py use_sim_time:=true` |
| **7. Multi-Waypoint Mission Node** | `ros2 run jackal_mission mission_node --ros-args -r __ns:=/j100_0000 -p use_sim_time:=true` |
| **8. Keyboard Teleop** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/j100_0000/cmd_vel` |

---

## 📋 Deliverables & Verification Matrix

| Deliverable | Location / Command | Verification |
|---|---|:---:|
| **Source Code & Workspace** | `src/jackal_vision/`, `src/jackal_mission/` | ✅ Clean build (`colcon build`) |
| **SLAM Occupancy Grid** | `maps/warehouse_map.yaml`, `maps/warehouse_map.pgm` | ✅ 0.05m resolution |
| **AMCL Localization** | `ros2 topic echo /j100_0000/amcl_pose` | ✅ Stable covariance & particles |
| **Nav2 Path Planning** | `ros2 topic echo /j100_0000/plan` | ✅ MPPI controller + Navfn |
| **YOLO Object Detection** | `ros2 topic echo /j100_0000/yolo_detector/detections` | ✅ `vision_msgs/Detection2DArray` |
| **Mission Reports** | `/tmp/jackal_mission_reports/mission_*.json`, `.txt` | ✅ Auto-generated summaries |
| **Technical Documentation** | `TECHNICAL_NOTE.md`, `TESTING_GUIDE.md` | ✅ Complete lab architecture notes |

---

## 📖 Documentation Reference

- **[TESTING_GUIDE.md](TESTING_GUIDE.md)**: Detailed step-by-step testing instructions, multi-terminal workflows, and evidence capture.
- **[TECHNICAL_NOTE.md](TECHNICAL_NOTE.md)**: In-depth technical note covering architecture, transforms (TF), message schemas, and design trade-offs.
