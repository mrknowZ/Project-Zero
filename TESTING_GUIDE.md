# Project Zero — Comprehensive Testing & Execution Guide

Step-by-step commands to run, test, and **capture evidence** for the Clearpath Jackal autonomy and vision pipeline in Gazebo Harmonic simulation and on physical robot hardware.

---

## 0. Build & Environment Preparation

Before launching any nodes, build the workspace and install required dependencies:

```bash
cd ~/ali/Project-Zero
source /opt/ros/humble/setup.bash
pip install ultralytics opencv-python-headless tornado
rosdep install -r --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

---

## 0.1 Clean Reset & Process Management (Reproducibility)

> **Important for Reproducibility**: Before starting a new test run or if a previous simulation was interrupted, always reset lingering background processes and ROS 2 daemons to ensure a 100% clean test environment:

```bash
# Run the all-in-one cleanup script:
./scripts/clean_all.sh
```

---

## 1. Outdoor Farm & 3D Autonomy Stack (Advanced Phase)

Launch the Jackal J100 in the outdoor farm simulation (`orchard` or `solar_farm`) with the 3D Voxel Costmap, Slope Traversability Safety, Semantic Social Keepout, RTAB-Map 3D SLAM, and Web Mission Control:

```bash
# Launch in orchard environment (or pass 'solar_farm')
./scripts/run_outdoor_farm_test.sh orchard
```

### Web Mission Control Dashboard (`http://localhost:8080`)
Open your web browser at **`http://localhost:8080`**.

#### Features Available:
1. **Live Dual-Stream Video**: Real-time camera feed with YOLOv8 bounding boxes and fallback to raw optical stream.
2. **Manual Driving & Tactile D-Pad**:
   * Use on-screen D-Pad or keyboard keys (<kbd>W</kbd>, <kbd>A</kbd>, <kbd>S</kbd>, <kbd>D</kbd>, <kbd>Space</kbd>).
   * **Speed Multipliers**: Toggle between **Slow (0.4 m/s)**, **Normal (0.7 m/s)**, and **Fast (1.0 m/s)**.
3. **Manual Override & Autonomy Interlock**:
   * **Default Startup**: The robot initializes in **Manual / Idle Mode** (`auto_start_exploration: false`) — it will **never wander or dispatch goals without your command**.
   * **Instant Human Takeover**: Driving manually automatically pauses autonomous exploration and routes velocity via `rc_teleop/cmd_vel` (Priority 12 in `twist_mux`), preempting Nav2.
   * **`[ ✋ Manual Override ]`** (Amber Button): Pauses autonomous exploration, cancels active Nav2 goals, and yields 100% control to the operator.
   * **`[ ⚡ Resume Auto Exploration ]`** (Blue Button): Dispatches autonomous frontier exploration to map unknown areas.
   * **`[ 🛑 EMERGENCY STOP ]`** (Red Button): Locks `twist_mux` via `/j100_0000/platform/emergency_stop` (Priority 255), engages software brake, and halts motion instantly.
4. **Waypoint Navigation Goal Dispatcher**:
   * Enter Target $X$ and $Y$ coordinates and click **Send Goal** to dispatch an autonomous Nav2 waypoint directly from the web browser.
5. **Terrain Inclinometer & Attitude Horizon**:
   * Real-time roll/pitch attitude indicator on an HTML5 `<canvas>` instrument.
   * Total terrain slope angle indicator with **SAFE** (<15°), **CAUTION** (15-25°), and **DANGER** (>25°) safety chips.
6. **3D Semantic Spatial Inventory**:
   * Live table of detected objects (persons, vehicles, plants) with 3D map coordinates $(X, Y, Z)$ and detection counts.
7. **Map Saver**:
   * Click **`💾 Save SLAM Map to Disk`** to write 2D/3D maps to `maps/jackal_farm_map`.

---

## 2. Warehouse Top-Level Launch (ONE COMMAND — Complete System)

Run the entire baseline autonomy and vision pipeline in the warehouse world:

```bash
cd ~/ali/Project-Zero
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jackal_mission system.launch.py
```

> **What happens automatically:**
> 1. Gazebo Harmonic starts with the Jackal robot in the warehouse environment.
> 2. The simulation clock is unpaused automatically after 5 seconds.
> 3. Pointcloud-to-laserscan node bridges 3D LiDAR into 2D laser scan topics.
> 4. Map server and AMCL load `warehouse_map.yaml` and initialize robot localization.
> 5. Nav2 autonomy controllers, planners, costmaps, and behavior trees activate.
> 6. YOLOv8 object detector begins real-time inference on the onboard camera.
> 7. RViz2 visualizer opens displaying the map, 3D Jackal chassis, costmaps, LiDAR, and YOLO feed.
> 8. Autonomous mission orchestrator begins waypoint navigation with obstacle avoidance.

---

## 3. Interactive Navigation Mode (Manual 2D Goal Pose & Pose Estimation)

To launch the baseline stack in interactive mode (allowing you to manually set initial poses and send navigation goals via RViz):

```bash
ros2 launch jackal_mission system.launch.py mission:=false
```

### In RViz2:
1. **Initial Pose (Localization)**:
   * Click **`2D Pose Estimate`** (key `p`) in the top toolbar.
   * Click at `(0, 0)` on the map and drag the green arrow **Right ($+X$ axis)**.
   * Watch the green AMCL particle cloud snap tightly around the Jackal model.
2. **Nav Goal (Autonomous Driving)**:
   * Click **`2D Goal Pose`** / **`Nav2 Goal`** (key `g`) in the top toolbar.
   * Click on any reachable open space (e.g. `X = 2.5, Y = 1.0`) and drag in the desired heading direction.
   * Watch the global path (Red) and local MPPI trajectory (Blue) drive the robot to the goal!

---

## 4. Real Jackal Hardware Deployment

To run the complete system on the physical Jackal robot in the lab:

```bash
# 1. Connect controller and ensure safety clearance
# 2. Run system launch on real robot
ros2 launch jackal_mission system.launch.py \
    sim:=false \
    use_sim_time:=false \
    setup_path:=/etc/clearpath/ \
    map:=/path/to/lab_map.yaml
```

---

## 5. Manual PlayStation (PS4 / PS5) Controller Driving

Clearpath robots use a **Deadman Safety Switch** (the robot will **only move** while the enable button is held down):

| Action | Controller Button / Axis |
|---|---|
| **Drive (Normal Speed)** | **Hold `L1`** + Push **Left Thumbstick** (Up/Down for Speed, Left/Right for Steering) |
| **Drive (Turbo Speed)** | **Hold `R1`** + Push **Left Thumbstick** |
| **Instant Stop** | **Release `L1` / `R1`** |

---

## 6. SLAM Mapping (Generate New Maps)

### Option A: All-in-One Mapping Bringup
```bash
./scripts/run_mapping.sh
```
Teleoperate the robot around the environment to construct the map. Once complete, save the map in a new terminal:
```bash
./scripts/save_map.sh maps/my_new_map
```

---

## 7. Verification & Evidence Capture Commands

### A. Check Manual Override & Velocity Multiplexer
```bash
# Check raw teleoperation output
ros2 topic echo /j100_0000/rc_teleop/cmd_vel --once

# Check platform odometry velocity
ros2 topic echo /j100_0000/platform/odom --once
```

### B. Check Exploration Status & Command Bus
```bash
# Echo current autonomy status
ros2 topic echo /j100_0000/exploration/status --once

# Dispatch manual pause command via CLI
ros2 topic pub --once /j100_0000/exploration/command std_msgs/msg/String "{data: 'PAUSE'}"

# Dispatch resume exploration command via CLI
ros2 topic pub --once /j100_0000/exploration/command std_msgs/msg/String "{data: 'START'}"
```

### C. Check Inclinometer & Slope Safety
```bash
ros2 topic echo /j100_0000/slope_safety/telemetry --once
ros2 topic echo /j100_0000/slope_safety/status --once
```

### D. Check 3D Semantic Inventory
```bash
ros2 topic echo /j100_0000/semantic_map/objects_json --once
```

### E. Check Localization & Transforms
```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/j100_0000/tf -r /tf_static:=/j100_0000/tf_static
```

### F. Check Vision Pipeline (YOLOv8)
```bash
ros2 topic echo /j100_0000/yolo_detector/detections --once
ros2 run rqt_image_view rqt_image_view /j100_0000/yolo_detector/detections_image
```

---

## 8. Evidence Checklist for Deliverables

| Requirement | Description | Evidence Output |
|---|---|---|
| **Req 1** | Clean ROS2 Workspace | Build succeeds cleanly via `colcon build` |
| **Req 2** | Sensor Integration & TF | `frames_*.pdf`, `slam_topics.txt` |
| **Req 3** | Mapping (SLAM Toolbox / RTAB-Map) | `maps/warehouse_map.yaml`, `maps/jackal_farm_map.yaml` |
| **Req 4** | Localization (AMCL / RTAB-Map) | Particle cloud, `amcl_pose.txt` |
| **Req 5** | Navigation (Nav2 & 3D Voxel Costmaps) | Goal trajectory, costmap inflation, `nav2_feedback.txt` |
| **Req 6** | Vision (YOLOv8 & 3D Semantic Mapping) | `yolo_detections.txt`, bounding box stream, spatial inventory |
| **Req 7** | Mission Control & Web Dashboard | Port 8080 Telemetry Bridge, Manual Override interlock, Inclinometer |
