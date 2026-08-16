# Project Zero — Comprehensive Testing & Execution Guide

Step-by-step commands to run, test, and **capture evidence** for the Clearpath Jackal autonomy and vision pipeline in Gazebo Harmonic simulation and on physical robot hardware.

---

## 0. Build & Environment Preparation

Before launching any nodes, build the workspace and install required dependencies:

```bash
cd ~/ali/Project-Zero
source /opt/ros/humble/setup.bash
pip install ultralytics opencv-python-headless
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

## 1. Top-Level Launch (ONE COMMAND — Complete System)

Run the entire autonomy and vision pipeline with a single command:

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

## 2. Interactive Navigation Mode (Manual 2D Goal Pose & Pose Estimation)

To launch the entire stack in interactive mode (allowing you to manually set initial poses and send navigation goals):

```bash
ros2 launch jackal_mission system.launch.py mission:=false
```

### In RViz2:
1. **Initial Pose (Localization)**:
   - Click **`2D Pose Estimate`** (key `p`) in the top toolbar.
   - Click at `(0, 0)` on the map and drag the green arrow **Right ($+X$ axis)**.
   - Watch the green AMCL particle cloud snap tightly around the Jackal model.
2. **Nav Goal (Autonomous Driving)**:
   - Click **`2D Goal Pose`** / **`Nav2 Goal`** (key `g`) in the top toolbar.
   - Click on any reachable open space (e.g. `X = 2.5, Y = 1.0`) and drag in the desired heading direction.
   - Watch the global path (Red) and local MPPI trajectory (Blue) drive the robot to the goal!

---

## 3. Real Jackal Hardware Deployment

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

## 4. Manual PlayStation (PS4 / PS5) Controller Driving

Clearpath robots use a **Deadman Safety Switch** (the robot will **only move** while the enable button is held down):

| Action | Controller Button / Axis |
|---|---|
| **Drive (Normal Speed)** | **Hold `L1`** + Push **Left Thumbstick** (Up/Down for Speed, Left/Right for Steering) |
| **Drive (Turbo Speed)** | **Hold `R1`** + Push **Left Thumbstick** |
| **Instant Stop** | **Release `L1` / `R1`** |

---

## 5. SLAM Mapping (Generate New Maps)

### Option A: All-in-One Mapping Bringup
```bash
./scripts/run_mapping.sh
```
Teleoperate the robot around the environment to construct the map. Once complete, save the map in a new terminal:
```bash
./scripts/save_map.sh maps/my_new_map
```

---

## 6. Verification & Evidence Capture Commands

### A. Check Localization (AMCL)
```bash
ros2 topic echo /j100_0000/amcl_pose --once
ros2 run tf2_ros tf2_echo map odom --ros-args -r /tf:=/j100_0000/tf -r /tf_static:=/j100_0000/tf_static
```

### B. Check Costmaps & Obstacle Avoidance
```bash
ros2 topic hz /j100_0000/global_costmap/costmap
ros2 topic hz /j100_0000/local_costmap/costmap
ros2 topic hz /j100_0000/local_costmap/published_footprint
```

### C. Check Vision Pipeline (YOLOv8)
```bash
ros2 topic echo /j100_0000/yolo_detector/detections --once
ros2 run rqt_image_view rqt_image_view /j100_0000/yolo_detector/detections_image
```

### D. Check Mission Reports
Mission reports are saved automatically in `/tmp/jackal_mission_reports/`:
```bash
cat /tmp/jackal_mission_reports/mission_*.txt
cat /tmp/jackal_mission_reports/mission_*.json
```

---

## 7. Evidence Checklist for Deliverables

| Requirement | Description | Evidence Output |
|---|---|---|
| **Req 1** | Clean ROS2 Workspace | Build succeeds cleanly via `colcon build` |
| **Req 2** | Sensor Integration & TF | `frames_*.pdf`, `slam_topics.txt` |
| **Req 3** | Mapping (SLAM Toolbox) | `maps/warehouse_map.yaml`, `maps/warehouse_map.pgm` |
| **Req 4** | Localization (AMCL) | Particle cloud, `amcl_pose.txt` |
| **Req 5** | Navigation (Nav2) | Goal trajectory, costmap inflation, `nav2_feedback.txt` |
| **Req 6** | Vision (YOLOv8) | `yolo_detections.txt`, bounding box stream |
| **Req 7** | System Integration | `/tmp/jackal_mission_reports/mission_*.json`, `.txt` |
