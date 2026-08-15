# Project Zero — Technical Note

**SAFiR Lab · Clearpath Jackal (J100) Autonomy Pipeline**

**Stack:** ROS2 Humble · Ubuntu 22.04 · Gazebo Harmonic

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Gazebo Harmonic (Sim)                         │
│  ┌────────────┐  ┌──────────────┐  ┌───────────────────────────┐     │
│  │  Jackal    │  │  Velodyne    │  │   Camera (simulated RGB)  │     │
│  │  J100      │  │  VLP-16     │  │                           │     │
│  └─────┬──────┘  └──────┬──────┘  └────────────┬──────────────┘     │
│        │                │                       │                     │
└────────┼────────────────┼───────────────────────┼─────────────────────┘
         │ /cmd_vel       │ /sensors/             │ /sensors/
         │                │  lidar3d_0/points     │  camera_0/color/image
         │                ▼                       │
         │   ┌────────────────────────┐           │
         │   │ pointcloud_to_         │           │
         │   │   laserscan_node       │           │
         │   │ (3D PointCloud → 2D)   │           │
         │   └──────────┬─────────────┘           │
         │              │ /sensors/               │
         │              │  lidar2d_0/scan         │
         │              ▼                         ▼
         │   ┌─────────────────────┐   ┌─────────────────────┐
         │   │   SLAM Toolbox      │   │   jackal_vision     │
         │   │   (mapping mode)    │   │   (YOLOv8 detector) │
         │   │      — or —         │   │                     │
         │   │   AMCL              │   │  → Detection2DArray │
         │   │   (localization)    │   │  → annotated image  │
         │   └──────────┬──────────┘   └──────────┬──────────┘
         │              │ /map                    │ ~/detections
         │              ▼                         │
         │   ┌─────────────────────┐              │
         │   │       Nav2          │              │
         │   │  (planner +         │              │
         │   │   controller +      │              │
         │   │   recovery)         │              │
         │   └──────────┬──────────┘              │
         │              │                         │
         ▼              ▼                         ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        jackal_mission                                │
│              (waypoint sequencer + report generator)                 │
│  • Sends goals via Nav2 BasicNavigator API                          │
│  • Collects detections at each waypoint                              │
│  • Generates JSON + text summary report                              │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 2. Package Structure

```
Project-Zero/                          (colcon workspace root)
├── src/
│   ├── clearpath_common/              Upstream: URDF, control, platform packages
│   ├── clearpath_simulator/           Upstream: Gazebo Harmonic integration
│   ├── clearpath_nav2_demos/          Upstream: SLAM / Nav2 launch + configs (j100)
│   │   └── config/j100/
│   │       ├── localization.yaml      AMCL & MapServer parameter tuning
│   │       └── nav2.yaml              Nav2 stack configuration & plugin classes
│   ├── jackal_vision/                 ★ YOLO object detection package
│   │   ├── jackal_vision/
│   │   │   └── yolo_detector_node.py  YOLOv8 ROS2 node (vision_msgs output)
│   │   ├── config/
│   │   │   ├── yolo_params.yaml       Model weights, inference rate, classes
│   │   │   └── mission_viz.rviz       Pre-configured RViz with Nav2 + Camera + YOLO
│   │   └── launch/
│   │       ├── vision.launch.py       Detector bringup
│   │       └── rviz.launch.py         Namespaced RViz2 visualizer bringup
│   └── jackal_mission/                ★ Mission orchestrator & top-level bringup
│       ├── jackal_mission/
│       │   └── mission_node.py        Autonomous navigator & report generator
│       ├── config/
│       │   ├── waypoints.yaml         Waypoint coordinates & dwell durations
│       │   ├── small_mission.yaml     Obstacle avoidance test waypoints
│       │   └── mission_params.yaml    Report directory & lifecycle configs
│       └── launch/
│           ├── system.launch.py       ★ ONE-COMMAND top-level full system launch
│           ├── mission.launch.py      Mission stack bringup (Nav2 + AMCL + YOLO)
│           └── rviz.launch.py         RViz2 launch helper
├── launch/
│   ├── system.launch.py               Root top-level launch alias
│   └── pointcloud_to_laserscan.launch.py
├── maps/
│   ├── warehouse_map.pgm              Occupancy grid image
│   └── warehouse_map.yaml             Map metadata
├── IMPLEMENTATION.md                  Workspace & architecture notes
├── TECHNICAL_NOTE.md                  (this document)
├── TESTING_GUIDE.md                   Step-by-step test & verification guide
└── README.md                          Project overview & guidelines
```

---

## 3. Key Design Choices

### 3.1 Pointcloud-to-LaserScan Bridge
The Jackal's `robot.yaml` configures a Velodyne VLP-16 as a `lidar3d` sensor, but Nav2 and AMCL expect 2D scan data on `sensors/lidar2d_0/scan`. Rather than modifying upstream robot description packages, we run `pointcloud_to_laserscan_node` to convert the 3D point cloud into a virtual 2D scan slice ($\pm0.2\,\text{m}$, $20\,\text{m}$ max range).

### 3.2 SLAM Toolbox for Mapping
SLAM Toolbox was selected for generating the warehouse occupancy grid due to its tight integration with Nav2 lifecycle nodes and out-of-the-box loop-closure stability.

### 3.3 AMCL for Localization
AMCL (Adaptive Monte Carlo Localization) is deployed with a likelihood field model ($500 - 2000$ particles) to publish the `map → odom` transform. The `mission_node` sets the initial pose estimate programmatically on startup with `(0,0)` timestamps to ensure seamless transform lookups under simulation time.

### 3.4 Nav2 for Navigation
Nav2 coordinates path planning and motion control using the MPPI controller. Recovery behaviors (spin, backup, wait) are fully enabled to handle dynamic or transient warehouse obstacles.

### 3.5 YOLOv8 for Object Detection
- **Model**: YOLOv8n (nano) running locally on CPU/GPU.
- **Classes**: Person, backpack, bicycle, chair, car, truck, plant (configurable in `yolo_params.yaml`).
- **Standard Interface**: Publishes `vision_msgs/Detection2DArray` and annotated camera frames on `~/detections_image`.

### 3.6 Mission Orchestrator & Reporting
`mission_node` utilizes the `BasicNavigator` API to sequence navigation through waypoints (`shelf_a`, `shelf_b`, `loading_dock`), dwell for object detection, and generate machine-readable JSON and human-readable text reports in `/tmp/jackal_mission_reports/`.

---

## 4. Simulation Clock Synchronization Requirement

When running in simulation (`use_sim_time:=true`), ROS 2 nodes synchronize their internal timers, TF lookups, and lifecycle transitions to the `/clock` topic published by Gazebo. 

Because Gazebo launches paused by default (allowing robot meshes and ROS controllers to finish loading into memory), `system.launch.py` automatically triggers the clock unpause service after 5 seconds:

```bash
ign service -s /world/warehouse/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'
```

---

## 5. Deployment Instructions

### Mode 1: ONE-COMMAND Top-Level Bringup (Recommended)

```bash
cd ~/ali/Project-Zero
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launches Gazebo, auto-unpauses clock, starts AMCL, Nav2, YOLO, RViz2, Camera View, and Mission
ros2 launch jackal_mission system.launch.py
```

*Optional Launch Flags:*
* `sim:=false use_sim_time:=false` — Run on physical Jackal hardware.
* `mission:=false` — Keep autonomous navigation in manual/RViz goal-setting mode.
* `camera_view:=false` — Disable separate high-resolution rqt camera window.
* `rviz:=false` — Run headless.

---

### Mode 2: Individual / Step-by-Step Component Execution

```bash
# 1. Simulation
ros2 launch clearpath_gz simulation.launch.py

# 2. Clock Unpause
ign service -s /world/warehouse/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'

# 3. 3D-to-2D LiDAR Bridge
ros2 launch launch/pointcloud_to_laserscan.launch.py

# 4. Localization (AMCL + Map Server)
ros2 launch clearpath_nav2_demos localization.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/ \
    map:=$(pwd)/maps/warehouse_map.yaml

# 5. Nav2 Stack
ros2 launch clearpath_nav2_demos nav2.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/

# 6. YOLOv8 Object Detection
ros2 launch jackal_vision vision.launch.py namespace:=j100_0000 use_sim_time:=true

# 7. RViz2 Visualizer
ros2 run rviz2 rviz2 \
    -d src/jackal_vision/config/mission_viz.rviz \
    --ros-args -r __ns:=/j100_0000 -p use_sim_time:=true

# 8. Mission Sequencer
ros2 run jackal_mission mission_node --ros-args -r __ns:=/j100_0000 -p use_sim_time:=true
```

---

### Visualization Utilities

* **Standalone Camera & YOLO GUI**:
  ```bash
  ros2 run rqt_image_view rqt_image_view /j100_0000/yolo_detector/detections_image
  ```
* **TF Tree Inspection**:
  ```bash
  ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true
  ```
