# Project Zero — SLAM & Localization Testing Guide

Step-by-step commands to test each component and **capture evidence** for your deliverables.

> Run all commands on your ROS2 Humble machine (Ubuntu 22.04) with a display for RViz2/Gazebo.

---

## 0. Build Everything First

```bash
cd ~/Project-Zero   # or wherever your workspace is
source /opt/ros/humble/setup.bash
pip install ultralytics opencv-python-headless
rosdep install -r --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

---

## 1. Test SLAM (Mapping) — Capture Evidence

### Start simulation
```bash
# Terminal 1
ros2 launch clearpath_gz simulation.launch.py
```

### Start the pointcloud-to-laserscan bridge
```bash
# Terminal 2
source install/setup.bash
ros2 launch launch/pointcloud_to_laserscan.launch.py
```

### Start SLAM
```bash
# Terminal 3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 launch clearpath_nav2_demos slam.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/
```

### Teleop the robot to build the map
```bash
# Terminal 4
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/j100_0000/cmd_vel
```

### 📸 Evidence to Capture (SLAM)

**a) Screenshot of RViz2 showing the map being built**
- Open RViz2, add displays: Map, LaserScan, RobotModel, TF
- Take a screenshot while the map is partially built
- Take another after full map is built

**b) List active topics**
```bash
ros2 topic list | tee slam_topics.txt
```

**c) Check map topic is publishing**
```bash
ros2 topic echo /j100_0000/map --once | head -20 | tee slam_map_sample.txt
```

**d) Check scan topic**
```bash
ros2 topic echo /j100_0000/sensors/lidar2d_0/scan --once | head -20 | tee slam_scan_sample.txt
```

**e) Save the TF tree**
```bash
ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true
# Produces frames_YYYY-MM-DD_HH.MM.SS.pdf
```

**f) Save the map**
```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_new_map \
    --ros-args -p use_sim_time:=true
```

---

## 2. Test Localization (AMCL) — Capture Evidence

### Stop SLAM (Ctrl+C Terminal 3). Keep simulation + pointcloud bridge running.

### Start localization with the saved map
```bash
# Terminal 3 (reuse)
source install/setup.bash
ros2 launch clearpath_nav2_demos localization.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/ \
    map:=$(pwd)/maps/warehouse_map.yaml
```

### 📸 Evidence to Capture (Localization)

**a) Screenshot of RViz2 showing AMCL particle cloud**
- In RViz2, add: Map, LaserScan, PoseArray (`/j100_0000/particle_cloud`)
- Set initial pose via RViz2's "2D Pose Estimate" tool
- Screenshot showing particles converging on the robot's position

**b) Check AMCL is publishing**
```bash
ros2 topic echo /j100_0000/amcl_pose --once | tee amcl_pose.txt
```

**c) Check the map → odom transform exists**
```bash
ros2 run tf2_ros tf2_echo map odom --ros-args -r /tf:=/j100_0000/tf -r /tf_static:=/j100_0000/tf_static
```
Take a screenshot of this output (should show a valid transform updating).

**d) Drive around and verify localization tracks**
Use teleop again, drive a loop, and screenshot the particle cloud staying converged.

---

## 3. Test Navigation (Nav2) — Capture Evidence

### Start Nav2 (in addition to localization)
```bash
# Terminal 5
source install/setup.bash
ros2 launch clearpath_nav2_demos nav2.launch.py \
    use_sim_time:=true \
    setup_path:=$HOME/clearpath/
```

### 📸 Evidence to Capture (Navigation)

**a) Send a goal via RViz2**
- Use the "Nav2 Goal" tool in RViz2
- Screenshot showing the planned path and the robot following it

**b) Test obstacle avoidance**
- In Gazebo, place an object in the robot's path
- Screenshot showing the robot replanning around the obstacle

**c) Test recovery behaviour**
- Send a goal that requires navigating a tight space
- Screenshot/note showing spin/backup recovery

**d) Log navigation feedback**
```bash
ros2 topic echo /j100_0000/navigate_to_pose/_action/feedback --once | tee nav2_feedback.txt
```

---

## 4. Test Vision (YOLO) — Capture Evidence

> **Prerequisite**: You need a camera sensor in `robot.yaml` and image topics publishing.

```bash
# Terminal 2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=j100_0000
```

### 📸 Evidence to Capture (Vision)

**a) Check detections topic**
```bash
ros2 topic echo /j100_0000/yolo_detector/detections --once | tee yolo_detections.txt
```

**b) View annotated image in RViz2**
- Add Image display for `/j100_0000/yolo_detector/detections_image`
- Screenshot showing bounding boxes on detected objects

---

## 5. Full Mission Test — Capture Evidence

```bash
# Terminal: start full pipeline (sim must already be running)
source install/setup.bash
ros2 launch jackal_mission mission.launch.py \
    use_sim_time:=true \
    map:=$(pwd)/maps/warehouse_map.yaml
```

### 📸 Evidence to Capture (Mission)

**a) Screenshot of RViz2 showing the robot navigating between waypoints**

**b) Mission report files**
```bash
ls -la /tmp/jackal_mission_reports/
cat /tmp/jackal_mission_reports/mission_*.txt
```
Include the report content in your technical note.

---

## Quick Evidence Checklist

| Evidence Item | File/Screenshot | Requirement |
|---|---|---|
| Map in RViz2 (building) | screenshot | Req 3 |
| Map in RViz2 (complete) | screenshot | Req 3 |
| Saved map files (.pgm + .yaml) | `maps/` | Req 3 |
| SLAM topics list | `slam_topics.txt` | Req 2 |
| TF tree PDF | `frames_*.pdf` | Req 2 |
| AMCL particle cloud | screenshot | Req 4 |
| AMCL pose output | `amcl_pose.txt` | Req 4 |
| map→odom transform | screenshot | Req 4 |
| Nav2 path planning | screenshot | Req 5 |
| Obstacle avoidance | screenshot | Req 5 |
| YOLO detections | screenshot + `yolo_detections.txt` | Req 6 |
| Annotated image in RViz2 | screenshot | Req 6 |
| Mission report | `/tmp/jackal_mission_reports/` | Req 7 |
| Robot navigating waypoints | screenshot | Req 7 |
