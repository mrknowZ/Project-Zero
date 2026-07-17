# SAFiR Lab – Project Zero

## Objective

Project Zero ensures every new student has the baseline technical skills required to do research in the SAFiR Lab. By the end, you should be able to:

- Develop and maintain a ROS2 workspace.
- Work effectively with Git and Linux.
- Deploy and debug robotic software on real hardware.
- Build maps using SLAM.
- Localize and navigate autonomously.
- Integrate camera and LiDAR data.
- Perform basic object detection.
- Document and reproduce your work.

**Duration:** ~2 weeks. **Mode of work:** mostly autonomous. This document tells you *what* to achieve and *what tools the lab uses*, not how to use them step by step. Reading the official docs of each tool is part of the exercise; that's how research in this lab actually works.

If you get stuck for more than ~half a day on something that feels like a setup/environment problem rather than a real technical question, ask. Don't burn days on a broken install.

## Stack

So you don't waste time guessing versions or chasing ROS1 tutorials:

- **ROS2 distro:** Humble, on Ubuntu 22.04 or Jazzy, on Ubuntu 24.04.
- **Simulator:** Gazebo Harmonic, via Clearpath's own packages: [clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator)  (brings up the robot in Gazebo) and [clearpath_nav2_demos](https://github.com/clearpathrobotics/clearpath_nav2_demos) (SLAM/Nav2 launch files and configs already tuned for our platforms). Start from these rather than building your own launch files from scratch. In the future, you'll likely use AirSim (based on the Unreal Engine) or Unity, but for now, we're going to keep it simple.
- **Hardware:** Clearpath Jackal, 2D LiDAR (Velodyne VLP-16), RGBD camera (yet to be integrated: Luxonis OAK-D-S2-FF), onboard computer.

## Platform

Develop in simulation first. Final validation must run on the real Jackal.

**Before you touch the physical robot:** get the lab's hardware safety briefing (deadman switch location and use, safe floor clearance, who's around when you test).

## Task

Build a complete autonomy pipeline where the robot can:

1. Build a map of an unknown environment.
2. Localize within that map.
3. Navigate autonomously to user-defined goals.
4. Detect predefined objects with the onboard camera.
5. Be packaged so another lab member can run it without your help.

## Requirements

### 1. ROS2 Workspace
A clean workspace: proper package structure, launch files, config files, documentation. The whole system should start with a minimal number of commands: one launch file per major capability, ideally one top-level launch file for the full demo.

### 2. Sensor Integration
Verify and document: camera topics, LiDAR topics, the TF tree, and relevant calibration parameters. Show correct visualization in RViz2 (`ros2 run tf2_tools view_frames` is a fast way to sanity-check the TF tree).

### 3. Mapping
Generate an occupancy map with [**SLAM Toolbox**](https://github.com/SteveMacenski/slam_toolbox). Deliver: the saved map, the launch procedure, and a short note on the parameters you chose and why (e.g. resolution, max laser range). [RTAB-Map](https://github.com/introlab/rtabmap) is a valid alternative.

### 4. Localization
Deploy localization on the generated map. It must handle pose initialization, recover from localization errors, and stay stable while navigating. [**AMCL**](https://docs.nav2.org/configuration/packages/configuring-amcl.html) is the standard choice; **SLAM Toolbox's localization mode** is a valid alternative.

### 5. Navigation
Deploy autonomous navigation with [**Nav2**](https://docs.nav2.org/). The robot must reach user-defined goals, avoid obstacles, and recover from common navigation failures (stuck planner, transient obstacle blocking the path, etc.). Test this on purpose: put something in the way and watch what happens.

### 6. Vision
Detect at least three object categories (e.g. backpack, car, bicycle, person). Any modern detector is fine (YOLO and similar are the default choice). Publish detections over ROS2. Use `vision_msgs` (e.g. `Detection2D`) rather than a custom message type, so anyone else in the lab can consume your output without reading your code first.

### 7. System Integration
A single demonstrable mission: start the robot → load the map → localize → navigate to multiple waypoints → detect objects encountered along the way → report results (e.g., a log, a summary message, a simple file, a video).

## Deliverables

**Repository**: source code, launch files, config files, README.

**Technical note (~2 pages)**: system architecture, package structure, key design choices, known limitations, deployment instructions.

**Live demonstration**: 10–15 minutes on the Jackal.

## Definition of Done

A lab member unfamiliar with your implementation should be able to: clone the repo → follow the README → launch the complete system → reproduce the demonstration.

**If it can't be reproduced by someone else, the project isn't done, regardless of what you got working live.**

## Suggested Pacing (guideline, not a schedule to follow rigidly)

- **End of week 1:** sensors verified, mapping and localization working in simulation, first autonomous goal reached.
- **Mid week 2:** vision pipeline running, full mission integrated in simulation.
- **End of week 2:** ported to the real Jackal, repo and technical note finalized, demo-ready.

## Optional Extensions

If you finish early, pick one: frontier exploration, semantic mapping, visual place recognition, open-vocabulary object detection, dynamic obstacle handling, multi-robot mapping, learning-based navigation, or integration with Air Sim / Unity / Isaac Sim.

Have fun!
