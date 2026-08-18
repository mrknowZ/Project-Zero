#!/bin/bash
# ==============================================================================
# Phase 4: Full Advanced Autonomy, 3D Semantic Mapping & Web/Tablet Telemetry UI
# ==============================================================================

set -e

WORLD=${1:-warehouse}

echo "======================================================================"
echo "  JACKAL J100 • ADVANCED AUTONOMOUS SYSTEM & WEB MISSION CONTROL"
echo "  - 3D LiDAR SLAM & Elevation Mapping"
echo "  - 6-DoF Terrain Slope & Inclinometer Safety Monitor"
echo "  - Autonomous Frontier Exploration & Zero-Teleop Mapping"
echo "  - YOLOv8 Object Detection & 3D Semantic Spatial Landmark Registry"
echo "  - Web/Tablet Real-Time Mission Control Dashboard: http://localhost:8080"
echo "======================================================================"
echo ""
echo ">>> Environment: $WORLD"

# 1. Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /home/holetown/ali/Project-Zero/install/setup.bash

# 2. Launch full system stack
ros2 launch /home/holetown/ali/Project-Zero/launch/advanced_system.launch.py \
  world:=$WORLD \
  use_sim_time:=true \
  auto_start_exploration:=true
