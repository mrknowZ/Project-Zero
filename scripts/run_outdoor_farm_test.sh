#!/bin/bash
# ==============================================================================
# Run Jackal J100 Outdoor Farm Simulation Test
# Environment: orchard (or solar_farm)
# Stack: 3D Voxel Costmap + Slope Traversability + Semantic Social Zones + Web Dashboard
# ==============================================================================

set -e

WORLD=${1:-orchard}

echo "======================================================================"
echo " Starting Outdoor Farm Autonomy Stack in World: ${WORLD}"
echo " Web Dashboard: http://localhost:8080"
echo "======================================================================"

# 1. Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /home/holetown/ali/Project-Zero/install/setup.bash

# 2. Launch full system stack with farm world
ros2 launch /home/holetown/ali/Project-Zero/launch/advanced_system.launch.py \
    world:=${WORLD} \
    use_sim_time:=true \
    auto_start_exploration:=true
