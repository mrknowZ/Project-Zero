#!/bin/bash
set -e

# ==============================================================================
# Phase 2: 3D Terrain, Slope Traversability & Inclinometer Safety Monitor
# Generalized for Indoor Warehouses and Outdoor Rough / Sloped Environments
# ==============================================================================

WORLD_NAME="${1:-warehouse}"

source /opt/ros/humble/setup.bash
source /home/holetown/ali/Project-Zero/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/holetown/ali/Project-Zero/install/share:/opt/ros/humble/share
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/holetown/ali/Project-Zero/install/share:/opt/ros/humble/share

echo ">>> Launching 3D Terrain & Slope Traversability System (Environment: ${WORLD_NAME})..."
ros2 launch launch/view_3d_terrain.launch.py world:=${WORLD_NAME}
