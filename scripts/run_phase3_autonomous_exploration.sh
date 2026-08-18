#!/bin/bash
set -e

# ==============================================================================
# Phase 3: Autonomous Frontier Exploration & Zero-Teleoperation Mapping
# Jackal autonomously discovers frontiers, builds maps, and monitors slope safety
# ==============================================================================

WORLD_NAME="${1:-warehouse}"

source /opt/ros/humble/setup.bash
source /home/holetown/ali/Project-Zero/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/holetown/ali/Project-Zero/install/share:/opt/ros/humble/share
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/holetown/ali/Project-Zero/install/share:/opt/ros/humble/share

echo ">>> Launching Phase 3 Autonomous Frontier Exploration Stack (World: ${WORLD_NAME})..."
ros2 launch launch/autonomous_exploration.launch.py world:=${WORLD_NAME}
