#!/bin/bash
set -e

# ==============================================================================
# RTAB-Map 3D Multi-Modal SLAM, Terrain Elevation & Slope Mapping Bringup
# ==============================================================================

source /opt/ros/humble/setup.bash
source /home/holetown/ali/Project-Zero/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/holetown/ali/Project-Zero/install/share:/opt/ros/humble/share
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/holetown/ali/Project-Zero/install/share:/opt/ros/humble/share

echo ">>> Launching RTAB-Map 3D Multi-Modal SLAM with Velodyne VLP-16 & Camera..."
ros2 launch launch/rtabmap_3d_system.launch.py "$@"
