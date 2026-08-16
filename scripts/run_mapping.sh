#!/usr/bin/env bash
# ==============================================================================
# SAFiR Lab — Project Zero: SLAM Mapping Bringup
# ==============================================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "======================================================================"
echo " Starting SAFiR Lab SLAM Mapping Pipeline"
echo " - Gazebo Harmonic Warehouse Simulation"
echo " - 3D LiDAR PointCloud-to-LaserScan Converter"
echo " - SLAM Toolbox Online Mapping"
echo " - Nav2 Autonomy Stack (for setting goals while mapping)"
echo " - RViz2 Visualizer (Live Map Display)"
echo "======================================================================"
echo "Use the '2D Goal Pose' tool in RViz2 to navigate around the warehouse,"
echo "or run teleop in another terminal: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/j100_0000/cmd_vel"
echo "To save the map when done: ./scripts/save_map.sh <map_name>"
echo "======================================================================"

ros2 launch jackal_mission mapping.launch.py
