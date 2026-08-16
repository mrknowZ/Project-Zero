#!/usr/bin/env bash
# ==============================================================================
# SAFiR Lab — Project Zero: Save SLAM Map Script
# ==============================================================================
set -e

MAP_NAME="${1:-my_warehouse_map}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_DIR="${WORKSPACE_DIR}/maps"

mkdir -p "${MAP_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Saving map to ${MAP_DIR}/${MAP_NAME}..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" --ros-args -r __ns:=/j100_0000 -r map:=/j100_0000/map

echo "✓ Map successfully saved:"
echo "  - YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
echo "  - PGM:  ${MAP_DIR}/${MAP_NAME}.pgm"
