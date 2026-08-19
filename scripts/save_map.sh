#!/usr/bin/env bash
# ==============================================================================
# SAFiR Lab — Project Zero: Save SLAM Map Script
# ==============================================================================
set -e

MAP_NAME="${1:-outdoor_map}"
NAMESPACE="${2:-j100_0751}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_DIR="${WORKSPACE_DIR}/maps"

mkdir -p "${MAP_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash" || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

echo "============================================================"
echo "Saving SLAM map to: ${MAP_DIR}/${MAP_NAME}"
echo "Namespace: /${NAMESPACE}"
echo "Domain ID: ${ROS_DOMAIN_ID}"
echo "============================================================"

ros2 run nav2_map_server map_saver_cli \
    -f "${MAP_DIR}/${MAP_NAME}" \
    --ros-args -r __ns:="/${NAMESPACE}" -r map:="/${NAMESPACE}/map"

echo ""
echo "✅ Map successfully saved!"
echo "  • YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
echo "  • PGM:  ${MAP_DIR}/${MAP_NAME}.pgm"
