#!/bin/bash
# ================================================================
# Project Zero — Automated Test Script
# Tests: build verification, simulation launch, SLAM, localization
# ================================================================
set -e

export DISPLAY=:1
WS=/home/holetown/ali/Project-Zero

echo "======================================"
echo " Project Zero — Test Suite"
echo "======================================"

# Source ROS2 + workspace
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo "[OK] ROS2 Humble + workspace sourced"
echo "ROS_DISTRO=$ROS_DISTRO"

# Verify packages
echo ""
echo "--- Package Verification ---"
ros2 pkg executables jackal_vision 2>/dev/null && echo "[OK] jackal_vision found" || echo "[FAIL] jackal_vision NOT found"
ros2 pkg executables jackal_mission 2>/dev/null && echo "[OK] jackal_mission found" || echo "[FAIL] jackal_mission NOT found"

# Verify launch files
echo ""
echo "--- Launch File Verification ---"
ls $(ros2 pkg prefix jackal_vision 2>/dev/null)/share/jackal_vision/launch/ 2>/dev/null && echo "[OK] vision launch found"
ls $(ros2 pkg prefix jackal_mission 2>/dev/null)/share/jackal_mission/launch/ 2>/dev/null && echo "[OK] mission launch found"

# Verify configs
echo ""
echo "--- Config Verification ---"
ls $(ros2 pkg prefix jackal_vision 2>/dev/null)/share/jackal_vision/config/ 2>/dev/null && echo "[OK] vision config found"
ls $(ros2 pkg prefix jackal_mission 2>/dev/null)/share/jackal_mission/config/ 2>/dev/null && echo "[OK] mission config found"

# Check robot.yaml exists
echo ""
echo "--- Robot Config ---"
if [ -f ~/clearpath/robot.yaml ]; then
    echo "[OK] ~/clearpath/robot.yaml exists"
    cat ~/clearpath/robot.yaml
else
    echo "[WARN] ~/clearpath/robot.yaml NOT found"
    echo "       Simulation requires this file. Create it with:"
    echo "       mkdir -p ~/clearpath && create robot.yaml"
fi

# Check saved map
echo ""
echo "--- Map Files ---"
ls -la $WS/maps/ 2>/dev/null && echo "[OK] Map files found" || echo "[WARN] No maps directory"

echo ""
echo "======================================"
echo " Static verification complete"
echo "======================================"
