#!/usr/bin/env bash
# ==============================================================================
# SAFiR Lab - Project Zero: Process Cleanup Script
# Kills all simulation, ROS 2, Nav2, Gazebo, RViz, and vision nodes cleanly.
# ==============================================================================

echo ">>> Terminating all active ROS2, Gazebo, Nav2, and RViz processes..."
ps -ef | grep -E "ros|gz|ign|nav2|planner|controller|amcl|mission|yolo|rviz|rqt|slam" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null || true

sleep 1
echo ">>> Resetting ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true

echo ">>> All processes cleaned successfully!"
