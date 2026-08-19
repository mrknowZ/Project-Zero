#!/usr/bin/env bash
# ==============================================================================
# Clearpath Jackal J100 • Automatic Hardware Deployment & Setup Script
# Installs dependencies, sets up environment, and builds the workspace on real robot
# ==============================================================================

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}====================================================================${NC}"
echo -e "${GREEN}    🚀 CLEARPATH JACKAL J100 — ONBOARD DEPLOYMENT INSTALLER        ${NC}"
echo -e "${BLUE}====================================================================${NC}"

# 1. Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${GREEN}[1/6] Sourcing ROS 2 Humble...${NC}"
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}[ERROR] ROS 2 Humble is not installed in /opt/ros/humble!${NC}"
    exit 1
fi

# 2. Install Python AI and Web Dependencies
echo -e "${GREEN}[2/6] Installing Python Perception & Web Streaming packages...${NC}"
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python-headless tornado psutil

# 3. Install System ROS 2 Dependencies via rosdep
echo -e "${GREEN}[3/6] Resolving and installing ROS 2 system dependencies...${NC}"
sudo apt-get update || true
rosdep update || true
rosdep install -r --from-paths src -i -y || true

# 4. Cache YOLOv8 Model Weights locally
echo -e "${GREEN}[4/6] Verifying YOLOv8 model weights...${NC}"
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# 5. Clean Build Workspace
echo -e "${GREEN}[5/6] Building colcon workspace with symlink-install...${NC}"
colcon build --symlink-install

# 6. Setup Robot Environment in ~/.bashrc (if not already present)
echo -e "${GREEN}[6/6] Configuring environment in ~/.bashrc...${NC}"
BASHRC="$HOME/.bashrc"
WORKSPACE_SETUP="$(pwd)/install/setup.bash"

if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
fi

if ! grep -q "$WORKSPACE_SETUP" "$BASHRC"; then
    echo "source $WORKSPACE_SETUP" >> "$BASHRC"
fi

if ! grep -q "export ROS_DOMAIN_ID" "$BASHRC"; then
    echo "export ROS_DOMAIN_ID=42" >> "$BASHRC"
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$BASHRC"
fi

echo -e "\n${BLUE}====================================================================${NC}"
echo -e "${GREEN}    ✅ INSTALLATION COMPLETE & READY FOR PHYSICAL FLIGHT!           ${NC}"
echo -e "${BLUE}====================================================================${NC}"
echo -e "To start on the real Jackal robot:"
echo -e "  ${YELLOW}source install/setup.bash${NC}"
echo -e "  ${YELLOW}ros2 launch launch/advanced_system.launch.py sim:=false use_sim_time:=false setup_path:=/etc/clearpath/${NC}"
echo -e "\nAccess the Web Mission Control Dashboard from your phone or laptop:"
echo -e "  ${GREEN}http://<JACKAL_IP>:8080${NC}"
