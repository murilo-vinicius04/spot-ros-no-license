#!/bin/bash

# Spot ROS2 Docker Runner Script
# Provides GUI support, GPU access, and proper volume mounting

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🤖 Spot ROS2 Docker Runner${NC}"
echo -e "${BLUE}=============================${NC}"

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    echo -e "${RED}❌ Docker is not running. Please start Docker first.${NC}"
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "docker-compose.yaml" ]; then
    echo -e "${RED}❌ docker-compose.yaml not found. Please run this script from the spot_ws directory.${NC}"
    exit 1
fi

# Set up X11 forwarding for GUI applications
echo -e "${YELLOW}🖥️  Setting up X11 forwarding...${NC}"
xhost +local:docker >/dev/null 2>&1 || echo -e "${YELLOW}⚠️  Warning: Could not set xhost permissions${NC}"

# Build the container if it doesn't exist
echo -e "${YELLOW}🔨 Building/updating container...${NC}"
docker-compose build

# Function to run the container with all necessary permissions
run_container() {
    local command="$1"
    
    docker run -it --rm \
        --name spot-ros2-runtime \
        --hostname spot-ros2 \
        --network host \
        --privileged \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XAUTHORITY=$XAUTH" \
        --env="BOSDYN_CLIENT_USERNAME=${BOSDYN_CLIENT_USERNAME:-admin}" \
        --env="BOSDYN_CLIENT_PASSWORD=${BOSDYN_CLIENT_PASSWORD:-your_password}" \
        --env="SPOT_IP=${SPOT_IP:-192.168.80.3}" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$PWD:/workspaces/spot_ws:rw" \
        --volume="/dev:/dev:rw" \
        --device="/dev/dri:/dev/dri" \
        --gpus all 2>/dev/null || \
    docker run -it --rm \
        --name spot-ros2-runtime \
        --hostname spot-ros2 \
        --network host \
        --privileged \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="BOSDYN_CLIENT_USERNAME=${BOSDYN_CLIENT_USERNAME:-admin}" \
        --env="BOSDYN_CLIENT_PASSWORD=${BOSDYN_CLIENT_PASSWORD:-your_password}" \
        --env="SPOT_IP=${SPOT_IP:-192.168.80.3}" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$PWD:/workspaces/spot_ws:rw" \
        --volume="/dev:/dev:rw" \
        --device="/dev/dri:/dev/dri" \
        spot_ws_spot-ros2:latest \
        bash -c "cd /workspaces/spot_ws && source install/setup.bash && $command"
}

# Check command line arguments
case "${1:-shell}" in
    "shell" | "bash")
        echo -e "${GREEN}🚀 Starting interactive shell...${NC}"
        echo -e "${BLUE}📍 Working directory: /workspaces/spot_ws${NC}"
        echo -e "${BLUE}🔧 ROS2 environment will be sourced automatically${NC}"
        echo ""
        run_container "bash"
        ;;
    "driver")
        echo -e "${GREEN}🚀 Starting Spot driver...${NC}"
        run_container "ros2 launch spot_driver spot_driver.launch.py launch_rviz:=true arm:=true"
        ;;
    "control")
        echo -e "${GREEN}🚀 Starting ros2_control with arm...${NC}"
        run_container "ros2 launch spot_ros2_control spot_ros2_control_with_arm.launch.py hardware_interface:=robot"
        ;;
    "mock")
        echo -e "${GREEN}🚀 Starting mock simulation...${NC}"
        run_container "ros2 launch spot_ros2_control spot_ros2_control_with_arm.launch.py hardware_interface:=mock launch_rviz:=true"
        ;;
    "rviz")
        echo -e "${GREEN}🚀 Starting RViz only...${NC}"
        run_container "rviz2"
        ;;
    "build")
        echo -e "${GREEN}🔨 Building workspace...${NC}"
        run_container "colcon build --symlink-install"
        ;;
    "test")
        echo -e "${GREEN}🧪 Running tests...${NC}"
        run_container "colcon test && colcon test-result --verbose"
        ;;
    *)
        echo -e "${GREEN}🚀 Running custom command: $*${NC}"
        run_container "$*"
        ;;
esac

# Cleanup X11 permissions
echo -e "${YELLOW}🧹 Cleaning up X11 permissions...${NC}"
xhost -local:docker >/dev/null 2>&1 || true

echo -e "${GREEN}✅ Done!${NC}" 