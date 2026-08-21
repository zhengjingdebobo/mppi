#!/bin/bash
###############################################################################
# EXACT MPPI ROS2 Workspace - Build Script
#
# This script builds the ROS2 bridge workspace with recommended options.
#
# Usage:
#   chmod +x build.sh
#   ./build.sh
#
# Options:
#   ./build.sh clean       - Clean build (removes build/, install/, log/)
#   ./build.sh <package>   - Build specific package only
#
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get workspace root (script location)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}EXACT MPPI ROS2 Workspace Build${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

PYTHON_BIN="$(command -v python3 || command -v python)"

ensure_ros_python_build_deps() {
    if [ -z "$PYTHON_BIN" ]; then
        echo -e "${RED}✗ Python interpreter not found in PATH${NC}"
        exit 1
    fi

    local missing_packages=()

    if ! "$PYTHON_BIN" -c "import catkin_pkg" >/dev/null 2>&1; then
        missing_packages+=(catkin_pkg)
    fi
    if ! "$PYTHON_BIN" -c "import em" >/dev/null 2>&1; then
        missing_packages+=(empy)
    fi
    if ! "$PYTHON_BIN" -c "import lark" >/dev/null 2>&1; then
        missing_packages+=(lark)
    fi

    if [ ${#missing_packages[@]} -gt 0 ]; then
        echo -e "${YELLOW}Installing missing ROS Python build dependencies into:${NC} $PYTHON_BIN"
        "$PYTHON_BIN" -m pip install "${missing_packages[@]}"
    fi

    echo -e "${GREEN}✓ Using Python: $PYTHON_BIN${NC}"
}

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2 not sourced. Attempting to source ROS2 Kilted...${NC}"
    if [ -f "/opt/ros/kilted/setup.bash" ]; then
        source /opt/ros/kilted/setup.bash
        echo -e "${GREEN}✓ ROS2 Kilted sourced${NC}"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ ROS2 Humble sourced${NC}"
    else
        echo -e "${RED}✗ ROS2 not found. Please source ROS2 first:${NC}"
        echo -e "  ${YELLOW}source /opt/ros/kilted/setup.bash${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ Using ROS2 $ROS_DISTRO${NC}"
fi
echo ""

ensure_ros_python_build_deps
echo ""

# Handle command line arguments
if [ "$1" == "clean" ]; then
    echo -e "${YELLOW}Cleaning workspace...${NC}"
    rm -rf build/ install/ log/
    echo -e "${GREEN}✓ Workspace cleaned${NC}"
    echo ""
fi

# Build command
if [ -n "$1" ] && [ "$1" != "clean" ]; then
    # Build specific package
    PACKAGE=$1
    echo -e "${YELLOW}Building package: $PACKAGE${NC}"
    echo ""
    colcon build --packages-select "$PACKAGE" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PYTHON_BIN"
else
    # Build all packages
    echo -e "${YELLOW}Building all packages...${NC}"
    echo ""
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PYTHON_BIN"
fi

# Check build result
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Build Successful!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "To use the workspace, source the setup file:"
    echo -e "  ${YELLOW}source install/setup.bash${NC}"
    echo ""
    echo -e "Quick start:"
    echo -e "  ${YELLOW}source install/setup.bash${NC}"
    echo -e "  ${YELLOW}ros2 launch exact_mppi_jax sim_corridor_external_ref_launch.py${NC}"
    echo ""
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Build Failed!${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo -e "Please check the error messages above."
    echo -e "Common issues:"
    echo -e "  - Missing dependencies: run ${YELLOW}./setup.sh${NC}"
    echo -e "  - ROS2 not sourced: run ${YELLOW}source /opt/ros/kilted/setup.bash${NC}"
    echo ""
    exit 1
fi
