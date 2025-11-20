#!/bin/bash
# Setup script for ROS2 Balance Car Vision System

set -e

echo "=========================================="
echo "ROS2 Balance Car Vision System Setup"
echo "=========================================="

# Check if ROS2 is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS2 is not sourced. Please source ROS2 first:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Detected ROS2 distro: $ROS_DISTRO"

# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-rqt-image-view \
    v4l-utils

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Build the workspace
echo ""
echo "Building workspace..."
colcon build --symlink-install

# Source the workspace
echo ""
echo "Setup complete!"
echo ""
echo "To use the system, run:"
echo "  source install/setup.bash"
echo "  ros2 launch balance_car_full.launch.py"
echo ""
echo "For more information, see README.md"
