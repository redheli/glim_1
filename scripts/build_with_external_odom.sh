#!/bin/bash
# Build script for GLIM with external odometry support
# Run this inside the Docker container: koide3/glim_ros2:humble_cuda12.2

set -e

echo "=== Building GLIM with External Odometry Support ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Navigate to glim directory
cd /glim

# Build with colcon (ROS2)
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_WITH_VIEWER=ON

echo "=== Build Complete ==="
echo ""
echo "To use external odometry, update config/config.json:"
echo '  "config_odometry": "config_odometry_external.json"'
echo ""
echo "Then set pose_file_path in config/config_odometry_external.json"
