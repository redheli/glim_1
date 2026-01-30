#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

# Source glim workspace if it exists
if [ -f /glim/install/setup.bash ]; then
    source /glim/install/setup.bash
fi

# Check if we should run offline_viewer or start interactive shell
if [ "$1" = "offline_viewer" ]; then
    shift  # Remove 'offline_viewer' from arguments
    exec ros2 run glim_ros offline_viewer "$@"
elif [ "$1" = "glim" ]; then
    shift  # Remove 'glim' from arguments
    exec ros2 run glim_ros glim_ros "$@"
else
    # Default: start interactive bash shell
    exec /bin/bash
fi
