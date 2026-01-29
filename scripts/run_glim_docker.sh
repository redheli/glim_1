#!/bin/bash
# Run GLIM Docker container with proper mounts for external odometry workflow
#
# Usage: ./run_glim_docker.sh [container_name]

CONTAINER_NAME=${1:-glim-dev}

docker run --rm -it --privileged --net=host --ipc=host --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e XAUTHORITY=/root/.Xauthority \
  -v /home/max/ground_map/koide3/glim:/glim \
  -v /home/max/ground_map:/data \
  -v /home/max/ground_map/annotator:/annotator \
  -w /glim \
  --name $CONTAINER_NAME \
  koide3/glim_ros2:humble_cuda12.2 \
  /bin/bash
