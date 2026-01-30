#!/bin/bash
# Helper script to build and run the custom glim docker container
#
# Usage:
#   ./run_glim_custom.sh                    # Interactive shell
#   ./run_glim_custom.sh offline_viewer     # Run offline_viewer directly
#   ./run_glim_custom.sh glim               # Run glim_ros directly
#   ./run_glim_custom.sh --build            # Rebuild image and start interactive shell
#   ./run_glim_custom.sh --build offline_viewer  # Rebuild image and run offline_viewer

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKERFILE_PATH="${SCRIPT_DIR}/../docker/Dockerfile.custom"
IMAGE_NAME="glim_custom:humble_cuda12.2"
CONTAINER_NAME="glim"

# Parse --build flag
BUILD_FLAG=""
if [[ "$1" == "--build" ]]; then
    BUILD_FLAG="--build"
    shift
fi

# Remaining arguments (e.g., offline_viewer, glim, or nothing for interactive)
CMD_ARGS="$@"

# Build the custom image if it doesn't exist or if --build flag is passed
if [[ "${BUILD_FLAG}" == "--build" ]] || [[ "$(docker images -q ${IMAGE_NAME} 2> /dev/null)" == "" ]]; then
    echo "Building custom glim image..."
    docker build -t ${IMAGE_NAME} -f ${DOCKERFILE_PATH} ${SCRIPT_DIR}/../docker/
fi

# Remove existing container with the same name if it exists
docker rm -f ${CONTAINER_NAME} 2>/dev/null

# Run the container
docker run -it --rm \
    --net=host \
    --ipc=host \
    --pid=host \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e ROS_IP=127.0.0.1 \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,video,display \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/dri:/dev/dri \
    -v /home/max/ground_map/koide3/glim/:/glim/ \
    -v /home/max/:/home_max \
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME} ${CMD_ARGS}
