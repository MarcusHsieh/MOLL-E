#!/bin/bash
# Launch Docker container for MOLL-E development on Jetson Nano

# Exit
set -e

echo "--- Setting up variables for Docker ---"

# Mount host MOLL-E folder into container
HOST_MOLLE_PATH=../../MOLL-E
CONTAINER_MOLLE_PATH=/MOLL-E

# --- Configuration ---
ROS_DOMAIN_ID_TO_USE=30
# DOCKER_IMAGE="dustynv/ros:humble-ros-core-l4t-r32.7.1"
DOCKER_IMAGE="molle-num3"
# --- End Configuration ---

echo "Host MOLL-E Folder: ${HOST_MOLLE_PATH}"
echo "Container MOLL-E Folder: ${CONTAINER_MOLLE_PATH}"
echo "Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID_TO_USE}"
echo "Using Docker Image: ${DOCKER_IMAGE}"
echo "----------------------------------------"

# --- Pre-run Checks ---
if [ ! -d "${HOST_MOLLE_PATH}" ]; then
  echo "Error: Host folder '${HOST_MOLLE_PATH}' not found."
  echo "Please create it first (e.g., mkdir -p ${HOST_MOLLE_PATH})"
  exit 1
fi
echo "--- Pre-run checks passed ---"

echo "--- Launching Docker Container ---"
docker run -it --rm \
    --runtime nvidia \
    --net=host \
    --ipc=host \
    --privileged \
    -v "${HOST_MOLLE_PATH}:${CONTAINER_MOLLE_PATH}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/i2c-1:/dev/i2c-1 \
    -v /dev/ttyACM0:/dev/ttyACM0 \
    -e DISPLAY=${DISPLAY} \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TO_USE} \
    ${DOCKER_IMAGE}

echo "--- Docker container exited ---"
