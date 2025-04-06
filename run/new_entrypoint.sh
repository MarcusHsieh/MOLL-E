#!/bin/bash
set -e

# SOURCE BEFORE RUNNING COLCON BUILD
source /opt/ros/humble/setup.bash

if [ -f /MOLL-E/install/local_setup.bash ]; then
  source /MOLL-E/install/local_setup.bash
fi

echo "-------------------------"
echo "Running colcon build..."
cd /MOLL-E
colcon build
echo "colcon build complete."
echo "-------------------------"

echo "Sourcing workspace install/setup.bash..."
if [ -f /MOLL-E/install/setup.bash ]; then
    source /MOLL-E/install/setup.bash
else
    echo "Warning: /MOLL-E/install/setup.bash not found."
fi
echo "-------------------------"

echo "Calling original ROS entrypoint..."

exec /ros_entrypoint.sh "$@"
