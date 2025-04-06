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

# Launch the LiDAR node
ros2 launch ldlidar_sl_ros2 ld14p.launch.py &
echo "LiDAR node launched."

# Launch the IMU node
ros2 launch mpu6050driver mpu6050driver_launch.py &
echo "IMU node launched."

# Launch the Motor Controller node
ros2 run motor_pkg motor_controller_node &
echo "Motor Controller node launched."

# Launch the SLAM Toolbox node
ros2 launch slam_toolbox online_sync_launch.py
echo "SLAM Toolbox launched."

wait