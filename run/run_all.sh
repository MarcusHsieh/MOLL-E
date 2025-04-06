# Launch the LiDAR node
ros2 launch ldlidar_sl_ros2 ld14p.launch.py &
echo "LiDAR node launched."

# Launch the IMU node
ros2 launch mpu6050driver mpu6050driver_launch.py &
echo "IMU node launched."

# Launch the Motor Controller node
ros2 run motor_pkg motor_controller_node &
echo "Motor Controller node launched."

wait