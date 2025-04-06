# Make sure ROS 2 Humble environment sourced
source /opt/ros/humble/install/setup.bash

# for joystick
# pip3 install websockets

echo "-------------------------"
echo "Running colcon build..."
cd /MOLL-E
colcon build --symlink-install
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

# Camera
ros2 run camera_ros2 camera_node
echo "Camera node launched."

# wget https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js -O web/nipplejs.min.js
# Joystick
# ros2 run web_joy_publisher web_joy_node
# echo "Joystick node launched."

# Launch the Motor Controller node
ros2 run motor_ros2 motor_controller_node &
echo "Motor Controller node launched."

# Launch the SLAM Toolbox node
ros2 launch slam_toolbox online_sync_launch.py &
echo "SLAM Toolbox launched."

# Launch rosboard node
ros2 run rosboard rosboard_node &
echo "rosboard launched."

wait