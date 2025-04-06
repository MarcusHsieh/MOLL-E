#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- LIDAR Launch ---
    # Include the LiDAR launch file from the ldlidar_sl_ros2 package.
    ldlidar_share = get_package_share_directory('ldlidar_sl_ros2')
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_share, 'launch', 'ld14p.launch.py')
        )
    )

    # --- IMU Launch ---
    # Include the IMU launch file from the mpu6050driver package.
    imu_share = get_package_share_directory('mpu6050driver')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_share, 'launch', 'mpu6050driver_launch.py')
        )
    )

    # --- Motor Controller Node ---
    # Launch the motor controller node from the motor_pkg package.
    motor_node = Node(
        package='motor_pkg',
        executable='motor_controller_node',
        name='motor_controller_node',
        output='screen'
        # parameters=[{'turn_scale': 1.0}]
    )

    ld = LaunchDescription()
    ld.add_action(ldlidar_launch)
    ld.add_action(imu_launch)
    ld.add_action(motor_node)

    return ld
