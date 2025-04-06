import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        os.path.expanduser('~/MOLL-E/src/robot_localization_config'),
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_localization_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([ekf_node])
