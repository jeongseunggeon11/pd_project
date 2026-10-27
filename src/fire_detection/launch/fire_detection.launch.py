import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fire_detection'), 'config', 'fire_detection.yaml')

    return LaunchDescription([
        Node(
            package='fire_detection',
            executable='fire_detector',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
    ])
