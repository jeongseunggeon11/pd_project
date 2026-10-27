import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fire_mission'), 'config', 'mission.yaml')

    return LaunchDescription([
        Node(
            package='fire_mission',
            executable='mission_node',
            output='screen',
            emulate_tty=True,
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='fire_mission',
            executable='spray_nozzle',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
