import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('fire_sim'), 'config', 'fire_sim.yaml')
    params = LaunchConfiguration('fire_params_file')
    min_delay = ParameterValue(LaunchConfiguration('min_delay'), value_type=float)
    max_delay = ParameterValue(LaunchConfiguration('max_delay'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('fire_params_file', default_value=default_params),
        DeclareLaunchArgument('min_delay', default_value='30.0'),
        DeclareLaunchArgument('max_delay', default_value='120.0'),

        # Uses the Gazebo spawn/remove services and /sim/ground_truth_poses bridged by
        # fire_robot_gazebo/sim.launch.py
        Node(
            package='fire_sim',
            executable='fire_generator',
            output='screen',
            parameters=[params, {'use_sim_time': True,
                                 'min_delay': min_delay, 'max_delay': max_delay}],
        ),
        Node(
            package='fire_sim',
            executable='fire_manager',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
    ])
