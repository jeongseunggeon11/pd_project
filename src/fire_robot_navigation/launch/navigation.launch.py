import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_dir = get_package_share_directory('fire_robot_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    common_args = {
        'map': map_yaml,
        'params_file': params_file,
        'use_sim_time': 'true',
        'autostart': 'true',
    }.items()

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(nav_dir, 'maps', 'house.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'localization_launch.py')),
            launch_arguments=common_args,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments=common_args,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
            condition=IfCondition(rviz),
        ),
    ])
