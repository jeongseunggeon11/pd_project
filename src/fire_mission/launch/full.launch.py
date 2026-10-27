"""Whole scenario: simulation, Nav2, fire simulation, detection and the mission state machine.

Jetson-oriented defaults: no Gazebo GUI, no RViz. Override with gui:=true / rviz:=true for demos.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, args=None):
    """Include a launch file in its own scope, so launch arguments with the same name in
    different files (e.g. 'params_file') do not leak into each other."""
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                 launch_arguments=(args or {}).items())])


def generate_launch_description():
    # Wait until Gazebo has spawned the robot (create exits), however long the GUI takes
    # to load, so Nav2 and the rest do not compete with Gazebo during its startup.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'fire_robot',
                   '-x', '4.5', '-y', '2.0', '-z', '0.01', '-Y', '0.0'],
    )
    rest = [
        LogInfo(msg='Robot spawned, starting Nav2, fire simulation, detection and mission'),
        include('fire_robot_navigation', 'navigation.launch.py',
                {'rviz': LaunchConfiguration('rviz')}),
        include('fire_sim', 'fire_sim.launch.py',
                {'min_delay': LaunchConfiguration('min_delay'),
                 'max_delay': LaunchConfiguration('max_delay')}),
        include('fire_detection', 'fire_detection.launch.py'),
        include('fire_mission', 'mission.launch.py'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Gazebo GUI (false recommended on Jetson for RTF)'),
        DeclareLaunchArgument(
            'headless_rendering', default_value='false',
            description='Force EGL headless rendering; auto-on when gui:=false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('min_delay', default_value='30.0'),
        DeclareLaunchArgument('max_delay', default_value='120.0'),

        include('fire_robot_gazebo', 'sim.launch.py',
                {'gui': LaunchConfiguration('gui'),
                 'headless_rendering': LaunchConfiguration('headless_rendering'),
                 'spawn': 'false'}),
        spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=lambda event, _context: rest if event.returncode == 0 else [
                LogInfo(msg='Robot spawn failed, not starting the rest')])),
    ])
