import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_dir = get_package_share_directory('fire_robot_gazebo')
    description_dir = get_package_share_directory('fire_robot_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world = os.path.join(gazebo_dir, 'worlds', 'house.sdf')
    bridge_config = os.path.join(gazebo_dir, 'config', 'bridge.yaml')

    gui = LaunchConfiguration('gui')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    gz_launch = os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('spawn', default_value='true',
                              description='Spawn the robot here (false: the parent launch spawns it)'),
        DeclareLaunchArgument('x', default_value='4.5'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Gazebo server (+ GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r -s {world}'}.items(),
            condition=UnlessCondition(gui),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'description.launch.py')),
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'fire_robot',
                       '-x', x, '-y', y, '-z', '0.01', '-Y', yaw],
            condition=IfCondition(LaunchConfiguration('spawn')),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        ),
        # Simulation-side tools (fire simulation, spray effect): Gazebo spawn/remove
        # services and ground-truth poses. Robot logic must not use these.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sim_tools_bridge',
            output='screen',
            arguments=[
                '/world/house/create@ros_gz_interfaces/srv/SpawnEntity',
                '/world/house/remove@ros_gz_interfaces/srv/DeleteEntity',
                '/world/house/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[('/world/house/dynamic_pose/info', '/sim/ground_truth_poses')],
        ),
    ])
