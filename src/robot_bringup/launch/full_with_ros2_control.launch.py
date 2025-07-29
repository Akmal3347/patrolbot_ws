from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    robot_desc_pkg = get_package_share_directory('robot_desc')
    bringup_pkg = get_package_share_directory('robot_bringup')

    # Load URDF with ros2_control
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([robot_desc_pkg, 'urdf', 'robot.urdf.xacro'])
    ])

    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Controller manager spawner
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            PathJoinSubstitution([bringup_pkg, 'config', 'diffbot_controllers.yaml'])
        ],
        output='screen'
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_base_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_description_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
