from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Generate robot description from Xacro
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('robot_desc'),
                'urdf',
                'diff_robot.urdf.xacro'
            ])
        ])
    }

    # Path to RViz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'rviz',
        'robot.rviz'
    ])

    # Path to RPLIDAR launch
    rplidar_launch = PathJoinSubstitution([
        FindPackageShare('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    ])
    
    return LaunchDescription([
        # Launch RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
        ),

        # Motor driver node
        Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

        # Odometry node
        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # Robot State Publisher (from Xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"use_gui": False}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # Optional: teleop node (needs manual terminal input to control)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',  # opens in a separate terminal
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])