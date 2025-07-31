from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # using xacro to read the URDF file
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
    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'rviz',
        'robot.rviz'
    ])

    return LaunchDescription([
        # # Launch RPLIDAR (commented out)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(rplidar_launch),
        # ),

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

        # Robot State Publisher (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]

        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # Optional: teleop node (keyboard control)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',  # opens a new terminal window for keyboard input
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
