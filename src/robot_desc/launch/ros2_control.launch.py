from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('robot_desc')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    controller_yaml = PathJoinSubstitution([pkg_share, 'config', 'test_controllers.yaml'])

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            parameters=[
                {
                    "robot_description": Command([
                        FindExecutable(name='xacro'),
                        ' ',
                        xacro_file,
                        ' ', 
                        'use_mock:=true'
                    ])
                },
                controller_yaml
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])
