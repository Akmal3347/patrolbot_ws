from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load URDF using Xacro with ros2_control
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_desc'), 
            'urdf', 
            'robot.urdf.xacro'
        ])
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config',
        'diffbot_controllers.yaml'
    ])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Controller Manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controller_config],
            output='screen'
        ),

        # Spawner: joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Spawner: diff_drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diffbot_base_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])
