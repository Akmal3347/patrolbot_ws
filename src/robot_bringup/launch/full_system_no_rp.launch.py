from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF path directly
    urdf_path = os.path.join(
        get_package_share_directory('robot_desc'),
        'urdf',
        'diff_robot.urdf'
    )

    # Read the URDF file
    with open(urdf_path, 'r') as file:
        robot_description = {'robot_description': file.read()}

    # RViz config path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'rviz',
        'robot.rviz'
    ])

    return LaunchDescription([
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

        # Robot State Publisher (using URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # Teleop twist keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',  # opens in a new terminal
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
