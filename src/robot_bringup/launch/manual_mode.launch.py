import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load URDF file content
    urdf_path = os.path.join(
        get_package_share_directory('robot_desc'),
        'urdf',
        'diff_robot.urdf'
    )
    with open(urdf_path, 'r') as file:
        robot_description_content = file.read()

    # RViz config file path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'rviz',
        'robot.rviz'
    ])

    # Motor Driver Node (start after 0.5s)
    motor_node = TimerAction(
        period=0.5,
        actions=[Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
        )]
    )

    # Odometry Node (start after 1s)
    odom_node = TimerAction(
        period=1.0,
        actions=[Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        )]
    )

    # Robot State Publisher Node (start after 2s)
    state_pub_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False
            }]
        )]
    )

    # RViz2 Node (start after 4s)
    rviz_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )]
    )

    return LaunchDescription([
        motor_node,
        odom_node,
        state_pub_node,
        rviz_node,
    ])
