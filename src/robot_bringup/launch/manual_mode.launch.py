from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare 

def generate_launch_description():
    # Generate robot description using xacro
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_desc'),
            'urdf',
            'robot.urdf.xacro'
        ])
    ])

    # RPLIDAR launch file path
    rplidar_launch = PathJoinSubstitution([
        FindPackageShare('rplidar_ros'),
        'launch',
        'rplidar.launch.py'
    ])

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
            output='screen',
        ),

        # Odometry node
        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),

        # RPLiDAR node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),

        # Robot state publisher (for URDF from xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # RViz (optional for visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
