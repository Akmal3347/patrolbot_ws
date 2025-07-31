from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory  
import os

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='', description='Robot namespace'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz_config', 
        default_value=os.path.join(
            get_package_share_directory('robot_bringup'), 
            'rviz', 
            'robot.rviz'
        ),
        description='Rviz config file'
    )
    
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,
        rviz_arg,

        Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            namespace=ns,
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600
            }]
        ),

        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            namespace=ns,
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600,
                'wheel_radius': 0.033,
                'wheel_base': 0.16,
                'ticks_per_revolution': 3200
            }]
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            namespace=ns,
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=ns,
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
