from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load URDF file content (IMPORTANT!)
    urdf_file = os.path.join(
        get_package_share_directory('robot_desc'),
        'urdf',
        'diff_robot.urdf' 
    )
    with open(urdf_file, 'r') as infp:
        robot_description = {'robot_description': infp.read()}

    # Load controller config YAML
    import yaml
    controller_yaml = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'diffbot_controllers.yaml'
    )
    with open(controller_yaml, 'r') as f:
        controller_params = yaml.safe_load(f)

    return LaunchDescription([
        # Load robot description from URDF
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # Start ros2_control_node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            parameters=[robot_description, controller_params],
            output='screen'
        ),

        # Spawner: joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Spawner: diffbot_base_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diffbot_base_controller'],
            output='screen'
        ),
    ])