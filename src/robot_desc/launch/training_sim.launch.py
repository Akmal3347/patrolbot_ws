import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to your robot_desc package
    pkg_robot_desc = get_package_share_directory('robot_desc')
    
    # 2. Point specifically to your "diff_robot.urdf" file
    urdf_file_path = os.path.join(pkg_robot_desc, 'urdf', 'diff_robot.urdf')

    # 3. Launch Gazebo (Empty World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. Spawn the Robot
    # FIXED: Removed '-topic', 'robot_description'
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'patrolbot_robot_final',
                   '-file', urdf_file_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])