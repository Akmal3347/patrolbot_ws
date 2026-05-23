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

    # --- NEW: Define the path to your Custom World file ---
    # option A: If you saved it in your home folder
    world_file_path = '/home/akmalzarif/patrolbot_tall_world.world' 
    
    # option B: If you saved it inside the package (Best Practice)
    # world_file_path = os.path.join(pkg_robot_desc, 'worlds', 'patrolbot_tall_world.world')
    # ------------------------------------------------------

    # 3. Launch Gazebo (With Custom World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        # --- ADD THIS ARGUMENT ---
        launch_arguments={'world': world_file_path}.items()
    )

    # 4. Spawn the Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'patrolbot_robot_final',
                   '-file', urdf_file_path,
                   '-z', '0.05'], # Lift robot slightly so it doesn't spawn in floor
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
