import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot_desc = get_package_share_directory('robot_desc')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    urdf_file_path = os.path.join(pkg_robot_desc, 'urdf', 'diff_robot.urdf')
    world_path = os.path.expanduser('~/patrolbot_ws1/patrolbot_ws/src/robot_desc/worlds/patrolbot_tall_world.world')

    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    # FIX: Set environment variables so Gazebo can find your STL meshes
    mesh_path = os.path.join(pkg_robot_desc, 'meshes')
    resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_gazebo_ros, 'share'), ':', mesh_path]
    )
    model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[pkg_robot_desc, ':', mesh_path]
    )

    # 1. Gazebo (starts first — everything else depends on it)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Robot state publisher (can start immediately alongside Gazebo)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Spawn robot into Gazebo (delay 3 s to let Gazebo finish loading)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'patrolbot_robot_final',
                    '-file',   urdf_file_path,
                    '-x', '-2.0',
                    '-y',  '0.6',   # matches goal_handler phase-1 reset position
                    '-z',  '0.0',  # Low spawn height to prevent flipping
                    '-Y',  '0.0',
                ],
                output='screen'
            )
        ]
    )

    # 4. Goal handler (delay 4 s — needs Gazebo services to be available)
    goal_handler = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_desc',
                executable='goal_handler.py',
                name='goal_handler',
                output='screen'
            )
        ]
    )

    # 5. RL Environment node (delay 5 s — needs goal_handler and robot to be up)
    rl_environment = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_desc',
                executable='patrolbot_env.py',
                name='rl_environment',
                output='screen'
            )
        ]
    )

    # 6. PPO Agent (delay 8 s — needs all services from rl_environment to be ready)
    ppo_agent = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='robot_desc',
                executable='ppo_agent.py',
                name='ppo_agent',
                output='screen',
                arguments=['1', '2000', '200']
            )
        ]
    )

    return LaunchDescription([
        resource_path,
        model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        goal_handler,
        rl_environment,
        # ppo_agent,
    ])