from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
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

    # Map YAML path (from robot_maps package)
    map_yaml_file = os.path.join(
        get_package_share_directory('robot_maps'),
        'maps',
        'simple_map.yaml'
    )

    # Nav2 params (you can fine-tune later)
    nav2_params_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'nav2.yaml'
    )
    
    return LaunchDescription([
        #Motor driver node
        #Node(
        #    package='robot_base',
        #    executable='motor_driver_node',
        #    name='motor_driver_node',
        #    output='screen'
        #),

        #Odometry node
        #Node(
        #    package='robot_base',
        #    executable='odometry_node',
        #    name='odometry_node',
        #    output='screen'
        #),
        
        #Motor + Odometry node (merged)
        Node(
             package='robot_base',
             executable='motor_odometry_node',
             name='motor_odometry_node',
              output='screen',
              parameters=[{
                  'invert_left': False,
                  'invert_right': False
               }]
        ),
        
        # Thermal Camera Node
        #Node(
        #    package='robot_base',
        #    executable='thermal_camera_node',
        #    name='thermal_camera_node',
        #    output='screen'
        #),
        
        # Lidar Node
          Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 1000000,
                'frame_id': 'lidar_link',
                'inverted': False,
                'flip_x_axis':False,
                'angle_compensate': True,
            }],
        ),


        # Robot State Publisher (using URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # Joint State Publisher (headless)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': False}]
        ),
        
        
        # SLAM Toolbox
        #Node(
        #    package='slam_toolbox',
        #    executable='async_slam_toolbox_node',
        #    name='slam_toolbox',
        #    output='screen',
        #    parameters=[PathJoinSubstitution([
        #        FindPackageShare('robot_bringup'),
        #        'config',
        #        'slam_toolbox.yaml'
        #  ])]
        #),

        
        # Map Server (static map)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),

        # AMCL (localization using the static map)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file]
        ),

       
        # Nav2 Planner
        #Node(
        #    package='nav2_planner',
        #    executable='planner_server',
        #    name='planner_server',
        #    output='screen',
        #    parameters=[nav2_params_file]
        #),

        # Nav2 Controller
        #Node(
        #    package='nav2_controller',
        #    executable='controller_server',
        #    name='controller_server',
        #    output='screen',
        #    parameters=[nav2_params_file]
        #),

        # Nav2 BT Navigator
        #Node(
        #    package='nav2_bt_navigator',
        #    executable='bt_navigator',
        #    name='bt_navigator',
        #    output='screen',
        #    parameters=[nav2_params_file]
        #),
       
        # Nav2 Behaviors
        #Node(
        #    package='nav2_behaviors',
        #    executable='behavior_server',
        #    name='behavior_server',
        #    output='screen',
        # parameters=[nav2_params_file]
        # 
        #),
       
        # Lifecycle Manager
        #Node(
        #    package='nav2_lifecycle_manager',
        #    executable='lifecycle_manager',
        #    name='lifecycle_manager_navigation',
        #    output='screen',
        #   parameters=[{
        #        'use_sim_time': False,
        #        'autostart': True,
        #        'node_names': [
        #            'map_server',
        #            'amcl',
        #            'planner_server',
        #            'controller_server',
        #            'bt_navigator',
        #            'behavior_server'
        #        ]
        #    }]
        #),
         
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
            prefix='xterm -e',
            output='screen',
           # remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])

# === Delay Nav2 stack startup (5 seconds) ===
    delayed_nav2_nodes = TimerAction(
        period=5.0,
        actions=[
            map_server_node,
            amcl_node,
            planner_node,
            controller_node,
            bt_navigator_node,
            behavior_node,
            lifecycle_node
        ]
    )

    # === Final Launch Description ===
    return LaunchDescription([
        motor_odometry_node,
        lidar_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        delayed_nav2_nodes,
        rviz_node,
        teleop_node
    ])
