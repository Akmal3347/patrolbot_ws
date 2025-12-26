#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Default configurations for RPLidar S2
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB1")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="1000000")  # ✅ S2 baudrate
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="Standard")  # safer than DenseBoost
    motor_enable = LaunchConfiguration("motor_enable", default="true")

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("channel_type", default_value=channel_type,
                              description="Specifying channel type of lidar"),
        DeclareLaunchArgument("serial_port", default_value=serial_port,
                              description="Specifying USB port connected to lidar"),
        DeclareLaunchArgument("serial_baudrate", default_value=serial_baudrate,
                              description="Specifying USB port baudrate for lidar"),
        DeclareLaunchArgument("frame_id", default_value=frame_id,
                              description="Specifying frame_id of lidar"),
        DeclareLaunchArgument("inverted", default_value=inverted,
                              description="Whether to invert scan data"),
        DeclareLaunchArgument("angle_compensate", default_value=angle_compensate,
                              description="Whether to enable angle compensation"),
        DeclareLaunchArgument("scan_mode", default_value=scan_mode,
                              description="Specifying scan mode of lidar"),
        DeclareLaunchArgument("motor_enable", default_value=motor_enable,
                              description="Whether to enable lidar motor"),

        # RPLidar Node
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[{
                "channel_type": channel_type,
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "scan_mode": scan_mode,
                "motor_enable": motor_enable,
                "use_ros_time": True   # ✅ Only added line
            }],
            output="screen"
        ),
    ])

