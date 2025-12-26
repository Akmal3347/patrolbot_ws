cmake_minimum_required(VERSION 3.8)
project(robot_base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

# Motor Driver Node
add_executable(motor_driver_node src/motor_driver_node.cpp)
ament_target_dependencies(motor_driver_node rclcpp geometry_msgs)

# Odometry Node
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#Combined Motor + Odometry Node
add_executable(motor_odometry_node src/motor_odometry_node.cpp)
ament_target_dependencies(motor_odometry_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

# Thermal Camera Node
add_executable(thermal_camera_node src/thermal_camera_node.cpp)
ament_target_dependencies(thermal_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install
install(TARGETS
  motor_driver_node
  odometry_node
  motor_odometry_node 
  thermal_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

