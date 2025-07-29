cmake_minimum_required(VERSION 3.8)
project(robot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(/usr/include)


# Executables
add_executable(motor_driver_node src/motor_driver_node.cpp)
add_executable(odometry_node src/odometry_node.cpp)
#add_executable(arduino_serial_node src/arduino_serial_node.cpp)

# Link dependencies for each target
ament_target_dependencies(motor_driver_node
  rclcpp
  geometry_msgs

)

ament_target_dependencies(odometry_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

#ament_target_dependencies(arduino_serial_node
# rclcpp
#)


# Install targets
install(TARGETS
  motor_driver_node
  odometry_node
#  arduino_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch or other resources (if any)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
