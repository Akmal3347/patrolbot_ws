cmake_minimum_required(VERSION 3.5)
project(robot_desc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
