cmake_minimum_required(VERSION 3.5)
project(robot_desc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the directories (URDF, meshes, etc.)
install(
  DIRECTORY urdf meshes rviz launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
# Install the control script
install(PROGRAMS
  scripts/ijkl_control.py
  scripts/patrolbot_env.py
  scripts/ppo_agent.py
  scripts/goal_handler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()