cmake_minimum_required(VERSION 3.16)
project(robot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependency list
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${Dependency} REQUIRED)
endforeach()

# Create shared library
add_library(robot_hardware SHARED
  src/diffbot_system.cpp
)

# C++17 support
target_compile_features(robot_hardware PUBLIC cxx_std_17)

# Include paths (FIXED)
target_include_directories(robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencies
ament_target_dependencies(robot_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Optional: link serial if you're using it
target_link_libraries(robot_hardware PUBLIC serial)

# DLL visibility macro for Windows builds
target_compile_definitions(robot_hardware PRIVATE "ROBOT_HARDWARE_BUILDING_DLL")

# Export plugin description (FIXED filename)
pluginlib_export_plugin_description_file(hardware_interface robot_hardware_plugin.xml)

# Install the shared library
install(TARGETS robot_hardware
  EXPORT export_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files (FIXED path)
install(
  DIRECTORY include/
  DESTINATION include
)


# Optional: setup for testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# Export
ament_export_targets(export_robot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
