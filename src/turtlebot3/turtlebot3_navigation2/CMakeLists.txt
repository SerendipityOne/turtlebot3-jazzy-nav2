################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(initial_pose_publisher
  src/initial_pose_message.cpp
  src/initial_pose_publisher.cpp
)
target_include_directories(initial_pose_publisher PRIVATE include)
ament_target_dependencies(initial_pose_publisher
  geometry_msgs
  lifecycle_msgs
  rclcpp
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY behavior_trees launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
