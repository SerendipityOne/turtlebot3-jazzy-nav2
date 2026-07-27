################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${GZ_SIM_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${gz_sim_vendor_INCLUDE_DIRS}
  ${GZ_SIM_INCLUDE_DIRS}
)

set(EXEC_NAME "turtlebot3_drive")

add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
target_link_libraries(${EXEC_NAME}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
)

add_library(dataset_utils src/dataset_utils.cpp)
target_include_directories(dataset_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(gso_dataset_generator src/gso_dataset_generator.cpp)
target_link_libraries(gso_dataset_generator
  dataset_utils
  ${OpenCV_LIBRARIES}
  gz-msgs::gz-msgs
  gz-transport::gz-transport)
ament_target_dependencies(gso_dataset_generator
  cv_bridge message_filters rclcpp sensor_msgs)

add_executable(house_perception_acceptance src/house_perception_acceptance.cpp)
target_link_libraries(house_perception_acceptance
  dataset_utils
  ${OpenCV_LIBRARIES}
  gz-msgs::gz-msgs
  gz-transport::gz-transport)
ament_target_dependencies(house_perception_acceptance
  cv_bridge geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros vision_msgs)

add_executable(final_perception_acceptance src/final_perception_acceptance.cpp)
target_link_libraries(final_perception_acceptance
  dataset_utils
  ${OpenCV_LIBRARIES}
  gz-msgs::gz-msgs
  gz-transport::gz-transport)
ament_target_dependencies(final_perception_acceptance
  cv_bridge rclcpp sensor_msgs std_msgs vision_msgs)

add_executable(house_hard_negative_generator src/house_hard_negative_generator.cpp)
target_link_libraries(house_hard_negative_generator
  dataset_utils
  ${OpenCV_LIBRARIES}
  gz-msgs::gz-msgs
  gz-transport::gz-transport)
ament_target_dependencies(house_hard_negative_generator
  cv_bridge rclcpp sensor_msgs)

set(OBSTACLE_LIBS
  obstacles
  obstacle1
  obstacle2
)

foreach(lib ${OBSTACLE_LIBS})
  add_library(${lib} SHARED src/${lib}.cpp)
  target_link_libraries(${lib}
    PUBLIC
    gz-sim::gz-sim
    gz-math::gz-math
  )
endforeach()

################################################################################
# Install
################################################################################
install(TARGETS
  ${EXEC_NAME}
  dataset_utils
  gso_dataset_generator
  house_hard_negative_generator
  house_perception_acceptance
  final_perception_acceptance
  obstacles
  obstacle1
  obstacle2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dataset_utils test/test_dataset_utils.cpp)
  target_link_libraries(test_dataset_utils dataset_utils)
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(
  gazebo_ros_pkgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
)
ament_package()
