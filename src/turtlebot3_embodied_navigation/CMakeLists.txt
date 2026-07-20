cmake_minimum_required(VERSION 3.8)
project(turtlebot3_embodied_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(embodied_core
  src/detection_history.cpp
  src/perception_utils.cpp)
target_include_directories(embodied_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(embodied_core PUBLIC ${OpenCV_LIBRARIES})

add_executable(object_detector src/object_detector_node.cpp)
target_link_libraries(object_detector embodied_core ${OpenCV_LIBRARIES})
ament_target_dependencies(object_detector
  cv_bridge message_filters rclcpp sensor_msgs vision_msgs)

install(TARGETS embodied_core object_detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection_history test/test_detection_history.cpp)
  target_link_libraries(test_detection_history embodied_core)
  ament_add_gtest(test_perception_utils test/test_perception_utils.cpp)
  target_link_libraries(test_perception_utils embodied_core ${OpenCV_LIBRARIES})
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
