cmake_minimum_required(VERSION 3.16)
project(online_mapping_logger LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visual_inertial REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(online_mapping_logger_node
  src/online_mapping_logger_node.cpp
)

add_library(online_mapping_logger_core
  src/keyframe_assembler.cpp
  src/session_writer.cpp
)

target_include_directories(online_mapping_logger_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(online_mapping_logger_core
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
  visual_inertial
  tf2
  tf2_ros
)

target_compile_definitions(online_mapping_logger_core PUBLIC
  ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS=1
)
target_compile_definitions(online_mapping_logger_node PRIVATE
  ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS=1
)
ament_target_dependencies(online_mapping_logger_core
  apriltag_msgs
)
ament_target_dependencies(online_mapping_logger_node
  apriltag_msgs
)

target_include_directories(online_mapping_logger_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(online_mapping_logger_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
  visual_inertial
  tf2
  tf2_ros
)

target_link_libraries(online_mapping_logger_node
  online_mapping_logger_core
)

install(TARGETS online_mapping_logger_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS online_mapping_logger_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/inspect_logged_session_covariance.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
