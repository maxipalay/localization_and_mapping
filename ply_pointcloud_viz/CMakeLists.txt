cmake_minimum_required(VERSION 3.16)
project(ply_pointcloud_viz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ply_pointcloud_viz_node
  src/ply_pointcloud_viz_node.cpp
)

ament_target_dependencies(ply_pointcloud_viz_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS ply_pointcloud_viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
