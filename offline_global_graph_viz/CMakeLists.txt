cmake_minimum_required(VERSION 3.16)
project(offline_global_graph_viz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(offline_global_graph_viz_node
  src/visualization_node.cpp
)

target_include_directories(offline_global_graph_viz_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(offline_global_graph_viz_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(offline_global_graph_viz_node
  yaml-cpp
)

install(
  TARGETS offline_global_graph_viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
