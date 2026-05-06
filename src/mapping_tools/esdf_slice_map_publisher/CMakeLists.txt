cmake_minimum_required(VERSION 3.16)
project(esdf_slice_map_publisher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(esdf_slice_map_publisher_node
  src/esdf_slice_map_publisher_node.cpp
)

ament_target_dependencies(esdf_slice_map_publisher_node
  nav_msgs
  rclcpp
  sensor_msgs
)

target_link_libraries(esdf_slice_map_publisher_node
  ${OpenCV_LIBS}
  yaml-cpp
)

install(
  TARGETS esdf_slice_map_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
