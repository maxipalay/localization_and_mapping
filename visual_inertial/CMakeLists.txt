cmake_minimum_required(VERSION 3.16)
project(visual_inertial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visual_inertial_lib REQUIRED)  # uses the installed export
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slam_msgs REQUIRED)

add_executable(tracking_node src/tracking_node.cpp)

# Pull include dirs/defs from deps (including your lib)
ament_target_dependencies(tracking_node
  rclcpp sensor_msgs cv_bridge image_transport OpenCV visual_inertial_lib tf2_ros)

target_link_libraries(tracking_node
  visual_inertial_lib::visual_inertial_lib
)

install(TARGETS tracking_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
