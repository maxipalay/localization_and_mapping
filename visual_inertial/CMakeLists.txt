cmake_minimum_required(VERSION 3.16)
project(visual_inertial LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visual_inertial_frontend REQUIRED)  # uses the installed export
find_package(visual_inertial_common REQUIRED)
find_package(visual_inertial_optimization REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(eigen3_cmake_module REQUIRED)

add_executable(tracking_node src/tracking_node.cpp)
add_executable(optimization_node src/optimization_node.cpp)
add_executable(path_viz_node src/path_viz_node.cpp)
add_executable(tracks_viz_node src/tracks_viz_node.cpp)

# Pull include dirs/defs from deps (including your lib)
ament_target_dependencies(tracking_node
  rclcpp sensor_msgs cv_bridge image_transport OpenCV visual_inertial_frontend visual_inertial_common tf2_ros)

ament_target_dependencies(tracks_viz_node
  rclcpp sensor_msgs cv_bridge image_transport OpenCV)

ament_target_dependencies(optimization_node
  rclcpp sensor_msgs visual_inertial_common visual_inertial_optimization tf2_ros)

ament_target_dependencies(path_viz_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

target_link_libraries(tracking_node
  visual_inertial_frontend::visual_inertial_frontend
  visual_inertial_common::visual_inertial_common
)

target_link_libraries(optimization_node
  visual_inertial_optimization::visual_inertial_optimization
  visual_inertial_common::visual_inertial_common
  TBB::tbb
)

install(TARGETS tracking_node optimization_node path_viz_node tracks_viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  "msg/ImuBias.msg"
  "msg/Tracks.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(tracking_node "${cpp_typesupport_target}")
target_link_libraries(tracks_viz_node "${cpp_typesupport_target}")
target_link_libraries(optimization_node "${cpp_typesupport_target}")

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
